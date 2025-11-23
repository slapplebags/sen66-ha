/*
  ESP32 + SEN66 + MQTT + Home Assistant, with WiFiManager captive-portal config

  On first boot (or if BOOT/GPIO0 held low at power-on), device starts an AP:
    SSID: SEN66-<chipid>   (no password by default)
  Portal lets you enter Wi-Fi + MQTT settings. They’re saved in NVS and reused.

  Requires:
    - WiFiManager by tzapu
    - PubSubClient by Nick O'Leary
    - SensirionI2cSen66 by Sensirion
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// Increase MQTT packet size for HA discovery payloads
#define MQTT_MAX_PACKET_SIZE 768

#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <SensirionI2cSen66.h>

// ---------------- Settings & constants ----------------
#ifndef NO_ERROR
#define NO_ERROR 0
#endif

// Hold this pin LOW at boot to force config portal (use BOOT button on most ESP32 dev boards)
#define CONFIG_TRIGGER_PIN 0

// Sensor & publish intervals
const uint32_t READ_INTERVAL_MS = 10000;

// ---------------- Globals ----------------
SensirionI2cSen66 sensor;
WiFiClient espClient;
PubSubClient mqtt(espClient);
Preferences prefs;

static char errorMessage[64];
static int16_t error;

String device_id;
String mqtt_client_id;
String base_topic;
String avail_topic;

// Runtime settings (loaded/saved via Preferences)
struct AppConfig {
  char mqtt_host[64]      = "10.10.10.10";
  char mqtt_port[8]       = "1883";
  char mqtt_user[64]      = "user";
  char mqtt_pass[64]      = "password";
  char ha_prefix[32]      = "homeassistant";
  char friendly_name[32]  = "SEN66 Air Sensor";
} cfg;

bool shouldSaveConfig = false;
bool discovery_published = false;
unsigned long last_read_ms = 0;

// ---------------- Forward declarations ----------------
void loadConfig();
void saveConfig();
void startConfigPortal();
void ensureMqtt();
void publishDiscovery();
void publishAvailability(const char* state);
void publishState(float pm1, float pm25, float pm4, float pm10,
                  float rh, float tempC, float vocIndex, float noxIndex, uint16_t co2);
void publishOne(const String& suffix, const String& value);
void publishConfigSensor(const String& object_id, const String& name, const String& unit,
                         const String& device_class, const String& state_class, const String& icon,
                         const String& state_topic_suffix, const String& value_template);
String chipId();
void safeTopicify(String &s);

// ---------------- WiFiManager save callback ----------------
void saveConfigCallback() { shouldSaveConfig = true; }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(CONFIG_TRIGGER_PIN, INPUT_PULLUP);

  // Sensor init
  Wire.begin();
  sensor.begin(Wire, SEN66_I2C_ADDR_6B);

  error = sensor.deviceReset();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("deviceReset() error: %s\n", errorMessage);
  }
  delay(1200);

  // Derive device_id from SEN66 serial if available, else CHIP ID
  int8_t serialNumber[32] = {0};
  error = sensor.getSerialNumber(serialNumber, 32);
  if (error == NO_ERROR) {
    device_id = "sen66_" + String((const char*)serialNumber);
  } else {
    device_id = "sen66_" + chipId();
  }
  safeTopicify(device_id);
  mqtt_client_id = "esp32_" + device_id;

  // Load stored MQTT config
  loadConfig();

  // If button held OR WiFi not configured, show captive portal
  bool forcePortal = (digitalRead(CONFIG_TRIGGER_PIN) == LOW);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setDebugOutput(true);
  wm.setSaveConfigCallback(saveConfigCallback);

  // Build AP name
  String apName = "SEN66-" + chipId();

  // Custom params for MQTT (persisted by us, not WiFiManager)
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT Host", cfg.mqtt_host, sizeof(cfg.mqtt_host));
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT Port", cfg.mqtt_port, sizeof(cfg.mqtt_port));
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT User", cfg.mqtt_user, sizeof(cfg.mqtt_user));
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT Password", cfg.mqtt_pass, sizeof(cfg.mqtt_pass));
  WiFiManagerParameter p_ha_prefix("ha_prefix", "HA Discovery Prefix", cfg.ha_prefix, sizeof(cfg.ha_prefix));
  WiFiManagerParameter p_friendly("friendly_name", "Device Friendly Name", cfg.friendly_name, sizeof(cfg.friendly_name));

  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_ha_prefix);
  wm.addParameter(&p_friendly);

  bool wifi_ok = false;

  if (forcePortal) {
    Serial.println("Config trigger active — starting captive portal...");
    wifi_ok = wm.startConfigPortal(apName.c_str());  // blocks until saved or timeout
  } else {
    // Try auto-connect; if it fails, start portal
    Serial.println("Attempting Wi-Fi autoconnect...");
    wifi_ok = wm.autoConnect(apName.c_str()); // starts portal if needed
  }

  if (!wifi_ok) {
    Serial.println("Wi-Fi not configured/connected. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  // Copy new values if changed via portal
  strncpy(cfg.mqtt_host, p_mqtt_host.getValue(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, p_mqtt_port.getValue(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, p_mqtt_user.getValue(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, p_mqtt_pass.getValue(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, p_ha_prefix.getValue(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, p_friendly.getValue(), sizeof(cfg.friendly_name));

  if (shouldSaveConfig) {
    saveConfig();
    Serial.println("Config saved.");
  }

  // Derive topics after we have device_id & (maybe) new settings
  base_topic  = "sen66/" + device_id;
  avail_topic = base_topic + "/status";

  Serial.println("Base topic: " + base_topic);
  Serial.println("Availability topic: " + avail_topic);

  // Start continuous measurement
  error = sensor.startContinuousMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("startContinuousMeasurement() error: %s\n", errorMessage);
  }

  // MQTT server
  mqtt.setServer(cfg.mqtt_host, atoi(cfg.mqtt_port));

  // Increase buffer size for large HA discovery payloads
  mqtt.setBufferSize(512);

  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ---------------- Loop ----------------
void loop() {
  ensureMqtt();

  unsigned long now = millis();
  if (now - last_read_ms >= READ_INTERVAL_MS) {
    last_read_ms = now;

    float pm1=0, pm25=0, pm4=0, pm10=0;
    float rh=0, tempC=0, voc=0, nox=0;
    uint16_t co2=0;

    error = sensor.readMeasuredValues(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
    if (error != NO_ERROR) {
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.printf("readMeasuredValues() error: %s\n", errorMessage);
    } else {
      Serial.printf("PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f RH=%.1f T=%.2f VOC=%.1f NOx=%.1f CO2=%u\n",
                    pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
      publishState(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
    }
  }

  mqtt.loop();
}

// --------------- Config (NVS) ---------------
void loadConfig() {
  prefs.begin("sen66", true);
  String host = prefs.getString("mqtt_host", cfg.mqtt_host);
  String port = prefs.getString("mqtt_port", cfg.mqtt_port);
  String user = prefs.getString("mqtt_user", cfg.mqtt_user);
  String pass = prefs.getString("mqtt_pass", cfg.mqtt_pass);
  String pref = prefs.getString("ha_prefix", cfg.ha_prefix);
  String name = prefs.getString("friendly", cfg.friendly_name);
  prefs.end();

  strncpy(cfg.mqtt_host, host.c_str(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, port.c_str(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, user.c_str(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, pass.c_str(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, pref.c_str(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, name.c_str(), sizeof(cfg.friendly_name));

  // Ensure we always have a valid discovery prefix
  if (strlen(cfg.ha_prefix) == 0) {
    strncpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  }

  Serial.printf("Loaded config: MQTT %s:%s, HA prefix: '%s', name: '%s'\n",
                cfg.mqtt_host, cfg.mqtt_port, cfg.ha_prefix, cfg.friendly_name);
}

void saveConfig() {
  prefs.begin("sen66", false);
  prefs.putString("mqtt_host", cfg.mqtt_host);
  prefs.putString("mqtt_port", cfg.mqtt_port);
  prefs.putString("mqtt_user", cfg.mqtt_user);
  prefs.putString("mqtt_pass", cfg.mqtt_pass);
  prefs.putString("ha_prefix", cfg.ha_prefix);
  prefs.putString("friendly",  cfg.friendly_name);
  prefs.end();
}

// --------------- WiFiManager (manual trigger) ---------------
void startConfigPortal() {
  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);
  String apName = "SEN66-" + chipId();
  wm.startConfigPortal(apName.c_str());
}

// --------------- MQTT helpers ---------------
void ensureMqtt() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.printf("MQTT connecting to %s:%s ...\n", cfg.mqtt_host, cfg.mqtt_port);

    if (strlen(cfg.mqtt_user)) {
      if (mqtt.connect(
            mqtt_client_id.c_str(),
            cfg.mqtt_user, cfg.mqtt_pass,
            avail_topic.c_str(), 1, true, "offline"
          )) {
        Serial.println("MQTT connected (auth).");
      } else {
        Serial.printf("MQTT connect failed (state %d). Retrying...\n", mqtt.state());
        delay(2000);
        continue;
      }
    } else {
      if (mqtt.connect(
            mqtt_client_id.c_str(),
            avail_topic.c_str(), 1, true, "offline"
          )) {
        Serial.println("MQTT connected.");
      } else {
        Serial.printf("MQTT connect failed (state %d). Retrying...\n", mqtt.state());
        delay(2000);
        continue;
      }
    }

    publishAvailability("online");
    publishDiscovery();
  }
}

void publishAvailability(const char* state) {
  mqtt.publish(avail_topic.c_str(), state, true);
}

void publishDiscovery() {
  if (discovery_published) {
    Serial.println("Discovery already published, skipping.");
    return;
  }

  Serial.printf("Publishing Home Assistant discovery with prefix '%s' and device_id '%s'\n",
                cfg.ha_prefix, device_id.c_str());

  publishConfigSensor("pm1",        "PM1.0",       "µg/m³", "pm1",           "measurement", "mdi:blur",            "pm1",        "");
  publishConfigSensor("pm25",       "PM2.5",       "µg/m³", "pm25",          "measurement", "",                    "pm25",       "");
  publishConfigSensor("pm4",        "PM4.0",       "µg/m³", "",              "measurement", "mdi:blur",            "pm4",        "");
  publishConfigSensor("pm10",       "PM10",        "µg/m³", "pm10",          "measurement", "",                    "pm10",       "");
  publishConfigSensor("humidity",   "Humidity",    "%",     "humidity",      "measurement", "",                    "humidity",   "");
  publishConfigSensor("temperature","Temperature", "°C",    "temperature",   "measurement", "",                    "temperature","");
  publishConfigSensor("voc_index",  "VOC Index",   "",      "",              "measurement", "mdi:chemical-weapon", "voc_index",  "");
  publishConfigSensor("nox_index",  "NOx Index",   "",      "",              "measurement", "mdi:chemical-weapon", "nox_index",  "");
  publishConfigSensor("co2eq",      "CO2 (eq)",    "ppm",   "carbon_dioxide","measurement", "",                    "co2eq",      "");

  discovery_published = true;
  Serial.println("Discovery publish complete.");
}

void publishOne(const String& suffix, const String& value) {
  String topic = base_topic + "/" + suffix;
  mqtt.publish(topic.c_str(), value.c_str(), false);
}

void publishState(float pm1, float pm25, float pm4, float pm10,
                  float rh, float tempC, float vocIndex, float noxIndex, uint16_t co2) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  publishOne("pm1",         String(pm1, 1));
  publishOne("pm25",        String(pm25, 1));
  publishOne("pm4",         String(pm4, 1));
  publishOne("pm10",        String(pm10, 1));
  publishOne("humidity",    String(rh, 1));
  publishOne("temperature", String(tempC, 2));
  publishOne("voc_index",   String(vocIndex, 1));
  publishOne("nox_index",   String(noxIndex, 1));
  publishOne("co2eq",       String(co2));
}

// --------------- Discovery config publisher ---------------
void publishConfigSensor(
  const String& object_id,
  const String& name,
  const String& unit,
  const String& device_class,
  const String& state_class,
  const String& icon,
  const String& state_topic_suffix,
  const String& value_template
) {
  // Topic: <prefix>/sensor/<device_id>/<object_id>/config
  String topic = String(cfg.ha_prefix) + "/sensor/" + device_id + "/" + object_id + "/config";

  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + device_id + "_" + object_id + "\",";
  payload += "\"state_topic\":\"" + base_topic + "/" + state_topic_suffix + "\",";
  payload += "\"availability_topic\":\"" + avail_topic + "\",";

  if (unit.length())         payload += "\"unit_of_measurement\":\"" + unit + "\",";
  if (device_class.length()) payload += "\"device_class\":\"" + device_class + "\",";
  if (state_class.length())  payload += "\"state_class\":\"" + state_class + "\",";
  if (icon.length())         payload += "\"icon\":\"" + icon + "\",";
  if (value_template.length()) payload += "\"value_template\":" + value_template + ",";

  payload += "\"device\":{";
  payload +=   "\"identifiers\":[\"" + device_id + "\"],";
  payload +=   "\"manufacturer\":\"Sensirion\",";
  payload +=   "\"model\":\"SEN66\",";
  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  Serial.println("Discovery topic: " + topic);
  Serial.println("Discovery payload: " + payload);
  Serial.printf("Payload length for '%s': %d bytes\n",
                object_id.c_str(), payload.length());

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("Discovery publish for '%s': %s\n",
                object_id.c_str(), ok ? "OK" : "FAILED");
}

// --------------- Utilities ---------------
String chipId() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t low = (uint32_t)(mac & 0xFFFFFF);
  char buf[9];
  snprintf(buf, sizeof(buf), "%06X", low);
  return String(buf);
}

void safeTopicify(String &s) {
  s.replace(" ", "_");
  s.replace("/", "_");
  s.replace("\\", "_");
  s.replace("+", "_");
  s.replace("#", "_");
}
