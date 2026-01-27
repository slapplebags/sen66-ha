/*
  ESP32 + (SEN66 OR BMV080 OR PMS-UART OR IPS7100-UART) + MQTT + Home Assistant Discovery
  with WiFiManager captive-portal config

  Portal lets you enter Wi-Fi + MQTT + HA prefix + friendly name + sensor type (sen66|bmv080|pms|ips7100).

  FIX (Jan 2026):
  - chipId() now uses FULL 48-bit MAC (12 hex chars) instead of low 24 bits.
    This prevents two boards from colliding in Home Assistant discovery unique_id/topics.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// Increase MQTT packet size for HA discovery payloads
#define MQTT_MAX_PACKET_SIZE 1024

#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiManager.h>

// SEN66
#include <SensirionI2cSen66.h>

// BMV080
#include "SparkFun_BMV080_Arduino_Library.h"

// ---------------- Settings & constants ----------------
#ifndef NO_ERROR
#define NO_ERROR 0
#endif

#define CONFIG_TRIGGER_PIN 0

// Publish interval (MQTT + HA entities update cadence)
const uint32_t READ_INTERVAL_MS = 10000;

// BMV080 wants frequent servicing; this keeps it happy even when publishing slower
const uint32_t BMV_SERVICE_MS = 50;

// BMV080 default I2C addr on SparkFun breakout
#define BMV080_ADDR 0x57

// PMS-style UART sensor (Plantower-compatible frames 0x42 0x4D 0x00 0x1C ...)
static constexpr uint32_t PMS_BAUD = 9600;

// IPS/OPS-7100 UART CSV output
static constexpr uint32_t IPS_BAUD = 115200;

// Your working pins (KEEP AS-IS):
static constexpr int PMS_TX_PIN = D3; // XIAO TX -> sensor RX
static constexpr int PMS_RX_PIN = D4; // XIAO RX <- sensor TX

// ---------------- Globals ----------------
WiFiClient espClient;
PubSubClient mqtt(espClient);
Preferences prefs;

// SEN66 instance
SensirionI2cSen66 sen66;

// BMV080 instance
SparkFunBMV080 bmv080;

// UART instance (shared by PMS or IPS7100 depending on sensor_type)
HardwareSerial pmsSerial(1);

static char errorMessage[64];
static int16_t error;

// Runtime IDs/topics
String device_id;
String mqtt_client_id;
String base_topic;
String avail_topic;

// ---------------- Config ----------------
enum SensorType : uint8_t {
  SENSOR_SEN66   = 0,
  SENSOR_BMV080  = 1,
  SENSOR_PMS     = 2,
  SENSOR_IPS7100 = 3
};

struct AppConfig {
  char mqtt_host[64]      = "10.10.10.10";
  char mqtt_port[8]       = "1883";
  char mqtt_user[64]      = "user";
  char mqtt_pass[64]      = "password";
  char ha_prefix[32]      = "homeassistant";
  char friendly_name[32]  = "name";
  char sensor_type[16]    = "model"; // "sen66" or "bmv080" or "pms" or "ips7100"
} cfg;

SensorType activeSensor = SENSOR_SEN66;

bool shouldSaveConfig = false;
bool discovery_published = false;

unsigned long last_publish_ms = 0;
unsigned long last_bmv_service_ms = 0;

// BMV080 cached values (updated frequently)
float bmv_pm1 = NAN, bmv_pm25 = NAN, bmv_pm10 = NAN;
bool  bmv_obstructed = false;

// PMS cached values
uint16_t pms_pm1 = 0, pms_pm25 = 0, pms_pm10 = 0;
bool pms_has_reading = false;

// IPS7100 cached values (float ug/m3)
float ips_pm1 = NAN, ips_pm25 = NAN, ips_pm10 = NAN;
bool  ips_has_reading = false;
static String ips_line_buf;

// ---------------- Forward declarations ----------------
void loadConfig();
void saveConfig();
void ensureMqtt();

void publishDiscovery();
void publishAvailability(const char* state);
void publishOne(const String& suffix, const String& value);

void publishConfigSensor(const String& object_id, const String& name, const String& unit,
                         const String& device_class, const String& state_class, const String& icon,
                         const String& state_topic_suffix);

void publishConfigBinarySensor(const String& object_id, const String& name,
                               const String& device_class, const String& icon,
                               const String& state_topic_suffix,
                               const String& payload_on, const String& payload_off);

void publishSen66State(float pm1, float pm25, float pm4, float pm10,
                       float rh, float tempC, float vocIndex, float noxIndex, uint16_t co2);

void publishBmv080State(float pm1, float pm25, float pm10, bool obstructed);
void publishPmsState(uint16_t pm1, uint16_t pm25, uint16_t pm10);
void publishIps7100State(float pm1, float pm25, float pm10);

String chipId();
void safeTopicify(String &s);

bool initSen66();
bool initBmv080();
bool initPms();
bool initIps7100();

// PMS frame reader
bool pmsReadAtm(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10);

// IPS7100 line reader/parser
bool ipsReadCsvLine(String &outLine, uint32_t timeout_ms);
bool ipsParsePmFromLine(const String& line, float &pm1, float &pm25, float &pm10);

// ---------------- WiFiManager save callback ----------------
void saveConfigCallback() { shouldSaveConfig = true; }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(CONFIG_TRIGGER_PIN, INPUT_PULLUP);

  Wire.begin();

  // Load stored config first (so portal defaults are correct)
  loadConfig();

  // Decide active sensor from cfg
  String st0 = String(cfg.sensor_type);
  st0.trim(); st0.toLowerCase();
  if (st0 == "bmv080")        activeSensor = SENSOR_BMV080;
  else if (st0 == "pms")      activeSensor = SENSOR_PMS;
  else if (st0 == "ips7100")  activeSensor = SENSOR_IPS7100;
  else                        activeSensor = SENSOR_SEN66;

  // Create a device_id that is stable and unique per board.
  device_id = chipId(); // (will be overridden in initSen66/initBmv080/initPms/initIps7100)

  // WiFiManager
  bool forcePortal = (digitalRead(CONFIG_TRIGGER_PIN) == LOW);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setDebugOutput(true);
  wm.setSaveConfigCallback(saveConfigCallback);

  String apName = "AIR-" + chipId();

  // Custom params for MQTT & HA & Sensor Type
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT Host", cfg.mqtt_host, sizeof(cfg.mqtt_host));
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT Port", cfg.mqtt_port, sizeof(cfg.mqtt_port));
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT User", cfg.mqtt_user, sizeof(cfg.mqtt_user));
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT Password", cfg.mqtt_pass, sizeof(cfg.mqtt_pass));
  WiFiManagerParameter p_ha_prefix("ha_prefix", "HA Discovery Prefix", cfg.ha_prefix, sizeof(cfg.ha_prefix));
  WiFiManagerParameter p_friendly("friendly_name", "Device Friendly Name", cfg.friendly_name, sizeof(cfg.friendly_name));
  WiFiManagerParameter p_sensor_type("sensor_type", "Sensor Type (sen66|bmv080|pms|ips7100)", cfg.sensor_type, sizeof(cfg.sensor_type));

  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_ha_prefix);
  wm.addParameter(&p_friendly);
  wm.addParameter(&p_sensor_type);

  bool wifi_ok = false;
  if (forcePortal) {
    Serial.println("Config trigger active — starting captive portal...");
    wifi_ok = wm.startConfigPortal(apName.c_str());
  } else {
    Serial.println("Attempting Wi-Fi autoconnect...");
    wifi_ok = wm.autoConnect(apName.c_str());
  }

  if (!wifi_ok) {
    Serial.println("Wi-Fi not configured/connected. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  // Copy values back from portal
  strncpy(cfg.mqtt_host, p_mqtt_host.getValue(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, p_mqtt_port.getValue(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, p_mqtt_user.getValue(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, p_mqtt_pass.getValue(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, p_ha_prefix.getValue(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, p_friendly.getValue(), sizeof(cfg.friendly_name));
  strncpy(cfg.sensor_type, p_sensor_type.getValue(), sizeof(cfg.sensor_type));

  // Normalize sensor type
  String st = String(cfg.sensor_type);
  st.trim();
  st.toLowerCase();
  if (st != "bmv080" && st != "pms" && st != "ips7100") st = "sen66";
  strncpy(cfg.sensor_type, st.c_str(), sizeof(cfg.sensor_type));

  if (strlen(cfg.ha_prefix) == 0) {
    strncpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  }

  if (shouldSaveConfig) {
    saveConfig();
    Serial.println("Config saved.");
  }

  // Determine active sensor now that config is final
  String stf = String(cfg.sensor_type);
  stf.toLowerCase();
  activeSensor = (stf == "bmv080")  ? SENSOR_BMV080 :
                 (stf == "pms")     ? SENSOR_PMS :
                 (stf == "ips7100") ? SENSOR_IPS7100 :
                                      SENSOR_SEN66;

  // Init sensor and compute device_id (SEN66 prefers serial-based id)
  bool sensor_ok = false;
  if (activeSensor == SENSOR_SEN66) {
    sensor_ok = initSen66();
  } else if (activeSensor == SENSOR_BMV080) {
    sensor_ok = initBmv080();
  } else if (activeSensor == SENSOR_PMS) {
    sensor_ok = initPms();
  } else {
    sensor_ok = initIps7100();
  }

  if (!sensor_ok) {
    Serial.println("Sensor init failed. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  // Make sure device_id is safe for topics
  safeTopicify(device_id);

  mqtt_client_id = "esp32_" + device_id;

  // Topics depend on sensor type
  String prefix =
    (activeSensor == SENSOR_SEN66)   ? "sen66/"   :
    (activeSensor == SENSOR_BMV080)  ? "bmv080/"  :
    (activeSensor == SENSOR_PMS)     ? "pms/"     :
                                       "ips7100/";
  base_topic  = prefix + device_id;
  avail_topic = base_topic + "/status";

  Serial.println("Base topic: " + base_topic);
  Serial.println("Availability topic: " + avail_topic);

  // MQTT setup
  mqtt.setServer(cfg.mqtt_host, atoi(cfg.mqtt_port));
  mqtt.setBufferSize(1024);

  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ---------------- Loop ----------------
void loop() {
  ensureMqtt();

  // Keep BMV080 serviced frequently even if we publish slowly
  if (activeSensor == SENSOR_BMV080) {
    unsigned long now = millis();
    if (now - last_bmv_service_ms >= BMV_SERVICE_MS) {
      last_bmv_service_ms = now;

      if (bmv080.readSensor()) {
        bmv_pm10 = bmv080.PM10();
        bmv_pm25 = bmv080.PM25();
        bmv_pm1  = bmv080.PM1();
        bmv_obstructed = bmv080.isObstructed();
      }
    }
  }

  // For IPS7100, keep consuming UART so buffer doesn't overrun.
  // We parse the latest valid line we see between publishes.
  if (activeSensor == SENSOR_IPS7100) {
    String l;
    // Non-blocking-ish: grab any complete lines available
    while (ipsReadCsvLine(l, 0)) {
      float pm1, pm25, pm10;
      if (ipsParsePmFromLine(l, pm1, pm25, pm10)) {
        ips_pm1 = pm1;
        ips_pm25 = pm25;
        ips_pm10 = pm10;
        ips_has_reading = true;
      }
    }
  }

  // Publish cadence
  unsigned long now = millis();
  if (now - last_publish_ms >= READ_INTERVAL_MS) {
    last_publish_ms = now;

    if (activeSensor == SENSOR_SEN66) {
      float pm1=0, pm25=0, pm4=0, pm10=0;
      float rh=0, tempC=0, voc=0, nox=0;
      uint16_t co2=0;

      error = sen66.readMeasuredValues(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
      if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.printf("SEN66 readMeasuredValues() error: %s\n", errorMessage);
      } else {
        Serial.printf("SEN66 PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f RH=%.1f T=%.2f VOC=%.1f NOx=%.1f CO2=%u\n",
                      pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
        publishSen66State(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
      }
    } else if (activeSensor == SENSOR_BMV080) {
      if (!isnan(bmv_pm1) && !isnan(bmv_pm25) && !isnan(bmv_pm10)) {
        Serial.printf("BMV080 PM1=%.2f PM2.5=%.2f PM10=%.2f Obstructed=%s\n",
                      bmv_pm1, bmv_pm25, bmv_pm10, bmv_obstructed ? "true" : "false");
        publishBmv080State(bmv_pm1, bmv_pm25, bmv_pm10, bmv_obstructed);
      } else {
        Serial.println("BMV080: no readings yet (still warming/servicing).");
      }
    } else if (activeSensor == SENSOR_PMS) {
      uint16_t pm1=0, pm25=0, pm10=0;
      if (pmsReadAtm(pm1, pm25, pm10)) {
        pms_pm1 = pm1; pms_pm25 = pm25; pms_pm10 = pm10;
        pms_has_reading = true;
        Serial.printf("PMS (ATM) PM1=%u PM2.5=%u PM10=%u\n", pm1, pm25, pm10);
        publishPmsState(pm1, pm25, pm10);
      } else {
        Serial.println("PMS: no valid frame yet (check wiring/baud).");
      }
    } else {
      if (ips_has_reading && !isnan(ips_pm1) && !isnan(ips_pm25) && !isnan(ips_pm10)) {
        Serial.printf("IPS7100 PM1.0=%.3f PM2.5=%.3f PM10=%.3f\n", ips_pm1, ips_pm25, ips_pm10);
        publishIps7100State(ips_pm1, ips_pm25, ips_pm10);
      } else {
        Serial.println("IPS7100: no parsed readings yet (waiting on CSV line).");
      }
    }
  }

  mqtt.loop();
}

// --------------- Sensor init ---------------
bool initSen66() {
  sen66.begin(Wire, SEN66_I2C_ADDR_6B);

  error = sen66.deviceReset();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("SEN66 deviceReset() error: %s\n", errorMessage);
  }
  delay(1200);

  // Prefer SEN66 serial number for device_id
  int8_t serialNumber[32] = {0};
  error = sen66.getSerialNumber(serialNumber, 32);
  if (error == NO_ERROR) {
    device_id = "sen66_" + String((const char*)serialNumber);
  } else {
    device_id = "sen66_" + chipId();
  }

  error = sen66.startContinuousMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("SEN66 startContinuousMeasurement() error: %s\n", errorMessage);
    return false;
  }

  return true;
}

bool initBmv080() {
  if (!bmv080.begin(BMV080_ADDR, Wire)) {
    Serial.println("BMV080 not detected at 0x57. Check wiring/jumper.");
    return false;
  }
  Serial.println("BMV080 found!");

  bmv080.init();

  if (!bmv080.setMode(SF_BMV080_MODE_CONTINUOUS)) {
    Serial.println("BMV080 setMode(CONTINUOUS) failed.");
    return false;
  }

  device_id = "bmv080_" + chipId();
  return true;
}

bool initPms() {
  // Start UART and give sensor time to boot/stream
  pmsSerial.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(1200);

  device_id = "pms_" + chipId();
  return true;
}

bool initIps7100() {
  // Start UART at 115200 and allow sensor to start streaming CSV
  pmsSerial.begin(IPS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  pmsSerial.setTimeout(50);
  ips_line_buf.reserve(256);

  delay(500);

  device_id = "ips7100_" + chipId();
  return true;
}

// --------------- PMS frame reader (ATM only) ---------------
static inline uint16_t u16be(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static bool pmsReadExact(uint8_t* out, size_t n, uint32_t timeout_ms) {
  uint32_t start = millis();
  size_t got = 0;
  while (got < n && (millis() - start) < timeout_ms) {
    while (pmsSerial.available() && got < n) {
      out[got++] = (uint8_t)pmsSerial.read();
    }
    delay(1);
  }
  return got == n;
}

bool pmsReadAtm(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10) {
  static constexpr size_t FRAME_LEN = 32;
  uint8_t f[FRAME_LEN];

  while (pmsSerial.available()) {
    int b = pmsSerial.read();
    if (b < 0) break;
    if ((uint8_t)b != 0x42) continue;

    uint32_t t0 = millis();
    while (!pmsSerial.available() && (millis() - t0) < 50) delay(1);
    if (!pmsSerial.available()) return false;

    uint8_t b2 = (uint8_t)pmsSerial.read();
    if (b2 != 0x4D) continue;

    f[0] = 0x42;
    f[1] = 0x4D;

    if (!pmsReadExact(f + 2, FRAME_LEN - 2, 200)) return false;

    // length must be 0x001C
    if (u16be(f + 2) != 0x001C) continue;

    // checksum: sum bytes 0..29 equals uint16 at 30..31
    uint32_t sum = 0;
    for (int i = 0; i < 30; i++) sum += f[i];
    if ((uint16_t)sum != u16be(f + 30)) continue;

    // ATM values at offsets 10/12/14
    pm1  = u16be(f + 10);
    pm25 = u16be(f + 12);
    pm10 = u16be(f + 14);
    return true;
  }

  return false;
}

// --------------- IPS7100 CSV reader/parser ---------------
bool ipsReadCsvLine(String &outLine, uint32_t timeout_ms) {
  uint32_t start = millis();

  while (true) {
    while (pmsSerial.available()) {
      char c = (char)pmsSerial.read();
      if (c == '\n') {
        ips_line_buf.trim();     // removes trailing \r and spaces
        outLine = ips_line_buf;
        ips_line_buf = "";
        return outLine.length() > 0;
      } else if (c != '\r') {
        if (ips_line_buf.length() < 600) ips_line_buf += c;
        else ips_line_buf = ""; // safety reset
      }
    }

    if (timeout_ms == 0) return false;
    if ((millis() - start) >= timeout_ms) return false;
    delay(1);
  }
}

static bool ipsFindCsvValue(const String& line, const char* key, float &out) {
  String k = String(key) + ",";
  int idx = line.indexOf(k);
  if (idx < 0) return false;

  int start = idx + k.length();
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();

  String num = line.substring(start, end);
  num.trim();
  if (!num.length()) return false;

  out = num.toFloat();
  return true;
}

bool ipsParsePmFromLine(const String& line, float &pm1, float &pm25, float &pm10) {
  bool ok1  = ipsFindCsvValue(line, "PM1.0", pm1);
  bool ok25 = ipsFindCsvValue(line, "PM2.5", pm25);
  bool ok10 = ipsFindCsvValue(line, "PM10",  pm10);
  return ok1 && ok25 && ok10;
}

// --------------- Config (NVS) ---------------
void loadConfig() {
  prefs.begin("aircfg", true);
  String host = prefs.getString("mqtt_host", cfg.mqtt_host);
  String port = prefs.getString("mqtt_port", cfg.mqtt_port);
  String user = prefs.getString("mqtt_user", cfg.mqtt_user);
  String pass = prefs.getString("mqtt_pass", cfg.mqtt_pass);
  String pref = prefs.getString("ha_prefix", cfg.ha_prefix);
  String name = prefs.getString("friendly", cfg.friendly_name);
  String styp = prefs.getString("sensor_type", cfg.sensor_type);
  prefs.end();

  strncpy(cfg.mqtt_host, host.c_str(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, port.c_str(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, user.c_str(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, pass.c_str(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, pref.c_str(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, name.c_str(), sizeof(cfg.friendly_name));
  strncpy(cfg.sensor_type, styp.c_str(), sizeof(cfg.sensor_type));

  if (strlen(cfg.ha_prefix) == 0) strncpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  if (strlen(cfg.sensor_type) == 0) strncpy(cfg.sensor_type, "sen66", sizeof(cfg.sensor_type));

  Serial.printf("Loaded config: MQTT %s:%s, HA prefix='%s', name='%s', sensor_type='%s'\n",
                cfg.mqtt_host, cfg.mqtt_port, cfg.ha_prefix, cfg.friendly_name, cfg.sensor_type);
}

void saveConfig() {
  prefs.begin("aircfg", false);
  prefs.putString("mqtt_host", cfg.mqtt_host);
  prefs.putString("mqtt_port", cfg.mqtt_port);
  prefs.putString("mqtt_user", cfg.mqtt_user);
  prefs.putString("mqtt_pass", cfg.mqtt_pass);
  prefs.putString("ha_prefix", cfg.ha_prefix);
  prefs.putString("friendly",  cfg.friendly_name);
  prefs.putString("sensor_type", cfg.sensor_type);
  prefs.end();
}

// --------------- MQTT helpers ---------------
void ensureMqtt() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.printf("MQTT connecting to %s:%s ...\n", cfg.mqtt_host, cfg.mqtt_port);

    bool ok = false;
    if (strlen(cfg.mqtt_user)) {
      ok = mqtt.connect(
        mqtt_client_id.c_str(),
        cfg.mqtt_user, cfg.mqtt_pass,
        avail_topic.c_str(), 1, true, "offline"
      );
    } else {
      ok = mqtt.connect(
        mqtt_client_id.c_str(),
        avail_topic.c_str(), 1, true, "offline"
      );
    }

    if (!ok) {
      Serial.printf("MQTT connect failed (state %d). Retrying...\n", mqtt.state());
      delay(2000);
      continue;
    }

    Serial.println("MQTT connected.");
    publishAvailability("online");
    publishDiscovery();
  }
}

void publishAvailability(const char* state) {
  mqtt.publish(avail_topic.c_str(), state, true);
}

void publishOne(const String& suffix, const String& value) {
  String topic = base_topic + "/" + suffix;
  mqtt.publish(topic.c_str(), value.c_str(), false);
}

// --------------- Home Assistant Discovery ---------------
void publishDiscovery() {
  if (discovery_published) {
    Serial.println("Discovery already published, skipping.");
    return;
  }

  const char* sensorName =
    (activeSensor == SENSOR_SEN66)   ? "sen66" :
    (activeSensor == SENSOR_BMV080)  ? "bmv080" :
    (activeSensor == SENSOR_PMS)     ? "pms" :
                                       "ips7100";

  Serial.printf("Publishing HA discovery prefix='%s' device_id='%s' sensor='%s'\n",
                cfg.ha_prefix, device_id.c_str(), sensorName);

  if (activeSensor == SENSOR_SEN66) {
    publishConfigSensor("pm1",        "PM1.0",       "µg/m³", "pm1",            "measurement", "mdi:blur",            "pm1");
    publishConfigSensor("pm25",       "PM2.5",       "µg/m³", "pm25",           "measurement", "",                    "pm25");
    publishConfigSensor("pm4",        "PM4.0",       "µg/m³", "",               "measurement", "mdi:blur",            "pm4");
    publishConfigSensor("pm10",       "PM10",        "µg/m³", "pm10",           "measurement", "",                    "pm10");
    publishConfigSensor("humidity",   "Humidity",    "%",     "humidity",       "measurement", "",                    "humidity");
    publishConfigSensor("temperature","Temperature", "°C",    "temperature",    "measurement", "",                    "temperature");
    publishConfigSensor("voc_index",  "VOC Index",   "",      "",               "measurement", "mdi:chemical-weapon", "voc_index");
    publishConfigSensor("nox_index",  "NOx Index",   "",      "",               "measurement", "mdi:chemical-weapon", "nox_index");
    publishConfigSensor("co2eq",      "CO2 (eq)",    "ppm",   "carbon_dioxide", "measurement", "",                    "co2eq");
  } else if (activeSensor == SENSOR_BMV080) {
    publishConfigSensor("pm1",   "PM1.0",  "µg/m³", "pm1",  "measurement", "mdi:blur", "pm1");
    publishConfigSensor("pm25",  "PM2.5",  "µg/m³", "pm25", "measurement", "",         "pm25");
    publishConfigSensor("pm10",  "PM10",   "µg/m³", "pm10", "measurement", "",         "pm10");

    publishConfigBinarySensor("obstructed", "Optics Obstructed",
                              "problem", "mdi:alert-circle",
                              "obstructed",
                              "ON", "OFF");
  } else if (activeSensor == SENSOR_PMS) {
    publishConfigSensor("pm1",   "PM1.0", "µg/m³", "pm1",  "measurement", "mdi:blur", "pm1");
    publishConfigSensor("pm25",  "PM2.5", "µg/m³", "pm25", "measurement", "",         "pm25");
    publishConfigSensor("pm10",  "PM10",  "µg/m³", "pm10", "measurement", "",         "pm10");
  } else {
    // IPS7100: CSV float values
    publishConfigSensor("pm1",   "PM1.0", "µg/m³", "pm1",  "measurement", "mdi:blur", "pm1");
    publishConfigSensor("pm25",  "PM2.5", "µg/m³", "pm25", "measurement", "",         "pm25");
    publishConfigSensor("pm10",  "PM10",  "µg/m³", "pm10", "measurement", "",         "pm10");
  }

  discovery_published = true;
  Serial.println("Discovery publish complete.");
}

void publishConfigSensor(
  const String& object_id,
  const String& name,
  const String& unit,
  const String& device_class,
  const String& state_class,
  const String& icon,
  const String& state_topic_suffix
) {
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

  payload += "\"device\":{";
  payload +=   "\"identifiers\":[\"" + device_id + "\"],";

  if (activeSensor == SENSOR_SEN66) {
    payload += "\"manufacturer\":\"Sensirion\",\"model\":\"SEN66\",";
  } else if (activeSensor == SENSOR_BMV080) {
    payload += "\"manufacturer\":\"Bosch\",\"model\":\"BMV080\",";
  } else if (activeSensor == SENSOR_IPS7100) {
    payload += "\"manufacturer\":\"Piera Systems\",\"model\":\"IPS-7100\",";
  } else {
    payload += "\"manufacturer\":\"PMS-compatible\",\"model\":\"PMS UART\",";
  }

  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("HA sensor config publish '%s': %s (len=%d)\n",
                object_id.c_str(), ok ? "OK" : "FAILED", payload.length());
}

void publishConfigBinarySensor(
  const String& object_id,
  const String& name,
  const String& device_class,
  const String& icon,
  const String& state_topic_suffix,
  const String& payload_on,
  const String& payload_off
) {
  String topic = String(cfg.ha_prefix) + "/binary_sensor/" + device_id + "/" + object_id + "/config";

  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + device_id + "_" + object_id + "\",";
  payload += "\"state_topic\":\"" + base_topic + "/" + state_topic_suffix + "\",";
  payload += "\"availability_topic\":\"" + avail_topic + "\",";
  payload += "\"payload_on\":\"" + payload_on + "\",";
  payload += "\"payload_off\":\"" + payload_off + "\",";

  if (device_class.length()) payload += "\"device_class\":\"" + device_class + "\",";
  if (icon.length())         payload += "\"icon\":\"" + icon + "\",";

  payload += "\"device\":{";
  payload +=   "\"identifiers\":[\"" + device_id + "\"],";
  payload +=   "\"manufacturer\":\"Bosch\",";
  payload +=   "\"model\":\"BMV080\",";
  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("HA binary_sensor config publish '%s': %s (len=%d)\n",
                object_id.c_str(), ok ? "OK" : "FAILED", payload.length());
}

// --------------- State publishers ---------------
void publishSen66State(float pm1, float pm25, float pm4, float pm10,
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

void publishBmv080State(float pm1, float pm25, float pm10, bool obstructed) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  publishOne("pm1",  String(pm1, 2));
  publishOne("pm25", String(pm25, 2));
  publishOne("pm10", String(pm10, 2));
  publishOne("obstructed", obstructed ? "ON" : "OFF");
}

void publishPmsState(uint16_t pm1, uint16_t pm25, uint16_t pm10) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  publishOne("pm1",  String(pm1));
  publishOne("pm25", String(pm25));
  publishOne("pm10", String(pm10));
}

void publishIps7100State(float pm1, float pm25, float pm10) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  // Publish with 3 decimals (sensor is float)
  publishOne("pm1",  String(pm1, 3));
  publishOne("pm25", String(pm25, 3));
  publishOne("pm10", String(pm10, 3));
}

// --------------- Utilities ---------------
String chipId() {
  // FULL 48-bit MAC -> 12 hex chars. Prevents collisions between boards.
  uint64_t mac = ESP.getEfuseMac(); // MAC is in low 48 bits
  char buf[13]; // 12 + NUL
  snprintf(buf, sizeof(buf), "%012llX",
           (unsigned long long)(mac & 0xFFFFFFFFFFFFULL));
  return String(buf);
}

void safeTopicify(String &s) {
  s.replace(" ", "_");
  s.replace("/", "_");
  s.replace("\\", "_");
  s.replace("+", "_");
  s.replace("#", "_");
}
