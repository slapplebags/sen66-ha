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
static constexpr uint32_t READ_INTERVAL_MS = 10000;

// BMV080 wants frequent servicing; this keeps it happy even when publishing slower
static constexpr uint32_t BMV_SERVICE_MS = 50;

// BMV080 default I2C addr on SparkFun breakout
static constexpr uint8_t BMV080_ADDR = 0x57;

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
  char sensor_type[16]    = "sen66"; // "sen66"|"bmv080"|"pms"|"ips7100"
} cfg;

static SensorType activeSensor = SENSOR_SEN66;

static bool shouldSaveConfig = false;
static bool discovery_published = false;

static unsigned long last_publish_ms = 0;
static unsigned long last_bmv_service_ms = 0;

// IPS7100 line buffer
static String ips_line_buf;

// ---------------- Unified measurement container ----------------
struct Measurements {
  // Always publish these (if available) as float µg/m³
  float pm1  = NAN;
  float pm25 = NAN;
  float pm10 = NAN;

  // SEN66-only extra
  float pm4  = NAN;

  // SEN66 env/index extras
  float rh      = NAN;
  float tempC   = NAN;
  float vocIdx  = NAN;
  float noxIdx  = NAN;
  uint16_t co2  = 0;
  bool has_env  = false;

  // BMV080-only
  bool obstructed = false;
  bool has_obstructed = false;

  bool has_pm() const {
    return !isnan(pm1) && !isnan(pm25) && !isnan(pm10);
  }
};

static Measurements meas;

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

String chipId();
void safeTopicify(String &s);
static void normalizeSensorTypeInCfg();
static SensorType sensorTypeFromCfg();
static void copyParam(char* dst, size_t dst_sz, const char* src);

// Sensor init
bool initSen66();
bool initBmv080();
bool initPms();
bool initIps7100();

// Sensor reads into `meas`
static bool readSen66(Measurements& m);
static void serviceBmv080(Measurements& m);   // frequent
static bool readBmv080Snapshot(Measurements& m); // publish-time snapshot
static bool readPms(Measurements& m);
static bool serviceIps7100(Measurements& m);  // consume UART continuously

// PMS frame reader
bool pmsReadAtm(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10);

// IPS7100 line reader/parser
bool ipsReadCsvLine(String &outLine, uint32_t timeout_ms);
bool ipsParsePmFromLine(const String& line, float &pm1, float &pm25, float &pm10);

// Publishing
static void publishPmTrio(const Measurements& m);
static void publishSen66Extras(const Measurements& m);
static void publishBmv080Extras(const Measurements& m);

// ---------------- WiFiManager save callback ----------------
void saveConfigCallback() { shouldSaveConfig = true; }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(CONFIG_TRIGGER_PIN, INPUT_PULLUP);
  Wire.begin();

  loadConfig();
  normalizeSensorTypeInCfg();
  activeSensor = sensorTypeFromCfg();

  // Default device_id: per-board; sensors may override (e.g. SEN66 serial)
  device_id = chipId();

  // WiFiManager
  const bool forcePortal = (digitalRead(CONFIG_TRIGGER_PIN) == LOW);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setDebugOutput(true);
  wm.setSaveConfigCallback(saveConfigCallback);

  String apName = "AIR-" + chipId();

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

  // Copy values back from portal (safe copies)
  copyParam(cfg.mqtt_host, sizeof(cfg.mqtt_host), p_mqtt_host.getValue());
  copyParam(cfg.mqtt_port, sizeof(cfg.mqtt_port), p_mqtt_port.getValue());
  copyParam(cfg.mqtt_user, sizeof(cfg.mqtt_user), p_mqtt_user.getValue());
  copyParam(cfg.mqtt_pass, sizeof(cfg.mqtt_pass), p_mqtt_pass.getValue());
  copyParam(cfg.ha_prefix, sizeof(cfg.ha_prefix), p_ha_prefix.getValue());
  copyParam(cfg.friendly_name, sizeof(cfg.friendly_name), p_friendly.getValue());
  copyParam(cfg.sensor_type, sizeof(cfg.sensor_type), p_sensor_type.getValue());

  normalizeSensorTypeInCfg();

  if (strlen(cfg.ha_prefix) == 0) {
    strlcpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  }

  if (shouldSaveConfig) {
    saveConfig();
    Serial.println("Config saved.");
  }

  activeSensor = sensorTypeFromCfg();

  // Init sensor and compute device_id (SEN66 prefers serial-based id)
  bool sensor_ok = false;
  switch (activeSensor) {
    case SENSOR_SEN66:   sensor_ok = initSen66(); break;
    case SENSOR_BMV080:  sensor_ok = initBmv080(); break;
    case SENSOR_PMS:     sensor_ok = initPms(); break;
    case SENSOR_IPS7100: sensor_ok = initIps7100(); break;
  }

  if (!sensor_ok) {
    Serial.println("Sensor init failed. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  safeTopicify(device_id);
  mqtt_client_id = "esp32_" + device_id;

  // Base topic namespace by sensor type (keeps your existing scheme)
  const char* prefix =
    (activeSensor == SENSOR_SEN66)   ? "sen66/"   :
    (activeSensor == SENSOR_BMV080)  ? "bmv080/"  :
    (activeSensor == SENSOR_PMS)     ? "pms/"     :
                                       "ips7100/";

  base_topic  = String(prefix) + device_id;
  avail_topic = base_topic + "/status";

  Serial.println("Base topic: " + base_topic);
  Serial.println("Availability topic: " + avail_topic);

  mqtt.setServer(cfg.mqtt_host, atoi(cfg.mqtt_port));
  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);

  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ---------------- Loop ----------------
void loop() {
  ensureMqtt();

  // Keep BMV080 serviced frequently even if we publish slower
  if (activeSensor == SENSOR_BMV080) {
    serviceBmv080(meas);
  }

  // Keep consuming IPS7100 UART so buffer doesn't overrun
  if (activeSensor == SENSOR_IPS7100) {
    serviceIps7100(meas);
  }

  // Publish cadence
  const unsigned long now = millis();
  if (now - last_publish_ms >= READ_INTERVAL_MS) {
    last_publish_ms = now;

    Measurements snap; // publish snapshot (avoid partial updates)
    snap = meas;

    bool ok = false;

    switch (activeSensor) {
      case SENSOR_SEN66:
        ok = readSen66(snap);
        if (ok) {
          Serial.printf("SEN66 PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f RH=%.1f T=%.2f VOC=%.1f NOx=%.1f CO2=%u\n",
                        snap.pm1, snap.pm25, snap.pm4, snap.pm10,
                        snap.rh, snap.tempC, snap.vocIdx, snap.noxIdx, snap.co2);
        }
        break;

      case SENSOR_BMV080:
        ok = readBmv080Snapshot(snap);
        if (ok) {
          Serial.printf("BMV080 PM1=%.1f PM2.5=%.1f PM10=%.1f Obstructed=%s\n",
                        snap.pm1, snap.pm25, snap.pm10, snap.obstructed ? "true" : "false");
        }
        break;

      case SENSOR_PMS:
        ok = readPms(snap);
        if (ok) {
          Serial.printf("PMS (ATM) PM1=%.1f PM2.5=%.1f PM10=%.1f\n",
                        snap.pm1, snap.pm25, snap.pm10);
        }
        break;

      case SENSOR_IPS7100:
        ok = snap.has_pm();
        if (ok) {
          Serial.printf("IPS7100 PM1=%.1f PM2.5=%.1f PM10=%.1f\n",
                        snap.pm1, snap.pm25, snap.pm10);
        }
        break;
    }

    if (ok) {
      if (!discovery_published && mqtt.connected()) publishDiscovery();

      // Common PM trio always published consistently (float, 1 decimal)
      publishPmTrio(snap);

      // Sensor-specific extras
      if (activeSensor == SENSOR_SEN66) publishSen66Extras(snap);
      if (activeSensor == SENSOR_BMV080) publishBmv080Extras(snap);
    } else {
      Serial.println("No valid readings at publish time.");
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
  pmsSerial.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(1200);
  device_id = "pms_" + chipId();
  return true;
}

bool initIps7100() {
  pmsSerial.begin(IPS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  pmsSerial.setTimeout(50);
  ips_line_buf.reserve(256);
  delay(500);
  device_id = "ips7100_" + chipId();
  return true;
}

// --------------- Sensor reads/services ---------------
static bool readSen66(Measurements& m) {
  float pm1=0, pm25=0, pm4=0, pm10=0;
  float rh=0, tempC=0, voc=0, nox=0;
  uint16_t co2=0;

  error = sen66.readMeasuredValues(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("SEN66 readMeasuredValues() error: %s\n", errorMessage);
    return false;
  }

  m.pm1  = pm1;
  m.pm25 = pm25;
  m.pm4  = pm4;
  m.pm10 = pm10;

  m.rh     = rh;
  m.tempC  = tempC;
  m.vocIdx = voc;
  m.noxIdx = nox;
  m.co2    = co2;
  m.has_env = true;

  return true;
}

static void serviceBmv080(Measurements& m) {
  const unsigned long now = millis();
  if (now - last_bmv_service_ms < BMV_SERVICE_MS) return;
  last_bmv_service_ms = now;

  if (bmv080.readSensor()) {
    // Treat everything as float µg/m³
    m.pm1  = bmv080.PM1();
    m.pm25 = bmv080.PM25();
    m.pm10 = bmv080.PM10();

    m.obstructed = bmv080.isObstructed();
    m.has_obstructed = true;
  }
}

static bool readBmv080Snapshot(Measurements& m) {
  // Values should already be kept warm by serviceBmv080()
  return m.has_pm();
}

static bool readPms(Measurements& m) {
  uint16_t pm1=0, pm25=0, pm10=0;
  if (!pmsReadAtm(pm1, pm25, pm10)) return false;

  // Normalize to float µg/m³ for consistency
  m.pm1  = float(pm1);
  m.pm25 = float(pm25);
  m.pm10 = float(pm10);
  return true;
}

static bool serviceIps7100(Measurements& m) {
  bool updated = false;
  String l;

  while (ipsReadCsvLine(l, 0)) {
    float pm1, pm25, pm10;
    if (ipsParsePmFromLine(l, pm1, pm25, pm10)) {
      m.pm1  = pm1;
      m.pm25 = pm25;
      m.pm10 = pm10;
      updated = true;
    }
  }

  return updated;
}

// --------------- Publishing (consistent formatting) ---------------
static void publishPmTrio(const Measurements& m) {
  // consistent: float, one decimal, µg/m³
  publishOne("pm1",  String(m.pm1, 1));
  publishOne("pm25", String(m.pm25, 1));
  publishOne("pm10", String(m.pm10, 1));
}

static void publishSen66Extras(const Measurements& m) {
  publishOne("pm4", String(m.pm4, 1));

  if (m.has_env) {
    publishOne("humidity",    String(m.rh, 1));
    publishOne("temperature", String(m.tempC, 2));
    publishOne("voc_index",   String(m.vocIdx, 1));
    publishOne("nox_index",   String(m.noxIdx, 1));
    publishOne("co2eq",       String(m.co2));
  }
}

static void publishBmv080Extras(const Measurements& m) {
  if (m.has_obstructed) {
    publishOne("obstructed", m.obstructed ? "ON" : "OFF");
  }
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

    if (u16be(f + 2) != 0x001C) continue;

    uint32_t sum = 0;
    for (int i = 0; i < 30; i++) sum += f[i];
    if ((uint16_t)sum != u16be(f + 30)) continue;

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

  strlcpy(cfg.mqtt_host, host.c_str(), sizeof(cfg.mqtt_host));
  strlcpy(cfg.mqtt_port, port.c_str(), sizeof(cfg.mqtt_port));
  strlcpy(cfg.mqtt_user, user.c_str(), sizeof(cfg.mqtt_user));
  strlcpy(cfg.mqtt_pass, pass.c_str(), sizeof(cfg.mqtt_pass));
  strlcpy(cfg.ha_prefix, pref.c_str(), sizeof(cfg.ha_prefix));
  strlcpy(cfg.friendly_name, name.c_str(), sizeof(cfg.friendly_name));
  strlcpy(cfg.sensor_type, styp.c_str(), sizeof(cfg.sensor_type));

  if (strlen(cfg.ha_prefix) == 0) strlcpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  if (strlen(cfg.sensor_type) == 0) strlcpy(cfg.sensor_type, "sen66", sizeof(cfg.sensor_type));

  normalizeSensorTypeInCfg();

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
static void haDeviceMeta(String& outManufacturer, String& outModel) {
  switch (activeSensor) {
    case SENSOR_SEN66:   outManufacturer = "Sensirion";     outModel = "SEN66"; break;
    case SENSOR_BMV080:  outManufacturer = "Bosch";         outModel = "BMV080"; break;
    case SENSOR_IPS7100: outManufacturer = "Piera Systems"; outModel = "IPS-7100"; break;
    case SENSOR_PMS:     outManufacturer = "PMS-compatible"; outModel = "PMS UART"; break;
  }
}

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

  // PM trio is always present across all sensors (consistent topics + units)
  publishConfigSensor("pm1",  "PM1.0", "µg/m³", "pm1",  "measurement", "mdi:blur", "pm1");
  publishConfigSensor("pm25", "PM2.5", "µg/m³", "pm25", "measurement", "",         "pm25");
  publishConfigSensor("pm10", "PM10",  "µg/m³", "pm10", "measurement", "",         "pm10");

  // SEN66 extras
  if (activeSensor == SENSOR_SEN66) {
    publishConfigSensor("pm4",         "PM4.0",       "µg/m³", "",               "measurement", "mdi:blur",            "pm4");
    publishConfigSensor("humidity",    "Humidity",    "%",     "humidity",       "measurement", "",                    "humidity");
    publishConfigSensor("temperature", "Temperature", "°C",    "temperature",    "measurement", "",                    "temperature");
    publishConfigSensor("voc_index",   "VOC Index",   "",      "",               "measurement", "mdi:chemical-weapon", "voc_index");
    publishConfigSensor("nox_index",   "NOx Index",   "",      "",               "measurement", "mdi:chemical-weapon", "nox_index");
    publishConfigSensor("co2eq",       "CO2 (eq)",    "ppm",   "carbon_dioxide", "measurement", "",                    "co2eq");
  }

  // BMV080 extras
  if (activeSensor == SENSOR_BMV080) {
    publishConfigBinarySensor("obstructed", "Optics Obstructed",
                              "problem", "mdi:alert-circle",
                              "obstructed",
                              "ON", "OFF");
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

  String manufacturer, model;
  haDeviceMeta(manufacturer, model);

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
  payload +=   "\"manufacturer\":\"" + manufacturer + "\",";
  payload +=   "\"model\":\"" + model + "\",";
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

  String manufacturer, model;
  haDeviceMeta(manufacturer, model);

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
  payload +=   "\"manufacturer\":\"" + manufacturer + "\",";
  payload +=   "\"model\":\"" + model + "\",";
  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("HA binary_sensor config publish '%s': %s (len=%d)\n",
                object_id.c_str(), ok ? "OK" : "FAILED", payload.length());
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

// ---------------- Small helpers ----------------
static void copyParam(char* dst, size_t dst_sz, const char* src) {
  if (!dst || dst_sz == 0) return;
  if (!src) { dst[0] = 0; return; }
  strlcpy(dst, src, dst_sz);
}

static void normalizeSensorTypeInCfg() {
  String st = String(cfg.sensor_type);
  st.trim();
  st.toLowerCase();
  if (st != "bmv080" && st != "pms" && st != "ips7100" && st != "sen66") st = "sen66";
  strlcpy(cfg.sensor_type, st.c_str(), sizeof(cfg.sensor_type));
}

static SensorType sensorTypeFromCfg() {
  String st = String(cfg.sensor_type);
  st.trim(); st.toLowerCase();
  if (st == "bmv080") return SENSOR_BMV080;
  if (st == "pms") return SENSOR_PMS;
  if (st == "ips7100") return SENSOR_IPS7100;
  return SENSOR_SEN66;
}
