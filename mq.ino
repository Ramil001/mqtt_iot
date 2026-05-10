/*
 * TTGO T-A7670SA + Neo-M8N + MPU-6050 -> MQTT mit/tracker/ingest
 * RMC+GGA+GSV — fix, GPS, MPU tilt.
 *
 * BLE mit/<METRICS_HUB_ID>/metrics: ENABLE_BLE_PRESSURE_METRICS 1 + METRICS_HUB_ID.
 * BLE init лише після GPRS; паузи модема як у вашій стабільній 1.2.3 (3500/300/3000).
 * SIM7600/A7670: AT+CNMP=2 на багатьох прошивках = лише UMTS; без 3G реєстрації GPRS не буде — спробуйте MODEM_NETWORK_MODE 0.
 * LilyGo T-A7670: UART до PWR, потім BOARD_POWERON(12), reset(5), DTR(25)=LOW, PWRKEY — без DTR модем часто мовчить на AT.
 * Інші ревізії (T-Call): інші піни — див. LilyGo utilities.h; задайте MODEM_* / MODEM_RESET_PIN -1 якщо немає reset.
 */
#define DEVICE_TOKEN "trk_37d4068b29dad73a49c2cf9c49b3e7bf703272a6"
#define APN          "internet"
#define APN_USER     ""
#define APN_PASS     ""
#define MQTT_HOST    "iot.mit.kh.ua"
#define MQTT_PORT    1883
#define MQTT_USER    "mit_iot"
#define MQTT_PASS    "5KtXCmjcPcVriwpYyidzK0"
#define FW_VERSION   "1.4.5-lilygo-dtr-seq"
#ifndef SIM_PIN
#define SIM_PIN ""
#endif
#ifndef MODEM_NETWORK_MODE
#define MODEM_NETWORK_MODE 2
#endif
#ifndef MODEM_NETWORK_WAIT_MS
#define MODEM_NETWORK_WAIT_MS 120000
#endif
#ifndef MODEM_POST_PWRKEY_MS
#define MODEM_POST_PWRKEY_MS 8500
#endif
#ifndef MODEM_INIT_RETRIES
#define MODEM_INIT_RETRIES 8
#endif
#ifndef MODEM_INIT_RETRY_GAP_MS
#define MODEM_INIT_RETRY_GAP_MS 1500
#endif
#ifndef MODEM_UART_AUTO_SWAP
#define MODEM_UART_AUTO_SWAP 1
#endif

#define GPS_RX_PIN 22
#define GPS_TX_PIN 21
#define I2C_SDA_PIN 33
#define I2C_SCL_PIN 32
#define MODEM_RX 27
#define MODEM_TX 26
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 12
#ifndef MODEM_DTR_PIN
#define MODEM_DTR_PIN 25
#endif
#ifndef MODEM_RESET_PIN
#define MODEM_RESET_PIN 5
#endif
#ifndef MODEM_RESET_LEVEL
#define MODEM_RESET_LEVEL HIGH
#endif
#ifndef MODEM_RAIL_STABLE_MS
#define MODEM_RAIL_STABLE_MS 600
#endif
#ifndef MODEM_PWRKEY_HOLD_MS
#define MODEM_PWRKEY_HOLD_MS 100
#endif
#define MQTT_TELEMETRY_MS 2000
#define MQTT_RETRY_MS 8000

/* 0 = як рання прошивка без BLE. 1 = BLE-метрики (потрібен METRICS_HUB_ID). */
#ifndef ENABLE_BLE_PRESSURE_METRICS
#define ENABLE_BLE_PRESSURE_METRICS 0
#endif
#ifndef METRICS_HUB_ID
#define METRICS_HUB_ID ""
#endif
#ifndef BLE_SCAN_INTERVAL_MS
#define BLE_SCAN_INTERVAL_MS 45000
#endif
#ifndef BLE_SCAN_DURATION_S
#define BLE_SCAN_DURATION_S 1
#endif
#ifndef BLE_NAME_PREFIX
#define BLE_NAME_PREFIX ""
#endif
#ifndef BLE_MFGR_COMPANY_ID
#define BLE_MFGR_COMPANY_ID 0
#endif
#ifndef BLE_OFF_BATTERY
#define BLE_OFF_BATTERY 3
#endif
#ifndef BLE_OFF_PRESS_RAW
#define BLE_OFF_PRESS_RAW 4
#endif
#ifndef BLE_OFF_TEMP_RAW
#define BLE_OFF_TEMP_RAW 6
#endif
#ifndef BLE_SCALE_PRESS
#define BLE_SCALE_PRESS 0.01f
#endif
#ifndef BLE_SCALE_TEMP
#define BLE_SCALE_TEMP 0.1f
#endif
#ifndef BLE_SCAN_START_RETURNS_POINTER
#define BLE_SCAN_START_RETURNS_POINTER 1
#endif

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <Arduino.h>
#include <esp_system.h>
#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define SerialMon Serial
#define SerialAT Serial1
#define SerialGPS Serial2

TinyGsm modem(SerialAT);
TinyGsmClient mqttClient(modem);
PubSubClient mqtt(mqttClient);

#if ENABLE_BLE_PRESSURE_METRICS
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#endif

static String deviceToken = String(DEVICE_TOKEN);
static String imeiCached;
static uint32_t lastMqttMs;
static uint32_t lastMqttTryMs;
#if ENABLE_BLE_PRESSURE_METRICS
static uint32_t lastBleScanMs;
static bool bleStackInited = false;
struct BleDedup { bool used; uint8_t mac[6]; uint32_t tms; float press; };
static BleDedup bleDedup[12];
#endif

static char nmea[256];
static size_t nmeaN;
static bool gpsFix = false;
static double gpsLat = 0.0;
static double gpsLon = 0.0;
static int gpsSatsUsed = -1;
static int gpsSatsInView = -1;
static float gpsHdop = -1.0f;

static int imuAddr = -1;
static const uint8_t IMU_ADDRS[] = {0x68, 0x69};

static char nmeaTmp[292];

enum { GPS_MAX_CHARS_PER_LOOP = 128 };


/* NMEA ddmm.mmmm (lat) / dddmm.mmmm (lon): first degw chars = deg, rest = min. */
static double nmeaCoord(const char *ddmm, char hemi, int degw) {
  if (!ddmm || !ddmm[0] || degw < 2) return NAN;
  int dot = -1;
  for (size_t i = 0; ddmm[i] && i < 32; i++)
    if (ddmm[i] == '.') dot = (int)i;
  if (dot < 1) return NAN;

  int dw = degw;
  /* Some receivers send lon as "3613.xxx" (36 deg 13 min) not "03613.xxx". */
  if (degw == 3 && dot == 4 && ddmm[0] != '0') dw = 2;

  size_t len = (size_t)dot;
  if (len < (size_t)dw + 1) return NAN;

  char dgp[8], mn[18];
  if (dw >= (int)sizeof(dgp)) return NAN;
  memcpy(dgp, ddmm, (size_t)dw);
  dgp[dw] = 0;
  strncpy(mn, ddmm + dw, sizeof(mn) - 1);
  mn[sizeof(mn) - 1] = 0;
  if (!mn[0]) return NAN;

  double o = atof(dgp) + atof(mn) / 60.0;
  if (hemi == 'S' || hemi == 'W') o = -o;
  return o;
}

/** $GPRMC / $GNRMC */
static void handleRmcLine(char *line) {
  if (!strstr(line, "RMC,")) return;

  strncpy(nmeaTmp, line, sizeof(nmeaTmp) - 1);
  nmeaTmp[sizeof(nmeaTmp) - 1] = 0;

  char *comma = strchr(nmeaTmp, ',');
  if (!comma) return;

  char *save = nullptr;
  (void)strtok_r(comma + 1, ",", &save);
  char *stat = strtok_r(nullptr, ",", &save);
  if (stat && stat[0] == 'V') {
    gpsFix = false;
    return;
  }
  if (!stat || stat[0] != 'A') return;

  char *flat = strtok_r(nullptr, ",", &save);
  char *fh = strtok_r(nullptr, ",", &save);
  char *flon = strtok_r(nullptr, ",", &save);
  char *gh = strtok_r(nullptr, ",", &save);
  if (!flat || !fh || !flon || !gh) return;

  double la = nmeaCoord(flat, fh[0], 2);
  double lo = nmeaCoord(flon, gh[0], 3);
  if (!isnan(la) && !isnan(lo)) {
    gpsLat = la;
    gpsLon = lo;
    gpsFix = true;
  }
}

/** $..GGA */
static void handleGgaLine(char *line) {
  if (!strstr(line, "GGA,")) return;

  strncpy(nmeaTmp, line, sizeof(nmeaTmp) - 1);
  nmeaTmp[sizeof(nmeaTmp) - 1] = 0;

  char *comma = strchr(nmeaTmp, ',');
  if (!comma) return;

  char fld[22][14];
  char *tok = comma + 1;
  size_t nf = 0;
  while (nf < sizeof(fld) / sizeof(fld[0])) {
    char *sep = strchr(tok, ',');
    char *star = strchr(tok, '*');
    char *cut = sep;
    if (sep && star && star < sep) cut = nullptr;
    else if (!sep && star) cut = star;
    else if (sep && (!star || star > sep)) cut = sep;

    size_t ln = cut ? (size_t)(cut - tok) : strlen(tok);
    if (ln >= sizeof(fld[0])) ln = sizeof(fld[0]) - 1;
    memcpy(fld[nf], tok, ln);
    fld[nf][ln] = 0;
    nf++;
    if (!cut || *cut == '*') break;
    tok = cut + 1;
  }

  if (nf < 8) return;

  int q = atoi(fld[5]);
  if (q > 0) gpsFix = true;
  /* GGA q==0 не скидає fix: RMC(A) вже міг дати координати; інакше довгі NO_FIX при used>0. */

  if (fld[6][0]) gpsSatsUsed = atoi(fld[6]);
  else if (q > 0) gpsSatsUsed = 0;

  gpsHdop = -1.0f;
  if (nf > 7 && fld[7][0]) {
    float h = strtof(fld[7], nullptr);
    if (h > 0.05f && h < 51.0f) gpsHdop = h;
  }

  if (q > 0 && fld[1][0] && fld[2][0] && fld[3][0] && fld[4][0]) {
    double la = nmeaCoord(fld[1], fld[2][0], 2);
    double lo = nmeaCoord(fld[3], fld[4][0], 3);
    if (!isnan(la) && !isnan(lo)) {
      gpsLat = la;
      gpsLon = lo;
    }
  }
}

/** Перший фрагмент GSV містить загальну кількість видимих супутників у третій колонці після першого коми заголовку. */
static void handleGsvLine(char *line) {
  if (!strstr(line, "GSV,")) return;

  strncpy(nmeaTmp, line, sizeof(nmeaTmp) - 1);
  nmeaTmp[sizeof(nmeaTmp) - 1] = 0;

  char *comma = strchr(nmeaTmp, ',');
  if (!comma) return;

  char *fld[24];
  int nf = nmeaCsvFields(comma + 1, fld, 23);
  if (nf >= 4 && fld[1][0] && atoi(fld[1]) == 1 && fld[2][0])
    gpsSatsInView = atoi(fld[2]);
}

static void parseNmeaLine(char *ln) {
  if (strstr(ln, "RMC,")) handleRmcLine(ln);
  else if (strstr(ln, "GGA,")) handleGgaLine(ln);
  else if (strstr(ln, "GSV,")) handleGsvLine(ln);
}

static void pollGps() {
  int drained = 0;
  while (SerialGPS.available() && drained++ < GPS_MAX_CHARS_PER_LOOP) {
    char c = (char)SerialGPS.read();
    if (c == '\n' || c == '\r') {
      if (nmeaN >= 8 && nmea[0] == '$') {
        nmea[nmeaN] = 0;
        parseNmeaLine(nmea);
      }
      nmeaN = 0;
    } else if (nmeaN < sizeof(nmea) - 1) {
      if (c >= ' ' && c <= '~') nmea[nmeaN++] = c;
      else if (nmeaN > 0) nmeaN = 0;
    } else
      nmeaN = 0;
  }
}

static int rssiDbm() {
  int q = modem.getSignalQuality();
  if (q == 99 || q < 0) return -120;
  return -113 + q * 2;
}

static const char *simStatusLabel(SimStatus st) {
  switch (st) {
    case SIM_READY: return "READY";
    case SIM_LOCKED: return "PIN";
    case SIM_ANTITHEFT_LOCKED: return "ANTITHEFT";
    default: return "ERR";
  }
}

static void bootLogResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  const char *name = "?";
  switch (r) {
    case ESP_RST_POWERON: name = "POWERON"; break;
    case ESP_RST_SW: name = "SW"; break;
    case ESP_RST_PANIC: name = "PANIC"; break;
    case ESP_RST_INT_WDT: name = "INT_WDT"; break;
    case ESP_RST_TASK_WDT: name = "TASK_WDT"; break;
    case ESP_RST_WDT: name = "WDT"; break;
    case ESP_RST_DEEPSLEEP: name = "DEEPSLEEP"; break;
    case ESP_RST_BROWNOUT: name = "BROWNOUT"; break;
    case ESP_RST_SDIO: name = "SDIO"; break;
    default: name = "OTHER"; break;
  }
  SerialMon.printf("[BOOT] esp_reset_reason=%d %s\n", (int)r, name);
  SerialMon.flush();
}

static void drainModemRx(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialAT.available()) (void)SerialAT.read();
    delay(10);
  }
}

static bool modemRawAtOnce(uint32_t baud, int rxPin, int txPin) {
  SerialAT.end();
  delay(120);
  SerialAT.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(200);
  drainModemRx(150);
  SerialAT.print("AT\r\n");
  SerialAT.flush();
  uint32_t t0 = millis();
  char acc[96];
  size_t n = 0;
  while (millis() - t0 < 700) {
    while (SerialAT.available()) {
      char c = (char)SerialAT.read();
      if (n < sizeof(acc) - 1) acc[n++] = c;
      acc[n] = 0;
      if (strstr(acc, "OK")) return true;
    }
    delay(3);
  }
  return false;
}

static bool modemFindUart(int rxPin, int txPin) {
  static const uint32_t rates[] = {115200u, 9600u, 57600u, 230400u};
  const unsigned nr = sizeof(rates) / sizeof(rates[0]);
  for (unsigned i = 0; i < nr; i++) {
    if (modemRawAtOnce(rates[i], rxPin, txPin)) {
      SerialMon.printf("[BOOT] modem AT OK baud=%lu rx=%d tx=%d\n", (unsigned long)rates[i], rxPin, txPin);
      return true;
    }
    SerialMon.printf("[BOOT] modem AT no reply baud=%lu\n", (unsigned long)rates[i]);
  }
  return false;
}

static void modemEnsureAtLink() {
  if (modemFindUart(MODEM_RX, MODEM_TX)) return;
#if MODEM_UART_AUTO_SWAP
  bootLog("[BOOT] modem probe: try UART swap (ESP RX/TX vs modem)");
  if (modemFindUart(MODEM_TX, MODEM_RX)) return;
#endif
  bootLog("[BOOT] modem probe failed: check wiring, modem supply, PWRKEY");
}

static bool initModemAt(const char *pin) {
  drainModemRx(200);
  for (int a = 1; a <= (int)MODEM_INIT_RETRIES; a++) {
    if (modem.init(pin)) {
      if (a > 1) SerialMon.printf("[BOOT] modem.init ok (try %d)\n", a);
      return true;
    }
    SerialMon.printf("[BOOT] modem.init fail try %d/%d, wait %lums\n", a, (int)MODEM_INIT_RETRIES,
                     (unsigned long)MODEM_INIT_RETRY_GAP_MS);
    delay((uint32_t)MODEM_INIT_RETRY_GAP_MS);
  }
  return false;
}

static void logModemNet(const char *tag) {
  SimStatus sim = modem.getSimStatus(8000);
  int csq = modem.getSignalQuality();
  int reg = (int)modem.getRegistrationStatus();
  String op = modem.getOperator();
  op.trim();
  SerialMon.printf("[BOOT] %s SIM=%s CSQ=%d(%ddBm) CGREG=%d op=%s\n", tag, simStatusLabel(sim), csq, rssiDbm(), reg,
                   op.length() ? op.c_str() : "-");
}

static bool ensureGprs() {
  if (modem.isGprsConnected()) return true;
  bootLog("[GPRS] gprsConnect...");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
    bootLog("[GPRS] FAIL");
    return false;
  }
  bootLog("[GPRS] ok");
  return true;
}

static bool mqttReconnect() {
  if (!ensureGprs()) return false;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(ENABLE_BLE_PRESSURE_METRICS ? 1024 : 512);
  String cid = String("trk_") + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (!mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
    SerialMon.printf("[MQTT] connect fail rc=%d\n", mqtt.state());
    return false;
  }
  bootLog("[MQTT] connected");
  return true;
}

static bool publishMqtt() {
  if (!mqtt.connected() && !mqttReconnect()) return false;

  float tlt = tiltDeg();
  StaticJsonDocument<512> doc;

  doc["gpsFix"] = gpsFix;
  if (gpsFix) {
    doc["lat"] = gpsLat;
    doc["lng"] = gpsLon;
  }
  doc["tiltDeg"] = tlt;
  doc["rssi"] = rssiDbm();
  doc["fwVersion"] = FW_VERSION;
  doc["mode"] = "disarmed";
  doc["deviceToken"] = deviceToken;
  if (imeiCached.length()) doc["imei"] = imeiCached;

  if (gpsSatsUsed >= 0) doc["gpsSatsUsed"] = gpsSatsUsed;
  if (gpsSatsInView >= 0) doc["gpsSatsInView"] = gpsSatsInView;
  if (gpsHdop >= 0) doc["gpsHdop"] = gpsHdop;

  char payload[512];
  size_t L = serializeJson(doc, payload, sizeof(payload));
  if (!L || L >= sizeof(payload)) return false;
  if (!mqtt.publish("mit/tracker/ingest", payload)) return false;

  if (gpsFix)
    SerialMon.printf("[MQTT] ok FIX lat=%.6f lng=%.6f used=%d in_view=%d tilt %.1f\n", gpsLat, gpsLon,
                     gpsSatsUsed >= 0 ? gpsSatsUsed : -1, gpsSatsInView >= 0 ? gpsSatsInView : -1,
                     (double)tlt);
  else
    SerialMon.printf("[MQTT] ok NO_FIX used=%d in_view=%d tilt %.1f\n",
                     gpsSatsUsed >= 0 ? gpsSatsUsed : -1, gpsSatsInView >= 0 ? gpsSatsInView : -1,
                     (double)tlt);
  return true;
}



static int nmeaCsvFields(char *buf, char **out, int cap) {
  char *star = strchr(buf, '*');
  if (star) *star = 0;
  int nf = 0;
  char *save = nullptr;
  for (char *t = strtok_r(buf, ",", &save); t && nf < cap; t = strtok_r(nullptr, ",", &save))
    out[nf++] = t;
  return nf;
}

static void bootLog(const char *msg) {
  SerialMon.println(msg);
  SerialMon.flush();
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool initImu() {
  for (uint8_t i = 0; i < sizeof(IMU_ADDRS); i++) {
    uint8_t a = IMU_ADDRS[i];
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    uint8_t who = 0;
    if (!i2cReadBytes(a, 0x75, &who, 1) || who == 0 || who == 0xFF) continue;
    imuAddr = (int)a;
    i2cWrite8(a, 0x6B, 0x00);
    i2cWrite8(a, 0x1B, 0x00);
    i2cWrite8(a, 0x1C, 0x00);
    SerialMon.printf("[IMU] 0x%02X\n", a);
    return true;
  }
  return false;
}

static float tiltDeg() {
  if (imuAddr < 0) return 0.0f;
  uint8_t raw[14];
  if (!i2cReadBytes((uint8_t)imuAddr, 0x3B, raw, 14)) return 0.0f;
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  float axg = ax / 16384.0f, ayg = ay / 16384.0f, azg = az / 16384.0f;
  float xy = sqrtf(axg * axg + ayg * ayg);
  return atan2f(xy, fabsf(azg)) * 57.29578f;
}

#if ENABLE_BLE_PRESSURE_METRICS
static bool bleHubOk() {
  return METRICS_HUB_ID[0] != '\0';
}

static bool bleParseMac6(const char *addr, uint8_t out[6]) {
  if (!addr || strlen(addr) < 17) return false;
  unsigned u[6];
  if (sscanf(addr, "%2x:%2x:%2x:%2x:%2x:%2x", &u[0], &u[1], &u[2], &u[3], &u[4], &u[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)u[i];
  return true;
}

static bool bleNameAllowed(const String &nm) {
  if (!BLE_NAME_PREFIX[0]) return true;
  if (!nm.length()) return false;
  return nm.startsWith(BLE_NAME_PREFIX);
}

static bool bleDecodeManufacturer(const uint8_t *b, size_t len, float *pressKpa, float *tempC, int *battPct) {
  if (len < (size_t)BLE_OFF_TEMP_RAW + 2 || !b) return false;
  if (BLE_MFGR_COMPANY_ID != 0) {
    uint16_t cid = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    if (cid != (uint16_t)BLE_MFGR_COMPANY_ID) return false;
  }
  if (BLE_OFF_BATTERY >= 0 && (size_t)BLE_OFF_BATTERY < len) {
    int bt = (int)b[BLE_OFF_BATTERY];
    *battPct = (bt >= 0 && bt <= 100) ? bt : 100;
  } else {
    *battPct = 100;
  }
  int16_t rawP = (int16_t)((uint16_t)b[BLE_OFF_PRESS_RAW] | ((uint16_t)b[BLE_OFF_PRESS_RAW + 1] << 8));
  int16_t rawT = (int16_t)((uint16_t)b[BLE_OFF_TEMP_RAW] | ((uint16_t)b[BLE_OFF_TEMP_RAW + 1] << 8));
  *pressKpa = (float)rawP * BLE_SCALE_PRESS;
  *tempC = (float)rawT * BLE_SCALE_TEMP;
  return true;
}

static bool bleShouldPublish(const uint8_t *mac, float press, uint32_t nowMs) {
  for (unsigned i = 0; i < sizeof(bleDedup) / sizeof(bleDedup[0]); i++) {
    if (!bleDedup[i].used) continue;
    if (memcmp(bleDedup[i].mac, mac, 6) != 0) continue;
    bool tmo = (nowMs - bleDedup[i].tms) > 60000u;
    bool chg = fabsf(press - bleDedup[i].press) > 0.08f;
    if (!tmo && !chg) return false;
    bleDedup[i].tms = nowMs;
    bleDedup[i].press = press;
    return true;
  }
  for (unsigned i = 0; i < sizeof(bleDedup) / sizeof(bleDedup[0]); i++) {
    if (!bleDedup[i].used) {
      memcpy(bleDedup[i].mac, mac, 6);
      bleDedup[i].used = true;
      bleDedup[i].tms = nowMs;
      bleDedup[i].press = press;
      return true;
    }
  }
  bleDedup[0].used = true;
  memcpy(bleDedup[0].mac, mac, 6);
  bleDedup[0].tms = nowMs;
  bleDedup[0].press = press;
  return true;
}

static bool publishBleMetric(const char *devName, const char *macStr, int bleRssi, float pressKpa, float tempC, int battPct) {
  if (!mqtt.connected() && !mqttReconnect()) return false;
  StaticJsonDocument<512> doc;
  doc["hub"] = METRICS_HUB_ID;
  doc["device"] = devName;
  doc["mac"] = macStr;
  doc["temp"] = tempC;
  doc["press"] = pressKpa;
  doc["battery"] = battPct;
  doc["rssi"] = String(bleRssi);
  if (imeiCached.length()) doc["imei"] = imeiCached.c_str();
  doc["fw_version"] = FW_VERSION;
  if (gpsFix) {
    doc["hub_lat"] = String(gpsLat, 6);
    doc["hub_lng"] = String(gpsLon, 6);
  }
  char payload[512];
  size_t L = serializeJson(doc, payload, sizeof(payload));
  if (!L || L >= sizeof(payload)) return false;
  char topic[96];
  snprintf(topic, sizeof(topic), "mit/%s/metrics", METRICS_HUB_ID);
  if (!mqtt.publish(topic, payload)) return false;
  SerialMon.printf("[BLE->MQTT] %s %s P=%.2f T=%.1f bat=%d\n", devName, macStr, (double)pressKpa, (double)tempC, battPct);
  return true;
}

static void bleScanAndPublishMetrics() {
  if (!bleHubOk()) return;
  if (!ensureGprs()) return;
  /* BLE після GPRS: ініціалізація стеку до модему ламала старт на деяких збірках. */
  if (!bleStackInited) {
    BLEDevice::init("");
    BLEDevice::getScan()->setActiveScan(true);
    bleStackInited = true;
    bootLog("[BLE] stack on (after GPRS ok)");
  }
  uint32_t now = millis();
  if (lastBleScanMs != 0 && (now - lastBleScanMs) < (uint32_t)BLE_SCAN_INTERVAL_MS) return;
  lastBleScanMs = now;
  BLEScan *scan = BLEDevice::getScan();
  scan->setActiveScan(true);
#if BLE_SCAN_START_RETURNS_POINTER
  BLEScanResults *foundPtr = scan->start((uint32_t)BLE_SCAN_DURATION_S, false);
  if (!foundPtr) {
    scan->clearResults();
    return;
  }
#else
  BLEScanResults foundVal = scan->start((uint32_t)BLE_SCAN_DURATION_S, false);
  BLEScanResults *foundPtr = &foundVal;
#endif
  for (int i = 0; i < foundPtr->getCount(); i++) {
    BLEAdvertisedDevice d = foundPtr->getDevice(i);
    if (!bleNameAllowed(d.getName())) continue;
    String mdStr = d.getManufacturerData();
    float pKpa = 0, tC = 0;
    int batt = 100;
    if (!bleDecodeManufacturer((const uint8_t *)mdStr.c_str(), (size_t)mdStr.length(), &pKpa, &tC, &batt)) continue;
    uint8_t m6[6];
    String addrStr = d.getAddress().toString();
    if (!bleParseMac6(addrStr.c_str(), m6)) continue;
    if (!bleShouldPublish(m6, pKpa, now)) continue;
    String nameStr = d.getName();
    const char *nm = nameStr.c_str();
    char fallback[24];
    if (!nm || !nm[0]) {
      snprintf(fallback, sizeof(fallback), "BLE-%02X%02X%02X%02X%02X%02X", m6[0], m6[1], m6[2], m6[3], m6[4], m6[5]);
      nm = fallback;
    }
    publishBleMetric(nm, addrStr.c_str(), d.getRSSI(), pKpa, tC, batt);
    mqtt.loop();
  }
  scan->clearResults();
}
#endif
static void modemPowerOn() {
  /* Порядок як у LilyGo examples/Network: спершу лінія живлення, reset, DTR, потім PWRKEY (тривалість імпульсу A7670 ~100 ms). */
  if (MODEM_POWER_ON >= 0) {
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_POWER_ON, HIGH);
    delay((uint32_t)MODEM_RAIL_STABLE_MS);
  }

  if (MODEM_RESET_PIN >= 0) {
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL ? LOW : HIGH);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL ? LOW : HIGH);
    delay(200);
  }

  if (MODEM_DTR_PIN >= 0) {
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
  }

  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay((uint32_t)MODEM_PWRKEY_HOLD_MS);
  digitalWrite(MODEM_PWRKEY, LOW);
}

void setup() {
  SerialMon.begin(115200);
  delay(2200);
  while (SerialMon.available()) (void)SerialMon.read();

  bootLog("");
  bootLog("[BOOT] firmware " FW_VERSION);
  bootLogResetReason();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  initImu() ? bootLog("[BOOT] IMU ok") : bootLog("[BOOT] no IMU");

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  bootLog("[BOOT] modem power");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(150);
  modemPowerOn();
  delay((uint32_t)MODEM_POST_PWRKEY_MS);

  modemEnsureAtLink();

  {
    const char *p = (SIM_PIN[0] != '\0') ? SIM_PIN : nullptr;
    if (!initModemAt(p)) bootLog("[BOOT] WARN modem.init failed (check PWRKEY, RX/TX, power)");
  }
  delay(500);
  modem.setPhoneFunctionality(1, false);
  delay(2000);

  imeiCached = modem.getIMEI();

  logModemNet("pre-CNMP");
  SerialMon.printf("[BOOT] setNetworkMode(%d)\n", (int)MODEM_NETWORK_MODE);
  modem.setNetworkMode(MODEM_NETWORK_MODE);
  delay(3000);
  logModemNet("post-CNMP");

  bootLog("[BOOT] waitForNetwork...");
  if (!modem.waitForNetwork((uint32_t)MODEM_NETWORK_WAIT_MS, false))
    bootLog("[BOOT] WARN waitForNetwork timeout");
  logModemNet("post-waitNet");

  bootLog("[BOOT] gprsConnect (retries)");
  bool gok = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    if (modem.gprsConnect(APN, APN_USER, APN_PASS)) {
      gok = true;
      break;
    }
    SerialMon.printf("[BOOT] gprs fail attempt %d/5, wait 6s\n", attempt);
    delay(6000);
  }
  if (!gok) bootLog("[BOOT] WARN gprs still down — loop will retry");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqttReconnect();

  bootLog("[BOOT] done");
  lastMqttTryMs = millis();
  lastMqttMs = millis();
}

void loop() {
  pollGps();
#if ENABLE_BLE_PRESSURE_METRICS
  bleScanAndPublishMetrics();
#endif
  mqtt.loop();

  uint32_t now = millis();
  if (!mqtt.connected()) {
    if (lastMqttTryMs == 0 || now - lastMqttTryMs >= MQTT_RETRY_MS) {
      lastMqttTryMs = now;
      mqttReconnect();
    }
  } else if (now - lastMqttMs >= (uint32_t)MQTT_TELEMETRY_MS) {
    if (publishMqtt())
      lastMqttMs = now;
    else if (mqtt.connected())
      mqtt.disconnect();
  }

  delay(20);
}
