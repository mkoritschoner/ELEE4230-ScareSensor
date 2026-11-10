#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"


#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #include "esp_eap_client.h"
  #define EAP_SET_IDENTITY(s) esp_eap_client_set_identity((const uint8_t*)(s), strlen(s))
  #define EAP_SET_USERNAME(s) esp_eap_client_set_username((const uint8_t*)(s), strlen(s))
  #define EAP_SET_PASSWORD(s) esp_eap_client_set_password((const uint8_t*)(s), strlen(s))
  #define EAP_ENABLE()        esp_wifi_sta_enterprise_enable()
#else
  #include "esp_wpa2.h"
  #define EAP_SET_IDENTITY(s) esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)(s), strlen(s))
  #define EAP_SET_USERNAME(s) esp_wifi_sta_wpa2_ent_set_username((uint8_t*)(s), strlen(s))
  #define EAP_SET_PASSWORD(s) esp_wifi_sta_wpa2_ent_set_password((uint8_t*)(s), strlen(s))
  #define EAP_ENABLE()        esp_wifi_sta_wpa2_ent_enable()
#endif

// ============================================================================
//  USER SETTINGS
// ============================================================================
// ---- Wi-Fi: set true for PAWS-Secure (WPA2-Enterprise), false for home Wi-Fi ----
const bool USE_ENTERPRISE_WIFI = true;

const char* ENT_SSID      = "PAWS-Secure";
const char* EAP_IDENTITY  = "myid";
const char* EAP_USERNAME  = "myid";
const char* EAP_PASSWORD  = "password";

const char* HOME_SSID     = "spectrum";
const char* HOME_PASSWORD = "password";

// ---- ThingSpeak ----
const char* TS_CHANNEL_ID = "3492346";
const char* TS_WRITE_KEY  = "xxx";

const uint32_t UPLOAD_INTERVAL_MS = 20000;  // periodic record + batch upload (for free tier: >= 15 s)
const uint32_t EVENT_MIN_GAP_MS   = 15500;  // an event can trigger an early upload after this gap

// ---- Pins ----
const int LED_PIN       = 2;
const int ADC_PIN       = 34;   // ADC1, shared by thermistor + ECM through the mux
const int MUX_PIN       = 13;   // LOW for ECM, HIGH for thermistor
const int CAP_DRIVE_PIN = 4;
const int CAP_SENSE_PIN = 35;
const int I2C_SDA       = 21;
const int I2C_SCL       = 22;

// ---- Thermistor chain  ----
const float VEX = 3.3f, R_BRIDGE = 470.0f, RP = 820.0f, RG = 5000.0f;
const float GAIN = 1.0f + (50000.0f / RG);
const float BIAS_V = 1.67f;
const float R0 = 1120.0f, T0_K = 298.15f, B_COEFF = 3636.0f;
const float THERM_CAL_OFFSET_K = 1.0f;   // BMP280 calibration offset used in the mux sketch
const float THERM_BRIDGE_SIGN  = 1.0f;

// ---- ADC ----
const float ADC_VREF = 3.3f;
const float ADC_MAXCOUNT = 4095.0f;

// ---- ECM: experimental calibration: dBA = 1.452*20*log10(V) + 72.16  (valid from 0.204 to 2.74 V) ----
const float ECM_V_FLOOR = 0.204f;   // <= this is "quiet" (about 50 dBA or less)
const float ECM_V_CEIL  = 2.74f;    // >= this is saturated (about 90 dBA or more)
const float ECM_DBA_FLOOR = 50.0f;
const float ECM_DBA_CEIL  = 90.0f;

// ---- BMP280 ----
const float BMP_TEMP_OFFSET_C = -3.0f;  // calibration offset from CP2
const float ALTITUDE_M        = 200.0f; // Athens, GA

// ---- IMU ----
const float OFFSET_X = -0.215f, OFFSET_Y = -0.015f, OFFSET_Z = -0.600f;

// ---- PPG  ----
const byte RATE_SIZE      = 8;
const byte ledBrightness  = 0xFF;
const byte sampleAverage  = 2;
const byte ledMode        = 2;
const int  sampleRate     = 100;
const int  pulseWidth     = 411;
const int  adcRange       = 16384;
const long IR_MIN_CONTACT = 10000;   // below this the sensor is not on skin, so heartrate is invalid

// ---- Cap sensor ----
const uint32_t CAP_THRESHOLD_US    = 70;
const uint8_t  CAP_DEBOUNCE_READS  = 3;      // consecutive touched reads needed
const uint32_t CAP_PERIOD_MS       = 50;

const uint32_t TOGGLE_LOCKOUT_MS   = 1500;

// ---- Loop timing ----
const uint32_t ECM_PERIOD_MS    = 10;
const uint32_t IMU_PERIOD_MS    = 10;
const uint32_t DETECT_PERIOD_MS = 10;
const uint32_t BMP_PERIOD_MS    = 2000;
const uint32_t THERM_PERIOD_MS  = 3000;
const uint32_t LOG_PERIOD_MS    = 100;

// ============================================================================
//  TYPES
// ============================================================================

// ===== DETECTOR BEGIN  =====
const float    FLINCH_THRESHOLD = 20.0f;    // m/s^2
const uint32_t WARMUP_MS        = 15000;    // learn baselines after start for this long (no detection)
const float    SOUND_JUMP_DB    = 15.0f;    // dBA above the slow baseline...
const float    SOUND_RISE_DB    = 10.0f;    // and this much above the quietest point of the last 0.5 s
const float    SOUND_MIN_DBA    = 65.0f;    // and at least this loud
const float    TAU_UP_S         = 8.0f;     // sound baseline falls faster than it rises
const float    TAU_DOWN_S       = 3.0f;     
const uint32_t FLINCH_PRE_MS    = 300;      
const uint32_t FLINCH_WINDOW_MS = 1500;     // flinch must occur within this time after the sound
const uint32_t HR_WINDOW_MS     = 10000;    // watch heart rate this long after the sound
const float    HR_RISE_BPM      = 10.0f;     // rise over baseline that counts as a startle response
const float    HR_BASE_TAU_S    = 20.0f;
const uint32_t HR_RECOVERY_MS   = 15000;    // HR baseline stays frozen this long after a scare window
const uint32_t COOLDOWN_MS      = 5000;     // no new detection right after a scare window

enum ScareEventType : uint8_t { EV_NONE = 0, EV_SCARE = 1, EV_NOREACT = 2, EV_PARTIAL = 3, EV_FELL = 4 };

struct ScareEvent {
  uint8_t  type;
  uint32_t tScare;      // millis() at sound onset
  uint32_t tEvent;      // millis() when this event was produced
  float    peakDba;     // loudest dBA seen (so far)
  float    jumpDb;      // dBA above baseline at onset
  float    baseDba;     // sound baseline at onset
  float    flinchPeak;  
  float    hrBase;      // baseline bpm at onset 
  float    hrPeak;      // highest bpm in the window 
  float    hrRise;      // hrPeak - hrBase
  bool     flinched;
  bool     hrUp;
  bool     hrValid;
};

// --- 1D Kalman Filter for Raw PPG IR Signal ---
class Kalman1D {
private:
  float q;           // Process noise covariance (responsiveness to actual PPG wave)
  float r;           // Measurement noise covariance (sensor noise)
  float x;           // Estimated state (filtered IR value)
  float p;           // Estimation error covariance
  float k;           // Kalman gain
  bool  initialized;

public:
  Kalman1D(float process_noise = 100.0f, float measurement_noise = 1500.0f)
    : q(process_noise), r(measurement_noise), p(1000.0f), x(0.0f), initialized(false) {}

  float update(float measurement) {
    if (!initialized) {
      x = measurement;
      p = 1000.0f;
      initialized = true;
      return x;
    }
    // Predict
    p = p + q;

    // Update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1.0f - k) * p;

    return x;
  }

  void reset() { initialized = false; }

  // Live tuning methods
  void setTunings(float process_noise, float measurement_noise) {
    q = process_noise;
    r = measurement_noise;
  }
};


Kalman1D irFilter(100.0f, 1500.0f); // (Q, R)
long g_irRaw = 0;
float g_irFiltered = 0.0f;

class ScareDetector {
 public:
  void begin(uint32_t now) {
    tStart = now; lastUpd = now; lastHist = now;
    baseInit = false; dbaSlow = ECM_DBA_FLOOR; histCount = 0; histIdx = 0;
    hrBase = NAN; hrBaseInit = false; hrFreezeUntil = now;
    lastFlinchMs = 0; lastFlinchMag = 0;
    phase = ARMED; aboveCount = 0; qHead = qTail = 0;
  }

  // dba: sound level; accel: IMU magnitude (m/s^2); hr: fast heart rate in bpm, NAN if invalid
  void update(uint32_t now, float dba, float accel, float hr) {
    float dt = (now - lastUpd) * 0.001f;
    lastUpd = now;
    if (dt > 1.0f) dt = 1.0f;

    // --- remember the most recent flinch so a flinch just BEFORE the sound still counts ---
    if (accel >= FLINCH_THRESHOLD) {
      if ((now - lastFlinchMs) > FLINCH_PRE_MS || lastFlinchMag <= 0) lastFlinchMag = accel;
      else if (accel > lastFlinchMag) lastFlinchMag = accel;
      lastFlinchMs = now;
    }

    // --- 0.5 s sound history ---
    if ((now - lastHist) >= 50) {
      hist[histIdx] = dba;
      histIdx = (histIdx + 1) % 10;
      if (histCount < 10) histCount++;
      lastHist = now;
    }
    float histMin = dba;
    for (uint8_t i = 0; i < histCount; i++) if (hist[i] < histMin) histMin = hist[i];

    if (!baseInit) { dbaSlow = dba; baseInit = true; }

    // --- heart-rate baseline (frozen during a scare window and its recovery) ---
    if (!isnan(hr) && phase != WATCHING && (int32_t)(now - hrFreezeUntil) >= 0) {
      if (!hrBaseInit) { hrBase = hr; hrBaseInit = true; }
      else {
        float a = dt / HR_BASE_TAU_S; if (a > 1.0f) a = 1.0f;
        hrBase += a * (hr - hrBase);
      }
    }

    switch (phase) {
      case ARMED: {
        bool warm = (now - tStart) >= WARMUP_MS;
        bool cond = warm && dba >= SOUND_MIN_DBA &&
                    (dba - dbaSlow) >= SOUND_JUMP_DB &&
                    (dba - histMin) >= SOUND_RISE_DB;
        aboveCount = cond ? (uint8_t)(aboveCount + 1) : 0;
        if (aboveCount >= 2) {                       // 2 consecutive samples
          startScare(now, dba);
        } else {
          float tau = (dba > dbaSlow) ? TAU_UP_S : TAU_DOWN_S;
          float a = dt / tau; if (a > 1.0f) a = 1.0f;
          dbaSlow += a * (dba - dbaSlow);
        }
        break;
      }
      case WATCHING: {
        if (dba > peakDba) peakDba = dba;
        if ((now - tScare) <= FLINCH_WINDOW_MS && accel >= FLINCH_THRESHOLD) {
          flinched = true;
          if (accel > flinchPeak) flinchPeak = accel;
        }
        if (!isnan(hr)) hrPeak = isnan(hrPeak) ? hr : fmaxf(hrPeak, hr);
        bool hrUp = hrKnownAtScare && !isnan(hrPeak) && (hrPeak - hrBaseAtScare) >= HR_RISE_BPM;
        bool done = (now - tScare) >= HR_WINDOW_MS;
        if ((flinched && hrUp) || done) finishScare(now, hrUp);
        break;
      }
      case COOLDOWN: {
        if ((int32_t)(now - cooldownEnd) >= 0) { phase = ARMED; aboveCount = 0; }
        break;
      }
    }
  }

  bool popEvent(ScareEvent& ev) {
    if (qHead == qTail) return false;
    ev = q[qTail];
    qTail = (qTail + 1) % QN;
    return true;
  }

  float   baselineDba() const { return dbaSlow; }
  float   baselineHr()  const { return hrBase; }
  uint8_t phaseId()     const { return (uint8_t)phase; }
  bool    warmedUp(uint32_t now) const { return (now - tStart) >= WARMUP_MS; }

 private:
  enum Phase : uint8_t { ARMED = 0, WATCHING = 1, COOLDOWN = 2 };
  static const uint8_t QN = 4;

  uint32_t tStart = 0, lastUpd = 0, lastHist = 0;
  bool     baseInit = false;
  float    dbaSlow = 50.0f;
  float    hist[10] = {0};
  uint8_t  histIdx = 0, histCount = 0;
  float    hrBase = NAN;
  bool     hrBaseInit = false;
  uint32_t hrFreezeUntil = 0;
  uint32_t lastFlinchMs = 0;
  float    lastFlinchMag = 0;
  Phase    phase = ARMED;
  uint8_t  aboveCount = 0;
  uint32_t cooldownEnd = 0;

  // current scare
  uint32_t tScare = 0;
  float    peakDba = 0, scareJump = 0, scareBase = 0, flinchPeak = 0;
  bool     flinched = false;
  float    hrBaseAtScare = NAN, hrPeak = NAN;
  bool     hrKnownAtScare = false;

  ScareEvent q[QN];
  uint8_t qHead = 0, qTail = 0;

  void push(const ScareEvent& e) { q[qHead] = e; qHead = (qHead + 1) % QN; }

  void fill(ScareEvent& e, uint8_t type, uint32_t now) {
    e.type = type; e.tScare = tScare; e.tEvent = now;
    e.peakDba = peakDba; e.jumpDb = scareJump; e.baseDba = scareBase;
    e.flinchPeak = flinchPeak; e.flinched = flinched;
    e.hrBase = hrKnownAtScare ? hrBaseAtScare : NAN;
    e.hrPeak = hrPeak;
    e.hrValid = hrKnownAtScare && !isnan(hrPeak);
    e.hrRise = e.hrValid ? (hrPeak - hrBaseAtScare) : 0.0f;
    e.hrUp = e.hrValid && e.hrRise >= HR_RISE_BPM;
  }

  void startScare(uint32_t now, float dba) {
    tScare = now; peakDba = dba; scareBase = dbaSlow; scareJump = dba - dbaSlow;
    flinched = false; flinchPeak = 0;
    if (lastFlinchMag > 0 && (now - lastFlinchMs) <= FLINCH_PRE_MS) {   // flinch just before the sound
      flinched = true; flinchPeak = lastFlinchMag;
    }
    hrKnownAtScare = hrBaseInit && !isnan(hrBase);
    hrBaseAtScare = hrBase; hrPeak = NAN;
    phase = WATCHING;
    ScareEvent e; fill(e, EV_SCARE, now); push(e);
  }

  void finishScare(uint32_t now, bool hrUp) {
    bool hrValid = hrKnownAtScare && !isnan(hrPeak);
    uint8_t type;
    if (flinched && (hrUp || !hrValid)) type = EV_FELL;       // no useable heartrate means the flinch alone decides
    else if (flinched || hrUp)          type = EV_PARTIAL;
    else                                type = EV_NOREACT;
    ScareEvent e; fill(e, type, now); push(e);
    phase = COOLDOWN; cooldownEnd = now + COOLDOWN_MS;
    hrFreezeUntil = now + HR_RECOVERY_MS;
    aboveCount = 0;
  }
};
// ===== DETECTOR END =====

// ---- Upload record: one row in the ThingSpeak channel ----
struct UploadRecord {
  uint32_t t_ms;        // millis() when it happened
  float    field[8];    // NAN = leave this field empty
  char     status[80];  // optional text
};

const int UPLOAD_QUEUE_LEN = 40;
const int UPLOAD_BATCH_MAX = 16;

// ============================================================================
//  GLOBALS
// ============================================================================
Adafruit_BMP280 bmp;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
MAX30105 particleSensor;
ScareDetector detector;
QueueHandle_t uploadQueue = NULL;

bool bmpOK = false, imuOK = false, ppgOK = false;
bool measuring = false;

// latest values from each sensor function
float g_bmpTempF = NAN, g_pressureHpa = NAN;
float g_thermV = NAN, g_thermTempF = NAN;
float g_ecmV = 0, g_dba = ECM_DBA_FLOOR;
float g_accelMag = 0;
long  g_ir = 0;
float g_bpmInst = 0, g_hrFast = NAN;
int   g_beatAvg = 0;
bool  g_hrValid = false;
uint32_t g_capDelayUs = 0;

// PPG beats
byte     rates[RATE_SIZE];
byte     rateSpot = 0, rateCount = 0;
long     lastBeat = 0;
uint32_t lastAcceptedBeat = 0;
float    recent3[3] = {0, 0, 0};
byte     recent3Count = 0, recent3Spot = 0;

// accumulators for the periodic record
float g_maxDbaInterval = 0, g_maxAccelInterval = 0;

// cap sensor state
uint8_t  capCount = 0;
bool     capLatched = false;
uint32_t lastToggle = 0;
bool     everToggled = false;

// scheduler timestamps
uint32_t tCap = 0, tEcm = 0, tImu = 0, tBmp = 0, tTherm = 0, tDetect = 0, tLog = 0, tPeriodic = 0;
uint32_t tLed = 0;
bool ledState = false;

// ============================================================================
//  SENSOR FUNCTIONS (one per sensor)
// ============================================================================

// ---- shared ADC read through the mux ----
float readMuxVoltage() {
  unsigned long sum = 0;
  for (int i = 0; i < 5; i++) sum += analogRead(ADC_PIN);
  float raw_adc = sum / 5.0f;
  return 0.964147f * (raw_adc * (ADC_VREF / ADC_MAXCOUNT) + 0.1505f); // ADC characterization from CP2
}

// ---- BMP280: temp (F) and sea-level pressure (hPa) ----
void readBMP() {
  if (!bmpOK) return;
  float tC = bmp.readTemperature() + BMP_TEMP_OFFSET_C;
  float pRaw = bmp.readPressure() / 100.0f;
  g_bmpTempF = tC * 9.0f / 5.0f + 32.0f;
  g_pressureHpa = bmp.seaLevelForAltitude(ALTITUDE_M, pRaw);
}

// ---- Thermistor: mux to thermistor, read, run the inverse chain, mux back to ECM ----
void readThermistor() {
  digitalWrite(MUX_PIN, HIGH);
  delayMicroseconds(200);                      // let the mux settle
  float v = readMuxVoltage();
  digitalWrite(MUX_PIN, LOW);
  delayMicroseconds(200);                      // settle back to ECM

  float V_amp    = v - BIAS_V;
  float V_bridge = THERM_BRIDGE_SIGN * V_amp / GAIN;
  float k        = 0.5f - (V_bridge / VEX);
  float Rc       = (k * R_BRIDGE) / (1.0f - k);
  float R_therm  = (Rc * RP) / (RP - Rc);
  if (!isfinite(R_therm) || R_therm <= 0.0f) return;   // keep the last good value
  float T_K = 1.0f / (logf(R_therm / R0) / B_COEFF + (1.0f / T0_K));
  T_K += THERM_CAL_OFFSET_K;
  g_thermV = v;
  g_thermTempF = (T_K - 273.15f) * 9.0f / 5.0f + 32.0f;
}

// ---- ECM: volts after peak detector -> dBA (from experimental calibration) ----
float ecmVoltageToDba(float v) {
  if (v <= ECM_V_FLOOR) return ECM_DBA_FLOOR;
  if (v >= ECM_V_CEIL)  return ECM_DBA_CEIL;
  return 1.452f * 20.0f * log10f(v) + 72.16f; //instead of +94 dB
}

void readECM() {
  digitalWrite(MUX_PIN, LOW);
  g_ecmV = readMuxVoltage();
  g_dba = ecmVoltageToDba(g_ecmV);
  if (g_dba > g_maxDbaInterval) g_maxDbaInterval = g_dba;
}

// ---- IMU: calibrated acceleration magnitude (m/s^2) ----
void readIMU() {
  if (!imuOK) return;
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  float cx = a.acceleration.x - OFFSET_X;
  float cy = a.acceleration.y - OFFSET_Y;
  float cz = a.acceleration.z - OFFSET_Z;
  g_accelMag = sqrtf(cx * cx + cy * cy + cz * cz);
  if (g_accelMag > g_maxAccelInterval) g_maxAccelInterval = g_accelMag;
}

// ---- PPG: drain the FIFO (no sample skipped), detect beats, keep BPM averages ----
void resetPPGState(uint32_t now) {
  rateSpot = 0; rateCount = 0; recent3Count = 0; recent3Spot = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  lastBeat = now; lastAcceptedBeat = 0;
  g_beatAvg = 0; g_bpmInst = 0; g_hrFast = NAN; g_hrValid = false;
}

float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

void readPPG(uint32_t now) {
  if (!ppgOK) return;
  particleSensor.check();
  
  while (particleSensor.available()) {
    long rawIr = (long)particleSensor.getFIFOIR();
    particleSensor.nextSample();
    
    g_irRaw = rawIr;
    
    // Apply Kalman filter to raw IR sample
    g_irFiltered = irFilter.update((float)rawIr);

    // Pass the filtered signal to beat detection (cast to long)
    if (checkForBeat((long)g_irFiltered)) {
      long delta = (long)now - lastBeat;
      lastBeat = now;
      float bpm = 60.0f / (delta / 1000.0f);
      g_bpmInst = bpm;
      
      if (bpm < 255 && bpm > 20) {
        rates[rateSpot++] = (byte)bpm;
        rateSpot %= RATE_SIZE;
        if (rateCount < RATE_SIZE) rateCount++;
        
        int sum = 0;
        for (byte x = 0; x < rateCount; x++) sum += rates[x];
        g_beatAvg = sum / rateCount;

        recent3[recent3Spot++] = bpm;
        recent3Spot %= 3;
        if (recent3Count < 3) recent3Count++;
        lastAcceptedBeat = now;
      }
    }
  }

  g_hrValid = (g_irRaw > IR_MIN_CONTACT) && recent3Count >= 3 &&
              lastAcceptedBeat != 0 && (now - lastAcceptedBeat) < 5000;
  g_hrFast = g_hrValid ? median3(recent3[0], recent3[1], recent3[2]) : NAN;
}

// ---- Capacitive touch: RC rise delay (us), true if touched ----
bool readCapSensor() {
  digitalWrite(CAP_DRIVE_PIN, LOW);
  delayMicroseconds(500);                      // discharge
  digitalWrite(CAP_DRIVE_PIN, HIGH);
  uint32_t start = micros();
  while (digitalRead(CAP_SENSE_PIN) == LOW) {
    if (micros() - start > 1000) break;        // timeout
  }
  uint32_t d = micros() - start;
  g_capDelayUs = d;
  return (d < 1000 && d > CAP_THRESHOLD_US);
}

// ============================================================================
//  THINGSPEAK: record building + background upload task
// ============================================================================
void initRecord(UploadRecord& r, uint32_t t) {
  r.t_ms = t;
  for (int i = 0; i < 8; i++) r.field[i] = NAN;
  r.status[0] = '\0';
}

// Builds {"write_api_key":"..","updates":[{...},{...}]}. 
// epochNow = current UTC epoch. nowMs = millis() at that instant.
// Returns the JSON length; *nUsed = how many records fit.
size_t buildBulkJson(char* buf, size_t cap, const char* apiKey, const UploadRecord* r, int n,
                     time_t epochNow, uint32_t nowMs, int* nUsed) {
  size_t len = snprintf(buf, cap, "{\"write_api_key\":\"%s\",\"updates\":[", apiKey);
  int used = 0;
  for (int i = 0; i < n; i++) {
    char item[420];
    int m = 0;
    bool first = true;
    item[m++] = '{';
    if (epochNow > 0) {
      time_t t = epochNow - (time_t)((uint32_t)(nowMs - r[i].t_ms) / 1000);
      struct tm tmv;
      gmtime_r(&t, &tmv);
      m += snprintf(item + m, sizeof(item) - m, "\"created_at\":\"%04d-%02d-%02d %02d:%02d:%02d +0000\"",
                    tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
      first = false;
    }
    for (int f = 0; f < 8; f++) {
      if (isnan(r[i].field[f])) continue;
      m += snprintf(item + m, sizeof(item) - m, "%s\"field%d\":%.2f", first ? "" : ",", f + 1, r[i].field[f]);
      first = false;
    }
    if (r[i].status[0]) {
      char clean[80];
      strncpy(clean, r[i].status, sizeof(clean) - 1);
      clean[sizeof(clean) - 1] = '\0';
      for (char* c = clean; *c; ++c) if (*c == '"' || *c == '\\') *c = '\'';
      m += snprintf(item + m, sizeof(item) - m, "%s\"status\":\"%s\"", first ? "" : ",", clean);
    }
    if (m >= (int)sizeof(item) - 2) m = sizeof(item) - 2;
    item[m++] = '}';
    item[m] = '\0';
    if (len + m + 4 >= cap) break;               // batch buffer full
    len += snprintf(buf + len, cap - len, "%s%s", used ? "," : "", item);
    used++;
  }
  len += snprintf(buf + len, cap - len, "]}");
  if (nUsed) *nUsed = used;
  return len;
}

// Queue a record without blocking the sensor loop. If the queue is full, drop the oldest.
void enqueueRecord(const UploadRecord& rec) {
  if (!uploadQueue) return;
  if (xQueueSend(uploadQueue, &rec, 0) != pdTRUE) {
    UploadRecord junk;
    xQueueReceive(uploadQueue, &junk, 0);
    xQueueSend(uploadQueue, &rec, 0);
  }
}

void connectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  if (USE_ENTERPRISE_WIFI) {
    Serial.println("# Wi-Fi: connecting (WPA2-Enterprise)...");
    EAP_SET_IDENTITY(EAP_IDENTITY);
    EAP_SET_USERNAME(EAP_USERNAME);
    EAP_SET_PASSWORD(EAP_PASSWORD);
    EAP_ENABLE();
    WiFi.begin(ENT_SSID);
  } else {
    Serial.println("# Wi-Fi: connecting (home network)...");
    WiFi.begin(HOME_SSID, HOME_PASSWORD);
  }
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) vTaskDelay(pdMS_TO_TICKS(500));
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("# Wi-Fi connected, IP ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("# Wi-Fi connection failed, will retry");
  }
}

static bool hasEventField(const UploadRecord& r) { return !isnan(r.field[6]); }

void uploadTask(void* arg) {
  static UploadRecord pending[UPLOAD_BATCH_MAX];
  static char body[6144];
  int nPending = 0;
  uint32_t lastUpload = 0;
  bool everUploaded = false;
  uint32_t lastWifiTry = 0;
  bool wifiTried = false;
  bool ntpStarted = false;
  uint32_t ntpStart = 0;

  for (;;) {
    // ---- keep Wi-Fi up ----
    if (WiFi.status() != WL_CONNECTED) {
      ntpStarted = false;
      if (!wifiTried || millis() - lastWifiTry >= 10000) {
        connectWiFi();
        lastWifiTry = millis();
        wifiTried = true;
      }
    } else if (!ntpStarted) {
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // UTC; used for created_at
      ntpStarted = true;
      ntpStart = millis();
    }

    // ---- move queued records into the batch (drop oldest periodic row if the batch is full) ----
    while (true) {
      if (nPending >= UPLOAD_BATCH_MAX) {
        if (uxQueueMessagesWaiting(uploadQueue) == 0) break;
        int drop = 0;
        for (int i = 0; i < nPending; i++) if (!hasEventField(pending[i])) { drop = i; break; }
        memmove(&pending[drop], &pending[drop + 1], (nPending - drop - 1) * sizeof(UploadRecord));
        nPending--;
      }
      if (xQueueReceive(uploadQueue, &pending[nPending], 0) != pdTRUE) break;
      nPending++;
    }

    // ---- upload when due ----
    if (nPending > 0 && WiFi.status() == WL_CONNECTED) {
      bool anyEvent = false;
      for (int i = 0; i < nPending; i++) if (hasEventField(pending[i])) anyEvent = true;
      uint32_t since = millis() - lastUpload;
      bool due = !everUploaded || since >= UPLOAD_INTERVAL_MS || (anyEvent && since >= EVENT_MIN_GAP_MS);
      time_t nowEpoch = time(nullptr);
      bool timeOK = nowEpoch > 1700000000;
      if (!timeOK && millis() - ntpStart < 30000) due = false;   // give NTP a moment so created_at is right

      if (due) {
        int used = 0;
        size_t len = buildBulkJson(body, sizeof(body), TS_WRITE_KEY, pending, nPending,
                                   timeOK ? nowEpoch : 0, millis(), &used);
        char url[120];
        snprintf(url, sizeof(url), "https://api.thingspeak.com/channels/%s/bulk_update.json", TS_CHANNEL_ID);

        WiFiClientSecure client;
        client.setInsecure();                                    // no cert pinning, data is not sensitive
        HTTPClient http;
        http.setTimeout(10000);
        int code = -1;
        if (http.begin(client, url)) {
          http.addHeader("Content-Type", "application/json");
          code = http.POST((uint8_t*)body, len);
        }
        if (code == 200 || code == 202) {
          Serial.printf("# ThingSpeak: uploaded %d record(s), HTTP %d\n", used, code);
          if (used < nPending) {
            memmove(&pending[0], &pending[used], (nPending - used) * sizeof(UploadRecord));
            nPending -= used;
          } else {
            nPending = 0;
          }
        } else {
          Serial.printf("# ThingSpeak: upload FAILED, HTTP %d (%s) -- will retry\n", code,
                        code > 0 ? http.getString().c_str() : HTTPClient::errorToString(code).c_str());
        }
        http.end();
        lastUpload = millis();
        everUploaded = true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ============================================================================
//  MEASUREMENT CONTROL, EVENTS, LED
// ============================================================================
void startMeasurement(uint32_t now) {
  measuring = true;
  if (ppgOK) {
    particleSensor.wakeUp();
    particleSensor.clearFIFO();
  }
  resetPPGState(now);
  detector.begin(now);
  g_maxDbaInterval = 0; g_maxAccelInterval = 0;
  tEcm = tImu = tBmp = tTherm = tDetect = tLog = tPeriodic = now;
  readBMP();
  readThermistor();
  digitalWrite(LED_PIN, HIGH);
  Serial.println("# MEASURING STARTED (warm-up 15 s: learning baselines)");
  Serial.println("ms,state,dBA,V_ecm,dBA_base,accel,IR,IR_kalman,bpm_inst,hr_fast,hr_base,beat_avg,bmp_F,press_hPa,therm_F,wifi");
}

void stopMeasurement() {
  measuring = false;
  if (ppgOK) particleSensor.shutDown();        // saves the 50 mA the LEDs would draw
  Serial.println("# MEASURING STOPPED");
}

void handleTouch(uint32_t now) {
  bool touched = readCapSensor();
  if (touched) { if (capCount < 255) capCount++; }
  else { capCount = 0; capLatched = false; }
  if (capCount >= CAP_DEBOUNCE_READS && !capLatched &&
      (!everToggled || (now - lastToggle) >= TOGGLE_LOCKOUT_MS)) {
    capLatched = true;
    everToggled = true;
    lastToggle = now;
    if (measuring) stopMeasurement(); else startMeasurement(now);
  }
}

void updateLed(uint32_t now) {
  if (measuring) { digitalWrite(LED_PIN, HIGH); return; }
  if (now - tLed >= 500) {                    // blink while idle
    tLed = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

void handleEvent(const ScareEvent& ev) {
  UploadRecord r;
  initRecord(r, ev.tEvent);
  float t_since = (ev.tEvent - ev.tScare) / 1000.0f;

  if (ev.type == EV_SCARE) {
    Serial.printf("# EVENT jumpscare detected: %.1f dBA (+%.1f dB over %.1f baseline)%s\n",
                  ev.peakDba, ev.jumpDb, ev.baseDba, ev.flinched ? " [flinch already seen]" : "");
    r.field[3] = ev.peakDba;
    r.field[4] = ev.hrBase;
    r.field[6] = EV_SCARE;
    r.field[7] = ev.jumpDb;
    snprintf(r.status, sizeof(r.status), "Jumpscare: %.0f dBA, +%.0f dB over baseline", ev.peakDba, ev.jumpDb);
  } else {
    const char* name = (ev.type == EV_FELL) ? "FELL FOR IT" : (ev.type == EV_PARTIAL) ? "partial reaction" : "no reaction";
    Serial.printf("# EVENT verdict (%.1f s after scare): %s | flinch %.1f m/s2 | HR base %.0f peak %.0f (%+.0f bpm)%s\n",
                  t_since, name, ev.flinchPeak, ev.hrBase, ev.hrPeak, ev.hrRise, ev.hrValid ? "" : " [HR unavailable]");
    r.field[3] = ev.peakDba;
    r.field[4] = ev.hrValid ? ev.hrPeak : NAN;
    r.field[5] = ev.flinched ? ev.flinchPeak : NAN;
    r.field[6] = ev.type;
    r.field[7] = ev.hrValid ? ev.hrRise : NAN;
    if (ev.hrValid)
      snprintf(r.status, sizeof(r.status), "%s: flinch %.0f m/s2, HR %+.0f bpm, %.0f dBA",
               name, ev.flinched ? ev.flinchPeak : 0.0f, ev.hrRise, ev.peakDba);
    else
      snprintf(r.status, sizeof(r.status), "%s: flinch %.0f m/s2, HR n/a, %.0f dBA",
               name, ev.flinched ? ev.flinchPeak : 0.0f, ev.peakDba);
  }
  enqueueRecord(r);
}

void runDetector(uint32_t now) {
  detector.update(now, g_dba, g_accelMag, g_hrValid ? g_hrFast : NAN);
  ScareEvent ev;
  while (detector.popEvent(ev)) handleEvent(ev);
}

void queuePeriodicRecord(uint32_t now) {
  UploadRecord r;
  initRecord(r, now);
  r.field[0] = g_bmpTempF;
  r.field[1] = g_pressureHpa;
  r.field[2] = g_thermTempF;
  r.field[3] = g_maxDbaInterval > 0 ? g_maxDbaInterval : g_dba;
  r.field[4] = g_beatAvg > 0 ? (float)g_beatAvg : NAN;
  r.field[5] = g_maxAccelInterval;
  enqueueRecord(r);
  g_maxDbaInterval = 0;
  g_maxAccelInterval = 0;
}

void logCsv(uint32_t now) {
  Serial.printf("%lu,%u,%.1f,%.3f,%.1f,%.1f,%ld,%.0f,%.0f,%.0f,%.0f,%d,%.1f,%.1f,%.1f,%d\n",
                (unsigned long)now, detector.warmedUp(now) ? detector.phaseId() : 9,
                g_dba, g_ecmV, detector.baselineDba(), g_accelMag, 
                g_irRaw, g_irFiltered,
                g_bpmInst,
                isnan(g_hrFast) ? 0.0f : g_hrFast,
                isnan(detector.baselineHr()) ? 0.0f : detector.baselineHr(),
                g_beatAvg, g_bmpTempF, g_pressureHpa, g_thermTempF,
                WiFi.status() == WL_CONNECTED ? 1 : 0);
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  pinMode(MUX_PIN, OUTPUT);
  digitalWrite(MUX_PIN, LOW);      // default to ECM
  pinMode(CAP_DRIVE_PIN, OUTPUT);
  digitalWrite(CAP_DRIVE_PIN, LOW);
  pinMode(CAP_SENSE_PIN, INPUT);

  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  Wire.begin(I2C_SDA, I2C_SCL);

  // ---- BMP280 ----
  bmpOK = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpOK) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
  Serial.println(bmpOK ? "# BMP280 OK" : "# BMP280 NOT FOUND (continuing without it)");

  // ---- MAX30102 ----
  ppgOK = particleSensor.begin(Wire, I2C_SPEED_FAST);
  if (ppgOK) {
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.setPulseAmplitudeRed(0xFF);
    particleSensor.setPulseAmplitudeIR(0xFF);
    particleSensor.shutDown();                 // off until a touch starts a session
  }
  Serial.println(ppgOK ? "# MAX30102 OK" : "# MAX30102 NOT FOUND (continuing without it)");

  // ---- LSM9DS1 ----
  imuOK = lsm.begin();
  if (imuOK) {
    // 16 g; 119 Hz output rate so no flinches are missed
    lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_16G, Adafruit_LSM9DS1::LSM9DS1_ACCELDATARATE_119HZ);
  }
  Serial.println(imuOK ? "# LSM9DS1 OK" : "# LSM9DS1 NOT FOUND (continuing without it)");

  // ---- background uploader on core 0 ----
  uploadQueue = xQueueCreate(UPLOAD_QUEUE_LEN, sizeof(UploadRecord));
  xTaskCreatePinnedToCore(uploadTask, "upload", 12288, NULL, 1, NULL, 0);

  Serial.println("# Ready. Touch the cap sensor to start/stop measuring.");
}

void loop() {
  uint32_t now = millis();

  if (now - tCap >= CAP_PERIOD_MS) { tCap = now; handleTouch(now); }
  updateLed(now);

  if (!measuring) { delay(2); return; }

  readPPG(now);                                                   // every pass: drains the FIFO
  if (now - tEcm    >= ECM_PERIOD_MS)    { tEcm = now;    readECM(); }
  if (now - tImu    >= IMU_PERIOD_MS)    { tImu = now;    readIMU(); }
  if (now - tBmp    >= BMP_PERIOD_MS)    { tBmp = now;    readBMP(); }
  if (now - tTherm  >= THERM_PERIOD_MS)  { tTherm = now;  readThermistor(); }
  if (now - tDetect >= DETECT_PERIOD_MS) { tDetect = now; runDetector(now); }
  if (now - tLog    >= LOG_PERIOD_MS)    { tLog = now;    logCsv(now); }
  if (now - tPeriodic >= UPLOAD_INTERVAL_MS) { tPeriodic = now; queuePeriodicRecord(now); }

  delay(1);
}
