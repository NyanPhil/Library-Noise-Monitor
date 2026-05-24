// =============================================================================
//  Library Noise Detector + QR Scanner  —  V3
//  Hardware : ESP32-S3 N16R8 + INMP441 I2S Microphone + OV5640 (S3-EYE)
//  Features : Dual-core FreeRTOS, DC offset filter, Watchdog-safe yielding,
//             3-level noise threshold, 3 LEDs + motor output,
//             Alert cooldown, optional Auto-calibration,
//             QR scanning with optional MJPEG stream,
//             Firebase Realtime Database push (audio + QR)
// =============================================================================

#include <driver/i2s.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ESP32QRCodeReader.h>
#include <climits>
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_task_wdt.h>

// =============================================================================
//  FEATURE TOGGLES
// =============================================================================
#define ENABLE_STREAM  true    // true  = stream + QR + Firebase over WiFi
                               // false = QR only, no WiFi, no Firebase

#define AUTO_CALIBRATE 1       // 1 = auto noise-floor calibration on boot
                               // 0 = use manual thresholds below

// =============================================================================
//  WiFi CREDENTIALS
// =============================================================================
const char *ssid     = "BANGUIS WIFI";
const char *password = "PLDTWIFI_banguis1";

// =============================================================================
//  FIREBASE CONFIG
// =============================================================================
#define FIREBASE_HOST  "micropit-91298-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH  "ataEj3J2JEJGSvQv69DpfvpASrKbGvTDrOUkb3fp"
#define DEVICE_ID      "unit_A"   // change to "unit_B" on the second device

#define AUDIO_PUSH_INTERVAL_MS   5000
#define QR_COOLDOWN_MS          10000
#define QR_MAX_ENTRIES              6

FirebaseData   fbdo;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

// FIX #2: guard flag — prevents Firebase calls before init is complete
volatile bool g_firebase_ready = false;

// =============================================================================
//  NOISE DETECTOR PINS
// =============================================================================
#define I2S_WS    41
#define I2S_SD     2
#define I2S_SCK   42
#define I2S_PORT  I2S_NUM_0

#define LED_QUIET_PIN    3
#define LED_NOISY_PIN   46
#define LED_LOUD_PIN    21
#define MOTOR_PIN       47

#define WARNING_COUNT_MAX  3 // max warning count for loud noise level

//INTERRUPT #2
#define WDT_TIMEOUT_SEC  10   // reboot if logicTask freezes for 10 seconds

// =============================================================================
//  QR SCANNER PINS
// =============================================================================
#define SCANNING_QR_PIN  40
#define QR_SCANNED_PIN   39

// =============================================================================
// INTERRUPT #1 - CALIBRATION PIN
#define RECAL_BUTTON_PIN  14
volatile bool g_recalibrate_requested = false;

// =============================================================================
//  AUDIO CONFIG
// =============================================================================
#define SAMPLE_RATE  16000
#define BLOCK_SIZE   512

// =============================================================================
//  MANUAL THRESHOLDS  (used when AUTO_CALIBRATE is 0)
// =============================================================================
#define MANUAL_QUIET_CEILING    50.0f
#define MANUAL_NOISY_THRESHOLD  50.0f
#define MANUAL_LOUD_THRESHOLD   65.0f

#define CALIBRATION_NOISY_OFFSET   8.0f
#define CALIBRATION_LOUD_OFFSET   15.0f

// =============================================================================
//  AUDIO / DETECTION TUNING
// =============================================================================
#define DB_CALIBRATION_OFFSET  120.0f
#define ALPHA                    0.1f
#define SUSTAINED_MS            3000
#define ALERT_COOLDOWN_MS      30000

#define BLINK_NOISY_MS   600
#define BLINK_LOUD_MS    150

// =============================================================================
//  NOISE LEVEL ENUM
// =============================================================================
typedef enum {
  LEVEL_QUIET,
  LEVEL_NOISY,
  LEVEL_LOUD
} NoiseLevel;

// =============================================================================
//  GLOBALS
// =============================================================================
QueueHandle_t dbQueue;

volatile float g_noisy_threshold = MANUAL_NOISY_THRESHOLD;
volatile float g_loud_threshold  = MANUAL_LOUD_THRESHOLD;

// Post-deletion cooldown — prevents re-saving a just-deleted QR code
volatile unsigned long long g_last_deletion_ms = 0;
#define QR_POST_DELETE_COOLDOWN_MS  5000   // 5 seconds, adjust to taste

volatile int g_warning_count = 0;   // local mirror of DB value


ESP32QRCodeReader reader(CAMERA_MODEL_ESP32S3_EYE);

// MJPEG stream constants
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char INDEX_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>QR Scanner</title>
  <style>
    body{margin:0;background:#111;display:flex;flex-direction:column;
         align-items:center;padding:16px;font-family:monospace;color:#0f0;}
    h2{margin:0 0 12px;letter-spacing:.2em;}
    img{max-width:100%;border:1px solid #0f04;border-radius:4px;}
    p{margin-top:10px;font-size:.8rem;color:#555;}
  </style>
</head>
<body>
  <h2>QR SCANNER STREAM</h2>
  <img src="/stream" alt="stream">
  <p>QR results in Serial Monitor</p>
</body></html>
)html";

// =============================================================================
//  DC OFFSET FILTER
// =============================================================================
static inline float dc_filter(float sample) {
  static float x_prev = 0.0f, y_prev = 0.0f;
  const  float R      = 0.9921f;
  float y   = sample - x_prev + R * y_prev;
  x_prev    = sample;
  y_prev    = y;
  return y;
}

// =============================================================================
//  LED HELPER  (non-blocking)
// =============================================================================
void updateLEDs(NoiseLevel level) {
  static unsigned long last_blink_ms = 0;
  static bool          blink_state   = false;
  unsigned long        now           = millis();

  switch (level) {
    case LEVEL_QUIET:
      digitalWrite(LED_QUIET_PIN, HIGH);
      digitalWrite(LED_NOISY_PIN, LOW);
      digitalWrite(LED_LOUD_PIN,  LOW);
      digitalWrite(MOTOR_PIN,     LOW);
      blink_state = false;
      break;

    case LEVEL_NOISY:
      digitalWrite(LED_QUIET_PIN, LOW);
      digitalWrite(LED_LOUD_PIN,  LOW);
      digitalWrite(MOTOR_PIN,     LOW);
      if (now - last_blink_ms >= BLINK_NOISY_MS) {
        blink_state   = !blink_state;
        last_blink_ms = now;
        digitalWrite(LED_NOISY_PIN, blink_state ? HIGH : LOW);
      }
      break;

    case LEVEL_LOUD:
      digitalWrite(LED_QUIET_PIN, LOW);
      digitalWrite(LED_NOISY_PIN, LOW);
      digitalWrite(MOTOR_PIN,     HIGH);
      if (now - last_blink_ms >= BLINK_LOUD_MS) {
        blink_state   = !blink_state;
        last_blink_ms = now;
        digitalWrite(LED_LOUD_PIN, blink_state ? HIGH : LOW);
      }
      break;
  }
}

// =============================================================================
//  FIREBASE HELPERS
// =============================================================================

unsigned long long getEpochMs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (unsigned long long)(tv.tv_sec)  * 1000ULL
       + (unsigned long long)(tv.tv_usec) / 1000ULL;
}
// Fetches warning_count from DB on boot to sync with whatever the dashboard set
void fetchWarningCount() {
  if (!g_firebase_ready) return;
  String path = "/devices/" DEVICE_ID "/warning_count";
  if (Firebase.getInt(fbdo, path)) {
    g_warning_count = fbdo.intData();
    Serial.printf("[Firebase] warning_count synced from DB: %d\n", g_warning_count);
  } else {
    // Node doesn't exist yet — write 0 to initialize it
    Firebase.setInt(fbdo, path, 0);
    g_warning_count = 0;
    Serial.println("[Firebase] warning_count initialized to 0 in DB");
  }
}

// Increments warning_count in DB and local mirror, capped at WARNING_COUNT_MAX
void incrementWarningCount() {
  if (!g_firebase_ready) return;
  if (g_warning_count >= WARNING_COUNT_MAX) {
    Serial.printf("[Warning] Count already at max (%d), ignoring.\n",
                  WARNING_COUNT_MAX);
    return;
  }
  g_warning_count++;
  String path = "/devices/" DEVICE_ID "/warning_count";
  if (!Firebase.setInt(fbdo, path, g_warning_count))
    Serial.printf("[Firebase] warning_count write failed: %s\n",
                  fbdo.errorReason().c_str());
  else
    Serial.printf("[Warning] Count incremented to %d / %d\n",
                  g_warning_count, WARNING_COUNT_MAX);
}
// Called from logicTask every AUDIO_PUSH_INTERVAL_MS
void pushAudioData(float db, float noisy_th, float loud_th, int level) {
  // FIX #2: bail out if Firebase hasn't been initialized yet
  if (!g_firebase_ready) return;

  String base = "/devices/" DEVICE_ID "/audio";
  FirebaseJson json;
  json.set("received_db",     db);
  json.set("noisy_threshold", noisy_th);
  json.set("loud_threshold",  loud_th);
  json.set("level",           level);
  json.set("updated_at",      (int64_t)getEpochMs());

  if (!Firebase.setJSON(fbdo, base, json))
    Serial.printf("[Firebase] Audio write failed: %s\n",
                  fbdo.errorReason().c_str());
}

// Called from onQrCodeTask on each valid scan
void pushQRCode(const char *payload) {
  if (!g_firebase_ready) return;

  String             base          = "/devices/" DEVICE_ID "/qr_codes";
  unsigned long long now           = getEpochMs();
  bool               was_deleted   = false;   // tracks if we just deleted this payload

  // --- Guard: if a deletion just happened, ignore scans during post-delete cooldown
  if ((now - g_last_deletion_ms) < QR_POST_DELETE_COOLDOWN_MS) {
    Serial.printf("[QR] Post-deletion cooldown active, skipping scan.\n");
    return;
  }

  // Step 1: read existing entries
  if (!Firebase.getJSON(fbdo, base)) {
    if (fbdo.errorReason() != "path not exist") {
      Serial.printf("[Firebase] QR read failed: %s\n",
                    fbdo.errorReason().c_str());
      return;
    }
  }

  String             raw           = fbdo.jsonString();
  StaticJsonDocument<4096> doc;
  DeserializationError err         = deserializeJson(doc, raw);

  String             duplicate_key = "";
  String             oldest_key    = "";
  unsigned long long oldest_ts     = ULLONG_MAX;
  int                entry_count   = 0;

  if (!err && doc.is<JsonObject>()) {
    for (JsonPair kv : doc.as<JsonObject>()) {
      entry_count++;
      const char        *stored_payload = kv.value()["payload"];
      unsigned long long stored_ts =
          (unsigned long long)kv.value()["scanned_at"].as<long long>();

      if (stored_ts < oldest_ts) {
        oldest_ts  = stored_ts;
        oldest_key = kv.key().c_str();
      }

      if (strcmp(stored_payload, payload) == 0) {
        unsigned long long age = now - stored_ts;
        if (age < QR_COOLDOWN_MS) {
          // Within cooldown — ignore entirely
          Serial.printf("[QR] Duplicate within cooldown (%.1f s), skipping.\n",
                        age / 1000.0);
          return;
        } else {
          // Past cooldown — mark for deletion
          duplicate_key = kv.key().c_str();
        }
      }
    }
  }

  // Step 2: delete expired duplicate, then stop — do NOT re-save immediately
  if (duplicate_key.length() > 0) {
    Firebase.deleteNode(fbdo, base + "/" + duplicate_key);
    entry_count--;
    was_deleted          = true;
    g_last_deletion_ms   = now;   // start post-deletion cooldown
    Serial.printf("[QR] Deleted expired entry for payload: %s\n"
                  "     Cooldown active — rescan to save again.\n", payload);
  }

  // Early return — deletion and re-save are now two separate scan events
  if (was_deleted) return;

  // Step 3: evict oldest if at capacity
  if (entry_count >= QR_MAX_ENTRIES && oldest_key.length() > 0) {
    Firebase.deleteNode(fbdo, base + "/" + oldest_key);
    Serial.printf("[QR] Evicted oldest entry: %s\n", oldest_key.c_str());
  }

  // Step 4: write new entry
  FirebaseJson entry;
  entry.set("payload",    payload);
  entry.set("scanned_at", (int64_t)now);

  if (!Firebase.pushJSON(fbdo, base, entry))
    Serial.printf("[Firebase] QR write failed: %s\n",
                  fbdo.errorReason().c_str());
  else
    Serial.printf("[QR] Saved: %s\n", payload);
}

// =============================================================================
//  AUDIO TASK -- Core 1, priority 3
// =============================================================================
void audioTask(void *pvParameters) {
  int32_t raw_samples[BLOCK_SIZE];
  size_t  bytes_read;
  float   smoothed_db = 0.0f;

  for (;;) {
    i2s_read(I2S_PORT, raw_samples, sizeof(raw_samples),
             &bytes_read, portMAX_DELAY);

    int   samples_count = bytes_read / sizeof(int32_t);
    float sum_sq        = 0.0f;

    for (int i = 0; i < samples_count; i++) {
      float sample  = (float)(raw_samples[i] >> 8);
      sample       /= 8388608.0f;
      sample        = dc_filter(sample);
      sum_sq       += sample * sample;
    }

    float rms    = sqrtf(sum_sq / samples_count);
    float db     = 20.0f * log10f(rms + 1e-9f);
    float db_spl = db + DB_CALIBRATION_OFFSET;

    smoothed_db = ALPHA * db_spl + (1.0f - ALPHA) * smoothed_db;

    xQueueSend(dbQueue, &smoothed_db, 0);
    vTaskDelay(1);
  }
}

// =============================================================================
//  LOGIC TASK -- Core 0, priority 1
//  FIX #1: stack increased from 4096 to 12288 to accommodate FirebaseJson
//          and the Firebase SSL/HTTP overhead inside pushAudioData().
// =============================================================================
void logicTask(void *pvParameters) {
  float         received_db        = 0.0f;
  NoiseLevel    current_level      = LEVEL_QUIET;
  unsigned long loud_since         = 0;
  unsigned long last_alert_ms      = 0;
  unsigned long last_audio_push_ms = 0;
  bool          alert_sent         = false;
  bool          warning_counted    = false; 


  esp_task_wdt_add(NULL);
  for (;;) {
      // Interrupt #2 — prove logicTask is alive; resets the 10-second watchdog
      esp_task_wdt_reset();
      // Poll DB for warning_count reset from dashboard (every 5 seconds)
      static unsigned long last_wcount_check_ms = 0;
      unsigned long now_wc = millis();
      if (g_firebase_ready && (now_wc - last_wcount_check_ms) >= 5000) {
        last_wcount_check_ms = now_wc;
        String path = "/devices/" DEVICE_ID "/warning_count";
        if (Firebase.getInt(fbdo, path)) {
          int db_val = fbdo.intData();
          if (db_val != g_warning_count) {
            g_warning_count = db_val;
            Serial.printf("[Warning] Count synced from DB: %d\n", g_warning_count);
          }
        }
      }

      // Interrupt #1 — handle recalibration request from button ISR
      if (g_recalibrate_requested) {
        g_recalibrate_requested = false;
        Serial.println("[RECAL] Button pressed — recalibrating...");
        #if AUTO_CALIBRATE
          runCalibration();
        #else
          Serial.println("[RECAL] AUTO_CALIBRATE is off, nothing to do.");
        #endif
      }
    if (xQueueReceive(dbQueue, &received_db, portMAX_DELAY)) {
  
      // Classify
      NoiseLevel new_level;
      if      (received_db >= g_loud_threshold)  new_level = LEVEL_LOUD;
      else if (received_db >= g_noisy_threshold) new_level = LEVEL_NOISY;
      else                                        new_level = LEVEL_QUIET;

      // Transition
      if (new_level != current_level) {
        if (current_level == LEVEL_LOUD && new_level != LEVEL_LOUD) {
          loud_since  = 0;
          alert_sent  = false;
          warning_counted  = false; 
        }
        if (new_level == LEVEL_LOUD && current_level != LEVEL_LOUD) {
          // Record when we first entered loud — don't act yet
          loud_since = millis();
        }
        current_level = new_level;
      }

      // Determine what to actually SHOW on LEDs
      // LEVEL_LOUD is only displayed after SUSTAINED_MS has elapsed.
      // Before that, treat it visually as LEVEL_NOISY so the red LED
      // and motor don't fire on brief transient spikes.
      NoiseLevel display_level = current_level;
      if (current_level == LEVEL_LOUD) {
        unsigned long now = millis();
        if ((now - loud_since) < SUSTAINED_MS) {
          display_level = LEVEL_NOISY;   // not sustained yet — show noisy instead
        } else {
          // Sustained threshold crossed — activate LED/motor and count together
          if (!warning_counted) {
            incrementWarningCount();     // <-- fires once, same moment LED turns on
            warning_counted = true;
          }
          if (!alert_sent && (now - last_alert_ms) >= ALERT_COOLDOWN_MS) {
            Serial.println("ALERT: Sustained loud talking detected!");
            last_alert_ms = now;
            alert_sent    = true;
          }
        }
      }

      updateLEDs(display_level);   // LEDs and motor follow display_level, not current_level

      // Firebase audio push every 3 seconds
      unsigned long now_push = millis();
      if (now_push - last_audio_push_ms >= AUDIO_PUSH_INTERVAL_MS) {
        pushAudioData(received_db, g_noisy_threshold,
                      g_loud_threshold, (int)current_level);
        last_audio_push_ms = now_push;
      }

      // Serial Plotter (uncomment if needed)
      // Serial.print("dB_SPL:");       Serial.print(received_db);
      // Serial.print(",NoisyThresh:"); Serial.print(g_noisy_threshold);
      // Serial.print(",LoudThresh:");  Serial.print(g_loud_threshold);
      // Serial.print(",Level:");       Serial.println((int)current_level);
    }
    vTaskDelay(1);
  }
}

// =============================================================================
//  QR RESULT TASK -- Core 0, priority 4
//  FIX #1: stack increased from 4096 to 16384 to accommodate
//          StaticJsonDocument<4096> + multiple String objects + Firebase calls.
// =============================================================================
void onQrCodeTask(void *pvParameters) {
  struct QRCodeData qrCodeData;
  while (true) {
    if (reader.receiveQrCode(&qrCodeData, 100)) {
      Serial.println("\n==============================");
      if (qrCodeData.valid) {
        Serial.print("QR Payload: ");
        Serial.println((const char *)qrCodeData.payload);
        pushQRCode((const char *)qrCodeData.payload);
        digitalWrite(SCANNING_QR_PIN, LOW);
        digitalWrite(QR_SCANNED_PIN,  HIGH);
        delay(3000);
        digitalWrite(QR_SCANNED_PIN,  LOW);
        digitalWrite(SCANNING_QR_PIN, HIGH);
      } else {
        Serial.print("QR Invalid: ");
        Serial.println((const char *)qrCodeData.payload);
        digitalWrite(SCANNING_QR_PIN, HIGH);
      }
      Serial.println("==============================\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// =============================================================================
//  MJPEG STREAM HANDLERS
// =============================================================================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb      = nullptr;
  esp_err_t    res     = ESP_OK;
  char         part_buf[128];
  uint8_t     *jpg_buf = nullptr;
  size_t       jpg_len = 0;

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    if (fb->format != PIXFORMAT_JPEG) {
      bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
      esp_camera_fb_return(fb); fb = nullptr;
      if (!ok) { res = ESP_FAIL; break; }
    } else {
      jpg_buf = fb->buf;
      jpg_len = fb->len;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

    if (fb)           { esp_camera_fb_return(fb); fb = nullptr; jpg_buf = nullptr; }
    else if (jpg_buf) { free(jpg_buf); jpg_buf = nullptr; }
    if (res != ESP_OK) break;
  }
  return res;
}

static void startStreamServer() {
  httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = 80;
  cfg.max_uri_handlers = 4;
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &cfg) != ESP_OK) return;
  httpd_uri_t idx = { "/",       HTTP_GET, index_handler,  nullptr };
  httpd_uri_t stm = { "/stream", HTTP_GET, stream_handler, nullptr };
  httpd_register_uri_handler(server, &idx);
  httpd_register_uri_handler(server, &stm);
}

// =============================================================================
//  I2S INIT
// =============================================================================
void initI2S() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 12,
    .dma_buf_len          = BLOCK_SIZE,
    .use_apll             = false
  };
  i2s_pin_config_t pin_config = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

// =============================================================================
//RECALIBRATE BLOCK
// ISR — keep it minimal, just set a flag
void IRAM_ATTR onRecalButton() {
  g_recalibrate_requested = true;
}

// =============================================================================
//  AUTO-CALIBRATION BLOCK
//  -------------------------------------------------------------------------
//  TO REMOVE: delete from here to the END marker, set AUTO_CALIBRATE 0 above.
//  -------------------------------------------------------------------------
#if AUTO_CALIBRATE

void runCalibration() {
  Serial.println("=== AUTO-CALIBRATION ===");
  Serial.println("Stay quiet for ~3 seconds...");

  int32_t raw_samples[BLOCK_SIZE];
  size_t  bytes_read;
  float   sum       = 0.0f;
  float   dc_x_prev = 0.0f, dc_y_prev = 0.0f;
  const   int rounds = 20;

  for (int r = 0; r < rounds; r++) {
    i2s_read(I2S_PORT, raw_samples, sizeof(raw_samples),
             &bytes_read, portMAX_DELAY);
    int   count  = bytes_read / sizeof(int32_t);
    float sum_sq = 0.0f;
    for (int i = 0; i < count; i++) {
      float sample = (float)(raw_samples[i] >> 8) / 8388608.0f;
      float y      = sample - dc_x_prev + 0.9921f * dc_y_prev;
      dc_x_prev    = sample; dc_y_prev = y; sample = y;
      sum_sq      += sample * sample;
    }
    float db_spl = 20.0f * log10f(sqrtf(sum_sq / count) + 1e-9f)
                   + DB_CALIBRATION_OFFSET;
    sum += db_spl;
    Serial.printf("  Sample %2d / %d : %.1f dB\n", r + 1, rounds, db_spl);
    delay(150);
  }

  float noise_floor = sum / rounds;
  g_noisy_threshold = noise_floor + CALIBRATION_NOISY_OFFSET;
  g_loud_threshold  = noise_floor + CALIBRATION_LOUD_OFFSET;

  Serial.printf("Noise floor    : %.1f dB SPL\n",          noise_floor);
  Serial.printf("Noisy threshold: %.1f dB SPL  (+%.0f)\n", g_noisy_threshold,
                CALIBRATION_NOISY_OFFSET);
  Serial.printf("Loud threshold : %.1f dB SPL  (+%.0f)\n", g_loud_threshold,
                CALIBRATION_LOUD_OFFSET);
  Serial.println("=== CALIBRATION DONE ===");
}

#endif
//  -------------------------------------------------------------------------
//  END OF CALIBRATION BLOCK
//  -------------------------------------------------------------------------

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nLibrary Noise Detector + QR Scanner  V3");

  if (!psramFound()) {
    Serial.println("[ERROR] PSRAM not found. Set Tools > PSRAM > OPI PSRAM");
    while (true) delay(1000);
  }
  Serial.printf("PSRAM: %.1f MB\n", ESP.getPsramSize() / 1048576.0);

  // Noise detector LEDs & motor
  pinMode(LED_QUIET_PIN, OUTPUT);
  pinMode(LED_NOISY_PIN, OUTPUT);
  pinMode(LED_LOUD_PIN,  OUTPUT);
  pinMode(MOTOR_PIN,     OUTPUT);
  digitalWrite(LED_QUIET_PIN, HIGH);
  digitalWrite(LED_NOISY_PIN, LOW);
  digitalWrite(LED_LOUD_PIN,  LOW);
  digitalWrite(MOTOR_PIN,     LOW);

  // QR indicator LEDs
  pinMode(SCANNING_QR_PIN, OUTPUT);
  pinMode(QR_SCANNED_PIN,  OUTPUT);
  digitalWrite(SCANNING_QR_PIN, HIGH);
  digitalWrite(QR_SCANNED_PIN,  LOW);

  // Recallibrate button
  pinMode(RECAL_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(RECAL_BUTTON_PIN, onRecalButton, FALLING);
  // I2S
  initI2S();

  // Noise calibration (runs before tasks start, no contention)
#if AUTO_CALIBRATE
  runCalibration();
#else
  Serial.printf("Manual thresholds -- noisy: %.1f dB  loud: %.1f dB\n",
                g_noisy_threshold, g_loud_threshold);
#endif

  // Camera
  Serial.println("Initialising camera via QR library...");
  reader.setup();
  Serial.println("Camera init done");

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_contrast(s, 2);
    s->set_sharpness(s, 2);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    Serial.println("Sensor tuning applied");
  }

  // FreeRTOS queue
  dbQueue = xQueueCreate(10, sizeof(float));

  // FIX #1: task stacks enlarged
  // Audio: Core 1 priority 3
  xTaskCreatePinnedToCore(audioTask,    "AudioTask",   4096,  NULL, 3, NULL, 1);
  // Camera/QR decode: Core 1 priority 2 (library-internal task)
  reader.beginOnCore(1);
  Serial.println("QR reader started on Core 1");
  // QR result handler: Core 0 priority 4 — FIX #1: 16384 bytes
  xTaskCreatePinnedToCore(onQrCodeTask, "QRResult",   16384,  NULL, 4, NULL, 0);
  // Logic/LED: Core 0 priority 1 — FIX #1: 12288 bytes
  xTaskCreatePinnedToCore(logicTask,    "LogicTask",  12288,  NULL, 1, NULL, 0);

  // Interrupt #2 — Watchdog via new struct API (ESP32 Arduino core v3.x)
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = 10000,   // 10 seconds
    .idle_core_mask = 0,       // don't watch idle tasks
    .trigger_panic  = true     // reboot on timeout instead of just printing
  };
  esp_task_wdt_reconfigure(&wdt_config);   // reconfigures the already-running TWDT

  // FIX #3: WiFi, NTP, and Firebase are all inside ENABLE_STREAM so they
  // only run (and only need each other) as a single coherent block.
  if (ENABLE_STREAM) {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi connected");

    startStreamServer();
    Serial.printf("Stream  -> http://%s\n", WiFi.localIP().toString().c_str());

    // NTP — required for epoch timestamps in Firebase records
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Syncing NTP");
    struct tm ti;
    while (!getLocalTime(&ti)) { delay(500); Serial.print("."); }
    Serial.println(" done");

    // Firebase
    fbConfig.host = FIREBASE_HOST;
    fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
    Firebase.begin(&fbConfig, &fbAuth);
    Firebase.reconnectWiFi(true);
    // FIX: give the SSL layer ~2 seconds to stabilize before first call
    Serial.print("Waiting for Firebase SSL...");
    delay(2000);
    Serial.println(" done");

    // FIX #2: only now is it safe for tasks to call pushAudioData/pushQRCode
    g_firebase_ready = true;
    Serial.println("Firebase ready");
    fetchWarningCount();
  } else {
    Serial.println("Stream + Firebase disabled. QR-only mode.");
  }

  Serial.println("All systems running.");
}

// =============================================================================
//  LOOP -- FreeRTOS owns execution
// =============================================================================
void loop() {
  delay(10000);
}
