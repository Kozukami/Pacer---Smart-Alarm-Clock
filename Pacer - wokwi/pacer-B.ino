// ============================================================
// PACER - Firmware (Person B)          ESP32 DevKit C v4 | Wokwi
// Serial Monitor: 115200 baud
//
// Fitur : virtual clock, state machine 12 state, alarm + nudge,
//         Delay (dijepit), Break, Reallocate, skor S-F, OLED, LED,
//         buzzer non-blocking, logEvent() satu pintu, loader jadwal
//         (JSON lewat Serial; MQTT nanti memanggil loadSchedule()).
//
// Perintah di Serial Monitor (ketik lalu Enter):
//   {"day":"Mon","tasks":[...]}  -> muat jadwal (format bagian 5.8)
//   default                      -> muat jadwal demo bawaan
//   status                       -> cetak keadaan perangkat
//   help                         -> daftar perintah
//
// Baris Serial yang diawali "#" = log debug. Baris yang diawali "{"
// = event JSON (sama persis dengan yang nanti dikirim ke MQTT).
//
// Library: Adafruit SSD1306, Adafruit GFX Library, ArduinoJson (v7).
// ============================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <string.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ---------------- KONSTANTA (ubah di sini saja) ----------------
#ifndef SPEED
#define SPEED 1.0            // menit simulasi per detik nyata (naikkan untuk video)
#endif
#ifndef NUDGE_SEC
#define NUDGE_SEC 10         // detik nyata antar re-buzz (10 untuk video)
#endif
#ifndef LOAD_DEFAULT_ON_BOOT
#define LOAD_DEFAULT_ON_BOOT 0   // 1 / 0 = 1 load / 0 jangan load jadwal demo saat boot (fallback/pengembangan)
#endif
#ifndef DEBUG_TICK
#define DEBUG_TICK 0         // 1 = cetak jam simulasi tiap menit
#endif

#define MAX_TASKS        6
#define NAME_MAX_LEN     20
#define IFTHEN_MAX_LEN   60
#define PRE_ALERT_MIN    5       // menit sebelum mulai
#define START_LEAD_MIN   10      // jam virtual mulai 10 menit sebelum tugas pertama
#define MIN_GAP_MIN      10      // jarak minimum antar tugas
#define MAX_DELAYS       2
#define DELAY_STEP_MIN   5
#define MAX_DELAY_LAST   30      // batas delay untuk tugas terakhir (tidak ada tugas berikutnya)
#define BREAK_MIN_TASK   30      // tugas >= 30 menit ditawari break
#define BREAK_MAX_MIN    30
#define MAX_REBUZZ       3
#define RESULT_HOLD_MS   2500    // lama layar DONE/MISSED/REALLOCATED (detik nyata)
#define BREAK_PROMPT_SEC 15      // prompt break tanpa jawaban = Skip
#define SELECT_TIMEOUT_SEC 20    // pemilih menit tanpa tombol = batal
#define OLED_PERIOD_MS   500
#define DEBOUNCE_MS      30
#define SERIAL_BUF       1536

// ---------------- KONTRAK PIN (bagian 5.1 revisi) ----------------
const uint8_t PIN_BTN[5]     = {25, 26, 27, 32, 33};  // B1..B5, INPUT_PULLUP
const uint8_t PIN_BUZZER     = 18;
const uint8_t PIN_LED_GREEN  = 4;
const uint8_t PIN_LED_YELLOW = 5;
const uint8_t PIN_LED_RED    = 19;
const uint8_t PIN_SDA        = 21;
const uint8_t PIN_SCL        = 22;
const uint8_t OLED_ADDR      = 0x3C;

// ---------------- TIPE DATA ----------------
enum State : uint8_t {
  ST_IDLE, ST_PRE_ALERT, ST_WAIT_CONFIRM, ST_DELAY_SELECT, ST_RUNNING,
  ST_BREAK_PROMPT, ST_BREAK_SELECT, ST_BREAK, ST_DONE, ST_MISSED,
  ST_REALLOCATED, ST_DAY_END
};
const char* const STATE_NAME[] = {
  "IDLE", "PRE_ALERT", "WAIT_CONFIRM", "DELAY_SELECT", "RUNNING",
  "BREAK_PROMPT", "BREAK_SELECT", "BREAK", "DONE", "MISSED",
  "REALLOCATED", "DAY_END"
};

enum Outcome : uint8_t { OUT_PENDING, OUT_DONE, OUT_REALLOCATED, OUT_MISSED };

struct Task {
  char     name[NAME_MAX_LEN + 1];
  uint8_t  tier;                    // 1..3
  int16_t  start;                   // menit sejak 00:00
  int16_t  end;
  char     ifThen[IFTHEN_MAX_LEN + 1];
  uint8_t  delaysUsed;
  Outcome  outcome;
  uint8_t  outcome10;               // faktor hasil x10 (10 = 1.0), supaya hitungan tanpa float
};

struct Note { uint16_t freq; uint16_t ms; };   // freq 0 = jeda

// ---------------- DATA GLOBAL ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false;

Task    tasks[MAX_TASKS];
uint8_t nTasks = 0;
uint8_t cur = 0;                    // indeks tugas yang sedang diproses
char    dayName[8] = "Mon";
State   state = ST_IDLE;
uint32_t stateEnterMs = 0;

// jam virtual
double   baseMin = 0;
uint32_t t0 = 0;
bool     clockPaused = false;
uint32_t pausedAt = 0;
int      lastTickMin = -1;

// per-tugas
bool     delayedWait = false;       // PRE_ALERT setelah Delay: tunggu tanpa bunyi pre-alert
bool     breakOffered = false;
int      breakEndMin = 0;
int      selValue = 5, selMax = 5;  // pemilih menit (delay / break)
uint32_t lastPressMs = 0;           // untuk timeout pemilih / prompt
uint8_t  nudgeCount = 0;
uint32_t lastBuzzMs = 0;

// input
bool     pressed[5];
bool     lastRaw[5]    = {HIGH, HIGH, HIGH, HIGH, HIGH};
bool     stableVal[5]  = {HIGH, HIGH, HIGH, HIGH, HIGH};
uint32_t lastChange[5] = {0, 0, 0, 0, 0};

// buzzer
const Note* patPtr = nullptr;
uint8_t  patLen = 0, patIdx = 0;
uint32_t patNoteStart = 0;
bool     patPlaying = false;

// OLED
bool     oledDirty = true;
uint32_t lastOledMs = 0;
char     overlayMsg[44] = "";
uint32_t overlayUntil = 0;
bool     overlayActive = false;

// ringkasan hari
int      sumNum10 = 0, sumDen = 0, sumScore = 0;
char     sumGrade[2] = "F";

// Serial
char     serialBuf[SERIAL_BUF];
size_t   serialLen = 0;
bool     serialOverflow = false;

const uint8_t TIER_WEIGHT[4] = {0, 3, 2, 2};

// Jadwal demo (bagian 5.3) - dipakai sebagai default/fallback
const char DEFAULT_SCHEDULE_JSON[] =
  "{\"day\":\"Mon\",\"tasks\":["
  "{\"name\":\"Math homework\",\"tier\":1,\"start\":\"14:00\",\"end\":\"14:40\",\"if_then\":\"If I feel distracted, I put my phone away.\"},"
  "{\"name\":\"Thesis writing\",\"tier\":1,\"start\":\"15:00\",\"end\":\"15:40\",\"if_then\":\"If I get stuck, I write one rough sentence.\"},"
  "{\"name\":\"Gym\",\"tier\":2,\"start\":\"16:00\",\"end\":\"16:30\",\"if_then\":\"If I feel tired, I start with 5 minutes only.\"},"
  "{\"name\":\"Rest and games\",\"tier\":3,\"start\":\"16:50\",\"end\":\"17:20\",\"if_then\":\"When the alarm sounds, I put the work away.\"}]}";

// Pola buzzer
const Note PAT_PRE[]    = {{1500,120},{0,120},{1500,120}};
const Note PAT_ALARM[]  = {{2000,200},{0,100},{2000,200},{0,100},{2000,200},{0,100},{2500,500}};
const Note PAT_NUDGE[]  = {{2500,150},{0,100},{2500,150},{0,100},{2500,300}};
const Note PAT_BREAK[]  = {{800,150},{0,80},{1200,200}};
const Note PAT_JINGLE[] = {{523,150},{659,150},{784,150},{1047,350}};
const Note PAT_MISSED[] = {{300,400},{0,100},{200,600}};
const Note PAT_ERROR[]  = {{400,200}};
const Note PAT_GRADE[]  = {{523,150},{659,150},{784,150},{1047,200},{784,150},{1047,400}};

// ---------------- DEKLARASI MAJU ----------------
void publishEvent(const char* json);
void emitEvent(JsonDocument& doc);
void logEvent(const char* event, const Task* t, int minVal, int outcome10);
void enterState(State s);
bool loadSchedule(const char* json);

// ---------------- MQTT SET UP -------------------
const char* MQTT_HOST = "y78c1fd2.ala.asia-southeast1.emqxsl.com";   // contoh: xxxx.s1.eu.hivemq.cloud
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "ORCA-CobaSIC-Admin4";
const char* MQTT_PASS = "rawr";
const char* TOPIC_EVENT    = "sic8/ORCA-CobaSIC/event";
const char* TOPIC_SCHEDULE = "sic8/ORCA-CobaSIC/schedule";

WiFiClientSecure espClient;
PubSubClient client(espClient);
uint32_t lastMqttTry = 0;

// ============================================================
// UTILITAS
// ============================================================
void copyStr(char* dst, size_t cap, const char* src) {
  size_t i = 0;
  if (src) for (; i < cap - 1 && src[i]; i++) dst[i] = src[i];
  dst[i] = 0;
}

void fmtTime(int minutes, char* out) {           // out >= 6 byte
  int m = ((minutes % 1440) + 1440) % 1440;
  snprintf(out, 6, "%02d:%02d", m / 60, m % 60);
}

bool parseHHMM(const char* s, int& out) {
  if (!s || strlen(s) != 5 || s[2] != ':') return false;
  const int idx[4] = {0, 1, 3, 4};
  for (int i = 0; i < 4; i++) if (s[idx[i]] < '0' || s[idx[i]] > '9') return false;
  int h = (s[0] - '0') * 10 + (s[1] - '0');
  int m = (s[3] - '0') * 10 + (s[4] - '0');
  if (h > 23 || m > 59) return false;
  out = h * 60 + m;
  return true;
}

void touchOled() { oledDirty = true; }

void showOverlay(const char* msg, uint16_t ms) {
  copyStr(overlayMsg, sizeof(overlayMsg), msg);
  overlayUntil = millis() + ms;
  overlayActive = true;
  oledDirty = true;
}

// ============================================================
// JAM VIRTUAL  (menit simulasi = baseMin + detik_nyata x SPEED)
// ============================================================
double simMinutesF() {
  uint32_t ref = clockPaused ? pausedAt : millis();
  return baseMin + ((ref - t0) / 1000.0) * SPEED;
}
int simNow() { return (int)floor(simMinutesF()); }

void pauseClock()  { if (!clockPaused) { clockPaused = true; pausedAt = millis(); } }
void resumeClock() { if (clockPaused)  { t0 += millis() - pausedAt; clockPaused = false; } }

// ============================================================
// OUTPUT: LED, BUZZER (non-blocking), OLED
// ============================================================
void setLeds(bool g, bool y, bool r) {
  digitalWrite(PIN_LED_GREEN, g);
  digitalWrite(PIN_LED_YELLOW, y);
  digitalWrite(PIN_LED_RED, r);
}

void startNote() {
  if (patPtr[patIdx].freq > 0) tone(PIN_BUZZER, patPtr[patIdx].freq);
  else noTone(PIN_BUZZER);
}
void startPattern(const Note* p, uint8_t n) {
  patPtr = p; patLen = n; patIdx = 0; patPlaying = true;
  patNoteStart = millis();
  startNote();
}
#define PLAY(p) startPattern(p, sizeof(p) / sizeof(Note))

void updateBuzzer() {
  if (!patPlaying) return;
  if (millis() - patNoteStart >= patPtr[patIdx].ms) {
    patIdx++;
    if (patIdx >= patLen) { noTone(PIN_BUZZER); patPlaying = false; return; }
    patNoteStart = millis();
    startNote();
  }
}

void txtN(int y, const char* s, uint8_t size) {   // teks ukuran n di baris y
  display.setTextSize(size);
  display.setCursor(0, y);
  display.print(s);
}
void txt(int y, const char* s) { txtN(y, s, 1); }

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  char a[32], b[8], c[8];

  if (overlayActive) { txt(24, overlayMsg); display.display(); return; }

  if (nTasks == 0) { txt(0, "PACER"); txt(24, "Belum ada jadwal"); txt(40, "Kirim dari planner"); display.display(); return; }
  Task& t = tasks[cur];
  int now = simNow();
  fmtTime(now, c);

  switch (state) {
    case ST_IDLE:
      snprintf(a, sizeof(a), "PACER  %s  %s", dayName, c); txt(0, a);
      txt(16, "Next task:");
      txt(28, t.name);
      fmtTime(t.start, b); snprintf(a, sizeof(a), "at %s (T%d)", b, t.tier); txt(44, a);
      break;
    case ST_PRE_ALERT:
      fmtTime(t.start, b);
      if (delayedWait) { txt(0, "DELAYED"); txt(16, t.name); snprintf(a, sizeof(a), "Starts at %s", b); txt(32, a); }
      else { txt(0, "Starting in 5 min"); txt(16, t.name); snprintf(a, sizeof(a), "at %s", b); txt(32, a); }
      break;
    case ST_WAIT_CONFIRM:
      fmtTime(t.start, b); fmtTime(t.end, c);
      snprintf(a, sizeof(a), "%s", t.name); txt(0, a);
      { char r[8]; fmtTime(t.end, r); snprintf(a, sizeof(a), "%s-%s  T%d", b, r, t.tier); txt(9, a); }
      txt(20, t.ifThen);
      txt(48, "1:Ready  2:Delay");
      snprintf(a, sizeof(a), "3:Realloc  (D %d/%d)", t.delaysUsed, MAX_DELAYS); txt(56, a);
      break;
    case ST_DELAY_SELECT: {
      txt(0, "Delay how long?");
      snprintf(a, sizeof(a), "%d min", selValue); txtN(12, a, 2);
      fmtTime(t.start + selValue, b); fmtTime(t.end + selValue, c);
      snprintf(a, sizeof(a), "New: %s-%s", b, c); txt(34, a);
      txt(48, "4:+5  5:-5");
      txt(56, "1:OK  3:Cancel");
      break; }
    case ST_RUNNING:
      txt(0, t.name);
      { int left = (int)ceil(t.end - simMinutesF()); if (left < 0) left = 0;
        snprintf(a, sizeof(a), "%d min", left); txtN(16, a, 2); }
      txt(40, "left");
      fmtTime(t.end, b); snprintf(a, sizeof(a), "ends %s", b); txt(56, a);
      break;
    case ST_BREAK_PROMPT:
      txt(0, t.name); txt(16, "Halfway!"); txt(28, "Take a break?");
      txt(48, "1:Break"); txt(56, "3:Skip");
      break;
    case ST_BREAK_SELECT:
      txt(0, "Break length");
      snprintf(a, sizeof(a), "%d min", selValue); txtN(12, a, 2);
      snprintf(a, sizeof(a), "max %d", selMax); txt(34, a);
      txt(48, "4:+5  5:-5"); txt(56, "1:OK");
      break;
    case ST_BREAK: {
      txt(0, "Break");
      int left = breakEndMin - (int)floor(simMinutesF()); if (left < 0) left = 0;
      snprintf(a, sizeof(a), "%d min", left); txtN(16, a, 2);
      txt(40, "left");
      break; }
    case ST_DONE:
      txt(0, "DONE!"); txt(16, t.name);
      snprintf(a, sizeof(a), "Outcome %d.%d", t.outcome10 / 10, t.outcome10 % 10); txt(32, a);
      break;
    case ST_MISSED:
      txtN(0, "MISSED", 2); txt(24, t.name); txt(40, "Outcome 0.0");
      break;
    case ST_REALLOCATED:
      txt(0, "MOVED"); txt(16, t.name); txt(32, "to tomorrow");
      break;
    case ST_DAY_END:
      snprintf(a, sizeof(a), "DAY END  %s", dayName); txt(0, a);
      display.setTextSize(4); display.setCursor(0, 16); display.print(sumGrade);
      snprintf(a, sizeof(a), "%d", sumScore);
      display.setTextSize(2); display.setCursor(48, 22); display.print(a);
      snprintf(a, sizeof(a), "pts %d.%d of %d", sumNum10 / 10, sumNum10 % 10, sumDen); txt(56, a);
      break;
  }
  display.display();
}

void refreshOled() {
  if (overlayActive && (int32_t)(overlayUntil - millis()) <= 0) { overlayActive = false; oledDirty = true; }
  if (!oledDirty && millis() - lastOledMs < OLED_PERIOD_MS) return;
  oledDirty = false;
  lastOledMs = millis();
  if (oledOk) drawScreen();
}

// ============================================================
// EVENT: satu-satunya pintu keluar (Serial + MQTT)
// ============================================================

void publishEvent(const char* json) {
  if (client.connected()) client.publish(TOPIC_EVENT, json);
}

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  static char buf[1536];
  static uint32_t lastHash = 0;
  if (len >= sizeof(buf)) { Serial.println("# MQTT: jadwal terlalu besar"); return; }
  memcpy(buf, payload, len);
  buf[len] = 0;
  uint32_t h = 2166136261u;                       // hash FNV-1a
  for (unsigned int i = 0; i < len; i++) { h ^= (uint8_t)buf[i]; h *= 16777619u; }
  if (h == lastHash) return;                      // retained yang sama (sambung ulang): abaikan
  if (loadSchedule(buf)) lastHash = h;
}

void mqttMaintain() {                      // non-blocking, dipanggil tiap loop()
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) { client.loop(); return; }
  if (millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();
  String id = "pacer-" + String((uint32_t)esp_random(), HEX);
  if (client.connect(id.c_str(), MQTT_USER, MQTT_PASS)) {
    client.subscribe(TOPIC_SCHEDULE);
    Serial.println("# MQTT terhubung");
  } else {
    Serial.printf("# MQTT gagal, rc=%d\n", client.state());
  }
}

void emitEvent(JsonDocument& doc) {
  char buf[320];
  serializeJson(doc, buf, sizeof(buf));
  Serial.println(buf);
  publishEvent(buf);
}

// Event per tugas. minVal < 0 = tidak dicetak. outcome10 < 0 = tidak dicetak.
void logEvent(const char* event, const Task* t, int minVal, int outcome10) {
  JsonDocument doc;
  char tm[6], oc[8];
  doc["day"] = dayName;
  if (t) { doc["task"] = t->name; doc["tier"] = t->tier; }
  doc["event"] = event;
  if (minVal >= 0) doc["min"] = minVal;
  if (outcome10 >= 0) { snprintf(oc, sizeof(oc), "%d.%d", outcome10 / 10, outcome10 % 10); doc["outcome"] = serialized(oc); }
  fmtTime(simNow(), tm);
  doc["sim"] = tm;
  emitEvent(doc);
}

// ============================================================
// SKOR
// ============================================================
void computeSummary() {
  sumNum10 = 0; sumDen = 0;
  for (uint8_t i = 0; i < nTasks; i++) {
    sumNum10 += TIER_WEIGHT[tasks[i].tier] * tasks[i].outcome10;
    sumDen   += TIER_WEIGHT[tasks[i].tier];
  }
  sumScore = sumDen ? (10 * sumNum10) / sumDen : 0;          // dibulatkan ke bawah
  long x = 10L * sumNum10;                                   // = skor x den
  const char* g = "F";
  if      (x >= 95L * sumDen) g = "S";
  else if (x >= 85L * sumDen) g = "A";
  else if (x >= 75L * sumDen) g = "B";
  else if (x >= 60L * sumDen) g = "C";
  else if (x >= 45L * sumDen) g = "D";
  copyStr(sumGrade, sizeof(sumGrade), g);
}

void logDaySummary() {
  JsonDocument doc;
  char nb[12];
  snprintf(nb, sizeof(nb), "%d.%d", sumNum10 / 10, sumNum10 % 10);
  doc["day"] = dayName;
  doc["event"] = "DAY_SUMMARY";
  doc["num"] = serialized(nb);
  doc["den"] = sumDen;
  doc["score"] = sumScore;
  doc["grade"] = sumGrade;
  emitEvent(doc);
}

// ============================================================
// STATE MACHINE: transisi
// ============================================================
// State di mana jam virtual berhenti: menunggu jawaban pengguna (alarm) atau memilih menit
bool isSelectState(State s) { return s == ST_WAIT_CONFIRM || s == ST_DELAY_SELECT || s == ST_BREAK_SELECT; }

void enterState(State s) {
  char tm[6]; fmtTime(simNow(), tm);
  Serial.printf("# [%s] %s -> %s\n", tm, STATE_NAME[state], STATE_NAME[s]);
  if (!isSelectState(state) && isSelectState(s)) pauseClock();   // waktu berhenti saat memilih menit
  if (isSelectState(state) && !isSelectState(s)) resumeClock();
  state = s;
  stateEnterMs = millis();
  lastPressMs = millis();
  switch (s) {
    case ST_RUNNING: case ST_BREAK_PROMPT: case ST_BREAK_SELECT: setLeds(1, 0, 0); break;
    case ST_BREAK:   setLeds(0, 1, 0); break;
    case ST_MISSED:  setLeds(0, 0, 1); break;
    default:         setLeds(0, 0, 0); break;
  }
  touchOled();
}

void beginAlarm() {                       // alarm pertama untuk tugas ini
  nudgeCount = 0;
  lastBuzzMs = millis();
  PLAY(PAT_ALARM);
  enterState(ST_WAIT_CONFIRM);
}

void advance() {                          // tugas selesai diproses -> berikutnya / akhir hari
  cur++;
  if (cur >= nTasks) {
    cur = nTasks - 1;                     // tetap valid untuk tampilan
    computeSummary();
    logDaySummary();
    PLAY(PAT_GRADE);
    enterState(ST_DAY_END);
  } else {
    enterState(ST_IDLE);
  }
}

void finishTask() {
  Task& t = tasks[cur];
  int o = 10 - 2 * t.delaysUsed; if (o < 0) o = 0;
  t.outcome = OUT_DONE; t.outcome10 = o;
  logEvent("DONE", &t, -1, o);
  PLAY(PAT_JINGLE);
  enterState(ST_DONE);
}

void missTask() {
  Task& t = tasks[cur];
  t.outcome = OUT_MISSED; t.outcome10 = 0;
  logEvent("MISSED", &t, -1, 0);
  PLAY(PAT_MISSED);
  enterState(ST_MISSED);
}

void reallocateTask() {
  Task& t = tasks[cur];
  t.outcome = OUT_REALLOCATED; t.outcome10 = 3;
  logEvent("REALLOCATED", &t, -1, 3);
  enterState(ST_REALLOCATED);
}

// batas delay: next start - 5 - end saat ini, kelipatan 5
int maxDelayMin() {
  const Task& t = tasks[cur];
  int limit;
  if (cur + 1 < nTasks) limit = tasks[cur + 1].start - PRE_ALERT_MIN - t.end;
  else { limit = MAX_DELAY_LAST; if (1439 - t.end < limit) limit = 1439 - t.end; }
  if (limit < 0) limit = 0;
  return (limit / DELAY_STEP_MIN) * DELAY_STEP_MIN;
}

int maxBreakMin() {
  int left = tasks[cur].end - simNow();
  int m = left < BREAK_MAX_MIN ? left : BREAK_MAX_MIN;
  if (m < 0) m = 0;
  return (m / 5) * 5;
}

// ============================================================
// STATE MACHINE: handler per state
// ============================================================
// IDLE: tunggu sampai 5 menit sebelum tugas berikutnya.
void stIdle() {
  if (nTasks == 0) return;
  if (simNow() >= tasks[cur].start - PRE_ALERT_MIN) {
    delayedWait = false;
    breakOffered = false;
    PLAY(PAT_PRE);
    enterState(ST_PRE_ALERT);
  }
}

// PRE_ALERT: tunggu jam mulai (juga dipakai menunggu setelah Delay).
void stPreAlert() {
  if (simNow() >= tasks[cur].start) beginAlarm();
}

// WAIT_CONFIRM: alarm berbunyi; B1 Ready, B2 Delay, B3 Reallocate; nudge tiap NUDGE_SEC.
void stWaitConfirm() {
  Task& t = tasks[cur];
  uint32_t now = millis();
  if (pressed[0]) {
    logEvent("STARTED", &t, -1, -1);
    enterState(ST_RUNNING);
  } else if (pressed[1]) {
    lastBuzzMs = now;
    int mx = maxDelayMin();
    if (t.delaysUsed >= MAX_DELAYS) {
      Serial.printf("# DELAY DITOLAK: batas %d kali sudah terpakai (%s)\n", MAX_DELAYS, t.name);
      showOverlay("Delay limit reached", 1500); PLAY(PAT_ERROR);
    } else if (mx < DELAY_STEP_MIN) {
      Serial.printf("# DELAY DITOLAK: tidak ada ruang sebelum tugas berikutnya (%s)\n", t.name);
      showOverlay("No room to delay", 1500); PLAY(PAT_ERROR);
    } else {
      selMax = mx; selValue = DELAY_STEP_MIN;
      enterState(ST_DELAY_SELECT);
    }
  } else if (pressed[2]) {
    reallocateTask();
  } else if (now - lastBuzzMs >= (uint32_t)NUDGE_SEC * 1000UL) {
    if (nudgeCount < MAX_REBUZZ) {
      nudgeCount++;
      lastBuzzMs = now;
      PLAY(PAT_NUDGE);
      Serial.printf("# RE-BUZZ %d/%d (%s)\n", nudgeCount, MAX_REBUZZ, t.name);
      touchOled();
    } else {
      missTask();
    }
  }
}

// DELAY_SELECT: B4 +5, B5 -5, B1 konfirmasi, B3 batal. Jam virtual berhenti.
void stDelaySelect() {
  Task& t = tasks[cur];
  if (pressed[3] || pressed[4] || pressed[0] || pressed[2]) lastPressMs = millis();
  if (pressed[3] && selValue + DELAY_STEP_MIN <= selMax) { selValue += DELAY_STEP_MIN; touchOled(); }
  if (pressed[4] && selValue - DELAY_STEP_MIN >= DELAY_STEP_MIN) { selValue -= DELAY_STEP_MIN; touchOled(); }
  if (pressed[0]) {                                   // konfirmasi
    t.start += selValue; t.end += selValue; t.delaysUsed++;
    logEvent("DELAYED", &t, selValue, -1);            // sim = jam saat alarm berbunyi (jam masih berhenti)
    delayedWait = true;
    enterState(ST_PRE_ALERT);
  } else if (pressed[2] || millis() - lastPressMs >= (uint32_t)SELECT_TIMEOUT_SEC * 1000UL) {
    enterState(ST_WAIT_CONFIRM);                      // batal: alarm lanjut, timer nudge diulang
    lastBuzzMs = millis();
  }
}

// RUNNING: LED hijau; tawarkan break di titik tengah; selesai di jam akhir.
void stRunning() {
  Task& t = tasks[cur];
  int now = simNow();
  if (now >= t.end) { finishTask(); return; }
  int len = t.end - t.start;
  if (!breakOffered && len >= BREAK_MIN_TASK && now >= t.start + len / 2) {
    breakOffered = true;
    if (maxBreakMin() >= 5) { PLAY(PAT_BREAK); enterState(ST_BREAK_PROMPT); }
  }
}

// BREAK_PROMPT: B1 Break, B3 Skip; tanpa jawaban BREAK_PROMPT_SEC = Skip.
void stBreakPrompt() {
  if (simNow() >= tasks[cur].end) { finishTask(); return; }
  if (pressed[0]) {
    selMax = maxBreakMin(); selValue = 5;
    if (selMax < 5) enterState(ST_RUNNING); else enterState(ST_BREAK_SELECT);
  } else if (pressed[2] || millis() - stateEnterMs >= (uint32_t)BREAK_PROMPT_SEC * 1000UL) {
    enterState(ST_RUNNING);
  }
}

// BREAK_SELECT: B4 +5, B5 -5 (5..30, dijepit sisa waktu), B1 konfirmasi. Jam berhenti.
void stBreakSelect() {
  if (pressed[3] || pressed[4] || pressed[0]) lastPressMs = millis();
  if (pressed[3] && selValue + 5 <= selMax) { selValue += 5; touchOled(); }
  if (pressed[4] && selValue - 5 >= 5)      { selValue -= 5; touchOled(); }
  if (pressed[0]) {
    breakEndMin = simNow() + selValue;
    logEvent("BREAK", &tasks[cur], selValue, -1);
    enterState(ST_BREAK);
  } else if (millis() - lastPressMs >= (uint32_t)SELECT_TIMEOUT_SEC * 1000UL) {
    enterState(ST_RUNNING);
  }
}

// BREAK: LED kuning; waktu akhir tugas tidak bergeser.
void stBreak() {
  int now = simNow();
  if (now >= tasks[cur].end) { finishTask(); return; }
  if (now >= breakEndMin) { PLAY(PAT_BREAK); enterState(ST_RUNNING); }
}

// DONE / MISSED / REALLOCATED: tampilkan hasil sebentar, lalu lanjut.
void stHold() {
  if (millis() - stateEnterMs >= RESULT_HOLD_MS) advance();
}

// ============================================================
// LOADER JADWAL (dipanggil dari Serial sekarang, dari MQTT callback nanti)
// ============================================================
bool rejectSchedule(const char* why) {
  Serial.printf("# SCHEDULE DITOLAK: %s\n", why);
  return false;
}

bool loadSchedule(const char* json) {
  if (state != ST_IDLE && state != ST_DAY_END) {
    return rejectSchedule("perangkat sedang menjalankan tugas (hanya diterima saat IDLE/DAY_END)");
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) { char m[60]; snprintf(m, sizeof(m), "JSON tidak valid (%s)", err.c_str()); return rejectSchedule(m); }

  const char* day = doc["day"].as<const char*>();
  if (!day || !day[0]) return rejectSchedule("field 'day' kosong");
  JsonArrayConst arr = doc["tasks"].as<JsonArrayConst>();
  if (arr.isNull() || arr.size() == 0) return rejectSchedule("field 'tasks' kosong");
  if (arr.size() > MAX_TASKS) Serial.printf("# PERINGATAN: %d tugas, hanya %d pertama yang dipakai\n", (int)arr.size(), MAX_TASKS);

  Task tmp[MAX_TASKS];
  int n = 0;
  for (JsonObjectConst o : arr) {
    if (n >= MAX_TASKS) break;
    char m[60];
    const char* nm = o["name"].as<const char*>();
    int tier = o["tier"].as<int>();
    int st = 0, en = 0;
    if (!nm || !nm[0])                         { snprintf(m, sizeof(m), "tugas %d: nama kosong", n + 1); return rejectSchedule(m); }
    if (tier < 1 || tier > 3)                  { snprintf(m, sizeof(m), "tugas %d: tier harus 1-3", n + 1); return rejectSchedule(m); }
    if (!parseHHMM(o["start"].as<const char*>(), st) || !parseHHMM(o["end"].as<const char*>(), en)) {
      snprintf(m, sizeof(m), "tugas %d: jam harus HH:MM", n + 1); return rejectSchedule(m);
    }
    if (en <= st)                              { snprintf(m, sizeof(m), "tugas %d: end harus setelah start", n + 1); return rejectSchedule(m); }
    Task& t = tmp[n];
    copyStr(t.name, sizeof(t.name), nm);
    copyStr(t.ifThen, sizeof(t.ifThen), o["if_then"].as<const char*>());
    t.tier = tier; t.start = st; t.end = en;
    t.delaysUsed = 0; t.outcome = OUT_PENDING; t.outcome10 = 0;
    n++;
  }
  // urutkan berdasarkan start (insertion sort)
  for (int i = 1; i < n; i++) {
    Task k = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j].start > k.start) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  for (int i = 0; i + 1 < n; i++) {
    if (tmp[i + 1].start - tmp[i].end < MIN_GAP_MIN) {
      char m[70]; snprintf(m, sizeof(m), "jarak/overlap tugas '%s' dan '%s' < %d menit", tmp[i].name, tmp[i + 1].name, MIN_GAP_MIN);
      return rejectSchedule(m);
    }
  }

  // commit
  for (int i = 0; i < n; i++) tasks[i] = tmp[i];
  nTasks = n; cur = 0;
  copyStr(dayName, sizeof(dayName), day);
  baseMin = tasks[0].start - START_LEAD_MIN; if (baseMin < 0) baseMin = 0;
  t0 = millis(); clockPaused = false; lastTickMin = -1;
  delayedWait = false; breakOffered = false;
  sumNum10 = sumDen = sumScore = 0; copyStr(sumGrade, sizeof(sumGrade), "F");
  noTone(PIN_BUZZER); patPlaying = false;

  { JsonDocument ev; char tm[6]; fmtTime(simNow(), tm);
    ev["day"] = dayName; ev["event"] = "SCHEDULE_LOADED"; ev["n"] = nTasks; ev["sim"] = tm;
    emitEvent(ev); }
  char ov[24]; snprintf(ov, sizeof(ov), "%d tasks loaded", nTasks);
  showOverlay(ov, 2000);
  enterState(ST_IDLE);
  return true;
}

// ============================================================
// INPUT
// ============================================================
void pollButtons() {                    // isi pressed[] (true satu kali per tekan)
  for (uint8_t i = 0; i < 5; i++) {
    pressed[i] = false;
    bool raw = digitalRead(PIN_BTN[i]);
    if (raw != lastRaw[i]) { lastRaw[i] = raw; lastChange[i] = millis(); }
    if (millis() - lastChange[i] >= DEBOUNCE_MS && raw != stableVal[i]) {
      stableVal[i] = raw;
      if (raw == LOW) pressed[i] = true;
    }
  }
}

void printStatus() {
  char a[6], b[6], c[6];
  fmtTime(simNow(), a);
  Serial.printf("# STATUS state=%s sim=%s day=%s tugas=%d aktif=%d SPEED=%.1f NUDGE_SEC=%d\n",
                STATE_NAME[state], a, dayName, nTasks, cur + 1, (double)SPEED, NUDGE_SEC);
  for (uint8_t i = 0; i < nTasks; i++) {
    fmtTime(tasks[i].start, b); fmtTime(tasks[i].end, c);
    Serial.printf("#  %d. %s | T%d | %s-%s | delay %d | outcome %d.%d\n", i + 1, tasks[i].name,
                  tasks[i].tier, b, c, tasks[i].delaysUsed, tasks[i].outcome10 / 10, tasks[i].outcome10 % 10);
  }
}

void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '{') { loadSchedule(line); }
  else if (!strcmp(line, "default")) { loadSchedule(DEFAULT_SCHEDULE_JSON); }
  else if (!strcmp(line, "status")) { printStatus(); }
  else if (!strcmp(line, "help")) {
    Serial.println("# Perintah: {json jadwal} | default | status | help");
  } else { Serial.println("# Perintah tidak dikenal. Ketik help."); }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (serialOverflow) Serial.println("# Baris Serial terlalu panjang (>1535 karakter), diabaikan");
      else if (serialLen > 0) { serialBuf[serialLen] = 0; handleLine(serialBuf); }
      serialLen = 0; serialOverflow = false;
      continue;
    }
    if (serialLen < SERIAL_BUF - 1) serialBuf[serialLen++] = c; else serialOverflow = true;
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 5; i++) pinMode(PIN_BTN[i], INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  setLeds(0, 0, 0);

  Wire.begin(PIN_SDA, PIN_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) Serial.println("# PERINGATAN: OLED tidak ditemukan (cek 0x3C, SDA=21, SCL=22)");

  Serial.println("# PACER siap. Ketik help untuk daftar perintah.");
  t0 = millis();
  state = ST_IDLE;
  touchOled();

  WiFi.begin("Wokwi-GUEST", "", 6);
  espClient.setInsecure();                  // hanya untuk simulasi; tulis di laporan
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(onMqttMessage);
  client.setBufferSize(2048);
  client.setKeepAlive(60);

#if LOAD_DEFAULT_ON_BOOT
  loadSchedule(DEFAULT_SCHEDULE_JSON);
#endif
}

void loop() {
  pollButtons();
  handleSerial();

  switch (state) {                      // satu switch untuk seluruh state machine
    case ST_IDLE:          stIdle();        break;
    case ST_PRE_ALERT:     stPreAlert();    break;
    case ST_WAIT_CONFIRM:  stWaitConfirm(); break;
    case ST_DELAY_SELECT:  stDelaySelect(); break;
    case ST_RUNNING:       stRunning();     break;
    case ST_BREAK_PROMPT:  stBreakPrompt(); break;
    case ST_BREAK_SELECT:  stBreakSelect(); break;
    case ST_BREAK:         stBreak();       break;
    case ST_DONE:
    case ST_MISSED:
    case ST_REALLOCATED:   stHold();        break;
    case ST_DAY_END:       break;           // menunggu jadwal baru
  }

#if DEBUG_TICK
  int m = simNow();
  if (m != lastTickMin) { lastTickMin = m; char tm[6]; fmtTime(m, tm); Serial.printf("# tick %s %s\n", tm, STATE_NAME[state]); }
#endif

  mqttMaintain();
  updateBuzzer();
  refreshOled();
}
