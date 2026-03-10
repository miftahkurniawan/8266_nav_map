/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║  ESP8266 NAV MAP v3 — OLED 128x64 Real Map Display     ║
 * ║                                                          ║
 * ║  Layout OLED:                                            ║
 * ║  ┌──────────────────────────┬─────────────────────────┐ ║
 * ║  │   N                      │  [ARROW]  belok         │ ║
 * ║  │  ↑                       │   350 m                 │ ║
 * ║  │   ════ track KML ═══●    ├─────────────────────────┤ ║
 * ║  │        ● GPS (user)      │ [→] lalu  80m           │ ║
 * ║  │   [  25m  ]              ├─────────────────────────┤ ║
 * ║  └──────────────────────────┴─────────────────────────┘ ║
 * ║   MAP_W=80px (radius 100m)   INFO_W=47px                 ║
 * ║                                                          ║
 * ║  WIRING:                                                 ║
 * ║    SSD1306 128x64:                                       ║
 * ║      SCL → D1 (GPIO5)                                   ║
 * ║      SDA → D2 (GPIO4)                                   ║
 * ║      VCC → 3.3V   GND → GND                            ║
 * ║    LED → D4 (GPIO2), active HIGH                        ║
 * ║                                                          ║
 * ║  LIBRARIES: PubSubClient, ArduinoJson 6.x,              ║
 * ║             Adafruit SSD1306, Adafruit GFX              ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ══════════════════════════════════════════════════════════
//  CONFIG — GANTI SESUAI JARINGAN ANDA
// ══════════════════════════════════════════════════════════
const char* WIFI_SSID     = "Gass!!";
const char* WIFI_PASSWORD = "astinapura";
const char* CHANNEL_ID    = "NAVKU001";
const char* MQTT_HOST     = "broker.emqx.io";
const int   MQTT_PORT     = 1883;

// ══════════════════════════════════════════════════════════
//  HARDWARE
// ══════════════════════════════════════════════════════════
#define LED_PIN    13          // D4, active HIGH
#define OLED_ADDR  0x3C
#define SCREEN_W   128
#define SCREEN_H   64

// ══════════════════════════════════════════════════════════
//  LAYOUT OLED
// ══════════════════════════════════════════════════════════
#define MAP_W      80         // lebar zona peta
#define MAP_H      64         // tinggi zona peta
#define MAP_CX     40         // pusat X peta = posisi user
#define MAP_CY     32         // pusat Y peta = posisi user
#define INFO_X     81         // mulai panel info
#define INFO_W     47         // lebar panel info

// ══════════════════════════════════════════════════════════
//  SKALA PETA: radius 100m tampil dalam MAP_CX/MAP_CY pixel
//  SCALE_X = MAP_CX / 100.0 = 0.40 px/m
//  SCALE_Y = MAP_CY / 100.0 = 0.32 px/m
// ══════════════════════════════════════════════════════════
#define RADIUS_M    100.0f
#define SCALE_X     (MAP_CX / RADIUS_M)
#define SCALE_Y     (MAP_CY / RADIUS_M)
#define MAX_PTS     80

// ══════════════════════════════════════════════════════════
//  DATA STRUCTS
// ══════════════════════════════════════════════════════════
struct MapPt {
  int8_t  dy;       // offset meter utara (+) / selatan (-)
  int8_t  dx;       // offset meter timur (+) / barat (-)
  uint8_t passed;   // 1=sudah dilewati
};

struct NavState {
  bool    active        = false;
  char    maneuver[24]  = "straight";
  int16_t distance      = 0;
  char    distUnit[4]   = "m";
  char    street[44]    = "";
  char    eta[8]        = "--:--";
  float   remain        = 0;
  char    remainUnit[4] = "km";
  int8_t  speed         = 0;
  int16_t hdg           = 0;         // heading derajat
  char    nextManeuver[24] = "straight";
  int16_t nextDist      = 0;
  // Map
  MapPt   pts[MAX_PTS];
  uint8_t ptCount       = 0;
  int8_t  userPtIdx     = 0;
  int8_t  targetDy      = 0;        // offset meter ke target berikutnya
  int8_t  targetDx      = 0;
};

// ══════════════════════════════════════════════════════════
//  GLOBALS
// ══════════════════════════════════════════════════════════
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
WiFiClient       wifiClient;
PubSubClient     mqtt(wifiClient);
NavState         nav;

unsigned long lastMsg  = 0;
unsigned long lastDraw = 0;
bool          mqttOk   = false;

// ══════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ══════════════════════════════════════════════════════════
void drawBoot(const char* msg);
void connectWiFi();
void connectMQTT();
void onMqttMsg(char* topic, byte* payload, unsigned int len);
void parseJson(const char* json);
void drawAll();
void drawMap();
void drawInfoPanel();
void drawTurnArrow(int cx, int cy, int r, const char* type);
void drawUserDot(int cx, int cy, int hdgDeg);
void drawNorthIndicator();
void drawScaleBar();
void drawDottedLine(int x1, int y1, int x2, int y2);
void drawIdleScreen();

// ══════════════════════════════════════════════════════════
//  SETUP & LOOP
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(4, 5);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.setRotation(0);
  oled.clearDisplay(); oled.display();

  drawBoot("Menghubungkan WiFi...");
  connectWiFi();
  drawBoot("WiFi OK. Ke MQTT...");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  mqtt.setBufferSize(4096);
  connectMQTT();
  drawBoot("Siap! Menunggu data...");
  delay(600);
}

void loop() {
  if (!mqtt.connected()) { mqttOk = false; connectMQTT(); }
  mqtt.loop();

  if (millis() - lastDraw > 120) {
    lastDraw = millis();
    drawAll();
  }

  // Timeout 10 detik tanpa pesan → nonaktif
  if (nav.active && millis() - lastMsg > 10000) {
    nav.active = false;
  }
}

// ══════════════════════════════════════════════════════════
//  WIFI / MQTT
// ══════════════════════════════════════════════════════════
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); drawBoot("WiFi...");
  }
}

void connectMQTT() {
  char cid[32];
  snprintf(cid, 32, "navmap_%04X", (uint16_t)ESP.getChipId());
  if (mqtt.connect(cid)) {
    char topic[64];
    snprintf(topic, 64, "navmqtt/%s/data", CHANNEL_ID);
    mqtt.subscribe(topic);
    mqttOk = true;
    digitalWrite(LED_PIN, HIGH);
  } else {
    delay(3000);
  }
}

// ══════════════════════════════════════════════════════════
//  JSON PARSE
// ══════════════════════════════════════════════════════════
void onMqttMsg(char* topic, byte* payload, unsigned int len) {
  lastMsg = millis();
  static char buf[4096];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len); buf[len] = '\0';
  parseJson(buf);
}

void parseJson(const char* json) {
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, json) != DeserializationError::Ok) return;

  nav.active   = doc["a"]   | 0;
  nav.distance = doc["d"]   | 0;
  nav.speed    = doc["sp"]  | 0;
  nav.hdg      = doc["hdg"] | (int)(doc["h"] | 0);
  nav.remain   = doc["r"]   | 0.0f;
  nav.nextDist = doc["nd"]  | 0;
  nav.userPtIdx= doc["pi"]  | 0;

  strlcpy(nav.maneuver,     doc["m"]  | "straight", sizeof(nav.maneuver));
  strlcpy(nav.distUnit,     doc["u"]  | "m",        sizeof(nav.distUnit));
  strlcpy(nav.street,       doc["s"]  | "",         sizeof(nav.street));
  strlcpy(nav.eta,          doc["e"]  | "--:--",    sizeof(nav.eta));
  strlcpy(nav.remainUnit,   doc["ru"] | "km",       sizeof(nav.remainUnit));
  strlcpy(nav.nextManeuver, doc["nm"] | "straight", sizeof(nav.nextManeuver));

  // Target offset
  JsonArray td = doc["td"];
  if (!td.isNull() && td.size() >= 2) {
    nav.targetDy = (int8_t)constrain((int)td[0], -100, 100);
    nav.targetDx = (int8_t)constrain((int)td[1], -100, 100);
  }

  // Titik track
  nav.ptCount = 0;
  JsonArray pts = doc["pts"];
  if (!pts.isNull()) {
    for (JsonArray pt : pts) {
      if (nav.ptCount >= MAX_PTS) break;
      nav.pts[nav.ptCount].dy     = (int8_t)constrain((int)(pt[0]|0), -100, 100);
      nav.pts[nav.ptCount].dx     = (int8_t)constrain((int)(pt[1]|0), -100, 100);
      nav.pts[nav.ptCount].passed = (uint8_t)(pt[2] | 0);
      nav.ptCount++;
    }
  }
}

// ══════════════════════════════════════════════════════════
//  MASTER DRAW
// ══════════════════════════════════════════════════════════
void drawAll() {
  oled.clearDisplay();
  if (!WiFi.isConnected()) {
    oled.setTextSize(1); oled.setTextColor(WHITE);
    oled.setCursor(4, 26); oled.print("Mencari WiFi..."); oled.display(); return;
  }
  if (!mqttOk) {
    oled.setTextSize(1); oled.setTextColor(WHITE);
    oled.setCursor(4, 26); oled.print("Mencari MQTT..."); oled.display(); return;
  }
  if (!nav.active) { drawIdleScreen(); oled.display(); return; }

  oled.drawFastVLine(MAP_W, 0, SCREEN_H, WHITE);
  drawMap();
  drawInfoPanel();
  oled.display();
}

// ══════════════════════════════════════════════════════════
//  MAP ZONE (80x64) — peta radius 100m, user di tengah
// ══════════════════════════════════════════════════════════

// Konversi offset meter → pixel
// dy+ = utara → y turun (piksel berkurang)
// dx+ = timur → x naik (piksel bertambah)
int mX(int8_t dx) { return MAP_CX + (int)(dx * SCALE_X + 0.5f); }
int mY(int8_t dy) { return MAP_CY - (int)(dy * SCALE_Y + 0.5f); }

void drawMap() {

  // ── A. Lingkaran batas radius 100m ──────────────────────
  // Radius X = MAP_CX - 1 pixel, Radius Y = MAP_CY - 1 pixel
  // Gambar ellipse karena skala X ≠ Y
  // Approx dengan 36 segmen
  float prevX = MAP_CX + (MAP_CX - 1);
  float prevY = MAP_CY;
  for (int deg = 10; deg <= 360; deg += 10) {
    float rad = deg * DEG_TO_RAD;
    float curX = MAP_CX + (MAP_CX - 1) * sin(rad);
    float curY = MAP_CY - (MAP_CY - 1) * cos(rad);
    oled.drawLine((int)prevX, (int)prevY, (int)curX, (int)curY, WHITE);
    prevX = curX; prevY = curY;
  }

  // ── B. Crosshair samar ───────────────────────────────────
  for (int xx = 2; xx < MAP_W - 2; xx += 8) oled.drawPixel(xx, MAP_CY, WHITE);
  for (int yy = 2; yy < MAP_H - 2; yy += 8) oled.drawPixel(MAP_CX, yy, WHITE);

  // ── C. Lingkaran radius 50m (setengah radius) ───────────
  float r50x = (MAP_CX - 1) * 0.5f;
  float r50y = (MAP_CY - 1) * 0.5f;
  prevX = MAP_CX + r50x;
  prevY = MAP_CY;
  for (int deg = 15; deg <= 360; deg += 15) {
    float rad = deg * DEG_TO_RAD;
    float curX = MAP_CX + r50x * sin(rad);
    float curY = MAP_CY - r50y * cos(rad);
    // Dotted: hanya tiap 2 segmen
    if ((deg / 15) % 2 == 0)
      oled.drawLine((int)prevX, (int)prevY, (int)curX, (int)curY, WHITE);
    prevX = curX; prevY = curY;
  }

  // ── D. Gambar track ──────────────────────────────────────
  if (nav.ptCount >= 2) {
    for (int i = 0; i < nav.ptCount - 1; i++) {
      int x1 = mX(nav.pts[i].dx),   y1 = mY(nav.pts[i].dy);
      int x2 = mX(nav.pts[i+1].dx), y2 = mY(nav.pts[i+1].dy);

      // Skip jika keduanya di luar area
      bool ok = (x1 >= 0 && x1 < MAP_W && y1 >= 0 && y1 < MAP_H) ||
                (x2 >= 0 && x2 < MAP_W && y2 >= 0 && y2 < MAP_H);
      if (!ok) continue;

      bool wasPassed = nav.pts[i].passed && nav.pts[i+1].passed;
      if (wasPassed) {
        // Jalur lewat → dotted tipis
        drawDottedLine(x1, y1, x2, y2);
      } else {
        // Jalur ke depan → garis tebal 2px
        oled.drawLine(x1, y1, x2, y2, WHITE);
        // Tebal +1 ke arah yang lebih dominan
        int adx = abs(x2 - x1), ady = abs(y2 - y1);
        if (adx >= ady)
          oled.drawLine(x1, y1+1, x2, y2+1, WHITE);
        else
          oled.drawLine(x1+1, y1, x2+1, y2, WHITE);
      }
    }

    // ── E. Titik node track ke depan ─────────────────────
    for (int i = 0; i < nav.ptCount; i++) {
      if (nav.pts[i].passed) continue;
      int px = mX(nav.pts[i].dx), py = mY(nav.pts[i].dy);
      if (px < 1 || px >= MAP_W - 1 || py < 1 || py >= MAP_H - 1) continue;
      // Titik akhir: kotak kecil
      if (i == nav.ptCount - 1 && !nav.pts[i].passed) {
        oled.drawRect(px-2, py-2, 5, 5, WHITE);
        oled.fillRect(px-1, py-1, 3, 3, WHITE);
      } else {
        oled.drawPixel(px, py, WHITE);
      }
    }
  }

  // ── F. Target berikutnya (berkedip) ──────────────────────
  {
    int tx = mX(nav.targetDx);
    int ty = mY(nav.targetDy);
    if (tx > 0 && tx < MAP_W - 1 && ty > 0 && ty < MAP_H - 1) {
      bool blink = (millis() / 400) % 2;
      if (blink) { oled.drawCircle(tx, ty, 4, WHITE); oled.drawCircle(tx, ty, 2, WHITE); }
      else        { oled.fillCircle(tx, ty, 3, WHITE); }
    }
  }

  // ── G. Posisi GPS User — SELALU di MAP_CX, MAP_CY ────────
  //    Hapus area user dulu agar bersih
  oled.fillCircle(MAP_CX, MAP_CY, 6, BLACK);
  drawUserDot(MAP_CX, MAP_CY, nav.hdg);

  // ── H. Dekorasi peta ─────────────────────────────────────
  drawNorthIndicator();
  drawScaleBar();
}

// Titik GPS user: lingkaran + panah heading
void drawUserDot(int cx, int cy, int hdgDeg) {
  // Background hitam (clear area)
  oled.fillCircle(cx, cy, 5, BLACK);
  // Lingkaran luar
  oled.drawCircle(cx, cy, 5, WHITE);
  // Isi dalam
  oled.fillCircle(cx, cy, 3, WHITE);
  // Titik hitam tengah
  oled.fillCircle(cx, cy, 1, BLACK);

  // Panah heading (segitiga ke arah gerak)
  float rad = hdgDeg * DEG_TO_RAD;
  float sinH = sin(rad), cosH = cos(rad);

  int tipX = cx + (int)(10 * sinH);
  int tipY = cy - (int)(10 * cosH);
  int lX   = cx + (int)(4 * sin(rad - 2.2f));
  int lY   = cy - (int)(4 * cos(rad - 2.2f));
  int rX   = cx + (int)(4 * sin(rad + 2.2f));
  int rY   = cy - (int)(4 * cos(rad + 2.2f));

  oled.fillTriangle(tipX, tipY, lX, lY, rX, rY, WHITE);
}

// Indikator Utara di pojok kiri atas peta
void drawNorthIndicator() {
  // Latar hitam kecil
  oled.fillRect(0, 0, 12, 14, BLACK);
  // Segitiga N
  oled.fillTriangle(6, 1, 2, 10, 10, 10, WHITE);
  // Titik tengah hitam
  oled.fillCircle(6, 7, 2, BLACK);
  // Label
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(3, 11); oled.print("N");
}

// Skala bar di pojok kanan bawah peta: 25m = 10px
void drawScaleBar() {
  int barLen = (int)(25 * SCALE_X); // = 10px
  int bx = MAP_W - barLen - 3;
  int by = MAP_H - 3;

  oled.fillRect(bx - 1, by - 8, barLen + 3, 9, BLACK);
  oled.drawFastHLine(bx, by, barLen, WHITE);
  oled.drawFastVLine(bx, by - 2, 3, WHITE);
  oled.drawFastVLine(bx + barLen, by - 2, 3, WHITE);
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(bx, by - 8);
  oled.print("25m");
}

// Garis putus-putus (jalur yang sudah lewat)
void drawDottedLine(int x1, int y1, int x2, int y2) {
  int dx = abs(x2 - x1), dy = abs(y2 - y1);
  int steps = max(dx, dy);
  if (steps == 0) { oled.drawPixel(x1, y1, WHITE); return; }
  for (int i = 0; i <= steps; i += 3) {
    int x = x1 + (x2 - x1) * i / steps;
    int y = y1 + (y2 - y1) * i / steps;
    if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H)
      oled.drawPixel(x, y, WHITE);
  }
}

// ══════════════════════════════════════════════════════════
//  INFO PANEL (47px kanan)
// ══════════════════════════════════════════════════════════
void drawInfoPanel() {
  int x = INFO_X;

  // ── Zona A: Arrow maneuver utama ────────────────────────
  drawTurnArrow(x + 23, 15, 13, nav.maneuver);

  char distBuf[12];
  if (nav.distance >= 1000)
    snprintf(distBuf, 12, "%.1fkm", nav.distance / 1000.0f);
  else
    snprintf(distBuf, 12, "%dm", (int)nav.distance);

  oled.setTextSize(1); oled.setTextColor(WHITE);
  int dw = strlen(distBuf) * 6;
  oled.setCursor(x + max(0, (INFO_W - dw) / 2), 30);
  oled.print(distBuf);

  // Divider A-B
  oled.drawFastHLine(x, 38, INFO_W, WHITE);

  // ── Zona B: Next maneuver ───────────────────────────────
  drawTurnArrow(x + 8, 45, 6, nav.nextManeuver);

  char nextBuf[10];
  if (nav.nextDist >= 1000) snprintf(nextBuf, 10, "%.1fk", nav.nextDist / 1000.0f);
  else                      snprintf(nextBuf, 10, "%d",    (int)nav.nextDist);

  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(x + 18, 39); oled.print("lalu");
  oled.setCursor(x + 18, 48); oled.print(nextBuf);
  if (nav.nextDist >= 1000) oled.print("k"); else oled.print("m");

  // Divider B-C
  oled.drawFastHLine(x, 54, INFO_W, WHITE);

  // ── Zona C: Speed + ETA ────────────────────────────────
  char spdBuf[5];
  snprintf(spdBuf, 5, "%d", (int)nav.speed);
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(x + 1, 56); oled.print(spdBuf); oled.print("k");

  int etaW = strlen(nav.eta) * 6;
  oled.setCursor(x + INFO_W - etaW - 1, 56);
  oled.print(nav.eta);
}

// ══════════════════════════════════════════════════════════
//  TURN ARROW — semua jenis maneuver
// ══════════════════════════════════════════════════════════
void drawTurnArrow(int cx, int cy, int r, const char* type) {
  String t = String(type);

  if (t == "arrive") {
    // Bendera tujuan
    oled.drawLine(cx - r/2, cy - r, cx - r/2, cy + r, WHITE);
    oled.fillTriangle(cx - r/2, cy - r, cx + r, cy - r/2, cx - r/2, cy, WHITE);
    oled.drawFastHLine(cx - r, cy + r, r, WHITE);
    return;
  }

  if (t == "uturn") {
    int ur = r * 6 / 10;
    oled.drawCircle(cx + ur, cy, ur, WHITE);
    // Hapus setengah bawah
    oled.fillRect(cx, cy, ur * 2 + ur + 1, r + 2, BLACK);
    oled.drawLine(cx, cy, cx, cy + r, WHITE);
    // Panah bawah
    oled.fillTriangle(cx - r/3, cy + r - r/2, cx + r/3, cy + r - r/2, cx, cy + r + r/3, WHITE);
    return;
  }

  if (t == "roundabout") {
    int cr = r * 5 / 8;
    oled.drawCircle(cx, cy, cr, WHITE);
    oled.fillTriangle(cx - r/3, cy - cr, cx + r/3, cy - cr, cx, cy - cr - r/2, WHITE);
    return;
  }

  // Batang bawah stem
  int sBot = cy + r;

  if (t == "straight") {
    oled.drawLine(cx, sBot, cx, cy - r, WHITE);
    oled.fillTriangle(cx - r/2, cy - r/2, cx + r/2, cy - r/2, cx, cy - r - r/3, WHITE);

  } else if (t == "turn-right") {
    oled.drawLine(cx - r/2, sBot, cx - r/2, cy, WHITE);
    oled.drawLine(cx - r/2, cy, cx + r, cy, WHITE);
    oled.fillTriangle(cx+r-r/3, cy-r/2, cx+r-r/3, cy+r/2, cx+r+r/2, cy, WHITE);

  } else if (t == "turn-left") {
    oled.drawLine(cx + r/2, sBot, cx + r/2, cy, WHITE);
    oled.drawLine(cx + r/2, cy, cx - r, cy, WHITE);
    oled.fillTriangle(cx-r+r/3, cy-r/2, cx-r+r/3, cy+r/2, cx-r-r/2, cy, WHITE);

  } else if (t == "turn-slight-right") {
    oled.drawLine(cx, sBot, cx, cy + r/3, WHITE);
    oled.drawLine(cx, cy + r/3, cx + r, cy - r, WHITE);
    oled.fillTriangle(cx+r-r/2, cy-r, cx+r+r/3, cy-r+r/2, cx+r+r/3, cy-r-r/3, WHITE);

  } else if (t == "turn-slight-left") {
    oled.drawLine(cx, sBot, cx, cy + r/3, WHITE);
    oled.drawLine(cx, cy + r/3, cx - r, cy - r, WHITE);
    oled.fillTriangle(cx-r+r/2, cy-r, cx-r-r/3, cy-r+r/2, cx-r-r/3, cy-r-r/3, WHITE);

  } else if (t == "turn-sharp-right") {
    oled.drawLine(cx - r/2, sBot, cx - r/2, cy + r/2, WHITE);
    oled.drawLine(cx - r/2, cy + r/2, cx + r, cy + r/2, WHITE);
    oled.fillTriangle(cx+r-r/3, cy, cx+r-r/3, cy+r, cx+r+r/2, cy+r/2, WHITE);

  } else if (t == "turn-sharp-left") {
    oled.drawLine(cx + r/2, sBot, cx + r/2, cy + r/2, WHITE);
    oled.drawLine(cx + r/2, cy + r/2, cx - r, cy + r/2, WHITE);
    oled.fillTriangle(cx-r+r/3, cy, cx-r+r/3, cy+r, cx-r-r/2, cy+r/2, WHITE);

  } else {
    // Default: lurus
    oled.drawLine(cx, sBot, cx, cy - r, WHITE);
    oled.fillTriangle(cx - r/2, cy - r/2, cx + r/2, cy - r/2, cx, cy - r - r/3, WHITE);
  }
}

// ══════════════════════════════════════════════════════════
//  IDLE SCREEN
// ══════════════════════════════════════════════════════════
void drawIdleScreen() {
  // Kompas besar di kiri
  int cx = 32, cy = 32, r = 26;
  oled.drawCircle(cx, cy, r, WHITE);
  oled.drawCircle(cx, cy, r - 2, WHITE);

  // Tick marks setiap 45°
  for (int deg = 0; deg < 360; deg += 45) {
    float rad = deg * DEG_TO_RAD;
    oled.drawLine(
      cx + (int)((r-2)*sin(rad)), cy - (int)((r-2)*cos(rad)),
      cx + (int)((r-6)*sin(rad)), cy - (int)((r-6)*cos(rad)), WHITE);
  }

  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(cx-3, cy-r+4); oled.print("N");
  oled.setCursor(cx-3, cy+r-11); oled.print("S");
  oled.setCursor(cx+r-9, cy-3); oled.print("E");
  oled.setCursor(cx-r+3, cy-3); oled.print("W");

  // Jarum utara
  oled.fillTriangle(cx, cy - r + 11, cx - 4, cy + 4, cx + 4, cy + 4, WHITE);
  oled.drawTriangle(cx, cy + r - 11, cx - 3, cy - 3, cx + 3, cy - 3, WHITE);

  // Center pin
  oled.fillCircle(cx, cy, 3, BLACK);
  oled.drawCircle(cx, cy, 3, WHITE);

  // Panel kanan
  oled.drawFastVLine(INFO_X, 0, SCREEN_H, WHITE);
  oled.setTextSize(1); oled.setTextColor(WHITE);

  oled.fillRect(INFO_X, 0, INFO_W, 12, WHITE);
  oled.setTextColor(BLACK);
  oled.setCursor(INFO_X + 2, 2); oled.print("NAV MAP");
  oled.setTextColor(WHITE);

  oled.setCursor(INFO_X + 2, 14); oled.print("v3.0");
  oled.setCursor(INFO_X + 2, 24); oled.print(CHANNEL_ID);

  oled.drawFastHLine(INFO_X, 34, INFO_W, WHITE);
  oled.setCursor(INFO_X + 2, 37);
  oled.print(mqttOk ? "MQTT  OK" : "MQTT ...");
  if (mqttOk) oled.fillCircle(SCREEN_W - 4, 54, 4, WHITE);
  else        { oled.drawCircle(SCREEN_W-4, 54, 4, WHITE);
                oled.drawLine(SCREEN_W-7,51,SCREEN_W-1,57,WHITE); }

  oled.drawFastHLine(INFO_X, 46, INFO_W, WHITE);
  oled.setCursor(INFO_X + 2, 49); oled.print("Menunggu");
  oled.setCursor(INFO_X + 2, 57); oled.print("navigasi...");
}

// ══════════════════════════════════════════════════════════
//  BOOT SCREEN
// ══════════════════════════════════════════════════════════
void drawBoot(const char* msg) {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(WHITE);

  // Header bar solid
  oled.fillRect(0, 0, SCREEN_W, 14, WHITE);
  oled.setTextColor(BLACK);
  oled.setCursor(4, 3); oled.print("NAV MAP  ESP8266 v3");
  oled.setTextColor(WHITE);

  oled.setCursor(2, 18); oled.print("Channel: "); oled.print(CHANNEL_ID);
  oled.setCursor(2, 30); oled.print(msg);

  // Progress dots animasi
  static uint8_t tick = 0;
  for (int i = 0; i < 8; i++) {
    int px = 8 + i * 15, py = 52;
    if (i < (tick % 9)) oled.fillCircle(px, py, 4, WHITE);
    else                oled.drawCircle(px, py, 4, WHITE);
  }
  tick++;
  oled.display();
}
