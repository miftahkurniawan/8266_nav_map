/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║  ESP8266 NAV MAP — OLED SSD1306 Navigation Display      ║
 * ║  • Mini-map: track KML + posisi user real-time          ║
 * ║  • Maneuver arrow: arah belok selanjutnya               ║
 * ║  • Stats: jarak, ETA, kecepatan, sisa                   ║
 * ║  • MQTT subscriber: broker.emqx.io:1883                 ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * WIRING:
 *   SSD1306 128x64:
 *     SCL → D1 (GPIO5)
 *     SDA → D2 (GPIO4)
 *     VCC → 3.3V
 *     GND → GND
 *   LED status → D4 (GPIO2) active HIGH
 *
 * LIBRARIES:
 *   - PubSubClient
 *   - ArduinoJson
 *   - Adafruit SSD1306
 *   - Adafruit GFX
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── CONFIG ────────────────────────────────────────────────
const char* WIFI_SSID     = "NAMA_WIFI_ANDA";
const char* WIFI_PASSWORD = "PASSWORD_WIFI_ANDA";
const char* CHANNEL_ID    = "NAVKU001";
const char* MQTT_HOST     = "broker.emqx.io";
const int   MQTT_PORT     = 1883;
#define     LED_PIN       13    // D4 — active HIGH
#define     OLED_ADDR     0x3C
#define     SCREEN_W      128
#define     SCREEN_H      64

// ── MAP AREA SPLIT ────────────────────────────────────────
// OLED dibagi dua zona:
//   [0..79]   x → MINI-MAP (80px lebar)
//   [80..127] x → PANEL INFO (48px lebar)
#define MAP_W   80
#define MAP_H   64
#define INFO_X  81
#define INFO_W  47

// ── MINI-MAP CONFIG ───────────────────────────────────────
#define MAX_PTS   20         // maks titik track yang bisa ditampung
#define MAP_ZOOM  400        // skala konversi derajat→pixel (semakin besar = zoom in)

// ── STRUCTS ───────────────────────────────────────────────
struct TrackPt {
  float lat;
  float lng;
};

struct NavState {
  bool  active       = false;
  char  maneuver[24] = "straight";
  int   distance     = 0;
  char  distUnit[4]  = "m";
  char  street[44]   = "";
  char  eta[8]       = "--:--";
  float remain       = 0;
  char  remainUnit[4]= "km";
  int   speed        = 0;
  int   heading      = 0;
  float lat          = 0;
  float lng          = 0;
  char  nextManeuver[24] = "straight";
  char  nextStreet[44]   = "";
  int   nextDist     = 0;
  // Mini-map
  TrackPt pts[MAX_PTS];
  int     ptCount    = 0;
  int     userPtIdx  = 0;
  int     hdg        = 0;
};

// ── GLOBALS ───────────────────────────────────────────────
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
WiFiClient       wifiClient;
PubSubClient     mqtt(wifiClient);
NavState         nav;

unsigned long lastMsgTime  = 0;
unsigned long lastDrawTime = 0;
bool          mqttConnected = false;

// ── FORWARD DECLARATIONS ──────────────────────────────────
void drawBoot(const char* msg);
void connectWiFi();
void connectMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int len);
void parseJson(const char* json);
void drawDisplay();
void drawMiniMap();
void drawInfoPanel();
void drawArrow(int cx, int cy, int r, const char* type, bool small_mode);
void drawTurnArrow(int cx, int cy, int r, const char* type);

// ── SETUP ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(4, 5); // SDA=D2(GPIO4), SCL=D1(GPIO5)
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.setRotation(0);
  oled.clearDisplay();
  oled.display();

  drawBoot("Menghubungkan WiFi...");
  connectWiFi();

  drawBoot("WiFi OK. MQTT...");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(2048); // payload besar (ada array pts)
  connectMQTT();

  drawBoot("Siap!");
  delay(600);
}

// ── LOOP ──────────────────────────────────────────────────
void loop() {
  if (!mqtt.connected()) {
    mqttConnected = false;
    connectMQTT();
  }
  mqtt.loop();

  // Gambar ulang setiap 150ms
  if (millis() - lastDrawTime > 150) {
    lastDrawTime = millis();
    drawDisplay();
  }

  // Timeout navigasi: jika tidak ada pesan 10 detik → nonaktif
  if (nav.active && millis() - lastMsgTime > 10000) {
    nav.active = false;
  }
}

// ── WIFI ──────────────────────────────────────────────────
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500); tries++;
  }
}

// ── MQTT ──────────────────────────────────────────────────
void connectMQTT() {
  char clientId[32];
  snprintf(clientId, 32, "esp8266nav_%04X", (uint16_t)ESP.getChipId());
  if (mqtt.connect(clientId)) {
    char topic[64];
    snprintf(topic, 64, "navmqtt/%s/data", CHANNEL_ID);
    mqtt.subscribe(topic);
    mqttConnected = true;
    digitalWrite(LED_PIN, HIGH);
  } else {
    delay(3000);
  }
}

// ── JSON PARSER ───────────────────────────────────────────
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  lastMsgTime = millis();
  char buf[2048];
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, payload, len);
  buf[len] = '\0';
  parseJson(buf);
}

void parseJson(const char* json) {
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) return;

  nav.active   = doc["a"] | 0;
  nav.distance = doc["d"] | 0;
  nav.speed    = doc["sp"] | 0;
  nav.heading  = doc["h"] | 0;
  nav.hdg      = doc["hdg"] | nav.heading;
  nav.remain   = doc["r"] | 0.0f;
  nav.nextDist = doc["nd"] | 0;
  nav.lat      = doc["la"] | 0.0f;
  nav.lng      = doc["ln"] | 0.0f;
  nav.userPtIdx = doc["pi"] | 0;

  strlcpy(nav.maneuver,     doc["m"]  | "straight", sizeof(nav.maneuver));
  strlcpy(nav.distUnit,     doc["u"]  | "m",        sizeof(nav.distUnit));
  strlcpy(nav.street,       doc["s"]  | "",         sizeof(nav.street));
  strlcpy(nav.eta,          doc["e"]  | "--:--",    sizeof(nav.eta));
  strlcpy(nav.remainUnit,   doc["ru"] | "km",       sizeof(nav.remainUnit));
  strlcpy(nav.nextManeuver, doc["nm"] | "straight", sizeof(nav.nextManeuver));
  strlcpy(nav.nextStreet,   doc["ns"] | "",         sizeof(nav.nextStreet));

  // Parse array titik track (pts)
  nav.ptCount = 0;
  JsonArray pts = doc["pts"];
  if (!pts.isNull()) {
    for (JsonArray pt : pts) {
      if (nav.ptCount >= MAX_PTS) break;
      nav.pts[nav.ptCount].lat = pt[0] | 0.0f;
      nav.pts[nav.ptCount].lng = pt[1] | 0.0f;
      nav.ptCount++;
    }
  }
}

// ── MAIN DRAW ─────────────────────────────────────────────
void drawDisplay() {
  oled.clearDisplay();

  if (!WiFi.isConnected()) {
    oled.setTextSize(1); oled.setTextColor(WHITE);
    oled.setCursor(10, 28); oled.print("Menghubungkan WiFi...");
    oled.display(); return;
  }
  if (!mqttConnected) {
    oled.setTextSize(1); oled.setTextColor(WHITE);
    oled.setCursor(10, 28); oled.print("Menghubungkan MQTT...");
    oled.display(); return;
  }
  if (!nav.active) {
    drawIdleScreen();
    oled.display(); return;
  }

  // Divider vertikal
  oled.drawFastVLine(INFO_X - 1, 0, SCREEN_H, WHITE);

  drawMiniMap();
  drawInfoPanel();

  oled.display();
}

// ── IDLE SCREEN ───────────────────────────────────────────
void drawIdleScreen() {
  // Kiri: ikon kompas besar
  oled.drawCircle(32, 32, 22, WHITE);
  oled.drawCircle(32, 32, 20, WHITE);
  // Jarum N
  oled.drawLine(32, 32, 32, 14, WHITE);
  oled.fillTriangle(29, 18, 35, 18, 32, 12, WHITE);
  // Jarum S
  oled.drawLine(32, 32, 32, 50, WHITE);
  oled.drawPixel(32, 51, WHITE);

  // Kanan: teks
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(68, 10); oled.print("NAV MAP");
  oled.setCursor(68, 22); oled.print("ESP8266");
  oled.drawFastHLine(68, 32, 58, WHITE);

  // Status
  char ch[20];
  snprintf(ch, 20, "Ch:%s", CHANNEL_ID);
  oled.setCursor(68, 36); oled.print(ch);
  oled.setCursor(68, 48);
  oled.print(mqttConnected ? "MQTT: OK" : "MQTT:...");

  // Dot MQTT
  if (mqttConnected) oled.fillCircle(124, 4, 3, WHITE);
  else { oled.drawCircle(124, 4, 3, WHITE); }
}

// ══════════════════════════════════════════════════════════
//  MINI-MAP (kiri, 80x64)
// ══════════════════════════════════════════════════════════
void drawMiniMap() {
  if (nav.ptCount < 2) {
    // Tidak ada data track → tampilkan kompas sederhana
    drawSimpleCompass();
    return;
  }

  // Titik referensi tengah = posisi user (nav.pts[nav.userPtIdx])
  int uidx = constrain(nav.userPtIdx, 0, nav.ptCount - 1);
  float refLat = nav.pts[uidx].lat;
  float refLng = nav.pts[uidx].lng;

  // Faktor skala lat→px dan lng→px
  // 1 derajat lat ≈ 111320m, 1 derajat lng ≈ 111320*cos(lat) m
  float cosLat = cos(refLat * PI / 180.0);
  // MAP_ZOOM = pixel per derajat (tuning zoom)
  float scLat = MAP_ZOOM;
  float scLng = MAP_ZOOM * cosLat;

  // Center map di tengah area peta
  int cx = MAP_W / 2;
  int cy = MAP_H / 2;

  // ── Gambar track (garis antar titik) ──────────────────
  for (int i = 0; i < nav.ptCount - 1; i++) {
    int x1 = cx + (int)((nav.pts[i].lng   - refLng) * scLng);
    int y1 = cy - (int)((nav.pts[i].lat   - refLat) * scLat);
    int x2 = cx + (int)((nav.pts[i+1].lng - refLng) * scLng);
    int y2 = cy - (int)((nav.pts[i+1].lat - refLat) * scLat);

    // Clip ke area peta
    if (x1 < 0 && x2 < 0) continue;
    if (x1 >= MAP_W && x2 >= MAP_W) continue;

    // Titik yang sudah dilewati = garis putus-putus (dotted)
    if (i < uidx) {
      // Garis sudah lewat — lebih tipis
      if (i % 2 == 0) oled.drawLine(x1, y1, x2, y2, WHITE);
    } else {
      // Garis ke depan — penuh
      oled.drawLine(x1, y1, x2, y2, WHITE);
    }
  }

  // ── Gambar titik-titik track kecil ─────────────────────
  for (int i = uidx + 1; i < nav.ptCount; i++) {
    int px = cx + (int)((nav.pts[i].lng - refLng) * scLng);
    int py = cy - (int)((nav.pts[i].lat - refLat) * scLat);
    if (px >= 0 && px < MAP_W && py >= 0 && py < MAP_H) {
      if (i == nav.ptCount - 1) {
        // Titik akhir — kotak
        oled.drawRect(px-2, py-2, 5, 5, WHITE);
      } else {
        oled.drawPixel(px, py, WHITE);
      }
    }
  }

  // ── Gambar heading arrow (posisi user) ─────────────────
  drawUserArrow(cx, cy, nav.hdg);
}

// Panah user di tengah mini-map
void drawUserArrow(int cx, int cy, int headingDeg) {
  // Lingkaran kecil
  oled.fillCircle(cx, cy, 4, BLACK);
  oled.drawCircle(cx, cy, 4, WHITE);

  // Panah heading
  float rad = headingDeg * PI / 180.0;
  int tx = cx + (int)(6 * sin(rad));
  int ty = cy - (int)(6 * cos(rad));
  oled.drawLine(cx, cy, tx, ty, WHITE);
  oled.fillCircle(tx, ty, 2, WHITE);
}

// Kompas sederhana jika tidak ada data track
void drawSimpleCompass() {
  int cx = MAP_W / 2, cy = MAP_H / 2, r = 22;
  oled.drawCircle(cx, cy, r, WHITE);

  // Jarum heading
  float rad = nav.hdg * PI / 180.0;
  int tx = cx + (int)((r - 4) * sin(rad));
  int ty = cy - (int)((r - 4) * cos(rad));
  oled.drawLine(cx, cy, tx, ty, WHITE);
  oled.fillCircle(tx, ty, 3, WHITE);

  // Label N
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(cx - 3, cy - r - 8);
  oled.print("N");

  // Derajat heading
  char buf[8];
  snprintf(buf, 8, "%d\xB0", nav.hdg);
  oled.setCursor(cx - strlen(buf) * 3, cy + r + 2);
  oled.print(buf);
}

// ══════════════════════════════════════════════════════════
//  INFO PANEL (kanan, 48x64)
// ══════════════════════════════════════════════════════════
void drawInfoPanel() {
  int x = INFO_X;

  // ── Zona 1: Arrow maneuver saat ini (atas, 28px tinggi) ─
  // Kotak atas untuk arrow
  drawTurnArrow(x + 23, 14, 12, nav.maneuver);

  // Jarak ke maneuver
  char distBuf[16];
  if (nav.distance >= 1000)
    snprintf(distBuf, 16, "%.1fkm", nav.distance / 1000.0);
  else
    snprintf(distBuf, 16, "%dm", nav.distance);

  oled.setTextSize(1); oled.setTextColor(WHITE);
  // Centered
  int dw = strlen(distBuf) * 6;
  oled.setCursor(x + (INFO_W - dw) / 2, 28);
  oled.print(distBuf);

  // Divider
  oled.drawFastHLine(x, 36, INFO_W, WHITE);

  // ── Zona 2: Next maneuver preview (tengah, 14px) ────────
  // Ikon next maneuver kecil
  drawTurnArrow(x + 8, 43, 6, nav.nextManeuver);

  // "lalu" + next dist
  char nextBuf[12];
  if (nav.nextDist >= 1000)
    snprintf(nextBuf, 12, "%.1fk", nav.nextDist / 1000.0);
  else
    snprintf(nextBuf, 12, "%d", nav.nextDist);

  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(x + 17, 39);
  oled.print("lalu");
  oled.setCursor(x + 17, 47);
  oled.print(nextBuf);
  if (nav.nextDist >= 1000) oled.print("km");
  else oled.print("m");

  // Divider
  oled.drawFastHLine(x, 53, INFO_W, WHITE);

  // ── Zona 3: Speed + status ──────────────────────────────
  // Speed (kiri)
  char spd[8];
  snprintf(spd, 8, "%d", nav.speed);
  oled.setTextSize(1); oled.setTextColor(WHITE);
  oled.setCursor(x + 1, 56);
  oled.print(spd);
  oled.print("k");

  // ETA (kanan)
  int etaw = strlen(nav.eta) * 6;
  oled.setCursor(x + INFO_W - etaw - 1, 56);
  oled.print(nav.eta);
}

// ══════════════════════════════════════════════════════════
//  TURN ARROW RENDERER
//  Menggambar ikon arah belok di OLED
//  cx,cy = pusat, r = radius/ukuran, type = string maneuver
// ══════════════════════════════════════════════════════════
void drawTurnArrow(int cx, int cy, int r, const char* type) {
  String t = String(type);

  if (t == "straight") {
    // Panah lurus ke atas
    oled.drawLine(cx, cy + r, cx, cy - r, WHITE);
    oled.fillTriangle(cx - r/2, cy - r/2, cx + r/2, cy - r/2, cx, cy - r - r/3, WHITE);

  } else if (t == "turn-right") {
    // Garis dari bawah ke kanan, lalu panah kanan
    oled.drawLine(cx - r/2, cy + r, cx - r/2, cy, WHITE);
    oled.drawLine(cx - r/2, cy, cx + r, cy, WHITE);
    oled.fillTriangle(cx + r - r/3, cy - r/2, cx + r - r/3, cy + r/2, cx + r + r/3, cy, WHITE);

  } else if (t == "turn-left") {
    // Garis dari bawah ke kiri, lalu panah kiri
    oled.drawLine(cx + r/2, cy + r, cx + r/2, cy, WHITE);
    oled.drawLine(cx + r/2, cy, cx - r, cy, WHITE);
    oled.fillTriangle(cx - r + r/3, cy - r/2, cx - r + r/3, cy + r/2, cx - r - r/3, cy, WHITE);

  } else if (t == "turn-slight-right") {
    oled.drawLine(cx, cy + r, cx, cy, WHITE);
    oled.drawLine(cx, cy, cx + r, cy - r, WHITE);
    oled.fillTriangle(cx + r - r/3, cy - r - r/3, cx + r + r/3, cy - r + r/3, cx + r + r/4, cy - r - r/4, WHITE);

  } else if (t == "turn-slight-left") {
    oled.drawLine(cx, cy + r, cx, cy, WHITE);
    oled.drawLine(cx, cy, cx - r, cy - r, WHITE);
    oled.fillTriangle(cx - r - r/3, cy - r - r/3, cx - r + r/3, cy - r + r/3, cx - r - r/4, cy - r - r/4, WHITE);

  } else if (t == "turn-sharp-right") {
    oled.drawLine(cx - r/2, cy + r, cx - r/2, cy + r/3, WHITE);
    oled.drawLine(cx - r/2, cy + r/3, cx + r, cy + r/3, WHITE);
    oled.fillTriangle(cx + r - r/3, cy - r/4, cx + r - r/3, cy + r, cx + r + r/3, cy + r/3, WHITE);

  } else if (t == "turn-sharp-left") {
    oled.drawLine(cx + r/2, cy + r, cx + r/2, cy + r/3, WHITE);
    oled.drawLine(cx + r/2, cy + r/3, cx - r, cy + r/3, WHITE);
    oled.fillTriangle(cx - r + r/3, cy - r/4, cx - r + r/3, cy + r, cx - r - r/3, cy + r/3, WHITE);

  } else if (t == "uturn") {
    // U-turn
    int ur = r * 2 / 3;
    oled.drawCircle(cx + ur/2, cy, ur, WHITE);
    // Hapus bagian bawah lingkaran
    oled.drawFastHLine(cx + ur/2 - ur - 2, cy, ur * 2 + 4, BLACK);
    oled.drawLine(cx + ur/2 - ur, cy, cx + ur/2 - ur, cy + r, WHITE);
    oled.fillTriangle(cx + ur/2 + ur - r/3, cy - r/2, cx + ur/2 + ur + r/3, cy - r/2, cx + ur/2 + ur, cy - r, WHITE);

  } else if (t == "roundabout") {
    // Lingkaran kecil
    oled.drawCircle(cx, cy, r * 2 / 3, WHITE);
    // Panah keluar atas
    oled.fillTriangle(cx - r/3, cy - r, cx + r/3, cy - r, cx, cy - r - r/2, WHITE);

  } else if (t == "arrive") {
    // Bendera / pin tujuan
    oled.drawLine(cx, cy - r, cx, cy + r, WHITE);
    oled.fillTriangle(cx, cy - r, cx + r + r/2, cy - r/2, cx, cy, WHITE);
    oled.drawFastHLine(cx - r/2, cy + r, r, WHITE);

  } else {
    // Default: lurus
    oled.drawLine(cx, cy + r, cx, cy - r, WHITE);
    oled.fillTriangle(cx - r/2, cy - r/2, cx + r/2, cy - r/2, cx, cy - r - r/3, WHITE);
  }
}

// ── BOOT SCREEN ───────────────────────────────────────────
void drawBoot(const char* msg) {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(WHITE);

  // Logo / judul
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("NAV MAP");
  oled.setTextSize(1);
  oled.setCursor(0, 10);
  oled.print("ESP8266 v2.0");
  oled.drawFastHLine(0, 20, SCREEN_W, WHITE);

  // Channel
  char ch[40];
  snprintf(ch, 40, "Ch: %s", CHANNEL_ID);
  oled.setCursor(0, 24);
  oled.print(ch);

  // Pesan status
  oled.setCursor(0, 40);
  oled.print(msg);

  // Spinner sederhana
  static int spin = 0;
  const char* spinChars[] = {"|", "/", "-", "\\"};
  oled.setCursor(120, 56);
  oled.print(spinChars[spin % 4]);
  spin++;

  oled.display();
}
