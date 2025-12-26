/* Mega 2560 – CoolingNeed m/ PIR-invert + lesbar Serial + auto-simulering.
   Pinner:
     DHT11 DATA -> D2,  VCC->5V, GND->GND
     PIR OUT    -> D4,  VCC->5V, GND->GND
     LED        -> D3 via 220R til anode, katode -> GND
   Serial: 115200
*/

#include <Arduino.h>

// ---------- Konfig ----------
const int   PIN_DHT = 2;
const int   PIN_PIR = 4;
const int   PIN_LED = 3;

const bool  PIR_ACTIVE_HIGH = true;   // Sett til false hvis PIR er "omvendt"

const float T_COMFORT_C = 24.0;
const float T_MAX_C     = 30.0;
const float HUMID_BOOST_TH  = 60.0;
const float HUMID_BOOST_MAX = 15.0;   // maks +15%-poeng
const unsigned long PRESENCE_HOLD_MS = 5UL*60UL*1000UL; // 5 min
const unsigned long DHT_PERIOD_MS    = 2000;            // les hver 2s

#define PRINT_JSON 0                 // sett 1 hvis du også vil ha JSON-linje

// ---------- Tilstand ----------
float g_tempC = NAN, g_humP = NAN;
bool  simMode = false;
int   failCount = 0;
unsigned long lastDHT = 0;
unsigned long lastPresenceMs = 0;

// ---------- Hjelp ----------
static float clamp01(float x){ return x<0?0:(x>1?1:x); }

// ---------- Robust DHT11 uten bibliotek ----------
static bool dht11_read_once(float &hum, float &tc) {
  uint8_t b[5] = {0,0,0,0,0};

  pinMode(PIN_DHT, OUTPUT);
  digitalWrite(PIN_DHT, LOW); delay(20);          // start >18 ms
  digitalWrite(PIN_DHT, HIGH); delayMicroseconds(30);
  pinMode(PIN_DHT, INPUT_PULLUP);

  if (pulseIn(PIN_DHT, LOW, 1000)  == 0) return false;
  if (pulseIn(PIN_DHT, HIGH, 1000) == 0) return false;

  for (int i=0; i<40; i++) {
    if (pulseIn(PIN_DHT, LOW, 1000) == 0) return false;
    unsigned long hi = pulseIn(PIN_DHT, HIGH, 1000);
    if (hi == 0) return false;
    b[i/8] <<= 1;
    b[i/8] |= (hi > 50) ? 1 : 0;  // terskel ~50us
  }
  if ((uint8_t)(b[0]+b[1]+b[2]+b[3]) != b[4]) return false;

  hum = b[0];     // DHT11: heltall
  tc  = b[2];
  return true;
}
static bool dht11_read(float &hum, float &tc) {
  for (int tries=0; tries<5; ++tries) {
    if (dht11_read_once(hum, tc)) return true;
    delay(50);
  }
  return false;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_LED, OUTPUT);
  randomSeed(analogRead(A0));     // for simulering

  Serial.println(F("=== CoolingNeed demo (PIR + DHT11 w/ auto-sim) ==="));
  Serial.println(F("[t]   Temp   Hum  Presence  Hold(s)  Base  +Hum  => Effective   Mode   Gauge"));
  Serial.println(F("-------------------------------------------------------------------------------"));
}

// ---------- Loop ----------
void loop() {
  // --- PIR med 'hold' og evt. invert ---
  int pirRaw = digitalRead(PIN_PIR);
  bool motion = PIR_ACTIVE_HIGH ? (pirRaw==HIGH) : (pirRaw==LOW);
  if (motion) lastPresenceMs = millis();

  unsigned long since = millis() - lastPresenceMs;
  bool  presence = (since < PRESENCE_HOLD_MS);
  float holdSec  = presence ? (float)(PRESENCE_HOLD_MS - since)/1000.0 : 0.0;

  // --- DHT eller simulering (hvert 2. sekund) ---
  if (millis() - lastDHT >= DHT_PERIOD_MS) {
    lastDHT = millis();
    float h, t;
    bool ok = dht11_read(h, t);

    if (ok) {
      g_tempC = t; g_humP = h;
      failCount = 0;
      simMode = false;
    } else {
      failCount++;
      if (failCount >= 8) simMode = true;

      if (simMode) {
        static float simT = 23.5, simH = 42.0;
        float driftT = (random(-4, 5)) * 0.05;      // -0.20 .. +0.20
        float driftH = (random(-3, 4)) * 0.5;       // -1.5 .. +1.5
        simT += driftT + (presence ? 0.05 : -0.03); // litt høyere når "folk i rommet"
        simH += driftH;
        if (simT < 20) simT = 20; if (simT > 33) simT = 33;
        if (simH < 30) simH = 30; if (simH > 85) simH = 85;
        g_tempC = simT; g_humP = simH;
      }
    }
  }

  // --- CoolingNeed ---
  float baseNeed = 0.0;
  if (isfinite(g_tempC)) {
    if (g_tempC <= T_COMFORT_C) baseNeed = 0.0;
    else if (g_tempC >= T_MAX_C) baseNeed = 100.0;
    else baseNeed = 100.0f * (g_tempC - T_COMFORT_C) / (T_MAX_C - T_COMFORT_C);
  }

  float humidAdj = 0.0;
  if (isfinite(g_humP) && g_humP > HUMID_BOOST_TH) {
    humidAdj = HUMID_BOOST_MAX * clamp01((g_humP - HUMID_BOOST_TH)/40.0f);
  }

  float needWithHum   = clamp01((baseNeed + humidAdj)/100.0f) * 100.0f;
  float effectiveNeed = presence ? needWithHum : (needWithHum * 0.5f);

  // LED = visuell indikator
  int pwm = (int)(clamp01(effectiveNeed/100.0f) * 255.0f + 0.5f);
  analogWrite(PIN_LED, pwm);

  // --- Lesbar utskrift 1x/s (med header + bargraf) ---
  static unsigned long t0 = 0;
  static int lineCount = 0;
  if (millis() - t0 >= 1000) {
    t0 = millis();
    unsigned long ts = millis()/1000;

    if (lineCount % 10 == 0) {
      Serial.println();
      Serial.println(F("[t]   Temp   Hum  Presence  Hold(s)  Base  +Hum  => Effective   Mode   Gauge"));
      Serial.println(F("-------------------------------------------------------------------------------"));
    }
    lineCount++;

    String tempStr = isfinite(g_tempC) ? String(g_tempC,1)+"C" : "NaN";
    String humStr  = isfinite(g_humP)  ? String((int)g_humP)+"%" : "NaN";

    Serial.print("["); Serial.print(ts); Serial.print("s]  ");
    Serial.print(tempStr); Serial.print("  ");
    Serial.print(humStr);  Serial.print("   ");
    Serial.print(presence ? "YES" : "NO "); Serial.print("       ");
    Serial.print((int)holdSec); Serial.print("      ");
    Serial.print((int)(baseNeed+0.5)); Serial.print("%   ");
    Serial.print((int)(humidAdj+0.5)); Serial.print("%   ");
    Serial.print("=> "); Serial.print((int)(effectiveNeed+0.5)); Serial.print("%    ");
    Serial.print(simMode ? "SIM  " : "REAL ");

    // ASCII-bargraf 0..20 tegn
    int bars = (int)(clamp01(effectiveNeed/100.0f)*20);
    Serial.print("  [");
    for (int i=0;i<20;i++) Serial.print(i<bars ? '#' : '-');
    Serial.println("]");

#if PRINT_JSON
    Serial.print("{\"t\":"); Serial.print(ts);
    Serial.print(",\"tempC\":");   if (isfinite(g_tempC)) Serial.print(g_tempC,1); else Serial.print("null");
    Serial.print(",\"hum\":");     if (isfinite(g_humP))  Serial.print(g_humP,0);  else Serial.print("null");
    Serial.print(",\"presence\":"); Serial.print(presence?1:0);
    Serial.print(",\"base\":");     Serial.print((int)(baseNeed+0.5));
    Serial.print(",\"humAdj\":");   Serial.print((int)(humidAdj+0.5));
    Serial.print(",\"need\":");     Serial.print((int)(effectiveNeed+0.5));
    Serial.print(",\"sim\":");      Serial.print(simMode?1:0);
    Serial.println("}");
#endif
  }

  delay(20);
}
