#include <Wire.h>
#include <U8g2lib.h>


// U8G2 OLED Display - Page buffer version (saves RAM)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


// DHT22 Sensor - manual bit-bang (no library needed, more reliable)
#define DHT_PIN 2
uint8_t dhtData[5];  // raw bytes from sensor

// Read DHT22 manually - returns true on success
// failStep: 0=ok, 1=no response, 2=ack LOW, 3=ack HIGH, 4=bit timeout, 5=checksum
uint8_t dhtFailStep = 0;

bool readDHT() {
  uint8_t bits[5] = {0};
  dhtFailStep = 0;

  // Send start signal: pull LOW for 1ms (DHT22 needs only 1ms, 20ms also works)
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(2);
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(40);
  pinMode(DHT_PIN, INPUT_PULLUP);

  noInterrupts();

  uint16_t loopCnt;

  loopCnt = 10000;
  while (digitalRead(DHT_PIN) == HIGH) { if (--loopCnt == 0) { interrupts(); dhtFailStep = 1; return false; } }
  loopCnt = 10000;
  while (digitalRead(DHT_PIN) == LOW)  { if (--loopCnt == 0) { interrupts(); dhtFailStep = 2; return false; } }
  loopCnt = 10000;
  while (digitalRead(DHT_PIN) == HIGH) { if (--loopCnt == 0) { interrupts(); dhtFailStep = 3; return false; } }

  // Read 40 bits (5 bytes)
  for (uint8_t i = 0; i < 40; i++) {
    loopCnt = 10000;
    while (digitalRead(DHT_PIN) == LOW) { if (--loopCnt == 0) { interrupts(); dhtFailStep = 4; return false; } }
    uint16_t highCycles = 0;
    loopCnt = 10000;
    while (digitalRead(DHT_PIN) == HIGH) {
      highCycles++;
      if (--loopCnt == 0) { interrupts(); dhtFailStep = 4; return false; }
    }

    bits[i / 8] <<= 1;
    if (highCycles > 15) {
      bits[i / 8] |= 1;
    }
  }

  interrupts();

  // Verify checksum
  uint8_t checksum = bits[0] + bits[1] + bits[2] + bits[3];
  if ((checksum & 0xFF) != bits[4]) { dhtFailStep = 5; return false; }

  memcpy(dhtData, bits, 5);
  return true;
}


// Pin definitions
#define BUTTON_PIN 4
#define FRESH_WATER_PIN A2
#define GREY_WATER_PIN A3

// Calibrated sensor supply voltage (mV) - measured when sensors read correctly
#define SENSOR_SUPPLY_MV 4708L


// Tank capacities in liters
const float FRESH_TANK_CAPACITY_L = 105.0;
const float GREY_TANK_CAPACITY_L = 45.0;


// Smoothing variables
float smoothedFreshWaterPercent = 0.0;
float smoothedGreyWaterPercent = 0.0;
const float SMOOTHING_FACTOR = 0.1;
bool firstReading = true;


// Display states
#define DISPLAY_OFF 0
#define DISPLAY_NORMAL 1
#define DISPLAY_STATS 2
#define DISPLAY_DEBUG 3
uint8_t displayState = DISPLAY_NORMAL;
bool splashDone = false;

// Button state
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool buttonHeld = false;
bool longPressHandled = false;
#define LONG_PRESS_MS 5000
#define SHORT_PRESS_MS 300

// Current sensor readings
float temp1, humidity1;
float freshWaterLevel, greyWaterLevel;
long currentVcc = 5000;

// Stats tracking
float tempMin = 999.0, tempMax = -999.0;
float humMin = 999.0, humMax = -999.0;
float freshUsedLiters = 0.0;
float greyFilledLiters = 0.0;
float anchorFreshPercent = -1.0;  // "confirmed" level - only moves on sustained change
float anchorGreyPercent = -1.0;
unsigned long statsStartTime = 0;
uint8_t warmupCount = 0;
#define WARMUP_READINGS 30  // Wait 30 seconds for smoothing to settle
unsigned long lastSensorRead = 0;
#define SENSOR_INTERVAL 2500
uint8_t refillConfirmCount = 0;
#define REFILL_CONFIRM_SECS 10  // Must stay above 90% for 10 seconds

// Robust usage tracking - require sustained change before counting
#define USAGE_THRESHOLD_PCT 2.0    // Must change by 2% to count (= ~2.1L fresh, ~0.9L grey)
#define USAGE_CONFIRM_COUNT 5      // Must stay at new level for 5 consecutive readings (~12.5s)
float candidateFreshPercent = -1.0;
float candidateGreyPercent = -1.0;
uint8_t freshDropCount = 0;
uint8_t greyRiseCount = 0;


// Function declarations
float getWaterLevelPercent(float resistance);
float readResistance(int pin);
float smoothValue(float newValue, float oldValue);
void updateDisplay();
void handleButton();
void drawWaterTank(int x, int y, int w, int h, float percent, const char* label, float liters);
void updateStats();
void resetStats();
void drawStatsScreen();
void showResetAnimation();
void showStatsEntry();


long readVcc() {
 ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
 delay(2);
 ADCSRA |= _BV(ADSC);
 while (bit_is_set(ADCSRA, ADSC));
 uint16_t result = ADC;
 long vcc = (1100L * 1023L) / result;
 return vcc;
}


float getWaterLevelPercent(float resistance) {
 // Calibrated max resistance at full tank (~162 ohm measured)
 // 0 ohm = empty tank (sensor out of water)
 // 160 ohm = full tank (sensor fully submerged)
  const float MAX_RESISTANCE = 160.0;
  if (resistance <= 0) return 0.0;
 if (resistance >= MAX_RESISTANCE) return 100.0;
  float percent = (resistance / MAX_RESISTANCE) * 100.0;
 return constrain(percent, 0, 100);
}


// Calculate resistance - sensor is between 5V and pin, 200 ohm pull-down to GND
float readResistance(int pin) {
 // Average multiple readings for stability
 long sum = 0;
 for (int i = 0; i < 10; i++) {
   sum += analogRead(pin);
   delay(2);
 }
 float reading = sum / 10.0;
  if (reading >= 1023) return 0;      // Sensor shorted (0 ohm)
 if (reading <= 0) return 999999;    // Sensor open circuit
  // Sensors are powered from a separate supply (~4708mV) that doesn't track
 // the Nano's VCC. Compensate using measured VCC as ADC reference:
 // Vout = Vs * R_pulldown / (R_sensor + R_pulldown)
 // ADC = (Vout / VCC) * 1023
 // R_sensor = R_pulldown * ((Vs * 1023) / (VCC * ADC) - 1)
  float resistance = 200.0 * ((SENSOR_SUPPLY_MV * 1023.0) / (currentVcc * reading) - 1.0);
 return resistance;
}


// Smooth a value with exponential smoothing
float smoothValue(float newValue, float oldValue) {
 if (firstReading) {
   firstReading = false;
   return newValue;
 }
 return oldValue + SMOOTHING_FACTOR * (newValue - oldValue);
}


// Draw a vertical water tank with fill level
void drawWaterTank(int x, int y, int w, int h, float percent, const char* label, float liters) {
 // Draw tank body with rounded corners
 u8g2.drawRFrame(x, y, w, h, 3);
  // Draw cap/lid on top (more detailed)
 int capW = 12;
 int capH = 4;
 int capX = x + w/2 - capW/2;
 u8g2.drawBox(capX, y - 2, capW, 3);  // Cap base
 u8g2.drawFrame(capX + 2, y - capH - 1, capW - 4, 3);  // Cap top
  // Draw fill level (vertical fill from bottom) with rounded bottom
 int fillHeight = (int)((percent / 100.0) * (h - 4));
 if (fillHeight > 0) {
   if (fillHeight > 3) {
     // Main fill area
     u8g2.drawBox(x + 2, y + h - 2 - fillHeight, w - 4, fillHeight - 1);
     // Rounded bottom corners of water
     u8g2.drawPixel(x + 2, y + h - 3);
     u8g2.drawPixel(x + w - 3, y + h - 3);
   } else {
     u8g2.drawBox(x + 2, y + h - 2 - fillHeight, w - 4, fillHeight);
   }
  
   // Draw wave effect at water surface (if not full)
   if (percent < 95 && percent > 5) {
     int waveY = y + h - 2 - fillHeight;
     u8g2.setDrawColor(0);
     for (int i = 0; i < w - 6; i += 6) {
       u8g2.drawPixel(x + 3 + i, waveY);
       u8g2.drawPixel(x + 4 + i, waveY);
     }
     u8g2.setDrawColor(1);
     for (int i = 3; i < w - 6; i += 6) {
       u8g2.drawPixel(x + 3 + i, waveY - 1);
     }
   }
 }
  // Draw percentage inside tank
 char pctStr[5];
 dtostrf(percent, 2, 0, pctStr);
 strcat(pctStr, "%");
 u8g2.setFont(u8g2_font_6x10_tf);
 int pctWidth = u8g2.getStrWidth(pctStr);
  if (percent > 50) {
   u8g2.setDrawColor(0);
   u8g2.drawStr(x + (w - pctWidth) / 2, y + h/2 + 4, pctStr);
   u8g2.setDrawColor(1);
 } else {
   u8g2.drawStr(x + (w - pctWidth) / 2, y + h/2 + 4, pctStr);
 }
  // Draw label and liters on same line below tank
 u8g2.setFont(u8g2_font_5x7_tf);
  // Build combined string: "FRESH 105L"
 char infoStr[16];
 strcpy(infoStr, label);
 strcat(infoStr, " ");
 char literPart[6];
 dtostrf(liters, 3, 0, literPart);
 strcat(infoStr, literPart);
 strcat(infoStr, "L");
  int infoWidth = u8g2.getStrWidth(infoStr);
 u8g2.drawStr(x + (w - infoWidth) / 2, y + h + 8, infoStr);
}


// Stats tracking functions
void resetStats() {
 tempMin = 999.0; tempMax = -999.0;
 humMin = 999.0; humMax = -999.0;
 freshUsedLiters = 0.0;
 greyFilledLiters = 0.0;
 anchorFreshPercent = smoothedFreshWaterPercent;
 anchorGreyPercent = smoothedGreyWaterPercent;
 candidateFreshPercent = -1.0;
 candidateGreyPercent = -1.0;
 freshDropCount = 0;
 greyRiseCount = 0;
 warmupCount = WARMUP_READINGS;  // Already warmed up after reset
 statsStartTime = millis();
}

void updateStats() {
 // Track temp/humidity min/max
 if (!isnan(temp1)) {
   if (temp1 < tempMin) tempMin = temp1;
   if (temp1 > tempMax) tempMax = temp1;
 }
 if (!isnan(humidity1)) {
   if (humidity1 < humMin) humMin = humidity1;
   if (humidity1 > humMax) humMax = humidity1;
 }

 // Wait for smoothing to converge before tracking water usage
 if (warmupCount < WARMUP_READINGS) {
   warmupCount++;
   anchorFreshPercent = smoothedFreshWaterPercent;
   anchorGreyPercent = smoothedGreyWaterPercent;
   return;
 }

 if (anchorFreshPercent < 0) {
   anchorFreshPercent = smoothedFreshWaterPercent;
   anchorGreyPercent = smoothedGreyWaterPercent;
   return;
 }

 // Auto-reset water stats when tank is refilled
 if (smoothedFreshWaterPercent >= 90.0 && anchorFreshPercent < 70.0) {
   refillConfirmCount++;
   if (refillConfirmCount >= REFILL_CONFIRM_SECS) {
     freshUsedLiters = 0.0;
     greyFilledLiters = 0.0;
     anchorFreshPercent = smoothedFreshWaterPercent;
     anchorGreyPercent = smoothedGreyWaterPercent;
     candidateFreshPercent = -1.0;
     candidateGreyPercent = -1.0;
     freshDropCount = 0;
     greyRiseCount = 0;
     statsStartTime = millis();
     refillConfirmCount = 0;
   }
 } else {
   refillConfirmCount = 0;
 }

 // FRESH water usage: only count when level drops by USAGE_THRESHOLD_PCT
 // and stays there for USAGE_CONFIRM_COUNT consecutive readings
 float freshDrop = anchorFreshPercent - smoothedFreshWaterPercent;
 if (freshDrop >= USAGE_THRESHOLD_PCT) {
   // Level is below anchor by threshold - start or continue confirming
   if (candidateFreshPercent < 0) {
     candidateFreshPercent = smoothedFreshWaterPercent;
     freshDropCount = 1;
   } else if (abs(smoothedFreshWaterPercent - candidateFreshPercent) < USAGE_THRESHOLD_PCT) {
     // Still near candidate level
     freshDropCount++;
     candidateFreshPercent = smoothedFreshWaterPercent;  // Track latest
   } else if (smoothedFreshWaterPercent < candidateFreshPercent) {
     // Dropped even further - update candidate
     candidateFreshPercent = smoothedFreshWaterPercent;
     freshDropCount = 1;
   }
   if (freshDropCount >= USAGE_CONFIRM_COUNT) {
     // Confirmed! Count the usage from anchor to current level
     float confirmedDrop = anchorFreshPercent - smoothedFreshWaterPercent;
     freshUsedLiters += (confirmedDrop / 100.0) * FRESH_TANK_CAPACITY_L;
     anchorFreshPercent = smoothedFreshWaterPercent;
     candidateFreshPercent = -1.0;
     freshDropCount = 0;
   }
 } else {
   // Level is near anchor (noise) - reset candidate
   candidateFreshPercent = -1.0;
   freshDropCount = 0;
   // If level rose slightly above anchor (noise), move anchor up to avoid false triggers later
   if (smoothedFreshWaterPercent > anchorFreshPercent + 0.5) {
     anchorFreshPercent = smoothedFreshWaterPercent;
   }
 }

 // GREY water tracking: only count when level rises by threshold and stays
 float greyRise = smoothedGreyWaterPercent - anchorGreyPercent;
 if (greyRise >= USAGE_THRESHOLD_PCT) {
   if (candidateGreyPercent < 0) {
     candidateGreyPercent = smoothedGreyWaterPercent;
     greyRiseCount = 1;
   } else if (abs(smoothedGreyWaterPercent - candidateGreyPercent) < USAGE_THRESHOLD_PCT) {
     greyRiseCount++;
     candidateGreyPercent = smoothedGreyWaterPercent;
   } else if (smoothedGreyWaterPercent > candidateGreyPercent) {
     candidateGreyPercent = smoothedGreyWaterPercent;
     greyRiseCount = 1;
   }
   if (greyRiseCount >= USAGE_CONFIRM_COUNT) {
     float confirmedRise = smoothedGreyWaterPercent - anchorGreyPercent;
     greyFilledLiters += (confirmedRise / 100.0) * GREY_TANK_CAPACITY_L;
     anchorGreyPercent = smoothedGreyWaterPercent;
     candidateGreyPercent = -1.0;
     greyRiseCount = 0;
   }
 } else {
   candidateGreyPercent = -1.0;
   greyRiseCount = 0;
   if (smoothedGreyWaterPercent < anchorGreyPercent - 0.5) {
     anchorGreyPercent = smoothedGreyWaterPercent;
   }
 }
}

void showResetAnimation() {
 for (int frame = 0; frame < 10; frame++) {
   u8g2.firstPage();
   do {
     u8g2.setFont(u8g2_font_8x13B_tf);
     if (frame < 5) {
       u8g2.drawStr(16, 35, "Resetting");
       // Draw expanding circle
       u8g2.drawCircle(64, 38, frame * 6);
     } else {
       u8g2.drawStr(28, 35, "Reset OK");
       u8g2.drawDisc(64, 52, 3);
     }
   } while (u8g2.nextPage());
   delay(100);
 }
}

void drawStatsScreen() {
 u8g2.setFont(u8g2_font_5x7_tf);
 char buf[24];

 // Title
 unsigned long uptimeSec = (millis() - statsStartTime) / 1000UL;
 unsigned long days = uptimeSec / 86400UL;
 unsigned long hours = (uptimeSec % 86400UL) / 3600UL;

 sprintf(buf, "STATS  Up:%ldd %ldh", days, hours);
 u8g2.drawStr(0, 7, buf);
 u8g2.drawHLine(0, 9, 128);

 // Temperature min/max
 if (tempMin < 900) {
   char tMin[7], tMax[7];
   dtostrf(tempMin, 4, 1, tMin);
   dtostrf(tempMax, 4, 1, tMax);
   sprintf(buf, "Temp:%s~%sC", tMin, tMax);
 } else {
   strcpy(buf, "Temp: no data");
 }
 u8g2.drawStr(0, 19, buf);

 // Humidity min/max
 if (humMin < 900) {
   char hMin[7], hMax[7];
   dtostrf(humMin, 4, 1, hMin);
   dtostrf(humMax, 4, 1, hMax);
   sprintf(buf, "Hum: %s~%s%%", hMin, hMax);
 } else {
   strcpy(buf, "Hum:  no data");
 }
 u8g2.drawStr(0, 28, buf);

 // Fresh water used
 char fUsed[7];
 dtostrf(freshUsedLiters, 5, 1, fUsed);
 sprintf(buf, "Fresh used:%sL", fUsed);
 u8g2.drawStr(0, 38, buf);

 // Grey water produced
 char gFill[7];
 dtostrf(greyFilledLiters, 5, 1, gFill);
 sprintf(buf, "Grey fill: %sL", gFill);
 u8g2.drawStr(0, 47, buf);

 // Average fresh usage per day
 float uptimeHrs = uptimeSec / 3600.0;
 if (uptimeHrs >= 1.0) {
   float uptimeDays = uptimeSec / 86400.0;
   char avg[7];
   dtostrf(freshUsedLiters / uptimeDays, 4, 1, avg);
   sprintf(buf, "Fresh:%sL/day avg", avg);
 } else {
   unsigned int mins = uptimeSec / 60;
   sprintf(buf, "Need 1h for avg (%um)", mins);
 }
 u8g2.drawStr(0, 56, buf);

 // Hint at bottom
 u8g2.drawStr(0, 64, "Tap:back  Hold:reset");
}


// Display function with graphics
void updateDisplay() {
 if (displayState == DISPLAY_OFF) return;

 u8g2.firstPage();
 do {
   if (displayState == DISPLAY_STATS) {
     drawStatsScreen();
   } else {
     // Normal display mode
     // Title bar with temperature/humidity
     u8g2.setFont(u8g2_font_6x10_tf);
     u8g2.drawStr(0, 10, "In:");
    
     if (!isnan(temp1)) {
       char str[6];
       dtostrf(temp1, 4, 1, str);
       u8g2.drawStr(18, 10, str);
       u8g2.drawStr(48, 10, "C");
     } else {
       u8g2.drawStr(18, 10, "ERR");
     }

     if (!isnan(humidity1)) {
       char humStr[6];
       dtostrf(humidity1, 4, 1, humStr);
       u8g2.drawStr(70, 10, humStr);
       u8g2.drawStr(100, 10, "%RH");
     }
    
     // Calculate liters
     float freshLiters = (smoothedFreshWaterPercent / 100.0) * FRESH_TANK_CAPACITY_L;
     float greyLiters = (smoothedGreyWaterPercent / 100.0) * GREY_TANK_CAPACITY_L;
    
     // Draw two vertical tanks side by side - full height
     drawWaterTank(8, 20, 45, 34, smoothedGreyWaterPercent, "GREY", greyLiters);
     drawWaterTank(75, 20, 45, 34, smoothedFreshWaterPercent, "FRESH", freshLiters);
   }

 } while (u8g2.nextPage());
}


void handleButton() {
 bool currentButtonState = digitalRead(BUTTON_PIN);

 // Button just pressed down
 if (currentButtonState == LOW && lastButtonState == HIGH) {
   buttonPressTime = millis();
   buttonHeld = true;
   longPressHandled = false;
 }

 // Button is being held - check for long press
 if (buttonHeld && currentButtonState == LOW && !longPressHandled) {
   unsigned long holdTime = millis() - buttonPressTime;
   if (holdTime >= LONG_PRESS_MS) {
     longPressHandled = true;
     if (displayState == DISPLAY_NORMAL) {
       // Long press on normal screen → enter stats with splash animation
       showStatsEntry();
       displayState = DISPLAY_STATS;
     } else if (displayState == DISPLAY_STATS) {
       // Long press on stats screen → reset stats
       showResetAnimation();
       resetStats();
       displayState = DISPLAY_STATS;
     }
   }
 }

 // Button released
 if (currentButtonState == HIGH && lastButtonState == LOW) {
   unsigned long holdTime = millis() - buttonPressTime;
   buttonHeld = false;
   if (!longPressHandled && holdTime >= 50) {
     // Short press
     if (displayState == DISPLAY_OFF) {
       displayState = DISPLAY_NORMAL;
     } else if (displayState == DISPLAY_NORMAL) {
       showCloudSweepOff();
       displayState = DISPLAY_OFF;
     } else if (displayState == DISPLAY_STATS) {
       displayState = DISPLAY_NORMAL;
     }
   }
 }

 lastButtonState = currentButtonState;
}


// Draw a simple cloud at position
void drawCloud(int x, int y, int size) {
 // Cloud made of overlapping circles
 int r = size;
 u8g2.drawDisc(x, y, r);
 u8g2.drawDisc(x + r, y - r/2, r);
 u8g2.drawDisc(x + r*2, y, r);
 u8g2.drawDisc(x + r, y + r/3, r - 1);
}


// Animation when turning off display - clouds sweep across screen
void showCloudSweepOff() {
 for (int frame = 0; frame < 20; frame++) {
   u8g2.firstPage();
   do {
     int baseX = frame * 10 - 80;

     drawCloud(baseX, 8, 5);
     drawCloud(baseX + 45, 10, 4);
     drawCloud(baseX + 90, 6, 6);

     drawCloud(baseX + 15, 25, 6);
     drawCloud(baseX + 60, 22, 5);
     drawCloud(baseX + 105, 28, 7);

     drawCloud(baseX + 5, 42, 7);
     drawCloud(baseX + 50, 40, 5);
     drawCloud(baseX + 95, 44, 6);

     drawCloud(baseX + 20, 58, 5);
     drawCloud(baseX + 65, 56, 6);
     drawCloud(baseX + 110, 60, 5);

   } while (u8g2.nextPage());
  
   delay(40);
 }
  // Clear display
 u8g2.firstPage();
 do {} while (u8g2.nextPage());
}


void showStatsEntry() {
 const char* phrases[] = {"Entering", "the void...", "Shhh...", "Sikret!"};
 for (int frame = 0; frame < 12; frame++) {
   u8g2.firstPage();
   do {
     int cloud1X = (frame * 4) % 150 - 20;
     int cloud2X = ((frame * 3) + 60) % 160 - 30;
     drawCloud(cloud1X, 12, 4);
     drawCloud(cloud2X, 8, 3);
     u8g2.setFont(u8g2_font_8x13B_tf);
     u8g2.drawStr(28, 35, "Chmurka");
     u8g2.drawStr(28, 52, "Monitor");
     u8g2.setFont(u8g2_font_5x7_tf);
     uint8_t idx = (frame / 3) % 4;
     u8g2.drawStr(28, 62, phrases[idx]);
   } while (u8g2.nextPage());
   delay(120);
 }
}


void showSplash() {
 for (int frame = 0; frame < 12; frame++) {
   u8g2.firstPage();
   do {
     int cloud1X = (frame * 4) % 150 - 20;
     int cloud2X = ((frame * 3) + 60) % 160 - 30;

     drawCloud(cloud1X, 12, 4);
     drawCloud(cloud2X, 8, 3);

     u8g2.setFont(u8g2_font_8x13B_tf);
     u8g2.drawStr(28, 35, "Chmurka");
     u8g2.drawStr(28, 52, "Monitor");
    
     u8g2.setFont(u8g2_font_5x7_tf);
     char loadStr[12] = "Starting";
     for (int i = 0; i < (frame % 4); i++) {
       strcat(loadStr, ".");
     }
     u8g2.drawStr(38, 62, loadStr);
    
   } while (u8g2.nextPage());
  
   delay(150);
 }
}


void setup() {
 Serial.begin(9600);


 u8g2.begin();
  // Show splash screen once
 showSplash();
 delay(2000);
 splashDone = true;
  // DHT11 needs ~1s after power-on; splash already gave us plenty of time
 pinMode(DHT_PIN, INPUT_PULLUP);
 delay(1500);
 Wire.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, LOW);

 statsStartTime = millis();

 // Quick pin diagnostic
 Serial.print(F("DHT pin D2 state: "));
 Serial.println(digitalRead(DHT_PIN) ? F("HIGH (OK)") : F("LOW (check wiring!)"));
 Serial.println(F("Chmurka monitor initialized"));
}


void loop() {
 handleButton();

 // Only read sensors every SENSOR_INTERVAL ms
 if (millis() - lastSensorRead < SENSOR_INTERVAL) return;
 lastSensorRead = millis();

 // Read DHT22 with retries
 temp1 = NAN;
 humidity1 = NAN;
 for (uint8_t attempt = 0; attempt < 3; attempt++) {
   if (attempt > 0) delay(200);
   bool ok = readDHT();
   if (ok) {
     // DHT22 format: 16-bit humidity, 16-bit temperature (bit 15 = sign)
     humidity1 = ((uint16_t)dhtData[0] << 8 | dhtData[1]) * 0.1;
     int16_t rawTemp = ((uint16_t)(dhtData[2] & 0x7F) << 8 | dhtData[3]);
     temp1 = rawTemp * 0.1;
     if (dhtData[2] & 0x80) temp1 = -temp1;  // negative temperature
     break;
   }
 }

 currentVcc = readVcc();


 // Read raw ADC values first for debugging
 int freshRaw = analogRead(FRESH_WATER_PIN);
 int greyRaw = analogRead(GREY_WATER_PIN);


 // Read resistance values
 float freshResistance = readResistance(FRESH_WATER_PIN);
 float greyResistance = readResistance(GREY_WATER_PIN);
  freshWaterLevel = getWaterLevelPercent(freshResistance);
 greyWaterLevel = getWaterLevelPercent(greyResistance);


 smoothedFreshWaterPercent = smoothValue(freshWaterLevel, smoothedFreshWaterPercent);
 smoothedGreyWaterPercent = smoothValue(greyWaterLevel, smoothedGreyWaterPercent);

 updateStats();

 // Serial debug output
 Serial.print(F("V:")); Serial.print(currentVcc);
 Serial.print(F(" T:")); Serial.print(temp1);
 Serial.print(F(" H:")); Serial.print(humidity1);
 Serial.print(F(" DHT:")); Serial.print(dhtFailStep);
 Serial.print(F(" D2:")); Serial.print(digitalRead(DHT_PIN));
 Serial.print(F(" F:")); Serial.print(freshRaw);
 Serial.print(F("/")); Serial.print(smoothedFreshWaterPercent, 0);
 Serial.print(F("% G:")); Serial.print(greyRaw);
 Serial.print(F("/")); Serial.print(smoothedGreyWaterPercent, 0);
 Serial.println(F("%"));


 updateDisplay();
}





