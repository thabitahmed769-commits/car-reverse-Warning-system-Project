#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// -------- Pins --------
#define LCD_SDA    19
#define LCD_SCL    18
#define BME_SDA    21
#define BME_SCL    22
#define TRIG_PIN   23
#define ECHO_PIN   34
#define BUZZER_PIN  2        
#define BTN_MUTE    35       

#define LCD_ADDR 0x27        
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

TwoWire I2C_BME = TwoWire(1);
Adafruit_BME280 bme;


const unsigned long LCD_INTERVAL = 300;  
unsigned long lastLcdMs = 0;

const float SOUND_SPEED_CM_PER_US = 0.0343f;
const unsigned long PULSE_TIMEOUT = 35000UL; 


const int TH_LOW_MAX  = 23; 
const int TH_MED_MAX  = 16;   
const int TH_HIGH_MAX = 10;  


enum State { SAFE, WARN_LOW, WARN_MED, WARN_HIGH };
State currentState = SAFE;


volatile bool muteRequested = false;
bool mute = false;


void IRAM_ATTR onMuteButton() {
  muteRequested = true;                  
}


long readDistanceOnceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
  if (dur == 0) return -1;
  return (long)((dur * SOUND_SPEED_CM_PER_US) / 2.0f + 0.5f);
}

long readDistanceAvgCm(uint8_t samples = 5) {
  long sum = 0; int count = 0;
  for (uint8_t i = 0; i < samples; i++) {
    long d = readDistanceOnceCm();
    if (d >= 0) { sum += d; count++; }
    delay(5);
  }
  return (count == 0) ? -1 : (sum / count);
}

State classify(long cm) {
  if (cm < 0 || cm > TH_LOW_MAX)   return SAFE;
  if (cm <= TH_HIGH_MAX)           return WARN_HIGH;
  if (cm <= TH_MED_MAX)            return WARN_MED;
  return WARN_LOW;
}

void playToneForState(State st) {
  if (mute) { noTone(BUZZER_PIN); return; }
  switch (st) {
    case WARN_HIGH: tone(BUZZER_PIN, 2000); break; 
    case WARN_MED:  tone(BUZZER_PIN, 1000); break; 
    case WARN_LOW:  tone(BUZZER_PIN,  600); break; 
    case SAFE:      noTone(BUZZER_PIN);     break; 
  }
}

const char* stateName(State st) {
  switch (st) {
    case SAFE:      return "SAFE";
    case WARN_LOW:  return "LOW ";
    case WARN_MED:  return "MED ";
    case WARN_HIGH: return "HIGH";
  }
  return "?";
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // LCD bus
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print("Car Warning Sys");
  lcd.setCursor(0,1); lcd.print("Init...");
  delay(700); lcd.clear();

  I2C_BME.begin(BME_SDA, BME_SCL);
  bool ok = bme.begin(0x76, &I2C_BME);
  if (!ok) ok = bme.begin(0x77, &I2C_BME);
  if (!ok) Serial.println("BME280 NOT FOUND");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pinMode(BTN_MUTE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_MUTE), onMuteButton, FALLING);

  Serial.println("Setup done.");
}

void loop() {

  if (muteRequested) {
    static unsigned long lastToggle = 0;
    unsigned long now = millis();
    if (now - lastToggle > 200) {  
      mute = !mute;
      lastToggle = now;
      Serial.print("Mute toggled: "); Serial.println(mute ? "ON" : "OFF");
    }
    muteRequested = false;
  }

  long distance = readDistanceAvgCm(5);         
  currentState = classify(distance);
  float tempC = bme.readTemperature();

  playToneForState(currentState);


  unsigned long now = millis();
  if (now - lastLcdMs >= LCD_INTERVAL) {
    lastLcdMs = now;

    lcd.clear();
    lcd.setCursor(0,0);
    if (distance < 0) lcd.print("Dist: --- cm ");
    else {
      lcd.print("Dist:");
      lcd.print(distance);
      lcd.print("cm ");
    }
    lcd.print(stateName(currentState));  

    lcd.setCursor(0,1);
    if (isnan(tempC)) lcd.print("Temp: ---.-C");
    else {
      lcd.print("Temp:");
      lcd.print(tempC, 1);
      lcd.print("C ");
      lcd.print(mute ? "MUTE" : "BEEP");
    }

   
    Serial.print("Dist=");
    Serial.print(distance);
    Serial.print("cm  State=");
    Serial.print(stateName(currentState));
    Serial.print("  Temp=");
    Serial.print(tempC, 1);
    Serial.print("C  Mute=");
    Serial.println(mute ? "ON" : "OFF");
  }


}
