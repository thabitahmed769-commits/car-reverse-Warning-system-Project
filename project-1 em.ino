#include <Arduino.h> // Main Arduino functions
#include <Wire.h> // For I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD display
#include <Adafruit_Sensor.h> // Required for BME280 sensor
#include <Adafruit_BME280.h> // Library for temperature sensor

// -------- Pins --------
#define LCD_SDA 19 // LCD SDA pin
#define LCD_SCL 18 // LCD SCL pin
#define BME_SDA 21 // BME280 SDA pin
#define BME_SCL 22 // BME280 SCL pin
#define TRIG_PIN 23 // Ultrasonic trigger pin
#define ECHO_PIN 34 // Ultrasonic echo pin
#define BUZZER_PIN 2 // Passive buzzer pin
#define BTN_MUTE 35 // Button pin (for mute)

// -------- LCD --------
#define LCD_ADDR 0x27 // LCD I2C address
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2); // Create LCD object (16x2)

// -------- I2C for BME sensor --------
TwoWire I2C_BME = TwoWire(1); // Use second I2C bus for BME280
Adafruit_BME280 bme; // Create BME object

// -------- Timing --------
const unsigned long LCD_INTERVAL = 300; // Update LCD every 300ms
unsigned long lastLcdMs = 0; // Store last update time

// -------- Distance constants --------
const float SOUND_SPEED_CM_PER_US = 0.0343f; // Speed of sound in cm/µs
const unsigned long PULSE_TIMEOUT = 35000UL; // Max waiting time for echo

// -------- Distance limits for tones --------
const int TH_LOW_MAX = 23; // 17–23 cm
const int TH_MED_MAX = 16; // 11–16 cm
const int TH_HIGH_MAX = 10; // ≤10 cm

// -------- State machine --------
enum State { SAFE, WARN_LOW, WARN_MED, WARN_HIGH }; // System states
State currentState = SAFE; // Start as SAFE

// -------- Mute button flags --------
volatile bool muteRequested = false; // Flag set by interrupt
bool mute = false; // True when buzzer is muted

// -------- Interrupt function --------
void IRAM_ATTR onMuteButton() {
muteRequested = true; // Mark that button was pressed
}

// -------- Function to read one distance measurement --------
long readDistanceOnceCm() {
digitalWrite(TRIG_PIN, LOW);
delayMicroseconds(3);
digitalWrite(TRIG_PIN, HIGH);
delayMicroseconds(12);
digitalWrite(TRIG_PIN, LOW);

unsigned long dur = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT); // Measure pulse
if (dur == 0) return -1; // If no echo, return -1
return (long)((dur * SOUND_SPEED_CM_PER_US) / 2.0f + 0.5f); // Convert to cm
}

// -------- Average multiple readings (For loop requirement) --------
long readDistanceAvgCm(uint8_t samples = 5) {
long sum = 0; int count = 0;
for (uint8_t i = 0; i < samples; i++) { // Loop to take multiple readings
long d = readDistanceOnceCm();
if (d >= 0) { sum += d; count++; } // Add only valid readings
delay(5);
}
return (count == 0) ? -1 : (sum / count); // Return average
}

// -------- Classify distance into system state (If / else) --------
State classify(long cm) {
if (cm < 0 || cm > TH_LOW_MAX) return SAFE; // Too far or invalid
if (cm <= TH_HIGH_MAX) return WARN_HIGH; // Close
if (cm <= TH_MED_MAX) return WARN_MED; // Medium
return WARN_LOW; // Low
}

// -------- Play sound based on current state --------
void playToneForState(State st) {
if (mute) { noTone(BUZZER_PIN); return; } // No sound if muted
switch (st) {
case WARN_HIGH: tone(BUZZER_PIN, 2000); break; // High tone (close)
case WARN_MED: tone(BUZZER_PIN, 1000); break; // Medium tone
case WARN_LOW: tone(BUZZER_PIN, 600); break; // Low tone
case SAFE: noTone(BUZZER_PIN); break; // No tone
}
}

// -------- Convert state to text for display --------
const char* stateName(State st) {
switch (st) {
case SAFE: return "SAFE";
case WARN_LOW: return "LOW ";
case WARN_MED: return "MED ";
case WARN_HIGH: return "HIGH";
}
return "?";
}


void setup() {
Serial.begin(115200); // Start Serial Monitor
delay(100);

// LCD setup
Wire.begin(LCD_SDA, LCD_SCL);
lcd.init(); lcd.backlight(); lcd.clear();
lcd.setCursor(0,0); lcd.print("Car Warning Sys");
lcd.setCursor(0,1); lcd.print("Init...");
delay(700); lcd.clear();

// BME280 sensor setup
I2C_BME.begin(BME_SDA, BME_SCL);
bool ok = bme.begin(0x76, &I2C_BME); // Try address 0x76
if (!ok) ok = bme.begin(0x77, &I2C_BME); // Try backup address 0x77
if (!ok) Serial.println("BME280 NOT FOUND"); // Print if not connected

// Ultrasonic and buzzer setup
pinMode(TRIG_PIN, OUTPUT);
pinMode(ECHO_PIN, INPUT);
pinMode(BUZZER_PIN, OUTPUT);
noTone(BUZZER_PIN); // Start silent

// Button setup (interrupt)
pinMode(BTN_MUTE, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(BTN_MUTE), onMuteButton, FALLING);

Serial.println("Setup done."); // Indicate setup complete
}


void loop() {

// Check if mute button was pressed
if (muteRequested) {
static unsigned long lastToggle = 0;
unsigned long now = millis();
if (now - lastToggle > 200) { // Debounce: ignore fast presses
mute = !mute; // Toggle mute ON/OFF
lastToggle = now;
Serial.print("Mute toggled: ");
Serial.println(mute ? "ON" : "OFF");
}
muteRequested = false; // Reset the flag
}

// Read distance and temperature
long distance = readDistanceAvgCm(5); // Get averaged distance
currentState = classify(distance); // Determine warning level
float tempC = bme.readTemperature(); // Read temperature from BME280

// Control buzzer based on state
playToneForState(currentState);

// Update LCD and Serial Monitor every 300ms
unsigned long now = millis();
if (now - lastLcdMs >= LCD_INTERVAL) {
lastLcdMs = now; // Save current time

lcd.clear();
lcd.setCursor(0,0);

// Show distance on LCD
if (distance < 0) lcd.print("Dist: --- cm ");
else {
lcd.print("Dist:");
lcd.print(distance);
lcd.print("cm ");
}
lcd.print(stateName(currentState)); // Show state name

// Show temperature and mute status on second line
lcd.setCursor(0,1);
if (isnan(tempC)) lcd.print("Temp: ---.-C");
else {
lcd.print("Temp:");
lcd.print(tempC, 1);
lcd.print("C ");
lcd.print(mute ? "MUTE" : "BEEP"); // Display mute status
}

// Print data to Serial Monitor (UART report)
Serial.print("Dist=");
Serial.print(distance);
Serial.print("cm State=");
Serial.print(stateName(currentState));
Serial.print(" Temp=");
Serial.print(tempC, 1);
Serial.print("C Mute=");
Serial.println(mute ? "ON" : "OFF");
}

}
