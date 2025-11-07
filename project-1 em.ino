
#include <Arduino.h>              // main Arduino functions
#include <Wire.h>                 // for I2C communication
#include <LiquidCrystal_I2C.h>    // library for the LCD display
#include <Adafruit_Sensor.h>      // needed for the BME280 sensor
#include <Adafruit_BME280.h>      // library for the BME280 temperature sensor

// pins used for lcd, sensor, buzzer and button
#define LCD_SDA    19             // LCD SDA pin
#define LCD_SCL    18             // LCD SCL pin
#define BME_SDA    21             // Bme280 SDA pin
#define BME_SCL    22             // Bme280 SCL pin
#define TRIG_PIN   23             // Ultrasonic trigger pin
#define ECHO_PIN   34             // Ultrasonic echo pin
#define BUZZER_PIN  2             // Buzzer pin
#define BTN_MUTE    35            // Button pin

// lcd setup
#define LCD_ADDR 0x27             // address for the lcd
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);   // lcd object with 16 columns and 2 rows


// bme sensor setup
TwoWire I2C_BME = TwoWire(1);     // create second I2C connection
Adafruit_BME280 bme;              // create bme sensor object


// update time for lcd
const unsigned long LCD_INTERVAL = 300;   // time delay between lcd updates
unsigned long lastLcdMs = 0;              // store last lcd update time

// values for ultrasonic sensor
const float SOUND_SPEED_CM_PER_US = 0.0343f;  // sound speed in cm per microsecond
const unsigned long PULSE_TIMEOUT = 35000UL;  // max waiting time for echo pulse

// distance ranges for the tones
const int TH_LOW_MAX  = 23;   // 17–23 cm range
const int TH_MED_MAX  = 16;   // 11–16 cm range
const int TH_HIGH_MAX = 10;   // 10 cm or less

// make states for the program
enum State { SAFE, WARN_LOW, WARN_MED, WARN_HIGH };  // 4 levels of warning
State currentState = SAFE;                            // start with SAFE state

// variable for button interrupt
volatile int muteState = LOW;  // LOW = buzzer active, HIGH = mute

// function runs when button is pressed
void IRAM_ATTR interrupt() {
  muteState = !muteState;   // toggle mute on/off each button press

}

// read distance once from ultrasonic sensor
long readDistanceOnceCm() {
  digitalWrite(TRIG_PIN, LOW);      // make trigger low
  delayMicroseconds(3);             // short delay
  digitalWrite(TRIG_PIN, HIGH);     // send trigger pulse
  delayMicroseconds(12);            // wait a bit
  digitalWrite(TRIG_PIN, LOW);      // stop trigger pulse
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);  // read echo time
  if (dur == 0) return -1;          // if no signal, return -1
  return (long)((dur * SOUND_SPEED_CM_PER_US) / 2.0f + 0.5f);  // convert to cm

}

// read distance several times and average it
long readDistanceAvgCm(uint8_t samples = 5) {
  long sum = 0; int count = 0;           // make variables for sum and count
  for (uint8_t i = 0; i < samples; i++) { // loop a few times
    long d = readDistanceOnceCm();        // read one distance
    if (d >= 0) { sum += d; count++; }    // add valid readings
   delay(5);                             // short delay between readings
  }
  return (count == 0) ? -1 : (sum / count);  // return average distance
}

void setup() {
  Serial.begin(115200);        // start serial monitor
  delay(100);                  // short delay

  // lcd start and message
  Wire.begin(LCD_SDA, LCD_SCL);    // start I2C for lcd
  lcd.init();                      // initialize lcd
  lcd.backlight();                 // turn on lcd light
  lcd.clear();                     // clear screen
  lcd.setCursor(0,0);              // set to first row
  lcd.print("Car Warning Sys");    // show title
  lcd.setCursor(0,1);              // go to second row
  lcd.print("Init...");            // print initializing
  delay(700);                      // wait a bit
  lcd.clear();                     // clear screen

  // start bme280 sensor
  I2C_BME.begin(BME_SDA, BME_SCL);           // start I2C for bme280
  bool ok = bme.begin(0x76, &I2C_BME);       // try address 0x76
  if (!ok) ok = bme.begin(0x77, &I2C_BME);   // if not, try 0x77
  if (!ok) Serial.println("BME280 NOT FOUND");  // print if not connected

  // setup pins
  pinMode(TRIG_PIN, OUTPUT);   // trigger pin as output
  pinMode(ECHO_PIN, INPUT);    // echo pin as input
  pinMode(BUZZER_PIN, OUTPUT); // buzzer as output
  pinMode(BTN_MUTE, INPUT_PULLUP); // button with pull-up resistor

  // connect button interrupt
  attachInterrupt(digitalPinToInterrupt(BTN_MUTE), interrupt, FALLING);  // run interrupt() when button pressed
  Serial.println("Setup done.");   // show setup complete

}

void loop() {
  // read distance and temperature
  long distance = readDistanceAvgCm(5);  // get average distance
  float tempC   = bme.readTemperature(); // read temperature
  
  // choose state based on distance
  State newState;
  if (distance < 0 || distance > TH_LOW_MAX) newState = SAFE;      // too far
  else if (distance <= TH_HIGH_MAX)          newState = WARN_HIGH;  // very close
  else if (distance <= TH_MED_MAX)           newState = WARN_MED;   // medium
  else                                       newState = WARN_LOW;   // low warning
  if (newState != currentState) currentState = newState;   // update state

  // control buzzer sound
  if (muteState == HIGH) {         // if mute is on
    noTone(BUZZER_PIN);            // stop buzzer
  } else {                         // if mute is off
    switch (currentState) {        // check which state we are in
      case WARN_HIGH: tone(BUZZER_PIN, 2000); break;  //  high tone
      case WARN_MED:  tone(BUZZER_PIN, 1000); break;  //  mid tone
      case WARN_LOW:  tone(BUZZER_PIN, 600);  break;  //  low tone
      case SAFE:      noTone(BUZZER_PIN);     break;  // safe no sound
    }
  }

  // update lcd every 300 ms
  unsigned long now = millis();                  // get current time
  if (now - lastLcdMs >= LCD_INTERVAL) {         // if time passed
    lastLcdMs = now;                             // save new time
    lcd.clear();                                 // clear lcd
    lcd.setCursor(0,0);                          // first line
    if (distance < 0) lcd.print("Dist: --- cm "); // show  if no reading
    else {
      lcd.print("Dist:"); lcd.print(distance); lcd.print("cm "); // show distance
    }
    lcd.setCursor(0,1);                          // second line
    lcd.print("Temp:");                          // show temperature
    lcd.print(tempC, 1);
    lcd.print("C ");
    lcd.print(muteState == HIGH ? "MUTE" : (     // show mute or state
      currentState==SAFE?"SAFE":
      currentState==WARN_LOW?"LOW":
      currentState==WARN_MED?"MED":"HIGH"
    ));
    // show same info in serial monitor
    Serial.print("Dist="); Serial.print(distance); Serial.print("cm ");
    Serial.print("State=");
    Serial.print(currentState==SAFE?"SAFE":
      currentState==WARN_LOW?"LOW":
      currentState==WARN_MED?"MED":"HIGH");
    Serial.print("  Mute=");
    Serial.println(muteState==HIGH?"ON":"OFF");
  }
}
