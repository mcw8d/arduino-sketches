// ****************
// Import Libraries
// ****************

// Adafruit libraries for the LCD Shield.
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <Adafruit_MCP23017.h>

// DS18B20 temp sensor libraries.
#include <OneWire.h>
#include <DallasTemperature.h>

// *******
// Defines
// *******

// Pin connections for DS18B20 bus, power and ground.
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4

// Pin used to control SSR for the Cooling device.
#define CONTROL_PIN 7

// ********************
// Initialize Libraries
// ********************

// Init the Adafruit LCD library.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Init the DS18B20 libraries.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;

// *******
// Globals
// *******

boolean displayOn = true;
boolean running = false;
boolean cooling = false;

double setPoint = 70.0;
double currentTemp;

unsigned long lastButtonPress = 0;
const int maxIdle = 10000;
unsigned long lastLogTime = 0;
const int logInterval = 10000;

// *********************
// Custom LCD Characters
// *********************
byte power[8] =
{
  B00000,
  B00100,
  B01110,
  B10101,
  B10001,
  B01110,
  B00000,
  B00000
};
byte degree[8] =
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};
  
// *****
// Setup
// *****
void setup() {
  Serial.begin(9600);
  
  Serial.println(F("Starting the lcd..."));
  lcd.begin(16,2);
  lcd.createChar(1, power);
  lcd.createChar(2, degree);
  lcd.print(F("Initializing..."));
  
  Serial.println(F("Setting control pin to ground..."));
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
  
  Serial.println(F("Starting the Dallas Temp sensor..."));
  pinMode(ONE_WIRE_GND, OUTPUT);
  digitalWrite(ONE_WIRE_GND, LOW);
  pinMode(ONE_WIRE_PWR, OUTPUT);
  digitalWrite(ONE_WIRE_PWR, HIGH);
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Sensor Error"));
    delay(30000);
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
  
  lastButtonPress = millis();
  lastLogTime = millis();
  
  Serial.println(F("Building UI..."));
  lcd.clear();
  updateUISetPoint();
  updateUICurrent();
  
  // Setup timers
  Serial.println(F("Creating timer interrupt..."));
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 0x3D08;
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  if (running) {
    setOutput();
  }
}  

// *********
// Main Loop
// *********
void loop() {
  unsigned long now = millis();
  
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0) {
    lastButtonPress = now;
  }
 
  float increment = 1;
  if (buttons && !displayOn) {
    lcd.setBacklight(1);
    displayOn = true;
  }
  
  if (buttons & BUTTON_SELECT) {
    toggleRunning();
  }
  
  if (buttons & BUTTON_RIGHT) {
    increment *= 10;
  }
  
  if (buttons & BUTTON_LEFT) {
    increment /= 10;
  }
  
  if (buttons & BUTTON_UP) {
    setPoint += increment;
    updateUISetPoint();
    delay(200);
  }
  
  if (buttons & BUTTON_DOWN) {
    setPoint -= increment;
    updateUISetPoint();
    delay(200);
  }
  
  if (sensors.isConversionAvailable(0)) {
    currentTemp = sensors.getTempF(tempSensor);
    sensors.requestTemperatures();
    updateUICurrent();
  }
  
  if ((now - lastLogTime) > logInterval) {
    Serial.print(now);
    Serial.print(F(","));
    Serial.print(setPoint);
    Serial.print(F(","));
    Serial.print(currentTemp);
    Serial.print(F(","));
    Serial.println(cooling);
    lastLogTime = now;
  }
  if (displayOn && (now - lastButtonPress) > maxIdle) {
    lcd.setBacklight(0);
    displayOn = false;
  }     
}

void toggleRunning() {
  lcd.setCursor(15, 0);
  if (running) {
    lcd.print(F(" "));
  } else {
    lcd.write(1);
  }
  running = !running;
}

void toggleCooling() {
  lcd.setCursor(14, 0);
  if (cooling) {
    lcd.print(F(" "));
  } else {
    lcd.print(F("*"));
  }
  cooling = !cooling;
}

void updateUISetPoint() {
  lcd.setCursor(0, 0);
  lcd.print(F("Tgt: "));
  lcd.print(setPoint, 1);
  lcd.write(2);
  if (setPoint >= 100) {
    lcd.print(F("F"));
  } else {
    lcd.print(F("F "));
  }
}

void updateUICurrent() {
  lcd.setCursor(0, 1);
  lcd.setCursor(0,1);
  lcd.print(F("Cur: "));
  lcd.print(currentTemp, 1);
  lcd.write(2);
  if (currentTemp >= 100) {
    lcd.print(F("F"));
  } else {
    lcd.print(F("F "));
  }
}

// Going to experiment with a naive toggle if off by more than .5 degree
void setOutput() {
  if (!cooling && currentTemp > setPoint && currentTemp - setPoint > 0.5) {
    toggleCooling();
    digitalWrite(CONTROL_PIN, HIGH);
  } else if (cooling && currentTemp < setPoint) {
    toggleCooling();
    digitalWrite(CONTROL_PIN, LOW);
  }
}
