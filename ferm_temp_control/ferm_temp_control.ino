#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <Adafruit_MCP23017.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
#define WHITE 0x7

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

float targetTempF = 68.0;

void setup() {
  Serial.begin(9600);
  
  Serial.println("Starting the lcd...");
  lcd.begin(16,2);
  lcd.setBacklight(WHITE);
  
  Serial.println("Starting the Dallas Temp sensor...");
  sensors.begin();
}

void loop() {
  Serial.print("Requesting temperature...");
  sensors.requestTemperatures();
  Serial.println("DONE");
  
  Serial.println("Attempting to print to LCD");
  lcd.setCursor(0, 0);
  lcd.print("Curr: ");
  lcd.print(sensors.getTempFByIndex(0));
  lcd.print(" F");
  lcd.setCursor(0, 1);
  lcd.print("Tgt: ");
  lcd.print(targetTempF);
  lcd.print(" F");
}
  

