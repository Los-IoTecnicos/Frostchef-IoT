// Includes for ESP32 Libraries
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MQUnifiedsensor.h>
#include <DHT.h>

// MQ2 Sensor Attributes
#define Board               ("ESP32")
#define Pin                 (4)
#define Threshold           (23)   // Gas threshold for unsafe condition
#define Type                ("MQ-2")
#define Voltage_Resolution  (3.3)
#define ADC_Bit_Resolution  (12)
#define RatioMQ2CleanAir    (9.83)

// DHT22 Sensor Setup
#define DHTPIN 5  // DHT sensor connected to GPIO 5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// MQ2 Library Instance Setup
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// Display Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// LEDs for System Status
const int greenLedPin = 2;   // Green LED for operational status
const int redLedPin = 15;    // Red LED for error indication

// Thresholds for Temperature and Humidity
const float tempThreshold = 10.0;   // Temperature threshold
const float humThreshold = 40.0;    // Humidity threshold

// Variables
int gas_value = 0;
float temperature = 0;
float humidity = 0;

void setup() {

  // LCD Screen Initialization
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.print("System Start");
  delay(1000);

  // Serial Port Communication Initialization
  Serial.begin(9600);

  // Parameter Setup for MQ2 to detect PPM concentration for LPG
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25);
  MQ2.setB(-2.222);

  // MQ2 Initialization
  MQ2.init();

  // DHT22 Initialization
  dht.begin();

  // Calibrate MQ2 Sensor
  Serial.print("Calibrating MQ2, please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 = MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calib MQ2:");
  lcd.print(calcR0);
  delay(500);

  // Calibration Exception Handling
  if (isinf(calcR0)) {
    Serial.print("Error: R0 is infinite. Check wiring");
    lcd.print("Error: R0 Inf");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.print("Error: R0 is zero. Check wiring");
    lcd.print("Error: R0 Zero");
    while (1);
  }

  // Set MQ2 Debug Log
  MQ2.serialDebug(true);

  // Configure LEDs
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // Start in Safe Condition
  digitalWrite(greenLedPin, HIGH); // Green LED ON to indicate system ready
  digitalWrite(redLedPin, LOW);    // Red LED OFF at start
}

void loop() {

  // DHT22 Sensor Readings
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Check if reading was successful
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT22 Sensor Error");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHT22 Error");
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  } else {
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print(" C, Hum: ");
    Serial.print(humidity);
    Serial.println(" %");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temperature);
    lcd.print("C H:");
    lcd.print(humidity);
    lcd.print("%");

    // Check temperature and humidity against thresholds
    if (temperature > tempThreshold || humidity > humThreshold) {
      lcd.setCursor(0, 1);
      lcd.print("Temp/Hum Alert!");
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, HIGH);
    } else {
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(redLedPin, LOW);
    }
  }

  // MQ2 Sensor Update and Readings
  MQ2.update();
  gas_value = analogRead(Pin);
  gas_value = map(gas_value, 0, 4095, 0, 100);

  Serial.print("Gas Level: ");
  Serial.print(gas_value);
  Serial.println(" %");

  lcd.setCursor(0, 1);
  lcd.print("Gas: ");
  lcd.print(gas_value);
  lcd.print("%");

  // Threshold Check for Unsafe Condition
  if (gas_value > Threshold) {
    lcd.setCursor(9, 1);
    lcd.print("Alert!");
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  } else {
    lcd.setCursor(9, 1);
    lcd.print("Safe ");
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  }

  // Wait before next read
  delay(2000);
}
