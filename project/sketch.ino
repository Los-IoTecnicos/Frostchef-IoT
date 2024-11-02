// Incluye la biblioteca para ESP32
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MQUnifiedsensor.h>

#include "mq2-setup.h"

// MQ2 Sensor Attributes
#define Board               ("ESP32")
#define Pin                 (4)
#define Threshold           (23)
#define Type                ("MQ-2")
#define Voltage_Resolution  (3.3)
#define ADC_Bit_Resolution  (12)
#define RatioMQ2CleanAir    (9.83)

// MQ2 Library Instance Setup
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// Display Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);



// Define el pin al que está conectado el sensor de proximidad
const int sensorPin = 2; // Cambia el número del pin según tu conexión

// Variables
int gas_value = 0;



void setup() {

  // LCD Screen Initialization
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.print("On Line");
  delay(1000);

  // Serial Port Communication Initialization
  Serial.begin(9600);
  // Parameter Setup to detect PPM concentration for LPG
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25);
  MQ2.setB(-2.222);

  // MQ2 Initialization
  MQ2.init();

  // Calibrate Sensor
  Serial.print("Calibrating Sensor, please wait.");
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 = MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }

  // Update Calibration Factor
  MQ2.setR0(calcR0/10);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ2 = ");
  lcd.print(calcR0);
  delay(500);

  // Calibration Exception Handling
  if(isinf(calcR0)) { Serial.print("Warning: R0 value is infinite. Please check your wiring"); while(1); }
  if(calcR0 == 0) { Serial.print("Warning: R0 value is zero. Please check your wiring"); while(1); }

  // Show MQ2 Debug Log
  MQ2.serialDebug(true);

  // Configura el sensorPin como entrada
  //pinMode(sensorPin, INPUT);
  pinMode(Threshold, INPUT);
}

void loop() {

  // Update Reading
  MQ2.update();
  MQ2.readSensor();
  MQ2.serialDebug();

  // Read Values
  gas_value = analogRead(Pin);
  gas_value = map(gas_value, 0, 4095, 0, 100);
  Serial.println(gas_value);

  // Show value in display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ2 = ");
  lcd.print(gas_value);
  lcd.print("%");

  // Show Threshold Indicator
  lcd.setCursor(0, 1);
  if (digitalRead(Threshold) == HIGH) {
    lcd.print("Unsafe");
  } else {
    lcd.print("Safe");
  }

  // Lee el valor del sensor de proximidad
  int sensorValue = digitalRead(sensorPin);

  // Imprime el valor en el monitor serial
  Serial.print("Valor del sensor de proximidad: ");
  Serial.println(sensorValue);

  // Espera un breve período de tiempo antes de volver a leer
  delay(2000);
}
