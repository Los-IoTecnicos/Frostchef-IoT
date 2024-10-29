// Incluye la biblioteca para ESP32
#include <Arduino.h>

// Define el pin al que está conectado el sensor de proximidad
const int sensorPin = 2; // Cambia el número del pin según tu conexión

void setup() {
  // Inicializa el monitor serial
  Serial.begin(9600);

  // Configura el sensorPin como entrada
  pinMode(sensorPin, INPUT);
}

void loop() {
  // Lee el valor del sensor de proximidad
  int sensorValue = digitalRead(sensorPin);

  // Imprime el valor en el monitor serial
  Serial.print("Valor del sensor de proximidad: ");
  Serial.println(sensorValue);

  // Espera un breve período de tiempo antes de volver a leer
  delay(1000);
}
