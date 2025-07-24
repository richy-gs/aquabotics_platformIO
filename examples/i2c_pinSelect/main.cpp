#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 4  // Replace with your chosen SDA pin
#define SCL_PIN 5  // Replace with your chosen SCL pin

void setup() {
  Serial.begin(115200);

  // Initialize I2C bus on your selected pins
  if (!Wire.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("Failed to initialize I2C!");
    while (1);
  }

  Serial.print("I2C initialized on SDA: ");
  Serial.print(SDA_PIN);
  Serial.print(", SCL: ");
  Serial.println(SCL_PIN);
}

void loop() {
  // Your I2C-related code goes here
}
