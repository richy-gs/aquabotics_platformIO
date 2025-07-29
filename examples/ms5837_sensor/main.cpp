#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;
#define SDA_PIN 4  // Replace with your chosen SDA pin
#define SCL_PIN 5  // Replace with your chosen SCL pin


void setup() {
  
  Serial.begin(115200);
  
  Serial.println("Starting");
  
  // Initialize I2C bus on your selected pins
  if (!Wire.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("Failed to initialize I2C!");
    while (1);
  }

  Serial.print("I2C initialized on SDA: ");
  Serial.print(SDA_PIN);
  Serial.print(", SCL: ");
  Serial.println(SCL_PIN);
  
  // Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  // Update pressure and temperature readings
  sensor.read();

  Serial.print("Pressure: "); 
  Serial.print(sensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(sensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(sensor.altitude()); 
  Serial.println(" m above mean sea level");

  delay(100);
}
