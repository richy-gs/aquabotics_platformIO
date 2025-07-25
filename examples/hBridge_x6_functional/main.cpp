#include <Arduino.h>

int enMotorPins[6] = {15, 3, 2, 37, 14, 9};
int inMotorPins[6] = {16, 18, 42, 40, 12, 11};
int inMotorPins2[6] = {17, 8, 41, 39, 13, 10}; 

void setup() {
  // // Set all the motor control pins to outputs
  for (int i = 0; i < 6; i++) {
    pinMode(enMotorPins[i], OUTPUT);
    pinMode(inMotorPins[i], OUTPUT);
    pinMode(inMotorPins2[i], OUTPUT);
  }

  // Turn off motors - Initial state
  for (int i = 0; i < 6; i++) {
    digitalWrite(inMotorPins[i], LOW);
    digitalWrite(inMotorPins2[i], LOW);
  }
}

void loop() {
  // Example of how to control the motors
  for(int i = 0; i < 6; i++)   {
    digitalWrite(inMotorPins[i], HIGH);
    digitalWrite(inMotorPins2[i], LOW);
  }  

  for(int i = 0; i < 6; i++) {
    analogWrite(enMotorPins[i], 155);
    delay(3000); // Wait before the next loop iteration
    analogWrite(enMotorPins[i], 0);
    digitalWrite(inMotorPins[i], LOW);
    digitalWrite(inMotorPins2[i], LOW);
  }

  delay(8500); // Wait before the next loop iteration
}

