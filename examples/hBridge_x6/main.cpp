#include <Arduino.h>

int enMotorPins[6] = {15, 3, 2, 37, 14, 9};
int inMotorPins[6] = {16, 18, 42, 40, 12, 11};
int inMotorPins2[6] = {17, 8, 41, 39, 13, 10}; 

// 2
// 14
// 15
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
  for(int i = 0; i < 6; i++) {
    if(i % 2 == 0) {
      digitalWrite(inMotorPins[i], HIGH); // Set even indexed pins HIGH
    } else {
      digitalWrite(inMotorPins2[i], LOW); // Set odd indexed pins LOW
    }
  }  

  // Accelerate from zero to maximum speed
  for (int i = 150; i < 256; i++) {
    for(int j = 0; j < 6; j++) {
      if(j%2 == 0) {
        analogWrite(enMotorPins[j], i);
      }
    }
    delay(100);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 150; --i) {
    for(int j = 0; j < 6; j++) {
      if(j%2 == 0) {
        analogWrite(enMotorPins[j], i);
      }
    }
    delay(100);
  }

  // Now turn off motors
  for (int i = 0; i < 6; i++) {
    digitalWrite(inMotorPins[i], LOW);
    digitalWrite(inMotorPins2[i], LOW);
  }
  delay(6000); // Wait before the next loop iteration
}

