#include <Arduino.h>

// #1 Motor Driver Connections
// // Motor A connections (No hay suficiente corriente)
// int enA = 15;
// int in1 = 16;
// int in2 = 17;
// // Motor B connections 
// int enB = 3;
// int in3 = 18;
// int in4 = 8;

// #2 Motor Driver Connections
// // Motor A connections (No hay suficiente corriente)
// int enA = 2;
// int in1 = 42;
// int in2 = 41;
// // Motor B connections
// int enB = 37;
// int in3 = 40;
// int in4 = 39;

// #3 Motor Driver Connections
// Motor A connections
int enA = 14;
int in1 = 12;
int in2 = 13;
// // Motor B connections
int enB = 9;
int in3 = 11;
int in4 = 10;

void setup() {
  // pinMode(38, OUTPUT);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  // digitalWrite(38, HIGH);
  // // digitalWrite(1, HIGH);
  // // Turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Accelerate from zero to maximum speed
  for (int i = 150; i < 256; i++) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(100);
  }

  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 150; --i) {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(100);
  }

  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

