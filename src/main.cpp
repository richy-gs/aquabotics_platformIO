// #include <Arduino.h>

// // // #1 Motor Driver Connections
// // // // Motor A connections (No hay suficiente corriente)
// // // int enA = 15;
// // // int in1 = 16;
// // // int in2 = 17;
// // // // Motor B connections 
// // // int enB = 3;
// // // int in3 = 18;
// // // int in4 = 8;

// // // #2 Motor Driver Connections
// // // // Motor A connections (No hay suficiente corriente)
// // // int enA = 2;
// // // int in1 = 42;
// // // int in2 = 41;
// // // // Motor B connections
// // // int enB = 37;
// // // int in3 = 40;
// // // int in4 = 39;

// // // #3 Motor Driver Connections
// // // Motor A connections
// // // int enA = 14;
// // // int in1 = 12;
// // // int in2 = 13;
// // // // // Motor B connections
// // // int enB = 9;
// // // int in3 = 11;
// // // int in4 = 10;

// // int enMotorPins[6] = {4, 16, 9, 14, 2, 37};
// // int inMotorPins[6] = {5, 7, 10, 12, 42, 40};
// // int inMotorPins2[6] = {6, 15, 11, 13, 41, 39}; 

// int enA = 9;
// int in1 = 10;
// int in2 = 11;
// // // Motor B connections
// int enB = 14;
// int in3 = 12;
// int in4 = 13;

// void setup() {
//   // pinMode(38, OUTPUT);
//   // Set all the motor control pins to outputs
//   pinMode(enA, OUTPUT);
//   pinMode(enB, OUTPUT);
//   pinMode(in1, OUTPUT);
//   pinMode(in2, OUTPUT);
//   pinMode(in3, OUTPUT);
//   pinMode(in4, OUTPUT);

//   // Turn off motors - Initial state
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, LOW);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, LOW);
// }

// void loop() {
//   // digitalWrite(38, HIGH);
//   // // digitalWrite(1, HIGH);
//   // // Turn on motors
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);


//   analogWrite(enA, 255);
//   analogWrite(enB, 255);
//   delay(12000); // Wait before the next loop iteration

//   // Accelerate from zero to maximum speed
//   // for (int i = 150; i < 256; i++) {
//   //   analogWrite(enA, i);
//   //   analogWrite(enB, i);
//   //   delay(100);
//   // }

//   // Decelerate from maximum speed to zero
//   // for (int i = 255; i >= 150; --i) {
//   //   analogWrite(enA, i);
//   //   analogWrite(enB, i);
//   //   delay(100);
//   // }

//   // Now turn off motors
//   // digitalWrite(in1, LOW);
//   // digitalWrite(in2, LOW);
//   // digitalWrite(in3, LOW);
//   // digitalWrite(in4, LOW);
// }



#include <Arduino.h>

// int enMotorPins[6] = {4, 16, 9, 14, 2, 37};
// int inMotorPins[6] = {5, 7, 10, 12, 42, 40};
// int inMotorPins2[6] = {6, 15, 11, 13, 41, 39}; 

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
  for(int i = 0; i < 6; i++)   {
    digitalWrite(inMotorPins[i], HIGH);
    digitalWrite(inMotorPins2[i], LOW);
  }  

  for(int i = 0; i < 6; i++) {
    analogWrite(enMotorPins[i], 255);
  }
  // analogWrite(enMotorPins[2], 255);
  // analogWrite(enMotorPins[3], 255);
  delay(12000); // Wait before the next loop iteration

  // // Accelerate from zero to maximum speed
  // for (int i = 150; i < 256; i++) {
  //   for(int j = 0; j < 6; j++) {
  //     if(j%2 == 0) {
  //       analogWrite(enMotorPins[j], i);
  //     }
  //   }
  //   delay(100);
  // }

  // // Decelerate from maximum speed to zero
  // for (int i = 255; i >= 150; --i) {
  //   for(int j = 0; j < 6; j++) {
  //     if(j%2 == 0) {
  //       analogWrite(enMotorPins[j], i);
  //     }
  //   }
  //   delay(100);
  // }

  // Now turn off motors
  // for (int i = 0; i < 6; i++) {
  //   digitalWrite(inMotorPins[i], LOW);
  //   digitalWrite(inMotorPins2[i], LOW);
  // }
  // delay(6000); // Wait before the next loop iteration
}

