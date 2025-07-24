#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define SDA_PIN 4  // Replace with your chosen SDA pin
#define SCL_PIN 5  // Replace with your chosen SCL pin

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // put your setup code here, to run once:
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

  myIMU.begin();
  delay(1000);
  int8_t temp=myIMU.getTemp();
  myIMU.setExtCrystalUse(true);

}

void loop() {
  // put your main code here, to run repeatedly:
imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


Serial.print(acc.x());
Serial.print(",");
Serial.print(acc.y());
Serial.print(",");
Serial.println(acc.z());


delay(BNO055_SAMPLERATE_DELAY_MS);
}
