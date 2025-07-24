// El codigo de abajo permite por controlar el led integrgado en la tarjeta ESP32-S3-DevKitC-1
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel LED_RGB(1, 38, NEO_GRBW + NEO_KHZ800);

void setup() {
  LED_RGB.begin();
  LED_RGB.setBrightness(150);
}

void loop() {
  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 0, 0)));
  LED_RGB.show();
  delay(2000);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 255, 0)));
  LED_RGB.show();
  delay(2000);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 0, 255)));
  LED_RGB.show();
  delay(2000);
}
