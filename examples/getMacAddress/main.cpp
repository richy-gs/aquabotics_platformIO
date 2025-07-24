// ---- Get the MAC address of the ESP32 board in station mode ---
// This code uses the esp_wifi.h library to read the MAC address of the ESP32 board
#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac( WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup(){
  Serial.begin(9600);
  Serial.print("Before WiFi.mode(WIFI_STA)");

  WiFi.mode(WIFI_STA);
  WiFi.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}
 
void loop(){
  readMacAddress();
  delay(5000); // Wait for 5 seconds before reading the MAC address again
  Serial.println("Reading MAC address again...");

}

