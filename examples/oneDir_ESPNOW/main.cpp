// #include <esp_now.h>
// #include <WiFi.h>

// // Dirección MAC del receptor (ajústala si usas una dirección específica)
// uint8_t broadcastAddress[] = {0x8C, 0xBF, 0xEA, 0x03, 0xB8, 0xDC};

// // Variable para almacenar el estado de envío
// String success;

// // Estructura de mensaje (ahora solo para texto)
// typedef struct struct_message {
//   char msg[32];
// } struct_message;

// // Mensaje a enviar
// struct_message outgoingMessage;

// esp_now_peer_info_t peerInfo;

// // Callback cuando se envían datos
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//   success = (status == ESP_NOW_SEND_SUCCESS) ? "Delivery Success :)" : "Delivery Fail :(";
// }

// void setup() {
//   Serial.begin(115200);

//   // Configura el modo WiFi
//   WiFi.mode(WIFI_STA);

//   // Inicializa ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   esp_now_register_send_cb(OnDataSent);

//   // Registrar peer
//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial.println("Failed to add peer");
//     return;
//   }
// }

// void loop() {
//   static int i = 0;

//   // Construir mensaje
//   snprintf(outgoingMessage.msg, sizeof(outgoingMessage.msg), "Hello World [%d]!", i);

//   // Enviar mensaje por ESP-NOW
//   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));

//   // Reportar resultado
//   if (result == ESP_OK) {
//     Serial.print("Sent: ");
//     Serial.println(outgoingMessage.msg);
//   } else {
//     Serial.println("Error sending the data");
//   }

//   i++; // Incrementar el contador
//   delay(2000); // Esperar 2 segundos entre envíos
// }


// --- This code is for an ESP32 receiver that uses ESP-NOW to receive messages from another ESP32 device.
#include <esp_now.h>
#include <WiFi.h>

// Estructura del mensaje recibido (debe coincidir con la del emisor)
typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message incomingMessage;

// Callback cuando se recibe un mensaje
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  Serial.print("Mensaje recibido: ");
  Serial.println(incomingMessage.msg);
}

void setup() {
  Serial.begin(115200);

  // Configurar Wi-Fi en modo estación
  WiFi.mode(WIFI_STA);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    return;
  }

  // Registrar la función de recepción
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receptor listo y esperando mensajes...");
}

void loop() {
  // No se necesita nada aquí para el receptor
}
