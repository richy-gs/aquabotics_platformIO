#include <esp_now.h>
#include <WiFi.h>

// Estructura del mensaje recibido (debe coincidir con la del emisor)
typedef struct struct_message {
  char msg[32];
} struct_message;

struct_message incomingMessage;

// Variables para el control del LED y el tiempo
const int ledPin = 1;              // LED en pin 1
unsigned long lastMessageTime = 0; // Último tiempo de mensaje recibido
const unsigned long connectionTimeout = 5000; // 5 segundos sin mensajes = sin conexión
unsigned long lastBlinkTime = 0;   // Para el parpadeo
const unsigned long blinkInterval = 500; // 0.5 segundos
bool ledState = LOW;

// Callback cuando se recibe un mensaje
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  Serial.print("Mensaje recibido: ");
  Serial.println(incomingMessage.msg);

  // Actualiza el tiempo del último mensaje recibido
  lastMessageTime = millis();
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

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
  unsigned long currentTime = millis();

  if ((currentTime - lastMessageTime) < connectionTimeout) {
    // Con "conexión": parpadeo fijo
    if ((currentTime - lastBlinkTime) >= blinkInterval) {
      ledState = !ledState; // Cambia el estado del LED
      digitalWrite(ledPin, ledState);
      lastBlinkTime = currentTime;
    }
  } else {
    // Sin "conexión": apaga el LED y reinicia el estado
    if (ledState != LOW) {
      ledState = LOW;
      digitalWrite(ledPin, LOW);
    }
  }
}
