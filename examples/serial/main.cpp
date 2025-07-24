/*
  Este código inicializa la comunicación serial a 9600 baudios y envía un mensaje de bienvenida ("Hello Richy!") al iniciar.
  En el bucle principal (loop), declara una variable entera 'x' inicializada en 0 y entra en un ciclo while infinito (ya que x siempre es mayor o igual a 0).
  Dentro del ciclo, imprime por el puerto serial el mensaje "Hello World" seguido del valor actual de 'x' y un signo de exclamación.
  Después de cada impresión, espera 1 segundo (1000 ms) y luego incrementa 'x' en 1, repitiendo el proceso indefinidamente.
*/
#include <Arduino.h>

void setup(){
  Serial.begin(9600);
  delay(500);
  Serial.println("Hello Richy!");
  delay(500);
}

void loop(){
  int x = 0;
  while(x >= 0){
    Serial.print("Hello World ");
    Serial.print(x);
    Serial.println("!");
    delay(1000); 
    x++;
  }
}
