
#include <Arduino.h>
#include "BluetoothSerial.h"

#define RXD2 16
#define TXD2 17

BluetoothSerial SerialBT;

void setup() {
    Serial.begin(115200);

    // UART con Nucleo 1
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // Iniciar Bluetooth
    SerialBT.begin("Proyecto 3 ESP32"); // Nombre del dispositivo Bluetooth

    Serial.println("ESP32 listo");
    Serial.println("Bluetooth iniciado");
    Serial.println("Esperando comandos L o G...");
}

void loop() {

    // Recibir comandos desde Bluetooth
    if (SerialBT.available()) {

        char comando = SerialBT.read();

        if (comando == 'L' || comando == 'G') {

            // Enviar comando a Nucleo 1
            Serial2.write(comando);

            // Mostrar en monitor serial
            Serial.print("Enviado a Nucleo 1: ");
            Serial.println(comando);

            // Confirmar por Bluetooth
            SerialBT.print("Comando enviado: ");
            SerialBT.println(comando);
        }
    }

    delay(10);
}
