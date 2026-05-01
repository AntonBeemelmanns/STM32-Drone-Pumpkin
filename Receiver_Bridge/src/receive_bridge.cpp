#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

struct DataPacket {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t arm;
    int16_t mode;
} dataPacket;

RF24 radio(9, 10);

const byte address[6] = "DRN01";

void setup() {
    Serial.begin(115200);
    // Warten, bis der Chip wirklich bereit ist
    delay(1000); 

    if (!radio.begin()) {
        Serial.println("FEHLER: Funkmodul nicht gefunden!");
        while (1); 
    }

    radio.setChannel(100);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.openReadingPipe(1, address);
    radio.setAutoAck(false);
    radio.startListening();

    Serial.println("SYSTEM_READY: Warte auf Sender...");
}

void loop() {
    if (radio.available()) {
        radio.read(&dataPacket, sizeof(DataPacket));
        
        // Sauberer CSV-Output für den STM32 und das Terminal
        Serial.print(dataPacket.pitch); Serial.print(",");
        Serial.print(dataPacket.roll);  Serial.print(",");
        Serial.print(dataPacket.yaw);   Serial.print(",");
        Serial.print(dataPacket.arm);   Serial.print(",");
        Serial.println(dataPacket.mode);
    }
}