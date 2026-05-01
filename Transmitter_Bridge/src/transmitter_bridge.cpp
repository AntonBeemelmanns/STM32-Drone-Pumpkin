/** 
 * Transmitter Bridge 
 * Program written for custom device, mainly consisting of an Arduino Nano board (or ATmega328P clone)
 * and an NRF24L01+ module. The MCU receives a formatted data string from a Ground Control Station (GCS)
 * via UART (USB), parses the telemetry values, and transmits them wirelessly to the drone.
 * 
 * Hardware: Arduino Nano, NRF24L01+
 * Library: RF24 by TMRh20 (v1.4.x or later) - https://github.com/nRF24/RF24
 * 
 * @author    Anton Beemelmanns
 * @version   1.2
 * @date      2026-05-01
 * @see       https://github.com/nRF24/RF24 for NRF24L01 driver implementation
 */



/* 
 * HARDWARE CONNECTIONS:
 * ---------------------
 * nRF24L01 Module:
 *   VCC    -> 3.3V (with 10uF Capacitor recommended)
 *   GND    -> GND
 *   CE     -> D9
 *   CSN    -> D10
 *   SCK    -> D13
 *   MOSI   -> D11
 *   MISO   -> D12
 * 
 * UART Interface:
 *   USB (Serial) -> Connected to PC (Ground Control Station)
 */



#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>



/**
 * DATA STRUCTURE:
 * The DataPacket struct defines the over-the-air payload.
 * Total size: 10 Bytes (5 x 16-bit integers).
 */
struct DataPacket {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t arm;
    int16_t mode;
} dataPacket;

// configure hardware: CE to pin 9, CSN to pin 10
RF24 radio(9, 10);

// channel name
const byte address[6] = "DRN01";

void setup() {

    Serial.begin(115200);
    
    while(!radio.begin()) {
        Serial.println("Hardware not found.");
        delay(500); 
    }

    // configure wireless connection
    radio.setChannel(100);
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_1MBPS);
    radio.setAutoAck(false);
    radio.stopListening();

    Serial.println("Setup successfull. Starting loop.");
}

void loop() {

    if(Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');

        int ctrlVals[5];
        int count = 0;

        // convert input to char array and tokenize 
        char* str = (char*)input.c_str();
        char* ptr = strtok(str, ",");

        // convert values to type int
        while (ptr != NULL && count < 5) {
            ctrlVals[count++] = atoi(ptr); 
            ptr = strtok(NULL, ",");
        }

        // write values to dataPacket and send to drone
        if (count == 5) {
            dataPacket.pitch = (int16_t)ctrlVals[0];
            dataPacket.roll  = (int16_t)ctrlVals[1];
            dataPacket.yaw   = (int16_t)ctrlVals[2];
            dataPacket.arm   = (int16_t)ctrlVals[3];
            dataPacket.mode  = (int16_t)ctrlVals[4];

            
            radio.write(&dataPacket, sizeof(DataPacket)); 
            Serial.println("Data sent to drone!");
        }
    }    
}