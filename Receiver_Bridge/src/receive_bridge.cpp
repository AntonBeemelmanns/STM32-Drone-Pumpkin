/** 
 * Receiver Bridge 
 * Program written for custom device, mainly consisting of an Arduino Nano board (or ATmega328P clone)
 * and an NRF24L01+ module. The MCU receives wireless telemetry data from the Transmitter Bridge
 * and outputs it as a formatted CSV string via UART to the Flight Controller (STM32).
 * 
 * Hardware: Arduino Nano, NRF24L01+
 * Library: RF24 by TMRh20 (v1.4.7) - https://github.com/nRF24/RF24
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
 *   D1 (TX) -> Connected to STM32 UART RX (USART1)
 */



#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>



/**
 * DATA STRUCTURE:
 * Must match the Transmitter's struct exactly to ensure correct data alignment.
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
    
    delay(1000); 

    if (!radio.begin()) {
        Serial.println("Error: Could not find hardware!");
        while (1); 
    }

    // configure wireless connection
    radio.setChannel(100);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.openReadingPipe(1, address);  // open pipe for incoming signals
    radio.setAutoAck(false);
    radio.startListening();             // set as receiver

    Serial.println("System ready: Waiting for transmitter...");
}

void loop() {

    if (radio.available()) {

        // read received data into dataPacket struct
        radio.read(&dataPacket, sizeof(DataPacket));
        
        // output as CSV string via UART for the STM32
        Serial.print(dataPacket.pitch); Serial.print(",");
        Serial.print(dataPacket.roll);  Serial.print(",");
        Serial.print(dataPacket.yaw);   Serial.print(",");
        Serial.print(dataPacket.arm);   Serial.print(",");
        Serial.println(dataPacket.mode);
    }
}