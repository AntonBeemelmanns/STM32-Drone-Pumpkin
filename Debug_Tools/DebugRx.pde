/**
 * STM32 Serial Debugger
 * Simple utility to display incoming serial data from the Nucleo board 
 * in a dedicated Processing window.
 * * User: Anton Beemelmanns
 * @version 1.0
 */

import processing.serial.*;

// --- Global Objects ---
Serial myPort;      
String rawData = ""; 

void setup() {
  size(400, 200); 
  
  // List all available COM ports
  printArray(Serial.list());
  
  try {
    // Initialize serial connection at 115200 baud
    // Adjust index [i] to match your port
    myPort = new Serial(this, Serial.list()[0], 115200);
    
    // Trigger serialEvent() when a newline character ('\n') is received
    myPort.bufferUntil('\n'); 
    
    println("Successfully connected to COM port.");
  } catch (Exception e) {
    println("Error: Could not open COM port. Please check the index!");
  }
}

void draw() {
  background(25);    
  fill(0, 255, 0);   
  textSize(16);
  
  text("Status of STM32:", 20, 40);
  
  if (rawData != null) {
    text(rawData, 20, 80); 
  }
}

/**
 * Event Handler: triggered whenever a full line of data arrives.
 */
void serialEvent(Serial p) {
  rawData = p.readString(); 
  
  if (rawData != null) {
    rawData = trim(rawData); 
    println("Data Received: " + rawData); 
  }
}
