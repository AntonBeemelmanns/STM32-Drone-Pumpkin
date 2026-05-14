/**
 * Drone IMU Visualization
 * Receives filtered data about pitch, roll & yaw from STM32 via Serial and visualizes it in 3D.
 * * @author  Anton Beemelmanns 
 * @version 1.1
 * @date    2026-05-14
 */

import processing.serial.*;

// --- Global Objects ---
Serial myPort;
float imuPitch, imuRoll; 

// --- Yaw Integration ---
float visualYaw = 0;    // Calculated absolute yaw angle for visualization
float imuYawRate = 0;   // Current rotation speed from STM32 (Gyro Z)

void setup() {
  size(1200, 900, P3D); 
  
  // Serial Port Initialization
  // Check Serial Port for device, might have to adjust portName index
  String portName = Serial.list()[0]; 
  myPort = new Serial(this, portName, 115200);
  
  // Wait for a newline character before triggering serialEvent
  myPort.bufferUntil('\n'); 
}

void draw() {
  background(20);
  lights(); 

  // Scene
  translate(width/2, height/2, 0); 
  scale(1.8);                      

  // Yaw Integration: Add up received Yaw Rates to visualize current Yaw position
  if (abs(imuYawRate) > 0.3) { 
    visualYaw += imuYawRate * 0.02; 
  }

  // Apply rotations
  // Adjust sign (+/-) to position and config of IMU
  rotateY(radians(visualYaw));     
  rotateX(radians(imuPitch));      
  rotateZ(radians(-imuRoll));      

  drawDrone();
}

/**
 * Event Handler: triggered when data is available in the serial buffer
 */
void serialEvent(Serial p) {
  String inString = p.readString();
  if (inString != null) {
    inString = trim(inString); 
    
    // Expected format: "IMU:pitch,roll,yawRate"
    if (inString.startsWith("IMU:")) {
      try {
        float[] data = float(split(inString.substring(4), ','));
        
        if (data.length >= 3) {
           imuPitch   = data[0];
           imuRoll    = data[1];
           imuYawRate = data[2]; 
           
           println("PARSED -> Pitch: " + imuPitch + " | Roll: " + imuRoll + " | YawRate: " + imuYawRate);
        }
      } catch (Exception e) {
        println("Error parsing Serial string.");
      }
    }
  }
}

/**
 * Renders the drone
 */
void drawDrone() {
  // body
  stroke(255, 30);  
  fill(40);         
  box(60, 15, 60);  
  
  // Red marker for the front 
  fill(255, 0, 0);  
  pushMatrix();
  translate(0, -10, 35); 
  box(20, 5, 10);
  popMatrix();
  
  // Render four propeller arms and rotors
  fill(120); 
  color propColor = color(255, 255, 255, 120); 
  
  for (int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(QUARTER_PI + HALF_PI * i); 
    translate(70, 0, 0);
    box(100, 5, 10); 
    
    // Propeller discs
    translate(40, -5, 0);
    rotateX(HALF_PI); 
    noStroke();
    fill(propColor);
    ellipse(0, 0, 65, 65); 
    popMatrix();
  }
}
