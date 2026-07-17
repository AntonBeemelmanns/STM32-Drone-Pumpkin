/**
 * Drone Ground Control Station (GCS)
 * Visualizes remote control inputs in a 3D environment and passes control string to serial port.
 * Uses GameControlPlus for HID interfacing and P3D for rendering.
 * @author  Anton Beemelmanns
 * @version 1.4
 * @date    2026-07-15
 */

import org.gamecontrolplus.*;
import processing.serial.*;

// --- Global Objects ---
Serial myPort;
ControlIO control;
ControlDevice device;

// --- Control Variables ---
float pitch, roll, yaw, throttle;
float threshold = 0.2;   // Deadzone for joystick to prevent stickdrift influence
boolean killSwitch = false; // Safety switch to kill motors until restart

/**
 * Initializes serial connection, controller hardware and graphics
 */
void setup(){
    // Initialize window for animation
  size(1200, 900, P3D);
  
  // get COM-Port and initialize 
  String[] ports = Serial.list();
  
  if(ports.length > 0){
    try{
      String portName = Serial.list()[0];
      myPort = new Serial(this, portName, 115200);
      println("Serial Port " + portName + " opened successfully.");
    }
    catch(Exception e){
      println("Com-Port is blocked or not available. No data is sent through port.");
      myPort = null;
    }
  }
  else{
    println("No device found at port. No data is sent through port.");
    myPort = null;
  }
  
  // Map controller using external configuaration file
  control = ControlIO.getInstance(this);
  device = control.getMatchedDevice("ps4_config.json");
  
  if (device == null) {
    println("ps4_config.json not found or unknown device.");
    exit();
  } 
  println("Setup successfull.");
}

/**
 * Main Rendering Loop (60 FPS)
 */
void draw(){
  // visualize background
  if (killSwitch){
    background(100, 0, 0); 
  } 
  else{
    background(20);
  }
   
  lights();       
  
  // Get inputs
  float rawP = device.getSlider("pitch").getValue();
  float rawR = device.getSlider("roll").getValue();
  float rawY = device.getSlider("yaw").getValue();
  float rawT = device.getSlider("throttle").getValue();
  
  // Apply deadzone and scaling (max tilt is 45 degrees)
  if(abs(rawP) > threshold){
    pitch = rawP * 45; 
  }
  else{
    pitch = 0;
  }
     
  if(abs(rawR) > threshold){
    roll = rawR * 45;
  }
  else{
    roll = 0;
  }
  
  if (abs(rawY) > threshold) {
    yaw = rawY * 100; 
  }
  else{
    yaw = 0;
  }
  
  // map throttle to PWM and constrain
  throttle = map(rawT, -1.0, 1.0, 1200, 1650);
  
  if (throttle < 1200) throttle = 1200;
  if (throttle > 1650) throttle = 1650;
  
  renderAnimation();
  
  // Send data at every second frame
  if(frameCount % 2 == 0){
    sendData();
  }
}

/**
 * Renders the graphics of scene and drone animation
 */
void renderAnimation(){
  // Render the scene
  translate(width/2, height/2, 0); 
  scale(1.8); 

  // Apply rotations
  rotateY(radians(-yaw) + PI); 
  rotateX(radians(pitch));
  rotateZ(radians(-roll));
  
  // Render the drone body
  stroke(255, 30); 
  fill(40);        
  box(60, 15, 60); 
  
  fill(255, 0, 0); 
  pushMatrix();
  translate(0, -10, 35); 
  box(20, 5, 10);
  popMatrix();
  
  // Render the drone propellers
  fill(120); 
  color propColor = color(255, 255, 255, 120); 
  
  for (int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(QUARTER_PI + HALF_PI * i); 
    translate(70, 0, 0);
    box(100, 5, 10);
    translate(40, -5, 0);
    rotateX(HALF_PI); 
    noStroke();
    fill(propColor);
    ellipse(0, 0, 65, 65); 
    popMatrix();
  }
}

/**
 * Sends controller inputs to serial port
 */
void sendData() {
    // read killswitch
    if(device.getButton("kill_switch").pressed()){
      killSwitch = true;
      println("!!! EMERGENCY KILL ALL MOTORS !!!");
    }
    
    // execute killswitch
    if(killSwitch && myPort != null){
      myPort.write("100,100,100,100,100\n");      
      return;
    }
    
    // read arm-button and mode-switch
    int arm = 0;
    
    if(device.getButton("arm_button").pressed()){
      arm = 1;
    }
    
    // dont send data to port if no device connected
    if(myPort == null){
      return;
    }
    
    // Output string to serial port
    String dataString = str(int(pitch)) + "," + str(int(roll)) + "," + str(int(yaw)) + "," + str(arm) + "," + str(int(throttle)) + "\n";
    println(dataString);
    myPort.write(dataString);
}
