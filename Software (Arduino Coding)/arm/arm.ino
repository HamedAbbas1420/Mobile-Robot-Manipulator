/*
       Arduino Robot Arm and Mecanum Wheels Robot
          Smartphone Control via Bluetooth
       by Dejan, www.HowToMechatronics.com
*/
#include <SoftwareSerial.h>

#include <Servo.h>
Servo servo01;

SoftwareSerial Bluetooth(0, 1); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)


int servo1Pos; // current position
int servo1PPos; // previous position
int servo01SP[50]; // for storing positions/steps
int speedDelay = 20;
int index = 0;
int dataIn;
int m = 0;
void setup() {
  
  servo01.attach(5);
  pinMode(7,OUTPUT);

 Bluetooth.begin(9600); // blutooth communication 
  Serial.begin(9600); // serial communication started
  
  // Move robot arm to initial position
  servo1PPos = 90;
  servo01.write(servo1PPos);
  
}
void loop() {
  
  // Check for incoming data
  while (Bluetooth.available()){  //Check if there is an available byte to read
  delay(10); //Delay added to make thing stable 
  char c = Bluetooth.read(); //Conduct a serial read
  dataIn += c; //build the string- either "On" or "off"

    // Move robot arm
    // Move servo 1 in positive direction
    while (dataIn == "12") {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos++;
      delay(speedDelay);
    }
    // Move servo 1 in negative direction
    while (dataIn == "22") {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos--;
      delay(speedDelay);
    }
    //led
    while (dataIn == "1") {
      if (Bluetooth.available() > 0) {
        digitalWrite(7,HIGH);
      }
      while (dataIn == "2") {
      if (Bluetooth.available() > 0) {
        digitalWrite(7,LOW);
      }
    
    dataIn ="";
  }
}
  }}
  
