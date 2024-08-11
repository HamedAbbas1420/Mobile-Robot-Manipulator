#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); //TX, RX respectively

Servo motor_1;
int servo1 = 90;
String device;
                
void setup() 
{
   BT.begin(9600);
  Serial.begin(9600);
  motor_1.attach(9);
  motor_1.write(servo1);         
        delay(10); 
}

void loop()
{
  while (BT.available()){  //Check if there is an available byte to read
  delay(10); //Delay added to make thing stable
  char c = BT.read(); //Conduct a serial read
  device += c; //build the string.
  }   
  if (device.length() > 0) {
    Serial.println(device);  
    //CW
    if(device == '1'){
      if(servo1<180){servo1 = servo1+1;}
      motor_1.write(servo1); }  


    //CCW
    else if(device == '2'){
      if(servo1>0){servo1 = servo1-1;} 
      motor_1.write(servo1);}
  device="";}
}
