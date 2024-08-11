//In this tutorial you will be creating an app for controlling a servo motor, you will be using slider in your app to move your servo from 0-180, You need a servo motor to be connected on arduino side, make sure you also connecting external powersupply so that your Arduino will not restart during this process,




#include <SoftwareSerial.h> // TX RX software library for bluetooth

#include <Servo.h> // servo library 
Servo myservo1; // servo name
Servo myservo2;
Servo myservo3;

int bluetoothTx = 10; // bluetooth tx to 10 pin
int bluetoothRx = 11; // bluetooth rx to 11 pin

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{
  myservo1.attach(5); // attach servo signal wire to pin 9
  myservo2.attach(6);
  myservo3.attach(9);
  //Setup usb serial connection to computer
  Serial.begin(9600);

  //Setup Bluetooth serial connection to android
  bluetooth.begin(9600);
}

void loop()
{
  //Read from bluetooth and write to usb serial
  if(bluetooth.available()> 0 ) // receive number from bluetooth
  {
    int servopos1 = bluetooth.read(); // save the received number to servopos
    Serial.println(servopos1); // serial print servopos current number received from bluetooth
    myservo1.write(servopos1); // roate the servo the angle received from the android app

    int servopos2 = bluetooth.read(); // save the received number to servopos
    Serial.println(servopos2); // serial print servopos current number received from bluetooth
    myservo2.write(servopos2); // roate the servo the angle received from the android app

    int servopos3 = bluetooth.read(); // save the received number to servopos
    Serial.println(servopos3); // serial print servopos current number received from bluetooth
    myservo3.write(servopos3); // roate the servo the angle received from the android app
  }


}
