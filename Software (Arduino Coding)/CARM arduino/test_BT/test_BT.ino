#include <SoftwareSerial.h>
int bluetoothRx = 0; // bluetooth rx to 0 pin
int bluetoothTx = 1; // bluetooth tx to 1 pin
SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

}

void loop() {
  if(bluetooth.available()>= 2 )
  {
    unsigned int servopos = bluetooth.read();
    unsigned int servopos1 = bluetooth.read();
    unsigned int realservo = (servopos1 *256) + servopos;

    Serial.println("Bluetooth data:");
    Serial.println(realservo);
    realservo="";
  }

}
