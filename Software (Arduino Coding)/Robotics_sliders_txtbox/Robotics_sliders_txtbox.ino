#include <SoftwareSerial.h> // TX RX software library for bluetooth
#include <Servo.h> // servo library 
Servo myservo1, myservo2, myservo3, myservo4; // servo name

int bluetoothTx = 10; // bluetooth tx to 10 pin
int bluetoothRx = 11; // bluetooth rx to 11 pin
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

int Px=10;
int Py=5;
int Pz=15;
int a2=20;
int a3=10;
double q3,q2,q1,k1,k2,t,tf,k11;
double theta_f3,theta_f2,theta_f1, theta1,theta2,theta3,theta01,theta02,theta03,theta1new,theta2new,theta3new,theta_f2new;

void setup()
{
  myservo1.attach(5); // attach servo signal wire to pin 9
  myservo2.attach(6);
  myservo3.attach(9);
  //Setup usb serial connection to computer
  Serial.begin(9600);

  theta01=0;
  theta02=0;
  theta03=0;
  k2=(2*a2*a3);
 

  //Setup Bluetooth serial connection to android
  bluetooth.begin(9600);
}

void loop()
{
  //Read from bluetooth and write to usb serial
  if(bluetooth.available()>= 2 )
  {
    unsigned int servopos = bluetooth.read();
    unsigned int servopos1 = bluetooth.read();
    unsigned int realservo = (servopos1 *256) + servopos;
    //Serial.println(realservo);
//________________sliders_____________________
    //servo1
    if (realservo >= 1000 && realservo <1360) {
      int servo1 = realservo;
      servo1 = realservo-1000;
      myservo1.write(servo1);
      delay(10);
    }
    //servo2
    if (realservo >= 2000 && realservo <2360) {
      int servo2 = realservo;
      servo2 = realservo-2000;
      myservo2.write(servo2);
      delay(10);
      //servo3
    }
    if (realservo >= 3000 && realservo <3360) {
      int servo3 = realservo;
      servo3 = realservo-3000;
      myservo3.write(servo3);
      delay(10);
    }/*
    if (realservo >= 4000 && realservo <4180) {
      int servo4 = realservo;
      servo4 = map(servo4, 4000, 4180, 0, 180);
      myservo4.write(servo4);
      Serial.println("Servo 4 ON");
      delay(10);
    }*/
//_____________Textboxes__________________
    //servo1
    if (realservo >= 6000 && realservo <6180) {
      int servo1 = realservo-10;
      //servo1 = map(servo1, 6000, 6180, 0, 180);
      myservo1.write(servo1);
      //Serial.println("Servo 1 ON");
      delay(10);
    }
    //servo2
    if (realservo >= 7000 && realservo <7180) {
      int servo2 = realservo;
      //servo2 = map(servo2, 7000, 7180, 0, 180);
      myservo2.write(servo2);
      //Serial.println("Servo 2 ON");
      delay(10);
      //servo3
    }
    if (realservo >= 8000 && realservo <8180) {
      int servo3 = realservo;
      servo3 = map(servo3, 8000, 8180, 0, 180);
      myservo3.write(servo3);
      //Serial.println("Servo 3 ON");
      delay(10);
    }
    //______________Kinematics Equations__________________
    
      //Read Px
      if (realservo >= 100 && realservo <130) {
      Px = realservo - 100.0;
      Serial.println("Px=");
      Serial.println(Px);
      delay(10);
    }
    //Read Py
    if (realservo >= 200 && realservo <230) {
      Py = realservo - 200.0;
      Serial.println("Py=");
      Serial.println(Py);
      delay(10);
      //Read Pz
    }
    if (realservo >= 300 && realservo <330) {
      Pz = realservo - 300.0;
      Serial.println("Pz=");
      Serial.println(Pz);
      delay(10);
      realservo="";
    }
    //Inverse Kinematics
    
    q3= (sq(Px)+sq(Py)+sq(Pz)-sq(a2)-sq(a3))/k2;;
 theta_f3=acos(q3)*180/3.14;                       //theta3
 k1=a2+a3*cos(theta_f3*3.14/180);
 q2=(sqrt(sq(a3)*sq(sin(theta_f3*3.14/180))*(sq(Px)+sq(Py)))-Pz*k1)/(sq(k1)+sq(a3)*sq(sin(theta_f3*3.14/180)));
 theta_f2=asin(q2)*180/3.14;                       //theta2
 q1=Px/(a2*cos(theta_f2*3.14/180)+a3*cos(theta_f2*3.14/180)*cos(theta_f3*3.14/180)-a3*sin(theta_f2*3.14/180)*sin(theta_f3*3.14/180));
 theta_f1=acos(q1)*180/3.14; //theta1

 if(theta_f2>=0){
  theta_f2new=theta_f2;
 }else{theta_f2new=360+theta_f2;}
/*Serial.println("k2");
Serial.println(k2);
Serial.println("q3");
Serial.println(q3);
Serial.println("q2");
Serial.println(q2);
Serial.println("q1");
Serial.println(q1);
Serial.println();
*/
/*Serial.println("theta_f1");
Serial.println(theta_f1);
Serial.println("theta_f2");
Serial.println(theta_f2new);
Serial.println("theta_f3");
Serial.println(theta_f3);
Serial.println();
    Serial.println("Bluetooth data:");
Serial.println(realservo);*/

  myservo1.write(theta_f1);
  myservo2.write(theta_f2new);
  myservo3.write(theta_f3+150);

    //realservo="";
  }
}
