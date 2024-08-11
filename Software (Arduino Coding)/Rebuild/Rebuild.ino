#include <SoftwareSerial.h> // TX RX software library for bluetooth
#include <Servo.h> // servo library 
Servo myservo1, myservo2, myservo3; // servo name

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

int LMP = 2;
int LMN = 3;
int RMP = 4;
int RMN = 7;

void setup()
{
  myservo1.attach(8); // attach servo signal wire to pin 9
  myservo2.attach(12);
  myservo3.attach(9);
  Serial.begin(9600);
  bluetooth.begin(9600);

  theta01=0;
  theta02=0;
  theta03=0;
  k2=(2*a2*a3);
 }

void loop()
{
  if(bluetooth.available()>= 2 )
  {
    unsigned int servopos = bluetooth.read();
    unsigned int servopos1 = bluetooth.read();
    unsigned int realservo = (servopos1 *256) + servopos;
    //Serial.println(realservo);

    //__________________________SLIDERS___________________________
    //servo1
    if (realservo >= 1000 && realservo <1360) {
      int servo1 = realservo;
      servo1 = servo1-1000;
      myservo1.write(servo1);
      delay(10);
    }
    //servo2
    if (realservo >= 2000 && realservo <2360) {
      int servo2 = realservo;
      servo2 = servo2-2000;
      myservo2.write(servo2);
      delay(10);
    }
    //servo3
    if (realservo >= 3000 && realservo <3360) {
      int servo3 = realservo;
      servo3 = servo3-3000;
      myservo3.write(servo3);
      Serial.println(servo3);
      delay(10);
    }
    //_____________(FORWARD KINEMATICS(Textboxes)__________________
    //servo1
    if (realservo >= 6000 && realservo <6180) {
      int servo1 = realservo;
      servo1 = servo1-6000;
      myservo1.write(servo1);
      delay(10);
    }
    //servo2
    if (realservo >= 7000 && realservo <7360) {
      int servo2 = realservo;
      servo2 = servo2-7000;
      myservo2.write(servo2);
      delay(10);
    }
    //servo3
    if (realservo >= 8000 && realservo <8360) {
      int servo3 = realservo;
      servo3 = servo3-8000;
      myservo3.write(servo3);
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
      //_______________INVERSE KINEMATICS_______________
    
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
  myservo1.write(theta_f1);
  myservo2.write(theta_f2new);
  myservo3.write(theta_f3);
  //Print kinematics equations
Serial.println("theta_f1");
Serial.println(theta_f1);
Serial.println("theta_f2");
Serial.println(theta_f2new);
Serial.println("theta_f3");
Serial.println(theta_f3);
Serial.println();
    }
    

//________________Diagnosis
//Print kinematics equations
/*Serial.println("theta_f1");
Serial.println(theta_f1);
Serial.println("theta_f2");
Serial.println(theta_f2new);
Serial.println("theta_f3");
Serial.println(theta_f3);
Serial.println();*/

//Print Bluetooth data
Serial.println("Bluetooth data:");
Serial.println(realservo);


  

  //______________________FORWARD KINEMATICS________________________
  /*Serial.println("X");
  Serial.println(5);
  Serial.println("Y");
  Serial.println(10);
  Serial.println("Z");
  Serial.println(15);*/

  //______________________MOBILE ROBOT_______________________
    if (realservo >= 12590 && realservo <12600)
    {
      //Move Forward
      digitalWrite(LMP, HIGH);
      digitalWrite(LMN, LOW);
      digitalWrite(RMP, LOW);
      digitalWrite(RMN, HIGH);
    }
    else if (realservo == 12849)
    {
      //Move Reverse
      digitalWrite(LMP, LOW);
      digitalWrite(LMN, HIGH);
      digitalWrite(RMP, HIGH);
      digitalWrite(RMN, LOW);
    }
    else if (realservo == 13617)
    {
      //Left Turn
      digitalWrite(LMP, LOW);
      digitalWrite(LMN, HIGH);
      digitalWrite(RMP, LOW);
      digitalWrite(RMN, HIGH);
    }
    else if (realservo == 13361)
    {
      //Right Turn
      digitalWrite(LMP, HIGH);
      digitalWrite(LMN, LOW);
      digitalWrite(RMP, HIGH);
      digitalWrite(RMN, LOW);
    }
    else if (realservo == 13105)
    {
      //Stop
      digitalWrite(LMP, LOW);
      digitalWrite(LMN, LOW);
      digitalWrite(RMP, LOW);
      digitalWrite(RMN, LOW);
    }
    realservo="";
  }
}
