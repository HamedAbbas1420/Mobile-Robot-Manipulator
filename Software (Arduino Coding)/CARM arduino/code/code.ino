#include <SoftwareSerial.h> // TX RX software library for bluetooth
#include <VarSpeedServo.h>  // servo speed library
VarSpeedServo myservo1, myservo2, myservo3, myservo4;

#define bluetoothRx  0 // bluetooth rx to 0 pin
#define bluetoothTx  1 // bluetooth tx to 1 pin
SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

float Px=10;
float Py=5;
float a1=20;
float a2=17.5;
float servoSpeed=0;
float dcSpeed=0;
long duration, cm;
double q2,q1,k1,k2,t,tf,k11,Pxo;
double theta_f2,theta_f1, theta1,theta2,theta01,theta02,theta03,theta1new,theta2new,theta3new,theta_f2new;

#define R1 2
#define R2 3
#define L1 4
#define L2  7
#define speedA 5
#define speedB 6
#define trigPin  10
#define echoPin 11
#define relay  13
int range = 5;//range in inches

void setup()
{
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(L1,OUTPUT);
  pinMode(speedA,OUTPUT);
  pinMode(speedB,OUTPUT);
  pinMode(L2,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo1.attach(8); //Shoulder
  myservo2.attach(9); //Elbow
  myservo3.attach(12); //Gripper
  myservo1.write(0,50);
  myservo2.write(40,50);
  myservo3.write(40,50);
  Serial.begin(9600);
  bluetooth.begin(9600);

  theta01=0;
  theta02=0;
  theta03=0;
 }

void loop()
{
  if(bluetooth.available()>= 2 )
  {
        //Scan
    
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm= duration*0.034/2;
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
    unsigned int servopos = bluetooth.read();
    unsigned int servopos1 = bluetooth.read();
    unsigned int realservo = (servopos1 *256) + servopos;
    //Serial.println(realservo);

    //Servo Motors Power (Relay)
    if (realservo == 5) {
      digitalWrite(relay,HIGH);
    }
    if (realservo == 6) {
      digitalWrite(relay,LOW);
    }
    //_____________//Read Servo motors Speed
    if (realservo >= 6500.0 && realservo <6756.0) {
      servoSpeed = realservo - 6500.0;
      //Serial.println("Py=");
      //Serial.println(Py);
      delay(10);
    }

    //__________________________SLIDERS___________________________
    //servo1 Base
    if (realservo >= 1000 && realservo <1360) {
      int servo1 = realservo;
      servo1 = servo1-1090;
      myservo1.write(servo1,servoSpeed);
      delay(10);
    }
    //servo2 Shoulder
    if (realservo >= 1500 && realservo <1860) {
      int servo2 = realservo;
      servo2 = servo2-1680;
      myservo2.write(servo2,servoSpeed);
      delay(10);
    }
    //servo3 Elbow
    if (realservo >= 2000 && realservo <2360) {
      int servo3 = realservo;
      servo3 = servo3-2000;
      myservo3.write(servo3,servoSpeed);
      //Serial.println(servo3);
      delay(10);
    }
    
    //servo4 Gripper
    if (realservo >= 2500 && realservo <2860) {
      int servo4 = realservo;
      servo4 = servo4-2500;
      myservo4.write(servo4,servoSpeed);
      //Serial.println(servo4);
      delay(10);
    }
    //_____________(FORWARD KINEMATICS(Textboxes)__________________
    //servo1 Base
    if (realservo >= 3500 && realservo <3860) {
      int servo1 = realservo;
      servo1 = servo1-3500;
      myservo1.write(servo1,servoSpeed);
      delay(10);
    }
    //servo2 Shoulder
    if (realservo >= 4000 && realservo <4360) {
      int servo2 = realservo;
      servo2 = servo2-4000;
      myservo2.write(servo2,servoSpeed);
      delay(10);
    }
    //servo3 Elbow
    if (realservo >= 4500 && realservo <4860) {
      int servo3 = realservo;
      servo3 = servo3-4500;
      myservo3.write(servo3,servoSpeed);
      delay(10);
    }
    //servo4 Gripper
    if (realservo >= 5000 && realservo <5360) {
      int servo4 = realservo;
      servo4 = servo4-5000;
      myservo4.write(servo4,servoSpeed);
      delay(10);
    }
    
    //______________Kinematics Equations__________________
    //Ultrsonic independent Mode
      //Read Px
      if (realservo >= 6000.0 && realservo <6100.0) {
      Px = realservo - 6000.0;
      //Serial.println("Px=");
      //Serial.println(Px);
      delay(10);
    }
    //Read Py
    if (realservo >= 6100.0 && realservo <6200.0) {
      Py = realservo - 6100.0;
      //Serial.println("Py=");
      //Serial.println(Py);
      delay(10);
    }
    //Ultrsonic Dependent Mode
    //Read Py
    if (realservo >= 6200.0 && realservo <6300.0) {
      Py = realservo - 6200.0;
      //Serial.println("Py=");
      //Serial.println(Py);
      delay(10);
    }
      //_______________INVERSE KINEMATICS_______________
      //Inverse kinematics
      if (realservo == 16){
      q2 = (Px*Px + Py*Py - a1*a1 - a2*a2)/(2*a1*a2);
      theta_f2 = acos(q2);
      k1 = a1 + a2 * cos(theta_f2*3.14/180);
      k2 = a2*sin(theta_f2*3.14/180);
      q1 = (k1*Py - k2*Px)/(k1*k1 + k2*k2);
      theta_f1 = asin(q1);
      myservo1.write(theta_f1,servoSpeed);
      myservo2.write(theta_f2,servoSpeed);
      }
      //Ultrasonic dependent Inverse kinematics
      Pxo = cm + 12;
      if (realservo == 17){
      q2 = (Pxo*Pxo + Py*Py - a1*a1 - a2*a2)/(2*a1*a2);
      theta_f2 = acos(q2);
      k1 = a1 + a2 * cos(theta_f2*3.14/180);
      k2 = a2*sin(theta_f2*3.14/180);
      q1 = (k1*Py - k2*Pxo)/(k1*k1 + k2*k2);
      theta_f1 = asin(q1);
      myservo1.write(theta_f1,servoSpeed);
      myservo2.write(theta_f2,servoSpeed);
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

  //______________________MOBILE ROBOT_______________________
    // Read DC Speed value 
    if (realservo >= 7000 && realservo <7255) {
      dcSpeed = realservo;
    }
    //Move Forward
    if (realservo == 11)
    {
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);
      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
    }
    //Move Reverse
    else if (realservo == 13)
    {
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
    }
    //Turn Left
    else if (realservo == 15)
    {
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);
      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
    }
    //Turn Right
    else if (realservo == 14)
    { 
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
    }
    //Stop
    else if (realservo == 12)
    {
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
    }

    //____________________Ultrasonic____________________

  //Right manipulation
    if (realservo == 9) { 
      if(cm < 20){
        //stop
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
    }
      else{
      //Turn Right
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, HIGH);
      digitalWrite(R2, LOW);
      digitalWrite(L1, HIGH);
      digitalWrite(L2, LOW);
    }}
    //Left manipulation
    if (realservo == 10) { 
      if(cm < 20){
        //stop
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      digitalWrite(L1, LOW);
      digitalWrite(L2, LOW);
    }
      else{
      //Turn Left
      analogWrite(speedA,dcSpeed);
      analogWrite(speedB,dcSpeed);
      digitalWrite(R1, LOW);
      digitalWrite(R2, HIGH);
      digitalWrite(L1, LOW);
      digitalWrite(L2, HIGH);
    }}
    //____________________Line Follower_________________
    //Receive Mode Date
    /*if (realservo == 7) { //Active Mode
      irState= digitalRead(irSensor);
      if (irState == HIGH){
        analogWrite(speedA,dcSpeed);
        analogWrite(speedB,dcSpeed);
        digitalWrite(R1, LOW);
        digitalWrite(R2, HIGH);
        digitalWrite(L1, HIGH);
        digitalWrite(L2, LOW);
      }else{
        analogWrite(speedA,dcSpeed);
        analogWrite(speedB,dcSpeed);
        digitalWrite(R1, LOW);
        digitalWrite(R2, HIGH);
        digitalWrite(L1, LOW);
        digitalWrite(L2, HIGH);
      }
    }
    if (realservo == 8) { //Inactive Mode
        digitalWrite(R1, LOW);
        digitalWrite(R2, LOW);
        digitalWrite(L1, LOW);
        digitalWrite(L2, LOW);
    }*/

    //Delete Previous data to receive the new coming data 
    realservo="";
  }
}
