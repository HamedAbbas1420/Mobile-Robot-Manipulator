
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
int Px=5;
int Py=10;
int Pz=20;

int a2=20;
int a3=10;

double q3,q2,q1,k1,k2,t,tf,k11;
double theta_f3,theta_f2,theta_f1, theta1,theta2,theta3,theta01,theta02,theta03,theta_f2new;

void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);
  theta01=0;
  theta02=0;
  theta03=0;
  tf=3;
  k2=(2*a2*a3);
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

Serial.println("k2");
Serial.println(k2);
Serial.println("q3");
Serial.println(q3);
Serial.println("q2");
Serial.println(q2);
Serial.println("q1");
Serial.println(q1);
Serial.println();

Serial.println("theta_f1");
Serial.println(theta_f1);
Serial.println("theta_f2");
Serial.println(theta_f2new);
Serial.println("theta_f3");
Serial.println(theta_f3);
Serial.println();
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  
  
}

void loop() {
  servo1.write(theta_f1);
  servo2.write(theta_f2new);
  servo3.write(theta_f3+150);
  /*delay(2000);
  servo1.write(0);
  servo2.write(0*-1);
  servo3.write(0+150);
  delay(2000);*/
}
