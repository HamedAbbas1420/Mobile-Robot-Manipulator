#include <VarSpeedServo.h>
VarSpeedServo servo1;
void setup() {
  servo1.attach(9);
  //servo1.write(0,255,true);
  //delay(2000);

}

void loop() {
  servo1.write(0,20);
 
  

}
