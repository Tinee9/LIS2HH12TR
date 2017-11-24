#include <Tiny9_LIS2HH12.h>         // Tiny9 LI2HH12TR Library

LIS2HH12 lis = LIS2HH12();
int gravity = 0; //+/-2g = 0, +/- 4g = 2, +/- 8g = 3; 1 is not available
int x,y,z; 
float x1, y1, z1;
void setup() {
  // put your setup code here, to run once:
  lis.powerOn();
  //lis.scale(gravity);
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("Tiny9 LIS2HH12 Accelerometer Hook Up Guide Example");
  Serial.println();
  
}

void loop() {
  // put your main code here, to run repeatedly:
    
  
  lis.readAccel(&x, &y, &z); 
  scale();
  Serial.print(x1);
  Serial.print(", ");
  Serial.print(y1);
  Serial.print(", ");
  Serial.println(z1);
  //Serial.println(gravity<<4);
  delay(1000);
  
}

void scale(){
  if(gravity == 2){
      x1 = (float)x*0.000244;
      y1 = (float)y*0.000244;
      z1 = (float)z*0.000244;
  }
  else if(gravity == 1){
      x1 = (float)x*0.000122;
      y1 = (float)y*0.000122;
      z1 = (float)z*0.000122;
  }
  else{
      x1 = (float)x*0.000061;
      y1 = (float)y*0.000061;
      z1 = (float)z*0.000061;    
  }  
}

