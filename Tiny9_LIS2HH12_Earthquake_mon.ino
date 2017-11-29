#include <Tiny9_LIS2HH12.h>         // Tiny9 LI2HH12TR Library

LIS2HH12 lis = LIS2HH12();
const int redledPin =  4;// the number of the red LED pin
const int yellowledPin =  3;// the number of the yellow LED pin
const int greenledPin =  2;// the number of the green LED pin
int gravity = 0; //+/-2g = 0, +/- 4g = 2, +/- 8g = 3; 1 is not available
int x,y,z; 
float diff = 0.00;
int major = 0;
int minor = 0;
float x1, y1, z1;
void setup() {
  // put your setup code here, to run once:
  lis.powerOn();
  // set the digital pin as output:
  pinMode(redledPin, OUTPUT);
  pinMode(yellowledPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
  //lis.scale(gravity);
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("Tiny9 LIS2HH12 earthquake Hook Up Guide Example");
  Serial.println();
  digitalWrite(redledPin, HIGH);
  delay(1000);
  digitalWrite(redledPin, LOW);  
  digitalWrite(yellowledPin, HIGH);
  delay(1000);
  digitalWrite(yellowledPin, LOW);
  digitalWrite(greenledPin, HIGH);  
}

void loop() {
  // put your main code here, to run repeatedly:

  lis.readAccel(&x, &y, &z); 
  scale();
  diff = abs(z1);
  if(major == 0){//major earthquake over 5.0
    if(diff >1.0){
        digitalWrite(redledPin, HIGH);
        major = 1;
    }
    if(minor == 0){//minor earthquake over 3.0
      if(diff >0.1){
        digitalWrite(yellowledPin, HIGH);
        minor = 1;
      }
    }    
  }
    Serial.print(diff);
    if(major == 1){
      Serial.println(" Major Earthquake");
    }
    else if(minor ==1){
      Serial.println(" Minor Earthquake");
    }
    else{
      Serial.println(" Micro/No Earthquake");
    }
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
      z1 = (float)z*0.000061-1.000;    
  }  
}

