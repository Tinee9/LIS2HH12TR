#include <Tiny9_LIS2HH12.h>         // Tiny9 LI2HH12TR Library

LIS2HH12 lis = LIS2HH12();
const int pwm_balancer =  5;// the pin number of the PWM command
const int redledPin =  4;// the number of the red LED pin
const int blueledPin =  3;// the number of the yellow LED pin
const int greenledPin =  2;// the number of the green LED pin
int gravity = 0; //+/-2g = 0, +/- 4g = 2, +/- 8g = 3; 1 is not available
int x,y,z; 
float diff = 0.00;
int major = 0;
int minor = 0;
int pwm = 123; // pwm can only go up to 0-255
int pwm_old = 0;
float x1, y1, z1;
void setup() {
  // put your setup code here, to run once:
  lis.powerOn();
  // set the digital pin as output:
  pinMode(redledPin, OUTPUT);
  pinMode(blueledPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
   pinMode(pwm_balancer, OUTPUT);
  //lis.scale(gravity);
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("Tiny9 LIS2HH12 earthquake Hook Up Guide Example");
  Serial.println();
  digitalWrite(redledPin, LOW);
  delay(1000);
  digitalWrite(redledPin, HIGH);  
  digitalWrite(blueledPin, LOW);
  delay(1000);
  digitalWrite(blueledPin, HIGH);
  digitalWrite(greenledPin, LOW);  
}

void loop() {
  // put your main code here, to run repeatedly:

  lis.readAccel(&x, &y, &z); 
  scale();
  Serial.print("x =");
  Serial.println(x1);
  Serial.print("y =");
  Serial.println(y1);
  Serial.print("z =");
  Serial.println(z1);
  pwm_old = pwm;
  if(x1 < 0){
    pwm = pwm_old - 1;
  }
  else if(x1> 0){
    pwm = pwm_old + 1;
  }
  if(pwm >255){
    pwm = 255;
    pwm_old = 255;
  }
  if(pwm < 0){
    pwm = 0;
    pwm_old = 0;
  }
  analogWrite(pwm_balancer, pwm);
  delay(50);  
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

