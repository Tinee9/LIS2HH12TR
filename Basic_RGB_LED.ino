const int redledPin =  4;// the number of the red LED pin
const int blueledPin =  3;// the number of the yellow LED pin
const int greenledPin =  2;// the number of the green LED pin
void setup() {
  // put your setup code here, to run once:
  pinMode(redledPin, OUTPUT);
  //pinMode(yellowledPin, OUTPUT);
  //pinMode(greenledPin, OUTPUT);
}

void loop() {
  int i = 0;
  // put your main code here, to run repeatedly:
  for( i=100; i<150; i++){
  analogWrite(redledPin, i );
  delay(500);
  }
}
