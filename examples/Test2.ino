#include <VarSpeedServo.h> 
int motorPin = 2;

VarSpeedServo myservo1; 
VarSpeedServo myservo2;
VarSpeedServo myservo3;
VarSpeedServo myservo4;
VarSpeedServo myservo5;

void setup() {
  pinMode(motorPin, OUTPUT);
  myservo1.attach(11);
 myservo2.attach(10); 
 myservo3.attach(6); 
 myservo4.attach(5); 
 myservo5.attach(3);  // attaches the servo on pin 9 to the servo object 
} 

void loop() {
  myservo1.write(150, 30, true);
myservo2.write(140, 30, true);
myservo4.write(160, 30, true);
myservo3.write(160, 30, true);

digitalWrite(motorPin, HIGH);
delay(500);
 myservo3.write(90, 30, true);
myservo2.write(90, 30, true);
myservo1.write(70, 30, true);
 myservo3.write(140, 30, true);
myservo5.write(180, 30, true);
delay(500);
digitalWrite(motorPin, LOW);
delay(500);
  myservo3.write(90, 30, true);       
}
