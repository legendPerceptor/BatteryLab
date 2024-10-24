#include <Servo.h> 

Servo myservo0;
Servo myservo1;

// Initialize the pump and valve objects
// Servo myservo0 is the pump
// Servo myservo1 is the valve

void setup() 
{ 
  myservo0.attach(9);
  myservo1.attach(8);

  myservo0.write(0);  
  myservo1.write(0);  

  // Serial communication setup
  Serial.begin(9600);
} 

void loop() 
{
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'P') {
      pick();
    } else if (command == 'D') {
      drop();
    } else if (command == 'O') {
      off();
    } else if (command == 'C') {
      continue_pick();  
    }
  }
}

void continue_pick() {
  myservo0.write(180);   
  myservo1.write(0);
}

void off() {
  myservo0.write(0);  
  myservo1.write(0);
}

void pick() {
  myservo0.write(180);   
  myservo1.write(0);
  delay(800);
  myservo0.write(0);  
  myservo1.write(0);
}

void drop() {
  myservo0.write(0);   
  myservo1.write(180);
  delay(800);
  myservo0.write(0);  
  myservo1.write(0);
}