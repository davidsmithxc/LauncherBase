#include <Servo.h>

Servo myservo;
int servo_val = 0;

int max_speed = 100;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
  myservo.write(servo_val);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  /*
  for (servo_val = 80; servo_val <= max_speed; servo_val += 1){
    myservo.write(servo_val);
    Serial.println(servo_val);
    delay(1500);
  }
  for (servo_val = 100; servo_val >= max_speed; servo_val -= 1){
    myservo.write(servo_val);
    Serial.println(servo_val);
    delay(1500);
  }
  */
  myservo.write(120);

}
