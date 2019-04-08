#include <Servo.h>
#include "./src/elevation_controller/elevation.h"

Servo elevation_servo;
Elevation elevation(2.0, 0.1, 0.0);

unsigned long t_elevation;
int print_cnt = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing launcher base controls");

  elevation_servo.attach(9);
  elevation_servo.write(90);

  Serial.println("starting elevation sensor");
  elevation.start_ele_sensor();

  elevation.set_ele_ref(35.0);

}

void loop() {
  // put your main code here, to run repeatedly:
  t_elevation = millis();
  float ctrl = elevation.control_val();
  safe_servo(ctrl);
  delay(7);

  print_cnt += 1;
  if (print_cnt > 100){
    Serial.print(ctrl);
    Serial.print(" | ");
    Serial.print(elevation.get_ele_act());
    Serial.print(" | ");
    Serial.println(isnan(elevation.get_ele_act()));
    print_cnt = 0;
  }

}

/*
 * A helper function to stop actuation if the sensor gets unplugged
 * TODO: Make generic somehow?
 */

void safe_servo(float s_val)
{
  if(elevation.sens_ok())
  {
    elevation_servo.write(s_val);
  } else {
    elevation_servo.write(90);
    Serial.println("Sensor Failure!");
  }
}
