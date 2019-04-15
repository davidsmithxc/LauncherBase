// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <math.h>
#include "elevation.h"

Servo myservo;
Elevation elevation(0.0, 0.0, 0.0);

float p[3] = {0.0, 0.0, 0.0};
float dp[3] = {1.0, 1.0, 1.0};
float p_best[3] = {5.0, 1.0, 0};
unsigned long best_time = 10000;
unsigned long run_time;
float threshold = 0.75;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Start Elevation Tuner");

  myservo.attach(9);
  myservo.write(90);

  Serial.println("starting sensor");
  elevation.start_ele_sensor();
  
}

void loop() {
  Serial.println("begin twiddle");
  while ((dp[0] + dp[1] + dp[2]) > threshold)
  {
    Serial.println("New loop");
    Serial.print(dp[0]);
    Serial.print(" ");
    Serial.print(dp[1]);
    Serial.print(" ");
    Serial.println(dp[2]);
    Serial.print("Current sum dp: ");
    Serial.println((dp[0] + dp[1] + dp[2]));
    for(int i=0; i<3; i++)
    {
       p[i] += dp[i];
       run_time = gains_test(p);
       
       if (run_time < best_time)
       {
         // There was an improvement
         best_time = run_time;
         dp[i] *= 1.1;
         Serial.print("dp * 1.1: ");
         Serial.print(dp[0]);
         Serial.print(" ");
         Serial.print(dp[1]);
         Serial.print(" ");
         Serial.println(dp[2]);
         for(int j=0; j<3; j++)
         {
             p_best[j] = p[j];
         }
         Serial.print("New Best! ");
         Serial.println(best_time);
         Serial.print("\tkp: ");
         Serial.print(p[0]);
         Serial.print(" ");
         Serial.print("ki: ");
         Serial.print(p[1]);
         Serial.print(" ");
         Serial.print("kd: ");
         Serial.println(p[2]);         
       } else { 
         //# There was no improvement
         p[i] -= 2*dp[i];  //# Go into the other direction
         run_time = gains_test(p);
    
         if (run_time < best_time)
         {
           // There was improvement
           best_time = run_time;
           dp[i] *= 1.05;
           Serial.print("dp * 1.05: ");
           Serial.print(dp[0]);
           Serial.print(" ");
           Serial.print(dp[1]);
           Serial.print(" ");
           Serial.println(dp[2]);

           for(int j=0; j<3; j++)
           {
              p_best[j] = p[j];
           }
           Serial.print("New Best! ");
           Serial.println(best_time);
           Serial.print("\tkp: ");
           Serial.print(p[0]);
           Serial.print(" ");
           Serial.print("ki: ");
           Serial.print(p[1]);
           Serial.print(" ");
           Serial.print("kd: ");
           Serial.println(p[2]);
         } else {
           // There was no improvement
           p[i] += dp[i];
           // As there was no improvement, the step size in either
           // direction, the step size might simply be too big.
           dp[i] *= 0.95;
           Serial.print("dp * 0.95: ");
           Serial.print(dp[0]);
           Serial.print(" ");
           Serial.print(dp[1]);
           Serial.print(" ");
           Serial.println(dp[2]);

         }
       }  
    }
  }
  
  Serial.println("Final Results: ");
  Serial.print("Time: ");
  Serial.print(best_time);
  Serial.print("\tkp: ");
  Serial.print(p[0]);
  Serial.print(" ");
  Serial.print("ki: ");
  Serial.print(p[1]);
  Serial.print(" ");
  Serial.print("kd: ");
  Serial.println(p[2]);

  while(1){};
  
}

float gains_test(float gains[])
{
  int stable_count = 0;
  unsigned long t_start;
  unsigned long t_total;

  elevation.set_gains(gains);

  go_to_angle(20.0);

  t_start = millis();
  elevation.set_ele_ref(35.0);
  
  while((stable_count < 150) && ((millis() - t_start) < 10000))
  {
    safe_servo(elevation.control_val());
    if (elevation.verify())
    {
      stable_count += 1;
    } else {
      stable_count = 0;
    }
    delay(10);
  }

  t_total = millis() - t_start;
  Serial.print("Gains: ");
  Serial.print(gains[0]);
  Serial.print(" ");
  Serial.print(gains[1]);
  Serial.print(" ");
  Serial.print(gains[2]);
  Serial.print(" | ");
  Serial.println(t_total);
  return(t_total);
}

void go_to_angle(float ref_angle)
{
  float g[3] = {1.0, 2.5, 0.5};
  elevation.set_gains(g);
  int stable_count = 0;
  elevation.set_ele_ref(ref_angle);
  
  while(stable_count < 150)
  {
    safe_servo(elevation.control_val());
    if (elevation.verify())
    {
      stable_count += 1;
    } else {
      stable_count = 0;
    }
    delay(10);
  }
  
}

void safe_servo(float s_val)
{
  if(elevation.sens_ok())
  {
    myservo.write(s_val);
  } else {
    myservo.write(90);
    Serial.println("Sensor Failure!");
  }
}

