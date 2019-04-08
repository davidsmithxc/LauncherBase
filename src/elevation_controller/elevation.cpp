// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <math.h>
#include "elevation.h"

// TODO: Create some sort of global defines
// Used for software SPI
#define LIS3DH_CLK 13 // SCL
#define LIS3DH_MISO 12 // SDO
#define LIS3DH_MOSI 11 // SDA/SDI
// Used for hardware & software SPI
#define LIS3DH_CS 10 // CS

#define G 9.80665
#define RAD2DEG (360.0 / (2.0 * PI))

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
// Adafruit_LIS3DH lis = Adafruit_LIS3DH();

Elevation::Elevation(float p, float i, float d) {
  // Set the member variables
  kp = p;
  ki = i;
  kd = d;

  set_val = 35.0;
  tol_elev = 0.25; // sensor tolerance limit; eg don't try beyond this accuracy
  avg_angle_x = 0.0;
  no_error = false;

  Xk_ = 35.0; // Initialize estimate to common target value
  Pk_ = 1.0; // Initialize covariance (err est)
  R = 0.125; // Measurement error (deg) for LIS3DH
  Q = 0.01; // Process noise

}

void Elevation::start_ele_sensor(void)
{
  // TODO: Add error codes here
  // TODO: Change to while loop to retry accelerometer detection
  while (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Reattempt sensor start");
    delay(500);
  }
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!  
}

float Elevation::control_val(void) {
  /* Initialize some static variables ------------------ */
  static float elev_error = 0;
  static float elev_error_old = 0;
  static float elev_error_delta = 0;
  static float elev_error_sum = 0;

  static int servo_val_old = 90;

  static float u_val = 0.0;

  static float readings[10];
  static float *ptr_readings = &readings[0];
  static int reading_cnt = 0;

  /* ---------------------------------------------------- */
  int servo_val = 90;
  
  
  /* Get a new, normalized sensor event */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* do the reading 
   *  (1) get the raw acceleration value
   *  (2) Convert m/s^2 to degrees
   */
  float raw_y = event.acceleration.y;
  float angle_y = asin(raw_y/G) * RAD2DEG;

  float raw_x = event.acceleration.x;
  float angle_x = asin(raw_x/G) * RAD2DEG;
  // Invert x-reading based on current mounting of accel in system
  angle_x = angle_x * -1;

  // Check if connection was loose -> Zeroes
  sensor_failure = ((angle_x == 0.0) && (angle_y == 0.0));
  if (sensor_failure) return 90.0;
  // Check if reading over shock limit -> nan
  if (isnan(angle_x)) return 90.0;

  /* Compensate for quandrant using y-axis information
   * The resulting angular reference frame will cover 0:180 and -180:0
   */
  if (angle_y > 0.0)
  {
    if (angle_x > 0.0)
    {
      angle_x = 180.0 - angle_x;
    } else {
      angle_x = -180.0 - angle_x;
    }
  }

  float raw_z = event.acceleration.z;
  float angle_z = asin(raw_x/G) * RAD2DEG;

  /* Apply 1D kalman filter */
  // #TODO: Add control singal U to state estimate
  avg_angle_x = kf_step(angle_x);

  if (reading_cnt < 10){
    reading_cnt += 1;
  } else {
    reading_cnt = 0;
  }
  
  /* PID Loop -------------------------------------------------------------- */
  
  /* Compute elevation error
   * Ep: Current error; set to zero if within tolerance value
   * Ei: Limited integrator, which only integrates in 
   */
  
  /* Sensor filter
   *  Create a deadband near zero to settle the controller.
   *  If the readings are withing +/- TOL_ELEV, then set errror to zero.
   */
   
  /* Ep */
  elev_error = avg_angle_x - set_val;
  if(abs(elev_error) <= tol_elev){
    elev_error = 0;
    no_error = true;
  } else {
    no_error = false;
  }
  
  /* Ed */
  elev_error_delta = elev_error_old - elev_error;

  /* Ei */
  if(elev_error == 0){
    elev_error_sum = 0;
  } else if (abs(servo_val - 90) < 5) {
    elev_error_sum += elev_error;
  }

  /* Integrator wind-up limit */
  elev_error_sum = min(500, elev_error_sum);
  elev_error_sum = max(-500, elev_error_sum);


  /* calculate control value
   * Limit control value to 100
   */
   
  u_val = (elev_error) * kp + (elev_error_sum) * ki + (elev_error_delta) * kd;
  u_val = min(100, u_val);
  u_val = max(-100, u_val);
  
  /* map output to sevo range */
  servo_val = map(u_val, -100, 100, 0, 180);

  /* Deadzone hysteresis -----------------------------------------------------------------
   * if the controller is in the deadband (85-90) and the controller is trying to
   *  increase the control value, then bump to known drive value. Else the controller
   *  may be stuck in the deadzone.
   *  
   *  If the controller is ramping down the control value, do not limit to deadzone floor.
   */

  /* 
  if((servo_val_old < 95 && servo_val_old > 90) && servo_val > 90){
    servo_val = 95;
  } else if((servo_val_old > 85 && servo_val_old < 90) && servo_val < 90) {
    servo_val = 85;
  }
  */

  /* store some values for use in next loop */
  servo_val_old = servo_val;
  elev_error_old = elev_error;

  /* output control signal to drive servo */
  return servo_val;
}

void Elevation::set_ele_ref(float ref)
{
  set_val = ref;
}

void Elevation::set_gains(float pid[])
{
  kp = pid[0];
  ki = pid[1];
  kd = pid[2];
}

float Elevation::get_kp(void)
{
  return kp;
}

float Elevation::get_ki(void)
{
  return ki;
}

float Elevation::get_kd(void)
{
  return kd;
}

float Elevation::get_ele_act(void)
{
  return avg_angle_x;
}

bool Elevation::verify(void)
{
  return no_error;
}

float Elevation::kf_step(float Zk)
{
  float Kk = Pk_ / (Pk_ + R);
  float Xk = Xk_ + Kk * (Zk - Xk_);
  float Pk = (1 - Kk) * Pk_ + fabs(Xk_ - Xk) * Q;

  static bool was_nan = false;

  Xk_ = Xk;
  Pk_ = Pk;

  if(isnan(Xk) && !was_nan){
    Serial.print("KF nan - Zk: ");
    Serial.print(Zk);
    Serial.print(" | Kk: ");
    Serial.print(Kk);
    Serial.print(" | Pk: ");
    Serial.println(Pk);

    was_nan = true;
  }

  return Xk;

}

bool Elevation::sens_ok(void)
{
  return !sensor_failure;
}

