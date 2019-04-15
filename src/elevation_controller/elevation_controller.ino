// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <math.h>

// Used for software SPI
#define LIS3DH_CLK 13 // SCL
#define LIS3DH_MISO 11 // SDO
#define LIS3DH_MOSI 12 // SDA/SDI
// Used for hardware & software SPI
#define LIS3DH_CS 10 // CS

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
// Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

Servo myservo;
int servo_val = 90;

const int numReadings = 10;
int readIndex = 0;
float total = 0;
float average = 0;
float readings[numReadings];

int printIndex = 0;
const int CYCLE_s = 10;
const int PRINT = 1000 / CYCLE_s; // print once per second
const float TOL_ELEV = 1.0; // sensor tolerance limit; eg don't try beyond this accuracy
const bool PID_INFO = false;
const bool ACCEL_INFO = true;
const float G = 9.80665;
const float RAD2DEG = 360.0 / (2.0 * PI);

bool init_done = false;

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");

  myservo.attach(9);
  myservo.write(servo_val);
  
  // TODO: Change to while loop to retry accelerometer detection
  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

}

void loop() {
  /* Initialize some static variables ------------------ */
  static float elev_error = 0;
  static float elev_error_old = 0;
  static float elev_error_delta = 0;
  static float elev_error_sum = 0;

  static int servo_val_old = 90;
  
  static float kp = 20.0;
  static float ki = 0.0;
  static float kd = 0.0;

  static float set_val = 35.0;
  static float u_val = 0.0;

  /* ---------------------------------------------------- */
  
  
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

  /* do some lightweight filter of readings using rolling queue buffer */
  total = total - readings[readIndex];
  readings[readIndex] = angle_x;
  total = total + readings[readIndex];
  float avg_angle_x = round(10.0 * total / numReadings) / 10.0;
  avg_angle_x = angle_x;
  
  /* Increment read and print index; check if readings needs to be reset */
  readIndex += 1;
  printIndex += 1;
  
  if (readIndex >= numReadings){
    readIndex = 0;
  }

  /* Debug info for accelerometer */
  if (printIndex == PRINT && ACCEL_INFO){
    
    Serial.print("Y: ");
    Serial.print(raw_y);
    Serial.print(" m/s^2 | "); 
    Serial.print(angle_y); 
    Serial.print(" deg");
    
    Serial.print("\t\tX: ");
    Serial.print(raw_x);
    Serial.print(" m/s2^2 | ");
    Serial.print(angle_x);
    Serial.print(" deg | ");
    Serial.print(avg_angle_x);
    Serial.print(" deg (avg)");

    /*
    Serial.print("\t\tZ: ");
    Serial.print(raw_z);
    Serial.print(" m/s2^2 | ");
    Serial.print(angle_z);
    Serial.print(" deg");
    */

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
  if(abs(elev_error) <= TOL_ELEV){
    elev_error = 0;
    init_done = true;
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
  int servo_val = map(u_val, -100, 100, 0, 180);

  /* Deadzone hysteresis -----------------------------------------------------------------
   * if the controller is in the deadband (85-90) and the controller is trying to
   *  increase the control value, then bump to known drive value. Else the controller
   *  may be stuck in the deadzone.
   *  
   *  If the controller is ramping down the control value, do not limit to deadzone floor.
   */
   
  if((servo_val_old < 95 && servo_val_old > 90) && servo_val > 90){
    servo_val = 95;
  } else if((servo_val_old > 85 && servo_val_old < 90) && servo_val < 90) {
    servo_val = 85;
  }

  /* output control signal to drive servo */
  myservo.write(servo_val);

  /* store some values for use in next loop */
  servo_val_old = servo_val;
  elev_error_old = elev_error;

  
  /* print some debug info for the PID controller */
  if (printIndex == PRINT && PID_INFO){
    Serial.print("\tEp: "); Serial.print(elev_error);
    Serial.print("\tEi: "); Serial.print(elev_error_sum);
    Serial.print("\tEd: "); Serial.print(elev_error_delta);

    Serial.print("\tUval: "); Serial.print(u_val);
    Serial.print("\tSval: "); Serial.print(servo_val);

    Serial.print("\tErr: "); Serial.print(elev_error);

  }

  // add final check for resetting print index in loop without printing
  if (printIndex == PRINT) {
    printIndex = 0;
    Serial.println();
  }
    
 // Delay for cycle time
 // TODO: adjust for run time of loop to have actual match expected
  delay(CYCLE_s); 
}

void init_offset(int offset_val){
  sensors_event_t event; 
  lis.getEvent(&event);

  bool done = false;

  while(!done){
    float raw_y = event.acceleration.y;
    int angle_y =  int(raw_y / 9.81 * 90.0);
    done = (angle_y == offset_val);
    myservo.write(0);
  }

  myservo.write(90);
  
}

