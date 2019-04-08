#ifndef Elevation_h
#define Elevation_h

#include "Arduino.h"

class Elevation
{
  public:
    Elevation(float p, float i, float d);
    float control_val(void);
    float get_ele_act(void);
    void set_ele_ref(float ref);
    float get_kp(void);
    float get_ki(void);
    float get_kd(void);
    void set_gains(float pid[]);
    void start_ele_sensor(void);
    bool verify(void);
    bool sens_ok(void);
    
  private:
    float kf_step(float Zk);
    float kp;
    float ki;
    float kd;
    float set_val;
    float servo_val;
    float tol_elev;
    float avg_angle_x;
    bool no_error;
    bool sensor_failure;

    // Kalman Filter Parameters
    float Xk_;
    float Pk_;
    float Q;
    float R;
};

#endif
