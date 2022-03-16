// 16.03.2022

#include <stdio.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 10
#define SPEED 6
#define TIME_STEP 64

// function to keep doing smt for a while in seconds
void wait(float duration){
  float start_time = wb_robot_get_time();
  do{ wb_robot_step(TIME_STEP);
  } while (wb_robot_get_time() < start_time + duration);
}

int main() {
  // initialize variables
  WbDeviceTag ls0, ls1, left_motor, right_motor, ds_left, ds_right;
  wb_robot_init();

  // get a handler to the light sensors.
  ls0 = wb_robot_get_device("ls0"); // left
  ls1 = wb_robot_get_device("ls1"); // right

  wb_light_sensor_enable(ls0, TIME_STEP);
  wb_light_sensor_enable(ls1, TIME_STEP);
  
  // initialize distance sensor
  ds_left = wb_robot_get_device("ds0_l");
  ds_right = wb_robot_get_device("ds1_r");
  
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);
  
  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != 1) {
    // read light sensor values
    const double ls0_value = wb_light_sensor_get_value(ls0);
    const double ls1_value = wb_light_sensor_get_value(ls1);
    // read distance sensor values
    double ds_left_val = wb_distance_sensor_get_value(ds_left);
    double ds_right_val = wb_distance_sensor_get_value(ds_right);
    
    // go through the light source
    double left_speed = (1024 - ls0_value) / 100.0;
    left_speed = (left_speed < MAX_SPEED) ? left_speed : MAX_SPEED;
    double right_speed = (1024 - ls1_value) / 100.0;
    right_speed = (right_speed < MAX_SPEED) ? right_speed : MAX_SPEED;
    printf("\nleft speed: %f", left_speed);
    
    // object avoidance
    if(ds_left_val < 800){
      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, -1*right_speed);
      wait(0.5);
      }
    else if(ds_right_val < 800){
      wb_motor_set_velocity(left_motor, -1*left_speed);
      wb_motor_set_velocity(right_motor, right_speed);
      wait(0.5);
      }
    else{
      // set the motor speeds with respect to light intensity
      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);
      }   
  }
  wb_robot_cleanup();
  return 0;
}
