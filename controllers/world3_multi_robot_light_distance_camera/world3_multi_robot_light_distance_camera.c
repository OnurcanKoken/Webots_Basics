/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Description:  An example of use of a camera, a distance sensor, a light sensor devices 
 with recognition, light detection and object avoidance capabilities.
*/

// here only 1 robot is used to scan the robots around itself!
// robot spins and checks the number of recognized robots
// estimates the swarm size

#include <stdio.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <time.h>

#define MAX_SPEED 10
#define SPEED 6.2
#define TIME_STEP 64

int main(){
  WbDeviceTag camera, ls0, ls1, left_motor, right_motor, ds_left, ds_right;

  int total_objects_ever = 0;
  int number_of_objects = 0;
  int prev_number_of_objects = 0;
  int scan_swarm = 0; // if 1, scan 180
  int current_time; // to calculate waiting time
  int t_scan;
  
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

  // Get the camera device, enable it and the recognition 
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control). 
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* Set the motors speed */
  wb_motor_set_velocity(left_motor, -SPEED);
  wb_motor_set_velocity(right_motor, SPEED);
  scan_swarm = 1;
  
  // Main loop 
  int scan_swarm_first_time = 1;
  int swarm_size;
  while (wb_robot_step(TIME_STEP) != -1) {
    // Get current number of object recognized 
    
    if(scan_swarm ==1 && scan_swarm_first_time ==1){
      t_scan = clock();
      scan_swarm_first_time = 0;
      swarm_size = total_objects_ever;
    }

    if(scan_swarm == 1){
      current_time = clock() - t_scan; // check how long it has been
      double time_taken = ((double)current_time)/CLOCKS_PER_SEC; // in seconds
      wb_motor_set_velocity(left_motor, -SPEED);
      wb_motor_set_velocity(right_motor, SPEED);
    
      if(time_taken > 2){
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        scan_swarm = 1;
        scan_swarm_first_time = 1;
        printf("robot name: %s, swarm size: %d, time: %f", wb_robot_get_name(), swarm_size, (((double)(clock()))/CLOCKS_PER_SEC));
        total_objects_ever = 0;
        number_of_objects = 0;
        prev_number_of_objects = 0;
        swarm_size = 0;
      }
    }
    
    prev_number_of_objects = number_of_objects;
    number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    
    if(prev_number_of_objects < number_of_objects){
      total_objects_ever = total_objects_ever + (number_of_objects - prev_number_of_objects);
      if(scan_swarm == 1){
        swarm_size = swarm_size + (number_of_objects - prev_number_of_objects);
      }
    }
    
    printf("\nTotal Recognized %d robots.\n", total_objects_ever);

    // read light sensor values
    const double ls0_value = wb_light_sensor_get_value(ls0);
    const double ls1_value = wb_light_sensor_get_value(ls1);

    // read distance sensor values
//    double ds_left_val = wb_distance_sensor_get_value(ds_left);
//    double ds_right_val = wb_distance_sensor_get_value(ds_right);
    

    double left_speed = (1024 - ls0_value) / 100.0;
    left_speed = (left_speed < MAX_SPEED) ? left_speed : MAX_SPEED;
    double right_speed = (1024 - ls1_value) / 100.0;
    right_speed = (right_speed < MAX_SPEED) ? right_speed : MAX_SPEED;
  }
  wb_robot_cleanup();
  return 0;
}