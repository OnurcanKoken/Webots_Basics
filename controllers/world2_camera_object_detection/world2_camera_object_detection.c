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
 * Description:  An example of use of a camera device with recognition capability.
 */

#include <stdio.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/distance_sensor.h>
#define MAX_SPEED 6.28

#define SPEED 1.5
#define TIME_STEP 64

#include <time.h>

int main() {

  WbDeviceTag camera;
  wb_robot_init();

  /* Get the camera device, enable it and the recognition */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  
  /* get a handler to the motors and set target position to infinity (speed control). */

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // internal variables
  WbDeviceTag ds_left = wb_robot_get_device("ds0");
  WbDeviceTag ds_right = wb_robot_get_device("ds1");
  
  // initialise devices
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);
  
  // Set the motors speed 
  // wb_motor_set_velocity(left_motor, -SPEED);
  // wb_motor_set_velocity(right_motor, SPEED);

  // Main loop 
  while (wb_robot_step(TIME_STEP) != -1) {
  
    double object_size = 0;
    double largest_object = 0;
    
    // initialize motor speeds at 50% of MAX_SPEED.
    double left_speed = 0.5 * MAX_SPEED;
    double right_speed = 0.5 * MAX_SPEED;
    
    // Get current number of object recognized 
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    printf("\nRecognized %d objects.\n", number_of_objects);

    // Get and display all the objects information 
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    
    for (int i = 0; i < number_of_objects; ++i) {
      printf("Model of object %d: %s\n", i, objects[i].model);
      printf("Id of object %d: %d\n", i, objects[i].id);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
             objects[i].orientation[2], objects[i].orientation[3]);
      printf("Size of object %d: %lf %lf\n", i, objects[i].size[0], objects[i].size[1]);
      printf("Position of the object %d on the camera image: %d %d\n", i, objects[i].position_on_image[0],
             objects[i].position_on_image[1]);
      printf("Size of the object %d on the camera image: %d %d\n", i, objects[i].size_on_image[0], objects[i].size_on_image[1]);
      
      object_size = objects[i].size_on_image[0] * objects[i].size_on_image[1];
      
      if (object_size >= largest_object){
        largest_object = object_size;
        }
    }

    // write actuators inputs
    wb_motor_set_velocity(left_motor, -left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

  } // end of while 
  
  // cleanup the Webots API
  wb_robot_cleanup();
  
  return 0; // EXIT
}