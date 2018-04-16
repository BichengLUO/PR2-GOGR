/*
 * File:          pr2_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/device.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16
#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

WbDeviceTag kinectColor;
WbDeviceTag kinectRange;
WbDeviceTag head_tilt_motor;
WbDeviceTag head_tilt_sensor;

// Simpler step function
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

void initialize_devices() {
  kinectColor = wb_robot_get_device("kinect color");
  kinectRange = wb_robot_get_device("kinect range");
  wb_camera_enable(kinectColor, TIME_STEP);
  wb_range_finder_enable(kinectRange, TIME_STEP);
  
  head_tilt_motor = wb_robot_get_device("head_tilt_joint");
  head_tilt_sensor = wb_robot_get_device("head_tilt_joint_sensor");
  wb_position_sensor_enable(head_tilt_sensor, TIME_STEP);
}

void set_head_tilt(double tilt, bool wait_on_feedback) {
  wb_motor_set_position(head_tilt_motor, tilt);

  if (wait_on_feedback) {
    while (! ALMOST_EQUAL(wb_position_sensor_get_value(head_tilt_sensor), tilt))
      step();
  }
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  initialize_devices();
  set_head_tilt(M_PI_4, true);
  while (wb_robot_step(TIME_STEP) != -1) {

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
