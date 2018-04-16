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

// helper constants to distinguish the motors
enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };
enum { SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL };
enum { LEFT_FINGER, RIGHT_FINGER, LEFT_TIP, RIGHT_TIP };

WbDeviceTag kinectColor;
WbDeviceTag kinectRange;
WbDeviceTag head_tilt_motor;
WbDeviceTag head_tilt_sensor;
WbDeviceTag torso_motor;
WbDeviceTag torso_sensor;

WbDeviceTag left_arm_motors[5];
WbDeviceTag left_arm_sensors[5];
WbDeviceTag right_arm_motors[5];
WbDeviceTag right_arm_sensors[5];

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

  left_arm_motors[SHOULDER_ROLL] = wb_robot_get_device("l_shoulder_pan_joint");
  left_arm_motors[SHOULDER_LIFT] = wb_robot_get_device("l_shoulder_lift_joint");
  left_arm_motors[UPPER_ARM_ROLL] = wb_robot_get_device("l_upper_arm_roll_joint");
  left_arm_motors[ELBOW_LIFT] = wb_robot_get_device("l_elbow_flex_joint");
  left_arm_motors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint");
  left_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("l_shoulder_pan_joint_sensor");
  left_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("l_shoulder_lift_joint_sensor");
  left_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("l_upper_arm_roll_joint_sensor");
  left_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("l_elbow_flex_joint_sensor");
  left_arm_sensors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint_sensor");

  right_arm_motors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint");
  right_arm_motors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint");
  right_arm_motors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint");
  right_arm_motors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint");
  right_arm_motors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint");
  right_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint_sensor");
  right_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint_sensor");
  right_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint_sensor");
  right_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint_sensor");
  right_arm_sensors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint_sensor");

  for (int i = 0; i < 5; ++i) {
    wb_position_sensor_enable(left_arm_sensors[i], TIME_STEP);
    wb_position_sensor_enable(right_arm_sensors[i], TIME_STEP);
  }

  torso_motor = wb_robot_get_device("torso_lift_joint");
  torso_sensor = wb_robot_get_device("torso_lift_joint_sensor");
  wb_position_sensor_enable(torso_sensor, TIME_STEP);
}

void set_head_tilt(double tilt, bool wait_on_feedback) {
  wb_motor_set_position(head_tilt_motor, tilt);

  if (wait_on_feedback) {
    while (! ALMOST_EQUAL(wb_position_sensor_get_value(head_tilt_sensor), tilt))
      step();
  }
}

void set_torso_height(double height, bool wait_on_feedback) {
  wb_motor_set_position(torso_motor, height);

  if (wait_on_feedback) {
    while (! ALMOST_EQUAL(wb_position_sensor_get_value(torso_sensor), height))
      step();
  }
}

void set_right_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_roll, bool wait_on_feedback) {
  wb_motor_set_position(right_arm_motors[SHOULDER_ROLL], shoulder_roll);
  wb_motor_set_position(right_arm_motors[SHOULDER_LIFT], shoulder_lift);
  wb_motor_set_position(right_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
  wb_motor_set_position(right_arm_motors[ELBOW_LIFT], elbow_lift);
  wb_motor_set_position(right_arm_motors[WRIST_ROLL], wrist_roll);

  if (wait_on_feedback) {
    while (
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]), wrist_roll)
    ) {
      step();
    }
  }
}

void set_left_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_roll, bool wait_on_feedback) {
  wb_motor_set_position(left_arm_motors[SHOULDER_ROLL], shoulder_roll);
  wb_motor_set_position(left_arm_motors[SHOULDER_LIFT], shoulder_lift);
  wb_motor_set_position(left_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
  wb_motor_set_position(left_arm_motors[ELBOW_LIFT], elbow_lift);
  wb_motor_set_position(left_arm_motors[WRIST_ROLL], wrist_roll);

  if (wait_on_feedback) {
    while (
      ! ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[WRIST_ROLL]), wrist_roll)
    ) {
      step();
    }
  }
}

void set_initial_position() {
  set_head_tilt(M_PI_4, false);
  set_torso_height(0.2, true);
  set_left_arm_position(1.0, 1.35, 0.0, -2.2, 0.0, false);
  set_right_arm_position(-1.0, 1.35, 0.0, -2.2, 0.0, true);
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  initialize_devices();
  set_initial_position();

  while (wb_robot_step(TIME_STEP) != -1) {

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
