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
#include <webots/touch_sensor.h>
#include <webots/supervisor.h>
#include <webots/display.h>

#include <stdio.h>
#include <stdlib.h>

#include "depth_detect.h"

#define TIME_STEP 16
#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

//#define CAPTURE_BACK_DEPTH

// helper constants to distinguish the motors
enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };
enum { SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL };
enum { LEFT_FINGER, RIGHT_FINGER, LEFT_TIP, RIGHT_TIP };

WbDeviceTag display;
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

WbDeviceTag right_finger_motors[4];
WbDeviceTag right_finger_sensors[4];
WbDeviceTag left_finger_motors[4];
WbDeviceTag left_finger_sensors[4];
WbDeviceTag left_finger_contact_sensors[2];
WbDeviceTag right_finger_contact_sensors[2];

// Simpler step function
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

void initialize_devices() {
  display = wb_robot_get_device("display");
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
    wb_motor_set_velocity(left_arm_motors[i], 1.0);
    wb_motor_set_velocity(right_arm_motors[i], 1.0);
  }

  torso_motor = wb_robot_get_device("torso_lift_joint");
  torso_sensor = wb_robot_get_device("torso_lift_joint_sensor");
  wb_position_sensor_enable(torso_sensor, TIME_STEP);

  left_finger_motors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_joint");
  left_finger_motors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_joint");
  left_finger_motors[LEFT_TIP] = wb_robot_get_device("l_gripper_l_finger_tip_joint");
  left_finger_motors[RIGHT_TIP] = wb_robot_get_device("l_gripper_r_finger_tip_joint");
  left_finger_sensors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_joint_sensor");
  left_finger_sensors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_joint_sensor");
  left_finger_sensors[LEFT_TIP] = wb_robot_get_device("l_gripper_l_finger_tip_joint_sensor");
  left_finger_sensors[RIGHT_TIP] = wb_robot_get_device("l_gripper_r_finger_tip_joint_sensor");

  right_finger_motors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_joint");
  right_finger_motors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_joint");
  right_finger_motors[LEFT_TIP] = wb_robot_get_device("r_gripper_l_finger_tip_joint");
  right_finger_motors[RIGHT_TIP] = wb_robot_get_device("r_gripper_r_finger_tip_joint");
  right_finger_sensors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_joint_sensor");
  right_finger_sensors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_joint_sensor");
  right_finger_sensors[LEFT_TIP] = wb_robot_get_device("r_gripper_l_finger_tip_joint_sensor");
  right_finger_sensors[RIGHT_TIP] = wb_robot_get_device("r_gripper_r_finger_tip_joint_sensor");

  left_finger_contact_sensors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_tip_contact_sensor");
  left_finger_contact_sensors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_tip_contact_sensor");
  right_finger_contact_sensors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_tip_contact_sensor");
  right_finger_contact_sensors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_tip_contact_sensor");

  for (int i = 0; i < 2; ++i) {
    wb_touch_sensor_enable(left_finger_contact_sensors[i], TIME_STEP);
    wb_touch_sensor_enable(right_finger_contact_sensors[i], TIME_STEP);
  }

  for (int i = 0; i < 4; ++i) {
    wb_position_sensor_enable(left_finger_sensors[i], TIME_STEP);
    wb_position_sensor_enable(right_finger_sensors[i], TIME_STEP);
  }
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

void set_gripper(bool left, bool open, double torqueWhenGripping, bool wait_on_feedback) {
  WbDeviceTag motors[4];
  motors[LEFT_FINGER]  = left ? left_finger_motors[LEFT_FINGER]  : right_finger_motors[LEFT_FINGER];
  motors[RIGHT_FINGER] = left ? left_finger_motors[RIGHT_FINGER] : right_finger_motors[RIGHT_FINGER];
  motors[LEFT_TIP]     = left ? left_finger_motors[LEFT_TIP]     : right_finger_motors[LEFT_TIP];
  motors[RIGHT_TIP]    = left ? left_finger_motors[RIGHT_TIP]    : right_finger_motors[RIGHT_TIP];

  WbDeviceTag sensors[4];
  sensors[LEFT_FINGER]  = left ? left_finger_sensors[LEFT_FINGER]  : right_finger_sensors[LEFT_FINGER];
  sensors[RIGHT_FINGER] = left ? left_finger_sensors[RIGHT_FINGER] : right_finger_sensors[RIGHT_FINGER];
  sensors[LEFT_TIP]     = left ? left_finger_sensors[LEFT_TIP]     : right_finger_sensors[LEFT_TIP];
  sensors[RIGHT_TIP]    = left ? left_finger_sensors[RIGHT_TIP]    : right_finger_sensors[RIGHT_TIP];

  WbDeviceTag contacts[2];
  contacts[LEFT_FINGER]  = left ? left_finger_contact_sensors[LEFT_FINGER]  : right_finger_contact_sensors[LEFT_FINGER];
  contacts[RIGHT_FINGER] = left ? left_finger_contact_sensors[RIGHT_FINGER] : right_finger_contact_sensors[RIGHT_FINGER];

  static bool firstCall = true;
  static double maxTorque = 0.0;
  if (firstCall) {
    maxTorque = wb_motor_get_available_torque(motors[LEFT_FINGER]);
    firstCall = false;
  }

  int i;
  for (i = 0; i < 4; ++i)
    wb_motor_set_available_torque(motors[i], maxTorque);

  if (open) {
    static const double targetOpenValue = 0.5;
    for (i = 0; i < 4; ++i)
      wb_motor_set_position(motors[i],  targetOpenValue);

    if (wait_on_feedback) {
      while (! ALMOST_EQUAL(wb_position_sensor_get_value(sensors[LEFT_FINGER]), targetOpenValue))
        step();
    }
  } else {
    static const double targetCloseValue = 0.0;
    for (i = 0; i < 4; ++i)
      wb_motor_set_position(motors[i],  targetCloseValue);

    if (wait_on_feedback) {
      // wait until the 2 touch sensors are fired or the target value is reached
      while (
        (wb_touch_sensor_get_value(contacts[LEFT_FINGER]) == 0.0 ||
         wb_touch_sensor_get_value(contacts[RIGHT_FINGER]) == 0.0) &&
        ! ALMOST_EQUAL(wb_position_sensor_get_value(sensors[LEFT_FINGER]), targetCloseValue)
      ) {
        step();
      }
      double current_position = wb_position_sensor_get_value(sensors[LEFT_FINGER]);
      for (i = 0; i < 4; ++i) {
        wb_motor_set_available_torque(motors[i], torqueWhenGripping);
        wb_motor_set_position(motors[i], fmax(0.0, 0.95 * current_position));
      }
    }
  }
}

void set_initial_position() {
  set_head_tilt(M_PI_4, false);
  set_torso_height(0.2, true);
  set_left_arm_position(1.0, 0.0, 0.0, 0.0, 0.0, false);
  set_right_arm_position(-1.0, 0.0, 0.0, 0.0, 0.0, true);
  set_left_arm_position(1.0, 1.35, 0.0, -2.2, 0.0, false);
  set_right_arm_position(-1.0, 1.35, 0.0, -2.2, 0.0, true);

  wb_display_attach_camera(display, kinectColor);
}

void init_depth_detector() {
  WbNodeRef kinect_node = wb_supervisor_node_get_from_def("kinect");
  const double *cam_pos = wb_supervisor_node_get_position(kinect_node);
  const double *cam_rot = wb_supervisor_node_get_orientation(kinect_node);
  int w = wb_range_finder_get_width(kinectRange);
  int h = wb_range_finder_get_height(kinectRange);
  double h_fov = wb_range_finder_get_fov(kinectRange);
  double v_fov = 2 * atan(tan(h_fov * 0.5 ) / (w / (double)h));
  init_depth_detect(w, h, h_fov, v_fov, cam_pos, cam_rot);
}

void detect_obj() {
  const float *depth = wb_range_finder_get_range_image(kinectRange);
  int min_x, min_y, max_x, max_y;
  double point[3];
  depth_detect(depth, point, &min_x, &min_y, &max_x, &max_y);
  printf("Object depth rect: [%d %d %d %d]\n", min_x, min_y, max_x, max_y);
  wb_display_draw_rectangle(display, min_x, min_y, max_x - min_x, max_y - min_y);
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  initialize_devices();
  set_initial_position();
#ifdef CAPTURE_BACK_DEPTH
  const float *back_depth = wb_range_finder_get_range_image(kinectRange);
  int w = wb_range_finder_get_width(kinectRange);
  int h = wb_range_finder_get_height(kinectRange);
  save_back_depth(w, h, back_depth);
#endif
  init_depth_detector();

  while (wb_robot_step(TIME_STEP) != -1) {
    detect_obj();
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  clear_depth_detect();
  return 0;
}
