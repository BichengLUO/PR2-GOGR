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
#include <string.h>

#include "depth_detect.h"
#include "../pr2_reach/grasp_plan.h"

// PR2 constants
#define MAX_WHEEL_SPEED 3.0 // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492 // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098 // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08 // wheel radius

#define TIME_STEP 16
#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

#define SHOULDER_ROLL_CNT 5
#define SHOULDER_LIFT_CNT 5
#define UPPER_ARM_ROLL_CNT 5
#define ELBOW_LIFT_CNT 5
#define WRIST_LIFT_CNT 5

//#define CAPTURE_BACK_DEPTH
//#define CAPTURE_GRIPPER_DEPTH
#define IMAGE_TAKEN_CNT 5

#define MAXIMUM_NUMBER_OF_COORDINATES 1000  // Size of point cloud

// helper constants to distinguish the motors
enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };
enum { SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_LIFT, WRIST_ROLL };
enum { LEFT_FINGER, RIGHT_FINGER, LEFT_TIP, RIGHT_TIP };

WbDeviceTag display;
WbDeviceTag kinectColor;
WbDeviceTag kinectRange;
WbDeviceTag head_tilt_motor;
WbDeviceTag head_tilt_sensor;
WbDeviceTag torso_motor;
WbDeviceTag torso_sensor;

WbDeviceTag left_arm_motors[6];
WbDeviceTag left_arm_sensors[6];
WbDeviceTag right_arm_motors[6];
WbDeviceTag right_arm_sensors[6];

WbDeviceTag right_finger_motors[4];
WbDeviceTag right_finger_sensors[4];
WbDeviceTag left_finger_motors[4];
WbDeviceTag left_finger_sensors[4];
WbDeviceTag left_finger_contact_sensors[2];
WbDeviceTag right_finger_contact_sensors[2];

WbNodeRef indicator_node;
WbFieldRef indicator_trans_field;

WbDeviceTag wheel_motors[8];
WbDeviceTag wheel_sensors[8];

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
  left_arm_motors[WRIST_LIFT] = wb_robot_get_device("l_wrist_flex_joint");
  left_arm_motors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint");
  left_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("l_shoulder_pan_joint_sensor");
  left_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("l_shoulder_lift_joint_sensor");
  left_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("l_upper_arm_roll_joint_sensor");
  left_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("l_elbow_flex_joint_sensor");
  left_arm_sensors[WRIST_LIFT] = wb_robot_get_device("l_wrist_flex_joint_sensor");
  left_arm_sensors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint_sensor");

  right_arm_motors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint");
  right_arm_motors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint");
  right_arm_motors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint");
  right_arm_motors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint");
  right_arm_motors[WRIST_LIFT] = wb_robot_get_device("r_wrist_flex_joint");
  right_arm_motors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint");
  right_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint_sensor");
  right_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint_sensor");
  right_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint_sensor");
  right_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint_sensor");
  right_arm_sensors[WRIST_LIFT] = wb_robot_get_device("r_wrist_flex_joint_sensor");
  right_arm_sensors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint_sensor");

  for (int i = 0; i < 6; ++i) {
    wb_position_sensor_enable(left_arm_sensors[i], TIME_STEP);
    wb_position_sensor_enable(right_arm_sensors[i], TIME_STEP);
    wb_motor_set_velocity(left_arm_motors[i], 0.5);
    wb_motor_set_velocity(right_arm_motors[i], 0.5);
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
    wb_motor_set_velocity(left_finger_motors[i], 0.5);
    wb_motor_set_velocity(right_finger_motors[i], 0.5);
  }

  indicator_node = wb_supervisor_node_get_from_def("Indicator");
  indicator_trans_field = wb_supervisor_node_get_field(indicator_node, "translation");

  wheel_motors[FLL_WHEEL]  = wb_robot_get_device("fl_caster_l_wheel_joint");
  wheel_motors[FLR_WHEEL]  = wb_robot_get_device("fl_caster_r_wheel_joint");
  wheel_motors[FRL_WHEEL]  = wb_robot_get_device("fr_caster_l_wheel_joint");
  wheel_motors[FRR_WHEEL]  = wb_robot_get_device("fr_caster_r_wheel_joint");
  wheel_motors[BLL_WHEEL]  = wb_robot_get_device("bl_caster_l_wheel_joint");
  wheel_motors[BLR_WHEEL]  = wb_robot_get_device("bl_caster_r_wheel_joint");
  wheel_motors[BRL_WHEEL]  = wb_robot_get_device("br_caster_l_wheel_joint");
  wheel_motors[BRR_WHEEL]  = wb_robot_get_device("br_caster_r_wheel_joint");
  wheel_sensors[FLL_WHEEL] = wb_robot_get_device("fl_caster_l_wheel_joint_sensor");
  wheel_sensors[FLR_WHEEL] = wb_robot_get_device("fl_caster_r_wheel_joint_sensor");
  wheel_sensors[FRL_WHEEL] = wb_robot_get_device("fr_caster_l_wheel_joint_sensor");
  wheel_sensors[FRR_WHEEL] = wb_robot_get_device("fr_caster_r_wheel_joint_sensor");
  wheel_sensors[BLL_WHEEL] = wb_robot_get_device("bl_caster_l_wheel_joint_sensor");
  wheel_sensors[BLR_WHEEL] = wb_robot_get_device("bl_caster_r_wheel_joint_sensor");
  wheel_sensors[BRL_WHEEL] = wb_robot_get_device("br_caster_l_wheel_joint_sensor");
  wheel_sensors[BRR_WHEEL] = wb_robot_get_device("br_caster_r_wheel_joint_sensor");

  for (int i = 0; i < 8; ++i) {
    wb_position_sensor_enable(wheel_sensors[i], TIME_STEP);
    // init the motors for speed control
    wb_motor_set_position(wheel_motors[i], INFINITY);
    wb_motor_set_velocity(wheel_motors[i], 0.0);
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

void set_right_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_lift, double wrist_roll, bool wait_on_feedback) {
  wb_motor_set_position(right_arm_motors[SHOULDER_ROLL], shoulder_roll);
  wb_motor_set_position(right_arm_motors[SHOULDER_LIFT], shoulder_lift);
  wb_motor_set_position(right_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
  wb_motor_set_position(right_arm_motors[ELBOW_LIFT], elbow_lift);
  wb_motor_set_position(right_arm_motors[WRIST_LIFT], wrist_lift);
  wb_motor_set_position(right_arm_motors[WRIST_ROLL], wrist_roll);

  if (wait_on_feedback) {
    while (
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_LIFT]), wrist_lift) ||
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

void set_wheels_speeds(
  double fll, double flr, double frl, double frr,
  double bll, double blr, double brl, double brr
) {
  wb_motor_set_velocity(wheel_motors[FLL_WHEEL], fll);
  wb_motor_set_velocity(wheel_motors[FLR_WHEEL], flr);
  wb_motor_set_velocity(wheel_motors[FRL_WHEEL], frl);
  wb_motor_set_velocity(wheel_motors[FRR_WHEEL], frr);
  wb_motor_set_velocity(wheel_motors[BLL_WHEEL], bll);
  wb_motor_set_velocity(wheel_motors[BLR_WHEEL], blr);
  wb_motor_set_velocity(wheel_motors[BRL_WHEEL], brl);
  wb_motor_set_velocity(wheel_motors[BRR_WHEEL], brr);
}

void set_wheels_speed(double speed) {
  set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed);
}

void stop_wheels() {
  set_wheels_speeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void robot_go_forward(double distance) {
  double max_wheel_speed = distance > 0 ? MAX_WHEEL_SPEED : - MAX_WHEEL_SPEED;
  set_wheels_speed(max_wheel_speed);

  double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);

  while (true) {
    double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position)); // travel distance done by the wheel

    if (wheel0_travel_distance > fabs(distance))
      break;

    // reduce the speed before reaching the target
    if (fabs(distance) - wheel0_travel_distance < 0.025)
      set_wheels_speed(0.1 * max_wheel_speed);

    step();
  }

  stop_wheels();
}

void set_initial_position() {
  set_head_tilt(M_PI_4, false);
  set_torso_height(0.2, true);
  set_left_arm_position(1.0, 0.0, 0.0, 0.0, 0.0, false);
  set_right_arm_position(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
  set_left_arm_position(1.0, 1.35, 0.0, -2.2, 0.0, false);
  set_right_arm_position(-1.0, 1.35, 0.0, -2.2, 0.0, 0.0, true);

  wb_display_attach_camera(display, kinectColor);
}

void init_depth_detector() {
  WbNodeRef kinect_node = wb_supervisor_node_get_from_def("kinect.RangeFinder");
  const double *cam_pos = wb_supervisor_node_get_position(kinect_node);
  const double *cam_rot = wb_supervisor_node_get_orientation(kinect_node);
  int w = wb_range_finder_get_width(kinectRange);
  int h = wb_range_finder_get_height(kinectRange);
  double h_fov = wb_range_finder_get_fov(kinectRange);
  double v_fov = 2 * atan(tan(h_fov * 0.5 ) / (w / (double)h));
  init_depth_detect(w, h, h_fov, v_fov, cam_pos, cam_rot);
}

void detect_obj(double point[3]) {
  const float *depth = wb_range_finder_get_range_image(kinectRange);
  int min_x, min_y, max_x, max_y;
  depth_detect(depth, point, &min_x, &min_y, &max_x, &max_y);
  printf("Object depth rect: [%d %d %d %d]\n", min_x, min_y, max_x, max_y);
  wb_display_draw_rectangle(display, min_x, min_y, max_x - min_x, max_y - min_y);
  printf("Object pos: [%lf, %lf, %lf]\n", point[0], point[1], point[2]);
  wb_supervisor_field_set_sf_vec3f(indicator_trans_field, point);
}

void get_finger_tip_center(double *pos) {
  WbNodeRef l_finger_node = wb_supervisor_node_get_from_def("r_gripper_l_finger_tip_link");
  WbNodeRef r_finger_node = wb_supervisor_node_get_from_def("r_gripper_r_finger_tip_link");
  const double *l_finger_pos = wb_supervisor_node_get_position(l_finger_node);
  const double *r_finger_pos = wb_supervisor_node_get_position(r_finger_node);
  pos[0] = (l_finger_pos[0] + r_finger_pos[0]) / 2.0;
  pos[1] = (l_finger_pos[1] + r_finger_pos[1]) / 2.0;
  pos[2] = (l_finger_pos[2] + r_finger_pos[2]) / 2.0;
}

void get_finger_joint_center(double *pos) {
  WbNodeRef l_finger_node = wb_supervisor_node_get_from_def("r_gripper_l_finger_link");
  WbNodeRef r_finger_node = wb_supervisor_node_get_from_def("r_gripper_r_finger_link");
  const double *l_finger_pos = wb_supervisor_node_get_position(l_finger_node);
  const double *r_finger_pos = wb_supervisor_node_get_position(r_finger_node);
  pos[0] = (l_finger_pos[0] + r_finger_pos[0]) / 2.0;
  pos[1] = (l_finger_pos[1] + r_finger_pos[1]) / 2.0;
  pos[2] = (l_finger_pos[2] + r_finger_pos[2]) / 2.0;
}

void normalize_dir(double *dir) {
  double len = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
  dir[0] /= len;
  dir[1] /= len;
  dir[2] /= len;
}

void wait_step(int steps_cnt) {
  for (int i = 0; i < steps_cnt; i++) {
    step();
  }
}

void create_pointcloud() {
  // If POINTCLOUD exists in the world then silently removes it.
  WbNodeRef existing_pc = wb_supervisor_node_get_from_def("POINTCLOUD");
  if (existing_pc)
    wb_supervisor_node_remove(existing_pc);

  int i;
  printf("here");
  char pc_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.
  
  // Create the POINTCLOUD Shape.
  strcat(pc_string, "DEF POINTCLOUD Shape {\n");
  strcat(pc_string, "  appearance Appearance {\n");
  strcat(pc_string, "    material Material {\n");
  strcat(pc_string, "      diffuseColor 1 0 0\n");
  strcat(pc_string, "      emissiveColor 1 0 0\n");
  strcat(pc_string, "    }\n");
  strcat(pc_string, "  }\n");
  strcat(pc_string, "  geometry DEF POINTCLOUD_POINT_SET IndexedLineSet {\n");
  strcat(pc_string, "    coord Coordinate {\n");
  strcat(pc_string, "      point [\n");
  for (i = 0; i < 2 * MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(pc_string, "      0 0 0\n");
  strcat(pc_string, "      ]\n");
  strcat(pc_string, "    }\n");
  strcat(pc_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(pc_string, "    0 0 -1\n");
  strcat(pc_string, "    ]\n");
  strcat(pc_string, "  }\n");
  strcat(pc_string, "}\n");

  // Import POINTCLOUD and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, pc_string);
}

void visualize_pointcloud(int w, int h) {
  // Get interesting references to the POINTCLOUD subnodes.
  WbNodeRef pc_point_set_node = wb_supervisor_node_get_from_def("POINTCLOUD_POINT_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(pc_point_set_node, "coord"));
  WbFieldRef point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  WbFieldRef coord_index_field = wb_supervisor_node_get_field(pc_point_set_node, "coordIndex");
  int cnt = 0;
  const float *depth = wb_range_finder_get_range_image(kinectRange);
  const float *back_depth = get_back_depth();
  for (int x = 0; x < w; x += 5) {
      for (int y = 0; y < h; y += 5) {
        if (depth[y * w + x] > 1.5 || ALMOST_EQUAL(depth[y * w + x], back_depth[y * w + x])) {
          continue;
        }
        float depth_val = depth[y * w + x];
        double point[3];
        convert_depth_pixel_to_point(depth_val, x, y, point);
        // Add in the point set.
        wb_supervisor_field_set_mf_vec3f(point_field, 2 * cnt, point);
        point[1] += 0.005;
        point[2] += 0.005;
        wb_supervisor_field_set_mf_vec3f(point_field, 2 * cnt + 1, point);
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * cnt, 2 * cnt);
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * cnt + 1, 2 * cnt + 1);
        cnt++;
        if (cnt >= MAXIMUM_NUMBER_OF_COORDINATES) return;
      }
  }
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  initialize_devices();
  set_initial_position();
  int w = wb_range_finder_get_width(kinectRange);
  int h = wb_range_finder_get_height(kinectRange);
  init_depth_detector();
  double obj_pos[3];
  detect_obj(obj_pos);
  //create_pointcloud();
  //visualize_pointcloud(w, h);
#ifdef CAPTURE_BACK_DEPTH
  const float *back_depth = wb_range_finder_get_range_image(kinectRange);
  save_back_depth(w, h, back_depth);
#endif
#ifndef CAPTURE_GRIPPER_DEPTH
  float *gripper_depths[IMAGE_TAKEN_CNT];
  for (int i = 0; i < IMAGE_TAKEN_CNT; i++) {
    gripper_depths[i] = malloc(w * h * sizeof(float));
    char filename[100];
    sprintf(filename, "gripper_depth/%03d.bin", i);
    load_depth(gripper_depths[i], filename);
  }
#endif

  init_planner(SHOULDER_ROLL_CNT, SHOULDER_LIFT_CNT, UPPER_ARM_ROLL_CNT, ELBOW_LIFT_CNT, WRIST_LIFT_CNT);
  double arm_params[5];
  plan_grasp(obj_pos, arm_params);
  set_gripper(false, true, 0.0, false);
  robot_go_forward(-0.2);
  set_right_arm_position(arm_params[0], arm_params[1], arm_params[2], arm_params[3], arm_params[4], 0.0, true);
  robot_go_forward(0.2);
#ifndef CAPTURE_GRIPPER_DEPTH
  set_gripper(false, false, 30.0, true);
  wait_step(100);
#endif
  set_right_arm_position(0.0, 0.8, -0.5, -2.0, 0.0, 0.0, true);
#ifdef CAPTURE_GRIPPER_DEPTH
  set_gripper(false, false, 10.0, true);
#endif
  double finger_tip_center[3], finger_joint_center[3];
  for (int i = 0; i < IMAGE_TAKEN_CNT; i++) {
    double wrist_roll = i * M_PI * 2.0 / IMAGE_TAKEN_CNT;
    set_right_arm_position(0.0, 0.8, -0.5, -2.0, 0.0, wrist_roll, true);
    wait_step(100);
    const float *depth = wb_range_finder_get_range_image(kinectRange);
    get_finger_tip_center(finger_tip_center);
    get_finger_joint_center(finger_joint_center);
    double finger_dir[3] = {finger_tip_center[0] - finger_joint_center[0],
                            finger_tip_center[1] - finger_joint_center[1],
                            finger_tip_center[2] - finger_joint_center[2]};
    normalize_dir(finger_dir);
    printf("[%d] Finger tip center: [%lf, %lf, %lf]\n", i, finger_tip_center[0], finger_tip_center[1], finger_tip_center[2]);
    printf("[%d] Finger joint center: [%lf, %lf, %lf]\n", i, finger_joint_center[0], finger_joint_center[1], finger_joint_center[2]);
    printf("[%d] Finger direction: [%lf, %lf, %lf]\n", i, finger_dir[0], finger_dir[1], finger_dir[2]);
    char filename[100];
#ifdef CAPTURE_GRIPPER_DEPTH
    sprintf(filename, "gripper_depth/%03d.bin", i);
    save_depth(w, h, depth, filename);
#else
    const unsigned char *image = wb_camera_get_image(kinectColor);
    sprintf(filename, "pointcloud_taken/%03d.ply", i);
    reconstruct_point_cloud_no_gripper_rotated(depth,
                                               image,
                                               filename,
                                               gripper_depths[i],
                                               finger_tip_center,
                                               finger_dir,
                                               -wrist_roll);
    sprintf(filename, "depth_taken/%03d.bin", i);
    save_depth(w, h, depth, filename);
#endif
    printf("[%d] Wrist roll: %lf\n", i, wrist_roll);
  }
  while (wb_robot_step(TIME_STEP) != -1) {

  };
#ifndef CAPTURE_GRIPPER_DEPTH
  for (int i = 0; i < IMAGE_TAKEN_CNT; i++) {
    free(gripper_depths[i]);
  }
#endif
  wb_robot_cleanup();
  clear_depth_detect();
  return 0;
}
