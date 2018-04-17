/*
 * File:          pr2_reach.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// PR2 constants
#define MAX_WHEEL_SPEED 3.0 // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492 // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098 // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08 // wheel radius

#define TIME_STEP 16
#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))
#define ALMOST_EQUAL_TOL(a, b, t) ((a < b + t) && (a > b - t))

#define MAXIMUM_NUMBER_OF_COORDINATES 200  // Size of the history

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

WbDeviceTag right_finger_motors[4];
WbDeviceTag right_finger_sensors[4];
WbDeviceTag left_finger_motors[4];
WbDeviceTag left_finger_sensors[4];
WbDeviceTag left_finger_contact_sensors[2];
WbDeviceTag right_finger_contact_sensors[2];

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
  }

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

void set_right_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_roll, bool wait_on_feedback) {
  wb_motor_set_position(right_arm_motors[SHOULDER_ROLL], shoulder_roll);
  wb_motor_set_position(right_arm_motors[SHOULDER_LIFT], shoulder_lift);
  wb_motor_set_position(right_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
  wb_motor_set_position(right_arm_motors[ELBOW_LIFT], elbow_lift);
  wb_motor_set_position(right_arm_motors[WRIST_ROLL], wrist_roll);
  double last_sensors[5];
  if (wait_on_feedback) {
    while (
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
      ! ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]), wrist_roll)
    ) {
      step();
      //Avoid freezing
      double cur_sensors[5];
      cur_sensors[0] = wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]);
      cur_sensors[1] = wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]);
      cur_sensors[2] = wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]);
      cur_sensors[3] = wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]);
      cur_sensors[4] = wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]);
      if (ALMOST_EQUAL_TOL(cur_sensors[0], last_sensors[0], 0.005) &&
          ALMOST_EQUAL_TOL(cur_sensors[1], last_sensors[1], 0.005) &&
          ALMOST_EQUAL_TOL(cur_sensors[2], last_sensors[2], 0.005) &&
          ALMOST_EQUAL_TOL(cur_sensors[3], last_sensors[3], 0.005) &&
          ALMOST_EQUAL_TOL(cur_sensors[4], last_sensors[4], 0.005)) {
            break;
          }
      memcpy(last_sensors, cur_sensors, 5 * sizeof(double));
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
  set_left_arm_position(2.0, 1.35, 0.0, -2.2, 0.0, true);
}

// Create the trail shape with the correct number of coordinates.
void create_trail_shape() {
  // If TRAIL exists in the world then silently removes it.
  WbNodeRef existing_trail = wb_supervisor_node_get_from_def("TRAIL");
  if (existing_trail)
    wb_supervisor_node_remove(existing_trail);

  int i;
  char trail_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.

  // Create the TRAIL Shape.
  strcat(trail_string, "DEF TRAIL Shape {\n");
  strcat(trail_string, "  appearance Appearance {\n");
  strcat(trail_string, "    material Material {\n");
  strcat(trail_string, "      diffuseColor 0 1 0\n");
  strcat(trail_string, "      emissiveColor 0 1 0\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n");
  strcat(trail_string, "    coord Coordinate {\n");
  strcat(trail_string, "      point [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 0\n");
  strcat(trail_string, "      ]\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 -1\n");
  strcat(trail_string, "    ]\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "}\n");

  // Import TRAIL and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, trail_string);
}

void traverse_all_arm_position() {
  create_trail_shape();

  // Get interesting references to the TRAIL subnodes.
  WbNodeRef trail_line_set_node = wb_supervisor_node_get_from_def("TRAIL_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trail_line_set_node, "coord"));
  WbFieldRef point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  WbFieldRef coord_index_field = wb_supervisor_node_get_field(trail_line_set_node, "coordIndex");

  int index = 0;           // This points to the current position to be drawn.
  bool first_step = true;  // Only equals to true during the first step.
  
  WbNodeRef l_finger_node = wb_supervisor_node_get_from_def("r_gripper_l_finger_tip_link");
  WbNodeRef r_finger_node = wb_supervisor_node_get_from_def("r_gripper_r_finger_tip_link");

  double shoulder_roll_min = wb_motor_get_min_position(right_arm_motors[SHOULDER_ROLL]) + 1.5;
  double shoulder_lift_min = wb_motor_get_min_position(right_arm_motors[SHOULDER_LIFT]) + 0.5;
  double upper_arm_roll_min = wb_motor_get_min_position(right_arm_motors[UPPER_ARM_ROLL]) + 0.5;
  double elbow_lift_min = wb_motor_get_min_position(right_arm_motors[ELBOW_LIFT]) + 0.5;

  double shoulder_roll_max = wb_motor_get_max_position(right_arm_motors[SHOULDER_ROLL]);
  double shoulder_lift_max = wb_motor_get_max_position(right_arm_motors[SHOULDER_LIFT]);
  double upper_arm_roll_max = wb_motor_get_max_position(right_arm_motors[UPPER_ARM_ROLL]);
  double elbow_lift_max = wb_motor_get_max_position(right_arm_motors[ELBOW_LIFT]);

  int shoulder_roll_cnt = 20;
  int shoulder_lift_cnt = 20;
  int upper_arm_roll_cnt = 20;
  int elbow_lift_cnt = 20;

  for (int i1 = 0; i1 <= shoulder_roll_cnt; i1++) {
    double shoulder_roll = shoulder_roll_min + (shoulder_roll_max - shoulder_roll_min - 0.01) * i1 / shoulder_roll_cnt;
    for (int i2 = 0; i2 <= shoulder_lift_cnt; i2++) {
      double shoulder_lift = shoulder_lift_min + (shoulder_lift_max - shoulder_lift_min - 0.01) * i2 / shoulder_lift_cnt;
      for (int i3 = 0; i3 <= upper_arm_roll_cnt; i3++) {
        double upper_arm_roll = upper_arm_roll_min + (upper_arm_roll_max - upper_arm_roll_min - 0.01) * i3 / upper_arm_roll_cnt;
        for (int i4 = 0; i4 <= elbow_lift_cnt; i4++) {
          double elbow_lift = elbow_lift_min + (elbow_lift_max - elbow_lift_min - 0.01) * i4 / elbow_lift_cnt;
          set_right_arm_position(shoulder_roll, shoulder_lift, upper_arm_roll, elbow_lift, 0.0, true);
          const double *l_finger_pos = wb_supervisor_node_get_position(l_finger_node);
          const double *r_finger_pos = wb_supervisor_node_get_position(r_finger_node);
          double pos[3] = {(l_finger_pos[0] + r_finger_pos[0]) / 2.0,
                               (l_finger_pos[1] + r_finger_pos[1]) / 2.0,
                               (l_finger_pos[2] + r_finger_pos[2]) / 2.0};
          printf("[%lf, %lf, %lf, %lf] [%lf, %lf, %lf]\n",
            shoulder_roll, shoulder_lift, upper_arm_roll, elbow_lift,
            pos[0], pos[1], pos[2]);
          FILE *f = fopen("reachability_map.txt", "a");
          fprintf(f, "%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
            shoulder_roll, shoulder_lift, upper_arm_roll, elbow_lift,
            pos[0], pos[1], pos[2]);
          fclose(f);

          // Add the new target translation in the line set.
          wb_supervisor_field_set_mf_vec3f(point_field, index, pos);

          // Update the line set indices.
          if (index > 0) {
            // Link successive indices.
            wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1), index - 1);
            wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1) + 1, index);
          } else if (index == 0 && first_step == false) {
            // Link the first and the last indices.
            wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0);
            wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
                                            MAXIMUM_NUMBER_OF_COORDINATES - 1);
          }
          // Unset the next indices.
          wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index, index);
          wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index + 1, index);

          // Update global variables.
          first_step = false;
          index++;
          index = index % MAXIMUM_NUMBER_OF_COORDINATES;
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  initialize_devices();
  set_initial_position();

  traverse_all_arm_position();
  while (wb_robot_step(TIME_STEP) != -1) {

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
