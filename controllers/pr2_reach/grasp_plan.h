#ifndef GRASP_PLAN
#define GRASP_PLAN

void init_planner(int shoulder_roll_cnt, int shoulder_lift_cnt, int upper_arm_roll_cnt, int elbow_lift_cnt, int wrist_lift_cnt);
void plan_grasp(double target_pos[], double *arm_params);
void clear_planner();

#endif