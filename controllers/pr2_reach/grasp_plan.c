#include "grasp_plan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

double *planner_data;
int total_cnt;

void init_planner(int shoulder_roll_cnt, int shoulder_lift_cnt, int upper_arm_roll_cnt, int elbow_lift_cnt) {
    FILE *f = fopen("reachability_map.txt", "r");
    total_cnt = shoulder_roll_cnt * shoulder_lift_cnt * upper_arm_roll_cnt * elbow_lift_cnt;
    planner_data = malloc(total_cnt * 7 * sizeof(double));
    for (int i = 0; i < total_cnt; i++) {
        fscanf(f, "%lf, %lf, %lf, %lf, %lf, %lf, %lf",
            planner_data + i * 7, planner_data + i * 7 + 1, planner_data + i * 7 + 2, planner_data + i * 7 + 3,
            planner_data + i * 7 + 4, planner_data + i * 7 + 5, planner_data + i * 7 + 6);
    }
    fclose(f);
}

void plan_grasp(double target_pos[], double *arm_params) {
    double min_dist_sq = 9999;
    int min_dist_ind = -1;
    for (int i = 0; i < total_cnt; i++)
    {
        double *pos = planner_data + i * 7 + 4;
        double dx = target_pos[0] - pos[0];
        double dy = target_pos[1] - pos[1];
        double dz = target_pos[2] - pos[2];
        double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            min_dist_ind = i;
        }
    }
    memcpy(arm_params, planner_data + min_dist_ind * 7, 4 * sizeof(double));
}

void clear_planner() {
    free(planner_data);
    total_cnt = 0;
}