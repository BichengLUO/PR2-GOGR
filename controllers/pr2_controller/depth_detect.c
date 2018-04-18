#include "depth_detect.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

float *back_depth;
int w;
int h;

double h_fov;
double v_fov;
double cam_pos[3];
double cam_rot[9];

void init_depth_detect(int w_, int h_, double h_fov_, double v_fov_, double cam_pos_[3], double cam_rot_[9]) {
    w = w_;
    h = h_;
    h_fov = h_fov_;
    v_fov = v_fov_;
    memcpy(cam_pos, cam_pos_, 3 * sizeof(double));
    memcpy(cam_rot, cam_rot_, 9 * sizeof(double));
    back_depth = malloc(w * h * sizeof(float));
    load_back_depth(back_depth);
}

void save_back_depth(const float *depth) {
    FILE *f = fopen("back_depth.txt", "wb");
    fwrite(depth, sizeof(float), w * h, f);
    fclose(f);
}

void load_back_depth(float *depth) {
    FILE *f = fopen("back_depth.txt", "rb");
    fread(depth, sizeof(float), w * h, f);
    fclose(f);
}

void depth_detect(const float *depth, double point[3]) {
    int min_x = 0, min_y = 0;
    int max_x = w - 1, max_y = h - 1;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            if (ALMOST_EQUAL(depth[y * w + x], back_depth[y * w + x])) {
                continue;
            }
            min_x = min(min_x, x);
            min_y = min(min_y, y);
            max_x = max(max_x, x);
            max_y = max(max_y, y);
        }
    }
    int center_x = (min_x + max_x) / 2;
    int center_y = (min_y + max_y) / 2;
    float depth_val = depth[center_y * w + center_x];
    convert_depth_pixel_to_point(depth_val, center_x, center_y, point);
}

void mat_dot(const double rot[9], const double point[3], double output[3]) {
    output[0] = rot[0] * point[0] + rot[1] * point[1] + rot[2] * point[2];
    output[1] = rot[3] * point[0] + rot[4] * point[1] + rot[5] * point[2];
    output[2] = rot[6] * point[0] + rot[7] * point[1] + rot[8] * point[2];
}

void convert_depth_pixel_to_point(float depth, int x, int y, double point[3]) {
    double local_point[3];
    local_point[0] = depth * sin((x / (double)w - 0.5) * h_fov);
    local_point[1] = depth * sin((y / (double)h - 0.5) * v_fov);
    local_point[2] = depth;
    mat_dot(cam_rot, local_point, point);
    point[0] += cam_pos[0];
    point[1] += cam_pos[1];
    point[2] += cam_pos[2];
}

void clear_depth_detect() {
    free(planner_data);
}