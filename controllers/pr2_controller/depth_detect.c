#include "depth_detect.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TOLERANCE 0.05
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

float *back_depth;
int w;
int h;

double h_fov;
double v_fov;
double cam_pos[3];
double cam_rot[9];

void init_depth_detect(int w_, int h_, double h_fov_, double v_fov_, const double cam_pos_[3], const double cam_rot_[9]) {
    w = w_;
    h = h_;
    h_fov = h_fov_;
    v_fov = v_fov_;
    memcpy(cam_pos, cam_pos_, 3 * sizeof(double));
    memcpy(cam_rot, cam_rot_, 9 * sizeof(double));
    back_depth = malloc(w * h * sizeof(float));
    load_back_depth(back_depth);
}

void save_gripper_depth(int w, int h, const float *depth, const char *name) {
    FILE *f = fopen(name, "wb");
    fwrite(depth, sizeof(float), w * h, f);
    fclose(f);
}

void save_back_depth(int w, int h, const float *depth) {
    FILE *f = fopen("back_depth.bin", "wb");
    fwrite(depth, sizeof(float), w * h, f);
    fclose(f);
}

void load_gripper_depth(float *depth, const char *name) {
    FILE *f = fopen(name, "rb");
    fread(depth, sizeof(float), w * h, f);
    fclose(f);
}

void load_back_depth(float *depth) {
    FILE *f = fopen("back_depth.bin", "rb");
    fread(depth, sizeof(float), w * h, f);
    fclose(f);
}

void depth_detect(const float *depth, double point[3],
                  int *min_x, int *min_y,
                  int *max_x, int *max_y) {
    *min_x = w - 1, *min_y = h - 1;
    *max_x = 0, *max_y = 0;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            if (ALMOST_EQUAL(depth[y * w + x], back_depth[y * w + x])) {
                continue;
            }
            *min_x = MIN(*min_x, x);
            *min_y = MIN(*min_y, y);
            *max_x = MAX(*max_x, x);
            *max_y = MAX(*max_y, y);
        }
    }
    int center_x = (*min_x + *max_x) / 2;
    int center_y = (*min_y + *max_y) / 2;
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
    point[0] = cam_pos[0] - point[0];
    point[1] = cam_pos[1] - point[1];
    point[2] += cam_pos[2];
}

void reconstruct_point_cloud(const float *depth, const unsigned char *image, const char *filename) {
    FILE *f = fopen(filename, "w");
    fprintf(f, "ply\n");
    fprintf(f, "format ascii 1.0\n");
    double *point = malloc(w * h * 3 * sizeof(double));
    unsigned char *color = malloc(w * h * 3 * sizeof(unsigned char));
    int cnt = 0;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            if (depth[y * w + x] > 1.5 || ALMOST_EQUAL(depth[y * w + x], back_depth[y * w + x])) {
                continue;
            }
            float depth_val = depth[y * w + x];
            convert_depth_pixel_to_point(depth_val, x, y, point + 3 * cnt);
            memcpy(color + 3 * cnt, image + y * 4 * w + x * 4, 3 * sizeof(unsigned char));
            cnt++;
        }
    }
    fprintf(f, "element vertex %d\n", cnt);
    fprintf(f, "property float x\n");
    fprintf(f, "property float y\n");
    fprintf(f, "property float z\n");
    fprintf(f, "property uchar diffuse_blue\n");
    fprintf(f, "property uchar diffuse_green\n");
    fprintf(f, "property uchar diffuse_red\n");
    fprintf(f, "end_header\n");
    for (int i = 0; i < cnt; i++) {
        fprintf(f, "%f %f %f %d %d %d\n",
                (float)point[i * 3], (float)point[i * 3 + 1], (float)point[i * 3 + 2],
                color[i * 3], color[i * 3 + 1], color[i * 3 + 2]);
    }
    free(point);
    free(color);
    fclose(f);
}

void reconstruct_point_cloud_no_gripper(const float *depth, const unsigned char *image, const char *filename, const float *gripper_depth) {
    FILE *f = fopen(filename, "w");
    fprintf(f, "ply\n");
    fprintf(f, "format ascii 1.0\n");
    double *point = malloc(w * h * 3 * sizeof(double));
    unsigned char *color = malloc(w * h * 3 * sizeof(unsigned char));
    int cnt = 0;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            if (depth[y * w + x] > 1.5 || ALMOST_EQUAL(depth[y * w + x], gripper_depth[y * w + x]) ||
                ALMOST_EQUAL(depth[y * w + x], back_depth[y * w + x])) {
                continue;
            }
            float depth_val = depth[y * w + x];
            convert_depth_pixel_to_point(depth_val, x, y, point + 3 * cnt);
            memcpy(color + 3 * cnt, image + y * 4 * w + x * 4, 3 * sizeof(unsigned char));
            cnt++;
        }
    }
    fprintf(f, "element vertex %d\n", cnt);
    fprintf(f, "property float x\n");
    fprintf(f, "property float y\n");
    fprintf(f, "property float z\n");
    fprintf(f, "property uchar diffuse_blue\n");
    fprintf(f, "property uchar diffuse_green\n");
    fprintf(f, "property uchar diffuse_red\n");
    fprintf(f, "end_header\n");
    for (int i = 0; i < cnt; i++) {
        fprintf(f, "%f %f %f %d %d %d\n",
                (float)point[i * 3], (float)point[i * 3 + 1], (float)point[i * 3 + 2],
                color[i * 3], color[i * 3 + 1], color[i * 3 + 2]);
    }
    free(point);
    free(color);
    fclose(f);
}

void clear_depth_detect() {
    free(back_depth);
}