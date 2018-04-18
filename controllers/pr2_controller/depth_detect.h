#ifdef DEPTH_DETECT
#define DEPTH_DETECT

#include <stdio.h>
#include <stdlib.h>

void init_depth_detect(int w_, int h_, double h_fov_, double v_fov_, double cam_pos_[3], double cam_rot_[9]);
void save_back_depth(const float *depth);
void load_back_depth(float *depth);

void depth_detect(float *depth, int w, int h, float *back_depth);
void convert_depth_pixel_to_point(float depth, int x, int y, double point[3]);

void mat_dot(const double rot[9], const double point[3], double output[3]);
void convert_depth_pixel_to_point(float depth, int x, int y, double point[3]);
void clear_depth_detect();

#endif