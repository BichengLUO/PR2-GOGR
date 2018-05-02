#ifndef DEPTH_DETECT
#define DEPTH_DETECT

void init_depth_detect(int w_, int h_, double h_fov_, double v_fov_, const double cam_pos_[3], const double cam_rot_[9]);
void save_depth(int w, int h, const float *depth, const char *name);
void save_back_depth(int w, int h, const float *depth);
void load_depth(float *depth, const char *name);
const float* get_back_depth();
void load_back_depth(float *depth);

void depth_detect(const float *depth, double point[3],
                  int *min_x, int *min_y,
                  int *max_x, int *max_y);
void convert_depth_pixel_to_point(float depth, int x, int y, double point[3]);

void mat_dot(const double rot[9], const double point[3], double output[3]);
void convert_depth_pixel_to_point(float depth, int x, int y, double point[3]);
void reconstruct_point_cloud(const float *depth, const unsigned char *image, const char *filename);
void reconstruct_point_cloud_no_gripper(const float *depth, const unsigned char *image, const char *filename, const float *gripper_depth);
void make_rotation_matrix(const double *axis_d, double rad, double *rot);
void reconstruct_point_cloud_no_gripper_rotated(const float *depth,
                                                const unsigned char *image,
                                                const char *filename,
                                                const float *gripper_depth,
                                                const double *axis_p,
                                                const double *axis_d,
                                                double rad);
void clear_depth_detect();

#endif