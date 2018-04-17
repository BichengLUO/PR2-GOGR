/*
 * File:          supervisor.c
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
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 16
#define TOLERANCE 0.001
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  const char *def = "Can";
  WbNodeRef node = wb_supervisor_node_get_from_def(def);
  
  double last_pos[3];
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos = wb_supervisor_node_get_position(node);
    if (!ALMOST_EQUAL(pos[0], last_pos[0]) || !ALMOST_EQUAL(pos[1], last_pos[1]) || !ALMOST_EQUAL(pos[2], last_pos[2])) {
      printf("%s: [%lf, %lf, %lf]\n", wb_supervisor_node_get_def(node), pos[0], pos[1], pos[2]);
    }
    memcpy(last_pos, pos, 3 * sizeof(double));
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
