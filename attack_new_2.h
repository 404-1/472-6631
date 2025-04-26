//this function gets the angle between 2 centroids to the positive x-axis
//it also gets the distance between the 2 centroids
int angle_distance(double x1, double y1, double x2, double y2, double& distance, double& angle);

//this function checks if th distance from the obstacle projection on the path to the obstacle is less than the obstacle radius and extra margin
int projection_blocked(double xs, double ys, double xor, double yor, double xo, double yo, double obs_rad, double& clear_angle);

//checks if self is close to edge
int edge_detection(double width1, double height1, double th, double xs_f, double ys_f, bool& boundary_detected);

//rotates the robot to the right (clockwise)
int clockwise(double& pw_l_new, double& pw_r_new);

//rotates the robot to the left (counterclockwise)
int counterclockwise(double& pw_l_new, double& pw_r_new);

//moves the robot to straight forward
int move_straight(double& pw_l_new, double& pw_r_new);

//decides when to rotate the robot to align with opponent for laser shooting
int rotate_to_opponent(double angle_s, double angle_r, bool& aligned, double& pw_l_new, double& pw_r_new);

//decides where to rotate the robot to align with opponent for laser shooting
int rotate_to_center(double width, double height, double xs, double ys, double angle_s, double& pw_l_new, double& pw_r_new);

//this function sets the goal target to shoot. It will be half the distance
//set dist_ratio from 0 to 1. It is the ration when the goal point would be between the robots
int goal(double xs_f, double ys_f, double xor_f, double yor_f, double ratio, double& goal_x, double& goal_y, double& dist_goal, double& angle_goal);

//decides when to rotate the robot to align with goal
int rotate_to_goal(double goal_x, double goal_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double& pw_l_new, double& pw_r_new);

// this function sets a detour goal point if obstacle exists between the robots.
// go to this point when there is an obstacle. In main loop, if statement if obstacle in path
// set an obstacle margin/radius obs_rad is the radius of obs, and margin is the theshold
int detour(double xo, double yo, double obs_rad, double& detour_x, double& detour_y);

//decides where to rotate the robot to align with detour
int rotate_to_detour(double detour_x, double detour_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double& pw_l_new, double& pw_r_new);

int line_aligned(double xs, double ys, double xor, double yor, double& aligned);