
// Calculate distance and angle between two points
int angle_distance(double x1, double y1, double x2, double y2, double& distance, double& angle);

// Check if obstacle blocks the projection path
int projection_blocked(double xs, double ys, double xor_, double yor, double xo, double yo, double obs_rad, bool& clear_angle);

// Detect if robot is near the image edges
int edge_detection(double width1, double height1, double th, double xs_f, double ys_f, bool& boundary_detected);

// Robot movement commands
int clockwise(double& pw_l_new, double& pw_r_new);
int counterclockwise(double& pw_l_new, double& pw_r_new);
int move_straight(double& pw_l_new, double& pw_r_new);

// Rotate to face the opponent
int rotate_to_opponent(double angle_s, double angle_r, bool& aligned, double& pw_l_new, double& pw_r_new);

// Rotate toward center of field
int rotate_to_center(double width, double height, double xs, double ys, double angle_s, double& pw_l_new, double& pw_r_new);

// Set goal target (between self and opponent)
int goal(double xs_f, double ys_f, double xor_f, double yor_f, double ratio, double& goal_x, double& goal_y, double& dist_goal, double& angle_goal);

// Rotate to face goal
int rotate_to_goal(double goal_x, double goal_y, double xs_f, double ys_f, double angle_s, int pw_l, int pw_r, int& pw_l_new, int& pw_r_new);

// Set detour point around obstacle
int detour(double xo, double yo, double obs_rad, double margin, double angle_r, double& detour_x, double& detour_y);

// Rotate to face detour
int rotate_to_detour(double detour_x, double detour_y, double xs_f, double ys_f, double angle_s, int pw_l, int pw_r, int& pw_l_new, int& pw_r_new);

// Check if robots are aligned along a line
int line_aligned(double xs, double ys, double xor_, double yor, bool& aligned);

// Attack function (master control for robot attack)
void attack(
    int gx, int gy, int rx, int ry,
    int ox, int oy, int bx, int by,
    double width1, double height1,
    double& pw_l, double& pw_r, double& laser
);

