void get_angle(double x1, double y1, double x2, double y2, double& angle); 

void get_distance(double x1, double y1, double x2, double y2, double& distance); 

void get_robo_midpoint(double x1, double y1, double x2, double y2, double& position_x, double& position_y); 

bool has_line_of_sight(double x1, double y1, double x2, double y2, const double obs_ic[], const double obs_jc[], const double obs_r[], int obs_count); 

bool is_in_obstacle(double x, double y, double obs_ic[], double obs_jc[], double obs_r[], int obs_count); 

bool find_vantage_point(
	int self_x,
	int self_y,
	int opp_x,
	int opp_y,
	double obs_ic[],
	double obs_jc[],
	double obs_r[],
	int obs_count,
	int& vantage_x,
	int& vantage_y); 

bool pointturn_to_angle(double currentDeg, double targetDeg, double& pw_l, double& pw_r); 

void go_to(int self_x, //goes to point in straight line
	int self_y,
	double currentDeg,
	int target_x,
	int target_y,
	double& pw_l,
	double& pw_r); 

bool find_detour_point(int self_x, int self_y,
	int goal_x, int goal_y,
	double obs_ic[], double obs_jc[], double obs_r[],
	int obs_count,
	int& detour_x, int& detour_y); 

void go_to_withavoidance(int self_x,
	int    self_y,
	double currentDeg,
	int    goal_x,
	int    goal_y,
	double& pw_l,
	double& pw_r,
	double obs_ic[], double obs_jc[], double obs_r[],
	int    obs_count); 

void run_attack(int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx, int& by, double obs_ic[],
	double  obs_jc[], double obs_r[], int obs_count, double& pw_l, double& pw_r, double& laser);