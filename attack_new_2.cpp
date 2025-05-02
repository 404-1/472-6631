#include <iostream>
#include <fstream>
#include <cmath>
#include <Windows.h>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "attack_new_2.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int angle_distance(double x1, double y1, double x2, double y2, double &distance, double &angle) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    distance = sqrt(dx * dx + dy * dy);
    angle = atan2(dy, dx) * (180.0 / 3.1415);
    if (angle < 0.0) angle += 360.0;
    if (angle >= 360.0) angle -= 360;
    return 0;
}

int projection_blocked(double xs, double ys, double xor_, double yor, double xo, double yo, double obs_rad, bool &clear_angle) {
    double dx = xor_ - xs;
    double dy = yor - ys;
    double dx_o = xo - xs;
    double dy_o = yo - ys;

    double proj_factor = (dx * dx_o + dy * dy_o) / (dx * dx + dy * dy);
    double proj_x = proj_factor * dx;
    double proj_y = proj_factor * dy;

    double p_x = xs + proj_x;
    double p_y = ys + proj_y;

    double dx_p = p_x - xo;
    double dy_p = p_y - yo;

    double dist_p = sqrt(dx_p * dx_p + dy_p * dy_p);

    if (dist_p <= (obs_rad + 20)) clear_angle = false;
    else clear_angle = true;

    return 0;
}

int edge_detection(double width1, double height1, double th, double xs_f, double ys_f, bool &boundary_detected) {
    double dist_l = xs_f;
    double dist_r = width1 - xs_f;
    double dist_b = ys_f;
    double dist_f = height1 - ys_f;

    if (dist_l <= th || dist_r <= th || dist_b <= th || dist_f <= th)
        boundary_detected = true;
    else
        boundary_detected = false;

    return 0;
}

int clockwise(double &pw_l_new, double &pw_r_new) {
    pw_l_new = 1000;
    pw_r_new = 1000;
    return 0;
}

int counterclockwise(double &pw_l_new, double &pw_r_new) {
    pw_l_new = 2000;
    pw_r_new = 2000;
    return 0;
}

int move_straight(double &pw_l_new, double &pw_r_new) {
    pw_l_new = 1000;
    pw_r_new = 2000;
    return 0;
}

int rotate_to_opponent(double angle_s, double angle_r, bool &aligned, double &pw_l_new, double &pw_r_new) {
   /* if (abs(angle_s - angle_r) <= 10) {
        aligned = true;
        move_straight(pw_l_new, pw_r_new);
    }
    else if (abs(angle_s - angle_r) > 0) {
        aligned = false;
        counterclockwise(pw_l_new, pw_r_new);
    }
    else {
        aligned = false;
        clockwise(pw_l_new, pw_r_new);
    }
    return 0;*/

    double difference = angle_r - angle_s;

    // Normalize angle to [-180, 180]
    if (difference > 180) difference -= 360;
    if (difference < -180) difference += 360;

    if (abs(difference) <= 10) {
        aligned = true;
        move_straight(pw_l_new, pw_r_new);
    }
    else if (difference > 0) {
        aligned = false;
        counterclockwise(pw_l_new, pw_r_new);
    }
    else {
        aligned = false;
        clockwise(pw_l_new, pw_r_new);
    }

    return 0;
}

int rotate_to_center(double width, double height, double xs, double ys, double angle_s, double &pw_l_new, double &pw_r_new) {
    double center_x = width / 2;
    double center_y = height / 2;
    double dx = center_x - xs;
    double dy = center_y - ys;

    double angle = atan2(dy, dx) * (180.0 / 3.1415);
    if (angle < 0) angle += 360.0;

    double difference = angle - angle_s;
    if (difference < 0) difference += 360.0;

    if (abs(difference) >= 50.0) counterclockwise(pw_l_new, pw_r_new);
    else move_straight(pw_l_new, pw_r_new);
   
    return 0;
}

int goal(double xs_f, double ys_f, double xor_f, double yor_f, double ratio, double &goal_x, double &goal_y, double &dist_goal, double &angle_goal) {
    double dx_r = xor_f - xs_f;
    double dy_r = yor_f - ys_f;

    goal_x = xs_f + ratio * dx_r;
    goal_y = ys_f + ratio * dy_r;

    double dx = goal_x - xs_f;
    double dy = goal_y - ys_f;

    dist_goal = sqrt(dx * dx + dy * dy);
    angle_goal = atan2(dy, dx) * (180.0 / 3.1415);
    if (angle_goal < 0) angle_goal += 360.0;

    return 0;
}

int rotate_to_goal(double goal_x, double goal_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double &pw_l_new, double &pw_r_new) {
    double dx = goal_x - xs_f;
    double dy = goal_y - ys_f;

    double angle = atan2(dy, dx) * (180.0 / 3.1415);
    if (angle < 0.0) angle += 360.0;

    if (angle_s <= angle - 10.0) {
        counterclockwise(pw_l_new, pw_r_new);
    }
    else if (angle_s >= angle + 10.0) {
        clockwise(pw_l_new, pw_r_new);
    }
    else {
        move_straight(pw_l_new, pw_r_new);
    }

    return 0;
}

int detour(double xo, double yo, double obs_rad, double margin, double angle_r, double &detour_x, double &detour_y) {
    double angle_r_rad = angle_r * (3.1415 / 180.0);
    double ninety_deg_rad = 90.0 * (3.1415 / 180.0);

    detour_x = xo + (obs_rad + margin) * cos(angle_r_rad + ninety_deg_rad);
    detour_y = yo + (obs_rad + margin) * sin(angle_r_rad + ninety_deg_rad);

    return 0;
}

int rotate_to_detour(double detour_x, double detour_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double &pw_l_new, double &pw_r_new) {
    double dx = detour_x - xs_f;
    double dy = detour_y - ys_f;

    double angle = atan2(dy, dx) * (180.0 / 3.1415);
    if (angle < 0.0) angle += 360.0;

    if (angle_s <= angle - 10.0) {
        counterclockwise(pw_l_new, pw_r_new);
    }
    else if (angle_s >= angle + 10.0) {
        clockwise(pw_l_new, pw_r_new);
    }
    else {
        move_straight(pw_l_new, pw_r_new);
    }

    return 0;
}

int line_aligned(double xs, double ys, double xor_, double yor, bool &aligned) {
    double dx = xor_ - xs;
    double dy = yor - ys;

    double m = dy / dx;
    double b = ys - (m * xs);

    double y1 = xor_ * m + (b + 10);
    double y2 = xor_ * m + (b - 10);

    if (y1 < yor && y2 > yor)
        aligned = true;
    else
        aligned = false;

    return 0;
}
