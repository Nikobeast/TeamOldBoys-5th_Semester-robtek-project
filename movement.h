#ifndef MOVEMENT_H
#define MOVEMENT_H

    //-------------------------------------------------------------------------------------------------------//
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>

class movement
{
public:
    movement();
    void eulerMove_calc(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Point nextDest);
    void update_marble_bool();

    double getSpeed();
    double getDir();

    bool eulerMove_trans();
    bool collect_marbles(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Point2d nextDest);

    cv::Vec2d getGoal(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Vec2d nextDest);

private:
    double theta_robot_2_goal, phi_robot, x, y, dest_x, dest_y;
    double RAD_2_DEG = 180/(acos(-1));
    double dir = 0;
    double speed = 0;

    bool turn_left;
    bool marble_direction = false;

    cv::Vec3d rotate_values;
};
    //-------------------------------------------------------------------------------------------------------//

#endif // MOVEMENT_H
