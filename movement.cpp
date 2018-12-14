#include "movement.h"

    //-------------------------------------------------------------------------------------------------------//

movement::movement()
{

}
    //-------------------------------------------------------------------------------------------------------//

bool movement::collect_marbles(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Point2d nextDest)
{
    if (!marble_direction)
    {
        eulerMove_calc(x_pose, y_pose, x_qpose, y_qpose, z_qpose, w_qpose, nextDest);
        eulerMove_trans();
        return true;
    }
    else
        return false;
}
    //-------------------------------------------------------------------------------------------------------//

cv::Vec2d movement::getGoal(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Vec2d nextDest){
    double siny_cosp,cosy_cosp,angle,dist;
    x = x_pose;
    y = y_pose;
    dest_x = nextDest[0];
    dest_y = nextDest[1];

    //-------------------------------Quaternions to Eulers------------------------------------------------------//
    siny_cosp = 2.0 * (w_qpose * z_qpose + x_qpose * y_qpose);
    cosy_cosp = 1.0 - 2.0 * (y_qpose * y_qpose + z_qpose * z_qpose);
    phi_robot = (atan2(siny_cosp, cosy_cosp));
    //--------------------------------------------//

    theta_robot_2_goal = (atan2((dest_y - y),(dest_x - x)));
    dist = sqrt(pow(dest_y - y,2)+pow(dest_x - x,2));
    angle = theta_robot_2_goal - phi_robot;
    if (angle > CV_PI)
        angle -= 2*CV_PI;
    else if(angle < -CV_PI) // "<=" ? ************************KIG*****************************
        angle += 2*CV_PI;
    return cv::Vec2d(angle,dist);
}
    //-------------------------------------------------------------------------------------------------------//

void movement::eulerMove_calc(double x_pose, double y_pose, double x_qpose, double y_qpose, double z_qpose, double w_qpose, cv::Point nextDest)
{
    double siny_cosp,cosy_cosp;
    x = x_pose;
    y = y_pose;

    //-------------------------------Quaternions to Eulers------------------------------------------------------//
    siny_cosp = 2.0 * (w_qpose * z_qpose + x_qpose * y_qpose);
    cosy_cosp = 1.0 - 2.0 * (y_qpose * y_qpose + z_qpose * z_qpose);
    phi_robot = (atan2(siny_cosp, cosy_cosp) * RAD_2_DEG);
    //--------------------------------------------//

    if (phi_robot < 0)
        phi_robot += 360;
    dest_x = nextDest.x;
    dest_y = nextDest.y;
    theta_robot_2_goal = (atan2((dest_y - y),(dest_x - x))) * RAD_2_DEG;
    if (theta_robot_2_goal < 0)
        theta_robot_2_goal += 360;
    if(((phi_robot < theta_robot_2_goal) && (abs(phi_robot-theta_robot_2_goal) < 180))
            || ((phi_robot > theta_robot_2_goal) && (abs(phi_robot-theta_robot_2_goal) > 180 )))
        turn_left=true;
    else
        turn_left=false;
}
    //-------------------------------------------------------------------------------------------------------//

bool movement::eulerMove_trans()
{
    speed = 0;
    double diff = abs(phi_robot - theta_robot_2_goal);

    if (diff >180)
        diff = abs(diff-360);

    if( diff >= 5)
    {
        speed = 0;
        if (turn_left)
            dir = -0.5;
         else
            dir = 0.5;
    }
    else
    {
        dir = 0;
        marble_direction = true;
        return true;
    }
    return false;
}
    //-------------------------------------------------------------------------------------------------------//

void movement::update_marble_bool()
{
    marble_direction = false;
}
    //-------------------------------------------------------------------------------------------------------//

double movement::getSpeed()
{
    return speed;
}
    //-------------------------------------------------------------------------------------------------------//

double movement::getDir()
{
    return dir;
}
    //-------------------------------------------------------------------------------------------------------//


