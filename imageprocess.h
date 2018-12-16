#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

    //-------------------------------------------------------------------------------------------------------//

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

class ImageProcess
{
public:
    ImageProcess();
    void loadIm(cv::Mat &im);
    void process();
    void printData();
    void calc_ball_placement(double x, double y, double qx, double qy, double qz, double qw);
    void hough_circles();
    void generateVoteSpace(cv::Mat &im);
    void analyse_votespace();
    void random(double length, double time);
    void generateWorld();
    void draw_path(double x, double y, double speed, bool draw);
    int Homing();
    double path_length();

    std::vector<cv::Point> get_ball_placement();
    std::vector<cv::Point> get_Marbles_location();
    cv::Point3d center_marble(double x, double y, double qx, double qy, double qz, double qw, double lidar, std::vector<cv::Vec2d> dest);
    cv::Vec2d calc_ball_placement1(double x, double y, double qx, double qy, double qz, double qw, double lidar);
    cv::Mat voteSpace;

    int Get_Biggest_marble(double x, double y, double qx, double qy, double qz, double qw);
private:
    int marbles_found, max, min, center;
    int MarbleDiameter = 1;

    double x_offset, y_offset;
    double CAMERA_PIXEL_WIDTH = 320;

    bool marble_is_centered = false;

    cv::Mat Path;
    cv::Mat im_proc;
    cv::Mat origin;
    cv::Mat ball_placement;
    cv::Mat HLS_clone[3];
    cv::Mat Blue_marble;

    std::vector<cv::Vec3d> balls;
    std::vector<cv::Point> ball_koord;
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point> estimated_marble_koord;
    std::vector<cv::Point> Marbles_location;

    cv::Vec2d updated_marble_koord;
    cv::Size s;

};

    //-------------------------------------------------------------------------------------------------------//

#endif // IMAGEPROCESS_H
