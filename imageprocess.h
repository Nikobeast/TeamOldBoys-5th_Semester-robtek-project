#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include<mutex>

class ImageProcess
{
public:
    ImageProcess();
    void loadIm(cv::Mat &im);
    void process();
    void printData();
    int return_midt();
    void calc_ball_placement(double x, double y, double qx, double qy, double qz, double qw);
    std::vector<cv::Point> get_ball_placement();
    void hough_circles();
    void generateVoteSpace(cv::Mat &im);
    cv::Mat voteSpace;
    void analyse_votespace();
    void random(double length, double time);
    void generateWorld();
    void draw_path(double x, double y, double speed, bool draw);
    double path_length();
    void draw_walls(double wall_x, double wall_y);
    void save_path(cv::Vec2d);


private:
    cv::Mat Path;
    cv::Mat im_proc;
    cv::Mat origin;
    cv::Mat tilter;
    cv::Mat ball_placement;
    int marbles_found;
    std::vector <cv::Vec3d>balls;
    cv::Mat HLS_clone[3];
    int max;
    int min;
    int center;
    cv::Size s;
    std::vector<cv::Point> ball_koord;
    double CAMERA_PIXEL_WIDTH = 320;
    std::vector<cv::Vec3f> circles;
    std::vector<cv::Point> estimated_marble_koord;
    double x_offset;
    double y_offset;
    int save_path_int;

    std::mutex mutex_variable;


};

#endif // IMAGEPROCESS_H
