#ifndef BRUSHFIRE_H
#define BRUSHFIRE_H

    //-------------------------------------------------------------------------------------------------------//

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <mutex>

class Brushfire
{
public:
    Brushfire();
    void CreateBrushfire();
    void PathPlanning();
    void afstikker(std::vector<cv::Point> Marbles);

    std::vector<cv::Vec2d> return_corners();
    std::vector<cv::Point2d> getPacMan_coords();

private:
    void afstikker(cv::Point);
    cv::Point offset(int x, int y);

    cv::Mat im;
    cv::Mat binary;
    cv::Mat binary1;
    cv::Mat skeleton;
    cv::Mat g_image;

    std::vector<cv::Point> sorted_points;
    std::vector<cv::Vec2d> sorted_corners;
    std::vector<cv::Point2d> PacMan_coords;

    cv::Point2d holder_til_koords;
    std::mutex mutex;
};

    //-------------------------------------------------------------------------------------------------------//

#endif // BRUSHFIRE_H
