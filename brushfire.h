#ifndef BRUSHFIRE_H
#define BRUSHFIRE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ctime>

class Brushfire
{
public:
    Brushfire();
    void CreateBrushfire2();
    void CreateBrushfire();
    void PathPlanning();
    int pixelValue(int, int);
    void CreateBrushfirePTR();
    int pixelValuePTR(int, int);
    void sobel_func();
    void wavefront();
    void laplacian();
    void new_sobel();
    void afstikker(int);
    void afstikker2(cv::Point, int);
    cv::Point offset(cv::Point p);
    std::vector<cv::Point> random_marbles();
    void small();
    long returnValue();

private:
    cv::Mat im;
    cv::Mat g_image;
    cv::Mat binary;
    cv::Mat binary1;
    cv::Mat skeleton;
    std::vector<cv::Point> sorted_points;
    long counter_skeleton;
};

#endif // BRUSHFIRE_H
