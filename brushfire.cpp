#include "brushfire.h"




    //-------------------------------------------------------------------------------------------------//
Brushfire::Brushfire()
{

    im = cv::imread("../Image/floor_plan2.png",cv::IMREAD_GRAYSCALE);

    cv::resize(im, im, cv::Size(), 15, 15, cv::InterpolationMasks());

}
    //-------------------------------------------------------------------------------------------------//

void Brushfire::CreateBrushfire()
{
    cv::threshold(im, binary, 100, 255, CV_THRESH_BINARY);

    cv::distanceTransform(binary,binary,CV_DIST_L1,3,cv::DIST_LABEL_PIXEL);
    cv::normalize(binary,binary,0,255,cv::NORM_MINMAX, CV_8UC1);
    cv::Mat distanceTransform = binary;
    cv::resize(distanceTransform,distanceTransform,cv::Size(),0.5,0.5,cv::InterpolationMasks());
    mutex.lock();
    cv::imshow("Distance transform", distanceTransform);
    mutex.unlock();
    g_image = binary.clone();
    PathPlanning();
}

    //-------------------------------------------------------------------------------------------------//
struct myclass {
    bool operator() (cv::Vec2d pt1, cv::Vec2d pt2) { return (pt1[0] < pt2[0]);}
} sortByX;
    //-------------------------------------------------------------------------------------------------//
struct myclass2 {
    bool operator() (cv::Vec2d pt1, cv::Vec2d pt2) { return (pt1[1] < pt2[1]);}
} sortByY;
    //-------------------------------------------------------------------------------------------------//
void Brushfire::PathPlanning()
{
    cv::Mat grad_x, grad_y, grad_xy, abs_grad_x, abs_grad_y, abs_grad_xy, gradient_image, element_holder, image_holder;
    cv::Size image_size;
    std::vector<cv::Vec4i> points;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contour_holder;
    std::vector<cv::Vec4i> hierarchy_holder;
    int ddepth = CV_16S;
    int scale = 4;
    int delta = 3;
    int k_size = 5;
    int white = 255;
    int dilation_size;
    int border_size = 80;
    int place;
    long long biggest;

    //-------------------------------------------------------------------------------------------------------//

    cv::Sobel(binary,grad_x,ddepth,2,0,k_size,scale,delta,cv::BORDER_DEFAULT);
    cv::Sobel(binary,grad_y,ddepth,0,2,k_size,scale,delta,cv::BORDER_DEFAULT);

    cv::convertScaleAbs(grad_x,abs_grad_x);
    cv::convertScaleAbs(grad_y,abs_grad_y);

    cv::addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,1,gradient_image);

    image_holder = binary.clone();

    cv::threshold(image_holder,image_holder,28,white,CV_THRESH_BINARY);

    cv::bitwise_and(gradient_image,image_holder,gradient_image);

    cv::threshold(gradient_image,gradient_image,36,white,CV_THRESH_BINARY);


    //-----------------------------------------------------------------------------------------------------//

    cv::Laplacian(binary,grad_xy,ddepth,k_size);

    cv::convertScaleAbs(grad_xy,abs_grad_xy);

    cv::threshold(abs_grad_xy,abs_grad_xy,70,white,CV_THRESH_BINARY);

/*
    cv::Mat viss = abs_grad_xy;
    cv::resize(viss,viss,cv::Size(),0.5,0.5,cv::InterpolationMasks());
    cv::imshow("Visnings",viss);
*/
    //----------------------------------------------------------------------------------------------------//

    dilation_size = 10;

    element_holder = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*dilation_size+1,2*dilation_size+1),cv::Point(dilation_size,dilation_size));

    cv::dilate(abs_grad_xy,abs_grad_xy,element_holder);


    //----------------------------------------------------------------------------------------------------//
    /// Turns border of image black.


    image_size = gradient_image.size();

    for(int i = 0; i < border_size; i++)
    {
        for(int j = 0; j < image_size.height; j++)
        {
            gradient_image.at<uchar>(cv::Point(i,j)) = 0;
        }
    }
    for(int i = image_size.width-border_size; i < image_size.width; i++)
    {
        for(int j = 0; j < image_size.height; j++)
        {
            gradient_image.at<uchar>(cv::Point(i,j)) = 0;
        }
    }
    for(int i = 0; i < border_size; i++)
    {
        for(int j = 0; j < image_size.width; j++)
        {
            gradient_image.at<uchar>(cv::Point(j,i)) = 0;
        }
    }
    for(int i = image_size.height-border_size; i < image_size.height; i++)
    {
        for(int j = 0; j < image_size.width; j++)
        {
            gradient_image.at<uchar>(cv::Point(j,i)) = 0;
        }
    }

    //------------------------------------------------------------------------------------------------------//

    cv::HoughLinesP(gradient_image,points,1,CV_PI/2,6);

    dilation_size = 10;

    ///Visualization of Hough Lines' endpoints:
/*
    cv::Mat test_color = gradient_image.clone();
    cv::cvtColor(test_color,test_color,CV_GRAY2BGR);
    for(int i = 0 ; i < points.size(); i++)
    {
        cv::Point p1 = {points[i][0],points[i][1]};
        cv::Point p2 = {points[i][2],points[i][3]};
        cv::circle(test_color,p1,10,cv::Scalar(0,0,255));
        cv::circle(test_color,p2,10,cv::Scalar(0,0,255));
    }
*/

    cv::bitwise_and(gradient_image,abs_grad_xy,gradient_image);

    cv::dilate(gradient_image,gradient_image,element_holder);

    image_holder = gradient_image.clone();

    for(int i = 0 ; i < points.size(); i++)
    {
        cv::Point p1 = {points[i][0],points[i][1]};
        cv::Point p2 = {points[i][2],points[i][3]};
        if(image_holder.at<uchar>(p1) == 255 && image_holder.at<uchar>(p2)== 255)
        {
             cv::line(gradient_image,p1,p2,cv::Scalar(255),80);
        }
    }

    //-------------------------------------------------------------------------------------------------------//

    cv::findContours(gradient_image,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));

    biggest = cv::contourArea(contours[0]);
    for(size_t j = 0; j < contours.size(); j++)
    {
        if(biggest < cv::contourArea(contours[(int)j]))
        {
            biggest = cv::contourArea(contours[(int)j]);
            place = (int)j;
        }
    }
    contour_holder.push_back(contours[place]);
    hierarchy_holder.push_back(hierarchy[place]);

    image_holder = cv::Mat::zeros(image_size, CV_8UC1);
   for(size_t i = 0; i < contour_holder.size(); i++)
    {
        cv::drawContours(image_holder,contour_holder,(int)i,cv::Scalar(white),CV_FILLED,8,hierarchy_holder,0,cv::Point());
    }

    cv::bitwise_and(gradient_image,image_holder,gradient_image);

    //-------------------------------------------------------------------------------------------------------//

    for(int i = 10; i > 0 ; i--)
    {

    cv::ximgproc::thinning(gradient_image,gradient_image,cv::ximgproc::THINNING_GUOHALL);

    dilation_size = i;

    element_holder = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*dilation_size+1,2*dilation_size+1),cv::Point(dilation_size,dilation_size));

    cv::dilate(gradient_image,gradient_image,element_holder);
    }

    //--------------------------------------------------------------------------------------------------------//
    skeleton = gradient_image.clone();

    cv::Mat combined = im.clone();

        for(int i = 0; i < combined.size().height; i++)
        {
            for(int j = 0; j < combined.size().width; j++)
            {
                if(gradient_image.at<uchar>(i,j) != 0)
                    combined.at<uchar>(i,j) = 20;
            }
        }

    std::vector<cv::Vec2d> corners;
    cv::goodFeaturesToTrack(gradient_image,corners,80,0.1,100);

    cv::Mat test_color = gradient_image.clone();
    cv::cvtColor(test_color,test_color,CV_GRAY2BGR);
    for(int i = 0 ; i < corners.size(); i++)
    {
        //cv::Point p1 = {corners[i][0],points[i][1]};

       // cv::circle(test_color,corners[i],10,cv::Scalar(0,0,255));
    }

    std::sort(corners.begin(), corners.end(), sortByX);

    //-------------------------------------------------------------------------------------------------//
    /// placement of circles sorted by X value, with colors changing each step in X.

    cv::circle(test_color,cv::Point(corners[0][0],corners[0][1]),10,cv::Scalar(0,0,255));
    cv::circle(test_color,cv::Point(corners[1][0],corners[1][1]),10,cv::Scalar(0,255,0));
    cv::circle(test_color,cv::Point(corners[2][0],corners[2][1]),10,cv::Scalar(255,0,0));
    cv::circle(test_color,cv::Point(corners[3][0],corners[3][1]),10,cv::Scalar(0,255,255));
    cv::circle(test_color,cv::Point(corners[4][0],corners[4][1]),10,cv::Scalar(255,0,255));
    cv::circle(test_color,cv::Point(corners[5][0],corners[5][1]),10,cv::Scalar(255,255,0));
    cv::circle(test_color,cv::Point(corners[6][0],corners[6][1]),10,cv::Scalar(255,255,255));

    cv::circle(test_color,cv::Point(corners[7][0],corners[7][1]),10,cv::Scalar(0,0,255));
    cv::circle(test_color,cv::Point(corners[8][0],corners[8][1]),10,cv::Scalar(0,255,0));
    cv::circle(test_color,cv::Point(corners[9][0],corners[9][1]),10,cv::Scalar(255,0,0));
    cv::circle(test_color,cv::Point(corners[10][0],corners[10][1]),10,cv::Scalar(0,255,255));
    cv::circle(test_color,cv::Point(corners[11][0],corners[11][1]),10,cv::Scalar(255,0,255));
    cv::circle(test_color,cv::Point(corners[12][0],corners[12][1]),10,cv::Scalar(255,255,0));
    sorted_corners = corners;




    cv::Mat vis = test_color;
    cv::resize(vis,vis,cv::Size(),0.8,0.8,cv::InterpolationMasks());
    cv::imshow("Connectivity map",vis);


    //-------------------------------------------------------------------------------------------------------//

}

std::vector<cv::Vec2d> Brushfire::return_corners()
{

    for (int i = 0; i < sorted_corners.size(); i++)
    {
        sorted_corners[i][0] = (((sorted_corners[i][0] - skeleton.size().width/2))/(1.41874*15));
        sorted_corners[i][1] = ((skeleton.size().height/2 - sorted_corners[i][1])/(1.41874*15));
    }
    return sorted_corners;
}

void Brushfire::afstikker(cv::Point h)
{

    //cv::Point h = {170,280};

    cv::Point r = h;

    //std::cout << "marbles: " << h << std::endl;
    std::vector<cv::Point> list_of_points;

    //cv::circle(g_image,h,5,cv::Scalar(255,0,0),2);

    int pixel_value;
    cv::Point placement;
    pixel_value = g_image.at<uchar>(h);
    int counter = 0;
    int matrixVal = 1;
    while(1 && counter < 200)
    {

    if(g_image.at<uchar>({h.x-matrixVal,h.y}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y});
        placement = {h.x-matrixVal,h.y};
    }

    if(g_image.at<uchar>({h.x+matrixVal,h.y}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y});
        placement = {h.x+matrixVal,h.y};
    }

    if(g_image.at<uchar>({h.x,h.y-matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x,h.y-matrixVal});
        placement = {h.x,h.y-matrixVal};
    }

    if(g_image.at<uchar>({h.x,h.y+matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x,h.y+matrixVal});
        placement = {h.x,h.y+matrixVal};
    }

    if(g_image.at<uchar>({h.x-matrixVal,h.y+matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y+matrixVal});
        placement = {h.x-matrixVal,h.y+matrixVal};
    }

    if(g_image.at<uchar>({h.x+matrixVal,h.y+matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y+matrixVal});
        placement = {h.x+matrixVal,h.y+matrixVal};
    }

    if(g_image.at<uchar>({h.x-matrixVal,h.y-matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y-matrixVal});
        placement = {h.x-matrixVal,h.y-matrixVal};
    }

    if(g_image.at<uchar>({h.x+matrixVal,h.y-matrixVal}) > pixel_value)
    {
        pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y-matrixVal});
        placement = {h.x+matrixVal,h.y-matrixVal};
    }


    list_of_points.push_back(h);
    list_of_points.push_back(placement);

    h = placement;

    if(skeleton.at<uchar>(placement) == 255)
        break;
    counter++;
    }


    int size = list_of_points.size();
    cv::line(skeleton,list_of_points[0],list_of_points[size-1],cv::Scalar(255),4);
    cv::circle(skeleton,r,4,cv::Scalar(255),4);

   /* std::cout << "0, x: " << list_of_points[0].x << std::endl;
    std::cout << "0, y: " << list_of_points[0].y << std::endl;
    std::cout << "size-1, x: " << list_of_points[size-1].x << std::endl;
    std::cout << "size-1, y: " << list_of_points[size-1].y << std::endl;
    std::cout << "R: " << r << std::endl;*/



    //holder_til_koords.x = ((list_of_points[size-1].x/15)*(17/24)-85/2);
    //holder_til_koords.y = (56/2 - (list_of_points[size-1].y/15)*(7/10));

    /*list_of_points[size-1].x = ((17*list_of_points[size-1].x)/(360)-42.5);
    list_of_points[size-1].y = (28-(7*list_of_points[size-1].y)/150);
    PacMan_coords.push_back(list_of_points[size-1]);

    list_of_points[0].x = ((17*list_of_points[0].x)/(360)-42.5);
    list_of_points[0].y = (28-(7*list_of_points[0].y)/150);
    PacMan_coords.push_back(list_of_points[0]);*/



    list_of_points[size-1].x = ((list_of_points[size-1].x - skeleton.size().width/2)/(15*1.41874));
    list_of_points[size-1].y = ((skeleton.size().height/2 - list_of_points[size-1].y)/(15*1.41874));
    PacMan_coords.push_back(list_of_points[size-1]);

    list_of_points[0].x = ((list_of_points[0].x - skeleton.size().width/2)/(15*1.41874));
    list_of_points[0].y = ((skeleton.size().height/2 - list_of_points[0].y)/(15*1.41874));
    PacMan_coords.push_back(list_of_points[0]);


    //std::cout << "0, x: " << list_of_points[0].x << std::endl;
    //std::cout << "0, y: " << list_of_points[0].y << std::endl;
    //std::cout << "size-1, x: " << list_of_points[size-1].x << std::endl;
    //std::cout << "size-1, y: " << list_of_points[size-1].y << std::endl;

    //std::cout << "BLA: " << list_of_points[size-1] << std::endl;


}

std::vector<cv::Point2d> Brushfire::getPacMan_coords()
{
    return PacMan_coords;
}

void Brushfire::afstikker(std::vector<cv::Point> Marbles)
{
   /* std::vector<cv::Point> list;

    list.push_back(offset(cv::Point(10,4)));
    list.push_back(offset(cv::Point(-3,-1)));
    list.push_back(offset(cv::Point(20,23)));
    list.push_back(offset(cv::Point(18,7)));
    list.push_back(offset(cv::Point(28,12)));
    list.push_back(offset(cv::Point(32,23)));
    list.push_back(offset(cv::Point(31,7)));
    list.push_back(offset(cv::Point(27,-5)));


    for(int i = 0; i < list.size(); i++)
        afstikker(list[i]);


*/
    /*afstikker({10*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 4*15*10/7});
    afstikker({-3*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-1)*15*10/7});
    afstikker({20*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 23*15*10/7});
    afstikker({18.5*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 7*15*10/7});
    afstikker({28*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 12*15*10/7});
    afstikker({32*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 23*15*10/7});
    afstikker({31*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - 7*15*10/7});
    afstikker({27*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-5)*15*10/7});
    afstikker({18*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-7)*15*10/7});
    afstikker({10*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-6)*15*10/7});
    afstikker({8.5*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-6.5)*15*10/7});
    afstikker({-9*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-13)*15*10/7});
    afstikker({39*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-15)*15*10/7});
    afstikker({41*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-22)*15*10/7});
    afstikker({25*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-23)*15*10/7});
    afstikker({18*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-17)*15*10/7});
    afstikker({6*15*24/17+skeleton.size().width/2,skeleton.size().height/2 - (-19)*15*10/7});*/

    int resize_x = 15*1.41874;
    int resize_y = 15*1.41874;
    int offset_x = skeleton.size().width/2;
    int offset_y = skeleton.size().height/2;

   // std::cout << skeleton.size().width << " LOL WIDTH: " << std::endl;
   // std::cout << skeleton.size().height << " LOL HØJ: " << std::endl;
    for (int i = 0; i < Marbles.size(); i++)
    {
        afstikker(cv::Point(Marbles[i].x*resize_x, Marbles[i].y*resize_y));
        //std::cout << "i: " << i << "; marbles: " << cv::Point(Marbles[i].x*resize_x, Marbles[i].y*resize_y) << std::endl;
    }
    cv::Mat ImageHolder = skeleton.clone();
    //cv::resize(pikholder,pikholder,cv::Size(),double(0.5))
    cv::resize(skeleton,ImageHolder,cv::Size(),0.5,0.5,cv::InterpolationMasks());

    cv::imshow("Connect marbles and Voronoi",ImageHolder);


}

cv::Point Brushfire::offset(int x, int y)
{
    cv::Point point_holder;

    point_holder = {0,0};// {p.x*15 + skeleton.size().width/2, skeleton.size().height/2 - p.y*15};

    return point_holder;
}
