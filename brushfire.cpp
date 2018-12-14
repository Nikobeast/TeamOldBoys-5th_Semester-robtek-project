#include "brushfire.h"

Brushfire::Brushfire()
{

    im = cv::imread("../Image/floor_plan.png",cv::IMREAD_GRAYSCALE);

    cv::resize(im, im, cv::Size(), 15, 15, cv::InterpolationMasks());

    cv::imwrite( "../Image/floor_plan_resized.png", im );

    counter_skeleton = 0;

}



std::vector<cv::Point> Brushfire::random_marbles()
{



    cv::Mat im_copy = cv::imread("../Image/floor_plan_gray.png",cv::IMREAD_GRAYSCALE);
    //cv::resize(im_copy,im_copy,cv::Size(), 0.0667,0.0667,cv::InterpolationMasks());
    std::vector<cv::Point> ball_placement;

    cv::threshold(im_copy,im_copy,150,255,CV_THRESH_BINARY);

    while (ball_placement.size() < 20)
    {
        int x = rand() % im_copy.size().width;
        int y = rand() % im_copy.size().height;

        if(im_copy.at<uchar>(y,x) == 255)
        {
            cv::Point holder = {x,y};
            im_copy.at<uchar>(holder) = 0;

            holder = {x*15+8,y*15+8};

            ball_placement.push_back(holder);
        }
    }


    //cv::resize(im_copy,im_copy,cv::Size(), 10,10,cv::InterpolationMasks());

   // cv::imshow("random marble",im_copy);

    return ball_placement;
}



void Brushfire::CreateBrushfire()
{
    cv::threshold(im, binary, 100, 255, CV_THRESH_BINARY);

    cv::distanceTransform(binary,binary,CV_DIST_L1,3,cv::DIST_LABEL_PIXEL);
    cv::normalize(binary,binary,0,255,cv::NORM_MINMAX, CV_8UC1);

    cv::imshow("Distance transform", binary);

    g_image = binary.clone();
    std::cout << "Hej " << std::endl;

    PathPlanning();
}


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
    int erosion_size;
    int border_size = 80;
    int place;
    long long smallest;
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

    /*cv::Mat viss = abs_grad_xy;
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

    skeleton = gradient_image.clone();
    //std::cout << skeleton.size().width <<" <- wiff, " << skeleton.size().height << "<- height" << std::endl;

    //--------------------------------------------------------------------------------------------------------//

    cv::Mat combined = im.clone();

        for(int i = 0; i < combined.size().height; i++)
        {
            for(int j = 0; j < combined.size().width; j++)
            {
                if(gradient_image.at<uchar>(i,j) != 0)
                    combined.at<uchar>(i,j) = 20;
            }
        }


     /*   cv::Mat vis = combined;
        cv::resize(vis,vis,cv::Size(),0.5,0.5,cv::InterpolationMasks());
        cv::imshow("Visning",vis);*/


    //-------------------------------------------------------------------------------------------------------//


        std::vector<cv::Point> corners;

        cv::goodFeaturesToTrack(gradient_image,corners,80,0.1,100);



        cv::Mat test_color = gradient_image.clone();
        cv::cvtColor(test_color,test_color,CV_GRAY2BGR);
        for(int i = 0 ; i < corners.size(); i++)
        {
            cv::circle(test_color,corners[i],10,cv::Scalar(0,0,255),5);
        }



    //----------------------------------------------------------------------------------------------------//



        cv::Mat vis = test_color;

      //  cv::resize(vis,vis,cv::Size(),0.067,0.067,cv::InterpolationMasks());
        cv::imshow("Visning",vis);
}

void Brushfire::afstikker2(cv::Point h, int z)
{

    //cv::Point h = {170,280};

    cv::Point r = h;


    std::vector<cv::Point> list_of_points;

    //cv::circle(g_image,h,5,cv::Scalar(255,0,0),2);

    int pixel_value;
    cv::Point placement;
    pixel_value = g_image.at<uchar>(h);
    int counter = 0;
    int tempHolder = pixel_value;
    int matrixVal = 2;

    int newPixel = skeleton.at<uchar>(h);

    if(newPixel != 255)
    {
        while(1 && counter < 200)
        {
        tempHolder = pixel_value;
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

        if(tempHolder == pixel_value)
        {
            if(g_image.at<uchar>({h.x-matrixVal,h.y}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y});
                placement = {h.x-matrixVal,h.y};
            }

            if(g_image.at<uchar>({h.x+matrixVal,h.y}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y});
                placement = {h.x+matrixVal,h.y};
            }

            if(g_image.at<uchar>({h.x,h.y-matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x,h.y-matrixVal});
                placement = {h.x,h.y-matrixVal};
            }

            if(g_image.at<uchar>({h.x,h.y+matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x,h.y+matrixVal});
                placement = {h.x,h.y+matrixVal};
            }

            if(g_image.at<uchar>({h.x-matrixVal,h.y+matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y+matrixVal});
                placement = {h.x-matrixVal,h.y+matrixVal};
            }

            if(g_image.at<uchar>({h.x+matrixVal,h.y+matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y+matrixVal});
                placement = {h.x+matrixVal,h.y+matrixVal};
            }

            if(g_image.at<uchar>({h.x-matrixVal,h.y-matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x-matrixVal,h.y-matrixVal});
                placement = {h.x-matrixVal,h.y-matrixVal};
            }

            if(g_image.at<uchar>({h.x+matrixVal,h.y-matrixVal}) == pixel_value)
            {
                pixel_value = g_image.at<uchar>({h.x+matrixVal,h.y-matrixVal});
                placement = {h.x+matrixVal,h.y-matrixVal};
            }

        }




        list_of_points.push_back(h);
        list_of_points.push_back(placement);

        h = placement;

        if(skeleton.at<uchar>(placement) == 255)
            break;
        counter++;
            if(counter == 200)
            {
                //cv::circle(skeleton,h,4,cv::Scalar(150),5);

                //std::cout << z << "<- iteration, counter_skeleton:    " <<counter_skeleton << std::endl;
                counter_skeleton++;
            }
        }

    }
    else
    {
        list_of_points.push_back(h);
        list_of_points.push_back(h);


    }

        //int size = list_of_points.size();

    //cv::line(skeleton,list_of_points[0],list_of_points[size-1],cv::Scalar(150),2);
    //cv::circle(skeleton,r,4,cv::Scalar(200),4);

    return;
}

void Brushfire::small()
{

    srand(time(NULL));

    return;

}


void Brushfire::afstikker(int h)
{
    std::vector<cv::Point> list = random_marbles();


    for(int i = 0; i < list.size(); i++)
        afstikker2(list[i], h);


    return;


}

cv::Point Brushfire::offset(cv::Point p)
{
    cv::Point point_holder;

    point_holder = {0,0};//

    return point_holder;
}

long Brushfire::returnValue()
{
    long holder = counter_skeleton;
    counter_skeleton = 0;
    return holder;
}

