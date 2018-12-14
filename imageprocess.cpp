#include "imageprocess.h"
    //-------------------------------------------------------------------------------------------------//

ImageProcess::ImageProcess()
{
    balls.push_back({0,0,0});
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::loadIm(cv::Mat &im)
{
   im_proc = im.clone();
   origin = im;
   ball_placement = im.clone();
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::hough_circles()
{
    double radius;
    balls.clear();

    cv::cvtColor(ball_placement, ball_placement, CV_BGR2HLS);
    cv::split(ball_placement,HLS_clone);
    cv::inRange(HLS_clone[0],cv::Scalar(100),cv::Scalar(140),HLS_clone[0]);

    cv::GaussianBlur(HLS_clone[0], HLS_clone[0], cv::Size(3,3),1.5,1.5);
    cv::HoughCircles(HLS_clone[0], circles, CV_HOUGH_GRADIENT, 1, HLS_clone[0].rows*0+5, 50, 15,0 ,0);


    for (int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        radius = cvRound(circles[i][2]);
        cv::circle(origin,center,2, cv::Scalar(0,0,255), 1, 8, 0);
        cv::circle(origin, center, radius, cv::Scalar(0,0,255), 1, 8 ,0);
        balls.push_back(cv::Vec3d( double(circles[i][0]),double(circles[i][1]),radius*2));
    }
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::calc_ball_placement(double x, double y, double qx, double qy, double qz, double qw)
{
    if (balls.size() == 0)
        return;
    double x_angle, dist, siny_cosp, cosy_cosp, phi_robot;
    int x_out, y_out, weight;

    for(int i = 0; i < balls.size(); i++)
    {

        x_angle = ((balls[i][0] - CAMERA_PIXEL_WIDTH/2)/(CAMERA_PIXEL_WIDTH/1.047));

        dist = 280/balls[i][2];

        //-------------------------------Quaternions to Eulers--------------------------------------------//
        siny_cosp = +2.0 * (qw * qz + qx * qy);
        cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
        phi_robot = atan2(siny_cosp, cosy_cosp);
        //-----------------------------------------------
        phi_robot -= x_angle;

        x_out = round(x+cos(phi_robot) * dist);
        y_out = round(y+sin(phi_robot) * dist);
        if((voteSpace.at<uchar>(cv::Point(x_out + (voteSpace.cols/2),(voteSpace.rows/2) - y_out)) < 251)
                && ( abs(x_out) < voteSpace.cols/2 ) && (abs(y_out) < voteSpace.rows/2))
        {
            weight = 7 - dist/5;
            if (weight > 0)
            voteSpace.at<uchar>(cv::Point(x_out + (voteSpace.cols/2),(voteSpace.rows/2) - y_out)) += weight;
        }
    }
    cv::namedWindow("Votespace", cv::WINDOW_NORMAL);
    cv::imshow("Votespace",voteSpace);
}
    //-------------------------------------------------------------------------------------------------//

cv::Vec2d ImageProcess::calc_ball_placement1(double x, double y, double qx, double qy, double qz, double qw, double lidar)
{

    double dist, siny_cosp, cosy_cosp, phi_robot, x_out, y_out;

    dist = lidar + MarbleDiameter;

    //-------------------------------Quaternions to Eulers--------------------------------------------//
    siny_cosp = 2.0 * (qw * qz + qx * qy);
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    phi_robot = atan2(siny_cosp, cosy_cosp);
    //-----------------------------------------------

    x_out = x + cos(phi_robot) * dist;
    y_out = y + sin(phi_robot) * dist;
    return cv::Vec2d(x_out,y_out);
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::generateVoteSpace(cv::Mat &im){
    voteSpace = im;
    cv::resize(voteSpace, voteSpace, cv::Size(),(double) 1/1.41874,(double) 1/1.41874, cv::InterpolationMasks());
    for(int i=0; i<voteSpace.rows; i++){
        for (int j=0; j<voteSpace.cols;j++){
            voteSpace.at<uchar>(i,j) = 0;
        }
    }
}
    //-------------------------------------------------------------------------------------------------//

std::vector<cv::Point> ImageProcess::get_ball_placement()
{
    return ball_koord;
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::analyse_votespace(){
    cv::Mat voteSpace_clone;
    voteSpace_clone = voteSpace.clone();

    //-------------------------------------------------
    for (int y = 0; y < voteSpace_clone.size().height; y++)
    {
        uchar* ptr = voteSpace_clone.ptr<uchar>(y);
        uchar* ptrplus = voteSpace_clone.ptr<uchar>(y + 1);
        uchar* ptrminus = voteSpace_clone.ptr<uchar>(y - 1);

        for(int x = 0; x < voteSpace_clone.size().width; x++)
        {
            if ( y == 0 || y == voteSpace_clone.size().height-1 || x == 0 || x == voteSpace_clone.size().width-1 )
                ptr[x] = 0;

            if(ptr[x] > 50)
            {

                                  int smallest = ptr[x];

                                  int et1 = ptrminus[x];
                                  if (smallest < et1)
                                  {
                                  smallest = et1;
                                  }
                                  int to1 = ptrminus[x-1];
                                  if (smallest < to1)
                                  {
                                      smallest = to1;
                                  }
                                  int tre1 = ptr[x-1];
                                  if (smallest < tre1)
                                  {
                                      smallest = tre1;
                                  }
                                  int fire1 = ptrplus[x];
                                  if (smallest < fire1)
                                  {
                                      smallest = fire1;
                                  }
                                  int fem1 = ptrplus[x+1];
                                  if (smallest < fem1)
                                  {
                                      smallest = fem1;
                                  }
                                  int seks1 = ptr[x+1];
                                  if (smallest < seks1)
                                  {
                                      smallest = seks1;
                                  }
                                  int syv1 = ptrminus[x+1];
                                  if (smallest < syv1)
                                  {
                                      smallest = syv1;
                                  }
                                  int otte1 = ptrplus[x-1];
                                  if (smallest < otte1)
                                  {
                                      smallest = otte1;
                                  }
                                  if (smallest != ptr[x])
                                    ptr[x] = 0;
                                  else
                                    ptr[x] = 255;


            }
            else
            ptr[x] = 0;
        }
    }

    cv::namedWindow("ANALYSED VOTESPACE", cv::WINDOW_NORMAL);
    cv::imshow("ANALYSED VOTESPACE", voteSpace_clone);

    for (int i = 0; i < voteSpace_clone.size().width; i++)
    {
        for (int j = 0; j < voteSpace_clone.size().height; j++)
        {
            if (voteSpace_clone.at<uchar>(cv::Point(i,j)) == 255)
                Marbles_location.push_back(cv::Point(i,j));
        }
    }
}
    //-------------------------------------------------------------------------------------------------//

cv::Point3d ImageProcess::center_marble(double x, double y, double qx, double qy, double qz, double qw, double lidar, std::vector<cv::Vec2d> dest)
{
    double marble_center_point = balls[0][0];

    if (!marble_is_centered && marble_center_point != 0)
    {
        if ( (abs(marble_center_point - (CAMERA_PIXEL_WIDTH/2)) <= 1) )
        {
            marble_is_centered = true;
            updated_marble_koord = calc_ball_placement1(x, y, qx, qy, qz, qw, lidar);
            cv::Vec2d xy_coord = {x,y};
           // dest.insert(dest.begin(),xy_coord);
           // dest.insert(dest.begin(),updated_marble_koord);
            return {0.2, 0, 0};

        }
        else
            {
                if ((marble_center_point - (CAMERA_PIXEL_WIDTH/2)) < 0)
                    return {0, -0.1, 0};

                else
                    return {0, 0.1, 0};
             }
    }

    if ( (abs(x - updated_marble_koord[0]) >= 0.05) && (abs(y - updated_marble_koord[1])>= 0.05) && marble_is_centered )
    {
        return {0.3, 0, 0};
    }
    else
    {
        marble_is_centered = false;
        return {0, 0, 1};
    }
}
    //-------------------------------------------------------------------------------------------------//

std::vector<cv::Point> ImageProcess::get_Marbles_location()
{
    return Marbles_location;
}
    //-------------------------------------------------------------------------------------------------//


void ImageProcess::random(double length, double time)
{

     std::ofstream mystream;
     mystream.open("wanker.txt");

     mystream << "x = [";
     mystream << time;

     mystream << "]; y = [";
     mystream << length;
     mystream << "];";

     mystream.close();
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::generateWorld()
{
    int x = 85*10;
    int y = 56*10;

    cv::Mat M(y, x, CV_8UC3, cv::Scalar(0,0,0));
    Path = M.clone();
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::draw_path(double x, double y, double speed, bool draw)
{
    if (draw)
    {
        x_offset = (x * 10) + Path.cols/2;
        y_offset = Path.rows/2 - (y * 10);

        cv::cvtColor(Path,Path,CV_BGR2HSV);

        int generated_speed = 215 * speed;
        Path.at<cv::Vec3b>(y_offset,x_offset) = {generated_speed,255,168};
        cv::cvtColor(Path, Path, CV_HSV2BGR);

        cv::namedWindow("Drawn path", cv::WINDOW_NORMAL);
        cv::imshow("Drawn path", Path);
    }
    else
        return;

}
    //-------------------------------------------------------------------------------------------------//

double ImageProcess::path_length()
{

    double path_length = 0;
    cv::Vec3b mandingo = {0, 0, 0};
    for (int i = 0; i < Path.size().height; i++)
    {
        for (int j = 0; j < Path.size().width; j++)
        {
            if (Path.at<cv::Vec3b>(i,j) != mandingo)
               {
                path_length++;
               }
        }
    }
   // std::cout << "Path length: " << path_length << std::endl;
   // std::cout << "Path length divided by 10: " << path_length/10 << std::endl;

    return path_length/10;
}
    //-------------------------------------------------------------------------------------------------//

void ImageProcess::process()
{
    balls.clear();
    marbles_found = 0;
    s = im_proc.size();
    cv::cvtColor(im_proc, im_proc, CV_BGR2HLS);
    cv::split(im_proc,HLS_clone);
    cv::inRange(HLS_clone[0],cv::Scalar(80),cv::Scalar(160),HLS_clone[0]);


    for (int i = 0; i < s.height;i++)
    {
        uchar* ptr = HLS_clone[0].ptr<uchar>(i);
        for (int j = 0; j < s.width; j++)
        {
            if(ptr[j] == 255)
            {
              if (!((i == 0) || (i == s.height)))
              {
                max = i;
                for (int r = j; r < s.width; r++)
                {
                    if (ptr[r] != 255)
                    {
                        j = ((r - j)/2) + j;

                        break;
                    }
                }
                for (int q = s.height - 1; q > i+1; q--)
                {
                    if (HLS_clone[0].at<uchar>(q,j) == 255)
                    {
                        min = q;
                        q = i;
                    }
                }
                center = ((min - max)/2) + max;
                if (abs(min-max) < s.height)
                {
                    if(abs(min-max) > 5)
                    {
                        marbles_found++;
                        balls.push_back(cv::Vec3d(j,center,abs(min-max)));
                        cv::circle(HLS_clone[0],cv::Point(j,center),abs(((min-max)/2))*1.5,100,-1);
                    }
                }
              }
              else
              {
                  balls.push_back(cv::Vec3d(s.width/2,s.height/2,280));
                  marbles_found++;
                  cv::circle(HLS_clone[0],cv::Point(j,center),abs(((min-max)/2))*1.5,100,-1);
                  i = s.height;
                  break;
              }

            }

        }
    }
    for (int i= 0;i<marbles_found;i++)
    {

        cv::circle(origin,cv::Point(balls[i][0],balls[i][1]),(balls[i][2]/2)+2,cv::Scalar(0,255,0),1); // tilføjer en størrer circle.
        cv::circle(origin,cv::Point(balls[i][0],balls[i][1]),2,cv::Scalar(0,255,0),1);

    }
}
    //-------------------------------------------------------------------------------------------------//


