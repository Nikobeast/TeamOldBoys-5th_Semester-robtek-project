#include "imageprocess.h"

ImageProcess::ImageProcess()
{
    balls.push_back({0,0,0});
    save_path_int = 0;
}
void ImageProcess::loadIm(cv::Mat &im)
{
   im_proc = im.clone();
   origin = im;
   ball_placement = im.clone();
}

int ImageProcess::return_midt()
{
    int abe = balls[0][0];
    //std::cout << "HEEEEEJ: " << abe << std::endl;
    return abe;

}

void ImageProcess::hough_circles()
{
   // cv::cvtColor(ball_placement, ball_placement, CV_RGB2GRAY);
    balls.clear();
    cv::cvtColor(ball_placement, ball_placement, CV_BGR2HLS);
    cv::split(ball_placement,HLS_clone);
    cv::inRange(HLS_clone[0],cv::Scalar(100),cv::Scalar(140),HLS_clone[0]);



    //cv::GaussianBlur(HLS_clone[0], HLS_clone[0], cv::Size(9,9),2,2);
    cv::GaussianBlur(HLS_clone[0], HLS_clone[0], cv::Size(3,3),1.5,1.5);
    cv::HoughCircles(HLS_clone[0], circles, CV_HOUGH_GRADIENT, 1, HLS_clone[0].rows*0+5, 50, 15,0 ,0);
    double radius;
    for (int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        radius = cvRound(circles[i][2]);
        cv::circle(origin,center,2, cv::Scalar(0,0,255), 1, 8, 0);
        cv::circle(origin, center, radius, cv::Scalar(0,0,255), 1, 8 ,0);
        balls.push_back(cv::Vec3d( double(circles[i][0]),double(circles[i][1]),radius*2));
    }

   // cv::imshow("display",HLS_clone[0]);

}

void ImageProcess::calc_ball_placement(double x, double y, double qx, double qy, double qz, double qw)
{
    if (balls.size() == 0)
        return;
    double x_angle;
    double dist;
    double siny_cosp;
    double cosy_cosp;
    double phi;
    int x_out;
    int y_out;
    int weight;
    for(int i = 0; i < balls.size(); i++)
    {
   // std::cout << "Balls #2 element: " << balls[i][2] << std::endl;
    x_angle = ((balls[i][0] - CAMERA_PIXEL_WIDTH/2)/(CAMERA_PIXEL_WIDTH/1.047));
    //std::cout << "x_angle: " << x_angle << std::endl;
    //std::cout << "angle: " << atan(4.0/10.0)*180/(acos(-1)) << std::endl;
    dist = 280/balls[i][2];
    siny_cosp = +2.0 * (qw * qz + qx * qy);
    cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
    phi = atan2(siny_cosp, cosy_cosp);
    //std::cout << phi;
    phi-=x_angle;
    //std::cout << " , " << phi << std::endl;
    //std::cout << x << " , " << y << " , " << dist << " , " << phi << " , " << cos(phi) << " , " << sin(phi) << " , ";
    x_out = round(x+cos(phi)*dist);
    y_out = round(y+sin(phi)*dist);
    if((voteSpace.at<uchar>(cv::Point(x_out+(voteSpace.cols/2),(voteSpace.rows/2)-y_out))<248) && ( abs(x_out) < voteSpace.cols/2 ) && (abs(y_out) < voteSpace.rows/2))
    {
        weight = 7 - dist/5;
        if (weight > 0)
        voteSpace.at<uchar>(cv::Point(x_out+(voteSpace.cols/2),(voteSpace.rows/2)-y_out))+=weight;
    }

    }
   /* mutex_variable.lock();
    cv::namedWindow("Votespace", cv::WINDOW_NORMAL);
     cv::imshow("Votespace",voteSpace);
     mutex_variable.lock();*/
}

void ImageProcess::generateVoteSpace(cv::Mat &im){
    voteSpace = im;
    cv::resize(voteSpace, voteSpace, cv::Size(),(double) 7/10,(double) 7/10, cv::InterpolationMasks());
    //std::cout << "cols: " << voteSpace.cols << ", rows: " << voteSpace.rows << std::endl;
    for(int i=0; i<voteSpace.rows; i++){
        for (int j=0; j<voteSpace.cols;j++){
            voteSpace.at<uchar>(i,j)=0;
           // std::cout << "cols: " << voteSpace.cols << ", rows: " << voteSpace.rows << std::endl;
        }
    }

}


std::vector<cv::Point> ImageProcess::get_ball_placement()
{

    return ball_koord;

}

void ImageProcess::analyse_votespace(){
    cv::Mat voteSpace_clone;
    voteSpace_clone = voteSpace.clone();


 /*   for (int y = 0; y < voteSpace_clone.size().height-1;y++)
    {
        uchar* ptr = voteSpace_clone.ptr<uchar>(y);
        uchar* ptr_next = voteSpace_clone.ptr<uchar>(y+1);

        for (int x = 0; x < voteSpace_clone.size().width-1; x++)
        {
            if(ptr[x] > 50)
            {
                if ((ptr[x] < ptr[x+1]) || (ptr[x] < ptr_next[x]) || (ptr[x] < ptr_next[x+1]))
                    ptr[x] = 0;
                else
                    ptr[x] = 255;
            }
            else
                ptr[x] = 0;

        }
    }

    for (int y = voteSpace_clone.size().height-1; y > 0; y--)
    {
        uchar* ptr = voteSpace_clone.ptr<uchar>(y);
        uchar* ptr_prior = voteSpace_clone.ptr<uchar>(y-1);

        for (int x = voteSpace_clone.size().width-1; x > 0; x--)
        {
            if(ptr[x] > 50)
            {
                if ((ptr[x] < ptr[x-1]) || (ptr[x] < ptr_prior[x]) || (ptr[x] < ptr_prior[x-1]))
                    ptr[x] = 0;
                else
                    ptr[x] = 255;
            }
            else
                ptr[x] = 0;

        }
    }

*/



    //-------------------------------------------------
    for (int y = 0; y < voteSpace_clone.size().height; y++)
    {
        uchar* ptr = voteSpace_clone.ptr<uchar>(y);
       // if (y != 0 && (y != voteSpace_clone.size().height-1))

        uchar* ptrplus = voteSpace_clone.ptr<uchar>(y+1);
        uchar* ptrminus = voteSpace_clone.ptr<uchar>(y-1);

        for(int x = 0; x < voteSpace_clone.size().width; x++)
        {
            if (y == 0 || y == voteSpace_clone.size().height-1 || x == 0 || x == voteSpace_clone.size().width-1)
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



    /*cv::Mat grad_x, grad_y,grad_xy;
        cv::Mat abs_grad_x, abs_grad_y, abs_grad_xy;
        int ddepth = CV_16S;
        int scale = 1;
        int delta = 0;
        int ksize = 1;
        cv::Mat grad;
        cv::Size s;
        s = voteSpace_clone.size();


        //cv::GaussianBlur( binary, binary, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

        // Gradient X
        cv::Sobel( voteSpace_clone, grad_x, ddepth, 1, 0, ksize, scale, delta, cv::BORDER_DEFAULT);
        // Gradient Y
        cv::Sobel( voteSpace_clone, grad_y, ddepth, 0, 1, ksize, scale, delta, cv::BORDER_DEFAULT);




        cv::convertScaleAbs( grad_x, abs_grad_x );
        cv::convertScaleAbs( grad_y, abs_grad_y );



        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

*/
   /* mutex_variable.lock();

    cv::namedWindow("NEW VOTE SPACE", cv::WINDOW_NORMAL);

    cv::imshow("NEW VOTE SPACE", voteSpace_clone);



    mutex_variable.unlock();*/

    tilter = voteSpace_clone;

}

void ImageProcess::random(double length, double time)
{

     std::ofstream mystream;
     mystream.open("wanker.txt");

     mystream << "x = [";
     mystream << time;

     /*
     for(int i = 0; i < tilter.size().height; i++)
     {
         for(int j = 0; j < tilter.size().width; j++)
         {

                if(tilter.at<uchar>(i,j) != 0)
                {
                    mystream << j-tilter.size().width/2;
                    mystream << " ";
                }

         }
     }*/
     mystream << "]; y = [";


     /*
     for(int i = 0; i < tilter.size().height; i++)
     {
         for(int j = 0; j < tilter.size().width; j++)
         {

                if(tilter.at<uchar>(i,j) != 0)
                {
                    mystream << tilter.size().height/2-i;
                    mystream << " ";
                }

         }
     }*/





     mystream << length;
     mystream << "];";

     mystream.close();

   // cv::namedWindow("HOUGH", cv::WINDOW_NORMAL);
   // cv::imshow("HOUGH", tilter);

}
void ImageProcess::generateWorld()
{
    int x = 85*10;
    int y = 56*10;

    cv::Mat M(y, x, CV_8UC3, cv::Scalar(0,0,0));
    Path = M.clone();

    std::cout << "generate World" << std::endl;

    //cv::Mat temp = cv::imread("../Image/drawn_path.png");

   /* for(int i = 0; i < temp.size().height; i++)
    {
        for(int j = 0; j < temp.size().width; j++)
        {
            cv::Vec3b hep = temp.at<cv::Vec3b>(i,j);
            Path.at<cv::Vec3b>(i,j) = hep;
        }
    }*/
}

void ImageProcess::draw_path(double x, double y, double speed, bool draw)
{
   // draw_walls(6,1); //BRUGES TIL FEJL TEST
    if (draw)
    {
    x_offset = (x*10)+Path.cols/2;
    y_offset = Path.rows/2 - (y*10);

    cv::cvtColor(Path,Path,CV_BGR2HSV);

   // std::cout << "Speed: " << speed << std::endl;
    int generated_speed = 150*speed; //215******************************************************************
    Path.at<cv::Vec3b>(y_offset,x_offset) = {generated_speed,255,168};
    cv::cvtColor(Path, Path, CV_HSV2BGR);
   /* mutex_variable.lock();

    cv::namedWindow("Display", cv::WINDOW_NORMAL);
    cv::imshow("Display", Path);
    mutex_variable.unlock();*/

    }
    else
        return;

}


void ImageProcess::save_path(cv::Vec2d end_point)
{
    draw_walls(end_point[0],end_point[1]); //BRUGES TIL TESTS DER VIRKER ************************************************************
    std::string text = "../Image/drawn_Paths";
    text+= std::to_string(save_path_int);
    text+=".png";
    save_path_int++;
    cv::imwrite(text, Path);
}



double ImageProcess::path_length()
{

    double path_length = 0;
    cv::Vec3b mandingo = {0, 0, 0};
    for (int i = 0; i < Path.size().height; i++)
    {
        for (int j = 0; j < Path.size().width; j++)
        {
           // std::cout << (int)Path.at<cv::Vec3b>(i,j)[0] << std::endl;
            if (Path.at<cv::Vec3b>(i,j) != mandingo)
               {
                path_length++;
               }
        }
    }
    std::cout << "Path length: " << path_length << std::endl;
    std::cout << "Path length divided by 10: " << path_length/10 << std::endl;

    return path_length/10;

}

void ImageProcess::draw_walls(double x_coord, double y_coord)
{
    cv::Point offset_koord = {12*10+Path.cols/2, Path.rows/2};//start
    cv::circle(Path,offset_koord,5,cv::Scalar(0,0,255),1);

    offset_koord = {x_coord*10+Path.cols/2, -1*y_coord*10+Path.rows/2}; //målet
    cv::circle(Path,offset_koord,5,cv::Scalar(0,255,0),1);
//-------------------------
    offset_koord = {3.5*10+Path.cols/2,Path.rows/2-2.8*10}; //gray wall punkt 1

    cv::Point offset_koord1 = {8.5*10+Path.cols/2,Path.rows/2+2.6*10}; //gray wall punkt 2

    cv::line(Path,offset_koord,offset_koord1,cv::Scalar(100,100,100),2);
//-------------------------
    offset_koord = {3.5*10+Path.cols/2,Path.rows/2+2.6*10};

    offset_koord1 = {8.5*10+Path.cols/2,Path.rows/2-2.8*10};

    cv::line(Path,offset_koord,offset_koord1,cv::Scalar(100,100,100),2);

    /*for(int i = (Path.rows/2+(-wall_y*10-4*10)); i < (Path.rows/2+(-wall_y*10+4*10)); i++)
    {
        for (int j = Path.cols/2+wall_x*10; j < Path.cols/2+wall_x*10+3; j++)
        {
            Path.at<cv::Vec3b>(i,j) = {100, 100, 100};
        }
    }
    */
    offset_koord = {-5*10+Path.cols/2,4.5*10+Path.rows/2};
    cv::Point offset_koord_1 = {20*10+Path.cols/2,4.5*10+Path.rows/2};
    cv::line(Path,offset_koord,offset_koord_1,cv::Scalar(255,255,255),2);

    offset_koord = {-5*10+Path.cols/2,-5*10+Path.rows/2};
    offset_koord_1 = {20*10+Path.cols/2,-5*10+Path.rows/2};
    cv::line(Path,offset_koord,offset_koord_1,cv::Scalar(255,255,255),2);
    /*mutex_variable.lock();

    cv::namedWindow("Display", cv::WINDOW_NORMAL);
    cv::imshow("Display", Path);
    mutex_variable.unlock();*/

}

void ImageProcess::process()
{
    balls.clear();  // ikke empty()
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
                        balls.push_back(cv::Vec3d(j,center,abs(min-max)));

                marbles_found++;
                cv::circle(HLS_clone[0],cv::Point(j,center),abs(((min-max)/2))*1.5,100,-1); // tilføjer en størrer circle. + 0.5 gange sig selv
                    }
                }
              }
              else
              {
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

  //  cv::namedWindow("Display", cv::WINDOW_NORMAL);

    //        cv::imshow("Display", HLS_clone[0]);


}


