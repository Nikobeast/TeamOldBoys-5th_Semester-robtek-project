#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

#include <laserscanner.h>
#include <fuzzyobstacleavoidance.h>
#include <fuzzybugcontroller.h>
#include <imageprocess.h>
#include <brushfire.h>
#include <movement.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <ratio>

//using namespace  std;
static boost::mutex mutex;

LaserScanner laser;

cv::Mat im;
ImageProcess myprocess;
Brushfire myBush;
movement move;
std::vector<cv::Vec2d>holder2;

bool time2 = false;
bool fuzzy_bool = true;
int num_image_before_save = 6;

int images_saved = num_image_before_save;

//static boost::mutex mutex;




void gazebo_start_seq()
{
    system("~/Documents/Projekt/github/clone_repository/gz.txt");
    sleep(5);
    return;
}

void gazebo_shutdown_seq(){
     system("~/Documents/Projekt/github/clone_repository/kill-gz.txt");
     sleep(5);
     return;
}

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

double x_pose = 0;
double y_pose = 0;
double x_qpose =0;
double y_qpose =0;
double z_pose = 0;
double w_pose = 0;


void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
   // pik = 1;

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {


       x_pose = _msg->pose(i).position().x();
       y_pose = _msg->pose(i).position().y();
       x_qpose= _msg->pose(i).orientation().x();
       y_qpose= _msg->pose(i).orientation().y();
       z_pose = _msg->pose(i).orientation().z();
       w_pose = _msg->pose(i).orientation().w();


     /* std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;*/
    }
  }
}






void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
  //abe++;


 // im = im.clone();



  cv::cvtColor(im, im, CV_BGR2RGB);
  //if (abe == 10)
 // {
 // cv::cvtColor(im, im, CV_BGR2RGB);
  myprocess.loadIm(im);
  //myprocess.hough_circles(); // hough-detection

  myprocess.process();   // our own detection
  myprocess.calc_ball_placement(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose);

  //abe = 0;
 // }
/*  mutex.lock();
  cv::namedWindow("camera", cv::WINDOW_NORMAL);

  cv::imshow("camera", im);
  mutex.unlock();*/
}

void lidarCallback(ConstLaserScanStampedPtr &msg) {

   laser.parseLaserScannerMessage(msg);




  float angle_min = float(msg->scan().angle_min());
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
       // double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);

        //std::cout << angle << " " << range << " " << intensity << std::endl;
  }
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

 /* mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();*/
}


void landing_destinations(std::vector<cv::Vec2d> &landing, int c, int y)
{
    cv::Vec2d next_point = {0,4};
    landing.push_back(next_point);
    next_point = {-7+c,4};
    landing.push_back(next_point);
    next_point = {-8+c,1};
    landing.push_back(next_point);
    next_point = {-5+c,0+y};
    landing.push_back(next_point);
    next_point = {0+c,0+y};
    landing.push_back(next_point);
    holder2.push_back(next_point);
}


void save_img()
{

    images_saved = 0;

    myprocess.path_length();

    myprocess.save_path(holder2[0]);
    holder2.erase(holder2.begin());


    std::cout << "ready to reset export" << std::endl;
}

int main(int _argc, char **_argv) {
int counter=2;




    while(counter)
{
bool all_check_pts_done = false;

while(!all_check_pts_done){


    im = cv::imread("../Image/floor_plan.png",cv::IMREAD_GRAYSCALE);
    myprocess.generateVoteSpace(im);

    //mutex.lock();
    std::thread first(gazebo_start_seq);
    first.detach();
    //mutex.unlock();



  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);


  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);


  //FUZZY LOGIC LITE OBJECT
    FuzzyBugController fuzzy_Pac_Man(&laser);
    FuzzyObstacleAvoidance fuzzy(&laser);


  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);


  myprocess.generateWorld();



  double newspeed = 0;
  double newdir = 0;
  bool PacManMode = false;
  bool draw = true;
  int taeller = 0;

    std::vector<cv::Vec2d> destinations;
   // cv::Vec2d next_point;

   /* for(int i = 8; i <= 15; i++)
    {
        for(int j = -4; j <= 4; j++ )
        {
            if(j != 0)
            {
                cv::Vec2d next_point ={i,j};
                destinations.push_back(next_point);
                landing_destinations(destinations);
            }
        }
    }*/
    int value = 0;
    //for(int i = 0; i < 40 ; i++)
    //{
        int count = 0;
        holder2.push_back({0,0});
        for(int i = 0; i < 100; i++)
        {
            for(int j = -2; j <= 2; j++)
            {

            cv::Vec2d next_point ={12,0};
            destinations.push_back(next_point);
            landing_destinations(destinations, count,j);
            }
            count--;
            if(count < -10)
                count = 0;
        }

    //}






  sleep(1);


  int export_image = 1;

  bool time1 = true;
  time_t begin,end; // time_t is a datatype to store time values.
  double difference;
  int x_temp;
  int y_temp;
  cv::Vec2d holder = {0,0};
 // int start_counter = 0;

  while (true) {
    //gazebo::common::Time::MSleep(10);
    mutex.lock();
    int keuy = cv::waitKey(1);
    mutex.unlock();


    if((abs(x_pose - destinations[0][0])<=0.25)&&(abs(y_pose-destinations[0][1])<=0.25))
    {

        holder = destinations[0];
        destinations.erase(destinations.begin());
        std::cout << "den næste destination er: " << destinations[0] <<"," <<destinations[1] << std::endl;

        images_saved++;
            cv::Vec2d temp = {12,0};
            if(holder == temp)
            {
                cv::Vec2d new_point = {6, 4};
                destinations.insert(destinations.begin(),new_point);
                save_img();
            }

            myprocess.generateWorld();

                     //overkrydset indsættes som det næste og images_saved--
        std::cout << "Checkpoint found! " << images_saved << std::endl;
        //export_image = 1;

        /*taeller++;
        if (destinations.size() == 0)
        {
        draw = false;
        }*/

    }

    if(((abs(x_pose - holder[0])>=0.25)||(abs(y_pose-holder[1])>=0.25))&& export_image == 2)
    {
                export_image = 1;
              //  std::cout << "export reset" << std::endl;
    }



    if(time1)
    {
        x_temp = x_pose;
        y_temp = y_pose;
        time1 = false;
        time (&begin); // note time before execution
    }

    time (&end); // note time after execution
    difference = difftime (end,begin);

    if(difference > 1 && fuzzy_bool == false)
    {
        fuzzy_bool = true;
    }

    if(difference > 5.0)
    {
       // std::cout << "tiden virker " << x_pose - x_temp << ", " << y_pose-y_temp << std::endl;

        if(abs((x_pose - x_temp)<=0.46)&&(abs(y_pose-y_temp<=0.25)))
        {



             if(export_image == 1)
             {
                 if(images_saved == num_image_before_save)
                 {
                     time2 = true;
                     fuzzy_bool = false;
                     save_img();
                 }
                 else
                 {

                 }
                 std::cout << "images saved: " << images_saved << std::endl;
                 export_image = 2;
             }

        }
        time1 = true;
    }

    if(time2)
    {
        newspeed = -0.5;
        newdir = -1;


        time2 = false;
        cv::Vec2d next_point ={3,1};
        destinations.erase(destinations.begin());
        destinations.insert(destinations.begin(),next_point);
        holder = destinations[0];
        std::cout << "den næste destination er: " << destinations[0] <<"," <<destinations[1] << std::endl;
    }





    if((destinations.size() != 0 && !PacManMode) && fuzzy_bool == true)
    {
        //std::cout << fuzzy_bool << "<- kig " << std::endl;
        fuzzy.buildController(move.getGoal(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, destinations[0]));
        newspeed = fuzzy.getControlOutput().speed;
        newdir = fuzzy.getControlOutput().direction;
    }


        ignition::math::Pose3d pose(newspeed, 0, 0, 0, 0, newdir);



        myprocess.draw_path(x_pose,y_pose, newspeed, draw);//KIIIIIIGGG HER


    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
}
}
