    //------------------------------Includes----------------------------------------------------------------------//
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <laserscanner.h>
#include <fuzzyobstacleavoidance.h>
#include <imageprocess.h>
#include <brushfire.h>
#include <movement.h>
#include <thread>
#include <chrono>
#include <qlearning.h>
#include <ctime>
#include <ratio>
static boost::mutex mutex;

    //------------------------------Initialization of Object-----------------------------------------------------------//

LaserScanner laser;
cv::Mat im;
ImageProcess myprocess;
Brushfire myBush;
movement move;
Qlearning myQ_learning;

    //------------------------------GAZEBO-STARTUP-----------------------------------------------------------//

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
    //-------------------------------------------------------------------------------------------------------//

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}
    //-------------------------------------------------------------------------------------------------------//

double x_pose = 0;
double y_pose = 0;
double x_qpose =0;
double y_qpose =0;
double z_pose = 0;
double w_pose = 0;

    //-------------------------------------------------------------------------------------------------------//

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx")
        {
           x_pose = _msg->pose(i).position().x();
           y_pose = _msg->pose(i).position().y();
           x_qpose= _msg->pose(i).orientation().x();
           y_qpose= _msg->pose(i).orientation().y();
           z_pose = _msg->pose(i).orientation().z();
           w_pose = _msg->pose(i).orientation().w();
        }
  }
}
    //-------------------------------------------------------------------------------------------------//

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    //------------------------------------------------//
  cv::cvtColor(im, im, CV_BGR2RGB);
  myprocess.loadIm(im);

  myprocess.hough_circles(); // Hough circle detection
  //myprocess.process();     // our own detection algorithm

  myprocess.calc_ball_placement(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose);
    //------------------------------------------------//
  mutex.lock();
  cv::namedWindow("camera", cv::WINDOW_NORMAL);

  cv::imshow("camera", im);
  mutex.unlock();
}

    //-----------------------------------------------------------------------------------------------//

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
    //------------------------------------------------//
  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);

    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);

  }
    //------------------------------------------------//
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

    //----------------------------Initialize Path Planning----------------------------------------------//
int main(int _argc, char **_argv) {

    //------------------------------------------------//
    myBush.CreateBrushfire();
    myQ_learning.Q_learning();
    int counter = 2;
    int destinations_size_holder = 0;
    std::vector<cv::Vec2d> Corners = myBush.return_corners();
    std::vector<int> Corner_path = myQ_learning.return_path();
    std::vector<cv::Vec2d> destinations;
    std::vector<cv::Vec2d> holder_destinations;

    //------------------------------------------------//

    for (int i = 0; i < Corner_path.size(); i++)
        {
            destinations.push_back(Corners[Corner_path[i]]);
            holder_destinations.insert(holder_destinations.begin(),Corners[Corner_path[i]]);
        }
    destinations.insert(destinations.end(),holder_destinations.begin(),holder_destinations.end());
    destinations_size_holder = destinations.size();
    

    //-------------------------Generate Image for VoteSpace----------------------------------------------//


    im = cv::imread("../Image/floor_plan.png",cv::IMREAD_GRAYSCALE);
    myprocess.generateVoteSpace(im);


   //-------------------------While-Loop for multiply gazebo startups------------------------------------//
while(counter){

    bool all_check_pts_done = false;

while(!all_check_pts_done){

    //------------------------Automatical Gazebo startup sequence----------------------------------------//

    mutex.lock();
    std::thread first(gazebo_start_seq);
    first.detach();
    mutex.unlock();
    myprocess.generateWorld();


  //---------------------------Load Gazebo & Subscriber---------------------------------------------------//

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

  //---------------------------Gazebo Publisher-------------------------------------------------------//
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


  //----------------------------FUZZY LOGIC LITE OBJECT---------------------------------------------//
   FuzzyObstacleAvoidance fuzzy(&laser);

  //---------------------------initialization of Local Variables-------------------------------------//

  int taeller_til_pac_location = 0;
  int x_temp;
  int y_temp;

  double newspeed = 0;
  double newdir = 0;
  double difference;

  bool PacManMode = false;
  bool PacManMode_allowed = false;
  bool draw = true;
  bool start_tid = true;
  bool Halvvejs = true;
  bool eating = true;
  bool time1 = true;
  bool time2 = false;
  bool fuzzy_bool = true;

  time_t begin,end;
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<cv::Point2d> PacMan_coords;
  sleep(1);
  sleep(8);



    //-------------------------------------------------------------------------------------------------------//
  while (true) {
    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();
    //-------------------------------------------------------------------------------------------------------//

    //-------------------------------------------------------------------------------------------------------//

    if(start_tid)
    {
        start = std::chrono::high_resolution_clock::now();
        start_tid = false;
    }

    //-------------------------------------------------------------------------------------------------------//

    if ( destinations.size()*2 <= destinations_size_holder && Halvvejs == true )
    {
        newspeed = 0;
        PacManMode_allowed = true;
        Halvvejs = false;
        myprocess.analyse_votespace();
        myBush.afstikker(myprocess.get_Marbles_location());
        PacMan_coords = myBush.getPacMan_coords();
    }

    //-------------------------------------------------------------------------------------------------------//

    if(PacManMode_allowed)
    {
        if (eating)
        {
            for (int i = 0; i < PacMan_coords.size(); i+=2)
            {
                if ((abs(x_pose - PacMan_coords[i].x)<=2) && (abs(y_pose-PacMan_coords[i].y)<=2))
                {
                    PacManMode = true;
                    taeller_til_pac_location = i;
                    eating = false;
                    break;
                }
            }
        }
    }
    
    //-------------------------------------------------------------------------------------------------------//

    if((abs(x_pose - destinations[0][0])<=0.25)&&(abs(y_pose-destinations[0][1])<=0.25))
    {
        destinations.erase(destinations.begin());
        if (destinations.size() == 0)
        {
        draw = false;
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elasped = finish-start;
        myprocess.path_length();
        }
    }
    //----------------------------------------------------------------------------------------------------------//
    if(time1)
    {
        x_temp = x_pose;
        y_temp = y_pose;
        time1 = false;
        time (&begin);
    }
    time (&end);
    difference = difftime(end,begin);

    if(difference > 1 && fuzzy_bool == false)
    {
        std::cout << "difference > 1: " << difference << std::endl;
        fuzzy_bool = true;
        difference = 0;
    }

    if(difference > 5.0)
    {
        std::cout << "x: " << abs(x_pose - x_temp) << " & y: " << abs(y_pose - y_temp) << std::endl;
        if ( abs(x_pose - x_temp) <= 1 && abs(y_pose - y_temp) <= 0.7 )
        {
            time2 = true;
            fuzzy_bool = false;
        }
        time1 = true;
    }

    if(time2)
    {
        std::cout << "pikkeholder " << fuzzy_bool << std::endl;
        newspeed = -0.5;
        newdir = -1;
        time2 = false;
    }
    //-------------------------------------------------------------------------------------------------------//

    if(destinations.size() != 0 && !PacManMode && fuzzy_bool == true)
    {
        fuzzy.buildController(move.getGoal(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, destinations[0]));
        newspeed = fuzzy.getControlOutput().speed;
        newdir = fuzzy.getControlOutput().direction;
    }
    //------------------------------------------------//
    if (destinations.size() != 0 && PacManMode && fuzzy_bool == true)
    {
        if (move.collect_marbles(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, PacMan_coords[taeller_til_pac_location+1]))
        {
            newdir = move.getDir();
            newspeed = move.getSpeed();
        }
        else if (myprocess.center_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, laser.getCenterDistance(), destinations).z == 1)
        {
            PacManMode = false;
            move.update_marble_bool();
            eating = true;
            PacMan_coords.erase(PacMan_coords.begin() + taeller_til_pac_location);
            PacMan_coords.erase(PacMan_coords.begin() + taeller_til_pac_location);
        }
        else
        {
            newspeed = myprocess.center_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, laser.getCenterDistance(), destinations).x;
            newdir = myprocess.center_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, laser.getCenterDistance(), destinations).y;
        }
    }


    //-------------------------------Set Speed and Direction-----------------------------------------------------//
        ignition::math::Pose3d pose(newspeed, 0, 0, 0, 0, newdir);
        myprocess.draw_path(x_pose,y_pose, newspeed, draw);

    //-------------------------------Convert to a pose message---------------------------------------------------//
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }
   //-------------------------------Shutdown of Gazebo Client----------------------------------------------------//
     gazebo::client::shutdown();

}  // while true
}  // while !all_check_pts_done
}  // while counter
