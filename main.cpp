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
  cv::cvtColor(im, im, CV_RGB2BGR);
  myprocess.loadIm(im);

  //myprocess.hough_circles(); // Hough circle detection
  myprocess.process();     // our own detection algorithm

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

bool correct_posistion(cv::Vec2d check_point, float margin)
{
    if((abs(x_pose - check_point[0])<=margin)&&(abs(y_pose-check_point[1])<=margin))
    {
        return true;
    }

    return false;

}

double end_angle;
bool search_room()
{
    double current_angle = move.get_search_angle(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose);

    if(abs(end_angle - current_angle)<25)
        return true;

    return false;


}

    //----------------------------Initialize Path Planning----------------------------------------------//
int main(int _argc, char **_argv) {

    //------------------------------------------------//
    myBush.CreateBrushfire();
    myQ_learning.Q_learning();
    int destinations_size_holder = 0;
    std::vector<cv::Vec2d> Corners = myBush.return_corners();
    std::vector<int> Corner_path = myQ_learning.return_path();
    std::vector<cv::Vec2d> destinations;
    std::vector<cv::Vec2d> reverse_destinations;

    //------------------------------------------------//

    for (int i = 0; i < Corner_path.size(); i++)
        {
            destinations.push_back(Corners[Corner_path[i]]);
            reverse_destinations.insert(reverse_destinations.begin(),Corners[Corner_path[i]]);
        }
   // destinations.insert(destinations.end(),holder_destinations.begin(),holder_destinations.end());
   // destinations_size_holder = destinations.size();
    reverse_destinations.erase(reverse_destinations.begin());

    //-------------------------Generate Image for VoteSpace----------------------------------------------//


    im = cv::imread("../Image/floor_plan.png",cv::IMREAD_GRAYSCALE);
    myprocess.generateVoteSpace(im);


   //-------------------------While-Loop for multiply gazebo startups------------------------------------//

    bool all_check_pts_done = false;

while(!all_check_pts_done){

    //------------------------Automatical Gazebo startup sequence----------------------------------------//

    mutex.lock();
    std::thread first(gazebo_start_seq);
    first.detach();
    mutex.unlock();
    myprocess.generateWorld();
    sleep(2);


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
  int state = 0;
  bool  done=false;
  bool  fuzzy_Bool=true;
  bool  time1=true;
  bool  time2=false;
  bool  first_half = true;
  double difference;

  double newspeed = 0;
  double newdir = 0;
  double angle_holder = 0;


  float margin_first_round = 0.75;
  float margin_pick_up_round = 0.25;


  time_t begin,end;
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<cv::Point2d> PacMan_coords;
    sleep(2); // 10 sekunder pause til optagelse



    //-------------------------------------------------------------------------------------------------------//
  while (true) {
    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();
    //-------------------------------------------------------------------------------------------------------//



    switch (state)
    {
    case 0:        //States 0,1,2 er opmåling. State 3 er afslutning på opmåling.
            std::cout << "state 0" << std::endl;
            fuzzy.buildController(move.getGoal(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, destinations[0]));
            newspeed = fuzzy.getControlOutput().speed;
            newdir = fuzzy.getControlOutput().direction;
            if(correct_posistion(destinations[0],margin_first_round)==true)
            {
                destinations.erase(destinations.begin());
                state = 1;
            }
            if(destinations.size()==0 && first_half)
            {
                state = 3;
            }
            else if(first_half == false)
                state = 4;
        break;
    case 1:
            std::cout << "state 1" << std::endl;
            angle_holder = move.get_angle();
            newspeed = 0;
            newdir = -0.5;
            if(angle_holder >= 30)
                end_angle = angle_holder-30;
            else
                end_angle = angle_holder +330;
            state = 2;
        break;
    case 2:
        std::cout << "state 2" << std::endl;
            if(search_room())
                state = 0;
        break;
    case 3:
        std::cout << "state 3" << std::endl;
        myprocess.analyse_votespace();
        myBush.afstikker(myprocess.get_Marbles_location());
        PacMan_coords = myBush.getPacMan_coords();
        state=4;
        break;
    case 4:
        std::cout << "state 4" << std::endl;
        first_half = false;
        fuzzy.buildController(move.getGoal(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, reverse_destinations[0]));
        newspeed = fuzzy.getControlOutput().speed;
        newdir = fuzzy.getControlOutput().direction;

        if(correct_posistion(reverse_destinations[0],margin_pick_up_round)==true)
        {
            reverse_destinations.erase(reverse_destinations.begin());
        }
        if(reverse_destinations.size()==0)
        {
            state = 100;
        }
        for(int number=0; number < PacMan_coords.size();number+=2)
        {
            std::cout << "LEDER EFTER UDSIKKER!!!!!****!*!*!*!*!*!" << std::endl;
            if((abs(x_pose-PacMan_coords[number].x)<=2)&&(abs(y_pose-PacMan_coords[number].y)<=2))
            {
                //std::cout << "x " << abs(x_pose-PacMan_coords[number].x) << " y " << abs(y_pose-PacMan_coords[number].y) << std::endl;
                state =5;
                newspeed=0;
                newdir=0;
                reverse_destinations.insert(reverse_destinations.begin(),PacMan_coords[number]);
                reverse_destinations.insert(reverse_destinations.begin(),PacMan_coords[number+1]);
                PacMan_coords.erase(PacMan_coords.begin()+number,PacMan_coords.begin()+number+1);
                break;
            }
        }
        break;
    case 5:
        std::cout << "state 5" << std::endl;
        if (move.collect_marbles(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose, reverse_destinations[0]))
        {
            newdir = move.getDir();
            newspeed = move.getSpeed();
        }
        else
        {
            reverse_destinations.erase(reverse_destinations.begin());
            move.update_marble_bool();
            state =6;
        }
        break;
    case 6:
        std::cout << "state 6" << std::endl;
            if (myprocess.Get_Biggest_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose) == 2)
            {
                reverse_destinations.erase(reverse_destinations.begin());
                state = 4;
            }
            if(myprocess.Get_Biggest_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose) == 1)
                newdir = 0.3;
            else if(myprocess.Get_Biggest_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose) == -1)
                newdir = -0.3;
            else
            {
                newdir = 0;
                state = 7;
            }
        break;
    case 7:
        std::cout << "state 7" << std::endl;
            if (myprocess.Get_Biggest_marble(x_pose,y_pose,x_qpose,y_qpose,z_pose,w_pose) != 2)
                newspeed = 0.5;
            else
             state = 4;
        break;
    case 99:
        std::cout << "state 99" << std::endl;
        break;
    case 100:
        std::cout << "state 100" << std::endl;
        time1=false;
        time2=false;
        fuzzy_Bool=true;
        newdir=0;
        newspeed=0;
        done=true;
        break;
    default:
        break;
    }
    //-------------------------------------------------------------------------------------------------------//

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

    if(difference > 1 && !fuzzy_Bool)
    {
        state=0;
        //std::cout << "difference > 1: " << difference << std::endl;
        difference = 0;
        fuzzy_Bool=true;
    }

    if(difference > 4.0)
    {
        //std::cout << "x: " << abs(x_pose - x_temp) << " & y: " << abs(y_pose - y_temp) << std::endl;
        if ( abs(x_pose - x_temp) <= 1 && abs(y_pose - y_temp) <= 0.7 )
        {
            state=99;
            time2 = true;
            fuzzy_Bool=false;
        }
        time1 = true;
    }

    if(time2)
    {
        newspeed = -0.5;
        newdir = (double(rand())/double(RAND_MAX))-0.5;
        time2 = false;
    }
    //-------------------------------------------------------------------------------------------------------//



    //-------------------------------Set Speed and Direction-----------------------------------------------------//
        ignition::math::Pose3d pose(newspeed, 0, 0, 0, 0, newdir);
        if(done)
        {
            return 0;
        }


    //-------------------------------Convert to a pose message---------------------------------------------------//
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }
   //-------------------------------Shutdown of Gazebo Client----------------------------------------------------//
     gazebo::client::shutdown();

}

}
