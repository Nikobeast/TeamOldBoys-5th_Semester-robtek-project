#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;

int main()
{


    int number_of_images =172;
    int save_path_int = 3;
    std::vector<cv::Mat> list_of_images;


    while(save_path_int < number_of_images)
    {

    std::string text = "/home/ole/Documents/Projekt/github/Image/drawn_Paths";
    text+= std::to_string(save_path_int);
    text+=".png";
    cv::Mat name = cv::imread(text);
    list_of_images.push_back(name);



    save_path_int += 5;

    }



    cv::Vec3b mandingo = {0,0,0};

    while(list_of_images.size()!=1)
    {
        for(int i = 0; i < list_of_images[0].size().height; i++)
        {
            for(int j = 0 ; j <list_of_images[0].size().width;j++)
            {
                if(list_of_images[1].at<cv::Vec3b>(i,j) != mandingo)
                {
                    list_of_images[0].at<cv::Vec3b>(i,j) = list_of_images[1].at<cv::Vec3b>(i,j);

                }
            }
        }

        list_of_images.erase(list_of_images.begin()+1);

    }


    cv::imwrite("/home/ole/Documents/Projekt/github/Image/combined_paths1.png",list_of_images[0]);
    cv::imshow("combined images", list_of_images[0]);

    cv::waitKey(0);



}
