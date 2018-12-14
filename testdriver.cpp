#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include "qlearning.h"


int main(){

   Qlearning Qlearn(16,20,7);

   //::::::::::::::::::::::::::::::::::::::::::::: SETUP PARAMETERS ::::::::::::::::::::::::::::::::::::::::::::::::::://

   int trials =1000;      // 1000
   int episodes = 5000;   // 5000
   int fuel = 500;        //  500
   bool limited= true;    // Set this to true to run the test with limited amount of energy.

   bool showPaths=false;  // Set this to true to print the chosen path of the first and last iteration of the first trial of each algorithm.
   bool show=false;       // Do not change this.
   if(showPaths)
   {
       show=true;
   }

   //:::::::::::::::::::::::::::::::::::::::::::::::::: TO HERE ::::::::::::::::::::::::::::::::::::::::::::::::::::::://


   //::::::::::::::::::::::::::: RUN THIS PART TO TEST THE 5 CHOSEN PARAMETERS, SIMPLE :::::::::::::::::::::::::::::::://
/*
   Qlearn.createFile(true);
   for(int i=1;i<=trials;i++)
   {
      Qlearn.Q_learning(episodes,fuel,0.1,0.9,0.1,show,limited);
      if (i%100==0)
        std::cout << i << "," << std::flush;
      show=false;
   }
   if(showPaths)
   {
       show=true;
   }
   std::cout << std::endl;
   Qlearn.printTable(trials);

   for(int i=1;i<=trials;i++)
   {
      Qlearn.Q_learning(episodes,fuel,0.0,0.7,0.2,show,limited);
      if (i%100==0)
        std::cout << i << "," << std::flush;
      show=false;
   }
   if(showPaths)
   {
       show=true;
   }
   std::cout << std::endl;
   Qlearn.printTable(trials);

   for(int i=1;i<=trials;i++)
   {
      Qlearn.Q_learning(episodes,fuel,0.05,0.7,0.2,show,limited);
      if (i%100==0)
        std::cout << i << "," << std::flush;
      show=false;
   }
   if(showPaths)
   {
       show=true;
   }
   std::cout << std::endl;
   Qlearn.printTable(trials);

   for(int i=1;i<=trials;i++)
   {
      Qlearn.Q_learning(episodes,fuel,0.05,0.99,0.2,show,limited);
      if (i%100==0)
        std::cout << i << "," << std::flush;
      show=false;
   }
   if(showPaths)
   {
       show=true;
   }
   std::cout << std::endl;
   Qlearn.printTable(trials);

   for(int i=1;i<=trials;i++)
   {
      Qlearn.Q_learning(episodes,fuel,0.05,0.6,0.25,show,limited);
      if (i%100==0)
        std::cout << i << "," << std::flush;
      show=false;
   }

   std::cout << std::endl;
   Qlearn.printTable(trials);


   Qlearn.createFile(false);
   std::cout << "faerdig" << std::endl;
   return 0;
*/
//:::::::::::::::::::::::::::::::::::::::::::::::::: TO HERE ::::::::::::::::::::::::::::::::::::::::::::::::::::::://


//::::::::::::::::::::::: RUN THIS PART TO TEST ALL 125 COMBINATIONS OF PARAMETERS, SIMPLE ::::::::::::::::::::::::://
 /*  float eps[5]= {0.00,0.01,0.05,0.10,0.15};
   int epslength = sizeof(eps)/sizeof(float);

   float gammas[5] = {0.6,0.7,0.8,0.9,0.99};
   int gammaslength = sizeof(gammas)/sizeof(float);

   float alphas[5] = {0.05,0.1,0.15,0.20,0.25};
   int alphaslength = sizeof(alphas)/sizeof(float);


   int testnr=1;
   Qlearn.createFile(true);
   for(int a=0;a<alphaslength;a++)
   {
       for(int g=0;g<gammaslength;g++)
       {
          for(int e=0;e<epslength;e++)
          {
              std::cout << "testnr " << testnr++ << "/"<< alphaslength*gammaslength*epslength << " :" << std::flush;
              for(int i=1;i<=trials;i++)
              {
                 Qlearn.Q_learning(episodes,fuel,eps[e],gammas[g],alphas[a],show,limited);
                 if (i%100==0)
                   std::cout << i << "," << std::flush;
                 show=false;
              }
              if(showPaths)
              {
                  show=true;
              }
              std::cout << std::endl;
              Qlearn.printTable(trials);
              std::cout << std::endl;
          }
       }
   }

   Qlearn.createFile(false);
    return 0;
*/
//:::::::::::::::::::::::::::::::::::::::::::::::::: TO HERE ::::::::::::::::::::::::::::::::::::::::::::::::::::::://

    //::::::::::::::::::::::::::: RUN THIS PART TO TEST THE 5 CHOSEN PARAMETERS, SIMPLE :::::::::::::::::::::::::::::::://

    Qlearn.createFile(true);
    for(int i=1;i<=trials;i++)
    {
       Qlearn.Q_learningComplex(episodes,fuel,0.1,0.9,0.1,show,limited);
       if (i%100==0)
         std::cout << i << "," << std::flush;
       show=false;
    }
    if(showPaths)
    {
        show=true;
    }
    std::cout << std::endl;
    Qlearn.printTable(trials);

    for(int i=1;i<=trials;i++)
    {
       Qlearn.Q_learningComplex(episodes,fuel,0.0,0.7,0.2,show,limited);
       if (i%100==0)
         std::cout << i << "," << std::flush;
       show=false;
    }
    if(showPaths)
    {
        show=true;
    }
    std::cout << std::endl;
    Qlearn.printTable(trials);

    for(int i=1;i<=trials;i++)
    {
       Qlearn.Q_learningComplex(episodes,fuel,0.05,0.7,0.2,show,limited);
       if (i%100==0)
         std::cout << i << "," << std::flush;
       show=false;
    }
    if(showPaths)
    {
        show=true;
    }
    std::cout << std::endl;
    Qlearn.printTable(trials);

    for(int i=1;i<=trials;i++)
    {
       Qlearn.Q_learningComplex(episodes,fuel,0.05,0.99,0.2,show,limited);
       if (i%100==0)
         std::cout << i << "," << std::flush;
       show=false;
    }
    if(showPaths)
    {
        show=true;
    }
    std::cout << std::endl;
    Qlearn.printTable(trials);

    for(int i=1;i<=trials;i++)
    {
       Qlearn.Q_learningComplex(episodes,fuel,0.05,0.6,0.25,show,limited);
       if (i%100==0)
         std::cout << i << "," << std::flush;
       show=false;
    }

    std::cout << std::endl;
    Qlearn.printTable(trials);


    Qlearn.createFile(false);
    std::cout << "faerdig" << std::endl;
    return 0;

 //:::::::::::::::::::::::::::::::::::::::::::::::::: TO HERE ::::::::::::::::::::::::::::::::::::::::::::::::::::::://


 //::::::::::::::::::::::: RUN THIS PART TO TEST ALL 125 COMBINATIONS OF PARAMETERS, SIMPLE ::::::::::::::::::::::::://
    float eps[5]= {0.00,0.01,0.05,0.10,0.15};
    int epslength = sizeof(eps)/sizeof(float);

    float gammas[5] = {0.6,0.7,0.8,0.9,0.99};
    int gammaslength = sizeof(gammas)/sizeof(float);

    float alphas[5] = {0.05,0.1,0.15,0.20,0.25};
    int alphaslength = sizeof(alphas)/sizeof(float);


    int testnr=1;
    Qlearn.createFile(true);
    for(int a=0;a<alphaslength;a++)
    {
        for(int g=0;g<gammaslength;g++)
        {
           for(int e=0;e<epslength;e++)
           {
               std::cout << "testnr " << testnr++ << "/"<< alphaslength*gammaslength*epslength << " :" << std::flush;
               for(int i=1;i<=trials;i++)
               {
                  Qlearn.Q_learningComplex(episodes,fuel,eps[e],gammas[g],alphas[a],show,limited);
                  if (i%100==0)
                    std::cout << i << "," << std::flush;
                  show=false;
               }
               if(showPaths)
               {
                   show=true;
               }
               std::cout << std::endl;
               Qlearn.printTable(trials);
               std::cout << std::endl;
           }
        }
    }

    Qlearn.createFile(false);
     return 0;

 //:::::::::::::::::::::::::::::::::::::::::::::::::: TO HERE ::::::::::::::::::::::::::::::::::::::::::::::::::::::://

}

