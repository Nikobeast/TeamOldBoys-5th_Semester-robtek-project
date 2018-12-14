#ifndef QLEARNING_H
#define QLEARNING_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <algorithm>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <limits>

//#define COLUMNS 120// bigworld
#define COLUMNS 20//smallworld
//#define COLUMNS 4//reinforcement world
//#define COLUMNS 80//ny world
//#define ROWS 80// bigworld
#define ROWS 15 //smallworld
//#define ROWS 3 //reinforcement
//#define ROWS 60 // ny world

struct state
{
    int x;
    int y;
    bool is_outside_environment;
};


enum action { UP, DOWN, LEFT, RIGHT };



class Qlearning
{
public:
    Qlearning();
    void Q_learning();
    std::vector<int> return_path();

private:
    int epsilon_greedy(long stateIndex,int room);
    float Q_max(long stateIndex,int room);
    void find_shortest_path();
    void find_shortest_path(std::vector<int> vec,int room, int state);
    void find_a_path();

    //********* Qlearnings variabler::::::

    std::vector<std::vector<float>> Q_table;

    float epsilon = 0.1;
    long counter = 0;
    float gamma = 0.9;
    float alpha = 0.1;
    //int rules[5][5] = {{0,1,1,0,1},{1,0,0,0,0},{1,0,0,1,0},{0,0,1,0,1},{1,0,0,1,0}};
   /* float rules[16][16] = {{0,0,0,0.8,0,0,0,0,0,0,0,0,0,0,0,0},//0
                         {0,0,0,0.7,0,0,0,0,0,0,0,0,0,0,0,0},//1
                         {0,0,0,0,1.8,0,0,0,0,0,0,0,0,0,0,0},//2
                         {0.8,0.7,0,0,2,0,1.2,0,0,0,0,0,0,0,0,0},//3
                         {0,0,1.8,2,0,1,1.2,0,0,0,0,0,1.5,0,0,0},//4
                         {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},//5
                         {0,0,0,1.2,1.2,0,0,0.9,1.1,0,0,0,0,0,0,0},//6
                         {0,0,0,0,0,0,0.9,0,0,0,0,0,0,0,0,0},//7
                         {0,0,0,0,0,0,1.1,0,0,0,0,0,0,0,0,0},//8
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.5},//9
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//10
                         {0,0,0,0,0,0,0,0,0,0,0,0,0.9,0,1.3,0},//11
                         {0,0,0,0,1.5,0,0,0,0,0,0,0.9,0,0,0,1.6},//12
                         {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,2},//13
                         {0,0,0,0,0,0,0,0,0,0,0,1.3,0,0,0,1.6},//14
                         {0,0,0,0,0,0,0,0,0,1.5,0,0,1.5,2,1.6,0}};//15*/

  /*  float rules[12][12] = {
                           {0,1.5,0,0,0,0,0,0,0,0,0,0},//0
                           {1.5,0,0.8,0,0.8,0,0,0,0,0,0,0},//1
                           {0,0.8,0,0.7,0,0,0,0,0,0,0,0},//2
                           {0,0,0.7,0,1.5,0,0,0,0,0,0,0},//3
                           {0,0.8,0,1.5,0,0.7,0,0,0,0,0,0}, //4
                           {0,0,0,0,0.7,0,2.2,0.5,0,0,0,0}, //5
                           {0,0,0,0,0,2.2,0,0,0,0,0,0},//6
                           {0,0,0,0,0,0.5,0,0,1,0,0,0},//7
                           {0,0,0,0,0,0,0,1,0,0.4,0,0},//8
                           {0,0,0,0,0,0,0,0,0.4,0,0.7,0},//9
                           {0,0,0,0,0,0,0,0,0,0.7,0,0.2},//10
                           {0,0,0,0,0,0,0,0,0,0,0.2,0}};//11*/
    float rules[13][13] = {
                               {0,0,0,0,6.5,0,0,0,0,0,0,0,0},//1
                               {0,0,0,0,0,0,0,0,9,0,0,0,0},//2
                               {0,0,0,3.6,0,0,0,0,0,0,0,0,0},//3
                               {0,0,3.6,0,0,4,0,0,0,0,0,0,0},//4
                               {6.5,0,0,0,0,0,0,3.3,0,0,0,0,0}, //5
                               {0,0,0,4,0,0,0,0,0,0,0,3.5,0}, //6
                               {0,0,0,0,0,0,0,4,0,0,0,0,3.6},//7
                               {0,0,0,0,3.3,0,4,0,0,0,3.6,0,0},//8
                               {0,9,0,0,0,0,0,0,0,2.3,0,0,0},//9
                               {0,0,0,0,0,0,0,0,2.3,0,2.9,2.7,0},//10
                               {0,0,0,0,0,0,0,3.6,0,2.9,0,0,7},//11
                               {0,0,0,0,0,3.5,0,0,0,2.7,0,0,0},//12
                               {0,0,0,0,0,0,3.6,0,0,0,7,0,0}};//13
    int nodes; //antal rum.

    int counter2;
    float reward;
    int shifter;
    bool tie;
    int states;
    int initial_state;
    int initial_room;

    std::vector<int> path;




     int visited = 0b0000000000011111;


};

#endif // QLEARNING_H
