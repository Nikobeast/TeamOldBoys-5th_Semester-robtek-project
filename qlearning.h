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
#include <queue>

class Qlearning
{
public:
    Qlearning(int totalRooms, int totalMarbles, int initialRoom);
    void Q_learning(int episodes, int episodes_length, float epsilon, float gamma, float alpha, bool showPath, bool limited);
    void Q_learningComplex(int episodes, int episodes_length, float epsilon, float gamma, float alpha, bool showPath, bool limited);
    std::vector<int> return_path();
    void createFile(bool begin);
    void printTable(int trials);

private:
    int epsilon_greedy(int s_rooms, int room);
    int epsilon_greedyComplex(int s_rooms, int room);
    float Q_max(int s_rooms,int room);
    float Q_maxComplex(int s_rooms,int room);
    void placeMarbles(bool print);
    void updateSearchedRooms(int action);
    void updateTable(int episodes, int index, int marbles);

    //********* Qlearnings variabler::::::

    std::vector<std::vector<float>> Q_table;
    std::vector<float> zero_holder;
    std::vector<int> marblePlacement;
    float pdf[16]= {0.055,0.110,0.005,0.095,0.070,0.100,0.070,0.0,0.065,0.045,0.055,0.035,0.090,0.035,0.075,0.095};  // antallet på dette array skal være lig med antal rum. Endvidere skal summen af tallene være 1 hverken mere eller mindre.
    std::vector<float> cdf;

    float epsilon; //
    float gamma;
    float alpha;

                           //     0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15
int complexRules[16][16] = {{    10,   0,   0,   0,   0,   0,   0,  54,   0,   0,   0,   0,   0,   0,   0,   0},//0
                            {     0,  10,  17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
                            {     0,  17,  10,  20,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
                            {     0,   0,  20,  10,  34,   0,   0,  43,   0,   0,   0,   0,   0,   0,   0,   0},//3
                            {     0,   0,   0,  34,  10,  31,  34,  33,   0,   0,   0,   0,   0,   0,   0,   0},//4
                            {     0,   0,   0,   0,  31,  10,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
                            {     0,   0,   0,   0,  34,   0,  10,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
                            {    54,   0,   0,  43,  33,   0,   0,  10,  35,  53,   0,   0,   0,   0,   0,   0},//7
                            {     0,   0,   0,   0,   0,   0,   0,  35,  10,   0,   0,   0,   0,   0,   0,   0},//8
                            {     0,   0,   0,   0,   0,   0,   0,  53,   0,  10,  23,  34,   0,   0,   0,   0},//9
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,  23,  10,  36,   0,   0,   0,   0},//10
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,  34,  36,  10,  43,  25,   0,   0},//11
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  43,  10,   0,   0,   0},//12
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  25,   0,  10,  30,   0},//13
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  30,  10,  25},//14
                            {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  25,  10}};//15

                          //     0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15
int simpleRules[16][16] = {{     0,   0,   0,   0,   0,   0,   0,  54,   0,   0,   0,   0,   0,   0,   0,   0},//0
                           {     0,   0,  17,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
                           {     0,  17,   0,  20,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
                           {     0,   0,  20,   0,  34,   0,   0,  43,   0,   0,   0,   0,   0,   0,   0,   0},//3
                           {     0,   0,   0,  34,   0,  31,  34,  33,   0,   0,   0,   0,   0,   0,   0,   0},//4
                           {     0,   0,   0,   0,  31,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
                           {     0,   0,   0,   0,  34,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
                           {    54,   0,   0,  43,  33,   0,   0,   0,  35,  53,   0,   0,   0,   0,   0,   0},//7
                           {     0,   0,   0,   0,   0,   0,   0,  35,   0,   0,   0,   0,   0,   0,   0,   0},//8
                           {     0,   0,   0,   0,   0,   0,   0,  53,   0,   0,  23,  34,   0,   0,   0,   0},//9
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,  23,   0,  36,   0,   0,   0,   0},//10
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,  34,  36,   0,  43,  25,   0,   0},//11
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  43,   0,   0,   0,   0},//12
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  25,   0,   0,  30,   0},//13
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  30,   0,  25},//14
                           {     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  25,   0}};//15

    int total_rooms; //antal rum.
    int total_marbles;

    int MarbInFirstRoom;

    int cur_room;
    int collected_marbles;
    int searched_rooms;

    int table_offset_room;

    int counter;
    int timeToLive = 0;
    float reward;
    int states;
    int initial_room;

    float avgValue;
    std::vector<float> valTable;
    std::ofstream mystream;
};

#endif // QLEARNING_H
