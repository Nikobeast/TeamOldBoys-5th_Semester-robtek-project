#include "qlearning.h"

Qlearning::Qlearning(int totalRooms, int totalMarbles, int initialRoom)
{
    total_rooms = totalRooms; // initialiseres med antal rum
    total_marbles = totalMarbles;

     table_offset_room = pow(2,total_rooms);
     states = total_rooms*table_offset_room;

    cdf.push_back(pdf[0]);
    for(int i=1;i<total_rooms;i++)
    {
        cdf.push_back(pdf[i]+cdf[i-1]);
    }
    std::cout << "Dette tal skal vaere praecist 1:  " << cdf[total_rooms-1] << std::endl;

    initial_room = initialRoom;

    for (int i=0;i<total_rooms;i++)
        zero_holder.push_back(0);
    for(int i = 0; i < states; i++)
    {
       Q_table.push_back(zero_holder); // Q_table er nu initialiseret til all zeroes.
    }
    srand(time(NULL));
}

void Qlearning::Q_learning(int episodes, int episode_length, float eps, float gam, float alph, bool showPath, bool limited)
{
    epsilon=eps;
    gamma=gam;
    alpha=alph;
    int movement;

    for(int i = 0; i < states; i++)
    {
       Q_table[i]=zero_holder; // Q_table er nu gen-initialiseret til all zeroes.
    }
    counter = 0;
    while(counter <= episodes)
    {
        cur_room = initial_room;
        searched_rooms = pow(2,cur_room);
        timeToLive=episode_length;
        if(!limited)
        {
            timeToLive=10000;
        }
        movement=0;
        placeMarbles(false);
        collected_marbles = 0;
        if (marblePlacement[cur_room])
        {
            collected_marbles=marblePlacement[cur_room];
            marblePlacement[cur_room]=0;
        }
        MarbInFirstRoom=collected_marbles;

        while (timeToLive > 0)
        {
            int action = epsilon_greedy(searched_rooms, cur_room);//Choose A from S using policy
            long index = cur_room*table_offset_room+searched_rooms;
            reward=0;

            updateSearchedRooms(action);

            if (marblePlacement[action])
            {
                reward = marblePlacement[action]*25;
                collected_marbles+=marblePlacement[action];
                marblePlacement[action]=0;
            }

            reward-=0.1*simpleRules[cur_room][action];

            Q_table[index][action] = Q_table[index][action]+alpha*(reward+gamma*Q_max(searched_rooms,action)-Q_table[index][action]);

            timeToLive-=simpleRules[cur_room][action]; // -1 hvis der blot ønskes at 1 step koster 1 episode
            movement+=simpleRules[cur_room][action];

            if ((collected_marbles >= (total_marbles))||(timeToLive<=0))
            {
                if(!limited)
                {
                    updateTable(episodes,counter,movement);
                }
                timeToLive = -1;
            }
            if(showPath && ((counter == 0)||(counter==episodes)))
                std::cout << cur_room << ", " << std::flush;
            cur_room = action;
        }
        if(limited)
        {
           updateTable(episodes,counter,collected_marbles);
        }
        if(showPath && ((counter == 0)||(counter==episodes)))
        {
            std::cout << cur_room << std::endl;
        }
        counter++;
    }
    return;
}

void Qlearning::Q_learningComplex(int episodes, int episode_length, float eps, float gam, float alph, bool showPath, bool limited)
{
    epsilon=eps;
    gamma=gam;
    alpha=alph;
    int movement;

    for(int i = 0; i < states; i++)
    {
       Q_table[i]=zero_holder; // Q_table er nu gen-initialiseret til all zeroes.
    }
    counter = 0;
    while(counter <= episodes)
    {
        searched_rooms = 0;
        cur_room = initial_room;
        timeToLive=episode_length;
        if(!limited)
        {
            timeToLive=10000;
        }
        movement=0;
        placeMarbles(false);
        collected_marbles = 0;
        if (marblePlacement[cur_room])
        {
            collected_marbles=marblePlacement[cur_room];
            marblePlacement[cur_room]=0;
        }
        MarbInFirstRoom=collected_marbles;

        while (timeToLive > 0)
        {
            int action = epsilon_greedyComplex(searched_rooms, cur_room);//Choose A from S using policy
            long index = cur_room*table_offset_room+searched_rooms;
            reward=0;

            if(cur_room==action)
            {
                updateSearchedRooms(action);

                if (marblePlacement[action])
                {
                    reward = marblePlacement[action]*25;
                    collected_marbles+=marblePlacement[action];
                    marblePlacement[action]=0;
                }
            }

            reward-=0.1*complexRules[cur_room][action];

            Q_table[index][action] = Q_table[index][action]+alpha*(reward+gamma*Q_maxComplex(searched_rooms,action)-Q_table[index][action]);

            timeToLive-=complexRules[cur_room][action]; // -1 hvis der blot ønskes at 1 step koster 1 episode
            movement+=complexRules[cur_room][action];

            if ((collected_marbles >= (total_marbles))||(timeToLive<=0))
            {
                if(!limited)
                {
                    updateTable(episodes,counter,movement);
                }
                timeToLive = -1;
            }
            if(showPath && ((counter == 0)||(counter==episodes)))
                std::cout << cur_room << ", " << std::flush;
            cur_room = action;
        }
        if(limited)
        {
           updateTable(episodes,counter,collected_marbles);
        }
        if(showPath && ((counter == 0)||(counter==episodes)))
        {
            std::cout << cur_room << std::endl;
        }
        counter++;
    }
    return;
}

float Qlearning::Q_max(int s_rooms, int room)
{
    float max = std::numeric_limits<float>::lowest();
    long index = room*table_offset_room+s_rooms;
    for(int i = 0; i < total_rooms;i++)
    {
        if((max < Q_table[index][i]) && (simpleRules[room][i] != 0))
            {
                max = Q_table[index][i];
            }
        }
    return max;
}

float Qlearning::Q_maxComplex(int s_rooms, int room)
{
    float max = std::numeric_limits<float>::lowest();
    long index = room*table_offset_room+s_rooms;
    for(int i = 0; i < total_rooms;i++)
    {
        if((max < Q_table[index][i]) && (complexRules[room][i] != 0))
            {
                max = Q_table[index][i];
            }
        }
    return max;
}

int Qlearning::epsilon_greedy(int s_rooms, int room)
{
    float test = (float)rand()/RAND_MAX;
    float max = std::numeric_limits<float>::lowest();
    std::vector<int> best_location;
    if(test > epsilon)
    {
        long index = room*table_offset_room+s_rooms;
        for(int i = 0; i < total_rooms;i++)
        {
            if((max < Q_table[index][i]) && (simpleRules[room][i] != 0))
            {
                max = Q_table[index][i];
                best_location.clear();
                best_location.push_back(i);
            }
            else if((max == Q_table[index][i]) && (simpleRules[room][i] != 0))
            {
                best_location.push_back(i);
            }
        }
    }
    else
    {
        best_location.clear();
        for (unsigned i=0;i<total_rooms;i++)
        {
            if(simpleRules[room][i]!=0)
                best_location.push_back(i);
        }
    }
    int choice = rand()%best_location.size();
    return best_location[choice];
}

int Qlearning::epsilon_greedyComplex(int s_rooms, int room)
{
    float test = (float)rand()/RAND_MAX;
    float max = std::numeric_limits<float>::lowest();
    std::vector<int> best_location;
    if(test > epsilon)
    {
        long index = room*table_offset_room+s_rooms;
        for(int i = 0; i < total_rooms;i++)
        {
            if((max < Q_table[index][i]) && (complexRules[room][i] != 0))
            {
                max = Q_table[index][i];
                best_location.clear();
                best_location.push_back(i);
            }
            else if((max == Q_table[index][i]) && (complexRules[room][i] != 0))
            {
                best_location.push_back(i);
            }
        }
    }
    else
    {
        best_location.clear();
        for (unsigned i=0;i<total_rooms;i++)
        {
            if(complexRules[room][i]!=0)
                best_location.push_back(i);
        }
    }
    int choice = rand()%best_location.size();
    return best_location[choice];
}


void Qlearning::placeMarbles(bool print)
{
    bool placed;
    marblePlacement.clear();
    for(int i=0;i<total_rooms;i++)
    {
     marblePlacement.push_back(0);
    }
    for(int i=0; i<total_marbles;i++)
    {
        placed=false;
        float prob = (float)rand()/RAND_MAX;
        for (unsigned j=0;j<cdf.size();j++ )
        {
            if(prob < cdf[j] && placed == false)
            {
                marblePlacement[j]++;
                placed=true;
            }
        }
        if(!placed)
        {
            std::cout << "FEJL I PLACEMENT" << std::endl;
            marblePlacement[rand()%total_rooms]++;
        }
    }
    if (print){
        std::cout << "Marbles i rum:" << std::endl;
        for (unsigned i=0;i<marblePlacement.size();i++)
            std::cout << marblePlacement[i] << ",";
        std::cout << std::endl;
    }
}

void Qlearning::updateSearchedRooms(int action)
{
    int shifter = 1;
    shifter = shifter << action;
    searched_rooms = searched_rooms | shifter;
}

void Qlearning::updateTable(int episodes, int index, int marbles)
{
    int spacing = 50; // normalt 50
    if((index%spacing)==0)
    {
    if (valTable.size()<(episodes/spacing))
        valTable.push_back(marbles);
    else
        valTable[index/spacing]+=marbles;
    }
}

void Qlearning::printTable(int trials)
{
         mystream << "A-G-E, " << alpha << ", " << gamma << ", " << epsilon << ", ";
         for(unsigned i=0;i<valTable.size()-1;i++)
            mystream << valTable[i]/float(trials) << ", ";
         mystream << valTable[valTable.size()-1]/float(trials) << std::endl;

         valTable.clear();
}
void Qlearning::createFile(bool begin){
    if (begin)
        mystream.open("dataOut.txt");
    else
        mystream.close();
}
