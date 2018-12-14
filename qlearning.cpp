#include "qlearning.h"


Qlearning::Qlearning()
{

    nodes = 13; // initialiseres med antal rum
    states = nodes*pow(2,nodes);
   // initial_state = 0b0000001000000000;
    initial_room = 0;

    initial_state = pow(2,initial_room);
    visited = 0;


    for(int i = 0; i < nodes; i++)
    {
        visited*=2;
        visited++;

    }

   // std::cout << visited << std::endl ;

}



void Qlearning::Q_learning()
{
    std::vector<int> zero_holder;



   for(int i = 0; i < states; i++)
   {
     //  Q_table.push_back({123,123,123,123,123});

       Q_table.push_back({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
   }

  srand(time(NULL));

   counter2 = 0;
   reward = -1;
   shifter = 1;




    while(counter2 < 500000)
    {
        int state = initial_state; //4 første bits -> rummet du er i, 12 sidste bits -> rum man har besøgt (er i rum 0, har besøgt rum 0)
        int room = initial_room;

       while (counter < 2000)
       {
           int action2 = epsilon_greedy(state, room);//Choose A from S using policy
           long index = room*pow(2,nodes)+state;

           shifter = 1;
           shifter = shifter << action2;
           state = state | shifter;
           if(state == visited)
           {
               reward = 100;
               counter = 1999;
           }
           else
           {

               reward = -1*rules[room][action2];

               if(reward == 0)
                   std::cout << "Warning: error in rulebase!" << std::endl;
           }
            Q_table[index][action2] = Q_table[index][action2]+alpha*(reward+gamma*Q_max(state,action2)-Q_table[index][action2]);
          // std::cout << "I am also alive *******************************************" << counter<< std::endl;

           room = action2;
            counter++;

       }
       counter2++;
       counter = 0;
       //std::cout << "Iteration: " << counter2 << std::endl;
    }
  /*  for(int i = 0; i < Q_table.size();i++)
    {
        std::cout << "("<<i<<")   ";
        bool tester = false;
        for(int j = 0; j < nodes ; j++)
        {

            if(Q_table[i][j] > 95)
                tester = true;
        }
        std::cout << std::endl;
    }*/
    //for(int i = 0; i < nodes; i++)
    //     std::cout << Q_table[initial_room*pow(2,nodes)+initial_state][i] <<", " <<  std::endl;

    //find_shortest_path();
    find_a_path();

    return;

}

void Qlearning::find_a_path()
{

    int state = initial_state;
    int room = initial_room;
    epsilon = 0;
    std::vector<int> vec;
    float value = Q_max(state,room);

    while(value < 99.9)
    {


    vec.push_back(room);
    //std::cout <<"room = " <<room <<std::endl;

    int action = epsilon_greedy(state, room);//Choose A from S using policy
    //long index = room*pow(2,nodes)+state;
    value = Q_max(state,room);
    shifter = 1;
    shifter = shifter << action;
    state = state | shifter;
    room = action;
    }
    vec.push_back(room);

    for(int i = 0; i < vec.size(); i++)
    {
        //std::cout << vec[i] << ", "<< std::endl;
        path.push_back(vec[i]);
    }
    return;
}

std::vector<int> Qlearning::return_path()
{
    return path;
}


void Qlearning::find_shortest_path(std::vector<int> vec, int room, int state)
{
    vec.push_back(room);
    bool local_tie = true;
    while (local_tie)
    {
        int action = epsilon_greedy(state, room);//Choose A from S using policy
        long index = room*pow(2,nodes)+state;
        local_tie = tie;
        shifter = 1;
        shifter = shifter << action;
        int ny_state = state | shifter;
            float hej = Q_max(state,room);
            if(hej > 99.9)
            {
                std::cout <<"\n\n\nKIIIIGGG:::: " << std::endl;
                for(unsigned int i = 0; i < vec.size();i++)
                    std::cout << vec[i] <<", " ;

                std::cout << action << "."<< std::endl;

                return;
            }
        Q_table[index][action] = 0;
        find_shortest_path(vec,action,ny_state);
    }
}

void Qlearning::find_shortest_path()
{
    int state = initial_state; //4 første bits -> rummet du er i, 12 sidste bits -> rum man har besøgt (er i rum 0, har besøgt rum 0)
    int room = initial_room;
    std::vector<int> routes;
    epsilon = 0;
    find_shortest_path(routes,room, state);
}

float Qlearning::Q_max(long state_index, int room)
{
    float max = std::numeric_limits<float>::lowest();
    long index = room*pow(2,nodes)+state_index;
    for(int i = 0; i < nodes;i++)
    {
        if((max < Q_table[index][i]) && (rules[room][i] != 0))
            {
                max = Q_table[index][i];
            }
        }
    return max;
}


int Qlearning::epsilon_greedy(long state_index, int room)
{
    float test = (float)rand()/RAND_MAX;
    float max = std::numeric_limits<float>::lowest();
    int max_location = 0;
    if(test > epsilon)
    {
        long index = room*pow(2,nodes)+state_index;
        for(int i = 0; i < nodes;i++)
        {
            if((max < Q_table[index][i]) && (rules[room][i] != 0))
            {
                tie = false;
                max = Q_table[index][i];
                max_location = i;
            }

            else if((max == Q_table[index][i]) && (rules[room][i] != 0))
            {
                tie = true;
                int randomHolder = rand() % 2;
                if(randomHolder == 1)
                {
                    max  = Q_table[index][i];
                    max_location = i;
                }
            }
        }
    }
    else
    {
        max_location = rand() % nodes;
        while(!rules[room][max_location])
            max_location = rand() % nodes;
    }
    return max_location;
}
