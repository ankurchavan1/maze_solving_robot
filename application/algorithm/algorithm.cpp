/**
 * @file algorithm.cpp
 * @author Aaqib Barodawala
 * @author Ankur Chavan
 * @author Krishna Hundekari
 * @brief Implementation of the algorithm to reach the goal in the maze.
 * @version 0.1
 * @date 2022-11-07
 *
 */

#include "../simulator/simulator.h"
#include "../robot/robot.h"
#include "algorithm.h"

#include <vector>
#include <array>
#include <time.h>
#include <cstdlib>
#include <utility>
#include <algorithm>
#include <set>
#include <map>
#include<iostream>
#include <string>
#include <vector>


/**
 * @brief Method to set the outer walls of the maze.
 * 
 */
void rwa2group12::Algorithm::init_outer_walls()
{
    set_maze_height();
    set_maze_width();

    for (int i = 0; i < m_maze_width; i++)
    {
        Simulator::setWall(i, 0, 's');
        Simulator::setWall(i, m_maze_height - 1, 'n');
    }

    for (int i = 0; i < m_maze_height; i++)
    {
        Simulator::setWall(0, i, 'w');
        Simulator::setWall(m_maze_width - 1, i, 'e');
    }
}

/**
 * @brief Sets the outer walls of the maze, generates a random goal and initiates the right-hand approach algorithm to solve the maze.
 * 
 */
void rwa2group12::Algorithm::run()
{
    init_outer_walls();
    generate_goal();
    // follow_wall_right();    // Remove the commment tag to run the follow_wall_right algorith and comment out the follow_wall_left line.
    follow_wall_left();        // Remove the commment tag to run the follow_wall_left algorith and comment out the follow_wall_right line.
    trace_back_fastest();
}




// To set the direction of the robot at the final goal position
void rwa2group12::Algorithm::set_m_final_dir_algorithm(char d){
    m_final_dir_algorithm= d;
}

// To create a copy of the vector of pair of coordinates from the robot class for optimal traceback
void rwa2group12::Algorithm::set_m_final_backtrace_algorithm(std::vector<std::pair<int, int>> v){
    m_final_backtrace_algorithm = v;
}

/**
 * @brief Implements the set_right_wall method to set the right wall of the cell.
 * 
 */
void rwa2group12::Algorithm::set_right_wall(int x, int y, char d)
{
    if (Simulator::wallRight())
    {
        if (d == 'n')
        {
            Simulator::setWall(x, y, 'e');
        }
        else if (d == 'e')
        {
            Simulator::setWall(x, y, 's');
        }
        else if (d == 's')
        {
            Simulator::setWall(x, y, 'w');
        }
        else
        {
            Simulator::setWall(x, y, 'n');
        }
    }
}

/**
 * @brief Implements the set_left_wall method to set the left wall of the cell.
 * 
 */
void rwa2group12::Algorithm::set_left_wall(int x, int y, char d)
{
    if (Simulator::wallLeft())
    {
        if (d == 'n')
        {
            Simulator::setWall(x, y, 'w');
        }
        else if (d == 'e')
        {
            Simulator::setWall(x, y, 'n');
        }
        else if (d == 's')
        {
            Simulator::setWall(x, y, 'e');
        }
        else
        {
            Simulator::setWall(x, y, 's');
        }
    }
}

/**
 * @brief Implements the set_front_wall method to set the front wall of the cell.
 * 
 */
void rwa2group12::Algorithm::set_front_wall(int x, int y, char d)
{
    if (Simulator::wallFront())
    {
        if (d == 'n')
        {
            Simulator::setWall(x, y, 'n');
        }
        else if (d == 'e')
        {
            Simulator::setWall(x, y, 'e');
        }
        else if (d == 's')
        {
            Simulator::setWall(x, y, 's');
        }
        else
        {
            Simulator::setWall(x, y, 'w');
        }
    }
}

/**
 * @brief Right-hand approach algorithm to reach the goal in the maze.
 * 
 */
void rwa2group12::Algorithm::follow_wall_right()
{
    auto robot = std::make_unique<rwa2group12::Robot>();
    int x=0;
    int y=0;
    robot->set_position_logx(x);
    robot->set_position_logy(y);


    std::cerr<<"TURN RIGHT ALGORITHM: "<<std::endl;
    std::cerr<<"Goal: "<<goal.first+1<<" "<<goal.second+1<<std::endl;
    while (x!=goal.first || y!=goal.second)
    {   
        set_right_wall(x, y, robot->get_direction());
        set_left_wall(x, y, robot->get_direction());
        set_front_wall(x, y, robot->get_direction());
        Simulator::setColor(x, y, 'c');
        if (!Simulator::wallRight())
        {
            std::cerr << "Turning Right !!!" << std::endl;
            robot->turn_right();
            robot->move_forward();
            if (robot->get_direction()=='n'){
                robot->set_direction('e');
                x++;
            }else if (robot->get_direction()=='e'){
                robot->set_direction('s');
                y--;
            }else if (robot->get_direction()=='s'){
                robot->set_direction('w');
                x--;
            }else if (robot->get_direction()=='w'){
                robot->set_direction('n');
                y++;
            }
            robot->set_position(x, y);

        }
        else if (!Simulator::wallFront())
        {
            std::cerr << "Moving Front !!!" << std::endl;
            if (robot->get_direction()=='n'){
                y++;
            }else if (robot->get_direction()=='e'){
                x++;
            }else if (robot->get_direction()=='s'){
                y--;
            }else if (robot->get_direction()=='w'){
                x--;
            }
            robot->move_forward();
            robot->set_position(x, y);
        }
        else if (!Simulator::wallLeft())
        {
            std::cerr << "Turning Left !!!" << std::endl;
            robot->turn_left();
            robot->move_forward();
            if (robot->get_direction()=='n'){
                robot->set_direction('w');
                x--;
            }else if (robot->get_direction()=='e'){
                robot->set_direction('n');
                y++;
            }else if (robot->get_direction()=='s'){
                robot->set_direction('e');
                x++;
            }else if (robot->get_direction()=='w'){
                robot->set_direction('s');
                y--;
            }
            robot->set_position(x, y);
        }
        else
        {
            std::cerr << "Turning around !!!" << std::endl;
            robot->turn_right();
            robot->turn_right();
            if (robot->get_direction()=='n'){
                robot->set_direction('s');
            }else if (robot->get_direction()=='e'){
                robot->set_direction('w');
            }else if (robot->get_direction()=='s'){
                robot->set_direction('n');
            }else if (robot->get_direction()=='w'){
                robot->set_direction('e');
            }
            robot->set_position(x, y);
        }



    // Setting the vectors of the coordinates of the path travelled in forward direction
    robot->set_position_logx(x);
    robot->set_position_logy(y);
    robot->set_m_position_backtrace(x,y);
    
    }

// Using the maps for removing the reduntant travel positions and get the optimal path of return travel
std::map<std::pair<int,int>, std::vector<int>> countMap;

for (int j=0; j < robot->get_m_position_backtrace().size() ;j++)
{
    auto result = countMap.insert(std::pair<std::pair<int,int>, std::vector<int>>(robot->get_m_position_backtrace().at(j), {}));
            result.first->second.push_back(j);
}

robot->set_m_final_backtrace(std::make_pair(0,0));

for (int j=0; j < robot->get_m_position_backtrace().size() ;j++)
{
    if(countMap.at(robot->get_m_position_backtrace().at(j)).size() >1)
        {   

            j=countMap.at(robot->get_m_position_backtrace().at(j)).back();

        }
    robot->set_m_final_backtrace(robot->get_m_position_backtrace().at(j));

}



// Copying the required data like the direction of the robot at the goal and the optimum return travel coordinate 
// from the robot class

set_m_final_backtrace_algorithm(robot->get_m_final_backtrace());

set_m_final_dir_algorithm(robot->get_direction());


}

/**
 * @brief Left-hand approach algorithm to reach the goal in the maze.
 * 
 */
void rwa2group12::Algorithm::follow_wall_left()
{
    auto robot = std::make_unique<rwa2group12::Robot>();
    int x=0;
    int y=0;
    robot->set_position_logx(x);
    robot->set_position_logy(y);

    std::cerr<<"TURN LEFT ALGORITHM: "<<std::endl;
    std::cerr<<"Goal: "<<goal.first+1<<" "<<goal.second+1<<std::endl;
    while (x!=goal.first || y!=goal.second)
    {
        set_right_wall(x, y, robot->get_direction());
        set_left_wall(x, y, robot->get_direction());
        set_front_wall(x, y, robot->get_direction());
        Simulator::setColor(x, y, 'c');
        if (!Simulator::wallLeft())
        {
            std::cerr << "Turning Left !!!" << std::endl;
            robot->turn_left();
            robot->move_forward();
            if (robot->get_direction()=='n'){
                robot->set_direction('w');
                x--;
            }else if (robot->get_direction()=='e'){
                robot->set_direction('n');
                y++;
            }else if (robot->get_direction()=='s'){
                robot->set_direction('e');
                x++;
            }else if (robot->get_direction()=='w'){
                robot->set_direction('s');
                y--;
            }
            robot->set_position(x, y);
        }
        else if (!Simulator::wallFront())
        {
            std::cerr << "Moving Front !!!" << std::endl;
            if (robot->get_direction()=='n'){
                y++;
            }else if (robot->get_direction()=='e'){
                x++;
            }else if (robot->get_direction()=='s'){
                y--;
            }else if (robot->get_direction()=='w'){
                x--;
            }
            robot->move_forward();
            robot->set_position(x, y);
        }
        else if (!Simulator::wallRight())
        {
            std::cerr << "Turning Right !!!" << std::endl;
            robot->turn_right();
            robot->move_forward();
            if (robot->get_direction()=='n'){
                robot->set_direction('e');
                x++;
            }else if (robot->get_direction()=='e'){
                robot->set_direction('s');
                y--;
            }else if (robot->get_direction()=='s'){
                robot->set_direction('w');
                x--;
            }else if (robot->get_direction()=='w'){
                robot->set_direction('n');
                y++;
            }
            robot->set_position(x, y);
        }
        else
        {
            std::cerr << "Turning around !!!" << std::endl;
            robot->turn_right();
            robot->turn_right();
            if (robot->get_direction()=='n'){
                robot->set_direction('s');
            }else if (robot->get_direction()=='e'){
                robot->set_direction('w');
            }else if (robot->get_direction()=='s'){
                robot->set_direction('n');
            }else if (robot->get_direction()=='w'){
                robot->set_direction('e');
            }
            robot->set_position(x, y);
        }
    
    // Setting the vectors of the coordinates of the path travelled in forward direction
    robot->set_position_logx(x);
    robot->set_position_logy(y);
    robot->set_m_position_backtrace(x,y);
    }


// Using the maps for removing the reduntant travel positions and get the optimal path of return travel
std::map<std::pair<int,int>, std::vector<int>> countMap;

for (int j=0; j < robot->get_m_position_backtrace().size() ;j++)
{
    auto result = countMap.insert(std::pair<std::pair<int,int>, std::vector<int>>(robot->get_m_position_backtrace().at(j), {}));
            result.first->second.push_back(j);
}

robot->set_m_final_backtrace(std::make_pair(0,0));

for (int j=0; j < robot->get_m_position_backtrace().size() ;j++)
{
    if(countMap.at(robot->get_m_position_backtrace().at(j)).size() >1)
        {   
            j=countMap.at(robot->get_m_position_backtrace().at(j)).back();

        }
    robot->set_m_final_backtrace(robot->get_m_position_backtrace().at(j));

}


// Copying the required data like the direction of the robot at the goal and the optimum return travel coordinate 
// from the robot class

set_m_final_backtrace_algorithm(robot->get_m_final_backtrace());

set_m_final_dir_algorithm(robot->get_direction());

}
/**
 * @brief Method to generate a random goal adjacent to the outer wall of the maze.
 * 
 */
void rwa2group12::Algorithm::generate_goal()
{
    srand((unsigned)time(NULL));
    int y = std::rand() % 16;
    if (y == 0 || y == 15)
    {
        int x = std::rand() % 16;
        goal.first=x;
        
        // Setting the color of the Goal cell to Red
        Simulator::setColor(x, y, 'R');
        // Printing "G" in the Goal cell
        Simulator::setText(x, y, "G");
    }
    else
    {
        std::array<int, 2> g{0, 15};
        int x = g.at(std::rand() % 2);
        goal.first=x;

        // Setting the color of the Goal cell to Red
        Simulator::setColor(x, y, 'R');
        // Printing "G" in the Goal cell
        Simulator::setText(x, y, "G");
    }
    goal.second=y;
}


/**
 * @brief Method to traverse the shortest path for the return travel
 * 
 */
void rwa2group12::Algorithm::trace_back_fastest()
{

// setting up a robot object for the travel

auto robot = std::make_unique<rwa2group12::Robot>();

robot->set_direction(m_final_dir_algorithm);

std::cerr << robot->get_direction() << std::endl;

// Turning back at the goal position
robot->turn_right();
robot->turn_right();

// Setting the new direction  of the robot at the goal postion 
if (robot->get_direction()=='n'){
        robot->set_directionnew('s');}
    else if (robot->get_direction()=='s'){
        robot->set_directionnew('n');}
    else if (robot->get_direction()=='e'){
        robot->set_directionnew('w');}
    else if (robot->get_direction()=='w'){
        robot->set_directionnew('e');}

// Traversing the return path based on the two consecutive coordinates of the optimum path

for(int j= m_final_backtrace_algorithm.size()-1;  j >= 1; j--)
{
    int x1= m_final_backtrace_algorithm.at(j).first;
    int x2= m_final_backtrace_algorithm.at(j-1).first;
    int y1= m_final_backtrace_algorithm.at(j).second;
    int y2= m_final_backtrace_algorithm.at(j-1).second;

    int dx = x2-x1;
    int dy = y2-y1;
    
    // Controlling the travel based on the changes in the two consecutive coordinates and current directoin of the robot
    
    if(dx == 0)
    {
        if(robot->get_directionnew() == 'n' && dy > 0){
            robot->move_forward();
            robot->set_directionnew('n');}
        else if(robot->get_directionnew() == 'n' && dy < 0){
            robot->turn_right();
            robot->turn_right();
            robot->set_directionnew('s');}
        else if(robot->get_directionnew() == 's' && dy > 0){
            robot->turn_right();
            robot->turn_right();
            robot->set_directionnew('n');}
        else if(robot->get_directionnew() == 's' && dy < 0){
            robot->move_forward();
            robot->set_directionnew('s');}
        else if(robot->get_directionnew() == 'e' && dy > 0){
            robot->turn_left();
            robot->move_forward();
            robot->set_directionnew('n');}   
        else if(robot->get_directionnew() == 'e' && dy < 0){
            robot->turn_right();
            robot->move_forward();
            robot->set_directionnew('s');} 
        else if(robot->get_directionnew() == 'w' && dy > 0){
            robot->turn_right();
            robot->move_forward();
            robot->set_directionnew('n');}   
        else if(robot->get_directionnew() == 'w' && dy < 0){
            robot->turn_left();
            robot->move_forward();
            robot->set_directionnew('s');}  
    }
    else if(dy ==0){

        if(robot->get_directionnew() == 'n' && dx > 0){
            robot->turn_right();
            robot->move_forward();
            robot->set_directionnew('e');} 
              
        else if(robot->get_directionnew() == 'n' && dx < 0){
            robot->turn_left();
            robot->move_forward();
            robot->set_directionnew('w');} 

        else if(robot->get_directionnew() == 's' && dx > 0){
            
            robot->turn_left();
            robot->move_forward();
            robot->set_directionnew('e');}  

        else if(robot->get_directionnew() == 's' && dx < 0){
            robot->turn_right();
            robot->move_forward();
            robot->set_directionnew('w');}  

        else if(robot->get_directionnew() == 'e' && dx > 0){
            robot->move_forward();
            robot->set_directionnew('e');}

        else if(robot->get_directionnew() == 'e' && dx < 0){
            robot->turn_right();
            robot->turn_right();
            robot->set_directionnew('w');}
        
        else if(robot->get_directionnew() == 'w' && dx > 0){
            robot->turn_right();
            robot->turn_right();
            robot->set_directionnew('e');}

        else if(robot->get_directionnew() == 'w' && dx < 0){
            robot->move_forward();
            robot->set_directionnew('w');}
       
    }

}

}
