/**
 * @file robot.cpp
 * @author Aaqib Barodawala
 * @author Ankur Chavan
 * @author Krishna Hundekari
 * @brief Implementation of the Robot class.
 * @version 0.1
 * @date 2022-11-07
 *
 */

#include "../simulator/simulator.h"
#include "robot.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <set>


// Turning left
void rwa2group12::Robot::turn_left(){
    Simulator::turnLeft();
}

// Turning Right
void rwa2group12::Robot::turn_right(){
    Simulator::turnRight();
}

// Moving forward
void rwa2group12::Robot::move_forward(){
    Simulator::moveForward();
}

// Setting the position
void rwa2group12::Robot::set_position(int x, int y){
    m_position.first=x;
    m_position.second=y;
}

// Setting the direction
void rwa2group12::Robot::set_direction(char d){
    m_direction=d;
}

// Setting the direction of robot at the goal position
void rwa2group12::Robot::set_directionnew(char d){
    m_new_direction=d;
}

// Appending the vector position_logx to record the x coordinates of travel
void rwa2group12::Robot::set_position_logx(int x){{
    m_position_log_x.push_back(x);
 }}

// Appending the vector position_logy to record the y coordinates of travel
void rwa2group12::Robot::set_position_logy(int y){
    m_position_log_y.push_back(y);
 }

// Appending the m_position_backtrace with the pair (x,y)coordinates of the forward travelled path
 void rwa2group12::Robot::set_m_position_backtrace(int x, int y){
    m_position_backtrace.push_back(std::make_pair(x,y));
 }

// Appending the m_final_backtrace with the pair (x,y)coordinates of the optimal return path
void rwa2group12::Robot::set_m_final_backtrace(std::pair<int,int> (x))
{
    m_final_backtrace.push_back(x);
}