/**
 * @file robot.h
 * @author Aaqib Barodawala
 * @author Ankur Chavan
 * @author Krishna Hundekari
 * @brief Defining the robot class.
 * @version 0.1
 * @date 2022-11-07
 *
 */

#ifndef __ROBOT_H__
#define __ROBOT_H__

#pragma once

#include<iostream>
#include<array>
#include<vector>

namespace rwa2group12 {
/**
* @brief This class represents the robot.
*/
class Robot {
public:
/**
* @brief Turn the robot left.
*
*/
void turn_left();
/**
* @brief Turn the robot right.
*
*/
void turn_right();
/**
* @brief Move the Robor forward by one cell.
*
*/
void move_forward();
/**
* @brief Gets the position of the robot.
*
* @return Position of the robot.
*/
std::pair<int, int> get_position(){return m_position;}
/**
* @brief Gets the direction of the robot.
*
* @return Direction of the robot.
*/
char get_direction(){return m_direction;}
/**
* @brief Sets the position of the robot.
* 
* @param x X coordinate of the cell.
* @param y Y coordinate of the cell.
*/
void set_position(int x, int y);
/**
* @brief Sets the direction of the robot.
* 
* @param d Direction of the robot. Can be 'n' (north), 's' (south), 'e' (east), 'w' (west)
*/
void set_direction(char d);
/**
 * @brief Get the position logx object for the reference
 * 
 * @return std::vector<int> return type of m_position_log_x
 */
std::vector<int> get_position_logx () { return m_position_log_x;}
/**
 * @brief Set the position logx object to append the x coordinate into the vector m_position_log_x
 * 
 * @param x int paranter which will be pushed back in the vector
 */
void set_position_logx(int x);
/**
 * @brief Get the position logy object for the reference
 * 
 * @return std::vector<int> return type of m_position_log_y
 */
std::vector<int> get_position_logy () { return m_position_log_y;}
/**
 * @brief Set the position logy object to append the y coordinate into the vector m_position_log_y
 * 
 * @param y int paranter which will be pushed back in the vector
 */
void set_position_logy(int y);
/**
 * @brief Get the m position backtrace object that the robot travelled during forward travel
 * 
 * @return auto return the pair of integer coordinates
 */
auto get_m_position_backtrace() {return m_position_backtrace;}
/**
 * @brief Set the m position backtrace object to append the vector with pair of coordinates at current location
 * 
 * @param x int x coordinate
 * @param y int y coordinate
 */
void set_m_position_backtrace(int x, int y);
/**
 * @brief Get the m final backtrace object for refernce
 * 
 * @return std::vector<std::pair<int, int>>  return type for m_final_backtrace
 */
std::vector<std::pair<int, int>> get_m_final_backtrace(){ return m_final_backtrace;}
/**
 * @brief Set the m final backtrace object to append the vector with optimal path coordinates
 * 
 */
void set_m_final_backtrace(std::pair<int,int> (x));
/**
 * @brief Set the directionnew object to update the direction of the robot at the goal 
 * 
 * @param d char for direction (n,e,w,s)
 */
void set_directionnew(char d);
/**
 * @brief Get the directionnew object that shows the direction at the goal for reference
 * 
 * @return char or direction (n,e,w,s)
 */
char get_directionnew(){return m_new_direction;}



private:
/**
* @brief Positional attribute of the robot
*
*/
std::pair<int, int> m_position;
/**
* @brief Directional attribute of the robot
*
*/
char m_direction{'n'};
/**
 * @brief Direction of the robot at the goal position
 * 
 */
char m_new_direction{'n'};
/**
 * @brief Logs the x coodinate during the forward travel in to a vector
 * 
 */
std::vector<int> m_position_log_x;
/**
 * @brief Logs the y coodinate during the forward travel in to a vector
 * 
 */
std::vector<int> m_position_log_y;
/**
 * @brief Logs the (x,y) pair of the coordinate traversed by the robot during forward motion in the vector
 * 
 */
std::vector<std::pair<int, int>> m_position_backtrace;
/**
 * @brief Logs the (x,y) pair of the most optimised path coordinate for return
 */
std::vector<std::pair<int, int>> m_final_backtrace;
};
} // namespace rwa2group12

#endif