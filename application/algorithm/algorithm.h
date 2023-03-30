/**
 * @file algorithm.h
 * @author Aaqib Barodawala
 * @author Ankur Chavan
 * @author Krishna Hundekari
 * @brief Defining the algorithm.
 * @version 0.1
 * @date 2022-11-07
 *
 */

#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#pragma once

#include<iostream>
#include<memory>
#include<array>
#include"../simulator/simulator.h"
#include<vector>


namespace rwa2group12 {
/**
* @brief This class represents the algorithm to reach the goal in maze.
*/
class Algorithm {
public:
/**
* @brief Initialize the outer walls, generate a random goal, and execute search algorithm.
*/
void run();
/**
* @brief Color the outer walls.
*/
void init_outer_walls();
/**
* @brief Maze solving algorithm with left-hand rule approach.
*/
void follow_wall_left();
/**
* @brief Maze solving algorithm with right-hand rule approach.
*/
void follow_wall_right();
/**
* @brief Generate random goal adjacent to an outer wall.
*/
void generate_goal();
/**
* @brief Set right wall in m_maze and in simulator.
*
* @param x X coordinate of the cell.
* @param y Y coordinate of the cell.
* @param d Direction of the wall. Can be 'n', 's', 'e', 'w'.
*/
void set_right_wall(int x, int y, char d);
/**
* @brief Set left wall in m_maze and in simulator.
*
* @param x X coordinate of the cell.
* @param y Y coordinate of the cell.
* @param d Direction of the wall. Can be 'n', 's', 'e', 'w'.
*/
void set_left_wall(int x, int y, char d);
/**
* @brief Set front wall in m_maze and in simulator.
*
* @param x X coordinate of the cell.
* @param y Y coordinate of the cell.
* @param d Direction of the wall. Can be 'n', 's', 'e', 'w'.
*/
void set_front_wall(int x, int y, char d);
/**
* @brief Set the height of the maze.
*/
void set_maze_height(){
    m_maze_height=Simulator::mazeHeight();
}
/**
* @brief Set the width of the maze.
*/
void set_maze_width(){
    m_maze_width=Simulator::mazeWidth();
}
/**
 * @brief Travererse the shortest return path to initial position
 * 
 */
void trace_back_fastest();
/**
 * @brief Set the m final backtrace algorithm object to copy 
 * the final backtrace coordinates from the robot class
 * 
 */
void set_m_final_backtrace_algorithm(std::vector<std::pair<int, int>> (v));
/**
 * @brief Get the m final dir algorithm object for reference
 * 
 * @return char  return the direction (n,w,e,s)
 */
char get_m_final_dir_algorithm() { return m_final_dir_algorithm;}
/**
 * @brief Set the m final dir algorithm object from the robot class for the direction of robot at goal
 * 
 * @param d takes char as a parameter for the direction (n,w,e,s)
 */
void set_m_final_dir_algorithm(char d);




private:
/**
 * @brief Create a pair for x y coordinates for the goal
 * 
 */
std::pair<int, int> goal;

/**
 * @brief Initialize maze height
 * 
 */
int m_maze_height;

/**
 * @brief Initialize maze width
 * 
 */
int m_maze_width;

/**
 * @brief Store the final direction of robot at the goal
 * 
 */
char m_final_dir_algorithm;

/**
 * @brief Vector of the pair of the cordinates for optimal return travel
 * 
 */
std::vector<std::pair<int, int>> m_final_backtrace_algorithm;

};
} // namespace rwa2group12

#endif