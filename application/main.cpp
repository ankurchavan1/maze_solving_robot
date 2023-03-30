/**
 * @file main.cpp
 * @author Aaqib Barodawala
 * @author Ankur Chavan
 * @author Krishna Hundekari
 * @brief This is the main function
 * @version 0.1
 * @date 2022-11-07
 *
 */

#include "simulator/simulator.h"
#include "algorithm/algorithm.h"
#include "robot/robot.h"


#include <iostream>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    
    auto A = std::make_unique<rwa2group12::Algorithm>();
    A->run();

}

