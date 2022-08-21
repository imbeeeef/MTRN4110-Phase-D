#ifndef JBL_PHASE_D_CONST_H
#define JBL_PHASE_D_CONST_H

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <chrono>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <queue>
#include <algorithm>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Keyboard.hpp>
#include <math.h>

using namespace std;
using namespace webots;

namespace phaseD_constants
{
    constexpr double maxMotorSpeed = 6.28;   // rad/s
    constexpr double wheel_radius = 0.0205;  // m
    constexpr double forwardTimestep = 81.5; // happy with this so far
    constexpr double forwardTimestepFraction = 40.75;
    constexpr double turnTimestep = 36.05; // for some reason, when i try and use 36, it completly fucks up. Use actual sensors and shit for next parts

    const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
    const std::string MAP_FILE_NAME = "../../Map.txt";
    const std::string OUTPUT_FILE_NAME = "../../Output.txt";
    const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
    const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
    int const numNodes = 45; // 0-44
}
#endif