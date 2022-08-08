/*
Lachlan Rolley z5162440

Developed on Windows
*/


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

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <math.h>

using namespace std;
using namespace webots;

constexpr double maxMotorSpeed = 6.28;  // rad/s
constexpr double wheel_radius = 0.0205;  // m
constexpr double forwardTimestep = 81.5; // happy with this so far 
constexpr double turnTimestep = 36.05; // for some reason, when i try and use 36, it completly fucks up. Use actual sensors and shit for next parts 

const std::string  MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

class RobotOverLord {
public:
    //setting up motors and sensors;
    webots::Robot robot;
    const int timeStep = static_cast<int>(robot.getBasicTimeStep());
    std::unique_ptr<webots::Motor> leftMotor{ robot.getMotor("left wheel motor") };
    std::unique_ptr<webots::Motor> rightMotor{ robot.getMotor("right wheel motor") };

    unique_ptr<webots::DistanceSensor>  distSensLeft = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsL"));
    unique_ptr<webots::DistanceSensor>  distSensForward = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsF"));
    unique_ptr<webots::DistanceSensor>  distSensRight = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsR"));

    std::string motionPlan;
    std::ofstream csvFile;

    int MotionPlanSize = 0;
    int commandNumber = 0;


    //these get updated when string read in
    char heading = 'F';
    int gridLat = 666;
    int gridLong = 666;
    //these also get initialised when string read in
    char obsticalForward = 'N';
    char obsticalLeft = 'N';
    char obsticalRight = 'N';


    void goForward() {
        commandNumber++;
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.5 * maxMotorSpeed);
        rightMotor->setVelocity(0.5 * maxMotorSpeed);
        robot.step(timeStep * forwardTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        updatePossition();
        checkSensors();
        printCommandLineAndCsv();


    }

    void goLeft() {
        commandNumber++;
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(-0.3 * maxMotorSpeed);
        rightMotor->setVelocity(0.3 * maxMotorSpeed);
        robot.step(timeStep * turnTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        updateHeading('L');
        checkSensors();
        printCommandLineAndCsv();


    }

    void goRight() {
        commandNumber++;
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.3 * maxMotorSpeed);
        rightMotor->setVelocity(-0.3 * maxMotorSpeed);
        robot.step(timeStep * turnTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        updateHeading('R');
        checkSensors();
        printCommandLineAndCsv();

    }

    void goDoLap() {
        goForward();
        goLeft();
        goForward();
        goRight();
        goForward();
        goForward();
        goForward();
        goLeft();
        goForward();
        goForward();
        goLeft();
        goForward();
        goRight();
        goForward();
        goRight();
        goForward();
        goLeft();
        goForward();
        goForward();
        goLeft();
        goForward();
        goRight();
        goForward();
        goForward();
        goLeft();
        goForward();
        goForward();
        goForward();
        goLeft();
        goForward();
        goForward();
        goForward();
        goLeft();
        goForward();
        goRight();
        goForward();
        goForward();
        goRight();
        goForward();
        goLeft();
        goForward();
        goForward();
        goForward();
        goLeft();
    }

    void enableDistSensors() {
        distSensLeft->enable(timeStep);
        distSensForward->enable(timeStep);
        distSensRight->enable(timeStep);
    }

    //this will update the obsticalLeft/Right/Forward with Y/N
    void checkSensors() {
        //check sensors 3 times then take average. It skits out a bit
        double L1, L2, L3, LA;
        double R1, R2, R3, RA;
        double F1, F2, F3, FA;

        L1 = distSensLeft->getValue();
        L2 = distSensLeft->getValue();
        L3 = distSensLeft->getValue();
        LA = (L1 + L2 + L3) / 3;

        R1 = distSensRight->getValue();
        R2 = distSensRight->getValue();
        R3 = distSensRight->getValue();
        RA = (R1 + R2 + R3) / 3;


        F1 = distSensForward->getValue();
        F2 = distSensForward->getValue();
        F3 = distSensForward->getValue();
        FA = (F1 + F2 + F3) / 3;
        robot.step(timeStep * 0.1);


        if (LA > 550) { obsticalLeft = 'N'; }
        else { obsticalLeft = 'Y'; };

        if (RA > 550) { obsticalRight = 'N'; }
        else { obsticalRight = 'Y'; };

        if (FA > 750) { obsticalForward = 'N'; }
        else { obsticalForward = 'Y'; };

        //cout << "LA: " << LA << ", FA: " << FA << ", RA:" << RA << endl;

    }

    void updateHeading(char a) {
        switch (a) {

        case 'L':
            if (heading == 'N') { heading = 'W'; }
            else if (heading == 'E') { heading = 'N'; }
            else if (heading == 'S') { heading = 'E'; }
            else if (heading == 'W') { heading = 'S'; };

            break;
        case 'R':
            if (heading == 'N') { heading = 'E'; }
            else if (heading == 'E') { heading = 'S'; }
            else if (heading == 'S') { heading = 'W'; }
            else if (heading == 'W') { heading = 'N'; };

            break;
        }
    }

    void updatePossition() {
        switch (heading) {
        case 'N':
            gridLong--;
            break;
        case 'E':
            gridLat++;
            break;
        case 'S':
            gridLong++;
            break;
        case 'W':
            gridLat--;
            break;
        }
    }

    void readMotionPlan() {

        std::ifstream myfile(MOTION_PLAN_FILE_NAME);
        

        if (myfile.is_open()) {
            myfile >> motionPlan;
            MotionPlanSize = motionPlan.length();
            if (MotionPlanSize > 2) {
                gridLat = stoi(motionPlan.substr(0, 1));
                gridLong = stoi(motionPlan.substr(1, 1));
                heading = motionPlan[2];
                commandNumber = 0;
                checkSensors();
                cout << "[z5162440_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << endl;
                cout << "[z5162440_MTRN4110_PhaseA] Motion Plan: " << motionPlan << endl;
                cout << "[z5162440_MTRN4110_PhaseA] Motion plan read in!" << endl;

            }
        }
        else cout << "couldnt open" << endl;
        myfile.close();
    }

    void createCsvFile() {
        csvFile.open(MOTION_EXECUTION_FILE_NAME);
        if (csvFile.is_open()) {
            csvFile << "Step, Row, Column, Heading, Left Wall, Front Wall, Right Wall, \n";

        }
    }

    void doSteps() {
        
        cout << "[z5162440_MTRN4110_PhaseA] Executing motion plan..." << endl;

        checkSensors();
        printCommandLineAndCsv();

        for (int i = 3; i < MotionPlanSize; i++) {
            switch (motionPlan[i]) {

            case 'F':
                goForward();
                break;
            case 'L':
                goLeft();
                break;

            case 'R':
                goRight();
                break;
            }
        }

        //idk why but if i dont do this, after the steps are completed, it just starts back up again.
        leftMotor->setPosition(0);
        rightMotor->setPosition(0);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        cout << "[z5162440_MTRN4110_PhaseA] Motion plan executed!" << endl;
        //csvFile.close();
    }

    void printCommandLineAndCsv() {
        cout << "[z5162440_MTRN4110_PhaseA] Step: " << std::setw(3) << std::setfill('0') << commandNumber << ", Row: " << gridLong << ", Column: " << gridLat << ", Heading: " << heading << ", Left Wall: " << obsticalLeft << ", Front Wall: " << obsticalForward << ", Right Wall: " << obsticalRight << endl;
        csvFile << commandNumber << ",";
        csvFile << gridLong << ",";
        csvFile << gridLat << ",";
        csvFile << heading << ",";
        csvFile << obsticalLeft << ",";
        csvFile << obsticalForward << ",";
        csvFile << obsticalRight << endl;

    }
};



//IN FUTURE, HAVE EVERYTHING IN A while(robot.timestep != -1)
int main() {




    RobotOverLord myRobot;
    const int timeStep = static_cast<int>(myRobot.robot.getBasicTimeStep());

    /*
    myRobot.enableDistSensors();
    myRobot.readMotionPlan();
    myRobot.createCsvFile();
    myRobot.doSteps();
    myRobot.robot.step(timeStep);

    */

    int i = 0;
    while (i < 50) {
        myRobot.goDoLap();
    }
    
    //cout << "yo wadup" << endl;





    return 0;
}