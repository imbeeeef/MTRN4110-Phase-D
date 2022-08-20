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
#include <vector>
#include <queue>
#include <algorithm>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Supervisor.hpp>
#include <math.h>

using namespace std;
using namespace webots;

constexpr double maxMotorSpeed = 6.28;   // rad/s
constexpr double wheel_radius = 0.0205;  // m
constexpr double forwardTimestep = 81.5; // happy with this so far
constexpr double turnTimestep = 36.05;   // for some reason, when i try and use 36, it completly fucks up. Use actual sensors and shit for next parts

const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const std::string MAP_FILE_NAME = "../../Map.txt";
const std::string OUTPUT_FILE_NAME = "../../Output.txt";
const std::string PATH_PLAN_FILE_NAME = "../../MotionPlan.txt";
int const numNodes = 45; // 0-44

class RobotOverLord
{
public:
    // setting up motors and sensors;
    webots::Supervisor robot;
    const int timeStep = static_cast<int>(robot.getBasicTimeStep());
    std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
    std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};

    unique_ptr<webots::DistanceSensor> distSensLeft = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsL"));
    unique_ptr<webots::DistanceSensor> distSensForward = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsF"));
    unique_ptr<webots::DistanceSensor> distSensRight = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor("dsR"));

    std::string motionPlan;
    std::ofstream csvFile;

    int MotionPlanSize = 0;
    int commandNumber = 0;

    // these get updated when string read in
    char heading = 'F';
    int gridLat = 666;
    int gridLong = 666;
    // these also get initialised when string read in
    char obsticalForward = 'N';
    char obsticalLeft = 'N';
    char obsticalRight = 'N';

    void goForward()
    {
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

    void goLeft()
    {
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

    void goRight()
    {
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

    void goDoLap()
    {
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

    void enableDistSensors()
    {
        distSensLeft->enable(timeStep);
        distSensForward->enable(timeStep);
        distSensRight->enable(timeStep);
    }

    // this will update the obsticalLeft/Right/Forward with Y/N
    void checkSensors()
    {
        // check sensors 3 times then take average. It skits out a bit
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

        if (LA > 550)
        {
            obsticalLeft = 'N';
        }
        else
        {
            obsticalLeft = 'Y';
        };

        if (RA > 550)
        {
            obsticalRight = 'N';
        }
        else
        {
            obsticalRight = 'Y';
        };

        if (FA > 750)
        {
            obsticalForward = 'N';
        }
        else
        {
            obsticalForward = 'Y';
        };

        // cout << "LA: " << LA << ", FA: " << FA << ", RA:" << RA << endl;
    }

    void updateHeading(char a)
    {
        switch (a)
        {

        case 'L':
            if (heading == 'N')
            {
                heading = 'W';
            }
            else if (heading == 'E')
            {
                heading = 'N';
            }
            else if (heading == 'S')
            {
                heading = 'E';
            }
            else if (heading == 'W')
            {
                heading = 'S';
            };

            break;
        case 'R':
            if (heading == 'N')
            {
                heading = 'E';
            }
            else if (heading == 'E')
            {
                heading = 'S';
            }
            else if (heading == 'S')
            {
                heading = 'W';
            }
            else if (heading == 'W')
            {
                heading = 'N';
            };

            break;
        }
    }

    void updatePossition()
    {
        switch (heading)
        {
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

    void readMotionPlan()
    {

        std::ifstream myfile(MOTION_PLAN_FILE_NAME);

        if (myfile.is_open())
        {
            myfile >> motionPlan;
            MotionPlanSize = motionPlan.length();
            if (MotionPlanSize > 2)
            {
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
        else
            cout << "couldnt open" << endl;
        myfile.close();
    }

    void createCsvFile()
    {
        csvFile.open(MOTION_EXECUTION_FILE_NAME);
        if (csvFile.is_open())
        {
            csvFile << "Step, Row, Column, Heading, Left Wall, Front Wall, Right Wall, \n";
        }
    }

    void doSteps()
    {

        cout << "[z5162440_MTRN4110_PhaseA] Executing motion plan..." << endl;

        checkSensors();
        printCommandLineAndCsv();

        for (int i = 3; i < MotionPlanSize; i++)
        {
            switch (motionPlan[i])
            {

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

        // idk why but if i dont do this, after the steps are completed, it just starts back up again.
        leftMotor->setPosition(0);
        rightMotor->setPosition(0);
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        cout << "[z5162440_MTRN4110_PhaseA] Motion plan executed!" << endl;
        // csvFile.close();
    }

    void printCommandLineAndCsv()
    {
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

class MapOverLord
{
public:
    // gona use these bad boys for path finding
    std::vector<std::vector<int>> adj;
    vector<vector<int>> paths;
    vector<int> path;
    vector<int> parent[numNodes];

    ofstream outfile;
    ofstream pathfile;

    // these guys gona represent each row in the array, legit just gota rowBuf[2].append ? each call to fgets when reading. Then can easily change elements in each one too
    std::vector<std::string> rowBuf{"[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ",
                                    "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ",
                                    "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ",
                                    "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] ", "[z5162440_MTRN4110_PhaseB] "};

    // these all shout be updated when this class is created
    int end = -1;
    int start = -1;
    char startBearing = 'N';

    int minDist = -1;
    int shortestPathIndex = -1;

    MapOverLord()
    {
        // this is gacked way to do it but fix later
        //  vector of vectors is used to store the graph
        //  in the form of an adjacency list
        // from tute
        outfile.open(OUTPUT_FILE_NAME);
        pathfile.open(PATH_PLAN_FILE_NAME);
        std::vector<std::vector<int>> a(numNodes);
        adj = a;
        readMap();
        printRowBuf();
    }
    void readMap()
    {
        cout << "[z5162440_MTRN4110_PhaseB] Reading in map from " << MAP_FILE_NAME << "..." << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Reading in map from " << MAP_FILE_NAME << "..." << endl;

        /*
            0  1  2  3  4  5  6  7  8
            9  10 11 12 13 14 15 16 17
            18 19 20 21 22 23 24 25 26
            27 28 29 30 31 32 33 34 35
            36 37 38 39 40 41 42 43 44

            37 coll
            11 row
            even rows 0 2 4 6 8 10 are hori walls (0 and 10 are skipped dont care)
            odd rows 1 3 5 7 9 are vert walls AND START AND END POINTS


            !!!!!We go down col by col!!!!!
            for(int i = 0; i<11; i++){
                for(int j = 0; j < 37; j++)
            }


            Even pattern (hori walls):
            skip skip get0
            skip skip skip get1
            skip skip skip get2
            skip skip skip get3
            skip skip skip get4
            skip skip skip get5
            skip skip skip get6
            skip skip skip get7
            skip skip skip get8
            Each get tells you if there is a hori wall. If not, add vertex, if coll = 0||10 dont care, if coll = 2, add between 0/1 1/10 2/11 3/12 ...




            Odd pattern (vert Walls and icons)
                walls from 1 5 9 13 17 21 25 29 33 37
                special icons 3 7 11 15 19 23 27 31 35



            The middle get is for finding the start and end shit, last get finds | wallgh

            */

        std::ifstream myfile(MAP_FILE_NAME);
        std::ifstream outfile(OUTPUT_FILE_NAME);
        int offset = 0;
        char charBuff;
        char charBuff2[1];

        if (myfile.is_open())
        {

            for (int i = 0; i < 11; i++)
            {
                // cout << "row " << i << " starting" << endl;

                // ugly clean later but for maths when inputing node number and edges foundf
                if (i == 1)
                {
                    offset = 0;
                }
                else if (i == 2)
                {
                    offset = 0;
                }
                else if (i == 3)
                {
                    offset = 9;
                }
                else if (i == 4)
                {
                    offset = 9;
                }
                else if (i == 5)
                {
                    offset = 18;
                }
                else if (i == 6)
                {
                    offset = 18;
                }
                else if (i == 7)
                {
                    offset = 27;
                }
                else if (i == 8)
                {
                    offset = 27;
                }
                else if (i == 9)
                {
                    offset = 36;
                }

                // EVEN LOGIC EVEN LOGIC EVEN LOGIC EVEN LOGIC EVEN LOGIC EVEN LOGIC
                //          gota check 3 7 11 15 19 22 27 31 35
                if (i % 2 == 0)
                {
                    if (i == 0 || i == 10)
                    { // we dont really care about these rows, 37 skips
                        for (int j = 0; j < 37; j++)
                        {
                            charBuff = myfile.get();
                            // cout << charBuff << endl;
                            // charBuff2[0] = charBuff;
                            // rowBuf[0].append(charBuff2);
                            appendRowBuf(charBuff, i);
                        }
                    }
                    else
                    {
                        for (int j = 0; j < 38; j++)
                        {
                            charBuff = myfile.get();
                            // if (charBuff == '-') {
                            //     cout << "found wall at: " << j << endl;
                            // }
                            switch (j)
                            {

                            case 3:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset, offset + 9);
                                }
                                break;

                            case 7:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 1, offset + 10);
                                }
                                break;

                            case 11:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 2, offset + 11);
                                }
                                break;

                            case 15:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 3, offset + 12);
                                }
                                break;

                            case 19:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 4, offset + 13);
                                }
                                break;

                            case 23:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 5, offset + 14);
                                }
                                break;

                            case 27:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 6, offset + 15);
                                }
                                break;

                            case 31:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 7, offset + 16);
                                }
                                break;

                            case 35:
                                if (charBuff == ' ')
                                {
                                    addEdge(adj, offset + 8, offset + 17);
                                }
                                break;
                            }
                            appendRowBuf(charBuff, i); // this is my gacked attempt at writing to row buffers cos couldnt figure out type addaption from char to char*. went al' symboly on me
                        }
                    }
                }

                // ODD LOGIC ODD LOGIC ODD LOGIC ODD LOGIC ODD LOGIC ODD LOGIC
                //      Odd pattern(vert Walls and icons)
                //      walls from 1 5 9 13 17 21 25 29 33 37      COL*10-col (i*10-1) gets to right index, + (0+1 5) (9 = 1+2) (13 = 2+3)
                //      special icons 3 7 11 15 19 23 27 31 35
                //
                else
                {
                    for (int j = 0; j < 38; j++)
                    {
                        charBuff = myfile.get();
                        // if (charBuff == '|') {
                        //     cout << "found wall at: " << j << endl;
                        // }
                        // cout << charBuff << endl;

                        switch (j)
                        {

                        case 3:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 5:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset, offset + 1);
                            }
                            break;
                        case 7:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 1;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 1;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 1;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 1;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 1;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 9:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 1, offset + 2);
                            }
                            break;
                        case 11:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 2;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 2;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 2;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 2;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 2;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 13:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 2, offset + 3);
                            }
                            break;
                        case 15:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 3;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 3;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 3;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 3;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 3;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 17:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 3, offset + 4);
                            }
                            break;
                        case 19:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 4;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 4;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 4;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 4;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 4;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 21:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 4, offset + 5);
                            }
                            break;
                        case 23:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 5;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 5;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 5;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 5;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 5;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 25:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 5, offset + 6);
                            }
                            break;
                        case 27:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 6;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 6;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 6;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 6;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 6;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 29:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 6, offset + 7);
                            }
                            break;
                        case 31:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 7;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 7;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 7;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 7;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 7;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        case 33:
                            if (charBuff == ' ')
                            {
                                addEdge(adj, offset + 7, offset + 8);
                            }
                            break;
                        case 35:
                            if (charBuff == 'x' || charBuff == '>' || charBuff == '<' || charBuff == '^' || charBuff == 'v')
                            {
                                if (charBuff == 'x')
                                {
                                    end = offset + 8;
                                }
                                else if (charBuff == '<')
                                {
                                    start = offset + 8;
                                    startBearing = '<';
                                }
                                else if (charBuff == '>')
                                {
                                    start = offset + 8;
                                    startBearing = '>';
                                }
                                else if (charBuff == '^')
                                {
                                    start = offset + 8;
                                    startBearing = '^';
                                }
                                else if (charBuff == 'v')
                                {
                                    start = offset + 8;
                                    startBearing = 'v';
                                }
                            }
                            break;
                        }
                        appendRowBuf(charBuff, i);
                    }
                }
            }
        }
        else
            cout << "couldnt open" << endl;
        myfile.close();
    }
    void addEdge(std::vector<std::vector<int>> &adj, int vertex_a, int vertex_b)
    {
        // copied from tute
        adj[vertex_a].push_back(vertex_b);
        adj[vertex_b].push_back(vertex_a);
    }
    void printStartandEnd()
    {
        cout << "Start at: " << start << " ob bearing: " << startBearing << endl;
        cout << "End at: " << end << endl;
    }
    void printAdjTable()
    {
        // coppied from tute
        int counter{0};
        for (auto vector : adj)
        {
            std::cout << "Node " << counter << " is connected to Node(s): ";
            for (auto elem : vector)
            {
                std::cout << elem << ", ";
            }
            std::cout << std::endl;
            counter++;
        }
    }
    void bfs(int start)
    {
        // this bfs only needs a start and no end.
        // its just meant to find all the parents in the shortest path to each node
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF

        // dist will contain shortest distance
        // from start to every other vertex
        vector<int> dist(numNodes, 200); // all the distances set to 200
        queue<int> q;

        // Insert source vertex in queue and make
        // its parent -1 and distance 0    THIS FROM THE GEEKS FOR GEEKS THING ABOVE
        q.push(start);
        parent[start] = {-1};
        dist[start] = 0;

        // Until Queue is empty
        while (!q.empty())
        {
            int u = q.front();
            q.pop();
            for (int v : adj[u])
            {
                if (dist[v] > dist[u] + 1)
                {

                    // A shorter distance is found
                    // So erase all the previous parents
                    // and insert new parent u in parent[v]
                    dist[v] = dist[u] + 1;
                    q.push(v);
                    parent[v].clear();
                    parent[v].push_back(u);
                }
                else if (dist[v] == dist[u] + 1)
                {

                    // Another candidate parent for
                    // shortes path found
                    parent[v].push_back(u);
                }
            }
        }
        minDist = dist[end];
        cout << "[z5162440_MTRN4110_PhaseB] Finding shortest paths..." << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Finding shortest paths..." << endl;
    }
    void find_paths(int u)
    { // give u end when calling first gota work back
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF
        // CODE BASED OFF https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/   CODE BASED OFF

        // Base Case
        if (u == -1)
        {
            paths.push_back(path);
            return;
        }

        // Loop for all the parents
        // of the given vertex
        for (int par : parent[u])
        {

            // Insert the current
            // vertex in path
            path.push_back(u);

            // Recursive call for its parent
            find_paths(par);

            // Remove the current vertex
            path.pop_back();
        }
    }
    void printAllParents()
    {
        int counter{0};
        for (auto vector : parent)
        {
            cout << "Node " << counter << " has parents: ";
            for (auto elem : vector)
            {
                std::cout << elem << ", ";
            }
            cout << endl;
            counter++;
        }
    }
    void printPaths()
    {
        int counter{1};
        for (auto vector : paths)
        {
            std::cout << "Path:  " << counter << " -> ";
            for (auto elem : vector)
            {
                std::cout << elem << ", ";
            }
            std::cout << std::endl;
            counter++;
        }
    }
    void printRowBuf()
    {

        // char c[20] = "wazzzzzzzzzzup";
        // rowBuf[3].append(c);
        // rowBuf[3].at(1) = 'y';

        // cout << rowBuf[3] << endl;
        cout << rowBuf[0] << endl;
        cout << rowBuf[1] << endl;
        cout << rowBuf[2] << endl;
        cout << rowBuf[3] << endl;
        cout << rowBuf[4] << endl;
        cout << rowBuf[5] << endl;
        cout << rowBuf[6] << endl;
        cout << rowBuf[7] << endl;
        cout << rowBuf[8] << endl;
        cout << rowBuf[9] << endl;
        cout << rowBuf[10] << endl;
        cout << "[z5162440_MTRN4110_PhaseB] Map read in!" << endl;

        outfile << rowBuf[0] << endl;
        outfile << rowBuf[1] << endl;
        outfile << rowBuf[2] << endl;
        outfile << rowBuf[3] << endl;
        outfile << rowBuf[4] << endl;
        outfile << rowBuf[5] << endl;
        outfile << rowBuf[6] << endl;
        outfile << rowBuf[7] << endl;
        outfile << rowBuf[8] << endl;
        outfile << rowBuf[9] << endl;
        outfile << rowBuf[10] << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Map read in!" << endl;
    }
    void appendRowBuf(char cha, int row)
    {
        switch (cha)
        {
        case ' ':
            rowBuf[row].append(" ");
            break;
        case '-':
            rowBuf[row].append("-");
            break;
        case '|':
            rowBuf[row].append("|");
            break;
        case '<':
            rowBuf[row].append("<");
            break;
        case '>':
            rowBuf[row].append(">");
            break;
        case '^':
            rowBuf[row].append("^");
            break;
        case 'v':
            rowBuf[row].append("v");
            break;
        case 'x':
            rowBuf[row].append("x");
            break;
        }
    }
    void printAllShortestPaths()
    {
        // aight we got all the shortest paths in a vector of vectors "paths"
        // i recon for each path copy the rowBuf,
        // somehow append each row depending where it is (probs make a function for this)
        // print to cout and outfile

        int pathLength = paths[1].size();

        int counter{1};
        for (auto vector : paths)
        {
            std::cout << "[z5162440_MTRN4110_PhaseB] Path - " << counter << ":" << endl;
            outfile << "[z5162440_MTRN4110_PhaseB] Path - " << counter << ":" << endl;
            std::vector<std::string> rowBufPath = rowBuf;

            int step = 0; // this just counts down the steps 15 -> 14 - >13 ->12
            for (auto elem : vector)
            {
                // ok in here, for each element, i need to update a specific cell in rowBufPath
                // remember row buff path is legit just a vector of strings that contain the rows we need to print out so can do some mathy thing
                //  also need to know length of path.

                // skip first element
                if (step == pathLength - 1)
                {
                    continue;
                }

                // update the rowBufPath with each step element
                rowBufPath = insertNumIntoPath(rowBufPath, step, elem);
                step++;
            }
            std::cout << std::endl;
            counter++;

            // put in function and in loop later
            cout << rowBufPath[0] << endl;
            cout << rowBufPath[1] << endl;
            cout << rowBufPath[2] << endl;
            cout << rowBufPath[3] << endl;
            cout << rowBufPath[4] << endl;
            cout << rowBufPath[5] << endl;
            cout << rowBufPath[6] << endl;
            cout << rowBufPath[7] << endl;
            cout << rowBufPath[8] << endl;
            cout << rowBufPath[9] << endl;
            cout << rowBufPath[10] << endl;
            outfile << rowBufPath[0] << endl;
            outfile << rowBufPath[1] << endl;
            outfile << rowBufPath[2] << endl;
            outfile << rowBufPath[3] << endl;
            outfile << rowBufPath[4] << endl;
            outfile << rowBufPath[5] << endl;
            outfile << rowBufPath[6] << endl;
            outfile << rowBufPath[7] << endl;
            outfile << rowBufPath[8] << endl;
            outfile << rowBufPath[9] << endl;
            outfile << rowBufPath[10] << endl;
        }
        std::cout << "[z5162440_MTRN4110_PhaseB] " << paths.size() << " shortest paths found!" << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] " << paths.size() << " shortest paths found!" << endl;
    }
    std::vector<std::string> insertNumIntoPath(std::vector<std::string> rowBufPath, int step, int location)
    {

        // if step is double didgit (will never be tripple) gota break it up
        /*
                                     20 offset to start at 0
                                     then add 4 for each row
        [z5162440_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- ---     rowBufPath[0]
        [z5162440_MTRN4110_PhaseB] | v             |                   |    rowBufPath[1]           0  1  2  3  4  5  6  7  8                   // so lets do 5 if statment to select row.
        [z5162440_MTRN4110_PhaseB]      ---                 ---             rowBufPath[2]
        [z5162440_MTRN4110_PhaseB] |           |           |   |       |    rowBufPath[3]           9  10 11 12 13 14 15 16 17                  // then - 9
        [z5162440_MTRN4110_PhaseB]  ---             --- ---     ---         rowBufPath[4]
        [z5162440_MTRN4110_PhaseB] |       |   |   | x                 |    rowBufPath[5]           18 19 20 21 22 23 24 25 26                  // - 18
        [z5162440_MTRN4110_PhaseB]          ---     --- --- ---             rowBufPath[6]                                                                           +20 * num*4          then insert num
        [z5162440_MTRN4110_PhaseB] |   |                   |           |    rowBufPath[7]           27 28 29 30 31 32 33 34 35                  // - 27
        [z5162440_MTRN4110_PhaseB]          ---         ---     ---         rowBufPath[8]
        [z5162440_MTRN4110_PhaseB] |   |           |           |       |    rowBufPath[9]           36 37 38 39 40 41 42 43 44                  // -36
        [z5162440_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- ---     rowBufPath[10]







        */

        char n1 = convIntChar1(step);
        char n2 = convIntChar2(step);

        if (location < 9)
        {
            if (step > 9)
            {
                rowBufPath[1].at(29 + (location * 4)) = n1; // this is just for formatting if number is 2 letters
                rowBufPath[1].at(30 + (location * 4)) = n2;
            }
            else
            {
                rowBufPath[1].at(29 + (location * 4)) = n2;
            }
        }
        else if (location > 8 && location < 18)
        {
            location = location - 9;

            if (step > 9)
            {
                rowBufPath[3].at(29 + (location * 4)) = n1; // this is just for formatting if number is 2 letters
                rowBufPath[3].at(30 + (location * 4)) = n2;
            }
            else
            {
                rowBufPath[3].at(29 + (location * 4)) = n2;
            }
        }
        else if (location > 17 && location < 27)
        {
            location = location - 18;

            if (step > 9)
            {
                rowBufPath[5].at(29 + (location * 4)) = n1; // this is just for formatting if number is 2 letters
                rowBufPath[5].at(30 + (location * 4)) = n2;
            }
            else
            {
                rowBufPath[5].at(29 + (location * 4)) = n2;
            }
        }
        else if (location > 26 && location < 36)
        {
            location = location - 27;

            if (step > 9)
            {
                rowBufPath[7].at(29 + (location * 4)) = n1; // this is just for formatting if number is 2 letters
                rowBufPath[7].at(30 + (location * 4)) = n2;
            }
            else
            {
                rowBufPath[7].at(29 + (location * 4)) = n2;
            }
        }
        else if (location > 35 && location < 45)
        {
            location = location - 36;

            if (step > 9)
            {
                rowBufPath[9].at(29 + (location * 4)) = n1; // this is just for formatting if number is 2 letters
                rowBufPath[9].at(30 + (location * 4)) = n2;
            }
            else
            {
                rowBufPath[9].at(29 + (location * 4)) = n2;
            }
        }

        return rowBufPath;
    }
    char convIntChar1(int x)
    {

        // try typecasting later

        // grab first num
        int a = x % 10;
        x = x - a;
        x = x / 10;
        // here we should have say 2 when given 27
        switch (x)
        {
        case 1:
            return '1';
            break;
        case 2:
            return '2';
            break;
        case 3:
            return '3';
            break;
        case 4:
            return '4';
            break;
        }
    }
    char convIntChar2(int x)
    {

        // try typecasting later

        // this one we just wana retur the 3 in 13
        int a = x % 10;
        switch (a)
        {
        case 1:
            return '1';
            break;
        case 2:
            return '2';
            break;
        case 3:
            return '3';
            break;
        case 4:
            return '4';
            break;
        case 5:
            return '5';
            break;
        case 6:
            return '6';
            break;
        case 7:
            return '7';
            break;
        case 8:
            return '8';
            break;
        case 9:
            return '9';
            break;
        case 0:
            return '0';
            break;
        }
    }
    void findleastTurns()
    {

        // aight so count the turns for each path
        // add these to a vector and find then min

        std::cout << "[z5162440_MTRN4110_PhaseB] Finding shortest path with least turns..." << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Finding shortest path with least turns..." << endl;

        vector<int> numOfTurns;

        for (auto vec : paths)
        {
            int turns = 0;
            int prevEle = -1;
            int prevPrevEle = -1;
            std::vector<int> pathsReversed = vec;
            std::reverse(pathsReversed.begin(), pathsReversed.end());

            for (auto ele : pathsReversed)
            {

                if (prevEle == -1)
                { // we are checking out the first node, skip
                    // dont have to do anything
                }
                else if (prevPrevEle == -1)
                { // we are in second node so check dirrection
                    if (prevEle == ele - 1 && startBearing != '>')
                    {
                        turns++;
                        if (startBearing == '<')
                        {
                            turns++;
                        }
                    } // do double turn if pointing complete wrong way
                    else if (prevEle == ele + 1 && startBearing != '<')
                    {
                        turns++;
                        if (startBearing == '>')
                        {
                            turns++;
                        }
                    }
                    else if (prevEle == ele - 9 && startBearing != 'v')
                    {
                        turns++;
                        if (startBearing == '^')
                        {
                            turns++;
                        }
                    }
                    else if (prevEle == ele + 9 && startBearing != '^')
                    {
                        turns++;
                        if (startBearing == 'v')
                        {
                            turns++;
                        }
                    }
                }
                else
                { // now we are on atleast the third possition in the path so check all in same line
                    if (prevEle == prevPrevEle + 1 && ele != prevEle + 1)
                    {
                        turns++;
                    } // going right
                    else if (prevEle == prevPrevEle - 1 && ele != prevEle - 1)
                    {
                        turns++;
                    } // going left
                    else if (prevEle == prevPrevEle + 9 && ele != prevEle + 9)
                    {
                        turns++;
                    } // going down
                    else if (prevEle == prevPrevEle - 9 && ele != prevEle - 9)
                    {
                        turns++;
                    } // going up
                }

                prevPrevEle = prevEle;
                prevEle = ele;
            }
            // ok now we got the amount of turns in the path
            // add it to the back of the vec

            numOfTurns.push_back(turns);
        }
        // int counter = 1;
        // for (auto yeet : numOfTurns) {
        //     cout << "Path " << counter << " has: " << yeet << " turns" << endl;
        //     counter++;
        // }

        // gota be carefull here cos gona return 0 if its path 1 and 1 if path 2 cos vectors start at 0;
        shortestPathIndex = std::min_element(numOfTurns.begin(), numOfTurns.end()) - numOfTurns.begin();
    }
    void printShortestPath()
    {

        int pathLength = paths[1].size();
        int counter{1};
        std::vector<std::string> rowBufPath = rowBuf;
        int step = 0;

        for (auto elem : paths[shortestPathIndex])
        {
            // ok in here, for each element, i need to update a specific cell in rowBufPath
            // remember row buff path is legit just a vector of strings that contain the rows we need to print out so can do some mathy thing
            //  also need to know length of path.

            // skip first element
            if (step == pathLength - 1)
            {
                continue;
            }

            // update the rowBufPath with each step element
            rowBufPath = insertNumIntoPath(rowBufPath, step, elem);
            step++;
        }
        std::cout << std::endl;
        counter++;

        // put in function and in loop later
        cout << rowBufPath[0] << endl;
        cout << rowBufPath[1] << endl;
        cout << rowBufPath[2] << endl;
        cout << rowBufPath[3] << endl;
        cout << rowBufPath[4] << endl;
        cout << rowBufPath[5] << endl;
        cout << rowBufPath[6] << endl;
        cout << rowBufPath[7] << endl;
        cout << rowBufPath[8] << endl;
        cout << rowBufPath[9] << endl;
        cout << rowBufPath[10] << endl;
        outfile << rowBufPath[0] << endl;
        outfile << rowBufPath[1] << endl;
        outfile << rowBufPath[2] << endl;
        outfile << rowBufPath[3] << endl;
        outfile << rowBufPath[4] << endl;
        outfile << rowBufPath[5] << endl;
        outfile << rowBufPath[6] << endl;
        outfile << rowBufPath[7] << endl;
        outfile << rowBufPath[8] << endl;
        outfile << rowBufPath[9] << endl;
        outfile << rowBufPath[10] << endl;

        std::cout << "[z5162440_MTRN4110_PhaseB] Shortest path with least turns found!" << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Shortest path with least turns found!" << endl;
    }
    void createPathPlan()
    {
        // ok so we got our shortest path in paths[shortestPathIndex]
        // just run through it and append steps to a string

        string pathPlanA = "[z5162440_MTRN4110_PhaseB] Path Plan (";
        // int num = 34;
        // pathPlan += to_string(num);

        int steps = 0;
        int prevEle = -1;
        int prevPrevEle = -1;
        std::vector<int> pathsReversed = paths[shortestPathIndex];
        std::reverse(pathsReversed.begin(), pathsReversed.end());

        string pathPlanB = "";

        //    0  1  2  3  4  5  6  7  8
        //    9  10 11 12 13 14 15 16 17
        //    18 19 20 21 22 23 24 25 26
        //    27 28 29 30 31 32 33 34 35
        //    36 37 38 39 40 41 42 43 44

        for (auto ele : pathsReversed)
        {

            if (prevEle == -1)
            { // we are checking out the first node, skip
                // here gona put in the 00 but have to do it for whatever place it starts
                if (start < 9)
                {
                    pathPlanB += "0";
                    pathPlanB += to_string(start);
                }
                else if (start < 18)
                {
                    pathPlanB += "1";
                    pathPlanB += to_string(start - 9);
                }
                else if (start < 27)
                {
                    pathPlanB += "2";
                    pathPlanB += to_string(start - 18);
                }
                else if (start < 36)
                {
                    pathPlanB += "3";
                    pathPlanB += to_string(start - 27);
                }
                else if (start < 45)
                {
                    pathPlanB += "4";
                    pathPlanB += to_string(start - 36);
                }
                if (startBearing == '^')
                {
                    pathPlanB += "N";
                }
                if (startBearing == '>')
                {
                    pathPlanB += "E";
                }
                if (startBearing == '<')
                {
                    pathPlanB += "W";
                }
                if (startBearing == 'v')
                {
                    pathPlanB += "S";
                }
                prevPrevEle = prevEle;
                prevEle = ele;
                continue;
            }
            else if (prevPrevEle == -1)
            { // we are in second node so check dirrection of start

                if (prevEle == ele - 1 && startBearing != '>')
                { // we went right
                    if (startBearing == '^')
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                    else if (startBearing == '<')
                    {
                        pathPlanB += "LL";
                        steps++;
                        steps++;
                    }
                    else if (startBearing == 'v')
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                }

                else if (prevEle == ele + 1 && startBearing != '<')
                {
                    if (startBearing == '^')
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                    else if (startBearing == '>')
                    {
                        pathPlanB += "LL";
                        steps++;
                    }
                    else if (startBearing == 'v')
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                }
                else if (prevEle == ele - 9 && startBearing != 'v')
                {
                    if (startBearing == '^')
                    {
                        pathPlanB += "LL";
                        steps++;
                        steps++;
                    }
                    else if (startBearing == '<')
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                    else if (startBearing == '>')
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                }
                else if (prevEle == ele + 9 && startBearing != '^')
                {
                    if (startBearing == 'v')
                    {
                        pathPlanB += "LL";
                        steps++;
                        steps++;
                    }
                    else if (startBearing == '<')
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                    else if (startBearing == '>')
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                }
            }
            else
            { // now we are on atleast the third possition in the path so check all in same line
                if (prevEle == prevPrevEle + 1 && ele != prevEle + 1)
                { // --> -->
                    if (prevEle == ele + 9)
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                    if (prevEle == ele - 9)
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                }

                else if (prevEle == prevPrevEle - 1 && ele != prevEle - 1)
                { // <-- <--
                    if (prevEle == ele + 9)
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                    if (prevEle == ele - 9)
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                }

                else if (prevEle == prevPrevEle + 9 && ele != prevEle + 9)
                { // v v
                    if (prevEle == ele + 1)
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                    if (prevEle == ele - 1)
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                }

                else if (prevEle == prevPrevEle - 9 && ele != prevEle - 9)
                { // ^ ^
                    if (prevEle == ele + 1)
                    {
                        pathPlanB += "L";
                        steps++;
                    }
                    if (prevEle == ele - 1)
                    {
                        pathPlanB += "R";
                        steps++;
                    }
                }
            }

            prevPrevEle = prevEle;
            prevEle = ele;
            pathPlanB += "F";
            steps++;
        }
        pathPlanA += to_string(steps);
        pathPlanA += " steps): ";
        pathPlanA += pathPlanB;
        cout << pathPlanA << endl;
        outfile << pathPlanA << endl;
        cout << "[z5162440_MTRN4110_PhaseB] Writing path plan to " << PATH_PLAN_FILE_NAME << "..." << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Writing path plan to " << PATH_PLAN_FILE_NAME << "..." << endl;
        pathfile << pathPlanB << endl;
        cout << "[z5162440_MTRN4110_PhaseB] Path plan written to " << PATH_PLAN_FILE_NAME << "!" << endl;
        outfile << "[z5162440_MTRN4110_PhaseB] Path plan written to " << PATH_PLAN_FILE_NAME << "!" << endl;
    }
    void YEET()
    {
        bfs(start);
        find_paths(end);
        printAllShortestPaths();
        findleastTurns();
        printShortestPath();
        createPathPlan();
    }
};

// IN FUTURE, HAVE EVERYTHING IN A while(robot.timestep != -1)
int main()
{
    system("python Phase_C_Final.py");
    MapOverLord *m = new MapOverLord{};
    m->YEET();
   
    RobotOverLord *myRobot = new RobotOverLord{};
    myRobot->robot.movieStartRecording("../../Video.mp4", 1280, 720, 'M', 25, 2, false);
    const int timeStep = static_cast<int>(myRobot->robot.getBasicTimeStep());
    myRobot->enableDistSensors();
    myRobot->readMotionPlan();
    myRobot->createCsvFile();
    myRobot->doSteps();
    myRobot->robot.step(timeStep);
    myRobot->robot.movieStopRecording();
    
    //Implementing motion tracking program
    while (!myRobot->robot.movieIsReady()){
      myRobot->robot.step(timeStep);
    }
    system("python robotTracking.py");
    
    // int i = 0;
    // while (i < 50)
    // {
        // myRobot.goDoLap();
    // }

    // cout << "yo wadup" << endl;

    return 0;
}