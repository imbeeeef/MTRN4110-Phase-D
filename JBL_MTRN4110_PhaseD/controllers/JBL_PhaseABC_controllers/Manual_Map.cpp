/*
Lachlan Rolley z5162440

Developed on Windows
*/

#include "JBL_PhaseD_Const.h"
using namespace phaseD_constants;

class manualMapRobot
{
public:
    webots::Robot *robot;
    manualMapRobot(webots::Robot robot)
    {
        robot = robot;
    }
    // setting up motors and sensors;

    const int timeStep = static_cast<int>(robot->getBasicTimeStep());
    const std::string OUTPUT_FILE_NAMEMAN = "../../ManualMap.txt"; // new
    std::unique_ptr<webots::Motor> leftMotor{robot->getMotor("left wheel motor")};
    std::unique_ptr<webots::Motor> rightMotor{robot->getMotor("right wheel motor")};

    unique_ptr<webots::DistanceSensor> distSensLeft = std::make_unique<webots::DistanceSensor>(*robot->getDistanceSensor("dsL"));
    unique_ptr<webots::DistanceSensor> distSensForward = std::make_unique<webots::DistanceSensor>(*robot->getDistanceSensor("dsF"));
    unique_ptr<webots::DistanceSensor> distSensRight = std::make_unique<webots::DistanceSensor>(*robot->getDistanceSensor("dsR"));

    ofstream outfile;

    // these get updated when string read in
    char heading = 'F';
    int gridLat = 666;
    int gridLong = 666;
    // these also get initialised when string read in
    char obsticalForward = 'N';
    char obsticalLeft = 'N';
    char obsticalRight = 'N';

    std::vector<std::string> rowBuf{"", "", "", "", "", "", "", "", "", "", ""}; // 11 empty string mby use {11}

    //////////////////////////EXTRA ADDED PART D STUFF ///////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Keyboard keyboard; // new

    int gridChecked[45] = {0};
    int gridWalls[45][4] = {-1}; // 0=N, 1=E, 2=S, 3=W

    //    0  1  2  3  4  5  6  7  8
    //    9  10 11 12 13 14 15 16 17
    //    18 19 20 21 22 23 24 25 26
    //    27 28 29 30 31 32 33 34 35
    //    36 37 38 39 40 41 42 43 44
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void goForward()
    {

        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.5 * maxMotorSpeed);
        rightMotor->setVelocity(0.5 * maxMotorSpeed);
        robot->step(timeStep * forwardTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);

        updatePossition();
        checkSensors();
        updateNeighbours();
    }

    void goLeft()
    {
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(-0.3 * maxMotorSpeed);
        rightMotor->setVelocity(0.3 * maxMotorSpeed);
        robot->step(timeStep * turnTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        updateHeading('L');
        checkSensors();
        // printCommandLineAndCsv();
    }

    void goRight()
    {
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.3 * maxMotorSpeed);
        rightMotor->setVelocity(-0.3 * maxMotorSpeed);
        robot->step(timeStep * turnTimestep); // maths in book
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
        updateHeading('R');
        checkSensors();
        // printCommandLineAndCsv();
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
        double L1, L2, L3, L4, L5, LA;
        double R1, R2, R3, R4, R5, RA;
        double F1, F2, F3, F4, F5, FA;

        // wana average 5 readings
        robot->step(timeStep);
        L1 = distSensLeft->getValue();
        R1 = distSensRight->getValue();
        F1 = distSensForward->getValue();
        robot->step(timeStep);
        L2 = distSensLeft->getValue();
        R2 = distSensRight->getValue();
        F2 = distSensForward->getValue();
        robot->step(timeStep);
        L3 = distSensLeft->getValue();
        R3 = distSensRight->getValue();
        F3 = distSensForward->getValue();
        robot->step(timeStep);
        L4 = distSensLeft->getValue();
        R4 = distSensRight->getValue();
        F4 = distSensForward->getValue();
        robot->step(timeStep);
        L5 = distSensLeft->getValue();
        R5 = distSensRight->getValue();
        F5 = distSensForward->getValue();
        robot->step(timeStep);

        LA = (L1 + L2 + L3 + L4 + L5) / 5;
        RA = (R1 + R2 + R3 + R4 + R5) / 5;
        FA = (F1 + F2 + F3 + F4 + F5) / 5;

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

        robot->step(timeStep);
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

    void printInstructions()
    {
        cout << endl
             << "You have entered Manual Driving Mode: Create Map eddition !" << endl
             << endl;
        cout << "When all the map grid locations have been fully explored, you will exit manual mode" << endl;
        cout << "If you wish to Leave manual mode, simply press L for leave" << endl;

        cout << "Please enter starting grid location" << endl;

        cout << "What row are you starting at ? [0,1,2,3,4]" << endl;
        getKeyRow();

        robot->step(30 * timeStep);

        cout << "What column are you starting at ? [0,1,2,3,4,5,6,7,8]" << endl;
        getKeyCol();

        cout << "What direction are you facing ? [n,e,s,w]" << endl;
        getKeyDirrection();

        cout << endl
             << "Thanks, you are at Lat: " << gridLat << ", Long: " << gridLong << ", facing: " << heading << endl;

        cout << "Press w-a-s-d Keys to move the robot (DONT FORGET, NEED TO CLICK ON THE MAP TO INPUT KEYS !)" << endl;
        cout << "Happy exploring ! (Press 'm' to see which locations you still need to check)" << endl
             << endl;
    }

    void getKeyDirrection()
    {
        int key = 50;
        while (robot->step(timeStep) != -1)
        {
            key = keyboard.getKey();
            if (key == 78 || key == 69 || key == 83 || key == 87)
            {
                break;
            }
        }
        if (key == 78)
        {
            heading = 'N';
        }
        else if (key == 69)
        {
            heading = 'E';
        }
        else if (key == 83)
        {
            heading = 'S';
        }
        else if (key == 87)
        {
            heading = 'W';
        }
    }

    void getKeyCol()
    {
        int key = 50;
        while (robot->step(timeStep) != -1)
        {
            // cout << "yo" << endl;
            key = keyboard.getKey();
            if (key > 47 && key < 58)
            {
                break;
            }
        }
        gridLong = key - 48;
    }

    void getKeyRow()
    {
        int key = 50;
        while (robot->step(timeStep) != -1)
        {
            // cout << "yo" << endl;
            key = keyboard.getKey();
            if (key > 47 && key < 53)
            {
                break;
            }
        }
        gridLat = key - 48;
    }

    int getKeyPressDrive()
    {
        int key = 3;
        // cout << "Please input W,A,S,D or Esc" << endl;
        while (robot->step(timeStep) != -1)
        {
            // cout << "yo" << endl;
            key = keyboard.getKey();
            if (key == 87 || key == 76 || key == 65 || key == 83 || key == 68 || key == 77)
            {
                break;
            }
            // cout << "entered: " << key << endl;
        }
        return key;
    }

    void checkFirstGrid()
    {
        checkSensors();
        int gridLocation = 9 * gridLong + gridLat;
        char startHeading = heading;

        if (heading == 'N')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }
        }
        else if (heading == 'E')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }
        }
        else if (heading == 'S')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }
        }
        else if (heading == 'W')
        {
            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }
        }

        goLeft();
        checkSensors();
        // this is checking the wall dirrectly behind the start point

        if (obsticalLeft == 'N')
        {
            if (startHeading == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else if (startHeading == 'E')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else if (startHeading == 'S')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else if (startHeading == 'W')
            {
                gridWalls[gridLocation][1] = 0;
            }
        }
        else
        {
            if (startHeading == 'N')
            {
                gridWalls[gridLocation][2] = 1;
            }
            else if (startHeading == 'E')
            {
                gridWalls[gridLocation][3] = 1;
            }
            else if (startHeading == 'S')
            {
                gridWalls[gridLocation][0] = 1;
            }
            else if (startHeading == 'W')
            {
                gridWalls[gridLocation][1] = 1;
            }
        }
        goRight();
        gridChecked[gridLocation] = 1;
    }

    void updateNeighbours()
    {
        int gridLocation = 9 * gridLong + gridLat;

        // these 4 if else statemenets are saying there cant be a wall from the dirrectoin the robot just came from
        if (heading == 'N')
        {
            gridWalls[gridLocation][2] = 0;
        }
        else if (heading == 'E')
        {
            gridWalls[gridLocation][3] = 0;
        }
        else if (heading == 'S')
        {
            gridWalls[gridLocation][0] = 0;
        }
        else if (heading == 'W')
        {
            gridWalls[gridLocation][1] = 0;
        }

        // now we use the values from the sensors to tell us if there are walls to the left and the right
        if (heading == 'N')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }
        }
        else if (heading == 'E')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }
        }
        else if (heading == 'S')
        {

            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][1] = 0;
            }
            else
            {
                gridWalls[gridLocation][1] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }
        }
        else if (heading == 'W')
        {
            if (obsticalForward == 'N')
            {
                gridWalls[gridLocation][3] = 0;
            }
            else
            {
                gridWalls[gridLocation][3] = 1;
            }

            if (obsticalLeft == 'N')
            {
                gridWalls[gridLocation][2] = 0;
            }
            else
            {
                gridWalls[gridLocation][2] = 1;
            }

            if (obsticalRight == 'N')
            {
                gridWalls[gridLocation][0] = 0;
            }
            else
            {
                gridWalls[gridLocation][0] = 1;
            }
        }

        gridChecked[gridLocation] = 1;

        // cout << "gridWalls[" << gridLocation << "]" << " has walls, 0:" << gridWalls[gridLocation][0] << " 1:" << gridWalls[gridLocation][1] << " 2:" << gridWalls[gridLocation][2] << " 3:" << gridWalls[gridLocation][3] << endl;
    }

    void move(int direcion)
    {
        int gridLocation = 9 * gridLong + gridLat;

        // cout << "MOVE gridWalls[" << gridLocation << "]" << " has walls, 0:" << gridWalls[gridLocation][0] << " 1:" << gridWalls[gridLocation][1] << " 2:" << gridWalls[gridLocation][2] << " 3:" << gridWalls[gridLocation][3] << endl;
        // cout << "dirrection = " << direcion << endl;
        if (direcion == 87)
        { // up

            if (gridWalls[gridLocation][0] == 1)
            {
                cout << "Sorry, there is a wall there" << endl;
                robot->step(timeStep * 20);
                return;
            }

            if (heading == 'N')
            {
                goForward();
            }
            else if (heading == 'E')
            {
                goLeft();
                goForward();
            }
            else if (heading == 'S')
            {
                goLeft();
                goLeft();
                goForward();
            }
            else if (heading == 'W')
            {
                goRight();
                goForward();
            }
        }

        else if (direcion == 65)
        { // left

            if (gridWalls[gridLocation][3] == 1)
            {
                cout << "Sorry, there is a wall there" << endl;
                robot->step(timeStep * 20);
                return;
            }

            if (heading == 'N')
            {
                goLeft();
                goForward();
            }
            else if (heading == 'E')
            {
                goLeft();
                goLeft();
                goForward();
            }
            else if (heading == 'S')
            {
                goRight();
                goForward();
            }
            else if (heading == 'W')
            {
                goForward();
            }
        }

        else if (direcion == 83)
        { // down

            if (gridWalls[gridLocation][2] == 1)
            {
                cout << "Sorry, there is a wall there" << endl;
                robot->step(timeStep * 20);
                return;
            }

            if (heading == 'N')
            {
                goLeft();
                goLeft();
                goForward();
            }
            else if (heading == 'E')
            {
                goRight();
                goForward();
            }
            else if (heading == 'S')
            {
                goForward();
            }
            else if (heading == 'W')
            {
                goLeft();
                goForward();
            }
        }
        else if (direcion == 68)
        { // right

            if (gridWalls[gridLocation][1] == 1)
            {
                cout << "Sorry, there is a wall there" << endl;
                robot->step(timeStep * 20);
                return;
            }

            if (heading == 'N')
            {
                goRight();
                goForward();
            }
            else if (heading == 'E')
            {
                goForward();
            }
            else if (heading == 'S')
            {
                goLeft();
                goForward();
            }
            else if (heading == 'W')
            {
                goLeft();
                goLeft();
                goForward();
            }
        }
    }

    void printRemaining()
    {
        cout << "Map:" << endl;
        cout << "0  1  2  3  4  5  6  7  8" << endl;
        cout << "9  10 11 12 13 14 15 16 17" << endl;
        cout << "18 19 20 21 22 23 24 25 26" << endl;
        cout << "27 28 29 30 31 32 33 34 35" << endl;
        cout << "36 37 38 39 40 41 42 43 44" << endl;
        cout << "You are missing:" << endl;
        for (int i = 0; i < 45; i++)
        {
            if (gridChecked[i] == 0)
            {
                cout << i << " ";
            }
        }
        cout << endl;
        robot->step(20 * timeStep);
    }

    void makeNotCrash()
    {

        while (robot->step(timeStep) != -1)
        {
        }
    }

    void printAllNeighbours()
    {
        for (int i = 0; i < 45; i++)
        {
            cout << "Grid " << i << ": ";
            for (int j = 0; j < 4; j++)
            {
                if (gridWalls[i][j] == 1)
                {
                    cout << j << " ";
                }
            }
            cout << endl;
        }
    }

    void addVertWalls(int bufLine, int startIndex)
    {

        int end = startIndex + 8;

        rowBuf[bufLine].append("|");
        while (startIndex < end)
        {
            if (gridWalls[startIndex][1] == 1)
            {
                rowBuf[bufLine].append("   |");
            }
            else
            {
                rowBuf[bufLine].append("    ");
            }
            startIndex++;
        }
        rowBuf[bufLine].append("   |");
    }

    void addHoriWalls(int bufLine, int startIndex)
    {

        int end = startIndex + 9;

        rowBuf[bufLine].append(" ");
        while (startIndex < end)
        {
            if (gridWalls[startIndex][2] == 1)
            {
                rowBuf[bufLine].append("--- ");
            }
            else
            {
                rowBuf[bufLine].append("    ");
            }
            startIndex++;
        }
    }

    void printMapTxt()
    {
        outfile.open(OUTPUT_FILE_NAMEMAN);

        /*

         --- --- --- --- --- --- --- --- ---     rowBufPath[0]
        | v             |                   |    rowBufPath[1]           0  1  2  3  4  5  6  7  8
             ---                 ---             rowBufPath[2]
        |           |           |   |       |    rowBufPath[3]           9  10 11 12 13 14 15 16 17
         ---             --- ---     ---         rowBufPath[4]
        |       |   |   | x                 |    rowBufPath[5]           18 19 20 21 22 23 24 25 26
                 ---     --- --- ---             rowBufPath[6]
        |   |                   |           |    rowBufPath[7]           27 28 29 30 31 32 33 34 35
                 ---         ---     ---         rowBufPath[8]
        |   |           |           |       |    rowBufPath[9]           36 37 38 39 40 41 42 43 44
         --- --- --- --- --- --- --- --- ---     rowBufPath[10]

        rowbuf 012 -> grid 0-8

        i recon just append each butter one by one.
        */
        rowBuf[0].append(" --- --- --- --- --- --- --- --- --- ");

        addVertWalls(1, 0);
        addVertWalls(3, 9);
        addVertWalls(5, 18);
        addVertWalls(7, 27);
        addVertWalls(9, 36);

        addHoriWalls(2, 0);
        addHoriWalls(4, 9);
        addHoriWalls(6, 18);
        addHoriWalls(8, 27);

        rowBuf[10].append(" --- --- --- --- --- --- --- --- --- ");

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
    }

    void activateManualDrive()
    {

        printInstructions();
        robot->step(timeStep);

        // game plan is we are just in one big continuous loop till we break out of it
        // keep waiting for input from the user and driving around
        // once the input is ESC or the map has been fully explored we leave
        // once we leave we print the map on a txt file

        bool ContinueLoop = true;

        checkFirstGrid();

        while (ContinueLoop)
        {
            int visitedCellsSum = 0;
            for (int i = 0; i < 45; i++)
            {
                visitedCellsSum = visitedCellsSum + gridChecked[i];
            }
            if (visitedCellsSum == 45)
            {
                cout << "Congrats, you have explored the whole maze" << endl;
                break;
            }

            int key = getKeyPressDrive();
            if (key == 76)
            { // l
                ContinueLoop = false;
                cout << "Warning, maze not fully explored. Map is created but not fully complete" << endl;
                printRemaining();
                break;
            }

            if (key == 77)
            { // esc
                printRemaining();
                continue;
            }
            move(key);
        }
        printMapTxt();
        cout << "Your map is in ../../ManualMap.txt " << endl;

        // printAllNeighbours();
        // cout << "Your map is in the text file Yeet" << endl;
        robot->step(timeStep);
    }
};

int ManualMain(webots::Robot robot)
{

    /////////////////////////////////////////////////////////////////////////////
    ///////////////////    need all this ///////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    // manualMapRobot myRobot(robot);
    manualMapRobot *myRobot = new manualMapRobot(robot);
    const int timeStep = static_cast<int>(myRobot->robot->getBasicTimeStep());
    myRobot->enableDistSensors();
    myRobot->keyboard.enable(timeStep); // added this in main !!!
    myRobot->activateManualDrive();
    myRobot->makeNotCrash();
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////

    return 0;
}