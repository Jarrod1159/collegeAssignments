/*-------------------------------------------------------------------------+
|	Final Project CSC412 - Spring 2023										|
|	A graphic front end for a box-pushing simulation.						|
|																			|
|	This application simply creates a glut window with a pane to display	|
|	a colored grid and the other to display some state information.			|
|																			|
|	Current GUI:															|
|		- 'ESC' --> exit the application									|
|		- ',' --> slows down the simulation									|
|		- '.' --> apeeds up  he simulation									|
|																			|
|	Created by Jean-Yves Hervé on 2018-12-05 (C version)					|
|	Revised 2023-04-27														|
+-------------------------------------------------------------------------*/
//
//  main.cpp
//  Final Project CSC412 - Spring 2023
//
//  Created by Jean-Yves Hervé on 2018-12-05, Rev. 2023-12-01
//	This is public domain code.  By all means appropriate it and change it to your
//	heart's content.
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <random>
#include <vector>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <unistd.h>
#include <mutex>
#include "glPlatform.h"
#include "typesAndConstants.h"
#include "gl_frontEnd.h"

using namespace std;

#if 0
//=================================================================
#pragma mark -
#pragma mark Function prototypes
//=================================================================
#endif

void displayGridPane(void);
void displayStatePane(void);
void printHorizontalBorder(ostringstream &outStream);
string printGrid(void);
void initializeApplication(void);
void cleanupAndQuit();
void generatePartitions();

#if 0
//=================================================================
#pragma mark -
#pragma mark Application-level global variables
//=================================================================
#endif

//	Don't touch
extern int gMainWindow, gSubwindow[2];

//-------------------------------------
//	Don't rename any of these variables
//-------------------------------------
//	The state grid's dimensions (arguments to the program)

int numRows;			//	height of the grid
int numCols;			//	width
int numBoxes = -1;		//	also the number of robots
int numDoors = -1;		//	The number of doors.
int numLiveThreads = 0; //	the number of live robot threads
std::mutex recordLock;
//	robot sleep time between moves (in microseconds)
int robotSleepTime = 1000000;
const char *OUTPUT_FILE_NAME = "robotLog.txt";
//	An array of C-string where you can store things you want displayed
//	in the state pane to display (for debugging purposes?)
//	Dont change the dimensions as this may break the front end
const int MAX_NUM_MESSAGES = 8;
const int MAX_LENGTH_MESSAGE = 32;
char **message;

//	Only absolutely needed if you tackle the partition EC
SquareType **grid;
std::vector<RobotInfo> robots; // use for multithread
std::mutex **locks;
//-----------------------------
//	CHANGE THIS
//-----------------------------
//	Here I hard-code myself some data for robots and doors.  Obviously this code
//	must go away.  I just want to show you how information gets displayed.
//	Obviously, you will need to allocate yourself some dynamic data structure to store
//	that information once you know the dimensions of the grid, number of boxes/robots and
//	doors.
//	Note that, even if you use the GUI version, it doesn't impose you a storage format at
//	all, since the drawing function draw a single robot/box/door at a time, and take that
//	object's parameters as individual arguments.
//	So, feel free to go vectors all the way if you like it better than int**
//	Just don't feel free to declare oversized arrays, in the style of
//	int robotLoc[1000][2];
//	I can guarantee you that this kind of stuff will get penalized harshly (it might have
//	been convenient, borderline cute, in CSC211, but by now it's absolutely embarrassing)
//
//	Also:  Please note that because this is a grid-based problem, I never think of x and y but
//			row and column (well, the "I" dealing with the planning problem.  The "I" doing
//			the dirty work underneath has to translate all of that row and column data into
//			x and y pixel coordinates for OpenGL for rendering
//		   So,... In all of these arrays of 2 int values, the first value (index 0)
//			is a row coordinate and the second value (index 1) is a column coordinate.
// int doorAssign[] = {1, 0, 0, 2, 1, 3};	//	door id assigned to each robot-box pair
// int robotLoc[][2] = {{12, 8}, {6, 9}, {3, 14}, {11, 15}, {14, 1}, {8, 13}};
// int boxLoc[][2] = {{6, 7}, {4, 12}, {13, 13}, {8, 12}, {7, 14}, {11, 9}};
// int doorLoc[][2] = {{3, 3}, {8, 11}, {7, 10}, {12, 6}};

//	The above hard-coded intialization should be replaced by random generation in
//	initializeApplication().
//	Of course, this means that you need to modify the type of the above variables
// int** robotLoc;
// int** boxLoc;
// int** doorAssign;
// int** doorLoc;
//	Or with a bit of retooling
vector<int> doorAssign;
// vector<int> boxAssign;
vector<GridPosition> robotLocation;
vector<GridPosition> boxLocation;
vector<GridPosition> doorLocation;
void robotThreadFunc(RobotInfo robot);
vector<Command> movementPlanner(int robotIndex);
void executeInstructions(int robotindex, vector<Command> instruct);
void moveRobot(Direction dir, int robotindex);
void pushBox(Direction dir, int robotindex);
vector<thread> robotThreads;
RobotInfo robot;
//	For extra credit section
std::random_device randomDevice;
std::default_random_engine engine(randomDevice());
vector<SlidingPartition> partitionList;
//	Change argument to 0.5 for equal probability of vertical and horizontal partitions
//	0 for only horizontal, and 1 for only vertical
bernoulli_distribution headsOrTails(1.0);

#if 0
//=================================================================
#pragma mark -
#pragma mark Function implementations
//=================================================================
#endif

//------------------------------------------------------------------------
//	You shouldn't have to change much in the main function besides
//	the initialization of numRows, numCos, numDoors, numBoxes.
//------------------------------------------------------------------------
int main(int argc, char **argv)
{
	//	We know that the arguments  of the program  are going
	//	to be the width (number of columns) and height (number of rows) of the
	//	grid, the number of boxes (and robots), and the number of doors.
	//	You are going to have to extract these.  For the time being,
	//	I hard code-some values

	if (argc != 5)
	{
		std::cout << "Proper format: " << argv[0] << " <numCols> <numRows> <numDoors> <numBoxes/Robots>" << std::endl;
		return 1;
	}

	numCols = atoi(argv[1]);  // width
	numRows = atoi(argv[2]);  // height
	numDoors = atoi(argv[3]); // doors
	numBoxes = atoi(argv[4]); // boxes (and robots)

	if (numDoors < 1 || numDoors > 3)
	{
		std::cout << " must chose 1, 2, or 3 doors" << std::endl;
	}
	//	Even though we extracted the relevant information from the argument
	//	list, I still need to pass argc and argv to the front-end init
	//	function because that function passes them to glutInit, the required call
	//	to the initialization of the glut library.

	initializeFrontEnd(argc, argv, displayGridPane, displayStatePane);

	//	Now we can do application-level initialization
	// robot.index = 0;
	// robot.isLive = true;
	initializeApplication();
	// robotThreadFunc(robot);
	// robotThreads[0].join();

	string outStr = printGrid();
	cout << outStr << endl;

	//	Now we enter the main loop of the program and to a large extend
	//	"lose control" over its execution.  The callback functions that
	//	we set up earlier will be called when the corresponding event
	//	occurs
	glutMainLoop();

	cleanupAndQuit();

	//	This will probably never be executed (the exit point will be in one of the
	//	call back functions).
	return 0;
}

void cleanupAndQuit()
{
	//	//	Free allocated resource before leaving (not absolutely needed, but
	//	//	just nicer.  Also, if you crash there, you know something is wrong
	//	//	in your code.
	for (int i = 0; i < numRows; i++)
		delete[] grid[i];
	delete[] grid;
	for (int k = 0; k < MAX_NUM_MESSAGES; k++)
		delete[] message[k];
	delete[] message;

	exit(0);
}

void initializeApplication(void)
{

	//	Allocate the grid
	grid = new SquareType *[numRows];
	for (int i = 0; i < numRows; i++)
		grid[i] = new SquareType[numCols];

	message = new char *[MAX_NUM_MESSAGES];
	for (int k = 0; k < MAX_NUM_MESSAGES; k++)
		message[k] = new char[MAX_LENGTH_MESSAGE + 1];

	//---------------------------------------------------------------
	//	This is the place where you should initialize the location
	//	of the doors, boxes, and robots, and create threads (not
	//	necessarily in that order).
	locks = new std::mutex *[numRows];
	for (int i = 0; i < numRows; ++i)
	{
		locks[i] = new std::mutex[numCols];
	}

	std::uniform_int_distribution<int> doorRowDist(1, numRows - 1);
	std::uniform_int_distribution<int> doorColDist(1, numCols - 1);
	std::uniform_int_distribution<int> boxRowDist(1, numRows - 2);
	std::uniform_int_distribution<int> boxColDist(1, numCols - 2);
	std::uniform_int_distribution<int> robotRowDist(0, numRows - 1);
	std::uniform_int_distribution<int> robotColDist(0, numCols - 1);
	std::uniform_int_distribution<int> doorDist(0, numDoors - 1);

	// Place doors randomly on the grid
	for (int i = 0; i < numDoors; i++)
	{
		// Keep generating random row and column until an empty square is found
		while (true)
		{
			int row = doorRowDist(engine);
			int col = doorColDist(engine);
			if (grid[row][col] == SquareType::FREE_SQUARE)
			{
				grid[row][col] = SquareType::DOOR;
				doorLocation.push_back({static_cast<unsigned int>(row), static_cast<unsigned int>(col)});
				// doorPositions.push_back(doorLoc); // Add door position to doorPositions
				break;
			}
		}
	}

	// place boxes randomly on the grid
	for (int i = 0; i < numBoxes; i++)
	{
		// Keep generating random row and column until an empty square is found
		while (true)
		{
			int row = boxRowDist(engine);
			int col = boxColDist(engine);
			if (grid[row][col] == SquareType::FREE_SQUARE)
			{
				grid[row][col] = SquareType::BOX;
				boxLocation.push_back({static_cast<unsigned int>(row), static_cast<unsigned int>(col)}); // Add box position to boxPositions
				break;
			}
		}
	}

	// place robots randomly on the grid. since the number of robots will equal the number of boxes, numBoxes in the for loop is the upper limit.
	for (int i = 0; i < numBoxes; i++)
	{
		// Keep generating random row and column until an empty square is found
		while (true)
		{
			int row = robotRowDist(engine);
			int col = robotColDist(engine);
			if (grid[row][col] == SquareType::FREE_SQUARE)
			{
				grid[row][col] = SquareType::ROBOT;
				// robotPositions.push_back(robotLoc);
				robotLocation.push_back({static_cast<unsigned int>(row), static_cast<unsigned int>(col)});

				// Add robot position to robotPositions
				// this is where the door is assigned randomly to a robot
				doorAssign.push_back(doorDist(engine));
				//		boxAssign.push_back(doorDist(random));
				break;
			}
		}
	}

	// SINCE THIS WAS THE CODE FOR THE TRAVELERS, THIS MIGHT BE BETTER TO CHANGE THIS INTO THREADS FOR OUR ROBOTS SINCE THE ROBOTS ARE THE THREADS IN THIS PROGRAM

	// THIS PART IS NOT FROM THE TRAVELERS. THIS WAS USING A ROBOTS VECTOR AND STRUCT AND SET THE ROBOT BOX PAIR, ONCE WE SET UP AN ARRAY OF PAIRS FOR THE ROBOTS AND BOXES
	// this section is where we will assign the robots to their corresponding boxes and specific door destination. the thread creation can come after that.

	// 1

	//---------------------------------------------------------------

	std::mutex liveTlocks;
	for (int i = 0; i < numBoxes; i++)
	{
		robot.index = i;
		robot.isLive = true;
		robotThreads.push_back(thread(robotThreadFunc, robot));
		liveTlocks.lock();
		numLiveThreads++;
		liveTlocks.unlock();
	}
	

	//	For extra credit
	// generatePartitions();
}

void executeInstructions(int robotindex, Command order)
{

	// might make this into a switch case instead
	if (order.type == MOVE)
	{

		moveRobot(order.dir, robotindex);
	}
	if (order.type == PUSH)
	{
		pushBox(order.dir, robotindex);
	}
}

vector<Command> movementPlanner(int robotIndex)
{
	// Get the positions of the box, robot, and door for the current robotSS
	GridPosition boxPos = boxLocation[robotIndex];
	GridPosition robotPos = robotLocation[robotIndex];
	GridPosition doorPos = doorLocation[doorAssign[robotIndex]];
	vector<Command> instructions;

	// Calculate the horizontal and vertical deltas between the box and door
	int deltaBoxRow = doorPos.row - boxPos.row;
	int deltaBoxCol = doorPos.col - boxPos.col;
	// Calculate the horizontal and vertical deltas between the robot and box
	int deltaRobotRow = boxPos.row - robotPos.row;
	int deltaRobotCol = boxPos.col - robotPos.col;
	bool repositionNeeded = false;
	if (deltaBoxRow != 0 && deltaBoxCol != 0)
	{
		repositionNeeded = true;
	}
	std::cout << repositionNeeded << std::endl;

	// if the box should move to the right,  then the robot must position itself to the box's left
	if (deltaBoxCol > 0)
		deltaRobotCol--;
	else if (deltaBoxCol < 0)
		deltaRobotCol++;

	// if the robot and its box are on the same row, moving in opposite diorections,
	//	move the robot
	if (deltaRobotRow == 0 && deltaRobotCol * deltaBoxCol < 0)
	{
		instructions.push_back(Command{MOVE, NORTH});
		deltaRobotRow = 1;
	}
	int absRobotCol = abs(deltaRobotCol);
	// Move the robot horizontally until it is aligned with the box
	for (int i = 0; i < absRobotCol; i++)
	{ // robot left of box and box is left of door
		if (deltaRobotCol > 0)
		{
			instructions.push_back(Command{MOVE, EAST});
		}
		// robot is right of box and box is right of door
		else
		{
			instructions.push_back(Command{MOVE, WEST});
		}
	}
	int absRobotRow = abs(deltaRobotRow);
	for (int i = 0; i < absRobotRow; i++)
	{

		if (deltaRobotRow > 0)
		{
			instructions.push_back(Command{MOVE, SOUTH});
		}
		else
		{
			instructions.push_back(Command{MOVE, NORTH});
		}
	}
	int absBoxCol = abs(deltaBoxCol);
	for (int i = 0; i < absBoxCol; i++)
	{
		if (deltaBoxCol > 0)
		{
			instructions.push_back(Command{PUSH, EAST});
		}
		else if (deltaBoxCol < 0)
		{
			instructions.push_back(Command{PUSH, WEST});
		}
	}
	int absBoxRow = abs(deltaBoxRow);
	for (int i = 0; i < absBoxRow; i++)
	{
		if (deltaBoxRow > 0)
		{
			if (repositionNeeded)
			{
				if (instructions.back().dir == EAST)
				{
					instructions.push_back(Command{MOVE, NORTH});
					instructions.push_back(Command{MOVE, EAST});
					repositionNeeded = false;
				}
				else if (instructions.back().dir == WEST)
				{
					instructions.push_back(Command{MOVE, NORTH});
					instructions.push_back(Command{MOVE, WEST});
					repositionNeeded = false;
				}
			}
			instructions.push_back(Command{PUSH, SOUTH});
		}
		else if (deltaBoxRow < 0)
		{
			if (repositionNeeded)
			{
				if (instructions.back().dir == EAST)
				{
					instructions.push_back(Command{MOVE, SOUTH});
					instructions.push_back(Command{MOVE, EAST});
					repositionNeeded = false;
				}
				else if (instructions.back().dir == WEST)
				{
					instructions.push_back(Command{MOVE, SOUTH});
					instructions.push_back(Command{MOVE, WEST});
					repositionNeeded = false;
				}
			}
			instructions.push_back(Command{PUSH, NORTH});
		}
	}

	return instructions;
}

void pushBox(Direction dir, int robotindex)
{
	ofstream records;
	records.open(OUTPUT_FILE_NAME, ios::app);
	switch (dir)
	{
	case NORTH:
		// Code to execute when dir is NORTH
		locks[boxLocation[robotindex].row - 1][boxLocation[robotindex].col].lock();
		boxLocation[robotindex].row--;
		robotLocation[robotindex].row--;
		locks[robotLocation[robotindex].row + 1][robotLocation[robotindex].col].unlock();
		recordLock.lock();
		records << "Robot " << robotindex << " push N " << endl;
		recordLock.unlock();
		break;
	case WEST:
		// Code to execute when dir is WEST
		locks[boxLocation[robotindex].row][boxLocation[robotindex].col - 1].lock();
		boxLocation[robotindex].col--;
		robotLocation[robotindex].col--;
		locks[robotLocation[robotindex].row][robotLocation[robotindex].col + 1].unlock();
		recordLock.lock();
		records << "Robot " << robotindex << " push W " << endl;
		recordLock.unlock();
		break;
	case SOUTH:
		// Code to execute when dir is SOUTH
		locks[boxLocation[robotindex].row + 1][boxLocation[robotindex].col].lock();
		boxLocation[robotindex].row++;
		robotLocation[robotindex].row++; 
		locks[robotLocation[robotindex].row - 1][robotLocation[robotindex].col].unlock();
		recordLock.lock();
		records << "Robot " << robotindex << " push S " << endl;
		recordLock.unlock();
		break;
	case EAST:
		// Code to execute when dir is EAST
		locks[boxLocation[robotindex].row][boxLocation[robotindex].col + 1].lock();
		boxLocation[robotindex].col++;
		robotLocation[robotindex].col++;
		locks[robotLocation[robotindex].row][robotLocation[robotindex].col - 1].unlock();
		recordLock.lock();
		records << "Robot " << robotindex << " push E " << endl;
		recordLock.unlock();
		break;
	default:
		// Code to execute when dir is not any of the above values
		break;
	}
}
void moveRobot(Direction dir, int robotindex)
{

	ofstream records;
	records.open(OUTPUT_FILE_NAME, ios::app);

	switch (dir)
	{
	case NORTH:

		locks[robotLocation[robotindex].row - 1][robotLocation[robotindex].col].lock();
		robotLocation[robotindex].row--;
		// Unlock the mutex at the previous robot position
		locks[robotLocation[robotindex].row + 1][robotLocation[robotindex].col].unlock();
		// Lock the log file mutex and write a log message
		recordLock.lock();
		records << "Robot " << robotindex << " move N " << endl;
		recordLock.unlock();

		break;
	case WEST:

		locks[robotLocation[robotindex].row][robotLocation[robotindex].col - 1].lock();
		robotLocation[robotindex].col--;
		locks[robotLocation[robotindex].row][robotLocation[robotindex].col + 1].unlock();

		// Lock the log file mutex and write a log message
		recordLock.lock();
		records << "Robot " << robotindex << " move W " << endl;
		recordLock.unlock();
		break;
	case SOUTH:

		locks[robotLocation[robotindex].row + 1][robotLocation[robotindex].col].lock();
		robotLocation[robotindex].row++;
		locks[robotLocation[robotindex].row - 1][robotLocation[robotindex].col].unlock();

		// Lock the log file mutex and write a log message
		recordLock.lock();
		records << "Robot " << robotindex << " move S " << endl;
		recordLock.unlock();
		break;
	case EAST:

		locks[robotLocation[robotindex].row][robotLocation[robotindex].col + 1].lock();
		robotLocation[robotindex].col++;
		locks[robotLocation[robotindex].row][robotLocation[robotindex].col - 1].unlock();

		// Lock the log file mutex and write a log message
		recordLock.lock();
		records << "Robot " << robotindex << " move E " << endl;
		recordLock.unlock();
		break;
	default:
		break;
	}

	records.close();
}

// multithreaded robots
void robotThreadFunc(RobotInfo robot)
{

	robot.isLive = true;
	vector<Command> orders;
	//	do planning (generate list of robot commands (move/push)
	// where we are going to implement the multithreaded aspect for the robots
	orders = movementPlanner(robot.index);
	// for (int i = 0; i < orders.size(); i++)
	// {

	// 	std::cout << "Order type : " << orders[i].type << "  orders Dir : " << orders[i].dir << std::endl;
	// }
	for (int i = 0; i < orders.size(); i++)
	{

		executeInstructions(robot.index, orders[i]);
		usleep(robotSleepTime);

		//		isAlive = still commands in list of commands
	}
	robotLocation[robot.index].col = -1;
	robotLocation[robot.index].row = -1;
	boxLocation[robot.index].col = -1;
	boxLocation[robot.index].row = -1;
	robot.isLive = false;
	numLiveThreads--;
}

#if 0
//=================================================================
#pragma mark -
#pragma mark You probably don't need to look/edit below
//=================================================================
#endif

//	Rather that writing a function that prints out only to the terminal
//	and then
//		a. restricts me to a terminal-bound app;
//		b. forces me to duplicate the code if I also want to output
//			my grid to a file,
//	I have two options for a "does it all" function:
//		1. Use the stream class inheritance structure (the terminal is
//			an iostream, an output file is an ofstream, etc.)
//		2. Produce an output file and let the caller decide what they
//			want to do with it.
//	I said that I didn't want this course to do too much OOP (and, to be honest,
//	I have never looked seriously at the "stream" class hierarchy), so we will
//	go for the second solution.
string printGrid(void)
{
	// ask professor if we need to change this
	//	some ugly hard-coded stuff
	static string doorStr[] = {"D0", "D1", "D2", "D3", "DD4", "D5", "D6", "D7", "D8", "D9"};
	static string robotStr[] = {"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"};
	static string boxStr[] = {"b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9"};

	if (numDoors > 10 || numBoxes > 10)
	{
		cout << "This function only works for small numbers of doors and robots" << endl;
		exit(1);
	}

	map<int, map<int, string>> strGrid;

	//	addd doors
	for (int k = 0; k < numDoors; k++)
	{
		strGrid[doorLocation[k].row][doorLocation[k].col] = doorStr[k];
	}
	//	add boxes
	for (int k = 0; k < numBoxes; k++)
	{
		strGrid[boxLocation[k].row][boxLocation[k].col] = boxStr[k];
		strGrid[robotLocation[k].row][robotLocation[k].col] = robotStr[k];
	}

	ostringstream outStream;

	//	print top border
	printHorizontalBorder(outStream);

	for (int i = 0; i < numRows; i++)
	{
		outStream << "|";
		for (int j = 0; j < numCols; j++)
		{
			if (strGrid[i][j].length() > 0)
				outStream << " " << strGrid[i][j];
			else
			{
				outStream << " . ";
			}
		}
		outStream << "|" << endl;
	}
	//	print bottom border
	printHorizontalBorder(outStream);

	strGrid.clear();
	return outStream.str();
}

void printHorizontalBorder(ostringstream &outStream)
{
	outStream << "+--";
	for (int j = 1; j < numCols; j++)
	{
		outStream << "---";
	}
	outStream << "-+" << endl;
}

void generatePartitions(void)
{
	const unsigned int NUM_PARTS = (numCols + numRows) / 4;

	//	I decide that a partition length  cannot be less than 3  and not more than
	//	1/4 the grid dimension in its Direction
	const unsigned int MIN_PARTITION_LENGTH = 3;
	const unsigned int MAX_HORIZ_PART_LENGTH = numCols / 4;
	const unsigned int MAX_VERT_PART_LENGTH = numRows / 4;
	const unsigned int MAX_NUM_TRIES = 20;
	uniform_int_distribution<unsigned int> horizPartLengthDist(MIN_PARTITION_LENGTH, MAX_HORIZ_PART_LENGTH);
	uniform_int_distribution<unsigned int> vertPartLengthDist(MIN_PARTITION_LENGTH, MAX_VERT_PART_LENGTH);
	uniform_int_distribution<unsigned int> rowDist(1, numRows - 2);
	uniform_int_distribution<unsigned int> colDist(1, numCols - 2);

	for (unsigned int w = 0; w < NUM_PARTS; w++)
	{
		//	Case of a vertical partition
		if (headsOrTails(engine))
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a column index
				unsigned int col = colDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startRow = 1 + rowDist(engine) % (numRows - length - 1);
				for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the partition is possible,
				if (goodPart)
				{
					//	add it to the grid and to the partition list
					SlidingPartition part;
					part.isVertical = true;
					for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
					{
						grid[row][col] = SquareType::VERTICAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
		// case of a horizontal partition
		else
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a row index
				unsigned int row = rowDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startCol = 1 + colDist(engine) % (numCols - length - 1);
				for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the wall first, add it to the grid and build SlidingPartition object
				if (goodPart)
				{
					SlidingPartition part;
					part.isVertical = false;
					for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
					{
						grid[row][col] = SquareType::HORIZONTAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
	}
}
