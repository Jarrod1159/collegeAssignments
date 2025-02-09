//
//  main.c
//  Cellular Automaton

/*-------------------------------------------------------------------------+
|	A graphic front end for a grid+state simulation.						|
|																			|
|	This application simply creates a glut window with a pane to display	|
|	a colored grid and the other to display some state information.			|
|	Sets up callback functions to handle menu, mouse and keyboard events.	|
|	Normally, you shouldn't have to touch anything in this code, unless		|
|	you want to change some of the things displayed, add menus, etc.		|
|	Only mess with this after everything else works and making a backup		|
|	copy of your project.  OpenGL & glut are tricky and it's really easy	|
|	to break everything with a single line of code.							|
|																			|
|	Current keyboard controls:												|
|																			|
|		- 'ESC' --> exit the application									|
|		- space bar --> resets the grid										|
|																			|
|		- 'c' --> toggle color mode on/off									|
|		- 'b' --> toggles color mode off/on									|
|		- 'l' --> toggles on/off grid line rendering						|
|																			|
|		- '+' --> increase simulation speed									|
|		- '-' --> reduce simulation speed									|
|																			|
|		- '1' --> apply Rule 1 (Conway's classical Game of Life: B3/S23)	|
|		- '2' --> apply Rule 2 (Coral: B3/S45678)							|
|		- '3' --> apply Rule 3 (Amoeba: B357/S1358)							|
|		- '4' --> apply Rule 4 (Maze: B3/S12345)							|
|																			|
+-------------------------------------------------------------------------*/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <pthread.h>
#include "gl_frontEndv3.h"

//==================================================================================
//	Custom data types
//==================================================================================
using ThreadInfo = struct
{
	int threadIndex;
	unsigned int startRow, endRow;
	pthread_t threadID;
	bool isFinished;
	

};

//==================================================================================
//	Function prototypes
//==================================================================================
void displayGridPane(void);
void displayStatePane(void);
void initializeApplication(void);
void *threadFuncV3(void *);
void swapGrids(void);
void oneThreadGeneration(unsigned int, unsigned int);
unsigned int cellNewState(unsigned int i, unsigned int j);

//==================================================================================
//	Precompiler #define to let us specify how things should be handled at the
//	border of the frame
//==================================================================================

#define FRAME_DEAD 0	//	cell borders are kept dead
#define FRAME_RANDOM 1	//	new random values are generated at each generation
#define FRAME_CLIPPED 2 //	same rule as elsewhere, with clipping to stay within bounds
#define FRAME_WRAP 3	//	same rule as elsewhere, with wrapping around at edges

//	Pick one value for FRAME_BEHAVIOR
#define FRAME_BEHAVIOR FRAME_DEAD

//==================================================================================
//	Application-level global variables
//==================================================================================

//	Don't touch
extern int GRID_PANE, STATE_PANE;
extern int gMainWindow, gSubwindow[2];

//	The state grid and its dimensions.  We now have two copies of the grid:
//		- currentGrid is the one displayed in the graphic front end
//		- nextGrid is the grid that stores the next generation of cell
//			states, as computed by our threads.
unsigned int **currentGrid;
unsigned int **nextGrid;
unsigned int numCols;
unsigned int numRows;
unsigned int numThreads;
int sleepTimer = 5000;
pthread_mutex_t myLock;

//	Piece of advice, whenever you do a grid-based (e.g. image processing),
//	you should always try to run your code with a non-square grid to
//	spot accidental row-col inversion bugs.
// const unsigned int NUM_ROWS = 400, NUM_COLS = 420;

//	the number of live threads (that haven't terminated yet)
unsigned int numLiveThreads = 0;

unsigned int rule = GAME_OF_LIFE_RULE;

unsigned int colorMode = 0;

ThreadInfo *info;
static int generation = 0;


//==================================================================================
//	These are the functions that tie the simulation with the rendering.
//	Some parts are "don't touch."  Other parts need your intervention
//	to make sure that access to critical section is properly synchronized
//==================================================================================

void displayGridPane(void)
{
	//	This is OpenGL/glut magic.  Don't touch
	glutSetWindow(gSubwindow[GRID_PANE]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//---------------------------------------------------------
	//	This is the call that makes OpenGL render the grid.
	//
	//---------------------------------------------------------
	drawGrid(currentGrid, numRows, numCols);

	//	This is OpenGL/glut magic.  Don't touch
	glutSwapBuffers();
	glutSetWindow(gMainWindow);
}

void displayStatePane(void)
{
	//	This is OpenGL/glut magic.  Don't touch
	glutSetWindow(gSubwindow[STATE_PANE]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//---------------------------------------------------------
	//	This is the call that makes OpenGL render information
	//	about the state of the simulation.
	//
	//---------------------------------------------------------
	drawState(numLiveThreads);

	//	This is OpenGL/glut magic.  Don't touch
	glutSwapBuffers();
	glutSetWindow(gMainWindow);
}

pthread_mutex_t** lockArray;

void initializeLockArray() {
    lockArray = new pthread_mutex_t*[numRows];
    for (unsigned int i = 0; i < numRows; i++) {
        lockArray[i] = new pthread_mutex_t[numCols];
        for (unsigned int j = 0; j < numCols; j++) {
            pthread_mutex_init(&lockArray[i][j], NULL);
        }
    }
}

//------------------------------------------------------------------------
//	You shouldn't have to change anything in the main function
//------------------------------------------------------------------------
int main(int argc, char **argv)
{



	// Verify that the program received three arguments
	if (argc != 4)
	{
		std::cerr << "Usage: " << argv[0] << " <width> <height> <numThreads>" << std::endl;
		return 1;
	}

	// Parse the arguments
	numCols = atoi(argv[1]);
	numRows = atoi(argv[2]);
	numThreads = atoi(argv[3]);

	
	if (numCols <= 5 || numRows <= 5 || numThreads <= 0 || numThreads > numRows)
	{
		std::cerr << "Invalid arguments" << std::endl;
		return 1;
	}
	//allocate array

	info = (ThreadInfo *)calloc(numThreads, sizeof(ThreadInfo));
	initializeLockArray();
	//	This takes care of initializing glut and the GUI.
	//	You shouldn’t have to touch this
	initializeFrontEnd(argc, argv, displayGridPane, displayStatePane);

	//	Now we can do application-level initialization
	initializeApplication();




//jyh
//	Yep to what's below.
	
	// not sure whether to call onegen or onethreadgen since you said yep to what's below but also said just call onethread in the readme file
	//oneGenerationV1();
	// loop 1: create all threads
	unsigned int m = numRows / numThreads;
	unsigned int p = numRows % numThreads;
	unsigned int startRow = 0, endRow = m - 1;
	
	for (int k = 0; k < numThreads; k++)
	{
		if (k < p)
			endRow++;

		info[k].threadIndex = k;
		info[k].startRow = startRow;
		info[k].endRow = endRow;
		startRow = endRow + 1;
		endRow += m;

		int errCode = pthread_create(&info[k].threadID, nullptr, threadFuncV3, info + k);
		numLiveThreads++;
		displayStatePane();
		if (errCode != 0)
		{
			printf ("could not pthread_create thread %d. %d/%s\n",
					 k, errCode, strerror(errCode));
			exit (EXIT_FAILURE);
		}
	}

	//	Now we enter the main loop of the program and to a large extend
	//	"lose control" over its execution.  The callback functions that
	//	we set up earlier will be called when the corresponding event
	//	occurs
	glutMainLoop();

	//	This will never be executed (the exit point will be in one of the
	//	call back functions).
	return 0;
}

//==================================================================================
//
//	This is a part that you may have to edit and add to.
//
//==================================================================================

void cleanupAndQuit(void)
{
	//	Free allocated resource before leaving (not absolutely needed, but
	//	just nicer.  Also, if you crash there, you know something is wrong
	//	in your code.
	for (unsigned int i = 1; i < numRows; i++)
	{
		delete[] currentGrid[i];
		delete[] nextGrid[i];
	}
	delete[] currentGrid;
	delete[] nextGrid;

	exit(0);
}

void initializeApplication(void)
{
	//  Allocate 2D grids
	//--------------------
	currentGrid = new unsigned int *[numRows];
	nextGrid = new unsigned int *[numRows];
	for (unsigned int i = 0; i < numRows; i++)
	{
		currentGrid[i] = new unsigned int[numCols];
		nextGrid[i] = new unsigned int[numCols];
	}

	//---------------------------------------------------------------
	//	All the code below to be replaced/removed
	//	I initialize the grid's pixels to have something to look at
	//---------------------------------------------------------------
	//	Yes, I am using the C random generator after ranting in class that the C random
	//	generator was junk.  Here I am not using it to produce "serious" data (as in a
	//	simulation), only some color, in meant-to-be-thrown-away code

	//	seed the pseudo-random generator
	srand((unsigned int)time(NULL));

	resetGrid();
}

//---------------------------------------------------------------------
//	Implement this function
//---------------------------------------------------------------------
// probably not complete i would assume. 
void* threadFuncV3(void* arg)
{
    ThreadInfo* data = static_cast<ThreadInfo*>(arg);

    while (true) {
        oneThreadGeneration(data->startRow,data->endRow);
        data->isFinished = true;

        // Wait for all threads to finish before swapping grids
        bool allThreadsFinished = true;
        for (int i = 0; i < numThreads; i++) {
            if (!info[i].isFinished) {
                allThreadsFinished = false;
				
                break;
            }
        }

        if (allThreadsFinished) {
            swapGrids();
			usleep(sleepTimer);
            generation++;
            for (int i = 0; i < numThreads; i++) {
                info[i].isFinished = false;
            }
        }
    }

    return nullptr;
}

void resetGrid(void)
{
	for (unsigned int i = 0; i < numRows; i++)
	{
		for (unsigned int j = 0; j < numCols; j++)
		{
			nextGrid[i][j] = rand() % 2;
		}
	}
	swapGrids();
}

//	This function swaps the current and next grids, as well as their
//	companion 2D grid.  Note that we only swap the "top" layer of
//	the 2D grids.
void swapGrids(void)
{
	unsigned int **tempGrid = currentGrid;
	currentGrid = nextGrid;
	nextGrid = tempGrid;
}

//	I have decided to go for maximum modularity and readability, at the
//	cost of some performance.  This may seem contradictory with the
//	very purpose of multi-threading our application.  I won't deny it.
//	My justification here is that this is very much an educational exercise,
//	my objective being for you to understand and master the mechanisms of
//	multithreading and synchronization with mutex.  After you get there,
//	you can micro-optimi1ze your synchronized multithreaded apps to your
//	heart's content.



void oneThreadGeneration(unsigned int startRow,unsigned int endRow){

	

	//static int generation = 0;
		for (unsigned int i = startRow; i < endRow; i++) {
			for (unsigned int j = 0; j < numCols; j++) {
				unsigned int randRow = rand()%numRows;
				unsigned int randCol = rand()%numRows;
				pthread_mutex_lock(&lockArray[randRow][randCol]);
				unsigned int newState = cellNewState(randRow, randCol);
		
				//    In black and white mode, only alive/dead matters
				//    Dead is dead in any mode
				if (colorMode == 0 || newState == 0)
				{
					nextGrid[randRow][randCol] = newState;
				}
				//    in color mode, color reflext the "age" of a live cell
				else
				{
					//    Any cell that has not yet reached the "very old cell"
					//    stage simply got one generation older
					if (currentGrid[randRow][randCol] < NB_COLORS-1)
						nextGrid[randRow][randCol] = currentGrid[randRow][randCol] + 1;
					//    An old cell remains old until it dies
					else
						nextGrid[randRow][randCol] = currentGrid[randRow][randCol];

				}
			pthread_mutex_unlock(&lockArray[randRow][randCol]);
        }
	}
}
	// revised version of the handout code that only works on a range of rows.
	
	// but definitely doesn't; mess with generation counter or swapGrids.


//	Here I give three different implementations
//	of a slightly different algorithm, allowing for changes at the border
//	All three variants are used for simulations in research applications.
//	I also refer explicitly to the S/B elements of the "rule" in place.
unsigned int cellNewState(unsigned int i, unsigned int j)
{
	//	First count the number of neighbors that are alive
	//----------------------------------------------------
	//	Again, this implementation makes no pretense at being the most efficient.
	//	I am just trying to keep things modular and somewhat readable
	int count = 0;

	//	Away from the border, we simply count how many among the cell's
	//	eight neighbors are alive (cell state > 0)
	if (i > 0 && i < numRows - 1 && j > 0 && j < numCols - 1)
	{
		//	remember that in C, (x == val) is either 1 or 0
		count = (currentGrid[i - 1][j - 1] != 0) +
				(currentGrid[i - 1][j] != 0) +
				(currentGrid[i - 1][j + 1] != 0) +
				(currentGrid[i][j - 1] != 0) +
				(currentGrid[i][j + 1] != 0) +
				(currentGrid[i + 1][j - 1] != 0) +
				(currentGrid[i + 1][j] != 0) +
				(currentGrid[i + 1][j + 1] != 0);
	}
	//	on the border of the frame...
	else
	{
#if FRAME_BEHAVIOR == FRAME_DEAD

		//	Hack to force death of a cell
		count = -1;

#elif FRAME_BEHAVIOR == FRAME_RANDOM

		count = rand() % 9;

#elif FRAME_BEHAVIOR == FRAME_CLIPPED

		if (i > 0)
		{
			if (j > 0 && currentGrid[i - 1][j - 1] != 0)
				count++;
			if (currentGrid[i - 1][j] != 0)
				count++;
			if (j < NUM_COLS - 1 && currentGrid[i - 1][j + 1] != 0)
				count++;
		}

		if (j > 0 && currentGrid[i][j - 1] != 0)
			count++;
		if (j < NUM_COLS - 1 && currentGrid[i][j + 1] != 0)
			count++;

		if (i < NUM_ROWS - 1)
		{
			if (j > 0 && currentGrid[i + 1][j - 1] != 0)
				count++;
			if (currentGrid[i + 1][j] != 0)
				count++;
			if (j < NUM_COLS - 1 && currentGrid[i + 1][j + 1] != 0)
				count++;
		}

#elif FRAME_BEHAVIOR == FRAME_WRAPPED

		unsigned int iM1 = (i + NUM_ROWS - 1) % NUM_ROWS,
					 iP1 = (i + 1) % NUM_ROWS,
					 jM1 = (j + NUM_COLS - 1) % NUM_COLS,
					 jP1 = (j + 1) % NUM_COLS;
		count = currentGrid[iM1][jM1] != 0 + currentGrid[iM1][j] != 0 + currentGrid[iM1][jP1] != 0 + currentGrid[i][jM1] != 0 + currentGrid[i][jP1] != 0 + currentGrid[iP1][jM1] != 0 + currentGrid[iP1][j] != 0 + currentGrid[iP1][jP1] != 0;

#else
#error undefined frame behavior
#endif

	} //	end of else case (on border)

	//	Next apply the cellular automaton rule
	//----------------------------------------------------
	//	by default, the grid square is going to be empty/dead
	unsigned int newState = 0;

	//	unless....

	switch (rule)
	{
	//	Rule 1 (Conway's classical Game of Life: B3/S23)
	case GAME_OF_LIFE_RULE:

		//	if the cell is currently occupied by a live cell, look at "Stay alive rule"
		if (currentGrid[i][j] != 0)
		{
			if (count == 3 || count == 2)
				newState = 1;
		}
		//	if the grid square is currently empty, look at the "Birth of a new cell" rule
		else
		{
			if (count == 3)
				newState = 1;
		}
		break;

	//	Rule 2 (Coral Growth: B3/S45678)
	case CORAL_GROWTH_RULE:

		//	if the cell is currently occupied by a live cell, look at "Stay alive rule"
		if (currentGrid[i][j] != 0)
		{
			if (count > 3)
				newState = 1;
		}
		//	if the grid square is currently empty, look at the "Birth of a new cell" rule
		else
		{
			if (count == 3)
				newState = 1;
		}
		break;

	//	Rule 3 (Amoeba: B357/S1358)
	case AMOEBA_RULE:

		//	if the cell is currently occupied by a live cell, look at "Stay alive rule"
		if (currentGrid[i][j] != 0)
		{
			if (count == 1 || count == 3 || count == 5 || count == 8)
				newState = 1;
		}
		//	if the grid square is currently empty, look at the "Birth of a new cell" rule
		else
		{
			if (count == 1 || count == 3 || count == 5 || count == 8)
				newState = 1;
		}
		break;

	//	Rule 4 (Maze: B3/S12345)							|
	case MAZE_RULE:

		//	if the cell is currently occupied by a live cell, look at "Stay alive rule"
		if (currentGrid[i][j] != 0)
		{
			if (count >= 1 && count <= 5)
				newState = 1;
		}
		//	if the grid square is currently empty, look at the "Birth of a new cell" rule
		else
		{
			if (count == 3)
				newState = 1;
		}
		break;

		break;

	default:
		printf("Invalid rule number\n");
		exit(5);
	}

	return newState;
}

