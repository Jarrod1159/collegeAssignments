//
//  gl_frontEnd.h
//  GL threads
//
//  Created by Jean-Yves Hervé on 2023-03-27
//

#ifndef GL_FRONT_END_H
#define GL_FRONT_END_H

#include "glPlatform.h"


//-----------------------------------------------------------------------------
//	Custom data types
//-----------------------------------------------------------------------------


typedef enum ColorLabel {
	BLACK_COL = 0,
	WHITE_COL,
	BLUE_COL,
	GREEN_COL,
	YELLOW_COL,
	RED_COL,
	//
	NB_COLORS
} ColorLabel;

//	Rules of the automaton (in C, it's a lot more complicated than in
//	C++/Java/Python/Swift to define an easy-to-initialize data type storing
//	arrays of numbers.  So, in this program I hard-code my rules
#define GAME_OF_LIFE_RULE	1
#define CORAL_GROWTH_RULE	2
#define AMOEBA_RULE			3
#define MAZE_RULE			4
#define HIGHLIFE_RULE		5
#define SEED_RULE			6


//-----------------------------------------------------------------------------
//	Function prototypes
//-----------------------------------------------------------------------------

void drawGrid(unsigned int**grid, unsigned int numRows, unsigned int numCols);
void drawState(unsigned int numLiveThreads);
void initializeFrontEnd(int argc, char** argv, void (*gridCB)(void), void (*stateCB)(void));

//	Functions implemented in main.c but called byt the glut callback functions
void resetGrid(void);
void oneGenerationV1(void);


#endif // GL_FRONT_END_H

