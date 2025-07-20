#ifndef MAZE_MAPPER
#define MAZE_MAPPER

#define MIN_Y 0
#define MIN_X 0
#define MAX_Y 31
#define MAX_X 127

#define MAZE_MAX_R_POS 80 // enough room for a 31 pixel maze
#define MAZE_MAX_L_POS 49
#define MAZE_MAX_T_POS 0  // top of maze/screen
#define MAZE_MAX_B_POS 31 // bottom of maze/screen

#define WIDTH_HEIGHT_CELL 5
#define HALF_CELL WIDTH_HEIGHT_CELL / 2

#define START_CELL_ORIGIN_X MAZE_MAX_R_POS - WIDTH_HEIGHT_CELL * 3   // gets the origin of the cell (top left corner)
#define START_CELL_ORIGIN_Y (MAZE_MAX_B_POS - 1) - WIDTH_HEIGHT_CELL // gets the origin of the cell (top left corner)

#include "mazeSolver.h"

void draw_cell(Maze maze, int columns, int rows); // is always three across based on the starting locations given in the worksheet
void draw_maze_walls();
void draw_special_cell(Maze maze, int columns, int rows, int type);

#endif