#ifndef MAZE_SOLVER
#define MAZE_SOLVER

#include <stdbool.h>

typedef struct Robot
{
    int direction; // N - 0, E - 1, S - 2, W - 3;
} Robot;

typedef struct Walls
{
    bool left;
    bool right;
    bool front;
    bool rear;
} Walls;

typedef struct Cell
{
    int visited;          // int if the cell is visited
    int x;                // cell x
    int y;                // cell y
    Walls walls;          // walls surrounding the walls
    bool is_intersection; // if the cell is an intersection
} Cell;

typedef struct Maze
{
    Cell cells[7][7]; // cells within the maze (with offset to stop negative cells!)
    int shelter_x;    // shelter x pos
    int shelter_y;    // shelter y pos
    int food_x;       // food x pos
    int food_y;       // food y pos
    int water_x;      // water x pos
    int water_y;      // water y pos
} Maze;

#endif