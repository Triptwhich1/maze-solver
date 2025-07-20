#include "mazeSolver.h"
#include "mazeMapper.h"

/*
 * Draws a cell in the maze, using the columns and rows.
 */
void draw_cell(Maze maze, int columns, int rows) // is always three across based on the starting locations given in the worksheet
{
    static bool first_cell = false;
    Cell current_cell = maze.cells[rows][columns];
    if (current_cell.visited) // make sure the cell isn't visited
    {

        int origin_x_pos;
        int origin_y_pos;

        int right_wall_y_pos;
        int right_wall_x_pos;
        int left_wall_y_pos;
        int left_wall_x_pos;
        int rear_wall_y_pos;
        int rear_wall_x_pos;
        int front_wall_y_pos;
        int front_wall_x_pos;

        if (!first_cell) // starting cell
        {
            first_cell = true;
            origin_x_pos = START_CELL_ORIGIN_X;
            origin_y_pos = START_CELL_ORIGIN_Y;
        }
        else
        {
            origin_x_pos = MAZE_MAX_L_POS + (WIDTH_HEIGHT_CELL + 1) * columns;
            origin_y_pos = MAZE_MAX_B_POS - (WIDTH_HEIGHT_CELL + 1) * rows;
            BTSendNumber(origin_x_pos);
            BTSendString("\n", 4);

            BTSendNumber(origin_y_pos);
            BTSendString("\n", 4);
        }

        if (current_cell.walls.right) // if the cell has a right wall then draw it
        {
            right_wall_x_pos = origin_x_pos;
            right_wall_y_pos = origin_y_pos;
            LCDLine(right_wall_x_pos, right_wall_y_pos, right_wall_x_pos + 6, right_wall_y_pos);
        }
        if (current_cell.walls.left) // if the cell has a left wall
        {
            left_wall_x_pos = origin_x_pos;
            left_wall_y_pos = origin_y_pos + 6;
            LCDLine(left_wall_x_pos, left_wall_y_pos, left_wall_x_pos + 6, left_wall_y_pos);
        }
        if (current_cell.walls.front) // if the cell has a front wall
        {
            front_wall_x_pos = origin_x_pos;
            front_wall_y_pos = origin_y_pos;
            LCDLine(front_wall_x_pos, front_wall_y_pos, front_wall_x_pos, front_wall_y_pos + 6);
        }
        if (current_cell.walls.rear) // if the cell has a rear wall
        {
            rear_wall_x_pos = origin_x_pos + 6;
            rear_wall_y_pos = origin_y_pos;
            LCDLine(rear_wall_x_pos, rear_wall_y_pos, rear_wall_x_pos, rear_wall_y_pos + 6);
        }
    }
}

void draw_special_cell(Maze maze, int columns, int rows, int type)
{
    int x_pos = (MAZE_MAX_L_POS + (WIDTH_HEIGHT_CELL + 1) * columns) + HALF_CELL;
    int y_pos = (MAZE_MAX_B_POS - (WIDTH_HEIGHT_CELL + 1) * rows) - HALF_CELL;

    switch (type)
    {
    case 0: // food
        if (maze.food_x == columns && maze.food_y == rows)
        {
            LCDPlot(x_pos - 1, y_pos - 1); // top left starting pos
            LCDPlot(x_pos, y_pos - 1);
            LCDPlot(x_pos + 1, y_pos - 1);
            LCDPlot(x_pos - 1, y_pos + 1);
            LCDPlot(x_pos, y_pos + 1);
            LCDPlot(x_pos + 1, y_pos + 1); // bottom right point
        }
        break;

    case 1: // water
        if (maze.water_x == columns && maze.water_y == rows)
        {
            LCDPlot(x_pos - 1, y_pos - 1); // top left starting pos
            LCDPlot(x_pos - 1, y_pos);
            LCDPlot(x_pos - 1, y_pos + 1);
            LCDPlot(x_pos + 1, y_pos + 1);
            LCDPlot(x_pos + 1, y_pos);
            LCDPlot(x_pos + 1, y_pos - 1); // bottom right point
        }
        break;

    case 2: // shelter
        if (maze.shelter_x == columns && maze.shelter_y == rows)
        {
            LCDPlot(x_pos, y_pos - 1); // top left starting pos
            LCDPlot(x_pos, y_pos);
            LCDPlot(x_pos - 1, y_pos);
            LCDPlot(x_pos + 1, y_pos);
            LCDPlot(x_pos, y_pos + 1);
        }
        break;

    default:
        break;
    }
}
/*
 * Draws the walls of the maze on the screen, using the maze_max positions
 */
void draw_maze_walls()
{
    LCDLine(MAZE_MAX_L_POS, MAZE_MAX_T_POS, MAZE_MAX_L_POS, MAZE_MAX_B_POS);
    LCDLine(MAZE_MAX_R_POS, MAZE_MAX_T_POS, MAZE_MAX_R_POS, MAZE_MAX_B_POS);
    LCDLine(MAZE_MAX_L_POS, MAZE_MAX_T_POS, MAZE_MAX_R_POS, MAZE_MAX_T_POS);
    LCDLine(MAZE_MAX_L_POS, MAZE_MAX_B_POS, MAZE_MAX_R_POS, MAZE_MAX_B_POS);
}
