/*
 * File:   recursiveMaze.c
 * Author: harvi
 *
 * Created on 17 April 2025, 15:59
 */

#include "xc.h"
#include "allcode_api.h"

#define CELL_SIZE_CM 150
#define MOTOR_SPEED_LEFT 30
#define MOTOR_SPEED_RIGHT 32
#define OBSTACLE_SENSOR_THRESHOLD 150
#define WALL_SENSOR_THRESHOLD 400
#define MOTOR_OFFSET 20
#define BACKWARDS_DISTANCE 20 // Distance to move backwards when avoiding an obstacle
#define ENABLE_DEBUG_IR 1
#define CELLS_IN_MAZE 25

typedef struct Cell
{
    int visited;
    int cell_state;
} Cell;

typedef struct Maze
{
    Cell cells[CELLS_IN_MAZE]; // cells within the maze
} Maze;

void initialise_maze(Maze *maze)
{
    for (int n = 0; n < CELLS_IN_MAZE; n++)
    {
        maze->cells[n].cell_state = 0; // nothing in them currently
        maze->cells[n].visited = 0;    // unvisited
    }
}

int read_line()
{
    static unsigned long last_time = 0;
    int seen_line = 0;
    unsigned long current_time = FA_ClockMS();

    if ((FA_ReadLine(0) < 100 && FA_ReadLine(1) < 100) && (current_time - last_time > 200))
    {
        last_time = current_time;
        seen_line = 1;
    }
    return seen_line;
}

/*
 * This function makes the robot stops 500ms after a line has been hit to stop
 * in the middle of the cell, for the robot to see what the next moves are for the DFS further in the program
 * @param pause_start_time pointer to when the pause started
 */
void stop_when_line_hit(unsigned long *pause_start_time, int *cell_number)
{
    static int motors_started = 0;
    static int stopping = 0;
    static int number_of_lines = 0;
    static unsigned long line_detect_time = 0;

    if (!motors_started && !stopping)
    {
        FA_SetMotors(MOTOR_SPEED_LEFT, MOTOR_SPEED_RIGHT);
        motors_started = 1;
    }

    if (read_line() && !stopping && line_detect_time == 0)
    {
        number_of_lines++;
        (*cell_number)++;
        FA_LCDNumber(number_of_lines, 60, 4, FONT_NORMAL, LCD_OPAQUE);
        line_detect_time = FA_ClockMS();
    }

    if (line_detect_time != 0 && FA_ClockMS() - line_detect_time >= 500 && !stopping)
    {
        *pause_start_time = FA_ClockMS();
        FA_SetMotors(0, 0);
        motors_started = 0;
        stopping = 1;
        line_detect_time = 0;
    }

    if (stopping && FA_ClockMS() - *pause_start_time >= 500) // Stops the robot after the line has been seen, this stops it for 500ms to get correct sensor readings
    {
        stopping = 0;
    }
}

bool is_viisted(Cell *cell)
{
    if (cell->visited == 1) // cell is visited
    {
        return true;
    }
    else // cell isn't already visited
    {
        return false;
    }
}

bool traverse_maze(unsigned long *pause_start_time, Maze *maze, int *cell_num)
{
    maze->cells[*cell_num].visited = 1; // Marks the current ell as visited

    if (*cell_num == CELLS_IN_MAZE)
    {
        return true;
    }

    if (FA_ReadIR(IR_RIGHT) > OBSTACLE_SENSOR_THRESHOLD)
    {
        FA_Right(90);
        stop_when_line_hit(pause_start_time, cell_num);
        if (!maze->cells[*cell_num].visited)
        {
            if (traverse_maze(maze, cell_num, pause_start_time))
            {
                return true;
            }
        }
        FA_Right(180);
        stop_when_line_hit(pause_start_time, cell_num);
    }

    if (FA_ReadIR(IR_FRONT) > OBSTACLE_SENSOR_THRESHOLD)
    {
        stop_when_line_hit(pause_start_time, cell_num);
        if (!maze->cells[*cell_num].visited)
        {
            if (traverse_maze(maze, cell_num, pause_start_time))
            {
                return true;
            }
        }
        FA_Right(180);
        stop_when_line_hit(pause_start_time, cell_num);
    }

    if (FA_ReadIR(IR_LEFT) > OBSTACLE_SENSOR_THRESHOLD)
    {
        FA_Left(90);
        stop_when_line_hit(pause_start_time, cell_num);
        if (!maze->cells[*cell_num].visited)
        {
            if (traverse_maze(maze, cell_num, pause_start_time))
            {
                return true;
            }
        }
        FA_Right(180);
        stop_when_line_hit(pause_start_time, cell_num);
    }

    return false;

    // if (FA_ReadIR(IR_FRONT) < 150)
    // {
    //     stop_when_line_hit(pause_start_time, cell_num);
    // }
    // else if (FA_ReadIR(IR_RIGHT) < 10 && FA_ReadIR(IR_FRONT) > OBSTACLE_SENSOR_THRESHOLD && FA_ReadIR(IR_LEFT) > 150)
    // {
    //     FA_Left(90);
    // }
    // else if (FA_ReadIR(IR_LEFT) < 10 && FA_ReadIR(IR_FRONT) > OBSTACLE_SENSOR_THRESHOLD && FA_ReadIR(IR_RIGHT) > 150)
    // {
    //     FA_Right(90);
    // }
    // else if (FA_ReadIR(IR_FRONT) > 150 && FA_ReadIR(IR_LEFT) > 150 && FA_ReadIR(IR_RIGHT) > 150)
    // {
    //     FA_Right(180);
    // }
}

void debug_line_sensors()
{
    FA_LCDClear();
    FA_LCDNumber(FA_ReadLine(0), 30, 4, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadLine(1), 60, 4, FONT_NORMAL, LCD_OPAQUE);
    FA_DelayMillis(2000);
}

void debug_IR()
{
    FA_LCDClear();
    FA_LCDNumber(FA_ReadIR(IR_FRONT_LEFT), 30, 20, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_FRONT), 60, 20, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_FRONT_RIGHT), 90, 20, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_RIGHT), 30, 12, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_REAR_RIGHT), 90, 4, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_REAR), 60, 4, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_REAR_LEFT), 30, 4, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(FA_ReadIR(IR_LEFT), 90, 12, FONT_NORMAL, LCD_OPAQUE);
    FA_DelayMillis(2000);
}

void debug_motors(int *motor_r, int *motor_l)
{

    if (FA_ReadSwitch(1) == 1)
    {
        (*motor_r)++;
    }
    if (FA_ReadSwitch(0) == 1)
    {
        (*motor_l)++;
    }
    FA_LCDNumber(*motor_r, 30, 12, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(*motor_l, 90, 12, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDClear();
}

void debug_turn()
{
    if (FA_ReadSwitch(1) == 1)
    {
        FA_Right(90);
    }
    if (FA_ReadSwitch(0) == 1)
    {
        FA_Left(90);
    }
    int left_encoder = FA_ReadEncoder(0);  // Left encoder measures rotation of the left wheel
    int right_encoder = FA_ReadEncoder(1); // Right encoder measures rotation of the right wheel

    FA_LCDNumber(left_encoder, 30, 20, FONT_NORMAL, LCD_OPAQUE);
    FA_LCDNumber(right_encoder, 90, 20, FONT_NORMAL, LCD_OPAQUE);
}

int main(void)
{

    FA_RobotInit();

    FA_LCDBacklight(50);  // Switch on backlight (half brightness)
    FA_DelayMillis(2000); // Pause 2 secs

    unsigned long pause_start_time = 0;
    Maze maze;
    initialise_maze(&maze); // Initialises Maze

    int cell_number = 0;

    int num_lines = 0;

    // int motor_l = MOTOR_SPEED_LEFT;
    // int motor_r = MOTOR_SPEED_RIGHT;

    while (1) // Execute this loop as long as robot is running
    {         // (this is equivalent to Arduino loop() function
              //        traverse_maze(&pause_start_time, &maze, &cell_number);
        // FA_SetMotors(motor_r, motor_l);
        // debug_motors(&motor_r, &motor_l);
        debug_turn();
    }
    return 0; // Actually, we should never get here...
}