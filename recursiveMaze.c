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
#define MOTOR_SPEED_RIGHT 30
#define OBSTACLE_SENSOR_THRESHOLD 150
#define WALL_SENSOR_THRESHOLD 400
#define MOTOR_OFFSET 20
#define BACKWARDS_DISTANCE 20 // Distance to move backwards when avoiding an obstacle
#define ENABLE_DEBUG_IR 1
#define CELLS_IN_MAZE 25;

typedef struct Cell
{
    int visited;
    int cell_num;
    int cell_state;
} Cell;

typedef struct Maze
{
    Cell cells[25];
} Maze;

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
void stop_when_line_hit(unsigned long *pause_start_time)
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

void traverse_maze(unsigned long *pause_start_time)
{
    stop_when_line_hit(pause_start_time);
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

int main(void)
{

    FA_RobotInit();

    FA_LCDBacklight(50);  // Switch on backlight (half brightness)
    FA_DelayMillis(2000); // Pause 2 secs

    unsigned long pause_start_time = 0;

    int num_lines = 0;

    while (1) // Execute this loop as long as robot is running
    {         // (this is equivalent to Arduino loop() function
        traverse_maze(&pause_start_time);
    }
    return 0; // Actually, we should never get here...
}