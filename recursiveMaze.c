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
#define OBSTACLE_SENSOR_THRESHOLD 200
#define MOTOR_OFFSET 20
#define BACKWARDS_DISTANCE 20 // Distance to move backwards when avoiding an obstacle
#define ENABLE_DEBUG_IR 1
#define CELLS_IN_MAZE 25

typedef struct Cell
{
    int visited;
    bool is_intersection;
} Cell;

typedef struct Maze
{
    Cell cells[CELLS_IN_MAZE]; // cells within the maze
    int shelter_pos;
    int food_pos;
    int water_pos;
} Maze;

typedef struct
{
    int items[CELLS_IN_MAZE];
    int top;
} Stack;

void initialise_maze(Maze *maze)
{
    for (int n = 0; n < CELLS_IN_MAZE; n++)
    {
        maze->cells[n].visited = 0; // unvisited
        maze->cells[n].is_intersection = false;
    }
}

void initialise_stack(Stack *stack)
{
    stack->top = -1; // empty stack
}

void push(Stack *stack, int item)
{
    if (stack->top < CELLS_IN_MAZE - 1) // checks for stack overflow
    {
        stack->top++;
        stack->items[stack->top] = item;
    }
}

int pop(Stack *stack)
{
    if (stack->top >= 0)
    {
        int item = stack->items[stack->top];
        stack->top--;
        return item;
    }
    else
    {
        return -1; // Return an invalid value to indicate the stack is empty
    }
}

int size(Stack *stack)
{
    return stack->top + 1;
}
/* Left wheel keeps messing up, so this is here to fix that hopefully.
 */
void adjust_wheel_encoders()
{
    int left_encoder = FA_ReadEncoder(0);
    int right_encoder = FA_ReadEncoder(1);

    int difference = left_encoder - right_encoder;
    if (difference > 15)
    {
        FA_SetMotors(MOTOR_SPEED_LEFT, MOTOR_SPEED_RIGHT);
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
 * in the middle of the cell, for the robot to see what the next moves are
 * @param pause_start_time pointer to when the pause started
 */
bool stop_when_line_hit(unsigned long *pause_start_time, int *cell_number, bool *backtrack, Stack *stack)
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
        if (*cell_number < CELLS_IN_MAZE - 1 && !(*backtrack))
        {
            if (stack->top < CELLS_IN_MAZE - 1)
            {
                push(stack, *cell_number);
                FA_BTSendString("cell added to stack \n", 30);
                FA_BTSendNumber(size(stack));
                (*cell_number)++;
            }
        }
        else if (*backtrack)
        {
            if (stack->top >= 0)
            {
                pop(stack); // Pop the last cell from the stack when backtracking
                FA_BTSendString("cell removed from stack \n", 30);
            }
            (*cell_number)--;
        }
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
        return true; // finished stopping
    }
    return false;
}

/*
 * This function makes the robot move either right, left or start backtracking
 * @param front sensor value for the front
 * @param right for the right
 * @param left for the left
 * @param *backtrack pointer to backtrack so the value can be changed after a dead end has been reached
 */
void movement(int front, int right, int left, bool *backtrack)
{
    if (front > OBSTACLE_SENSOR_THRESHOLD && left > OBSTACLE_SENSOR_THRESHOLD && right > OBSTACLE_SENSOR_THRESHOLD)
    {
        (*backtrack) = 1; // backtrack enabled
        FA_BTSendString("Backtracking", 20);
        FA_Left(180);
    }
    else if (right < 20 && left > OBSTACLE_SENSOR_THRESHOLD / 2) // obstacle threashold is now 125, so it will be detected from further
    {
        FA_BTSendString("Turning right", 20);
        FA_Right(90);
    }
    else if (left < 20 && right > OBSTACLE_SENSOR_THRESHOLD / 2)
    {
        FA_BTSendString("Turning left", 20);
        FA_Left(90);
    }
    else if (front > OBSTACLE_SENSOR_THRESHOLD / 2) // front is now detected much further away
    {
        FA_BTSendString("Front obstacle in the way", 40);
        FA_Right(90); // if it is more than the threashold then turn right (random direction)
    }
}

bool traverse_maze(unsigned long *pause_start_time, Maze *maze, int *cell_num, bool *backtrack, Stack *stack, bool *start_fix)
{
    if (maze->cells[*cell_num].is_intersection)
    {
        if (*backtrack)
        { // when backtracking isn't true just skip this
            FA_BTSendString("Back at an intersection again\n", 30);
            (*backtrack) = false; // backtracking is completed now that it is in an intersection again
        }
    }

    if (!maze->cells[*cell_num].visited)
    {
        maze->cells[*cell_num].visited = 1;
        FA_LCDNumber(*cell_num, 60, 16, FONT_NORMAL, LCD_OPAQUE);
    }

    if (*cell_num == CELLS_IN_MAZE - 1)
    {
        return true;
    }

    if (FA_ReadIR(IR_FRONT) > 300 && !(*start_fix))
    {
        (*start_fix) = true; // fix now applied
        FA_Right(90);        // specific weird edge case for starting in a spot where the robot needs to be rotated
        FA_BTSendString("Fix happened\n", 30);
    }

    if (stop_when_line_hit(pause_start_time, cell_num, backtrack, stack))
    {
        (*start_fix) = true; // didn't need fixing

        adjust_wheel_encoders();

        int front = FA_ReadIR(IR_FRONT);
        int left = FA_ReadIR(IR_LEFT);
        int right = FA_ReadIR(IR_RIGHT);

        if (((front < 10 && left < 10) || (front < 10 && right < 10) || (left < 10 && right < 10)) && !maze->cells[*cell_num].is_intersection) // marks the cell as an intersection
        {
            maze->cells[*cell_num].is_intersection = true;
            FA_BTSendString("Intersection found\n", 30);
        }

        movement(front, right, left, backtrack);
    }
    return false;
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

    bool state = FA_BTConnected();
    while (!state)
    {
        state = FA_BTConnected();
    }
    FA_BTSendNumber(10);

    FA_LCDBacklight(50);  // Switch on backlight (half brightness)
    FA_DelayMillis(2000); // Pause 2 secs

    unsigned long pause_start_time = 0;
    Maze maze;
    initialise_maze(&maze); // Initialises Maze

    Stack stack;
    initialise_stack(&stack);

    bool backtrack = false;
    int cell_number = 0;
    int num_lines = 0;
    bool start_fix = false;

    // int motor_l = MOTOR_SPEED_LEFT;
    // int motor_r = MOTOR_SPEED_RIGHT;

    while (1) // Execute this loop as long as robot is running
    {         // (this is equivalent to Arduino loop() function
        traverse_maze(&pause_start_time, &maze, &cell_number, &backtrack, &stack, &start_fix);
        //        debug_IR();
    }
    return 0; // Actually, we should never get here...
}