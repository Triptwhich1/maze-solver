/*
 * File:   mazeSolver.c
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
    int visited;
    int x;
    int y;
    Walls walls;
    bool is_intersection;
} Cell;

typedef struct Maze
{
    Cell cells[5][5]; // cells within the maze
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
    for (int r = 0; r < 5; r++)
    {
        for (int c = 0; c < 5; c++)
        {
            maze->cells[r][c].visited = 0; // unvisited
            maze->cells[r][c].is_intersection = false;

            maze->cells[r][c].x = r; // sets the x as the rows
            maze->cells[r][c].y = c; // sets the y as the columns

            maze->cells[r][c].walls.front = false;
            maze->cells[r][c].walls.left = false;
            maze->cells[r][c].walls.right = false;
            maze->cells[r][c].walls.rear = false;
        }
    }
}

// void initialise_stack(Stack *stack)
// {
//     stack->top = -1; // empty stack
// }

// void push(Stack *stack, int item)
// {
//     if (stack->top < CELLS_IN_MAZE - 1) // checks for stack overflow
//     {
//         stack->top++;
//         stack->items[stack->top] = item;
//     }
// }

// int pop(Stack *stack)
// {
//     if (stack->top >= 0)
//     {
//         int item = stack->items[stack->top];
//         stack->top--;
//         return item;
//     }
//     else
//     {
//         return -1; // Return an invalid value to indicate the stack is empty
//     }
// }

// int size(Stack *stack)
// {
//     return stack->top + 1;
// }
// /* Left wheel keeps messing up, so this is here to fix that hopefully.
//  */
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

void set_walls(int front, int right, int left, int rear, Walls *walls)
{
    walls->front = (front > OBSTACLE_SENSOR_THRESHOLD / 4); // sets the front wall based on if front > obstacle threashold as it returns either true or false
    walls->right = (right > OBSTACLE_SENSOR_THRESHOLD / 4);
    walls->left = (left > OBSTACLE_SENSOR_THRESHOLD / 4);
    walls->rear = (rear > OBSTACLE_SENSOR_THRESHOLD / 4);

    if (walls->front == true)
    {
        FA_BTSendString("X, ", 4);
    }
    else
    {
        FA_BTSendString("F, ", 4);
    }
    if (walls->left == true)
    {
        FA_BTSendString("X, ", 4);
    }
    else
    {
        FA_BTSendString("L, ", 4);
    }
    if (walls->right == true)
    {
        FA_BTSendString("X, ", 4);
    }
    else
    {
        FA_BTSendString("R, ", 4);
    }
    if (walls->rear == true)
    {
        FA_BTSendString("X, \n", 6);
    }
    else
    {
        FA_BTSendString("B, \n", 6);
    }
}

/*
 * This function makes the robot stops 500ms after a line has been hit to stop
 * in the middle of the cell, for the robot to see what the next moves are
 * @param pause_start_time pointer to when the pause started
 * @param cell_number current cell number that the robot is on
 * @param backtrack flag for when backtracking is or isn't in progress
 * @param stack that the route is being added to
 */
bool stop_when_line_hit(unsigned long *pause_start_time, int *rows, int *columns, bool *backtrack, Robot *robot)
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
        switch (robot->direction)
        {
        case 0:
            (*columns)++;
            break;
        case 1:
            (*rows)++;
            break;
        case 2:
            (*columns)--;
            break;
        case 3:
            (*rows)--;
            break;
        default:
            break;
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

char *get_facing(int direction)
{
    switch (direction)
    {
    case 0:
        return "North";
    case 1:
        return "East";
    case 2:
        return "South";
    case 3:
        return "West";
    default:
        return "";
    }
}

void set_direction(Robot *robot, int turn_type)
{
    switch (turn_type)
    {
    case 1:
        robot->direction = (robot->direction + 1) % 4; // right turn
        FA_BTSendString("Robot is now facing ", 25);
        FA_BTSendString(get_facing(robot->direction), 10);
        FA_BTSendString("\n", 5);
        break;
    case 2:
        robot->direction = (robot->direction + 3) % 4; // left turn
        FA_BTSendString("Robot is now facing ", 25);
        FA_BTSendString(get_facing(robot->direction), 10);
        FA_BTSendString("\n", 5);
        break;
    case 3:
        robot->direction = (robot->direction + 2) % 4; // turn around
        FA_BTSendString("Robot is now facing ", 25);
        FA_BTSendString(get_facing(robot->direction), 10);
        FA_BTSendString("\n", 5);
        break;
    default:
        break;
    }
}

/*
 * This function makes the robot move either right, left or start backtracking based on how many walls surround the current cell
 * @param front sensor value for the front
 * @param right for the right
 * @param left for the left
 * @param *backtrack pointer to backtrack so the value can be changed after a dead end has been reached
 */
void wall_based_movement(Cell cell, bool *backtrack, Robot *robot)
{
    if (cell.walls.front && cell.walls.left && cell.walls.right)
    {
        (*backtrack) = 1; // backtrack enabled
        FA_BTSendString("Backtracking", 20);
        FA_Left(180);
        set_direction(robot, 3); // backtracking turn
    }
    else if (!cell.walls.right) // if there are no walls on the right then turn righ
    {
        FA_BTSendString("Turning right", 20);
        FA_Right(90);
        set_direction(robot, 1); // right turn
    }
    else if (!cell.walls.left) // if no walls on the left then turn left
    {
        FA_BTSendString("Turning left", 20);
        FA_Left(90);
        set_direction(robot, 2); // left turn
    }
    else if (cell.walls.front)
    {
        FA_BTSendString("Front obstacle", 20);
        FA_Right(90); // turn 90 degrees right when there is something in front
        set_direction(robot, 1);
    }
}

void set_intersection(Cell *cell)
{
    if (!cell->is_intersection)
    {
        int open_paths = !cell->walls.front + !cell->walls.left + !cell->walls.right + !cell->walls.rear; // gets number of open paths
        if (open_paths > 2)                                                                               // if it more than 2 then it indicates an intersection
        {
            cell->is_intersection = true;
        }
    }
}

bool traverse_maze(unsigned long *pause_start_time, Maze *maze, bool *backtrack, bool *start_fix, Robot *robot, int *rows, int *columns)
{
    if (maze->cells[*rows][*columns].is_intersection)
    {
        if (*backtrack)
        { // when backtracking isn't true just skip this
            FA_BTSendString("Back at an intersection again\n", 30);
            (*backtrack) = false; // backtracking is completed now that it is in an intersection again
        }
    }

    if (!maze->cells[*rows][*columns].visited)
    {
        maze->cells[*rows][*columns].visited = 1;
    }

    if (FA_ReadIR(IR_FRONT) > 200 && !(*start_fix))
    {
        (*start_fix) = true; // fix now applied
        FA_Right(90);        // specific weird edge case for starting in a spot where the robot needs to be rotated
        FA_BTSendString("Fix happened\n", 30);
    }

    if (stop_when_line_hit(pause_start_time, rows, columns, backtrack, robot))
    {
        (*start_fix) = true; // didn't need fixing

        FA_BTSendString("row: ", 10);
        FA_BTSendNumber(*rows);
        FA_BTSendString("\n", 4);

        FA_BTSendString("column: ", 10);
        FA_BTSendNumber(*columns);
        FA_BTSendString("\n", 4);

        adjust_wheel_encoders();

        int front = FA_ReadIR(IR_FRONT);
        int left = FA_ReadIR(IR_LEFT);
        int right = FA_ReadIR(IR_RIGHT);
        int rear = FA_ReadIR(IR_REAR);

        set_walls(front, right, left, rear, &maze->cells[*rows][*columns].walls); // sets walls of cell

        set_intersection(&maze->cells[*rows][*columns]); // declares if cell is an intersection

        wall_based_movement(maze->cells[*rows][*columns], backtrack, robot); // updates the movement of the cell
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

void debug_walls()
{
    Cell cell;

    int front = FA_ReadIR(IR_FRONT);
    int right = FA_ReadIR(IR_RIGHT);
    int left = FA_ReadIR(IR_LEFT);
    int rear = FA_ReadIR(IR_REAR);

    set_walls(front, right, left, rear, &cell.walls);
}

int main(void)
{

    FA_RobotInit();

    Robot robot;
    robot.direction = 0; // relative direction, which in this case is north

    while (!FA_BTConnected()) // wait until bluetooth is connected
    {
    };

    FA_BTSendNumber(10);

    FA_LCDBacklight(50);  // Switch on backlight (half brightness)
    FA_DelayMillis(2000); // Pause 2 secs

    unsigned long pause_start_time = 0;
    Maze maze;
    initialise_maze(&maze); // Initialises Maze

    int rows = 0;
    int columns = 0;

    bool backtrack = false;
    bool start_fix = false;

    // int motor_l = MOTOR_SPEED_LEFT;
    // int motor_r = MOTOR_SPEED_RIGHT;

    while (1) // Execute this loop as long as robot is running
    {         // (this is equivalent to Arduino loop() function
        traverse_maze(&pause_start_time, &maze, &backtrack, &start_fix, &robot, &rows, &columns);
        //        debug_IR();
        // debug_walls();
        // FA_DelayMillis(3000);
    }
    return 0; // Actually, we should never get here...
}