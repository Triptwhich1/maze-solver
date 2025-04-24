/*
 * File:   mazeSolver.c
 * Author: harvi
 *
 * Created on 17 April 2025, 15:59
 */

#include "xc.h"
#include "allcode_api.h"
#include <stdbool.h>

#define MOTOR_SPEED_LEFT 30
#define MOTOR_SPEED_RIGHT 30
#define OBSTACLE_SENSOR_THRESHOLD 200

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
    Cell cells[7][7]; // cells within the maze (with offset to stop negative cells!)
    int shelter_pos;
    int food_pos;
    int water_pos;
} Maze;

void initialise_maze(Maze *maze)
{
    for (int r = 0; r < 7; r++)
    {
        for (int c = 0; c < 7; c++)
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

void adjust_for_wall()
{
    int front_left = FA_ReadIR(IR_FRONT_LEFT);
    int front_right = FA_ReadIR(IR_FRONT_RIGHT);
    int left = FA_ReadIR(IR_LEFT);
    int right = FA_ReadIR(IR_RIGHT);
}

void set_walls(int front, int right, int left, int rear, Walls *walls)
{
    walls->front = (front > OBSTACLE_SENSOR_THRESHOLD / 5); // sets the front wall based on if front > obstacle threashold as it returns either true or false
    walls->right = (right > OBSTACLE_SENSOR_THRESHOLD / 5);
    walls->left = (left > OBSTACLE_SENSOR_THRESHOLD / 5);
    walls->rear = (rear > OBSTACLE_SENSOR_THRESHOLD / 5);

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

void cell_to_grid(int direction, int *rows, int *columns)
{
    switch (direction)
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
}

/*
 * This function makes the robot stops 500ms after a line has been hit to stop
 * in the middle of the cell, for the robot to see what the next moves are
 * @param *pause_start_time pointer to when the pause_starts
 * @param *rows pointer to rows passed in
 * @param *columns pointer to the columns passed in
 * @param *backtrack backtrack flag for when the robot needs to backtrack to get to unexplored cells
 * @param *robot robot passed in, gives access to direction of the robot
 */
bool stop_when_line_hit(unsigned long *pause_start_time, int *rows, int *columns, bool *backtrack, Robot *robot)
{
    static int motors_started = 0;             // motors started flag
    static int stopping = 0;                   // stopping flag
    static unsigned long line_detect_time = 0; // ms passed since a line has been detected

    if (!motors_started && !stopping) // starts the motors at the beginning of the program, as after it needs to see a line to continue forward
    {
        if (FA_ReadIR(IR_FRONT) > OBSTACLE_SENSOR_THRESHOLD / 4) // specific edge case where robot starts facing a wall
        {
            FA_Right(90); // turns right
        }
        FA_SetMotors(MOTOR_SPEED_LEFT, MOTOR_SPEED_RIGHT); // motor then starts
        motors_started = 1;                                // started flag now positive
    }

    if (read_line() && !stopping && line_detect_time == 0) // checks if a line is detected, robot isn't stopping and if the line hasn't been detected recently
    {
        cell_to_grid(robot->direction, rows, columns);
        line_detect_time = FA_ClockMS();
    }

    if (line_detect_time != 0 && FA_ClockMS() - line_detect_time >= 500 && !stopping) // pauses the robot after a line has been detected
    {
        *pause_start_time = FA_ClockMS(); // gets the time when the pause started
        FA_SetMotors(0, 0);               // actually stops the robot
        motors_started = 0;
        stopping = 1; // puts the robot in a stopped state
        line_detect_time = 0;
    }

    if (stopping && FA_ClockMS() - *pause_start_time >= 500) // checks if robot has been stopped for a long enough time i.e. 500ms
    {
        stopping = 0;
        return true; // finished stopping
    }
    return false;
}

// remove this when submitting as it isn't needed
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

/*
 * Sets the direction of the robot after a turn has been made
 * @param *robot pointer to robot to update the direction after a turn has been made
 * @param turn_type is the type of turn taken, i.e. 1 is a right turn meaning that the direction is incremented once
 *        north -> east etc.
 */
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
 * This function allows the robot to move based on the amount of walls surrounding it and
 * allows for the prediction of the next cell that the robot is going to go into, to make sure
 * it doesn't visit already visited cells
 * @param maze passed in maze to get the current cell that the robot is in
 * @param row gets the cell that the robot is currently in, and to predict the next cell the robot will be in
 * @param column same as row
 * @param *backtrack backtracking flag, when robot meets a dead end it can go into cells that it has already
 *        visited before
 * @param *robot pointer to the robot, allows for the direction to updated if the robot can go into the predicted cell
 */
void wall_based_movement(Maze maze, int row, int column, bool *backtrack, Robot *robot)
{
    Cell cell = maze.cells[row][column];
    int next_row = 0;
    int next_column = 0;
    bool visited = false;

    if (cell.walls.front && cell.walls.left && cell.walls.right)
    {
        (*backtrack) = 1; // backtrack enabled
        FA_BTSendString("Backtracking", 20);
        FA_Left(180);
        set_direction(robot, 3); // backtracking turn
    }
    else
    {
        if (!cell.walls.right) // if there are no walls on the right then turn righ
        {
            FA_BTSendString("Turning right\n", 20);
            next_row = row;
            next_column = column;
            cell_to_grid((robot->direction + 1) % 4, &next_row, &next_column); // calculate next cell as if robot turned right
            if (!maze.cells[next_row][next_column].visited || *backtrack)
            {
                FA_Right(90);
                set_direction(robot, 1); // right turn
            }
            else
            {
                FA_BTSendString("Cell to the right is visited\n", 30);
            }
        }
        else if (!cell.walls.left) // if no walls on the left then turn left
        {
            FA_BTSendString("Turning left\n", 20);
            next_row = row;
            next_column = column;
            cell_to_grid((robot->direction + 3) % 4, &next_row, &next_column); // calculate next cell as if robot turned left
            if (!maze.cells[next_row][next_column].visited || *backtrack)
            {
                FA_Left(90);
                set_direction(robot, 2); // left turn
            }
            else
            {
                FA_BTSendString("Cell to the left is visited\n", 30);
            }
        }
        else
        {
            next_row = row;
            next_column = column;
            cell_to_grid(robot->direction, &next_row, &next_column); // gets the unvisited next cell
            if (!maze.cells[next_row][next_column].visited || *backtrack)
            {
                FA_BTSendString("Maze is probably done\n", 30);
            }
            FA_BTSendString("Continuing forward to unvisited cell\n", 30);
        }
    }
}

/*
 * This function sets the current cell as an intersection based on the amount of empty spaces surrounding the cell. If there is
 * more than two then the cell is an intersection, used for stopping backtracking in traverse_maze()
 * @param Cell *cell, pointer to the current cell and updates the is_intersection variable with true if it is an intersection
 */
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

/*
 * Main function to traverse the maze.
 * @param *pause_start_time pointer to when the robot last paused
 * @param *maze pointer to the initialised maze in the main function, allows for maze cells to be changed
 * @param *backtrack backtrack flag, changes when the robot encounters a previously marked intersection
 * @param *robot passes pointer of robot to stop_when_line_hit()
 * @param *rows pointer to current row
 * @param *columns pointer to current row
 */
void traverse_maze(unsigned long *pause_start_time, Maze *maze, bool *backtrack, Robot *robot, int *rows, int *columns)
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

    if (stop_when_line_hit(pause_start_time, rows, columns, backtrack, robot)) // once robot has stopped for long enough = true
    {

        FA_BTSendString("row: ", 10);
        FA_BTSendNumber(*rows);
        FA_BTSendString("\n", 4);

        FA_BTSendString("column: ", 10);
        FA_BTSendNumber(*columns);
        FA_BTSendString("\n", 4);

        int front = FA_ReadIR(IR_FRONT);
        int left = FA_ReadIR(IR_LEFT);
        int right = FA_ReadIR(IR_RIGHT);
        int rear = FA_ReadIR(IR_REAR);

        set_walls(front, right, left, rear, &maze->cells[*rows][*columns].walls); // sets walls of cell

        set_intersection(&maze->cells[*rows][*columns]); // declares if cell is an intersection

        wall_based_movement(*maze, *rows, *columns, backtrack, robot); // updates the movement of the cell
    }
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
    // FA_LCDNumber(FA_ReadIR(IR_FRONT_LEFT), 30, 20, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_FRONT), 60, 20, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_FRONT_RIGHT), 90, 20, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_RIGHT), 30, 12, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_REAR_RIGHT), 90, 4, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_REAR), 60, 4, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_REAR_LEFT), 30, 4, FONT_NORMAL, LCD_OPAQUE);
    // FA_LCDNumber(FA_ReadIR(IR_LEFT), 90, 12, FONT_NORMAL, LCD_OPAQUE);

    FA_BTSendString("Front left", 20);
    FA_BTSendNumber(FA_ReadIR(IR_FRONT_LEFT));
    FA_BTSendString("\n", 5);

    FA_BTSendString("Front Right", 20);
    FA_BTSendNumber(FA_ReadIR(IR_FRONT_RIGHT));
    FA_BTSendString("\n", 5);

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

    int rows = 2; // adds an offset of 2 to the columns/rows because there are minus numbers which are bad.
    int columns = 2;

    bool backtrack = false;

    // int motor_l = MOTOR_SPEED_LEFT;
    // int motor_r = MOTOR_SPEED_RIGHT;

    while (1) // Execute this loop as long as robot is running
    {         // (this is equivalent to Arduino loop() function
        traverse_maze(&pause_start_time, &maze, &backtrack, &start_fix, &robot, &rows, &columns);
    }
    return 0; // Actually, we should never get here...
}