#include "mazeMapper.h"
#include <stdbool.h>
#include <stdlib.h>

#define MOTOR_SPEED_LEFT 45
#define MOTOR_SPEED_RIGHT 40
#define OBSTACLE_SENSOR_THRESHOLD 200
#define LIGHT_SENSOR_THRESHOLD 400

void finished_maze() // plays an arpeggiated DMin7
{
    PlayNote(78, 125);
    PlayNote(87, 125);
    PlayNote(110, 125);
    PlayNote(130, 125);
    PlayNote(78 * 2, 125);
    PlayNote(87 * 2, 125);
    PlayNote(110 * 2, 125);
    PlayNote(130 * 2, 125);
    PlayNote(110 * 2, 125);
    PlayNote(87 * 2, 125);
    PlayNote(78 * 2, 125);
    PlayNote(130, 125);
    PlayNote(110, 125);
    PlayNote(87, 125);
    PlayNote(78, 125);

    Right(360);
}

/**
 * This function initialises the maze, by setting all values as 0 so they are initialises.
 */
void initialise_maze(Maze *maze)
{
    maze->food_x = -1;
    maze->food_y = -1;
    maze->shelter_x = -1;
    maze->shelter_x = -1;
    maze->water_x = -1;
    maze->water_y = -1;

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

/**
 * This function reads a large lines and returns if a line has been seen
 */
bool read_line()
{
    static unsigned long last_time = 0;
    int seen_line = false;
    unsigned long current_time = ClockMS();

    int left_line = ReadLine(0);
    int right_line = ReadLine(1);

    if ((left_line < 100 && right_line < 100) && (current_time - last_time > 200)) // reads a lines every 200ms
    {
        last_time = current_time;
        seen_line = true;
    }
    return seen_line;
}

/**
 * This function monitors the wheel encoders when they stop, this was a problem for the lower speeds for the robot
 * @param *last_left last time the left wheel encoder was read
 * @param *last_right last time the right wheel encoder was read
 * @param *time_last_checked, last time the wheel encoder was checked
 */
void monitor_wheel_encoders(int *last_left, int *last_right, unsigned long *time_last_checked)
{

    int left_encoder = ReadEncoder(0);
    int right_encoder = ReadEncoder(1);

    int left_movement = left_encoder - *last_left;
    int right_movement = right_encoder - *last_right;

    if (ClockMS() - *time_last_checked < 100)
    {
        *time_last_checked = ClockMS();
        if ((abs(left_movement) < 5 && abs(right_movement) > 10) || (abs(right_movement) < 5 && abs(left_movement) > 10))
        {
            Forwards(1); // nudges wheel a little bit
        }
    }

    *last_left = left_encoder;
    *last_right = right_encoder;
}

/**
 * This function adjusts the robot during the pause that the robot takes after it has seen a line
 * adjusts based on the sensor readings
 */
void adjust_for_wall()
{
    int front = ReadIR(IR_FRONT);
    int front_right = ReadIR(IR_FRONT_RIGHT);
    int front_left = ReadIR(IR_FRONT_LEFT);
    int right = ReadIR(IR_RIGHT);
    int left = ReadIR(IR_LEFT);

    const int threshold = 250;

    unsigned long start_time = ClockMS();
    while ((front < 500 || front_right < front_left + threshold || front_left < front_right + threshold || left < right + threshold / 2 || right < left + threshold / 2) && (ClockMS() - start_time < 500))
    {
        if (front_right + threshold < front_left)
        {
            Backwards(3);
            Right(10);
        }
        else if (front_left + threshold < front_right)
        {
            Backwards(3);
            Left(10);
        }
        else
        {
            break;
        }

        front = ReadIR(IR_FRONT);
        front_right = ReadIR(IR_FRONT_RIGHT);
        front_left = ReadIR(IR_FRONT_LEFT);
        left = ReadIR(IR_LEFT);
        right = ReadIR(IR_RIGHT);
    }

    if (front > 500) // until ir front isn't over 400 it reverses
    {
        Backwards(3);
    }
}

/**
 * This function is used to set the walls in the current cell
 * @param front, right, left, rear these are sensor readings that are passed in
 * @param *walls, this is the walls that are passed in to be changed depending on the sensor readings
 */
void set_walls(int front, int right, int left, int rear, Walls *walls)
{
    walls->front = (front > OBSTACLE_SENSOR_THRESHOLD / 5); // sets the front wall based on if front > obstacle threashold as it returns either true or false
    walls->right = (right > OBSTACLE_SENSOR_THRESHOLD / 5);
    walls->left = (left > OBSTACLE_SENSOR_THRESHOLD / 5);
    walls->rear = (rear > OBSTACLE_SENSOR_THRESHOLD / 5);

    if (walls->front == true)
    {
        BTSendString("X, ", 4);
    }
    else
    {
        BTSendString("F, ", 4);
    }
    if (walls->left == true)
    {
        BTSendString("X, ", 4);
    }
    else
    {
        BTSendString("L, ", 4);
    }
    if (walls->right == true)
    {
        BTSendString("X, ", 4);
    }
    else
    {
        BTSendString("R, ", 4);
    }
    if (walls->rear == true)
    {
        BTSendString("X, \n", 6);
    }
    else
    {
        BTSendString("B, \n", 6);
    }
}
/**
 * This function is used to get the next coordinates of the robot after a turn has been taken
 * @param direction, the robots direction that is passed in
 * @param *rows, the rows that are passed in and are changed when the direction is passed in
 * @param *columns, the columns that are passed in and are changed when the direction is passed in
 */
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

/**
 * Returns the direction the robot is currently facing in human readable terms
 * @param direction direction the robot is facing
 */
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

/**
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
        BTSendString("Robot is now facing ", 25);
        BTSendString(get_facing(robot->direction), 10);
        BTSendString("\n", 5);
        break;
    case 2:
        robot->direction = (robot->direction + 3) % 4; // left turn
        BTSendString("Robot is now facing ", 25);
        BTSendString(get_facing(robot->direction), 10);
        BTSendString("\n", 5);
        break;
    case 3:
        robot->direction = (robot->direction + 2) % 4; // turn around
        BTSendString("Robot is now facing ", 25);
        BTSendString(get_facing(robot->direction), 10);
        BTSendString("\n", 5);
        break;
    default:
        break;
    }
}

/**
 * This function makes the robot stops 500ms after a line has been hit to stop
 * in the middle of the cell, for the robot to see what the next moves are
 * @param *pause_start_time pointer to when the pause_starts
 * @param *rows pointer to rows passed in
 * @param *columns pointer to the columns passed in
 * @param *backtrack backtrack flag for when the robot needs to backtrack to get to unexplored cells
 * @param *robot robot passed in, gives access to direction of the robot
 * @param *number_of_seen_lines, dependent on how many additional lines are seen
 */
bool stop_when_line_hit(unsigned long *pause_start_time, int *rows, int *columns, bool *backtrack, Robot *robot, int *number_of_seen_lines)
{
    static int motors_started = 0;             // motors started flag
    static int stopping = 0;                   // stopping flag
    static unsigned long line_detect_time = 0; // ms passed since a line has been detected

    static int number_of_lines = 0;                    // increases for additional cells
    static bool big_line_detected = false;             // for when a big line is detected
    static bool another_line_detected = false;         // for when an additional line is detected i.e. for food & water
    static unsigned long another_line_detect_time = 0; // when that additional line has been detected

    static bool food_found = false;  // if food has been found
    static bool water_found = false; // if water has been found

    if (!motors_started && !stopping) // starts the motors at the beginning of the program, as after it needs to see a line to continue forward
    {
        if (ReadIR(IR_FRONT) > OBSTACLE_SENSOR_THRESHOLD / 4) // specific edge case where robot starts facing a wall
        {
            if (ReadIR(IR_REAR) < 30)
            {
                Right(180); // turns around
                set_direction(robot, 3);
            }
            else if (ReadIR(IR_LEFT) > 50)
            {
                BTSendString("Beginning right\n", 20);
                Right(90);
                set_direction(robot, 1);
            }
            else if (ReadIR(IR_RIGHT) > 50)
            {
                BTSendString("Beginning left\n", 20);
                Left(90);
                set_direction(robot, 2);
            }
        }
        SetMotors(MOTOR_SPEED_LEFT, MOTOR_SPEED_RIGHT); // motor then starts
        motors_started = 1;                             // started flag now positive
    }

    if (read_line() && !stopping && line_detect_time == 0 && !big_line_detected) // checks if a line is detected, robot isn't stopping and if the line hasn't been detected recently
    {
        cell_to_grid(robot->direction, rows, columns);
        big_line_detected = true;
        line_detect_time = ClockMS();
    }

    if (big_line_detected && line_detect_time != 0 && ClockMS() - line_detect_time > 200 && !stopping) // if after 250 ms since the big line has been seen
    {
        if (ReadLine(0) < 100 && ReadLine(1) < 100) // check line sensors
        {
            if (!another_line_detected)
            {
                another_line_detect_time = ClockMS();
                another_line_detected = true;
                number_of_lines++;
                if (number_of_lines == 2 && !food_found)
                {
                    food_found = true;
                }
                else if (number_of_lines == 3 && food_found)
                {
                    water_found = true;
                }
            }
        }
    }

    if (another_line_detected && (ClockMS() - another_line_detect_time > 100)) // if another line hasn't been detected in more than 125 ms reset
    {
        another_line_detected = false;
    }

    if (line_detect_time != 0 && ClockMS() - line_detect_time >= 450 && !stopping) // pauses the robot after a line has been detected
    {
        *pause_start_time = ClockMS(); // gets the time when the pause started
        SetMotors(0, 0);               // actually stops the robot
        motors_started = 0;
        stopping = 1; // puts the robot in a stopped state
        line_detect_time = 0;
    }

    if (stopping && ClockMS() - *pause_start_time < 1250) // whilst the robot has been stopped adjust itself
    {
        adjust_for_wall();
    }

    if (stopping && ClockMS() - *pause_start_time >= 1250) // checks if robot has been stopped for a long enough time i.e. 1250ms
    {
        *number_of_seen_lines = number_of_lines - 1; // updates the lines after the pause;
        stopping = 0;
        big_line_detected = false;
        number_of_lines = 0;
        another_line_detect_time = 0;
        another_line_detected = 0;
        return true; // finished stopping
    }
    return false;
}

/**
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

    if (cell.walls.front && cell.walls.left && cell.walls.right)
    {
        (*backtrack) = true; // backtrack enabled
        BTSendString("Backtracking", 20);
        Left(180);
        set_direction(robot, 3); // backtracking turn
    }
    else
    {
        if (!cell.walls.left) // if no walls on the left then turn left
        {
            BTSendString("Turning left\n", 20);
            next_row = row;
            next_column = column;
            cell_to_grid((robot->direction + 3) % 4, &next_row, &next_column); // calculate next cell as if robot turned left
            if (!maze.cells[next_row][next_column].visited || *backtrack)
            {
                Left(90);
                set_direction(robot, 2); // left turn
            }
            else
            {
                BTSendString("Cell to the left is visited\n", 30);
            }
        }
        else if (!cell.walls.right) // if there are no walls on the right then turn right
        {
            BTSendString("Turning right\n", 20);
            next_row = row;
            next_column = column;
            cell_to_grid((robot->direction + 1) % 4, &next_row, &next_column); // calculate next cell as if robot turned right
            if (!maze.cells[next_row][next_column].visited || *backtrack)
            {
                Right(90);
                set_direction(robot, 1); // right turn
            }
            else
            {
                BTSendString("Cell to the right is visited\n", 30);
            }
        }
    }
    ResetEncoders(); // resets encoders after a move
}

/**
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

/**
 * Main function to traverse the maze.
 * @param *pause_start_time pointer to when the robot last paused
 * @param *maze pointer to the initialised maze in the main function, allows for maze cells to be changed
 * @param *backtrack backtrack flag, changes when the robot encounters a previously marked intersection
 * @param *robot passes pointer of robot to stop_when_line_hit()
 * @param *rows pointer to current row
 * @param *columns pointer to current row
 * @param *num_of_cells pointer to the number of cells that is updated once a cell is traversed
 */
void traverse_maze(unsigned long *pause_start_time, Maze *maze, bool *backtrack, Robot *robot, int *rows, int *columns, int *num_of_cells)
{
    static int last_right; // last time the right encoder was reads value
    static int last_left;
    static unsigned long monitor_wheel_encoder_time = 0; // time since the encoders were read

    monitor_wheel_encoders(&last_left, &last_right, &monitor_wheel_encoder_time); // monitors the wheel encoders to make sure they aren't too far apart

    if (maze->cells[*rows][*columns].is_intersection) // if the cell is an intersection
    {
        if (*backtrack)
        {
            BTSendString("Back at an intersection again\n", 30);
            (*backtrack) = false; // backtracking is completed now that it is in an intersection again
        }
    }

    if (!maze->cells[*rows][*columns].visited) // if the cell isn't visited then do this
    {
        maze->cells[*rows][*columns].visited = 1;
        (*num_of_cells)++;
    }

    int number_of_seen_lines = 0;

    if (stop_when_line_hit(pause_start_time, rows, columns, backtrack, robot, &number_of_seen_lines)) // once robot has stopped for long enough = true
    {
        BTSendString("row: ", 10);
        BTSendNumber(*rows);
        BTSendString("\n", 4);

        BTSendString("column: ", 10);
        BTSendNumber(*columns);
        BTSendString("\n", 4);

        int front = ReadIR(IR_FRONT);
        int left = ReadIR(IR_LEFT);
        int right = ReadIR(IR_RIGHT);
        int rear = ReadIR(IR_REAR);

        set_walls(front, right, left, rear, &maze->cells[*rows][*columns].walls); // sets walls of cell

        set_intersection(&maze->cells[*rows][*columns]); // declares if cell is an intersection

        if (ReadLight() <= LIGHT_SENSOR_THRESHOLD && (maze->shelter_x == -1 && maze->shelter_y == -1)) // shelter is undiscovered
        {
            maze->shelter_x = *columns;
            maze->shelter_y = *rows;
        }

        switch (number_of_seen_lines) // depening on how many lines are seen
        {
        case 2:
            BTSendString("FOOD!\n", 10);
            Backwards(150);
            cell_to_grid((robot->direction + 2) % 4, rows, columns);
            PlayNote(440, 100);
            *backtrack = true;
            break;
        case 3:
            BTSendString("WATER!\n", 10);
            Backwards(150);
            cell_to_grid((robot->direction + 2) % 4, rows, columns);
            PlayNote(220, 100);
            *backtrack = true;
            break;
        }

        draw_cell(*maze, *columns, *rows); // draws cells in the maze
        if (*columns == maze->food_x && *rows == maze->food_y)
        {
            draw_special_cell(*maze, *columns, *rows, 0); // draws a food cell
        }
        else if (*columns == maze->water_x && *rows == maze->water_y)
        {
            draw_special_cell(*maze, *columns, *rows, 1); // draws a water cell
        }
        else if (*columns == maze->shelter_x && *rows == maze->shelter_y)
        {
            draw_special_cell(*maze, *columns, *rows, 2); // draws a shelter
        }

        wall_based_movement(*maze, *rows, *columns, backtrack, robot); // updates the movement of the cell
    }
}

int main(void)
{

    RobotInit();

    Robot robot;
    robot.direction = 0; // relative direction, which in this case is north

    LCDBacklight(50);  // Switch on backlight (half brightness)
    DelayMillis(2000); // Pause 2 secs

    unsigned long pause_start_time = 0;

    Maze maze;
    initialise_maze(&maze); // Initialises Maze

    int rows = 2; // adds an offset of 2 to the columns/rows because there are minus numbers which are bad.
    int columns = 2;
    int num_of_cells = 0;

    draw_maze_walls(); // draws the maze external walls

    bool backtrack = false;

    while (1)
    {
        if (num_of_cells == 25)
        {
            finished_maze();
            break;
        }

        traverse_maze(&pause_start_time, &maze, &backtrack, &robot, &rows, &columns, &num_of_cells);
    }
    return 0;
}