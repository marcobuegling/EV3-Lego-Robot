#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "clientcommunication.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000 )
#endif


typedef enum dmode_t {
    DRIVE_FORWARD,
    DRIVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
} dmode_t;

typedef enum direction_t {
    X_POS,
    Y_POS,
    X_NEG,
    Y_NEG,
} direction_t;

typedef enum color_t {
    NONE,
    RED,
    YELLOW,
    GREEN,
    BLUE,
    PINK,
} color_t;



const float TARGET_BRIGHTNESS = 600.0;
const int BASE_SPEED = 200;

uint8_t sn_motor_left;
uint8_t sn_motor_right;
uint8_t sn_color;
uint8_t sn_transport_motor;
uint8_t sn_ultrasonic_sensor;

int max_speed; //maximum speed of the motors

int active; //1 if the robot is active, 0 if not

int x_pos; //x-position of the robot
int y_pos; //y-position of the robot
direction_t direction; //direction in which the robot is driving

dmode_t driving_mode;
int stop_signal; //is set to 1 if the robot needs to stop because of an obstacle in front of it, 0 if not
int ignore_brightness; //is set to 1 if the robot should drive regardless of the brightness detected, 0 if not

int drive_across_flag; //is set to 1 if the robot should drive across the next crossing, 0 if not
int turn_left_flag; //is set to 1 if the robot should turn left on the next crossing, 0 if not
int turn_right_flag; //is set to 1 if the robot should turn right on the next crossing, 0 if not

int x_target; //x-position of the target (begin of branch leading to the target)
int y_target; //y-position of the target (begin of branch leading to the target)

struct server_robot_data res;
struct robot_data robot;
struct server_robot_data pickup;
struct server_robot_data dropoff;
int send_position_flag;

int shelf_reached; //is set to 1 if the robot is close enough to the shelf, 0 if not
int keg_loaded; //is set to 1 if the robot has loaded a keg, 0 if not
int keg_action_flag; //is set tp 1 if the robot should lift or place a keg, 0 if not

float cal_factor_r, cal_factor_g, cal_factor_b; //factors for color calibration
float col_r, col_g, col_b, brightness; //color and brightness values
color_t prev_color; //last detected color
int distance; //distance to object in front of the robot (measured by ultrasonic sensor)

//Variables for PID controller
float kp = 0.25;
float ki = 0.05;
float kd = 0.15;
float errors[20];

//Threads
pthread_t driving;
pthread_t color_sensor;
pthread_t driving_control;
pthread_t ultrasonic_sensor;



//Initalise the motors and store sn
void initMotors() {
    while (ev3_tacho_init() < 1) Sleep(1000);
    if (ev3_search_tacho(LEGO_EV3_L_MOTOR, &sn_motor_left, 0)) {
        get_tacho_max_speed(sn_motor_left, &max_speed);
        printf("Found left L-mMotor. Motor's sn: %d\n", sn_motor_left);
    } else {
        printf("Left motor not found.\n");
        exit(1);
    }
    if (ev3_search_tacho(LEGO_EV3_L_MOTOR, &sn_motor_right, 1)) {
        get_tacho_max_speed(sn_motor_right, &max_speed);
        printf("Found right L-motor. Motor's sn: %d\n", sn_motor_right);
    } else {
        printf("Right motor not found.\n");
        exit(1);
    }
}

//Initialise color sensor
void initColorSensor() {
    while (ev3_sensor_init() < 1) Sleep(1000);
    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
        printf("Found color sensor. Sensor's sn: %d\n", sn_color);
		set_sensor_mode(sn_color, "RGB-RAW");
    } else {
        printf("Color sensor not found!\n");
        exit(1);
    }
}

//Initialise motor for barrel transport
void initTransportMotor() {
    if (ev3_search_tacho(LEGO_EV3_M_MOTOR, &sn_transport_motor, 0)) {
        printf("Found M-motor. Motor's sn: %d\n", sn_transport_motor);
    } else {
        printf("M-motor not found.\n");
        exit(1);
    }
}

//Initialise ultrasonic sensor
void initUltrasonicSensor() {
    if (ev3_search_sensor(LEGO_EV3_US, &sn_ultrasonic_sensor, 0)) {
        printf("Found ultrasonic sensor. Sensor's sn: %d\n", sn_ultrasonic_sensor);
		set_sensor_mode(sn_ultrasonic_sensor, "US-DIST-CM");
    } else {
        printf("Ultrasonic sensor not found!\n");
        exit(1);
    }
}

//Initialise the server
struct server_robot_data initial_server(){
    init_communication(2, 50000);
    struct server_robot_data res2 = request_init_pos();
    wait_for_start();
    return res2;
}


//Calculate and set factors for calculating 'real' color value by reading color twenty times per color, calculating average and then dividing 1000 by average
void calibrateColorSensor() {
    printf("Put the robots light sensor over white ground. Calibrating in 3 seconds...\n");
    Sleep(3000);
    int raw_values[20];
    float avg_r = 0;
    float avg_g = 0;
    float avg_b = 0;

    if (get_sensor_value(0, sn_color, &raw_values[0])) {
        for (int i = 1; i < 20; i++) {
            get_sensor_value(0, sn_color, &raw_values[i]);
        }
        for (int i = 0; i < 20; i++) {
            avg_r += raw_values[i];
        }
        avg_r = avg_r / 20.0;
        cal_factor_r = 1000 / avg_r;
    } else {
        printf("Couldn't read color.\n");
        exit(1);
    }

    if (get_sensor_value(1, sn_color, &raw_values[0])) {
        for (int i = 1; i < 20; i++) {
            get_sensor_value(1, sn_color, &raw_values[i]);
        }
        for (int i = 0; i < 20; i++) {
            avg_g += raw_values[i];
        }
        avg_g = avg_g / 20.0;
        cal_factor_g = 1000 / avg_g;
    } else {
        printf("Couldn't read color.\n");
        exit(1);
    }
    
    if (get_sensor_value(2, sn_color, &raw_values[0])) {
        for (int i = 1; i < 20; i++) {
            get_sensor_value(2, sn_color, &raw_values[i]);
        }
        for (int i = 0; i < 20; i++) {
            avg_b += raw_values[i];
        }
        avg_b = avg_b / 20.0;
        cal_factor_b = 1000 / avg_b;
    } else {
        printf("Couldn't read color.\n");
        exit(1);
    }

    //printf("Read color values for calibration: %f %f %f                         \n", avg_r, avg_g, avg_b);
    //printf("Factors: %f  %f %f\n", cal_factor_r, cal_factor_g, cal_factor_b);
    printf("Color sensor calibrated.\n");
}


/////Route calculation//////

//Switch direction to opposite
direction_t switchDirection(direction_t dir) {
    switch(dir) {
        case X_POS:
            return X_NEG;
        case X_NEG:
            return X_POS;
        case Y_POS:
            return Y_NEG;
        case Y_NEG:
            return Y_POS;
    }
}

//Check if we are currently waiting to exit a branch.
bool leavingBranch(int x, int y, direction_t dir){
    if ((x == 1) && (y == 0) && (dir == Y_NEG)) return true;
    else if ((x == 7) && (y == 0) && (dir == Y_NEG)) return true;
    else if ((x == 2) && (y == 3) && (dir == X_NEG)) return true;
    else if ((x == 2) && (y == 5) && (dir == X_NEG)) return true;
    else if ((x == 6) && (y == 3) && (dir == X_POS)) return true;
    else if ((x == 6) && (y == 5) && (dir == X_POS)) return true;
    else if ((x == 1) && (y == 8) && (dir == Y_POS)) return true;
    else if ((x == 7) && (y == 8) && (dir == Y_POS)) return true;
    else return false;
}


// Finds the best direction for leaving a branch
dmode_t getExitDirection2(int curX, int curY, direction_t curDir, int goalX, int goalY){
    // If facing north
    if (curDir == Y_POS){
        // Check left is better than right
        if (abs(goalX-(curX-1)) < abs(goalX-(curX+1))) return TURN_LEFT;
        else return TURN_RIGHT;
    }
    // If facing south
    else if (curDir == Y_NEG){
        // Check left is better than right
        if (abs(goalX-(curX+1)) < abs(goalX-(curX-1))) return TURN_LEFT;
        else return TURN_RIGHT;
    }
    // If facing east
    else if (curDir == X_POS){
        // Check left is better than right
        if (abs(goalY-(curY+1)) < abs(goalY-(curY-1))) return TURN_LEFT;
        else return TURN_RIGHT;
    }
    // If facing west
    else if (curDir == X_NEG){
        // Check left is better than right
        if (abs(goalY-(curY-1)) < abs(goalY-(curY+1))) return TURN_LEFT;
        else return TURN_RIGHT;
    }
}


// find the best next direction
dmode_t bestDir2(int curX, int curY, direction_t curDir, int goalX, int goalY){
    // If facing north
    if (curDir == Y_POS){
        //Check if driving forward is good
        if (abs(goalY-(curY+1)) < abs(goalY-curY)) return DRIVE_FORWARD;       
        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalX-(curX-1)) < abs(goalX-(curX+1))) return TURN_LEFT;
            else return TURN_RIGHT;
        } else {
            //printf("Drive Forward: %i , %i", curX, curY);
            return DRIVE_FORWARD;
        }
    }

    // If facing south
    else if (curDir == Y_NEG){
        //Check if driving forward is good
        if (abs(goalY-(curY-1)) < abs(goalY-curY)) return DRIVE_FORWARD;
       // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalX-(curX+1)) < abs(goalX-(curX-1))) return TURN_LEFT;
            else return TURN_RIGHT;
        } else {
            return DRIVE_FORWARD;
        }
    }

    // If facing east
    else if (curDir == X_POS){
        //Check if driving forward is good
        if (abs(goalX-(curX+1)) < abs(goalX-curX)) return DRIVE_FORWARD;
        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalY-(curY+1)) < abs(goalY-(curY-1))) return TURN_LEFT;
            else return TURN_RIGHT;
        } else {
            return DRIVE_FORWARD;
        }
    }

    // If facing west
    else if (curDir == X_NEG){
        //Check if driving forward is good
        if (abs(goalX-(curX-1)) < abs(goalX-curX)) return DRIVE_FORWARD;
        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalY-(curY-1)) < abs(goalY-(curY+1))) return TURN_LEFT;
            else return TURN_RIGHT;
        } else {
            return DRIVE_FORWARD;
        }
    }
}


// Get entry turning direction for branch
dmode_t getEntryDirection2(int x, int y, direction_t dir){
    if ( ((x == 1) && (y == 0)) || ((x == 7) && (y == 0)) ){
        if (dir == X_POS) return TURN_LEFT;
        else return TURN_RIGHT;
    } 
    else if ( ((x == 2) && (y == 3)) || ((x == 2) && (y == 5)) ){
        if (dir == Y_POS) return TURN_RIGHT;
        else return TURN_LEFT;
    }
    else if ( ((x == 6) && (y == 5)) || ((x == 6) && (y == 3)) ){
        if (dir == Y_POS) return TURN_LEFT;
        else return TURN_RIGHT;
    } 
    else if ( ((x == 1) && (y == 8)) || ((x == 7) && (y == 8)) ){
        if (dir == X_POS) return TURN_RIGHT;
        else return TURN_LEFT;
    }    
}


//Find next direction from current location to goal
dmode_t findWay2(int curX, int curY, direction_t curDir, int goalX, int goalY){
    // Check if we have reached the goal
     if ((curX == goalX) && (curY == goalY)){
         return STOP;
     }
    // Lets check if we are leaving a branch
    bool leaveBranch = leavingBranch(curX, curY, curDir);
    // If we are leaving a branch, let's exit in a direction towards the goal
    if (leaveBranch){
        return getExitDirection2(curX, curY, curDir, goalX, goalY);
    }
    //Lets get the best direction
    return bestDir2(curX, curY, curDir, goalX, goalY);
 }


int8_t direction_to_rotation(direction_t dir){
    if (dir == X_NEG){
        return 0;
    } else if (dir == X_POS){
         return 2;
    } else if (dir == Y_NEG){
         return 3;
    } else if (dir == Y_POS){
         return 1;
    }
}

void generateRoute(int curX, int curY, direction_t curDir, int goalX, int goalY, position route[]){
    //Initialize the array for the route
    for(int i = 0; i < 32; i++) {
        robot.route[i].x = -1;
        robot.route[i].y = -1;
        robot.route[i].rotation = -1;
    }
    
    for (int i = 0; i < 32; i++){  
        // Fill in array of position structs
        route[i].y = curY;
        route[i].x = curX;
        route[i].rotation = direction_to_rotation(curDir);   
        //Get next position
        dmode_t dir = findWay2(curX, curY, curDir, goalX,  goalY);
        //Change current position and direction
        if (dir == DRIVE_FORWARD){
            // Direction stays the same
            if (curDir == X_POS) curX++;
            else if (curDir == X_NEG) curX--;
            else if (curDir == Y_POS) curY++;
            else if (curDir == Y_NEG) curY--;
        } 
        else if (dir == TURN_LEFT){
            if (curDir == X_POS){
                 curY++;
                 curDir = Y_POS;
            }
            else if (curDir == X_NEG){
                curY--;
                curDir = Y_NEG;
            } 
            else if (curDir == Y_POS){
                curX--;
                curDir = X_NEG;
            } 
            else if (curDir == Y_NEG){
                curX++;
                curDir = X_POS;
            } 
        }
        else if (dir == TURN_RIGHT){
            if (curDir == X_POS){
                 curY--;
                 curDir = Y_NEG;
            }
            else if (curDir == X_NEG){
                curY++;
                curDir = Y_POS;
            } 
            else if (curDir == Y_POS){
                curX++;
                curDir = X_POS;
            } 
            else if (curDir == Y_NEG){
                curX--;
                curDir = X_NEG;
            } 
        }
        else if (dir == STOP){
            return;
        }
    }
}


//Changes direction of the robot
void turnedLeft() {
    if (direction == Y_NEG) {
        direction = X_POS;
    } else {
        direction++;
    }
    printf("Turned left! Now facing: %d\n", direction);
}

//Changes direction of the robot
void turnedRight() {
    if (direction == X_POS) {
        direction = Y_NEG;
    } else {
        direction--;
    }
    printf("Turned right! Now facing: %d\n", direction);
}

void turnedAround() {
    if (direction == X_POS) direction = X_NEG;
    else if (direction == X_NEG) direction = X_POS;
    else if (direction == Y_POS) direction = Y_NEG;
    else if (direction == Y_NEG) direction = Y_POS;
    printf("Turned around! Now facing %d\n", direction);
}



int reachedTarget() {
    if ((x_pos == x_target) && (y_pos == y_target)) return 1; else return 0;
}

void getEntryDirection() {
    if ( ((x_pos == 1) && (y_pos == 0)) || ((x_pos == 7) && (y_pos == 0)) ){
        if (direction == X_POS) turn_left_flag = 1;
        else turn_right_flag = 1;
    } 

    else if ( ((x_pos == 2) && (y_pos == 3)) || ((x_pos == 2) && (y_pos == 5)) ){
        if (direction == Y_POS) turn_right_flag = 1;
        else turn_left_flag = 1;
    }

    else if ( ((x_pos == 6) && (y_pos == 5)) || ((x_pos == 6) && (y_pos == 3)) ){
        if (direction == Y_POS) turn_left_flag = 1;
        else turn_right_flag = 1;
    } 

    if ( ((x_pos == 1) && (y_pos == 8)) || ((x_pos == 7) && (y_pos == 8)) ){
        if (direction == X_POS) turn_right_flag = 1;
        else turn_left_flag = 1;
    }
}

//changed right flags to left flags and POS to NEG and NEG to POS
void getExitDirection() {
    // If facing north
    if (direction == Y_NEG){
        // Check left is better than right
        if (abs(x_target-(x_pos-1)) < abs(x_target-(x_pos-1))) turn_right_flag = 1;
        else turn_left_flag = 1;
    }

    // If facing south
    else if (direction == Y_POS){
        // Check left is better than right
        if (abs(x_target-(x_pos+1)) < abs(x_target-(x_pos-1))) turn_right_flag = 1;
        else turn_left_flag = 1;
    }

    // If facing east
    else if (direction == X_NEG){
        // Check left is better than right
        if (abs(y_target-(y_pos+1)) < abs(y_target-(y_pos-1))) turn_right_flag = 1;
        else turn_left_flag = 1;
    }

    // If facing west
    else if (direction == X_POS){
        // Check left is better than right
        if (abs(y_target-(y_pos-1)) < abs(y_target-(y_pos+1))) turn_right_flag = 1;
        else turn_left_flag = 1;
    }
}



direction_t rotationToDirection() {
    if (robot.rotation == 0) {
        return X_NEG;
    } else if (robot.rotation == 2) {
        return X_POS;
    } else if (robot.rotation == 3) {
        return Y_NEG;
    } else if (robot.rotation == 1) {
        return Y_POS;
    }
}

int8_t directionToRotation(){
    if (direction == X_NEG){
        return 0;
    }
    else if (direction == X_POS){
         return 2;
    }
    else if (direction == Y_NEG){
         return 3;
    }
    else if (direction == Y_POS){
         return 1;
    }
}

void sendPosition() {
    robot.coord_x = x_pos;
    robot.coord_y = y_pos;
    robot.rotation = directionToRotation();
    generateRoute(robot.coord_x, robot.coord_y, direction, x_target,  y_target, robot.route);
    send_position(x_pos, y_pos, robot.rotation, robot.route);
    send_position_flag = 0;
}

//Control speed of the motors
void runMotors(int l_motor_speed, int r_motor_speed) {
    set_tacho_speed_sp(sn_motor_left, l_motor_speed);
    set_tacho_speed_sp(sn_motor_right, r_motor_speed);
	set_tacho_command_inx(sn_motor_left, TACHO_RUN_FOREVER);
	set_tacho_command_inx(sn_motor_right, TACHO_RUN_FOREVER);
}

//Save error in errors array
void saveError(float error) {
    int array_length = sizeof(errors) / sizeof(errors[0]);
    for (int i = array_length - 1; i > 0; i--) {
        errors[i] = errors[i - 1];
    }
    errors[0] = error;
}

//Calc proportional term for pid controller
float calcProportionalTerm() {
    return kp * errors[0];
}

//Calc integral term for pid controller
float calcIntegralTerm() {
    float errorSum = 0;
    int array_length = sizeof(errors) / sizeof(errors[0]);
    for (int i = 0; i < array_length; i++) {
        errorSum += errors[i];
    }
    errorSum = errorSum / array_length;
    return ki * errorSum;
}

//Calc derivative term for pid controller
float calcDerivativeTerm() {
    return kd * (errors[0] - errors[1]);
}

//Calculate the speed of the motors that is needed to stay on the edge of the line
int calcMotorSpeedChange() {
    float error = (-1) * (TARGET_BRIGHTNESS - brightness);
    saveError(error);
    float u = calcProportionalTerm() + calcIntegralTerm() + calcDerivativeTerm();
    return (int) u;
}

//Function for driving thread
void* drive() {
    int speed_change;
    while (active) {
        switch (driving_mode) {
            case DRIVE_FORWARD:
                if (stop_signal) {
                    runMotors(0, 0);
                } else {
                    //printf("r: %f, g: %f, b: %f, brightness: %f\n", col_r, col_g, col_b, brightness);
                    if (ignore_brightness) {
                        speed_change = 0;
                    } else {
                        speed_change = calcMotorSpeedChange();
                    }
                    runMotors(BASE_SPEED + speed_change, BASE_SPEED - speed_change);
                }
                break;
            case DRIVE_BACKWARD:
                runMotors(-BASE_SPEED, -BASE_SPEED);
                break;
            case TURN_LEFT:
                runMotors(-BASE_SPEED, BASE_SPEED);
                break;
            case TURN_RIGHT:
                runMotors(BASE_SPEED, -BASE_SPEED);
                break;
            case STOP:
                runMotors(0, 0);
                break;
        }
        Sleep(10);
    }
    runMotors(0, 0);
}


//Reads raw rgb values, calculates 'real' rgb values and calculates the brightness by calculating the average of the rgb values
void readColorSensor() {
    int raw_r, raw_g, raw_b;
    if (get_sensor_value(0, sn_color, &raw_r)) {
        col_r = raw_r * cal_factor_r;
    }
    if (get_sensor_value(1, sn_color, &raw_g)) {
        col_g = raw_g * cal_factor_g;
    }
    if (get_sensor_value(2, sn_color, &raw_b)) {
        col_b = raw_b * cal_factor_b;
    }
    brightness = (col_r + col_g + col_b) / 3;
}

//Function for brightness color detection thread: reads and saves sensor values by calling function readColorSensor and then checks if a color has been recognised 
void* colorDetection() {
    int i = 0; //counts how many times in a row the rgb values have fulfilled the requirements for a certain color
    int recognized_color_recently = 0; //1 if a color has been recognized recently, 0 if not
    color_t detected_color; //last detected color
    ignore_brightness = 0;
    while (active) {
        readColorSensor();
        //color detection is only done if the brightness level is higher than 150
        if (brightness > 150 && brightness < 800) {
            //check requirements for red
            if ((col_r > col_g * 2) && (col_r > col_b * 2)) {
                if (detected_color == RED) {
                    i++;
                } else {
                    i = 0;
                }
                //ignore_brightness = 1;
                detected_color = RED;
            //check requirements for yellow
            } else if ((col_r > col_b * 2.5) && (col_g > col_b * 2.5) && (col_r * 1.5 > col_g)) {
                if (detected_color == YELLOW) {
                    i++;
                } else {
                    i = 0;
                }
                //ignore_brightness = 1;
                detected_color = YELLOW;
            //check requirements for green
            } else if ((col_g > 1.5 * col_b) && (col_g > col_r)) {
                if (detected_color == GREEN) {
                    i++;
                } else {
                    i = 0;
                }
                //ignore_brightness = 1;
                detected_color = GREEN;
            //check requirements for blue
            } else if ((col_b > 1.5 * col_r) && (col_b > col_g)) {
                if (detected_color == BLUE) {
                    i++;
                } else {
                    i = 0;
                }
                //ignore_brightness = 1;
                detected_color = BLUE;
            //check requirements for pink
            } else if ((col_r > 1.25 * col_g) && (col_r > col_b) && (col_b > col_g)) {
                if (detected_color == PINK) {
                    i++;
                } else {
                    i = 0;
                }
                //ignore_brightness = 1;
                detected_color = PINK;
            } else {
                recognized_color_recently = 0;
                i = 0;
                //ignore_brightness = 0;
                detected_color = NONE;
            }
            //the rgb values must have met the requirements for a certain color 3 times in a row, before the event for the color is called 
            if ((i > 2) && (recognized_color_recently == 0)) {
                recognized_color_recently = 1;
                switch (detected_color) {
                    case RED:
                        printf("Detected red! %f %f %f %d %d %d\n", col_r, col_g, col_b, x_pos, y_pos, direction);
                        switch(direction) {
                            case X_POS:
                                if (!keg_action_flag) {
                                    x_pos++;
                                    sendPosition();
                                    if (x_pos >= x_target - 1) {
                                        if (y_pos < y_target) {
                                            turn_left_flag = 1;
                                        } else if (y_pos > y_target) {
                                            turn_right_flag = 1;
                                        } else {
                                            drive_across_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case Y_POS:
                                break;
                            case X_NEG:
                                break;
                            case Y_NEG:
                                shelf_reached = 1;
                                break;
                        }
                        prev_color = RED;
                        break;
                    case YELLOW:
                        printf("Detected yellow! %f %f %f %d %d %d\n", col_r, col_g, col_b, x_pos, y_pos, direction);
                        switch(direction) {
                            case X_POS:
                                break;
                            case Y_POS:
                                shelf_reached = 1;
                                break;
                            case X_NEG:
                                if (!keg_action_flag) {
                                    x_pos--;
                                    sendPosition();
                                    if (x_pos <= x_target + 1) {
                                        if (y_pos < y_target) {
                                            turn_right_flag = 1;
                                        } else if (y_pos > y_target) {
                                            turn_left_flag = 1;
                                        } else {
                                            drive_across_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case Y_NEG:
                                break;
                        }
                        prev_color = YELLOW;
                        break;
                    case GREEN:
                        printf("Detected green! %f %f %f %d %d %d\n", col_r, col_g, col_b, x_pos, y_pos, direction);
                        switch(direction) {
                            case X_POS:
                                shelf_reached = 1;
                                break;
                            case Y_POS:
                                break;
                            case X_NEG:
                                break;
                            case Y_NEG:
                                if (!keg_action_flag) {
                                    y_pos--;
                                    sendPosition();
                                    if (y_pos <= y_target + 1) {
                                        if (x_pos < x_target) {
                                            turn_left_flag = 1;
                                        } else if (x_pos > x_target) {
                                            turn_right_flag = 1;
                                        } else {
                                            drive_across_flag = 1;
                                        }
                                    }
                                }
                                break;
                        }
                        prev_color = GREEN;
                        break;
                    case BLUE:
                        printf("Detected blue! %f %f %f %d %d %d\n", col_r, col_g, col_b, x_pos, y_pos, direction);
                        switch(direction) {
                            case X_POS:
                                break;
                            case Y_POS:
                                if (!keg_action_flag) {
                                    y_pos++;
                                    sendPosition();
                                    if (y_pos >= y_target - 1) {
                                        if (x_pos < x_target) {
                                            turn_right_flag = 1;
                                        } else if (x_pos > x_target) {
                                            turn_left_flag = 1;
                                        } else {
                                            drive_across_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case X_NEG:
                                shelf_reached = 1;
                                break;
                            case Y_NEG:
                                break;
                        }
                        prev_color = BLUE;
                        break;
                    case PINK:
                        printf("Detected pink! %f %f %f %d %d %d\n", col_r, col_g, col_b, x_pos, y_pos, direction);
                        switch(direction) {
                            case X_POS:
                                if (!keg_action_flag) {
                                    if (!(x_pos % 2)) {
                                        x_pos++;
                                        sendPosition();
                                        if (reachedTarget()) {
                                            getEntryDirection();
                                            Sleep(200);
                                            keg_action_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case Y_POS:
                                if (!keg_action_flag) {
                                    if (!(y_pos % 2)) {
                                        y_pos++;
                                        sendPosition();
                                        if (reachedTarget()) {
                                            getEntryDirection();
                                            Sleep(200);
                                            keg_action_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case X_NEG:
                                if (!keg_action_flag) {
                                    if (!(x_pos % 2)) {
                                        x_pos--;
                                        sendPosition();
                                        if (reachedTarget()) {
                                            getEntryDirection();
                                            Sleep(200);
                                            keg_action_flag = 1;
                                        }
                                    }
                                }
                                break;
                            case Y_NEG:
                                if (!keg_action_flag) {
                                    if (!(y_pos % 2)) {
                                        y_pos--;
                                        sendPosition();
                                        if (reachedTarget()) {
                                            getEntryDirection();
                                            Sleep(200);
                                            keg_action_flag = 1;
                                        }
                                    }
                                }
                                break;
                        }
                        prev_color = PINK;
                        break;
                }
                //printf("r: %f, g: %f, b: %f, brightness: %f\n", col_r, col_g, col_b, brightness);
            }
        }
    }
}


//Read and store value of ultrasonic sensor (saves 5 values and calculates the average)
void readUltrasonicSensor() {
    get_sensor_value(0, sn_ultrasonic_sensor, &distance);
}

//Function for distance thread: Sets stop_signal if needed
void* readDistance() {
    while (active) {
        //printf("\rDistance: %d  ", distance);
        if (!keg_action_flag) {
            readUltrasonicSensor();
            if (distance < 155) {
                stop_signal = 1;
            } else {
                stop_signal = 0;
            }
        }
    }
}


void liftKeg() {
    set_tacho_speed_sp(sn_transport_motor, -120);
    set_tacho_position_sp(sn_transport_motor, -420);
    set_tacho_command_inx(sn_transport_motor, TACHO_RUN_TO_REL_POS);
    Sleep(2500);
    keg_loaded = 1;
}

void placeKeg() {
    set_tacho_speed_sp(sn_transport_motor, 120);
    set_tacho_position_sp(sn_transport_motor, 420);
    set_tacho_command_inx(sn_transport_motor, TACHO_RUN_TO_REL_POS);
    Sleep(2500);
    keg_loaded = 0;
}


void updateTarget() {
    if (!keg_loaded) {
        x_target = pickup.coord_x;
        y_target = pickup.coord_y;
    } else if (keg_loaded) {
        x_target = dropoff.coord_x;
        y_target = dropoff.coord_y;
    }
    printf("Target at: %d %d\n", x_target, y_target);
}

void* driveControl() {
    while (active) {
        driving_mode = DRIVE_FORWARD;
        if (drive_across_flag) {
            Sleep(1000);
            ignore_brightness = 1;
            Sleep(250);
            ignore_brightness = 0;
            drive_across_flag = 0;
        } else if (turn_left_flag) {
            Sleep(1000);
            driving_mode = TURN_LEFT;
            turnedLeft();
            Sleep(920);
            sendPosition();
            turn_left_flag = 0;
        } else if (turn_right_flag) {
            Sleep(1000);
            driving_mode = TURN_RIGHT;
            turnedRight();
            Sleep(950);
            sendPosition();
            turn_right_flag = 0;
        } else if (keg_action_flag) {
            while (!shelf_reached) {
                driving_mode = DRIVE_FORWARD;
            }
            driving_mode = STOP;
            if (keg_loaded) {
                placeKeg();
                pickup = request_dropoff_job();
                updateTarget();
                printf("Pickup job: %d %d\n", pickup.coord_x, pickup.coord_y);
            } else {
                liftKeg();
                dropoff = request_dropoff_job();
                updateTarget();
                printf("Dropoff job: %d %d\n", dropoff.coord_x, dropoff.coord_y);
            }

            driving_mode = DRIVE_BACKWARD;
            Sleep(1600);
            getExitDirection();
            if (turn_left_flag) {
                driving_mode = TURN_LEFT;
                turnedLeft();
                sendPosition();
                printf("Turned left! Now facing: %d\n", direction);
                Sleep(920);
                turn_left_flag = 0;
            } else {
                driving_mode = TURN_RIGHT;
                turnedRight();
                sendPosition();
                printf("Turned right! Now facing: %d\n", direction);
                Sleep(800);
                turn_right_flag = 0;
            }
            shelf_reached = 0;
            keg_action_flag = 0;
            printf("Keg action flag unset!\n");
        }
    }
    driving_mode = STOP;
}



int main(void) {
    //Init the ev3
    if (ev3_init() < 1) return 1;
    printf("Hello!\n");
    
    int array_length = sizeof(errors) / sizeof(errors[0]);
    for (int i = 0; i < array_length; i++) {
        errors[i] = errors[0];
    }

    //Initialise (and test) motors and sensors
    while (ev3_tacho_init() < 1) Sleep(1000);
    while (ev3_sensor_init() < 1) Sleep(1000);
	initMotors();
    initColorSensor();
    initTransportMotor();
    initUltrasonicSensor();
    calibrateColorSensor();

    //Initialize server and return first position
    res = initial_server();
    robot.group_id = res.group_id;
    robot.coord_x = res.coord_x;
    robot.coord_y = res.coord_y;
    robot.rotation = res.rotation;
    send_position_flag;

    
    //Initialise position
    x_pos = robot.coord_x;
    y_pos = robot.coord_y;
    direction = rotationToDirection();
  
    pickup = request_pickup_job();
    printf("Pickup job: %d %d\n", pickup.coord_x, pickup.coord_y);
    sendPosition();
    
    printf("ev3 initialised. Put the device on the left edge of a black line. Starting position: %d %d %d. Starting in 3 seconds...\n", robot.coord_x, robot.coord_y, rotationToDirection());
    Sleep(3000);

    keg_loaded = 0;
    shelf_reached = 0;
    active = 1;
    updateTarget();
    driving_mode = STOP;
    placeKeg();
    
    pthread_create(&ultrasonic_sensor, NULL, readDistance, NULL);
    pthread_create(&driving, NULL, drive, NULL);
    pthread_create(&color_sensor, NULL, colorDetection, NULL);
    pthread_create(&driving_control, NULL, driveControl, NULL);
    Sleep(70000);
    active = 0;
    pthread_join(driving_control, NULL);
    pthread_join(color_sensor, NULL);
    pthread_join(driving, NULL);
    pthread_join(ultrasonic_sensor, NULL);
    

    //Uninit the ev3
    ev3_uninit();
    printf("Goodbye!\n");
    return 0;
}
