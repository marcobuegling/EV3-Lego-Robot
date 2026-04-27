#ifndef CLIENTCOMMUNICATION_H_
#define CLIENTCOMMUNICATION_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//Console colors
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

//Definition of Communication Codes
#define INIT_GREET 0 //COM_CODE of initial message sent by client
#define SND_POS 1 //COM_CODE for messages containing the current robot position;
#define REQ_JOB_PICKUP 2 //COM_CODE for messages requesting a new pickup job
#define REQ_JOB_DROPOFF 3 //COM_CODE for messages requesting a new dropoff job
#define SND_JOB_PICKUP 4 //COM_CODE for messages containing a new pickup job
#define SND_JOB_DROPOFF 5 //COM_CODE for messages containing a new dropoff job
#define REQ_INIT 6 //COM_CODE for messages requesting the initial position
#define SND_INIT 7 //COM_CODE for messages containing the initial position
#define RDY 8 //COM_CODE for messages signalling the readiness of the robot
#define START 9 //COM_CODE singalling the start of the game


/** @struct position
 * Struct for storing a position
 */
typedef struct position
{
    int8_t x;
    int8_t y;
    int8_t rotation;
}position;

/** @struct robot_data 
 * Data packtes send from your robot to the server
 */
struct robot_data{
    int8_t group_id; /*!< ID of your robot*/
    int8_t com_code; /*!< Communication code used for identifying purpose of the message*/
    int8_t coord_x; /*!< Current X-Coordinate of your robot*/
    int8_t coord_y; /*!< Current Y-Coordinate of your robot*/
    int8_t rotation; /*!< Current rotation of your robot*/
    position route[32]; /*!< Array containing the planned route of your robot. NOTE: Element 0 = Next position to drive to; Element N = last position of route; Element N + X = -1*/
};

/** @struct server_robot_data
 * Data packets send from the server to your robot.\n
 * Does not include a route
 */
struct server_robot_data{
    int8_t group_id; /*!< ID of your robot*/
    int8_t com_code; /*!< Communication code used for identifying purpose of the message*/
    int8_t coord_x; /*!< New target X-Coordinate */
    int8_t coord_y; /*!< New target Y-Coordinate */
    int8_t rotation; /*!< New target rotation */
    int8_t height; /*!< New target height */
};


/* Initializes communication between your robot and the server
 *   NOTE: Usage in private testing enviorment: 
 *       Create file named "ipconfig.txt" on same level as your main and write Server-IP to it
 *       By default the server is hosted on port 5000
*/
void init_communication(int group_number, unsigned short port);

/* Requests the initial postion of your robot from the server and returns a struct robot_data with new target coordinates
 *   NOTE: Blocking function (blocks till valid initialization is recieved)
 *      Sends request for init every 2 seconds
*/
struct server_robot_data request_init_pos();

/* Signals server your robot is ready and waits for START-Signal
 *   NOTE: Blocking function (blocks till START-Signal is recieved)
*/
void wait_for_start();

/* Sends the current position and route of the robot to the server
 *   NOTE: /
*/
void send_position(int x_coordinate, int y_coordinate, int rot, position route[]);

/* Request a new pickup job for your robot from the server and returns a struct server_robot_data with new target coordinates
 *   NOTE: /
*/
struct server_robot_data request_pickup_job();

/* Request a new dropoff job for your robot from the server and returns a struct server_robot_data with new target coordinates
 *   NOTE: /
*/
struct server_robot_data request_dropoff_job();

#endif

