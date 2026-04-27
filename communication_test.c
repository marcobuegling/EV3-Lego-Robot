#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "clientcommunication.h"
#include "path.c"

struct server_robot_data initial_server(){
init_communication(2, 50000);
struct server_robot_data res= request_init_pos();
wait_for_start();
return res;
}

direction_t rotation_to_direction(int rotation){
   if (rotation == 0){
        return X_NEG;
    }
    else if (rotation == 2){
         return X_POS;
    }
    else if (rotation == 3){
         return Y_NEG;
    }
    else if (rotation == 1){
         return Y_POS;
    }
}

// Server Test
int main(){
    
    // initialize server
    // and give first position
    struct server_robot_data res = initial_server();
    struct robot_data robot;
    robot.coord_x = res.coord_x;
    robot.coord_y = res.coord_y;
    robot.rotation = res.rotation;

    // initialize the 
    for(int i=0;i<32;i++)
    {
        robot.route[i].x = -1;
        robot.route[i].y = -1;
        robot.route[i].rotation = -1;
    }

    struct server_robot_data pickup = request_pickup_job();
    
    generateRoute(robot.coord_x, robot.coord_y, rotation_to_direction(robot.rotation), pickup.coord_x,  pickup.coord_y, robot.route);


    printf("PICKUP: %d, %d \n", pickup.coord_x, pickup.coord_y); 

    int i =0;
    // Now robot has the way
    // THIS SHOULD BE CHANGED!
    while(robot.route[i+1].x!= -1 && robot.route[i+1].y!= -1)
    {
        // move robot 
        // should update robot.coord_x and robot.coord_y and rotation
        send_position(robot.route[i].x,robot.route[i].y,robot.route[i].rotation, robot.route);
        sleep(2);
        i++;

    }

    // ROBOT CATCH
    //Robot coordination should be changed
    robot.coord_x = robot.route[i].x;
    robot.coord_y = robot.route[i].y;
    robot.rotation = robot.route[i].rotation;
   
    for(int i=0;i<32;i++)
    {
        robot.route[i].x = -1;
        robot.route[i].y = -1;
        robot.route[i].rotation = -1;
    }
    struct server_robot_data dropoff = request_dropoff_job();
    // print the dropoff position

    printf("DROPOFF: %d, %d \n", dropoff.coord_x, dropoff.coord_y); 
    i =0;
    generateRoute(robot.coord_x, robot.coord_y, rotation_to_direction(robot.rotation), dropoff.coord_x,  dropoff.coord_y, robot.route);
    while(robot.route[i].x!= -1 && robot.route[i].y!= -1)
    {
        // move robot 
        // should update robot.coord_x and robot.coord_y and rotation
        send_position(robot.route[i].x,robot.route[i].y,robot.route[i].rotation, robot.route);
        sleep(2);
        i++;

    }

    return 0;
}