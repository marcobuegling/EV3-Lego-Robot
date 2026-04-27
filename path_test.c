#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

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


typedef struct position {
     int8_t x;
     int8_t y;
     int8_t rotation;
} position;


position route[32];

// Print enum
void printDirection(dmode_t dir){
    switch(dir){
        case TURN_LEFT:
            printf("Turn Left");
            return;
        case TURN_RIGHT:
            printf("Turn Right");
            return;
        case DRIVE_FORWARD:
            printf("Drive forward");
            return;
        case STOP:
            printf("Stop!");
            return;
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
dmode_t getExitDirection(int curX, int curY, direction_t curDir, int goalX, int goalY){

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
dmode_t bestDir(int curX, int curY, direction_t curDir, int goalX, int goalY){

    // If facing north
    if (curDir == Y_POS){

        


        //Check if driving forward is good
        if ( (abs(goalY-(curY+1)) < abs(goalY-curY)) && (goalY-curY != 1 || goalX == curX)) return DRIVE_FORWARD;
        
        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalX-(curX-1)) < abs(goalX-(curX+1))) return TURN_LEFT;
            else return TURN_RIGHT;
        }
        else{
            return DRIVE_FORWARD;
        }
    }

    // If facing south
    else if (curDir == Y_NEG){

        //Check if driving forward is good
        if (abs(goalY-(curY-1)) < abs(goalY-curY) && (curY-goalY != 1 || goalX == curX)) return DRIVE_FORWARD;

       // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalX-(curX+1)) < abs(goalX-(curX-1))) return TURN_LEFT;
            else return TURN_RIGHT;
        }
        else{
            return DRIVE_FORWARD;
        }
    }

    // If facing east
    else if (curDir == X_POS){

       
        //Check if driving forward is good
        if (abs(goalX-(curX+1)) < abs(goalX-curX) && (goalX-curX != 1 || goalY == curY)) return DRIVE_FORWARD;

        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalY-(curY+1)) < abs(goalY-(curY-1))) return TURN_LEFT;
            else return TURN_RIGHT;
        }
        else{
            return DRIVE_FORWARD;
        }
    }

    // If facing west
    else if (curDir == X_NEG){

       
        //Check if driving forward is good
        if (abs(goalX-(curX-1)) < abs(goalX-curX) && (curX-goalX != 1 || goalY == curY)) return DRIVE_FORWARD;

        // Check if turning is legal and left is better than right
        if (curX%2==0 && curY%2==0){
            if (abs(goalY-(curY-1)) < abs(goalY-(curY+1))) return TURN_LEFT;
            else return TURN_RIGHT;
        }
        else{
            return DRIVE_FORWARD;
        }
    }
}


// Get entry turning direction for branch
dmode_t getEntryDirection(int x, int y, direction_t dir){

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

    if ( ((x == 1) && (y == 8)) || ((x == 7) && (y == 8)) ){
        if (dir == X_POS) return TURN_RIGHT;
        else return TURN_LEFT;
    } 
   
}



//Find next direction from current location to goal
dmode_t findWay(int curX, int curY, direction_t curDir, int goalX, int goalY){

    // Check if we have reached the goal
     if ((curX == goalX) && (curY == goalY)){
         return STOP;
     }

    // Lets check if we are leaving a branch
    bool leaveBranch = leavingBranch(curX, curY, curDir);

    // If we are leaving a branch, let's exit in a direction towards the goal
    if (leaveBranch){
        return getExitDirection(curX, curY, curDir, goalX, goalY);
    }

    //Lets get the best direction
    return bestDir(curX, curY, curDir, goalX, goalY);


 }



int8_t directionToRotation(direction_t dir){
    if (dir == X_NEG){
        return 0;
    }
    else if (dir == X_POS){
         return 2;
    }
    else if (dir == Y_NEG){
         return 3;
    }
    else if (dir == Y_POS){
         return 1;
    }
}

void generateRoute(int curX, int curY, direction_t curDir, int goalX, int goalY, position route[]){
    

    for (int i = 0; i < 32; i++){
        
        // Null array
        route[i].y = -1;
        route[i].x = -1;
        route[i].rotation = -1;

    }

    for (int i = 0; i < 32; i++){
        
        // Fill in array of position structs
        route[i].y = curY;
        route[i].x = curX;
        route[i].rotation = directionToRotation(curDir);
        
        //Get next position
        dmode_t dir = findWay(curX, curY, curDir, goalX,  goalY);
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




// int main(void) {

    
//     // Test leaving a branch
//     dmode_t nextDir;
//     printf("Test leaving a branch");

//     nextDir = findWay(1, 0,Y_NEG, 7, 0);
//     printf("\n 1. Turn Left==");
//     printDirection(nextDir);

//     nextDir = findWay(7, 0,Y_NEG, 6, 3);
//     printf("\n 2. Turn Right==");
//     printDirection(nextDir);

//     nextDir = findWay(1, 8,Y_POS, 6, 5);
//     printf("\n 3. Turn Right==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 5, X_NEG, 6, 5);
//     printf("\n 4. Turn Right==");
//     printDirection(nextDir);


//     // Test best directions
//     printf("\n\n Test best direction");




//     printf("\n Test(2,3, WEST) to (1,0)");
//     nextDir = findWay(2, 3, X_NEG, 1, 0);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 2, Y_NEG, 1, 0);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 1, Y_NEG, 1, 0);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 0, Y_NEG, 1, 0);
//     printf("\n Turn Right==");
//     printDirection(nextDir);

//     nextDir = findWay(1, 0, Y_NEG, 1, 0);
//     printf("\n Stop!==");
//     printDirection(nextDir);



//     printf("\n\n Test(1,0, SOUTH) to (2,3)");
//     nextDir = findWay(1, 0, Y_NEG, 2, 3);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 0, X_POS, 2, 3);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 1, Y_POS, 2, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 2, Y_POS, 2, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 3, Y_NEG, 2, 3);
//     printf("\n Stop!==");
//     printDirection(nextDir);



//     printf("\n\n Test(0,0, EAST) to (6,3)");
//     nextDir = findWay(0, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(1, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(3, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(4, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(5, 0, X_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(6, 0, X_POS, 6, 3);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

//     nextDir = findWay(6, 1, Y_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(6, 2, Y_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//      nextDir = findWay(6, 3, Y_POS, 6, 3);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

    


//     printf("\n\n Test (2,5, WEST) to (7,8)");
//     nextDir = findWay(2, 5, X_NEG, 7, 8);
//     printf("\n Turn Right==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 6, Y_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(2, 7, Y_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//      nextDir = findWay(2, 8, Y_POS, 7, 8);
//     printf("\n Turn Right==");
//     printDirection(nextDir);

//     nextDir = findWay(3, 8, X_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(4, 8, X_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(5, 8, X_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(6, 8, X_POS, 7, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);

//     nextDir = findWay(7, 8, X_POS, 7, 8);
//     printf("\n Stop!==");
//     printDirection(nextDir);




//     printf("\n \n Test Weird Tasks");
//     printf("\n Test (0,6, EAST) to (6,5)");
//     nextDir = findWay(0, 6, X_POS, 6, 5);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);
    
//     printf("\n Test (0,7, North) to (6,5)");
//     nextDir = findWay(0, 7, Y_POS, 6, 5);
//     printf("\n Turn Right==");
//     printDirection(nextDir);

//     printf("\n Test (0,0, South) to (6,5)");
//     nextDir = findWay(0, 0, Y_NEG, 6, 5);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

//     printf("\n Test (6,5, South) to (2,5)");
//     nextDir = findWay(6, 5, X_POS, 2, 5);
//     printf("\n Doesn't matter==");
//     printDirection(nextDir);

//     printf("\n Test (6,0, West) to (1,8)");
//     nextDir = findWay(6, 0, X_NEG, 1, 8);
//     printf("\n Drive Forward==");
//     printDirection(nextDir);



//     printf("\n \n Test Entry Into Branch");
//     printf("\n Test (2,5, North)");
//     nextDir = getEntryDirection(2, 5, Y_POS);
//     printf("\n Turn Right==");
//     printDirection(nextDir);

//     printf("\n Test (2,5, South)");
//     nextDir = getEntryDirection(2, 5, Y_NEG);
//     printf("\n Turn Left==");
//     printDirection(nextDir);

    
    
//     // printf("\n\n Route From (2,5, WEST) to (7,8)");
//     // generateRoute(2, 5, X_NEG, 7, 8);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }


//     // printf("\n\n Route From (2,5, WEST) to (7,0)");
//     // generateRoute(2, 5, X_NEG, 7, 0);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }


//     // printf("\n\n Route From (7,0, SOUTH) to (1,8)");
//     // generateRoute(7, 0, Y_NEG, 1, 8);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (0,0, EAST) to (6,5)");
//     // generateRoute(0, 0, X_POS, 6, 5);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }


//     // printf("\n\n Route From (6,5, EAST) to (2,3)");
//     // generateRoute(6, 5, X_POS, 2, 3);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (7, 8, NORTH) to (1,0)");
//     // generateRoute(7, 8, Y_POS, 1, 0);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (6,5, EAST) to (1,0)");
//     // generateRoute(6, 5, X_POS, 1, 0);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (6,5, EAST) to (1,0)");
//     // generateRoute(6, 5, X_POS, 1, 0);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (1,0, SOUTH) to (7,0)");
//     // generateRoute(1, 0, Y_NEG, 7, 0);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (2,5, WEST) to (6,5)");
//     // generateRoute(2, 5, X_NEG, 6, 5);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }

//     // printf("\n\n Route From (0, 0, EAST) to (7,8)");
//     // generateRoute(0, 0, X_POS, 7, 8);
//     // for (int i = 0; i < 32; i++){
//     //     printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
//     // }
   
void main(){
    printf("\n\n Route From (0,0 EAST) to (2,3)");
    generateRoute(0, 0, X_POS, 2, 3, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }

    printf("\n\n Route From (0,0 NORTH) to (2,3)");
    generateRoute(0, 0, Y_POS, 2, 3, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }

    printf("\n\n Route From (0,8 SOUTH) to (2,5)");
    generateRoute(0, 8, Y_NEG, 2, 5, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }

    printf("\n\n Route From (8, 0 NORTH) to (6,3)");
    generateRoute(8, 0, Y_POS, 6, 3, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }

    printf("\n\n Route From (0, 0 EAST) to (1,8)");
    generateRoute(0, 0, X_POS, 1, 8, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }

    printf("\n\n Route From (0, 0 EAST) to (1,0)");
    generateRoute(0, 0, X_POS, 1, 0, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }


    printf("\n\n Route From (8, 8 WEST) to (7,0)");
    generateRoute(8, 8, X_NEG, 7, 0, route);
        for (int i = 0; i < 32; i++){
        printf("\n (%i, %i) Rotation: %i", route[i].x, route[i].y, route[i].rotation);
    }


}