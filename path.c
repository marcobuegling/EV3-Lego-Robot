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
        }
        else{
            printf("Drive Forward: %i , %i", curX, curY);
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
        }
        else{
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
        }
        else{
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
        }
        else{
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

    if ( ((x == 1) && (y == 8)) || ((x == 7) && (y == 8)) ){
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
