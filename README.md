# Micromouse
This project involves making of a micromouse (a very small maze solving robot) based on the FloodFill algorithm.
Won the BITS Pilani, Pilani Micromous '24.

Components:
1. Teensy 4.1 Microcontroller
2. 300 RPM N20 DC Motors with encoders (2800 ticks at the shaft)
3. DRV8833 Motor Driver
4. VL53L0X ToF Sensors (x3)
5. 7805 Voltage Regulator

Functions in the code:

1. move():
It does the decision making for choosing the type of wall-following depending on the current presence of walls around the mouse.

2. findStartCoordinates1():
It finds the start position of the mouse i.e. at which corner of the maze did the mouse start.

3. followRightWall() / followLeftWall():
This function uses PID control to follow the wall on right/left side of the mouse.

4. step():
This function is called everytime the mouse moves from one cell of the maze to another. It sets the current coordinates, updates the presence of walls in the 'maze' data structure, floods the maze again and finds the new shortest path based on updated walls and then finally calculates the next turn to be taken.

5. nextTurn():
It calculates the next turn to be taken based on the current and next coordinates, and the current direction of the mouse.

6. setDestination():
This function sets the destination of the mouse as the center 4 squares of the maze. It is called in the setup() function.

7. initializeMaze():
It initialzes the maze based the provided dimensions of the maze. Initially maze is assumed to have only the side walls and no walls inside the maze.

8. resetMaze():
It resets the floodValues of each cell to -1 before calling the floodFill() function to flood the maze again.

9. isValidNeighbor(int x, int y):
checks if the coordinates passed in the function are valid neighbour of the current coordinates or not.

10. floodFill():






