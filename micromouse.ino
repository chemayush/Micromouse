#include <Wire.h>
#include <VL53L0X.h>
#include <Encoder.h>
#include <queue>

struct Cell {
  int x, y, floodValue;
  bool northWall, eastWall, southWall, westWall;
};

const int MAZE_SIZE = 4;
Cell maze[MAZE_SIZE][MAZE_SIZE];

const int CENTER_X = MAZE_SIZE/2;
const int CENTER_Y = MAZE_SIZE/2;

int currX = 3;
int currY = 3;

int nextX = 0;
int nextY = 0;

int prevX = 3;
int prevY = 3;

// Define the encoder pins
#define ENC1_A_PIN 3 // Encoder A pin for Motor 1
#define ENC1_B_PIN 2 // Encoder B pin for Motor 1
#define ENC2_A_PIN 4 // Encoder A pin for Motor 2
#define ENC2_B_PIN 5 // Encoder B pin for Motor 2

// Define the motor pins
#define MOTOR1_PIN_1 7 // Motor 1 pin 1
#define MOTOR1_PIN_2 6 // Motor 1 pin 2
#define MOTOR2_PIN_1 9 // Motor 2 pin 1
#define MOTOR2_PIN_2 8 // Motor 2 pin 2

int direction_increment = 1; // north +1, east +4, south -1, west -4
int currBlock = 1;

int lastDirection = 0;
int currDirection = 0;

double rightDistance = 0;
double leftDistance = 0;
double topDistance = 0;

// Define PID constants for wall following
double Kp = 4; // 4
double Ki = 0.0;
double Kd = 40; // 40

// Define PID constants for turning
double setpoint1 = (-2770 / 2) - 83; // Desired number of encoder ticks
double setpoint2 = (2853 / 2) + 83;
double turn_Kp = 15; // Proportional gain for turning // 15
double turn_Ki = 0.0; // Integral gain for turning
double turn_Kd = 0.01; // Derivative gain for turning // 0.01

// Define other constants
double targetDistance = 80.0; // Target distance from the left wall in mm

// Define variables for PID control
double error, lastError = 0.0;
double integral = 0.0;
double derivative;

// Define encoder objects
Encoder encoder1(ENC1_A_PIN, ENC1_B_PIN);
Encoder encoder2(ENC2_A_PIN, ENC2_B_PIN);

// Define motor speeds
int motorSpeed1 = 0;
int motorSpeed2 = 0;

VL53L0X sensor_right;
VL53L0X sensor_top;
VL53L0X sensor_left;

int blocksCrossed = 0;

void setup() {
  pinMode(MOTOR1_PIN_1, OUTPUT);
  pinMode(MOTOR1_PIN_2, OUTPUT);
  pinMode(MOTOR2_PIN_1, OUTPUT);
  pinMode(MOTOR2_PIN_2, OUTPUT);
  pinMode(13, OUTPUT);
  analogWriteResolution(10);
  
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  sensor_right.setBus(&Wire);
  sensor_top.setBus(&Wire1);
  sensor_left.setBus(&Wire2);

  sensor_right.init();
  sensor_top.init();
  sensor_left.init();

  sensor_right.setTimeout(500);
  sensor_top.setTimeout(500);
  sensor_left.setTimeout(500);

  sensor_right.startContinuous();
  sensor_top.startContinuous();
  sensor_left.startContinuous();

  Serial.begin(115200);

  initializeMaze();
  setDestination();
  //setWall(3, 0, 1);
  readTOF();
  setWalls();
  floodFill();
  printMaze();
  findNextCell();
  nextTurn();
  //setWalls();
  //turn90DegreesLeft();
}

void loop() {

  readTOF();
  step();
  move();

  if ( (currX == 1 && currY == 1) || (currX == 1 && currY == 2) || (currX == 2 && currY == 1) || (currX == 2 && currY == 2)) {
    while (1) {
      stop(); // Emergency stop if the micromouse reaches a certain block
    }
  }

  //Serial.println(currDirection);
  //Serial.print(nextX);
  //Serial.println(currDirection);
  //printMaze();
}

void move() {

  if (leftDistance > 90 && rightDistance > 90) {
    forward();
  } else if (rightDistance < 100) {
    followRightWall();
  } else if (leftDistance < 100) {
    followLeftWall();
  } else if (rightDistance < 100 && leftDistance < 100) {
    followLeftWall();
  } else {
    forward();
  }

}

void followRightWall() {

  /*if (topDistance < 55) {
    turn90DegreesLeft(); // Turn left if an obstacle is detected in front and a right turn is possible
    return;
  }*/

  // Calculate error for wall following
  error = 70 - rightDistance; // Adjust error calculation for right wall following

  // Calculate PID terms for wall following
  integral += error;
  derivative = error - lastError;

  // Calculate PID output for wall following
  double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Update last error for wall following
  lastError = error;

  // Adjust motor speeds based on PID output for wall following
  double leftSpeed = constrain(500 + output, 0, 700); // Adjust this based on your motor direction
  double rightSpeed = constrain(500 - output, 0, 700); // Adjust this based on your motor direction

  // Apply motor speeds for wall following
  analogWrite(MOTOR1_PIN_1, leftSpeed);
  analogWrite(MOTOR1_PIN_2, 0);
  analogWrite(MOTOR2_PIN_1, rightSpeed);
  analogWrite(MOTOR2_PIN_2, 0);
}

void followLeftWall() {

  /*if (topDistance < 55) {
    turn90DegreesRight();
    return;
  }*/

  // Calculate error for wall following
  error = targetDistance - leftDistance;

  // Calculate PID terms for wall following
  integral += error;
  derivative = error - lastError;

  // Calculate PID output for wall following
  double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Update last error for wall following
  lastError = error;

  // Adjust motor speeds based on PID output for wall following
  double leftSpeed = constrain(500 - output, 0, 700); // Adjust this based on your motor direction
  double rightSpeed = constrain(500 + output, 0, 700); // Adjust this based on your motor direction

  // Apply motor speeds for wall following
  analogWrite(MOTOR1_PIN_1, leftSpeed);
  analogWrite(MOTOR1_PIN_2, 0);
  analogWrite(MOTOR2_PIN_1, rightSpeed);
  analogWrite(MOTOR2_PIN_2, 0);
}

void step() {
  if (encoder1.read() >= 4015) {
    encoder1.write(0);
    blocksCrossed++;
    digitalWrite(13, (blocksCrossed%2)*4);
    prevX = currX;
    prevY = currY;
    if (currDirection == 0) {
      direction_increment = 1;
      currX--;
    } else if (currDirection == 1) {
      direction_increment = 4;
      currY++;
    } else if (currDirection == 2) {
      direction_increment = -1;
      currX++;
    } else if (currDirection == 3) {
      direction_increment = -4;
      currY--;
    }
    currBlock += direction_increment;

    setWalls();
    resetMaze();
    floodFill();
    //printMaze();
    //Serial.print(1);
    findNextCell();
    //Serial.print(2);
    nextTurn();
    //Serial.print(3);
  }
}

void nextTurn() {
  int dx = nextX - currX;
  int dy = nextY - currY;

  //Serial.print(currX);
  //Serial.println(currY);
  //Serial.println(currDirection);

  // Assuming your current direction is represented by an integer 
  // (0 - North, 1 - East, 2 - South, 3 - West)

  if (currDirection == 0) {
    if (dx == -1) { // Need to move North
      forward();
    } else if (dx == 1) { // Need to move South
      
      turn180Degrees();
    } else if (dy == 1) { // Need to move East
     
      turn90DegreesRight();
    } else if (dy == -1) { // Need to move West
      turn90DegreesLeft();
    } else {
      stop();
      return -1; // Error
    }
  } else if (currDirection == 1) {
    if (dx == -1) { // Need to move North
      turn90DegreesLeft();
    } else if (dx == 1) { // Need to move South
      turn90DegreesRight();
    } else if (dy == 1) { // Need to move East
      forward();
    } else if (dy == -1) { // Need to move West
      turn180Degrees();
    } else {
      stop();
      return -1; // Error
    }
  } else if (currDirection == 2) {
    if (dx == -1) { // Need to move North
      turn180Degrees();
    } else if (dx == 1) { // Need to move South
      forward();
    } else if (dy == 1) { // Need to move East
      turn90DegreesLeft();
    } else if (dy == -1) { // Need to move West
      turn90DegreesRight();
    } else {
      stop();
      return -1; // Error
    }
  } else if (currDirection == 3) {
    if (dx == -1) { // Need to move North
      turn90DegreesRight();
    } else if (dx == 1) { // Need to move South
      turn90DegreesLeft();
    } else if (dy == 1) { // Need to move East
      turn180Degrees();
    } else if (dy == -1) { // Need to move West
      forward();
    } else {
      stop();
      return -1; // Error
    }
  }
}

void turn90DegreesRight() {
  // Reset encoder counts to zero
  encoder1.write(0);
  encoder2.write(0);

  // Define PID constants for turning
  double turn_Kp = 1.5; // Proportional gain for turning
  double turn_Ki = 0.0; // Integral gain for turning
  double turn_Kd = 0.001; // Derivative gain for turning

  // Define variables for PID control
  double error1, error2, lastError1 = 0.0, lastError2 = 0.0;
  double integral1 = 0.0, integral2 = 0.0;
  double derivative1, derivative2;
  int motorSpeed1 = 0, motorSpeed2 = 0;

  // Loop until both motors reach their target encoder counts
  while (abs(encoder1.read()) < abs(setpoint1) || abs(encoder2.read()) < abs(setpoint2)) {
    // Read encoder values
    int encoderValue1 = encoder1.read();
    int encoderValue2 = encoder2.read();

    // Calculate error for each motor
    error1 = setpoint1 - encoderValue1;
    error2 = setpoint2 - encoderValue2;

    // Calculate integral term for each motor
    integral1 += error1;
    integral2 += error2;

    // Calculate derivative term for each motor
    derivative1 = error1 - lastError1;
    derivative2 = error2 - lastError2;

    // Calculate PID output for each motor
    motorSpeed1 = turn_Kp * error1 + turn_Ki * integral1 + turn_Kd * derivative1;
    motorSpeed2 = turn_Kp * error2 + turn_Ki * integral2 + turn_Kd * derivative2;

    motorSpeed1 = constrain(motorSpeed1, -512, 512);
    motorSpeed2 = constrain(motorSpeed2, -512, 512);
    // Update last errors for each motor
    lastError1 = error1;
    lastError2 = error2;

    // Apply motor speeds with direction for turning
    if (motorSpeed1 > 0) {
      analogWrite(MOTOR1_PIN_1, motorSpeed1);
      analogWrite(MOTOR1_PIN_2, 0);
    } else {
      analogWrite(MOTOR1_PIN_1, 0);
      analogWrite(MOTOR1_PIN_2, -motorSpeed1);
    }

    if (motorSpeed2 > 0) {
      analogWrite(MOTOR2_PIN_1, motorSpeed2);
      analogWrite(MOTOR2_PIN_2, 0);
    } else {
      analogWrite(MOTOR2_PIN_1, 0);
      analogWrite(MOTOR2_PIN_2, -motorSpeed2);
    }
  }

  lastDirection = currDirection;
  currDirection++;

  if (currDirection > 3) {
    currDirection = 0;
  }

  // Stop motors after the turn
  stop();

  encoder1.write(0);
  encoder2.write(0);
}

void turn90DegreesLeft() {
  // Reset encoder counts to zero

  double setpoint1 = (2770 / 1.8) ; // Desired number of encoder ticks
  double setpoint2 = (-2853 / 1.8);

  encoder1.write(0);
  encoder2.write(0);

  // Define PID constants for turning
  double turn_Kp = 15; // Proportional gain for turning
  double turn_Ki = 0.0; // Integral gain for turning
  double turn_Kd = 0.01; // Derivative gain for turning

  // Define variables for PID control
  double error1, error2, lastError1 = 0.0, lastError2 = 0.0;
  double integral1 = 0.0, integral2 = 0.0;
  double derivative1, derivative2;
  int motorSpeed1 = 0, motorSpeed2 = 0;

  // Loop until both motors reach their target encoder counts
  while (abs(encoder1.read()) < abs(setpoint1) || abs(encoder2.read()) < abs(setpoint2)) {

    int encoderValue1 = encoder1.read();
    int encoderValue2 = encoder2.read();

    // Calculate error for each motor
    error1 = setpoint1 - encoderValue1;
    error2 = setpoint2 - encoderValue2;

    // Calculate integral term for each motor
    integral1 += error1;
    integral2 += error2;

    // Calculate derivative term for each motor
    derivative1 = error1 - lastError1;
    derivative2 = error2 - lastError2;

    // Calculate PID output for each motor
    motorSpeed1 = turn_Kp * error1 + turn_Ki * integral1 + turn_Kd * derivative1;
    motorSpeed2 = turn_Kp * error2 + turn_Ki * integral2 + turn_Kd * derivative2;

    motorSpeed1 = constrain(motorSpeed1, -512, 512);
    motorSpeed2 = constrain(motorSpeed2, -512, 512);
    // Update last errors for each motor
    lastError1 = error1;
    lastError2 = error2;

    // Apply motor speeds with direction for turning left
    if (motorSpeed1 > 0) {
      analogWrite(MOTOR1_PIN_1, motorSpeed1);
      analogWrite(MOTOR1_PIN_2, 0);
    } else {
      analogWrite(MOTOR1_PIN_1, 0);
      analogWrite(MOTOR1_PIN_2, -motorSpeed1);
    }

    if (motorSpeed2 > 0) {
      analogWrite(MOTOR2_PIN_1, motorSpeed2);
      analogWrite(MOTOR2_PIN_2, 0);
    } else {
      analogWrite(MOTOR2_PIN_1, 0);
      analogWrite(MOTOR2_PIN_2, -motorSpeed2);
    }
  }

  lastDirection = currDirection;
  currDirection--;

  if (currDirection < 0) {
    currDirection = 3;
  }

  // Stop motors after the turn
  stop();

  encoder1.write(0);
  encoder2.write(0);
}

void turn180Degrees() {
  // Reset encoder counts to zero

  while (topDistance > 55) {
    readTOF();
    move();
  }
  encoder1.write(0);
  encoder2.write(0);

  // Define PID constants for turning
  double turn_Kp = 1.5; // Proportional gain for turning
  double turn_Ki = 0.0; // Integral gain for turning
  double turn_Kd = 0.001; // Derivative gain for turning

  // Define variables for PID control
  double error1, error2, lastError1 = 0.0, lastError2 = 0.0;
  double integral1 = 0.0, integral2 = 0.0;
  double derivative1, derivative2;
  int motorSpeed1 = 0, motorSpeed2 = 0;

  // Loop until both motors reach their target encoder counts
  while (abs(encoder1.read()) < abs(setpoint1*2+25) || abs(encoder2.read()) < abs(setpoint2*2)+25) {
    // Read encoder values
    int encoderValue1 = encoder1.read();
    int encoderValue2 = encoder2.read();

    // Calculate error for each motor
    error1 = setpoint1*2-40 - encoderValue1;
    error2 = setpoint2*2+25 - encoderValue2;

    // Calculate integral term for each motor
    integral1 += error1;
    integral2 += error2;

    // Calculate derivative term for each motor
    derivative1 = error1 - lastError1;
    derivative2 = error2 - lastError2;

    // Calculate PID output for each motor
    motorSpeed1 = turn_Kp * error1 + turn_Ki * integral1 + turn_Kd * derivative1;
    motorSpeed2 = turn_Kp * error2 + turn_Ki * integral2 + turn_Kd * derivative2;

    motorSpeed1 = constrain(motorSpeed1, -512, 512);
    motorSpeed2 = constrain(motorSpeed2, -512, 512);
    // Update last errors for each motor
    lastError1 = error1;
    lastError2 = error2;

    // Apply motor speeds with direction for turning
    if (motorSpeed1 > 0) {
      analogWrite(MOTOR1_PIN_1, motorSpeed1);
      analogWrite(MOTOR1_PIN_2, 0);
    } else {
      analogWrite(MOTOR1_PIN_1, 0);
      analogWrite(MOTOR1_PIN_2, -motorSpeed1);
    }

    if (motorSpeed2 > 0) {
      analogWrite(MOTOR2_PIN_1, motorSpeed2);
      analogWrite(MOTOR2_PIN_2, 0);
    } else {
      analogWrite(MOTOR2_PIN_1, 0);
      analogWrite(MOTOR2_PIN_2, -motorSpeed2);
    }
  }

  lastDirection = currDirection;
  currDirection += 2;

  if (currDirection > 3) {
    currDirection = currDirection - 4;
  }

  // Stop motors after the turn
  stop();

  encoder1.write(0);
  encoder2.write(0);
}

void stop() {
  analogWrite(MOTOR1_PIN_1, 0);
  analogWrite(MOTOR1_PIN_2, 0);
  analogWrite(MOTOR2_PIN_1, 0);
  analogWrite(MOTOR2_PIN_2, 0);
}

void forward() {
  /*if (topDistance < 55) {
    stop();
    if (rightDistance < leftDistance) {
      turn90DegreesRight();
    } else {
      turn90DegreesLeft();
    }
  }*/
  analogWrite(MOTOR1_PIN_1, 433);
  analogWrite(MOTOR1_PIN_2, 0);
  analogWrite(MOTOR2_PIN_1, 512);
  analogWrite(MOTOR2_PIN_2, 0);
}

void readTOF() {
  rightDistance = sensor_right.readRangeContinuousMillimeters();
  topDistance = sensor_top.readRangeContinuousMillimeters();
  leftDistance = sensor_left.readRangeContinuousMillimeters();
}

void setDestination() {
  for (int i = CENTER_X - 1; i <= CENTER_X; i++) {
    for (int j = CENTER_Y - 1; j <= CENTER_Y; j++) {
      maze[i][j].floodValue = 0;
    }
  }
}

void setDestination4x4(int x, int y) {
	maze[x][y].floodValue = 0;
}

void initializeMaze() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j].x = i;
      maze[i][j].y = j;
      maze[i][j].northWall = i == 0;       // Boundary walls
      maze[i][j].eastWall  = j == MAZE_SIZE - 1;
      maze[i][j].southWall = i == MAZE_SIZE - 1;
      maze[i][j].westWall  = j == 0;
      maze[i][j].floodValue = -1;          // -1 means not flooded yet
    }
  }
}

void resetMaze() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j].floodValue = -1;         
    }
  }
  
  setDestination();
}

void resetMaze4x4() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j].floodValue = -1;         
    }
  }
  
  setDestination();
}

bool isValidNeighbor(int x, int y) {
  return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

void floodFill() {
  std::queue<Cell> cellQueue;
  for (int i = CENTER_X - 1; i <= CENTER_X; i++) {
    for (int j = CENTER_Y - 1; j <= CENTER_Y; j++) {
      cellQueue.push(maze[i][j]);
    }
  }

  int dx[] = {-1, 0, 1, 0};
  int dy[] = {0, 1, 0, -1};

  while (!cellQueue.empty()) {
    Cell current = cellQueue.front();
    cellQueue.pop();

    for (int i = 0; i < 4; i++) {
      int neighborX = current.x + dx[i];
      int neighborY = current.y + dy[i];

      if (isValidNeighbor(neighborX, neighborY)) {
        Cell& neighbor = maze[neighborX][neighborY];

        // Check for wall and if not already flooded
        if (!current.northWall && i == 0 && neighbor.floodValue == -1) { 
          neighbor.floodValue = current.floodValue + 1;
          cellQueue.push(neighbor);
        } else if (!current.eastWall && i == 1 && neighbor.floodValue == -1) {
          neighbor.floodValue = current.floodValue + 1;
          cellQueue.push(neighbor);
        } else if (!current.southWall && i == 2 && neighbor.floodValue == -1) {
          neighbor.floodValue = current.floodValue + 1;
          cellQueue.push(neighbor);
        } else if (!current.westWall && i == 3 && neighbor.floodValue == -1) {
          neighbor.floodValue = current.floodValue + 1;
          cellQueue.push(neighbor);
        }
      }
    }
  }
}

void findNextCell() {
  int minFloodValue = maze[currX][currY].floodValue - 1; // Look for values one less
  int potentialNextX = currX;
  int potentialNextY = currY;
  int potentialDirection = -1; // -1 indicates no valid move found

  Cell current = maze[currX][currY];

  int dx[] = {-1, 0, 1, 0};
  int dy[] = {0, 1, 0, -1};

  // Prioritized directions based on currDirection
  int directions[4];
  if (currDirection == 0) { // North
      directions[0] = 0; // North
      directions[1] = 1; // East
      directions[2] = 3; // West
      directions[3] = 2; // South
  } else if (currDirection == 1) { // East
      directions[0] = 1; // East
      directions[1] = 0; // North
      directions[2] = 2; // South
      directions[3] = 3; // West
  } else if (currDirection == 2) { // South
      directions[0] = 2; // South
      directions[1] = 3; // West 
      directions[2] = 1; // East
      directions[3] = 0; // North
  } else if (currDirection == 3) { // West
      directions[0] = 3; // West
      directions[1] = 2; // South
      directions[2] = 0; // North
      directions[3] = 1; // East
  } 

  // Search for the next cell based on prioritized directions
  for (int i = 0; i < 4; i++) {
    int dir = directions[i];
    int neighborX = currX + dx[dir];
    int neighborY = currY + dy[dir];

    if (isValidNeighbor(neighborX, neighborY)) {
      Cell& neighbor = maze[neighborX][neighborY];

      // Check for wall and if not already flooded
      if (!current.northWall && dir == 0 && neighbor.floodValue == minFloodValue) { 
        potentialNextX = neighborX;
        potentialNextY = neighborY;
        potentialDirection = dir; 
        break; // Prioritize finding the next cell earlier in the loop for efficiency
      } else if (!current.eastWall && dir == 1 && neighbor.floodValue == minFloodValue) {
        potentialNextX = neighborX;
        potentialNextY = neighborY;
        potentialDirection = dir;
        break; 
      } else if (!current.southWall && dir == 2 && neighbor.floodValue == minFloodValue) {
        potentialNextX = neighborX;
        potentialNextY = neighborY;
        potentialDirection = dir;
        break;
      } else if (!current.westWall && dir == 3 && neighbor.floodValue == minFloodValue) {
        potentialNextX = neighborX;
        potentialNextY = neighborY;
        potentialDirection = dir;
        break;
      }
    }
  }

  // Update the next cell coordinates and direction if a valid move is found
  if (potentialDirection != -1) {
    nextX = potentialNextX;
    nextY = potentialNextY;
    //currDirection = potentialDirection;
  }

}

void setWall(int x, int y, int dir) {
	
	// 0 north, 1 east, 2 south, 3 west
	if (dir == 0) {
		maze[x][y].northWall = 1;
		if (x-1 >= 0) {
			maze[x-1][y].southWall = 1;
		}
	} else if (dir == 1) {
		maze[x][y].eastWall = 1;
		if (y+1 <= MAZE_SIZE-1) {
			maze[x][y+1].westWall = 1;
		}
	} else if (dir == 2) {
		maze[x][y].southWall = 1;
		if (x+1 <= MAZE_SIZE-1) {
			maze[x+1][y].northWall = 1;
		}
	} else if (dir == 3) {
		maze[x][y].westWall = 1;
		if (y-1 >= 0) {
			maze[x][y-1].eastWall = 1;
		}
	}
}

void printMaze() {
    for (int i=0; i<MAZE_SIZE; i++) {
    	for (int j=0; j<MAZE_SIZE; j++) {
    		Serial.print(maze[i][j].floodValue);
        Serial.print(" ");
		}
		Serial.println();
	}
	Serial.println();
}

bool isTop() {
  if (topDistance < 100) {
    return true;
  } else {
    return false;
  }
}

bool isRight() {
  if (rightDistance < 100) {
    return true;
  } else {
    return false;
  }
}

bool isLeft() {
  if (leftDistance < 100) {
    return true;
  } else {
    return false;
  }
}

void setWalls() {
  if (currDirection == 0) {
    if (isTop()) {
      setWall(currX, currY, 0);
    }

    if (isLeft()) {
      setWall(currX, currY, 3);
    }

    if (isRight()) {
      setWall(currX, currY, 1);
    }
  } else if (currDirection == 1) {
    if (isTop()) {
      setWall(currX, currY, 1);
    }

    if (isRight()) {
      setWall(currX, currY, 2);
    }

    if (isLeft()) {
      setWall(currX, currY, 0);
    }
  } else if (currDirection == 2) {
    if (isTop()) {
      setWall(currX, currY, 2);
    }

    if (isRight()) {
      setWall(currX, currY, 3);
    }

    if (isLeft()) {
      setWall(currX, currY, 1);
    }
  } else if (currDirection == 3) {
    if (isTop()) {
      setWall(currX, currY, 3);
    }

    if (isRight()) {
      setWall(currX, currY, 0);
    }

    if (isLeft()) {
      setWall(currX, currY, 2);
    }
  }
}