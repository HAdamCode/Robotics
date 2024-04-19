
/* Lab 4: maze solving
   last updated: gkn 20240327
*/


#include <PinChangeInterrupt.h>
#include <HCSR04.h> // If using any Ultrasonic Distance Sensors, 
// This code assumes the HC-SR04 Library by gamegine, but many work








// TODO: Copy constants and definitions from Lab 3








// Define IR Distance sensor Pins
#define frontIR A0
#define sideIR  A1


// If using any Ultrasonic - change pins for your needs
#define trig 5
#define echo 6
#define pushButton 2


#define A 0
#define B 1
#define EncoderMotorLeft  7
#define EncoderMotorRight 10


#define FORWARD             3
#define LEFT                2
#define RIGHT               4


// if side sensor is Ultrasonic
HCSR04 sideUS(trig, echo);
// if front sensor is Ultrasonic
//HCSR04 frontUS(trig, echo);


// Define the distance tolerance that indicates a wall is present
#define wallTol 13 //cm


int i = 0;
int moves[50]; // Empty array of 50 moves, probably more than needed, just in case
int finishedMoves[50];
int pos[50][2];





void setup() {
  //TODO: Include setup code for any pins other than motors or encoders
  Serial.begin(9600);
  motor_setup();
  encoder_setup();
  pinMode(pushButton, INPUT_PULLUP);


}








void loop()
{


  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release
  explore();
  run_motor(A, 0);
  run_motor(B, 0);
  solve();
  while (true) { //Inifnite number of runs, so you don't have to re-explore everytime a mistake happens
    while (digitalRead(pushButton) == 1); // wait for button push
    while (digitalRead(pushButton) == 0); // wait for button release
    runMaze();
    run_motor(A, 0);
    run_motor(B, 0);
  }
}




float readFrontDist() { 
  // If IR distance sensor
  int reading = analogRead(frontIR);
  float voltage = reading * 5.0 / 1023;
  Serial.println(voltage);
  float dist = 1/(0.0987 * voltage - .0504) - .24;  // Equation from your calibration;


  // if Ultrasonic
  // float dist = frontUS.dist(); //(returns in cm)
  if (dist <= 0) {
    dist = 13.1;
  } else {
    dist -= 4;
  }

  return dist;
}




float readSideDist() {
  // If IR distance sensor
//  int reading = analogRead(sideIR);
//  float dist = // Equation from your calibration;


  // IF Ultrasonic
  // float dist = sideUS.dist(); //(returns in cm)
  float dist = sideUS.dist();
//  Serial.print("Side Distance: ");
//  Serial.println(dist);
  
  return dist;
}






void explore() {
  int direction = 0;
  int currentX = 0;
  int currentY = 0;
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    float side = readSideDist()-2;
    float front = readFrontDist();
    if (side > wallTol && front > wallTol) {
      Serial.println("LEFT");
      turn(15, 26);
      moves[i] = LEFT;
      direction--;
      
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i+1] = FORWARD;
      if (direction % 4 == 0) {
        currentY++;
      } else if (direction % 4 == 1) {
        currentX++;
      } else if (direction % 4 == 2) {
        currentY--;
      } else {
        currentX--;
      }
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
    }
    else if (side > wallTol) {// If side is not a wall
      // turn and drive forward
      // Record actions
      Serial.println("LEFT");
      turn(15, 26);
      moves[i] = LEFT;
      direction--;
      
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i+1] = FORWARD;
      if (direction % 4 == 0) {
        currentY++;
      } else if (direction % 4 == 1) {
        currentX++;
      } else if (direction % 4 == 2) {
        currentY--;
      } else {
        currentX--;
      }
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
    }
    else if (front > wallTol) {// else if front is not a wall
      // drive forward
      // Record action
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i] = FORWARD;
      if (direction % 4 == 0) {
        currentY++;
      } else if (direction % 4 == 1) {
        currentX++;
      } else if (direction % 4 == 2) {
        currentY--;
      } else {
        currentX--;
      }
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
    } else {
      // turn away from side
      // Record action
      Serial.println("RIGHT");
      turn(15, -26);
      moves[i] = RIGHT;
      direction++;
      pos[i][0] = currentX;
      pos[i][1] = currentY;
      i++;
    }
  }
  Serial.println("Explore");
  for (int i = 0; i < 50; i++) {
    Serial.print(" X: ");
    Serial.print(pos[i][0]);
    Serial.print(" Y: ");
    Serial.print(pos[i][1]);
    Serial.print(" Direction: ");
    if (moves[i] == LEFT) {
      Serial.println("LEFT");
    } else if (moves[i] == RIGHT) {
      Serial.println("RIGHT");
    } else {
      Serial.println("FORWARD");
    }
  }
}


void solve() {
  // Write your own algorithm to solve the maze using the list of moves from explore
  int currentX = 0;
  int currentY = 0;
  int previousX = 0;
  int previousY = 0;
  int j = 0;
 for (int i = 0; i < 50; i++) {
  if (i != 0 && pos[i][0] == 0 && pos[i][1] == 0) break;
  bool isLeft = false;
  if (moves[i] == LEFT) {
//    isLeft = true;
    finishedMoves[j] = LEFT;
    j++;
    continue;
  }
  
  currentX = pos[i][0];
  currentY = pos[i][1];
  int currentMove = moves[i];
  int nextMove = moves[i+1];

  

  int positionToGoTo = 0;
  for (int k = 49; k > i; k--) {
    if (currentX == pos[k][0] && currentY == pos[k][1] && moves[k] != LEFT && moves[k] != RIGHT) {
      positionToGoTo = k;
      i = k;
      if (previousX == pos[k+1][0] || previousY == pos[k+1][1]) {
        finishedMoves[j] = currentMove;
        j++;
        if (moves[k+1] == LEFT) {
          i++;
        }
      } else {
        finishedMoves[j] = currentMove;
        j++;
        finishedMoves[j] = RIGHT;
        j++;
      }
      break;
    }
  }
  if (positionToGoTo == 0) {
    finishedMoves[j] = moves[i];
    j++;
  }
 }
 previousX = currentX;
 previousY = currentY;

   for (int i = 0; i < 50; i++) {
//    Serial.print("X: ");
//    Serial.print(pos[i][0]);
//    Serial.print(", Y: ");
//    Serial.println(pos[i][1]);
Serial.println(finishedMoves[i]);
  }
}








void runMaze() {
  
  // Execute each move in the moves array to run through the maze
  for (int j = 0; j < 50; j++) {
    if (finishedMoves[j] == FORWARD) {
      // call the function to drive forward
      driveForward(15, 25); // Assuming 20 is the distance for one cell
    } else if (finishedMoves[j] == LEFT) {
      // call the function to turn left
      turn(15, 26); // Assuming a 90 degree turn
    } else if (finishedMoves[j] == RIGHT) {
      // call the function to turn right
      turn(15, -26); // Assuming a 90 degree turn
    }
    // Include any necessary delay or checks to ensure the movement completes before continuing
  }
}








// Copy any necessary functions from Lab 3
// Consider putting those functions in another .ino file that can be called from this one
// (Any ino files in a folder are automatically imported to the one that shares
// a name with the folder title)
