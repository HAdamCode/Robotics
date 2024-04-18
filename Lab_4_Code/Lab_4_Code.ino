
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


#define FORWARD             0
#define LEFT                1
#define RIGHT               -1


// if side sensor is Ultrasonic
HCSR04 sideUS(trig, echo);
// if front sensor is Ultrasonic
//HCSR04 frontUS(trig, echo);


// Define the distance tolerance that indicates a wall is present
#define wallTol 13 //cm


int i = 0;
int moves[50]; // Empty array of 50 moves, probably more than needed, just in case






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
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    float side = readSideDist();
    float front = readFrontDist();
//    Serial.print("Side dist: ");
//    Serial.println(side);
    Serial.print("Forward dist: ");
    Serial.println(front);
    if (side > wallTol && front > wallTol) {
      // drive forward
      // Record action
      Serial.println("LEFT");
      turn(15, 26);
      moves[i] = LEFT;
      i++;
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i] = FORWARD;
      i++;
    }
    else if (side > wallTol) {// If side is not a wall
      // turn and drive forward
      // Record actions
      Serial.println("LEFT");
      turn(15, 26);
      moves[i] = LEFT;
      i++;
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i] = FORWARD;
      i++;
    }
    else if (front > wallTol) {// else if front is not a wall
      // drive forward
      // Record action
      Serial.println("FORWARD");
      driveForward(15, 25);
      moves[i] = FORWARD;
      i++;
    } else {
      // turn away from side
      // Record action
      Serial.println("RIGHT");
      turn(15, -26);
      moves[i] = RIGHT;
      i++;
    }
  }
}


void reverseArray(int arr[], int size) {
  for (int i = 0; i < size/2; i++) {
    int temp = arr[i];
    arr[i] = arr[size - i - 1];
    arr[size - i - 1] = temp;
  }
}



void solve() {
  // Write your own algorithm to solve the maze using the list of moves from explore
 
  // Pseudo-code for solving the maze
  // Reverse the moves list since we want to backtrack

  reverseArray(moves, sizeof(moves) / sizeof(moves[0]));

  // Loop over the moves and create a new path that represents the optimal solution
  for (int j = 0; j < i; j++) {
    // Invert the turns to get back
    if (moves[j] == LEFT) {
      moves[j] = RIGHT;
    } else if (moves[j] == RIGHT) {
      moves[j] = LEFT;
    }
    // FORWARD stays the same
  }


 
}








void runMaze() {
  
  // Execute each move in the moves array to run through the maze
  for (int j = 0; j < i; j++) {
    if (moves[j] == FORWARD) {
      // call the function to drive forward
      driveForward(15, 20); // Assuming 20 is the distance for one cell
    } else if (moves[j] == LEFT) {
      // call the function to turn left
      turn(15, 90); // Assuming a 90 degree turn
    } else if (moves[j] == RIGHT) {
      // call the function to turn right
      turn(15, -90); // Assuming a 90 degree turn
    }
    // Include any necessary delay or checks to ensure the movement completes before continuing
  }
}








// Copy any necessary functions from Lab 3
// Consider putting those functions in another .ino file that can be called from this one
// (Any ino files in a folder are automatically imported to the one that shares
// a name with the folder title)
