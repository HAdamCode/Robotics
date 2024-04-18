// TODO: Import header code from previous lab code to set up your motors, buttons, pins, etc.

#include <PinChangeInterrupt.h>
///////////////////////////////////////////////////////////////
// Two structures for use in this lab.
#define SHIELD true
//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13

struct velProfile {
  float t1;
  float t2;
  float tf;
  float vel;
};

struct state {
  float x;
  float v;
};
///////////////////////////////////////////////////////////////
// TODO: Input gains for each motor from motor testing code below
float Kp[] = {5.0, 5.0};
float Kd[] = {10.0, 10.0};

// TODO: Max speed of each motor in counts/sec
float maxSpeedA = 130;
float maxSpeedB = 130;


// TODO: Choose a reasonable speed for your robot to drive
float desiredMaxSpeed = 15; // Input desired speed in cm/s, convert later
int desiredDistance = 100; // Input desired test distance in cm

// volatile unsigned int leftEncoderCount = 0;
// volatile unsigned int rightEncoderCount = 0;

int PDdelay = 20;
unsigned long nextPDtime = 0;
int printDelay = 200;
unsigned long nextPrintTime = printDelay;

//int moves[] = {31 ,LEFT, 28, LEFT, 29, RIGHT, 107, RIGHT, 67, RIGHT, 34}; // Fill in this array will forward distances and turn directions in the maze (Like part A)
//int turns[] = {LEFT, LEFT, RIGHT, RIGHT,  RIGHT};

#define EncoderCountsPerRev 36 // Encoder counts per wheel revolution
#define DistancePerRev 13.1947 //pi*d
#define DegreesPerRev 106.4789 //(2*d)*180/b
#define CountersPerCM 2.728 //TODO: Fix this value

#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6


// Lab Specific definitions


// Defining these allows us to use letters in place of binary when
// controlling our motors
#define A 0
#define B 1
#define EncoderMotorLeft  7
#define EncoderMotorRight 10



// Initialize encoder counts to 0
// These are volatile because they change during interrupt functions
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

//void setup() {
//  //TODO: Include setup code for any pins other than motors or encoders
//  Serial.begin(9600);
//  motor_setup();
//  encoder_setup();
//
//  
//  pinMode(EncoderMotorLeft, INPUT_PULLUP); //set the pin to input
//
//
//  // The following code sets up the PinChange Interrupt
//  // Valid interrupt modes are: RISING, FALLING or CHANGE
//  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE);
//  // if you "really" want to know what's going on read the PinChange.h file :)
//  /////////////////////////////////////////////////
//
//  pinMode(EncoderMotorRight, INPUT_PULLUP);     //set the pin to input
//  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE);
//
//   // Activate motors at half-speed for testing
//  // digitalWrite(motorAdir, HIGH); // Adjust this based on your motor's wiring
//  // analogWrite(motorApwm, 128);   // 50% speed
//  // digitalWrite(motorBdir, HIGH); // Adjust this based on your motor's wiring
//  // analogWrite(motorBpwm, 128);   // 50% speed
//  
//  // delay(2000); // Run for 2 seconds
//  
//  // // Stop motors
//  // analogWrite(motorApwm, 0);
//  // analogWrite(motorBpwm, 0);
//}

//void loop() {
//  int i;
//  while (digitalRead(pushButton) == true); // Halts program by iterating digitalRead until button is pressed
//  for (i = 0; i < 11; i++) {
//    if (moves[i] == LEFT) {
//      turn(desiredMaxSpeed, 26);
//    } else if (moves[i] == RIGHT) {
//      turn(desiredMaxSpeed, -26);
//    } else {
//      driveForward(desiredMaxSpeed, moves[i]);
//    }
//
//    run_motor(A, 0);
//    run_motor(B, 0);
//    delay(600);
//  }
//  exit(i);
  // driveForward(desiredMaxSpeed, desiredDistance); // Drives forward (see below)
//}

void driveForward(float maxSpeed, float distance) {
//    Serial.println("-------------------Entered the FORWARD function ---------------------");

  // TODO: Convert maxSpeed to motorSpeed and countsDesired
  float motorSpeed = maxSpeed * CountersPerCM; // Convert from cm/s to counts/s
  float countsDesired =  (int) ((distance*EncoderCountsPerRev)/DistancePerRev); // Can use Lab 2B equation here

  float Xd, Vd, VA, VB;
  int encA, encB;

  // Reset encoder counts at the beginning of the movement.
  unsigned long startTime = millis();
  int cmdA = 0;
  int cmdB = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  int prevEncA = 0;
  int prevEncB = 0;
  nextPDtime = 0;
  
  //TODO: Give this inputs (see trapezoidal.ino)
  struct velProfile profile = genVelProfile(maxSpeed, distance, 0.2);
  //TODO); 

  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  while (millis() - startTime < profile.tf*1000 + 1000) {
    // Serial.println("TIme");
    // Serial.println(millis() - startTime);
    // Serial.println(profile.tf*1000 + 1000);
    unsigned long now = millis() - startTime;
    run_motor(A, cmdA);
    run_motor(B, cmdB);
    if (now > nextPDtime) {
      struct state desiredState = calculateState(now/1000.0, profile);
      Xd = desiredState.x; // Desired position
      Vd = desiredState.v; // Desired speed
      // Get current encoder counts
      encA = leftEncoderCount;
      encB = rightEncoderCount;
      // Get current speed (the 1000 converts PDdelay to seconds)
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;
      // Feed-forward values of pwm for speed based on max speed
      float pwmInA = map(Vd, 0, maxSpeedA, 0, 255); 
      float pwmInB = map(Vd, 0, maxSpeedB, 0, 255);

      // Get command values from controller (as a byte)
      cmdA = pdController(pwmInA, Vd-VA, Xd-encA, Kp[0], Kd[0]);
      cmdB = pdController(pwmInB, Vd-VB, Xd-encB, Kp[1], Kd[1]);

      // Update previous encoder counts
      prevEncA = encA;
      prevEncB = encB;

      // Set next time to update PD controller
      nextPDtime += PDdelay;
    }

    if (now > nextPrintTime) {
      // Output: time, error, velocity error, and pwm for both motors
//      Serial.println(now);
//      Serial.print("Motor A: ");
//      Serial.print(Xd - encA);
//      Serial.print("\t");
//      Serial.print(Vd - VA);
//      Serial.print("\t");
//      Serial.println(cmdA);
//      Serial.print("Motor B: ");
//      Serial.print(Xd - encB);
//      Serial.print("\t");
//      Serial.print(Vd - VB);
//      Serial.print("\t");
//      Serial.println(cmdB);
      nextPrintTime += printDelay;
    }
  }
  // Stop motors
  run_motor(A, 0);
  run_motor(B, 0);
//    Serial.println("-------------------Leaving the FORWARD function ---------------------");

}

void turn(float maxAngularSpeed, float degrees) { // TODO: Figure out how to set direction correctly
//  Serial.println("-------------------Entered the TURN function ---------------------");
  int direction = degrees/abs(degrees);
  // TODO: Modify forward drive function to convert it to turning
    float motorSpeed = maxAngularSpeed * CountersPerCM; // Convert from cm/s to counts/s
  float countsDesired = (int)((abs(degrees) * EncoderCountsPerRev) / DegreesPerRev);; // Can use Lab 2B equation here

  float Xd, Vd, VA, VB;
  int encA, encB;

  // Reset encoder counts at the beginning of the movement.
  unsigned long startTime = millis();
  int cmdA = 0;
  int cmdB = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  int prevEncA = 0;
  int prevEncB = 0;
  nextPDtime= 0;
  
  //TODO: Give this inputs (see trapezoidal.ino)
  struct velProfile profile = genVelProfile(maxAngularSpeed, abs(degrees) * (1 / DegreesPerRev) * EncoderCountsPerRev, .2);

  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  while (millis() - startTime < profile.tf*1000 + 1000) {
    unsigned long now = millis() - startTime;
    run_motor(A, cmdA * direction);
    run_motor(B, cmdB * -direction);
    if (now > nextPDtime) {
      struct state desiredState = calculateState(now/1000.0, profile);
      Xd = desiredState.x; // Desired position
      Vd = desiredState.v; // Desired speed
      // Get current encoder counts
      encA = leftEncoderCount;
      encB = rightEncoderCount;
      // Get current speed (the 1000 converts PDdelay to seconds)
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;
      // Feed-forward values of pwm for speed based on max speed
      float pwmInA = map(Vd, 0, maxSpeedA, 0, 255); 
      float pwmInB = map(Vd, 0, maxSpeedB, 0, 255);

      // Get command values from controller (as a byte)
      cmdA = pdController(pwmInA, Vd-VA, Xd-encA, Kp[0], Kd[0]);
      cmdB = pdController(pwmInB, Vd-VB, Xd-encB, Kp[1], Kd[1]);

      // Update previous encoder counts
      prevEncA = encA;
      prevEncB = encB;

      // Set next time to update PD controller
      nextPDtime += PDdelay;
    }

    if (now > nextPrintTime) {
      // Output: time, error, velocity error, and pwm for both motors
//      Serial.println(now);
//      Serial.print("Motor A: ");
//      Serial.print(Xd - encA);
//      Serial.print("\t");
//      Serial.print(Vd - VA);
//      Serial.print("\t");
//      Serial.println(cmdA);
//      Serial.print("Motor B: ");
//      Serial.print(Xd - encB);
//      Serial.print("\t");
//      Serial.print(Vd - VB);
//      Serial.print("\t");
//      Serial.println(cmdB);
      nextPrintTime += printDelay;
    }
  }
  // Stop motors
  run_motor(A, 0);
  run_motor(B, 0);
//    Serial.println("-------------------Leaving the TURN function ---------------------");

}


struct state calculateState(float time, struct velProfile vp) {
  // TODO: Calculate the desired position and velocity at given sample time.
  struct state output;
  if (time < .2) {
    output.v = (vp.vel/vp.tf) * time;
    output.x = output.v * time / 2;
  }
  else if (time<=vp.t2){
    output.v = vp.vel;
    output.x = (vp.vel * vp.t1 / 2) + (vp.vel * (time - vp.t1));
  }
  else if (time<vp.tf){
    output.v = (-vp.vel / vp.t1) * (time - vp.t2) + vp.vel;
    output.x = (vp.vel * vp.t1 / 2) + (vp.vel * (vp.t2 - vp.t1)) + ((time - vp.t2) * (vp.vel - output.v) / 2) + ((time - vp.t2) * output.v);
  }
  else{
    output.x = (vp.vel * vp.t1) + (vp.vel * (vp.t2 - vp.t1)); //adjust because it can't be zero, it's the whole distance
    output.v = 0;
  }
  //output.x = ; // output desired position
  //output.v = ; // output desired velocity
  return output;
}

int pdController(float FF, float Verr, float Xerr, float kp, float kd) {
  // Implements Feed-forward + feedback control
  int u = FF + kp*Xerr + kd*Verr;
  int cmd = constrain(u, 0, 255);
  return cmd;
}
