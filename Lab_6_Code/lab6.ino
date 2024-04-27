#include <Servo.h>
#include <IRremote.h>

#define IR_RECEIVE_PIN 11
#define Button1IR 22
#define Button2IR 25

Servo myservo;
int Echo_Pin = A0;
int Trig_Pin = A1;
#define Lpwm_pin 5
#define Rpwm_pin 6
int IN1 = 2;
int IN2 = 4;
int IN3 = 7;
int IN4 = 8;
int play = false;
const int buzzer = 9;

volatile int D_mix;
volatile int D_mid;
volatile int D_max;
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;
volatile int Right_IR_Value;
volatile int Left_IR_Value;

float Kp = 1.5;
float Kd = 0.5;
float last_error = 0;
unsigned char speed_val = 0;

float checkdistance()
{
    digitalWrite(Trig_Pin, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig_Pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig_Pin, LOW);
    float distance = pulseIn(Echo_Pin, HIGH) / 58.00;
    delay(10);
    return distance;
}

void move_with_PD()
{
    float error = Front_Distance;
    float derivative = error - last_error;
    last_error = error;

    speed_val = Kp * error + Kd * derivative;

    if (speed_val < 0)
        speed_val = 0;
    else if (speed_val > 255)
        speed_val = 255;

    go_forward(speed_val);
}

void Ultrasonic_obstacle_avoidance()
{
    Front_Distance = checkdistance();
    if (Front_Distance == 0)
    {
    }
    else if ((Front_Distance < 40) && (Front_Distance > 0))
    {
        stopp();
        tone(buzzer, 1000);
        delay(1000);
        noTone(buzzer);
        delay(100);
        myservo.write(180);
        delay(500);
        Left_Distance = checkdistance();
        delay(100);
        myservo.write(0);
        delay(500);
        Right_Distance = checkdistance();
        delay(100);
        if (Left_Distance > Right_Distance)
        {
            rotate_left(150);
            myservo.write(90);
            delay(300);
        }
        else
        {
            rotate_right(150);
            myservo.write(90);
            delay(300);
        }
    }
    else
    {
        move_with_PD();
    }
}

void Obstacle_Avoidance_Main()
{
    Ultrasonic_obstacle_avoidance();
}

void setup()
{
    myservo.attach(A2);
    Serial.begin(9600);
    D_mix = 10;
    D_mid = 20;
    D_max = 100;
    Front_Distance = 0;
    Left_Distance = 0;
    Right_Distance = 0;
    myservo.write(90);
    pinMode(Echo_Pin, INPUT);
    pinMode(Trig_Pin, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(Lpwm_pin, OUTPUT);
    pinMode(Rpwm_pin, OUTPUT);
    IrReceiver.begin(IR_RECEIVE_PIN);
    pinMode(buzzer, OUTPUT);
    Serial.println("Setup Finished");
}

void loop()
{
    if (IrReceiver.decode())
    {
        int command = IrReceiver.decodedIRData.command;
        switch (command)
        {
        case Button1IR:
            play = true;
            break;
        case Button2IR:
            play = false;
            break;
        default:
            Serial.println("Button not recognized");
            break;
        }
        IrReceiver.resume();
    }

    if (play)
    {
        Obstacle_Avoidance_Main();
    }
    else
    {
        stopp();
    }
}

void go_forward(unsigned char speed_val)
{
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(Lpwm_pin, speed_val);
    analogWrite(Rpwm_pin, speed_val);
}

void rotate_left(unsigned char speed_val)
{
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(Lpwm_pin, speed_val);
    analogWrite(Rpwm_pin, speed_val);
}

void rotate_right(unsigned char speed_val)
{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(Lpwm_pin, speed_val);
    analogWrite(Rpwm_pin, speed_val);
}

void stopp()
{
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
}
