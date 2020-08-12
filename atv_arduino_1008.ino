#include <AccelStepper.h>
#include "encoder_data.hpp"

#define pwmPin 5
#define reverse 4
#define motorInterfaceType 1

#undef c

//specifing arrays and variables to store values
unsigned long int a, b, c;
int x[15], ch1[15], ch[7],  i;

const int motor2 = 6;// defines pin 7 as connected to the motor
const int motor_speed = 7;
const int pwm_1 = 8;
const int dir_1 = 9;

long stepperSetMaxSpeed = 100000000;
long stepperSetAcceleration = 50000000;
int mappedOutputValue;

const int PIN_CS = 12;
const int PIN_CLOCK = 13;
const int PIN_DATA = 11;

const int stepperDefaultPosition = 0;

float leftSteer;
float rightSteer;
float maxLeftSteer = -50;
float maxRightSteer = 50;


int mappedOutputValueEncoder;

AccelStepper stepper = AccelStepper(motorInterfaceType, motor_speed, motor2);

void setup()
{
    Serial.begin(115200);
    stepper.setMaxSpeed(stepperSetMaxSpeed);
    stepper.setAcceleration(stepperSetAcceleration);

    pinMode (dir_1, OUTPUT);
    pinMode (pwm_1, OUTPUT);
    pinMode (motor_speed, OUTPUT);
    pinMode (motor2, OUTPUT);
    pinMode(2, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);
    pinMode(reverse, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING);
    // enabling interrupt at pin 2

    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_CLOCK, OUTPUT);
    pinMode(PIN_DATA, INPUT);
    digitalWrite(PIN_CLOCK, HIGH);
    digitalWrite(PIN_CS, LOW);
}

void loop()
{
    for (i = 0; i < 7; i++)
    {
        ch[i] = 0;
    }

    encoder_data::encoder_data();
    //Serial.println(mappedOutputValueEncoder);

    read_rc();

    outputGasSignal(ch[2]);
    pushBreak(ch[5]);
    turn(ch[4]);

    //Serial.print(ch[1]);Serial.print("\t");
    //Serial.print(ch[2]);Serial.print("\t");
    //Serial.print(ch[3]);Serial.print("\t");
    //Serial.print(ch[4]);Serial.print("\t");
    //Serial.print(ch[5]);Serial.print("\t");
    //Serial.print(ch[6]);Serial.print("\n");
}

void read_me()
{
    //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
    //this code gives channel values from 0-1000 values
    //    -: ABHILASH :-    //
    a = micros(); //store time value a when pin value falling
    c = a - b;  //calculating time inbetween two peaks
    b = a;      //
    x[i] = c;   //storing 15 value in array
    i = i + 1;
    if (i == 15)
    {
        for (int j = 0; j < 15; j++)
        {
            ch1[j] = x[j];
        }
        i = 0;
    }
}//copy store all values from temporary array another array after 15 reading


void read_rc()
{
    int i, j, k = 0;
    for (k = 14; k > -1; k--)
    {
        if (ch1[k] > 5000)
        {
            j = k;
        }
    }  //detecting separation space 10000us in that another array
    for (i = 1; i <= 6; i++)
    {
        ch[i] = max(0, min(1000, (ch1[i + j] - 1000)));
    }
}
//assign 6 channel values after separation space

void turn(int channel4)
{
    if (channel4 < 485)
    {
        if (leftSteer < maxLeftSteer)
        {
            leftSteer = leftSteer + 10;
            stepper.move(leftSteer);
            stepper.run();
        }
        else if (leftSteer >= maxLeftSteer)
        {
            leftSteer = maxLeftSteer;
            stepper.move(leftSteer);
            stepper.run();
        }
        //stepper.move(10);
        //stepper.move(leftSteer)
        //stepper.run();
    }
    if (channel4 > 515)
    {
        if (rightSteer < maxRightSteer)
        {
            rightSteer = rightSteer + 10;
            stepper.move(rightSteer);
            stepper.run();
        }
        else if (rightSteer >= maxRightSteer)
        {
            rightSteer = maxRightSteer;
            stepper.move(rightSteer);
            stepper.run();
        }
        //stepper.move(-10);
        //stepper.run();
    }

    //TODO: FIX BELOW LINES VVV ACCORDING TO TOP LINES ^^^
    if (channel4 < 515 && channel4 > 485 || mappedOutputValueEncoder > 85 && mappedOutputValueEncoder < 95)
    {
        //stepper.moveTo(0);
        //stepper.run();
        //delay(2);
        stepper.moveTo(stepperDefaultPosition);
        stepper.run();
        delay(2);
    }
}

void pushBreak(int channel5)
{
    if (channel5 > 10)
    {
        digitalWrite(dir_1, LOW);
        digitalWrite(pwm_1, HIGH);
    }
    if (channel5 < 10)
    {
        digitalWrite(dir_1, HIGH);
        digitalWrite(pwm_1, HIGH);
    }
}

void outputGasSignal(int speedInput)
{
    if (speedInput > 500 + 50)
    {
        mappedOutputValue = map(speedInput, 500, 1000, 45, 214);
        //Serial.print(mappedOutputValue);
        //Serial.print("\n");
        digitalWrite(reverse, LOW);
        analogWrite(pwmPin, mappedOutputValue);
    }
    else if (speedInput < 500 - 50 )
    {
        mappedOutputValue = map(speedInput, 490, 0, 45, 214);
        //Serial.print(mappedOutputValue);
        //Serial.print("\n");
        digitalWrite(reverse, HIGH);
        analogWrite(pwmPin, mappedOutputValue);
    }
    else
    {
        //Serial.print(mappedOutputValue);
        //Serial.print("\n");
        analogWrite(pwmPin, 15);
        digitalWrite(reverse, LOW);
    }

}
