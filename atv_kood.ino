#include "PPMReader.h"
#include <AccelStepper.h>

//Using Arduino Mega
//pins in use Digital - [2, 4, 5, 6, 7, 8, 9, 10, 11, 12], Analog - [0, 1, 2] 5V, GND
//free pins Digital - [0, 1, 3, 13], Analog - [3, 4, 5], 3.3V, GND, GND, VIN

//receiver attached to pin2
PPMReader ppmReader(2, 0, false);
static int count;
int ch[9] = {0, 0, 500, 0, 0, 0, 491, 0, 0};

//gas motor attached to pins 4(direction) and 5(pwm)
const int gasMotorDirection = 4;
const int gasMotorPwm = 5;
const int motorInterfaceType = 1;

//turning stepper motor attached to pins 6(direcrtion) and 7(pulse)
const int turningStepperDirection = 6;
const int turningStepperPulse = 7;

//break actuator attached to pins 8(pwm) and 9(direction)
const int breakActuatorPwm = 8;
const int breakActuatorDirection = 9;

//encoder attached to pins 10(data), 11(cs), 12(clock)
const int encoderDataPin = 10; //11
const int encoderCsPin = 11; //12
const int encoderClockPin = 12; //10

//turn lights attached to digital pwm pins 44(left) and 45(right)
#define leftTurnLight 44 //see muutub PWM pinniks
#define rightTurnLight 45 //see muutub PWM pinniks

//front and rear lights attached to digital pwm pin 46
#define rearFrontLights 46 //see muutub PWM pinniks

//all other variables
int mappedOutputValue;
int mappedOutputValueEncoder;
int delay_Micros = 80;
int encoder_reading;
long currentMicros = 0;
long previousMicros = 0;


unsigned static long previousMillisLeft = 0;
unsigned static long previousMillisRight = 0;
unsigned long currentMillisLeft = 0;
unsigned long currentMillisRight = 0;
const long turnLightsInterval = 3000;
boolean leftLightOn = false;
boolean rightLightOn = false;

float maxLeftTurn = 20.0;
float maxRightTurn = 100.0;
float centerPosition = 60.0;
float currentWheelPosition;

float leftSteer;
float rightSteer;
float maxLeftSteer = 240;
float maxRightSteer = 123;

//AccelStepper stepper = AccelStepper(motorInterfaceType, turningStepperPulse, turningStepperDirection);

unsigned long startTime;
unsigned long endTime;

void setup()
{
    delay(1000);
    Serial.begin(115200);
    pinMode (breakActuatorDirection, OUTPUT);
    pinMode (breakActuatorPwm, OUTPUT);
    pinMode (turningStepperPulse, OUTPUT);
    pinMode (turningStepperDirection, OUTPUT);
    pinMode(gasMotorPwm, OUTPUT);
    pinMode(gasMotorDirection, OUTPUT);

    pinMode(encoderCsPin, OUTPUT);
    pinMode(encoderClockPin, OUTPUT);
    pinMode(encoderDataPin, INPUT);

    digitalWrite(encoderClockPin, HIGH);
    digitalWrite(encoderCsPin, LOW);
}

void loop()
{
    //startTime = micros();

    ReadSSI();
    read_rc();

    turn_wheels(ch[0]);

    //endTime = micros();
    //startTime = endTime - startTime;
    //Serial.println(startTime);

    //outputGasSignal(ch[2]);

    //pushBreak(ch[4]);

    //leftTurningLights(ch[6]);

    //rightTurningLights(ch[6]);

    //turnFrontAndRearLightsOn(ch[5]);

    //turnBackwardMovingLights(ch[2]);

    //Serial.print(ch[0]); Serial.print("\t");
    //Serial.print(ch[2]); Serial.print("\t");
    //Serial.print(ch[3]); Serial.print("\t");
    //Serial.print(ch[4]); Serial.print("\t");
    //Serial.print(ch[5]); Serial.print("\t");
    //Serial.print(ch[6]); Serial.print("\t");
    //Serial.print(ch[5]); Serial.print("\t");
    //Serial.print(currentWheelPosition); Serial.print("\t");
    //Serial.print(ch[8]); Serial.print("\n");
}

void read_rc()
{
    while (ppmReader.get(count) != 0)
    {
        ch[count] = map(ppmReader.get(count), 1000, 2000, 0, 1000);
        count++;
    }
    count = 0;
}

void pushBreak(int channel4)
{
    if (channel4 > 10)
    {
        digitalWrite(breakActuatorDirection, LOW);
        digitalWrite(breakActuatorPwm, HIGH);
    }
    else if (channel4 < 10)
    {
        digitalWrite(breakActuatorDirection, HIGH);
        digitalWrite(breakActuatorPwm, HIGH);
    }
}

void turn_wheels(int channel0)
{
    if (channel0 < 485)
    {
        digitalWrite(turningStepperDirection, HIGH);
        digitalWrite(turningStepperPulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turningStepperPulse, LOW);

    }

    if (channel0 > 515)
    {
        digitalWrite(turningStepperDirection, LOW);
        digitalWrite(turningStepperPulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turningStepperPulse, LOW);

    }
}

void outputGasSignal(int channel2)
{
    if (channel2 > 515)
    {
        mappedOutputValue = map(channel2, 515, 1000, 45, 214);
        digitalWrite(gasMotorDirection, HIGH);
        analogWrite(gasMotorPwm, mappedOutputValue);
    }
    else if (channel2 < 485 )
    {
        mappedOutputValue = map(channel2, 485, 0, 45, 214);
        digitalWrite(gasMotorDirection, LOW);
        analogWrite(gasMotorPwm, mappedOutputValue);
    }
    else
    {
        analogWrite(gasMotorPwm, 45);
    }

}

void rightTurningLights (int channel6)
{
    if (channel6 < 400 )
    {
        previousMillisRight = currentMillisRight;
        rightLightOn = true;
    }
    if (currentMillisRight > 3000 && currentMillisRight - previousMillisRight <= turnLightsInterval && leftLightOn == false)
    {
        analogWrite(rightTurnLight, 255);
    }
    else if (currentMillisRight - previousMillisRight > turnLightsInterval)
    {
        rightLightOn = false;
        analogWrite(rightTurnLight, 0);
    }
}

void leftTurningLights (int channel6)
{
    if (channel6 > 600 )
    {
        previousMillisLeft = currentMillisLeft;
        leftLightOn = true;
    }
    if (currentMillisLeft > 3000 && currentMillisLeft - previousMillisLeft <= turnLightsInterval && rightLightOn == false)
    {
        analogWrite(leftTurnLight, 255);
    }
    else if ( currentMillisLeft - previousMillisLeft > turnLightsInterval)
    {
        leftLightOn = false;
        analogWrite(leftTurnLight, 0);
    }
}

void turnFrontAndRearLightsOn(int channel5)
{
    if (channel5 > 988)
    {
        analogWrite(rearFrontLights, 255);
    }
    else
    {
        analogWrite(rearFrontLights, 0);
    }
}