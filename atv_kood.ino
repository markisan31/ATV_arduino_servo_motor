#include "PPMReader.h"
#include <AccelStepper.h>

#define leftTurnLight 44
#define rightTurnLight 45
#define rearFrontLights 46

PPMReader ppmReader(2, 0, false);

static int count;
int ch[9] = {0, 0, 500, 0, 0, 0, 491, 0, 0};

const int gasMotorDirection = 4;
const int gasMotorPwm = 5;
const int motorInterfaceType = 1;

const int turningStepperDirection = 6;
const int turningStepperPulse = 7;
const int enabledStepperPin = 13;

const int breakActuatorPwm = 8;
const int breakActuatorDirection = 9;

const int encoderDataPin = 10;
const int encoderCsPin = 11;
const int encoderClockPin = 12;

int mappedOutputValue;
int mappedOutputValueEncoder;
int targetPosition;
int delay_Micros = 80;
int encoder_reading;
long currentMicros = 0;
long previousMicros = 0;

unsigned static long previousMillisLeft = 0;
unsigned static long previousMillisRight = 0;

unsigned long currentMillisLeft = 0;
unsigned long currentMillisRight = 0;

const long turnLightsInterval = 3000;
const int encoderMaxLeftSteer = 664;
const int encoderMaxRightSteer = 256;

unsigned long startTime;
unsigned long endTime;

boolean leftLightOn = false;
boolean rightLightOn = false;

void setup()
{
    Serial.begin(115200);

    pinMode (breakActuatorDirection, OUTPUT);
    pinMode (breakActuatorPwm, OUTPUT);

    pinMode (turningStepperPulse, OUTPUT);
    pinMode (turningStepperDirection, OUTPUT);
    pinMode (enabledStepperPin, OUTPUT);

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

    outputGasSignal(ch[2]);

    pushBreak(ch[4]);

    //leftTurningLights(ch[6]);

    //rightTurningLights(ch[6]);

    //turnFrontAndRearLightsOn(ch[5]);

    //turnBackwardMovingLights(ch[2]);


    /* debuggimiseks
    Serial.print(ch[0]); Serial.print("\t");
    Serial.print(ch[2]); Serial.print("\t");
    Serial.print(ch[3]); Serial.print("\t");
    Serial.print(ch[4]); Serial.print("\t");
    Serial.print(ch[5]); Serial.print("\t");
    Serial.print(ch[6]); Serial.print("\t");
    Serial.print(ch[5]); Serial.print("\t");
    Serial.print(currentWheelPosition); Serial.print("\t");
    Serial.print(ch[8]); Serial.print("\n");
    */
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
    channel0 = map(channel0, 0, 1000, 1000, 0);
    targetPosition = map(channel0, 0, 1000, encoderMaxRightSteer, encoderMaxLeftSteer);

    /* debuggimiseks
    Serial.print("Enkooder: \t"); Serial.print(mappedOutputValueEncoder);
    Serial.print("Pult: \t"); Serial.println(targetPosition);
    */

    if (targetPosition > mappedOutputValueEncoder + 10)
    {
        digitalWrite(enabledStepperPin, LOW);
        digitalWrite(turningStepperDirection, HIGH);
        digitalWrite(turningStepperPulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turningStepperPulse, LOW);
    }

    else if (targetPosition < mappedOutputValueEncoder - 10)
    {
        digitalWrite(enabledStepperPin, LOW);
        digitalWrite(turningStepperDirection, LOW);
        digitalWrite(turningStepperPulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turningStepperPulse, LOW);
    }
    else
    {
        digitalWrite(enabledStepperPin, HIGH);
    }
}

void outputGasSignal(int channel2)
{
    if (channel2 > 515)
    {
        mappedOutputValue = map(channel2, 515, 1000, 45, 80);
        digitalWrite(gasMotorDirection, HIGH);
        analogWrite(gasMotorPwm, mappedOutputValue);
    }
    else if (channel2 < 485 )
    {
        mappedOutputValue = map(channel2, 485, 0, 45, 80);
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
