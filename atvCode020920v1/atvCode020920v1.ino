#include "PPMReader.h"
#include <stdint.h>
#include <stdlib.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

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
const int enabledStepperPin = 13;

//break actuator attached to pins 8(pwm) and 9(direction)
const int breakActuatorPwm = 8;
const int breakActuatorDirection = 9;

//encoder attached to pins 10(data), 11(cs), 12(clock)
const int encoderDataPin = 10; //11
const int encoderCsPin = 11; //12
const int encoderClockPin = 12; //10

//turn lights attached to digital pwm pins 44(left) and 45(right)
#define leftTurnLight 44 
#define rightTurnLight 45

//front and rear lights attached to digital pwm pin 46
#define rearFrontLights 46 

//all other variables
int mappedOutputValue;
int mappedOutputValueEncoder;
int targetPosition;
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

const int encoderMaxLeftSteer = 664;
const int encoderMaxRightSteer = 256;

unsigned long startTime;
unsigned long endTime;


ros::NodeHandle  nh;

std_msgs::String str_msg;
std_msgs::Float32MultiArray axes;
std_msgs::Int32MultiArray buttons;
std_msgs::Float32 float1;
ros::Publisher chatter("chatter", &str_msg);

void joydata ( const sensor_msgs::Joy& joy)
{
  axes.data = joy.axes;
  buttons.data = joy.buttons;
}

ros::Subscriber<sensor_msgs::Joy> sub1("joy", joydata);

void setup()
{
    nh.initNode();
    nh.subscribe(sub1);
    //nh.advertise(chatter);
    
    Serial.begin(57600);

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
    currentMillisLeft, currentMillisRight = millis();
    
    nh.spinOnce();

    ReadSSI();

    read_rc();

    turn_wheels(ch[0], ch[7]);

    outputGasSignal(ch[2], ch[7]);

    pushBreak(ch[4], ch[7]);

    //leftTurningLights(ch[6], ch[7]);

    //rightTurningLights(ch[6], ch[7]);

    //turnFrontAndRearLightsOn(ch[5], ch[7]);

    //turnBackwardMovingLights(ch[2], ch[7]);

}


/*
 See funktsioon loeb andmed ressiiverist,
 muudab nende väärtust 0 ... 1000ni ja kirjutab neid massiivi ch[]
*/
void read_rc()
{
    while (ppmReader.get(count) != 0)
    {
        ch[count] = map(ppmReader.get(count), 1000, 2000, 0, 1000);
        count++;
    }
    count = 0;
}

/*
 See funktsioon võtab andmed ch massiivi 4 elemendist
 ja seejärel kontrollib. Kui väärtus on rohkem, kui 10
 siis aktuaatori draivi DIR pinnile saadetakse LOW signaal
 ja aktuaatori draivi PWM pinnile saadetakse HIGH signaal.
 Kui väärtus on vöhem, kui 10 siis aktuaatori draivi
 DIR pinnile saadetakse HIGH signaalja aktuaatori draivi
 PWM pinnile saadetakse HIGH signaal.
*/
void pushBreak(int channel4, int controlChannel)
{
    if (controlChannel < 500) {
      channel4 = map(buttons.data[0], 0, 1, 0, 1000);
    }
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

/*
 See funktsioon võtab andmed ch massiivi 0 elemendist
 ja seejärel kontrollib. Kui väärtus on vähem, kui 485
 siis samm mootori draivi DIR pinnile saadetakse HIGH signaal
 ja siis tsüklis iga 80 ms saadetakse pulse (ehk HIGH ja LOW
 signaalid üks teise järel) kuni tsükkel jõuab 10000ni. Kui
 väärtus on rohkem, kui 515 siis samm mootori draivi DIR
 pinnile saadetakse LOW signaal ja ülejöönud on sama.
*/

void turn_wheels(int channel0, int controlChannel)

{
    if (controlChannel < 500) {
      channel0 = map(axes.data[0], -1, 1, 0, 1000);
    }else{
      channel0 = map(channel0, 0, 1000, 1000, 0);
    }
    
    targetPosition = map(channel0, 0, 1000, encoderMaxRightSteer, encoderMaxLeftSteer);

    if (targetPosition > mappedOutputValueEncoder + 10)
    {
        digitalWrite(enabledStepperPin, LOW);
        digitalWrite(turningStepperDirection, HIGH);
        digitalWrite(turningStepperPulse, HIGH);  
        digitalWrite(turningStepperPulse, LOW);
    }

    else if (targetPosition < mappedOutputValueEncoder - 10)
    {
        digitalWrite(enabledStepperPin, LOW);
        digitalWrite(turningStepperDirection, LOW);
        digitalWrite(turningStepperPulse, HIGH);
        digitalWrite(turningStepperPulse, LOW);
    }
    else
    {
        digitalWrite(enabledStepperPin, HIGH);
    }
}

/*
 See funktsioon võtab andmed ch massiivi 2 elemendist
 ja seejärel kontrollib. Kui väärtus on rohkem kui 515
 siis väärtus muudetakse diaposoonist 515 ... 1000
 diaposoonini 45 ... 214, seejärel muudetud väärtus
 saadetakse signaalina läbi arduino analoog pinni
 kelly kontrollerile ja samal ajal saadetakse ka HIGH
 signaal kellysse mootori suuna muutmiseks. Kui väärtus
 on vähem kui 485 siis toimuvad umbes sama protsessid,
 vaid HIGH asemeel mootori suuna muutmiseks saadetakse LOW
 signaal. Kui signaal on vahemikus 485 .. 515 siis saadetakse
 väärtus 45 kellysse > mootori, see vool laseb mootoril olla
 töötavas seisundis. Kui mingi hetk mootor kaob selle voolu, siis
 peab tegema terve süsteemi restart.
 */
void outputGasSignal(int channel2, int controlChannel)
{

    if (controlChannel < 500) {
      channel2 = map(axes.data[1], -1, 1, 0, 1000);
    }
    
    if (channel2 > 515)
    {
        mappedOutputValue = map(channel2, 515, 1000, 45, 80);
        digitalWrite(gasMotorDirection, HIGH);
        analogWrite(gasMotorPwm, mappedOutputValue);
    }
    else if (channel2 < 485)
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

/*
See funktsioon võtab andmed ch massiivi 6 elemendist
ja seejärel kontrollib. Kui väärtus on vähem kui 400
siis boolean muutujasse rightLightOn muudab true'ks ja
seejörel kolm sekundit saadetakse HIGH signaali parempoolsete
suunatuledele. Kui taimer jõuab kolme sekundini siis boolean rightLightOn
muudab false'ks ja parempoolsete suunatuledele saadetakse LOW signaal.
*/
void rightTurningLights (int channel6, int controlChannel)
{
    if (controlChannel < 500) {
      //channel6 = map(buttons.data[?], -1, 1, 0, 1000);
    }
    
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

/*
See funktsioon võtab andmed ch massiivi 6 elemendist
ja seejärel kontrollib. Kui väärtus on rohkem kui 600
siis boolean muutujasse leftLightOn muudab true'ks ja
seejörel kolm sekundit saadetakse HIGH signaali vasakpoolsete
suunatuledele. Kui taimer jõuab kolme sekundini siis boolean leftLightOn
muudab false'ks ja vasakpoolsete suunatuledele saadetakse LOW signaal.
*/
void leftTurningLights (int channel6, int controlChannel)
{
    if (controlChannel < 500) {
      //channel6 = map(buttons.data[?], -1, 1, 0, 1000);
    }
    
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

/*See funktsioon võtab andmed ch massiivi 5 elemendist
ja seejärel kontrollib. Kui väärtus on rohkem kui 988
siis taga- ja esituled lülitakse sisse saadetes rearFrontLights
pinnile HIGH signaali. Vastasel juhul saadetakse LOW signaal.
*/
void turnFrontAndRearLightsOn(int channel5, int controlChannel)
{
    if (controlChannel < 500) {
      //channel5 = map(buttons.data[?], -1, 1, 0, 1000);
    }
  
    if (channel5 > 988)
    {
        analogWrite(rearFrontLights, 255);
    }
    else
    {
        analogWrite(rearFrontLights, 0);
    }
}
