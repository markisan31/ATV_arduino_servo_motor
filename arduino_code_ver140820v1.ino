#include "PPMReader.h"

//pins in use Digital - [2, 4, 5, 6, 7, 8, 9, 10, 11, 12], Analog - [0, 1, 2] 5V, GND
//free pins Digital - [0, 1, 3, 13], Analog - [3, 4, 5], 3.3V, GND, GND, VIN

//receiver attached to pin2
PPMReader ppmReader(2, 0, false);
static int count;
int ch[9] = {500, 500, 500, 500, 0, 491, 491, 0, 0};

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
const int encoderDataPin = 10;
const int encoderCsPin = 11;
const int encoderClockPin = 12;

//turn lights attached to analog pins 0(left) and 1(right)
#define leftTurnLight A0
#define rightTurnLight A1

//front and rear lights attached to analog pin 2
#define rearFrontLights A2

//all other variables
int mappedOutputValue;
int mappedOutputValueEncoder;
int delay_Micros = 80;
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

void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode (breakActuatorDirection, OUTPUT);
  pinMode (breakActuatorPwm, OUTPUT);
  pinMode (turningStepperPulse, OUTPUT);
  pinMode (turningStepperDirection, OUTPUT);
  pinMode(gasMotorPwm, OUTPUT);
  analogWrite(gasMotorPwm, 10);
  pinMode(gasMotorDirection, OUTPUT);
}

void loop() {

  currentMillisLeft = millis();
  currentMillisRight = millis();

  encoder_data();

  read_rc();

  turn_wheels(ch[0]);

  outputGasSignal(ch[2]);

  pushBreak(ch[4]);

  leftTurningLights(ch[6]);

  rightTurningLights(ch[6]);

//  turnBackwardMovingLights(ch[2]);

  

    Serial.print(ch[0]); Serial.print("\t");
    Serial.print(ch[2]); Serial.print("\t");
    Serial.print(ch[3]); Serial.print("\t");
    Serial.print(ch[4]); Serial.print("\t");
    Serial.print(ch[5]); Serial.print("\t");
    Serial.print(ch[6]); Serial.print("\t");
    Serial.print(ch[5]); Serial.print("\t");
    Serial.print(ch[8]); Serial.print("\n");
  turnFrontAndRearLightsOn(ch[5]);
}


void read_rc() {
  while (ppmReader.get(count) != 0) {
    ch[count] = map(ppmReader.get(count), 1000, 2000, 0, 1000);
    count++;
  }
  count = 0;
}

void pushBreak(int channel4) {
  if (channel4 > 10) {
    digitalWrite(breakActuatorDirection, LOW);
    digitalWrite(breakActuatorPwm, HIGH);
  }
  if (channel4 < 10) {
    digitalWrite(breakActuatorDirection, HIGH);
    digitalWrite(breakActuatorPwm, HIGH);
  }
}

void turn_wheels(int channel0) {
  if (channel0 < 485) {
    Serial.print("Hello");
    digitalWrite(turningStepperDirection, HIGH);
//    if (currentWheelPosition < maxLeftTurn) {
      for (int i = 0; i < 10000; i += 1) {
        currentMicros = micros();
        if (currentMicros - previousMicros >= delay_Micros) {
          previousMicros = currentMicros;
          digitalWrite(turningStepperPulse, HIGH);
          digitalWrite(turningStepperPulse, LOW);
        }
      }
//    } 
  }
 if (channel0 > 515) {
  Serial.print("Hello1");
  digitalWrite(turningStepperDirection, LOW);
//  if (currentWheelPosition >= maxRightTurn){
    for (int i = 0; i < 10000; i += 1) {
      currentMicros = micros();
      if (currentMicros - previousMicros >= delay_Micros) {
        previousMicros = currentMicros;
        digitalWrite(turningStepperPulse, HIGH);
        digitalWrite(turningStepperPulse, LOW);
      }
    }
//  }
 }
 if (channel0 < 515 && channel0 > 485) {

}
}

void outputGasSignal(int channel2) {
  if (channel2 > 500 + 10) {
    mappedOutputValue = map(channel2, 500, 1000, 45, 214);
    digitalWrite(gasMotorDirection, HIGH);
    analogWrite(gasMotorPwm, mappedOutputValue);
  }
  else if (channel2 < 500 - 10 ) {
    mappedOutputValue = map(channel2, 490, 0, 45, 214);
    digitalWrite(gasMotorDirection, LOW);
    analogWrite(gasMotorPwm, mappedOutputValue);

  }  else {
    analogWrite(gasMotorPwm, 45);
    digitalWrite(gasMotorDirection, HIGH);
  }

}

void encoder_data() {
  digitalWrite(encoderCsPin, HIGH);
  digitalWrite(encoderCsPin, LOW);
  int pos = 0;
  //  int mappedOutputValue = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(encoderClockPin, LOW);
    digitalWrite(encoderClockPin, HIGH);

    byte b = digitalRead(encoderDataPin) == HIGH ? 1 : 0;
    pos += b * pow(2, 10 - (i + 1));
    mappedOutputValueEncoder = map(pos, 0, 1020, 0, 359);
    currentWheelPosition = mappedOutputValueEncoder;
  }
  for (int i = 0; i < 6; i++) {
    digitalWrite(encoderClockPin, LOW);
    digitalWrite(encoderClockPin, HIGH);
  }
  digitalWrite(encoderClockPin, LOW);
  digitalWrite(encoderClockPin, HIGH);
}

void rightTurningLights (int channel6) {
  if (channel6 < 400 ) {
    previousMillisRight = currentMillisRight;
    rightLightOn = true;
  }
  if (currentMillisRight > 3000 && currentMillisRight - previousMillisRight <= turnLightsInterval && leftLightOn == false) {
    Serial.print("HelloRight");
    Serial.print("\n");
    digitalWrite(rightTurnLight, HIGH);
    //      rightLightOn = true;
  } else if (currentMillisRight - previousMillisRight > turnLightsInterval) {
    rightLightOn = false;
    digitalWrite(rightTurnLight, LOW);
  }
}

void leftTurningLights (int channel6) {
  if (channel6 > 600 ) {
    previousMillisLeft = currentMillisLeft;
    leftLightOn = true;
  }
  if (currentMillisLeft > 3000 && currentMillisLeft - previousMillisLeft <= turnLightsInterval && rightLightOn == false) {
    Serial.print("HelloLeft");
    Serial.print("\n");
    digitalWrite(leftTurnLight, HIGH);
    //      leftLightOn = true;
  } else if ( currentMillisLeft - previousMillisLeft > turnLightsInterval) {
    leftLightOn = false;
    digitalWrite(leftTurnLight, LOW);
  }

}

void turnFrontAndRearLightsOn(int channel5) {
  if (channel5 > 988) {
    digitalWrite(rearFrontLights, HIGH);
  } else {
    digitalWrite(rearFrontLights, LOW);
  }
}

void turnBackwardMovingLights(int channel2) {
  if (channel2 < 480) {
    digitalWrite(leftTurnLight, HIGH);
    digitalWrite(rightTurnLight, HIGH);
  } else {
    digitalWrite(leftTurnLight, LOW);
    digitalWrite(rightTurnLight, LOW);
  }
  
}
