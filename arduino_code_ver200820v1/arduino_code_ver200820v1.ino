#include "PPMReader.h"

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
const int encoderDataPin = 10;
const int encoderCsPin = 11;
const int encoderClockPin = 12;

//turn lights attached to analog pins 0(left) and 1(right)
#define leftTurnLight 44 //see muutub PWM pinniks
#define rightTurnLight 45 //see muutub PWM pinniks

//front and rear lights attached to analog pin 2
#define rearFrontLights 46 //see muutub PWM pinniks

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

//  leftTurningLights(ch[6]);

//  rightTurningLights(ch[6]);

//  turnBackwardMovingLights(ch[2]);

//    Serial.print(ch[0]); Serial.print("\t");
//    Serial.print(ch[2]); Serial.print("\t");
//    Serial.print(ch[3]); Serial.print("\t");
//    Serial.print(ch[4]); Serial.print("\t");
//    Serial.print(ch[5]); Serial.print("\t");
//    Serial.print(ch[6]); Serial.print("\t");
//    Serial.print(ch[5]); Serial.print("\t");
//    Serial.print(currentWheelPosition); Serial.print("\t");
//    Serial.print(ch[8]); Serial.print("\n");
//  turnFrontAndRearLightsOn(ch[5]);
}


/*
 See funktsioon loeb andmed ressiiverist, 
 muudab nende väärtust 0 ... 1000ni ja kirjutab neid massiivi ch[] 
*/
void read_rc() { 
  while (ppmReader.get(count) != 0) {
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
void pushBreak(int channel4) { 
  if (channel4 > 10) {
    digitalWrite(breakActuatorDirection, LOW);
    digitalWrite(breakActuatorPwm, HIGH);
  }
  else if (channel4 < 10) {
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
void turn_wheels(int channel0) {
  if (channel0 < 485) {
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
// if (channel0 < 515 && channel0 > 485) {
//
//}
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
void outputGasSignal(int channel2) {
  if (channel2 > 515) {
    mappedOutputValue = map(channel2, 515, 1000, 45, 214);
    digitalWrite(gasMotorDirection, HIGH);
    analogWrite(gasMotorPwm, mappedOutputValue);
  }
  else if (channel2 < 485 ) {
    mappedOutputValue = map(channel2, 485, 0, 45, 214);
    digitalWrite(gasMotorDirection, LOW);
    analogWrite(gasMotorPwm, mappedOutputValue);

  }  else {
    analogWrite(gasMotorPwm, 45);
  }

}

/*
See funktsioon saadab encoderi CS ja CLK pinnidele 
HIGH ja LOW signaalid mingis jörjekorras ( seda ma täpselt ei 
oska seletada, kood on võetud internetist, peaks lugema kuidas
enkooder töötab) ja siis enkooderist tulevad väärtused läbi D0 (data)
pinni mis programm muudab diapasoonist 0 .. 1020 diapasoonini 0 .. 359 
ja see väärtus kirjutatakse globaalse muutujasse currentWheelPosition 
*/
void encoder_data() {
  digitalWrite(encoderCsPin, HIGH);
  digitalWrite(encoderCsPin, LOW);
  int pos = 0;
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

/*
See funktsioon võtab andmed ch massiivi 6 elemendist
ja seejärel kontrollib. Kui väärtus on vähem kui 400
siis boolean muutujasse rightLightOn muudab true'ks ja 
seejörel kolm sekundit saadetakse HIGH signaali parempoolsete
suunatuledele. Kui taimer jõuab kolme sekundini siis boolean rightLightOn
muudab false'ks ja parempoolsete suunatuledele saadetakse LOW signaal.
*/
void rightTurningLights (int channel6) {
  if (channel6 < 400 ) {
    previousMillisRight = currentMillisRight;
    rightLightOn = true;
  }
  if (currentMillisRight > 3000 && currentMillisRight - previousMillisRight <= turnLightsInterval && leftLightOn == false) {
    analogWrite(rightTurnLight, 255);
  } else if (currentMillisRight - previousMillisRight > turnLightsInterval) {
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
void leftTurningLights (int channel6) {
  if (channel6 > 600 ) {
    previousMillisLeft = currentMillisLeft;
    leftLightOn = true;
  }
  if (currentMillisLeft > 3000 && currentMillisLeft - previousMillisLeft <= turnLightsInterval && rightLightOn == false) {
    analogWrite(leftTurnLight, 255);
  } else if ( currentMillisLeft - previousMillisLeft > turnLightsInterval) {
    leftLightOn = false;
    analogWrite(leftTurnLight, 0);
  }

}

/*See funktsioon võtab andmed ch massiivi 5 elemendist
ja seejärel kontrollib. Kui väärtus on rohkem kui 988
siis taga- ja esituled lülitakse sisse saadetes rearFrontLights
pinnile HIGH signaali. Vastasel juhul saadetakse LOW signaal.
*/
void turnFrontAndRearLightsOn(int channel5) {
  if (channel5 > 988) {
    analogWrite(rearFrontLights, 255);
  } else {
    analogWrite(rearFrontLights, 0);
  }
}

//void turnBackwardMovingLights(int channel2) {
//  if (channel2 < 480) {
//    digitalWrite(leftTurnLight, HIGH);
//    digitalWrite(rightTurnLight, HIGH);
//  } else {
//    digitalWrite(leftTurnLight, LOW);
//    digitalWrite(rightTurnLight, LOW);
//  }
//  
//}
