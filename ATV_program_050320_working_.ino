#include <AccelStepper.h>

//specifing arrays and variables to store values
unsigned long int a,b,c;
int x[15],ch1[15],ch[7],i;

int motor2 = 6;// defines pin 7 as connected to the motor
int motor_speed = 7;
int pwm_1 = 8;
int dir_1 = 9;
int mappedOutputValue;

#define pwmPin 5
#define reverse 4
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, motor_speed, motor2);

void setup() {
Serial.begin(9600);
  stepper.setMaxSpeed(1000000);
  stepper.setAcceleration(500000);
  pinMode (dir_1, OUTPUT);
  pinMode (pwm_1, OUTPUT);
  pinMode (motor_speed, OUTPUT);
  pinMode (motor2, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(reverse, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING);
  // enabling interrupt at pin 2



}
void loop() {
read_rc();

//Serial.print(ch[1]);Serial.print("\t");
//Serial.print(ch[2]);Serial.print("\t");
//Serial.print(ch[3]);Serial.print("\t");
//Serial.print(ch[4]);Serial.print("\t");
//Serial.print(ch[5]);Serial.print("\t");
//Serial.print(ch[6]);Serial.print("\n");
outputGasSignal(ch[2]);

if (ch[5] > 10){
  digitalWrite(dir_1,LOW);
  digitalWrite(pwm_1,HIGH);
}
if (ch[5] < 10){
  digitalWrite(dir_1,HIGH);
  digitalWrite(pwm_1,HIGH);
}

if (ch[4] < 485){
  stepper.move(-1);
  stepper.run();
}
if (ch[4] > 515){
  stepper.move(1);
  stepper.run();
}
if (ch[4] < 515 && ch[4] > 485){

  stepper.moveTo(0);
  stepper.run();
  delay(2);
}



//delay(100);
}

void read_me()  {
 //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
 //this code gives channel values from 0-1000 values
 //    -: ABHILASH :-    //
a=micros(); //store time value a when pin value falling
c=a-b;      //calculating time inbetween two peaks
b=a;        //
x[i]=c;     //storing 15 value in array
i=i+1;       if(i==15){for(int j=0;j<15;j++) {ch1[j]=x[j];}
             i=0;}}//copy store all values from temporary array another array after 15 reading
void read_rc(){
int i,j,k=0;
  for(k=14;k>-1;k--){if(ch1[k]>5000){j=k;}}  //detecting separation space 10000us in that another array
  for(i=1;i<=6;i++){ch[i]=max(0, min(1000, (ch1[i+j]-1000)));}}
  //assign 6 channel values after separation space



void outputGasSignal(int speedInput) {
  if (speedInput > 500 + 10) {
    mappedOutputValue = map(speedInput, 500, 1000, 45, 214);
    Serial.print(mappedOutputValue);
    Serial.print("\n");
    digitalWrite(reverse, LOW);
    analogWrite(pwmPin, mappedOutputValue);
  }
  else if (speedInput < 500 - 10 ) {
    mappedOutputValue = map(speedInput, 490, 0, 45, 214);
    Serial.print(mappedOutputValue);
    Serial.print("\n");
    digitalWrite(reverse, HIGH);
    analogWrite(pwmPin, mappedOutputValue);
  }
  else {
    Serial.print(mappedOutputValue);
    Serial.print("\n");
    analogWrite(pwmPin, 15);
    digitalWrite(reverse, LOW);
  }

}
