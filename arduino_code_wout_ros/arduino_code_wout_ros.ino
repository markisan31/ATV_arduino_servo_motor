#include "PPMReader.h"
#include "GyverTimer.h"

GTimer left_light_timer(MS);
GTimer right_light_timer(MS);

//receiver attareceiver_channelsed to pin2
PPMReader ppm_reader(2, 0, false);
static int count;
int receiver_channels[] = {500, 0, 500, 0, 0, 0, 491, 0, 0};

constexpr uint8_t gas_motor_direction = 4;
constexpr uint8_t gas_motor_pwm = 5;
constexpr uint8_t motor_interface_type = 1;

constexpr uint8_t break_position = 3;

constexpr uint8_t turning_stepper_direction = 6;
constexpr uint8_t turning_stepper_pulse = 7;
constexpr uint8_t disable_stepper_pin = 14;

constexpr uint8_t break_actuator_pwm = 8;
constexpr uint8_t break_actuator_direction = 9;

constexpr uint8_t encoder_data_pin = 21;
constexpr uint8_t encoder_cs_pin = 20;
constexpr uint8_t encoder_clock_pin = 19;

constexpr uint8_t left_turn_light = 44;
constexpr uint8_t right_turn_light = 45;
constexpr uint8_t rear_front_lights = 46;

uint16_t rc_starting_left_pos = 485;
uint16_t rc_starting_right_pos = 515;

uint16_t mapped_output_value;
uint16_t mapped_output_value_encoder;
uint16_t target_position;
uint16_t encoder_reading;

bool left_light_on = false;
bool right_light_on = false;

constexpr short encoder_max_left_steer = 664;
constexpr short encoder_max_right_steer = 256;


const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; 

boolean newData = false;
uint8_t serialConnectionCounter = 0;
uint8_t stearingLimiter = 70;
int remoteRotation = 1500;
int remoteThrottle = 0;
int remoteBrake = 1000;
int remoteLeftLight = 0;
int remoteRightLight = 0;
int remoteHorn = 1500;
int remoteControl = 1000; // if 2000 then serial interface works
int remoteGear = 1000;

void setup()
{

    
    Serial.begin(460800);

    pinMode (break_actuator_direction, OUTPUT);
    pinMode (break_actuator_pwm, OUTPUT);

    pinMode (turning_stepper_pulse, OUTPUT);
    pinMode (turning_stepper_direction, OUTPUT);
    pinMode (disable_stepper_pin, OUTPUT);

    pinMode(gas_motor_pwm, OUTPUT);
    pinMode(gas_motor_direction, OUTPUT);

    pinMode(encoder_cs_pin, OUTPUT);
    pinMode(encoder_clock_pin, OUTPUT);
    pinMode(encoder_data_pin, INPUT);

    pinMode(break_position, INPUT);

    digitalWrite(encoder_clock_pin, HIGH);
    digitalWrite(encoder_cs_pin, LOW);
    
    
}

void loop()
{
    

    read_ssi();

    read_rc();
    //readFromSerial();
    
    

    turn_wheels(receiver_channels[0], receiver_channels[7]);


    output_gas_signal(receiver_channels[2], receiver_channels[7]);

    push_break(receiver_channels[4], receiver_channels[7]);

    left_turning_lights(receiver_channels[6], receiver_channels[7]);

    right_turning_lights(receiver_channels[6], receiver_channels[7]);

    turn_front_and_rear_lights_on(receiver_channels[5], receiver_channels[7]);

    //turn_backward_moving_lights(receiver_channels[2], receiver_channels[7]);

}


/*
 See funktsioon loeb andmed ressiiverist,
 muudab nende väärtust 0 ... 1000ni ja kirjutab neid massiivi receiver_channels[]
*/
void read_rc()
{
    while (ppm_reader.get(count) != 0)
    {
        receiver_channels[count] = map(ppm_reader.get(count), 1000, 2000, 0, 1000);
        count++;
    }
    count = 0;
}
/*
 See funktsioon võtab andmed receiver_channels massiivi 4 elemendist
 ja seejärel kontrollib. Kui väärtus on rohkem, kui 10
 siis aktuaatori draivi DIR pinnile saadetakse LOW signaal
 ja aktuaatori draivi PWM pinnile saadetakse HIGH signaal.
 Kui väärtus on vöhem, kui 10 siis aktuaatori draivi
 DIR pinnile saadetakse HIGH signaalja aktuaatori draivi
 PWM pinnile saadetakse HIGH signaal.
*/
void push_break(int receiver_channel4, int control_receiver_channel)
{
   
    
    
    
    uint8_t value = digitalRead(break_position);
    
    if (receiver_channel4 > 10)
    {
        digitalWrite(break_actuator_direction, LOW);
        digitalWrite(break_actuator_pwm, HIGH);
    }
    else if (receiver_channel4 < 10 && value == 1)
    {
        digitalWrite(break_actuator_direction, HIGH);
        digitalWrite(break_actuator_pwm, HIGH);
    }
    else if (receiver_channel4 < 10 && value == 0)
    {
        digitalWrite(break_actuator_direction, HIGH);
        digitalWrite(break_actuator_pwm, LOW);
    }
}

/*
 See funktsioon võtab andmed receiver_channels massiivi 0 elemendist
 ja seejärel kontrollib. Kui väärtus on vähem, kui 485
 siis samm mootori draivi DIR pinnile saadetakse HIGH signaal
 ja siis tsüklis iga 80 ms saadetakse pulse (ehk HIGH ja LOW
 signaalid üks teise järel) kuni tsükkel jõuab 10000ni. Kui
 väärtus on rohkem, kui 515 siis samm mootori draivi DIR
 pinnile saadetakse LOW signaal ja ülejöönud on sama.
*/

void turn_wheels(int receiver_channel0, int control_receiver_channel)

{


    int target_map = map(receiver_channel0, 1000, 0, 0, 1000);
    target_position = map(target_map, 0, 1000, encoder_max_right_steer, encoder_max_left_steer);

    



    if (target_position > mapped_output_value_encoder + 10)
    {
        digitalWrite(disable_stepper_pin, LOW);
        digitalWrite(turning_stepper_direction, HIGH);
        digitalWrite(turning_stepper_pulse, HIGH);  
        digitalWrite(turning_stepper_pulse, LOW);
    }

    else if (target_position < mapped_output_value_encoder - 10)
    {
        digitalWrite(disable_stepper_pin, LOW);
        digitalWrite(turning_stepper_direction, LOW);
        digitalWrite(turning_stepper_pulse, HIGH);
        digitalWrite(turning_stepper_pulse, LOW);
    }
    else
    {
        digitalWrite(disable_stepper_pin, HIGH);
    }
}

/*
 See funktsioon võtab andmed receiver_channels massiivi 2 elemendist
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
void output_gas_signal(int receiver_channel2, int control_receiver_channel)
{

    
    if (receiver_channel2 > 515)
    {
        mapped_output_value = map(receiver_channel2, 515, 1000, 45, 240);
        digitalWrite(gas_motor_direction, LOW);
        analogWrite(gas_motor_pwm, mapped_output_value);
    }
    else if (receiver_channel2 < 485)
    {
        mapped_output_value = map(receiver_channel2, 485, 0, 45, 240);
        digitalWrite(gas_motor_direction, HIGH);
        analogWrite(gas_motor_pwm, mapped_output_value);
    }
    else
    {
        analogWrite(gas_motor_pwm, 45);
    }

}

/*
See funktsioon võtab andmed receiver_channels massiivi 6 elemendist
ja seejärel kontrollib. Kui väärtus on vähem kui 400
siis boolean muutujasse rightLightOn muudab true'ks ja
seejörel kolm sekundit saadetakse HIGH signaali parempoolsete
suunatuledele. Kui taimer jõuab kolme sekundini siis boolean rightLightOn
muudab false'ks ja parempoolsete suunatuledele saadetakse LOW signaal.
*/
void right_turning_lights (int receiver_channel6, int control_receiver_channel)
{
    if (receiver_channel6 < 400 && left_light_on == false)
    {
        right_light_timer.setTimeout(3000);
        right_light_on = true;
    }
    
    if(right_light_timer.isEnabled())
    {
        right_light_timer.isReady();
        analogWrite(right_turn_light, 255);
    }
    else
    {
        right_light_on = false;
        analogWrite(right_turn_light, 0);
    }
}

/*
See funktsioon võtab andmed receiver_channels massiivi 6 elemendist
ja seejärel kontrollib. Kui väärtus on rohkem kui 600
siis boolean muutujasse leftLightOn muudab true'ks ja
seejörel kolm sekundit saadetakse HIGH signaali vasakpoolsete
suunatuledele. Kui taimer jõuab kolme sekundini siis boolean leftLightOn
muudab false'ks ja vasakpoolsete suunatuledele saadetakse LOW signaal.
*/
void left_turning_lights (int receiver_channel6, int control_receiver_channel)
{

    
    if (receiver_channel6 > 600 && right_light_on)
    {
        left_light_timer.setTimeout(3000);
        left_light_on = true;
        
    }
    if(left_light_timer.isEnabled())
    {
        left_light_timer.isReady();
        analogWrite(left_turn_light, 255);
    }
    else
    {
        left_light_on = false;
        analogWrite(left_turn_light, 0);
    }
}

/*See funktsioon võtab andmed receiver_channels massiivi 5 elemendist
ja seejärel kontrollib. Kui väärtus on rohkem kui 988
siis taga- ja esituled lülitakse sisse saadetes rear_front_lights
pinnile HIGH signaali. Vastasel juhul saadetakse LOW signaal.
*/
void turn_front_and_rear_lights_on(int receiver_channel5, int control_receiver_channel)
{

  
    if (receiver_channel5 > 988)
    {
        analogWrite(rear_front_lights, 255);
        
    }
    else
    {
        analogWrite(rear_front_lights, 0);
        digitalWrite(break_actuator_direction, LOW);
        digitalWrite(break_actuator_pwm, HIGH);
    }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readFromSerial(){
    recvWithStartEndMarkers();
      if (newData == true) {
          strcpy(tempChars, receivedChars);
              // this temporary copy is necessary to protect the original data
              //   because strtok() used in parseData() replaces the commas with \0
          parseData();
          showParsedData();
          newData = false;
          serialConnectionCounter = 0;
      }
      else{
        
        if (serialConnectionCounter > 100){
            remoteRotation = 1500;
            remoteThrottle = 1500;
            //remoteHorn = mySensVals[5];
            remoteBrake = 1000;
            remoteLeftLight  = 2000;
            remoteRightLight = 2000;
        }
        else{
          serialConnectionCounter++;
        }
      }
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else  {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ",");       
    remoteRotation = atoi(strtokIndx); 

    strtokIndx = strtok(NULL, ",");      
    remoteThrottle = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    remoteBrake = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    remoteHorn = atoi(strtokIndx);
    remoteHorn = remoteRotation;
  

}

//============

void showParsedData() {
  //Serial.println(currentRotation);
 
    Serial.print("Rotation ");
    Serial.println(remoteRotation);
    Serial.print("Throttle ");
    Serial.println(remoteThrottle);
    Serial.print("Brake ");
    Serial.println(remoteBrake);

   
}
