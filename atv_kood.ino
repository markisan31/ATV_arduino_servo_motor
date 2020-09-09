#include "PPMReader.h"

static int count;
int ch[] = {0, 0, 500, 0, 0, 0, 491, 0, 0};

constexpr int8_t gas_motor_direction = 4;
constexpr int8_t gas_motor_pwm = 5;
constexpr int8_t motor_interface_type = 1;

constexpr int8_t turning_stepper_direction = 6;
constexpr int8_t turning_stepper_pulse = 7;
constexpr int8_t disable_stepper_pin = 13;

constexpr int8_t break_actuator_pwm = 8;
constexpr int8_t break_actuator_direction = 9;

constexpr int8_t encoder_data_pin = 10;
constexpr int8_t encoder_cs_pin = 11;
constexpr int8_t encoder_clock_pin = 12;

constexpr int8_t left_turn_light = 44
constexpr int8_t right_turn_light = 45
constexpr int8_t real_front_lights = 46

int8_t delay_micros = 80;
int8_t resolution = 10;
int16_t mapped_output_value;
int16_t mapped_output_value_encoder;
int16_t target_position;
int16_t encoder_reading;
short current_micros = 0;
short previous_micros = 0;

unsigned static short previous_millis_left = 0;
unsigned static short previous_millis_right = 0;

unsigned short current_millis_left = 0;
unsigned short current_millis_right = 0;

constexpr short turn_lights_interval = 3000;
constexpr short encoder_max_left_steer = 664;
constexpr short encoder_max_right_steer = 256;

unsigned short start_time;
unsigned short end_time;

bool left_light_on = false;
bool right_light_on = false;

void setup()
{
    Serial.begin(115200);

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

    digitalWrite(encoder_clock_pin, HIGH);
    digitalWrite(encoder_cs_pin, LOW);
}

void loop()
{
    //startTime = micros();

    read_ssi();

    read_rc();

    turn_wheels(ch[0]);

    //endTime = micros();
    //startTime = endTime - startTime;
    //Serial.println(startTime);

    output_gas_signal(ch[2]);

    push_break(ch[4]);

    //left_turning_lights(ch[6]);

    //rightTurningLights(ch[6]);

    //turn_front_and_rear_lights_on(ch[5])
    
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

void push_break(int channel4)
{
    if (channel4 > 10)
    {
        digitalWrite(break_actuator_direction, LOW);
        digitalWrite(break_actuator_pwm, HIGH);
    }
    else if (channel4 < 10)
    {
        digitalWrite(break_actuator_direction, HIGH);
        digitalWrite(break_actuator_pwm, HIGH);
    }
}

void turn_wheels(int channel0)
{
    channel0 = map(channel0, 0, 1000, 1000, 0);
    target_position = map(channel0, 0, 1000, encoder_max_right_steer, encoder_max_left_steer);

    /* debuggimiseks
    Serial.print("Enkooder: \t"); Serial.print(mappedOutputValueEncoder);
    Serial.print("Pult: \t"); Serial.println(targetPosition);
    */

    if (target_position > mapped_output_value_encoder + resolution)
    {
        digitalWrite(disable_stepper_pin, LOW);
        digitalWrite(turning_stepper_direction, HIGH);
        digitalWrite(turning_stepper_pulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turning_stepper_pulse, LOW);
    }

    else if (target_position < mapped_output_value_encoder - resolution)
    {
        digitalWrite(disable_stepper_pin, LOW);
        digitalWrite(turning_stepper_direction, LOW);
        digitalWrite(turning_stepper_pulse, HIGH);
        delayMicroseconds(10);
        digitalWrite(turning_stepper_pulse, LOW);
    }
    else
    {
        digitalWrite(disable_stepper_pin, HIGH);
    }
}

void output_gas_signal(int channel2)
{
    if (channel2 > 515)
    {
        mapped_output_value = map(channel2, 515, 1000, 45, 80);
        digitalWrite(gas_motor_direction, HIGH);
        analogWrite(gas_motor_pwm, mappedOutputValue);
    }
    else if (channel2 < 485 )
    {
        mapped_output_value = map(channel2, 485, 0, 45, 80);
        digitalWrite(gas_motor_direction, LOW);
        analogWrite(gas_motor_pwm, mapped_output_value);
    }
    else
    {
        analogWrite(gas_motor_pwm, 45);
    }

}

void right_turning_lights (int channel6)
{
    if (channel6 < 400 )
    {
        previous_millis_right = current_millis_right;
        right_light_on = true;
    }
    if (current_millis_right > 3000 && current_millis_right - previous_millis_right <= turn_lights_interval && left_light_on == false)
    {
        analogWrite(right_turn_light, 255);
    }
    else if (current_millis_right - previous_millis_right > turn_lights_interval)
    {
        right_light_on = false;
        analogWrite(right_turn_light, 0);
    }
}

void left_turning_lights (int channel6)
{
    if (channel6 > 600 )
    {
        previous_millis_left = current_millis_left;
        left_light_on = true;
    }
    if (current_millis_left > 3000 && current_millis_left - previous_millis_left <= turn_lights_interval && right_light_on == false)
    {
        analogWrite(left_turn_light, 255);
    }
    else if (current_millis_left - previous_millis_left > turn_lights_interval)
    {
        left_light_on = false;
        analogWrite(left_turn_light, 0);
    }
}

void turn_front_and_rear_lights_on(int channel5)
{
    if (channel5 > 988)
    {
        analogWrite(real_front_lights, 255);
    }
    else
    {
        analogWrite(real_front_lights, 0);
    }
}
