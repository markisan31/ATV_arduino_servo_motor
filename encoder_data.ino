#include <AccelStepper.h>

namespace encoder_data
{

AccelStepper stepper = AccelStepper(motorInterfaceType, motor_speed, motor2);

void encoder_data()
{
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_CS, LOW);

    int pos = 0;
    int mappedOutputValue = 0;

    for (int i = 0; i < 10; i++)
    {
        digitalWrite(PIN_CLOCK, LOW);
        digitalWrite(PIN_CLOCK, HIGH);
        byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
        pos += b * pow(2, 10 - (i + 1));
        mappedOutputValueEncoder = map(pos, 0, 1020, -180, 180); //Old low: 0, Old high: 1020, New low: -180, New high: 180

        if (mappedOutputValueEncoder < 85 && mappedOutputValueEncoder > 1)
        {
            //DO something
            Serial.println("Reached left!");
            Serial.println(mappedOutputValueEncoder);
        }
        else if (mappedOutputValueEncoder > 85)
        {
            Serial.println("Reached maximum left steer!"); Serial.println("Moved stepper motor to "); Serial.println(maxLeftSteer); Serial.println(mappedOutputValueEncoder);
            stepper.move(maxLeftSteer);
            stepper.run();
        }
        else if (mappedOutputValueEncoder < -1 && mappedOutputValueEncoder > -85)
        {
            //DO something
            Serial.println("Reached right!");
            Serial.println(mappedOutputValueEncoder);
        }
        else if (mappedOutputValueEncoder < -85)
        {
            Serial.println("Reached maximum right steer!"); Serial.println("Moved stepper motor to "); Serial.println(maxRightSteer); Serial.println(mappedOutputValueEncoder);
            stepper.move(maxRightSteer);
            stepper.run();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        digitalWrite(PIN_CLOCK, LOW);
        digitalWrite(PIN_CLOCK, HIGH);
    }

    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);

}
}
