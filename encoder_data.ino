#include <AccelStepper.h>

namespace encoder_data
{
void encoder_data()
{
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_CS, LOW);

    int pos = 0;
    int mappedOutputValue = 0;

    for (int i = 0; i<10; i++)
    {
        digitalWrite(PIN_CLOCK, LOW);
        digitalWrite(PIN_CLOCK, HIGH);
        byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
        pos += b * pow(2, 10-(i+1));
        mappedOutputValueEncoder = map(pos, 0, 1020, 0, 359); //Old low: 0, Old high: 1020, New low: -120, New high: 120

        if (mappedOutputValueEncoder >= 180)
        {
            //DO something
            Serial.println("0 < Right"); Serial.println("\n");
        }
        else if (mappedOutputValueEncoder <= 180)
        {
            //DO something
            Serial.println("0 > Left"); Serial.println("\n");
        }
    }

    for (int i=0; i<6; i++)
    {
        digitalWrite(PIN_CLOCK, LOW);
        digitalWrite(PIN_CLOCK, HIGH);
    }

    digitalWrite(PIN_CLOCK, LOW);
    digitalWrite(PIN_CLOCK, HIGH);

}
}
