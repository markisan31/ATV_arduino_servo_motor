void ReadSSI(void)  // Reads steering 12-bit value
{
    int i;
    char Resolution = 12;
    unsigned int bitStart = 0x0800;
    encoder_reading = 0;
    digitalWrite(encoderCsPin, LOW);
    delayMicroseconds(1);
    digitalWrite(encoderClockPin, LOW);
    for(i=(Resolution-1);i>=0;i--)
    {
        digitalWrite(encoderClockPin, HIGH);
        delayMicroseconds(1);
        if (digitalRead(encoderDataPin)) encoder_reading |= bitStart;
        digitalWrite(encoderClockPin, LOW);
        bitStart = bitStart >> 1;
        if (i == 0)
        {
            digitalWrite(encoderClockPin, HIGH);
            if (digitalRead(encoderDataPin)) encoder_reading |= bitStart;
        }
    }
    digitalWrite(encoderCsPin, HIGH);
    Serial.println(encoder_reading);
}
