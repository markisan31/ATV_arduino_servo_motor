void read_ssi(void)  // Reads steering encoder value, 12-bit
{
    int i;
    char resolution = 12;
    unsigned int bitStart = 0x0800;

    encoder_reading = 0;
    digitalWrite(encoder_cs_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(encoder_clock_pin, LOW);

    for(i=(resolution-1);i>=0;i--)
    {
        digitalWrite(encoder_clock_pin, HIGH);
        delayMicroseconds(1);

        if (digitalRead(encoder_data_pin)) encoder_reading |= bitStart;
        digitalWrite(encoder_clock_pin, LOW);
        bitStart = bitStart >> 1;

        if (i == 0)
        {
            digitalWrite(encoder_clock_pin, HIGH);
            if (digitalRead(encoder_data_pin)) encoder_reading |= bitStart;
        }
        
        mapped_output_value_encoder = map(encoder_reading, 0, 3000, 0, 720);

    }
    digitalWrite(encoder_cs_pin, HIGH);
}
