//int motor1Left = 5;// defines pin 5 as connected to the motor
//int motor1Right= 6;// defines pin 6 as connected to the motor
int motor2 = 7;// defines pin 7 as connected to the motor
//int motor2Right = 8;// defines pin 8 as connected to the motor
int motor_speed = 4;

int channel1 = 9; // defines the channels that are connected
int channel2 = 10;// to pins 9 and 10 of arduino respectively

int Channel1 ; // Used later to 
int Channel2 ; // store values

long delay_Micros =50; // Set value (reading refresh time)
long currentMicros = 0; long previousMicros = 0;

void  setup ()
{
//   pinMode (motor1Left, OUTPUT);// initialises the motor pins
//   pinMode (motor1Right, OUTPUT);
   pinMode (motor_speed, OUTPUT);
   pinMode (motor2, OUTPUT);
//   pinMode (motor2Right, OUTPUT);// as outputs
   pinMode (channel1, INPUT);// initialises the channels
   pinMode (channel2, INPUT);// as inputs
   
//   Serial.begin (9600); // Sets the baud rate to 9600 bps
//   digitalWrite (motor_speed, HIGH);
}

void  loop ()
{
  currentMicros = micros();  
  if(currentMicros - previousMicros >= delay_Micros){  
    previousMicros = currentMicros;  
    digitalWrite(motor_speed,HIGH);  
    delayMicroseconds(10); //Set Value
    digitalWrite(motor_speed,LOW);
    
  }
  Channel1 = (pulseIn (channel1, HIGH)); // Checks the value of channel1
  
//  Serial.println (Channel1); //Prints the channels value on the serial monitor

  if (Channel1 > 1300 && Channel1 < 1500 ) /*If these conditions are true, do the following. These are the values that I got from my transmitter, which you may customize according to your transmitter values */
  {
//    digitalWrite (motor1Left, LOW); // Sets both the
//    digitalWrite (motor1Right, LOW);// motors to low
  }

  if (Channel1 < 1300) // Checks if Channel1 is lesser than 1300
  {
//    digitalWrite (motor1Left, HIGH);// Turns the left
//    digitalWrite (motor1Right, LOW); // motor forward
//    Serial.println("Backward");
  }
  if (Channel1 > 1500) // Checks if Channel1 is greater than 1500
  {
//    digitalWrite (motor1Left, LOW);// Turns the right
//    digitalWrite (motor1Right, HIGH);// motor forward
//    Serial.println("Forward");
  }
  Channel2 = (pulseIn (channel2, HIGH)); // Checks the value of channel1
//  Serial.println (Channel2); //Prints the channels value value on the serial monitor

  if (Channel2 > 1300 && Channel1 < 1500 ) // If these conditions are true, do the following
  {
    
//    digitalWrite (motor2, LOW);// Sets both the
//    digitalWrite (motor2Right, LOW);// motors to low
    
  }

  if (Channel2 < 1300) // Checks if Channel2 is lesser than 1300
  {
//    digitalWrite (motor_speed, HIGH);
    digitalWrite (motor2, LOW);// Turns the left
//    digitalWrite (motor2Right, HIGH);// motor backward
    Serial.println("turn left");
  }
  if (Channel2 > 1500) // Checks if Channel2 is greater than 1500
  {
//    digitalWrite (motor_speed, HIGH);
    digitalWrite (motor2, HIGH);// Turns the right
//    digitalWrite (motor2Right, LOW);// motor backward
    Serial.println("turn right");
  }
}

//int change_dir()
//{
//  Channel2 = (pulseIn (channel2, HIGH));
//  if (Channel2 < 1300) {
//    digitalWrite (motor2, LOW);// Turns the right
//  } else if (Channel2 > 1500) {
//    digitalWrite (motor2, HIGH);// Turns the right
//  } else {
//    digitalWrite(motor_speed, LOW); 
//  }
//}
