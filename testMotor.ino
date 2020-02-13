#define Pulse 4
#define Dir 7
long delay_Micros =50; // Set value (reading refresh time)
long currentMicros = 0; long previousMicros = 0;

void setup(){
  pinMode(Pulse,OUTPUT);
  pinMode(Dir,OUTPUT);
  digitalWrite(Dir,LOW);
  }

void loop(){ 
  currentMicros = micros();  
  if(currentMicros - previousMicros >= delay_Micros){  
    previousMicros = currentMicros;  
    digitalWrite(Pulse,HIGH);  
    delayMicroseconds(100); //Set Value
    digitalWrite(Pulse,LOW);
  } 
}
