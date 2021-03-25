#include <Wire.h>
const int ledPin =  LED_BUILTIN;// the number of the LED pin
void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
     if ( c == 2 )
      digitalWrite(ledPin, HIGH);
      else if(c == 1) 
       digitalWrite(ledPin, LOW);
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
   if ( x == 2 )
      digitalWrite(ledPin, HIGH);
      else if(x == 1) 
       digitalWrite(ledPin, LOW);
}
