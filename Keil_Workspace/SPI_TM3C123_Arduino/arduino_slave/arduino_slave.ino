int ledPin = 9;      // LED connected to digital pin 9
int val = 0;         // variable to store the read value

void setup() {
  pinMode(ledPin, OUTPUT);  // sets the pin as output
 
  analogWrite(ledPin, 127); // analogRead values go from 0 to 1023, analogWrite values from 0 to 255

}

void loop() {
  
}
