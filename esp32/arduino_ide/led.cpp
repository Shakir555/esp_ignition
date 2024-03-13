//(c) 2024 Shakir Salam
//This software is released under an open-source license, 
//Allowing unrestricted use and modification
//Distribution in both source and binary forms. No rights reserved.

// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin GPIO18 as an output.
  pinMode(18, OUTPUT);
}

// the loop function runs over and over again forever
void loop() 
{
  digitalWrite(18, HIGH); // turn the LED on
  delay(500);             // wait for 500 milliseconds
  digitalWrite(18, LOW);  // turn the LED off
  delay(500);             // wait for 500 milliseconds
}
