const int dirPin = 2;                 // Direction pin for motor
const int stepPin = 3;                // Step pin for motor
const int stepsPerRevolution = 200;   // Steps for one full motor revolution(360/1.8)

void setup() {
  pinMode(stepPin, OUTPUT);  // Set step pin as output
  pinMode(dirPin, OUTPUT);   // Set direction pin as output

  digitalWrite(dirPin, HIGH);  // Set motor to rotate clockwise
}

void loop() {
  // Rotate motor clockwise
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);  // Step motor
    delayMicroseconds(2000);      // Wait 2000us
    digitalWrite(stepPin, LOW);   // Stop step
    delayMicroseconds(2000);      // Wait 2000us
  }

  delay(1000);  // Wait 1 second

  digitalWrite(dirPin, LOW);  // Change motor direction to counterclockwise

  // Rotate motor counterclockwise and faster
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);  // Step motor
    delayMicroseconds(1000);      // Wait 1000us 
    digitalWrite(stepPin, LOW);   // Stop step
    delayMicroseconds(1000);      // Wait 1000us
  }

  delay(1000);  // Wait 1 second
}
