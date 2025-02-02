#define dirPin 12                // Direction pin for motor
#define stepPin 14              // Step pin for motor
const int stepsPerRevolution = 200;   // Steps for one full motor revolution(360/1.8)

void setup() {
  pinMode(stepPin, OUTPUT);  // Set step pin as output
  pinMode(dirPin, OUTPUT);   // Set direction pin as output

  // Initially set motor to rotate clockwise
  digitalWrite(dirPin, HIGH);  
}

void loop() {
  // Rotate motor clockwise
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);  // Step motor
    delayMicroseconds(2000);      // Wait 2000us for slower speed
    digitalWrite(stepPin, LOW);   // Stop step
    delayMicroseconds(2000);      // Wait 2000us
  }

  delay(1000);  // Wait 1 second

  // Change motor direction to counterclockwise
  digitalWrite(dirPin, LOW);

  // Rotate motor counterclockwise and faster
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);  // Step motor
    delayMicroseconds(1000);      // Wait 1000us for faster speed 
    digitalWrite(stepPin, LOW);   // Stop step
    delayMicroseconds(1000);      // Wait 1000us
  }

  delay(1000);  // Wait 1 second
}
