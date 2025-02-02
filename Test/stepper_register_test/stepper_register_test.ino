#define dataPin 13   // 74HC595 Data pin (DS)
#define clockPin 12  // 74HC595 Clock pin (SHCP)
#define latchPin 14  // 74HC595 Latch pin (STCP)

#define steps 50       // Number of steps per movement
#define stepDelay 5    // Adjust for smoother motion

void setup() {
  Serial.begin(9600);
  
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  digitalWrite(latchPin, LOW); 
  shiftOut(dataPin, clockPin, MSBFIRST, 0);  
  digitalWrite(latchPin, HIGH);
}

void loop() {
  moveStepper(steps, 0);  // Clockwise
  delay(500);  

  moveStepper(steps, 1);  // Counterclockwise
  delay(500);  
}

// Function to move the stepper motor
void moveStepper(int stepCount, int dir) {
  sendToShiftRegister(dir, 0); // Set direction once before stepping

  for (int i = 0; i < stepCount; i++) {
    sendStepPulse(dir);  // Pass the correct direction
    delay(stepDelay);    // Shorter delay for smoother motion
  }
}

// Function to send direction to the shift register
void sendToShiftRegister(int dir, int step) {
  byte data = (dir << 1) | step;  
  digitalWrite(latchPin, LOW);  
  shiftOut(dataPin, clockPin, MSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}

// Function to generate step pulses properly
void sendStepPulse(int dir) {  
  sendToShiftRegister(dir, 1);  // STEP HIGH
  delayMicroseconds(200);       // Longer pulse for reliability
  sendToShiftRegister(dir, 0);  // STEP LOW
}
