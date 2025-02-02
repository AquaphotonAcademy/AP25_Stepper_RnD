// Define pin connections for the shift registers
#define dataPin 48   // 74HC595 Data pin (DS)
#define clockPin 47 // 74HC595 Clock pin (SHCP)
#define latchPin 21  // 74HC595 Latch pin (STCP)

#define steps 100      // Number of steps per movement
#define stepDelay 5    // Adjust for smoother motion

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  
  // Set the shift register pins as output
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  digitalWrite(latchPin, LOW);  // Start with latch pin low to prevent updates
  // shiftOut16(0);  // Clear both shift registers by sending zero
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);

  digitalWrite(latchPin, HIGH);  // Update the shift registers
}

void loop() {
  moveStepper(steps, 0);  // Move the stepper motor clockwise
  delay(500);  // Wait for half a second
  
  moveStepper(steps, 1);  // Move the stepper motor counterclockwise
  delay(500);  // Wait for half a second
}

// Function to move the stepper motor a specified number of steps
void moveStepper(int stepCount, int dir) {
  sendToShiftRegisters(dir, 0); // Set the direction before stepping

  for (int i = 0; i < stepCount; i++) {
    sendStepPulse(dir);  // Send a pulse to step the motor
    delay(stepDelay);    // Delay between steps for smoother motion
  }
}

// // Function to send direction and step signals to the shift registers
 void sendToShiftRegisters(int dir, int step) {
 uint8_t data = (dir << 3) | (step << 2);  // Set DIR at bit 9 and STEP at bit 8, others are zero

 digitalWrite(latchPin, LOW);  // Prepare to latch the data
 shiftOut16(data);
 digitalWrite(latchPin, HIGH);  // Update the shift registers
}

// Function to generate step pulses properly by toggling the STEP pin
void sendStepPulse(int dir) {  
  sendToShiftRegisters(dir, 1);  // Set STEP high
  delayMicroseconds(200);         // Hold the STEP high for a short time
  sendToShiftRegisters(dir, 0);  // Set STEP low
}

// Function to shift out 16-bit data to two cascaded shift registers
void shiftOut16(uint8_t data) {
  // Send the high byte (bits 8-15) to the second shift register
  shiftOut(dataPin, clockPin, MSBFIRST, data); 
  
  // Send the low byte (bits 0-7) to the first shift register (this will be all zeros)
  shiftOut(dataPin, clockPin, MSBFIRST, 0);        
}
