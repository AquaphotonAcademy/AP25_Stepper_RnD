// Define pin connections for the shift registers
#define dataPin 13   // 74HC595 Data pin (DS)
#define clockPin 12  // 74HC595 Clock pin (SHCP)
#define latchPin 14  // 74HC595 Latch pin (STCP)

#define steps 100     // Number of steps per movement
#define stepDelay 5   // Minimal delay between steps (milliseconds)

// Variable to store the first shift register's value
uint8_t firstRegisterValue = 128;  // Binary: 10000000 (example value)

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging

  // Set the shift register pins as outputs
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  // Initialize the first shift register with a predefined value
  digitalWrite(latchPin, LOW); 
  shiftOut(dataPin, clockPin, MSBFIRST, 0);  // Second register initialized with 0
  shiftOut(dataPin, clockPin, MSBFIRST, firstRegisterValue);  // Load first register
  digitalWrite(latchPin, HIGH);
}

void loop() {
  // Move the stepper motor counterclockwise
  moveStepper(steps, 0);
  delay(500);  // Wait for half a second
  
  // Move the stepper motor clockwise
  moveStepper(steps, 1);
  delay(500);  // Wait for half a second
}

// Function to move the stepper motor a specified number of steps
void moveStepper(int stepCount, int dir) {
  Serial.print("Setting direction: ");
  Serial.println(dir == 1 ? "Clockwise" : "Counterclockwise");

  // Set direction before stepping
  sendToShiftRegisters(dir, 0);  

  // Calculate second register value with initial state (STEP = 0)
  uint8_t secondRegisterData = (dir << 3) | (1 << 2);  

  // Print the register values in binary and decimal
  Serial.print("First Register: ");
  Serial.print(firstRegisterValue, BIN);  
  Serial.print(" (Decimal: ");
  Serial.print(firstRegisterValue, DEC);
  Serial.print(") | Second Register: ");
  Serial.print(secondRegisterData, BIN);
  Serial.print(" (Decimal: ");
  Serial.print(secondRegisterData, DEC);
  Serial.println(")");

  // Move the motor in a single sequence
  for (int i = 0; i < stepCount; i++) {
    sendStepPulse(dir);  // Send a step pulse
    delay(stepDelay);    // Small delay between steps
  }
}

// Function to send direction and step signals to the shift registers
void sendToShiftRegisters(int dir, int step) {
  uint8_t secondRegisterData = (dir << 3) | (step << 2);  // DIR at bit 3, STEP at bit 2
  
  digitalWrite(latchPin, LOW);  // Prepare to latch the data
  shiftOut(dataPin, clockPin, MSBFIRST, secondRegisterData);  // Send data to second register
  shiftOut(dataPin, clockPin, MSBFIRST, firstRegisterValue);  // Keep first register value
  digitalWrite(latchPin, HIGH);  // Update shift registers
}

// Function to generate step pulses properly by toggling the STEP pin
void sendStepPulse(int dir) {  
  sendToShiftRegisters(dir, 1);  // Set STEP high
  delayMicroseconds(200);        // Hold STEP high for a short duration
  sendToShiftRegisters(dir, 0);  // Set STEP low
  delayMicroseconds(200);        // Hold STEP low to complete the pulse
}
