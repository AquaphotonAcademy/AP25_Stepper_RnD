#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <AccelStepper.h>
#include <std_msgs/msg/int32.h>


// Define pin connections for the shift registers
#define dataPin 13   // 74HC595 Data pin (DS) 48
#define clockPin 12 // 74HC595 Clock pin (SHCP) 47
#define latchPin 14  // 74HC595 Latch pin (STCP) 21

#define steps 50      // Number of steps per movement
#define stepDelay 5    


// ROS-related variables
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int direction = 0;

void error_loop() {

   Serial.println("Error occurred! Entering error loop...");
   ESP.restart();
   delay(50);
  
}

// RCCHECK macro to check return values
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }


// Function to shift out 16-bit data to two cascaded shift registers
void shiftOut16(uint8_t data) {
  shiftOut(dataPin, clockPin, MSBFIRST, data);  
  shiftOut(dataPin, clockPin, MSBFIRST, 0);     
}

// Function to send direction and step signals to the shift registers
void sendToShiftRegisters(int dir, int step) {
  uint8_t data = (dir << 3) | (step << 2);

  digitalWrite(latchPin, LOW);  // Prepare to latch the data
  shiftOut16(data);
  digitalWrite(latchPin, HIGH); // Update the shift registers
}

// Function to generate step pulses properly by toggling the STEP pin
void sendStepPulse(int dir) {  
  sendToShiftRegisters(dir, 1);  // Set STEP high
  delayMicroseconds(200);        // Hold the STEP high for a short time
  sendToShiftRegisters(dir, 0);  // Set STEP low
}

// Function to move the stepper motor a specified number of steps
void moveStepper(int stepCount, int dir) {
  for (int i = 0; i < stepCount; i++) {
    sendStepPulse(dir);  
    delay(stepDelay);    
  }
}

// Callback function for receiving ROS 2 messages
void motor_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  if (msg->data == 1) {
    // Clockwise rotation
    direction = 0;
    moveStepper(steps, direction);
  } else if (msg->data == -1) {
    // Counterclockwise rotation
    direction = 1;
    moveStepper(steps, direction);
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set the shift register pins as output
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  digitalWrite(latchPin, HIGH);

  // Micro-ROS initialization
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "stepper_subscriber", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stepper_control"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &motor_callback, ON_NEW_DATA));
}



void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
