#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Define pin connections for the shift registers
#define dataPin 13   // 74HC595 Data pin (DS)
#define clockPin 12  // 74HC595 Clock pin (SHCP)
#define latchPin 14  // 74HC595 Latch pin (STCP)

#define steps 50      // Number of steps per movement
#define stepDelay 5   // Milliseconds delay between steps

// ROS-related variables
rcl_subscription_t stepper_subscriber;
std_msgs__msg__Int32 stepper_msg;
rcl_subscription_t test_subscriber;
std_msgs__msg__Int32 test_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int firstRegisterData = 0;
int secondRegisterData = 0;
int direction = 0;
unsigned long lastStepTime = 0;
int currentStep = 0;

// Function to handle errors
void error_loop() {
    ESP.restart();
    delay(50);
}

// RCCHECK macro to check return values
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// Function to send data to the shift registers
void sendToShiftRegisters() {

  digitalWrite(latchPin, LOW);  // Prepare to latch the data
  shiftOut(dataPin, clockPin, MSBFIRST, secondRegisterData);  // Send to second register 
  shiftOut(dataPin, clockPin, MSBFIRST, firstRegisterData);  // Keep first register independent
  digitalWrite(latchPin, HIGH); // Update the shift registers
}

// Function to generate step pulses
void sendStepPulse(int dir) {  
    secondRegisterData = (dir << 3) | (1 << 2); // Set STEP high
    sendToShiftRegisters();
    delayMicroseconds(200);  // Short delay for step pulse
    secondRegisterData = (dir << 3); // Set STEP low
    sendToShiftRegisters();
}

// Function to move the stepper motor non-blocking
void moveStepper(int stepCount, int dir) {
    direction = dir;
    secondRegisterData = (dir << 3);  // Set direction bit
    sendToShiftRegisters();

    // Move the motor in a single sequence
    for (int i = 0; i < stepCount; i++) {
      sendStepPulse(dir);  // Send a step pulse
      delay(stepDelay);    // Small delay between steps
  }
}

// Callback function for receiving ROS 2 messages (motor control)
void motor_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    if (msg->data == 1) {
        moveStepper(steps, 0); // Clockwise
    } else if (msg->data == -1) {
        moveStepper(steps, 1); // Counterclockwise
    }
}

// Callback function for updating first shift register (test signal)
void test_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    firstRegisterData = msg->data;
    sendToShiftRegisters();  // Ensure first register updates
}

void setup() {
    Serial.begin(9600);

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
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "stepper_subscriber", "", &support));

    // Create stepper subscriber
    RCCHECK(rclc_subscription_init_default(
        &stepper_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "stepper_control"));

    // Create test subscriber
    RCCHECK(rclc_subscription_init_default(
        &test_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "test_first"));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &stepper_subscriber, &stepper_msg, &motor_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &test_subscriber, &test_msg, &test_callback, ON_NEW_DATA));
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
