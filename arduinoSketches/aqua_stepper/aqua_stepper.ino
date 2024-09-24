#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <AccelStepper.h>
#include <std_msgs/msg/int32.h>

// Stepper configuration
AccelStepper aquaStepper(AccelStepper::DRIVER, stepPin, dirPin);

// ROS-related variables
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// Pin definitions
#define stepPin 2
#define dirPin 4
#define home_switch 9

#define steps 200  // Steps per revolution for the motor (360/1.8 degrees)

// #define LED_PIN 13

// Motor position and homing state
long current_position = 0;   // Current position in steps
long target_position = 0;    // Target position in degrees
long target_steps = 0;       // Target position in steps
bool homing_state = false;   // Homing state


// RCCHECK macro to check return values
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// Function to handle errors
// void error_loop() {
//   while (1) {
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
// }


// Function to convert degrees to steps
long degToSteps(long degrees) {
  return (degrees * steps) / 360;
}

// Homing motor function
void homeMotor() {
  if (!homing_state) {
    // Start homing process
    aquaStepper.setSpeed(100);  // Set speed for homing
    if (digitalRead(home_switch) == HIGH) {
      aquaStepper.move(-1); // Move backward until the switch is pressed
      aquaStepper.runSpeed(); // Run continuously at the set speed
    } else {
      // Homing completed
      aquaStepper.stop();  // Stop the motor
      current_position = 0;  // Set home position to 0 steps
      aquaStepper.setCurrentPosition(0);  // Mark stepper at position 0
      homing_state = true;  // Set homing state to true
    }
  }
}

// ROS 2 subscription callback
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  if (homing_state) {  // Only move if homing is complete
    long new_position = msg->data;  // Get the new position in degrees
    target_position = current_position + new_position;  // Add new position to current position
    target_steps = degToSteps(target_position);  // Convert total degrees to steps
    aquaStepper.moveTo(target_steps);  // Move stepper to the new total target
    current_position = target_position;
  }
}
  


void setup() {
  // Initialize micro-ROS
  set_microros_transports();

  // Pin setup
  pinMode(home_switch, INPUT_PULLUP);  // Setup home switch pin
  // pinMode(LED_PIN, OUTPUT);

  // Stepper motor setup
  aquaStepper.setMaxSpeed(1000);
  aquaStepper.setAcceleration(500);

  // Initialize allocator and support
  allocator = rcl_get_default_allocator();

  // Create init options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stepper_control"
  );

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  // Start homing process
  homeMotor();
}

void loop() {
  // Spin executor to process any incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Continue homing process if not completed
  if (!homing_state) {
    homeMotor();  // Keep homing until the limit switch is pressed
  }

  // Run the stepper to move towards the target
  aquaStepper.run();

  delay(10);
}


// // ROS 2 subscription callback (for absolute movement)
// const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  // if (homing_state) {  // Only move if homing is complete
  //   target_position = msg->data;  // Set the new target position in degrees
  //   target_steps = degToSteps(target_position);  // Convert degrees to steps
  //   aquaStepper.moveTo(target_steps); // Move stepper to target steps
  // }













