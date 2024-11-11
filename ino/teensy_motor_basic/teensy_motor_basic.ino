#include <micro_ros_arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

rcl_allocator_t         allocator;
rclc_support_t          support;
rclc_executor_t         executor;

rcl_node_t              node;

rcl_subscription_t      power_sub;
std_msgs__msg__Float32  power_msg;

#define                 l_dir1_pin 3
#define                 l_dir2_pin 4
#define                 l_pwm_pin 5

int     dir = 1; // go forward
float   factor = 0.0;
int     pwm_value = 255;

void drive() {
  pwm_value = abs(factor * 255);

  // forward
  if (dir > 0) {
    digitalWrite(l_dir1_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, pwm_value);
  }
  else if (dir < 0) {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, HIGH);
    analogWrite(l_pwm_pin, pwm_value);
  }
  else {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, pwm_value);
  }
}

void power_callback(const std_msgs__msg__Float32 *msgin) {
  factor = msgin -> data;

  if (factor > 0 || factor < 0)
  {
    dir = factor/abs(factor);
  }
  else
  {
    dir = 0;
  }
}

void setup() {
  set_microros_transports();

  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);
  pinMode(l_pwm_pin, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "motor_power_node", "", &support);

  rclc_subscription_init_default(
    &power_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "power_sub"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator); //timer + subscription

  rclc_executor_add_subscription(&executor, &power_sub, &power_msg, &power_callback, ON_NEW_DATA);
}

void loop() {
  drive();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0.1));
}
