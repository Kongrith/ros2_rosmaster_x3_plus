/* Encoder Library - Basic Example
   http://www.pjrc.com/teensy/td_libs_Encoder.html

   This example code is in the public domain.
*/

#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t   executor;
rcl_node_t node;

rcl_timer_t     timer1;

rcl_publisher_t encoder_pub;
std_msgs__msg__Int32 encoder_msg;

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(7, 8);
//   avoid using pins with LEDs attached

long pulse = 0;

void timer1_callback(rcl_timer_t *timer1, int64_t last_call_time) {
  encoder_msg.data = pulse;
  rcl_publish(&encoder_pub, &encoder_msg, NULL);
}

void setup() {
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "encoder_read_node", "", &support);

  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_value"
  );

  rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(1000),
    timer1_callback
  );


  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator
  );

  rclc_executor_add_timer(&executor, &timer1);
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  pulse = newPosition;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
