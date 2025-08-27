#include <micro_ros_arduino.h>
#include <FastLED.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#define LED_PIN 5
#define NUM_LEDS 100

CRGB leds[NUM_LEDS];

// ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Int32MultiArray sub_msg;
std_msgs__msg__Int32 pub_msg;

// Subscriber callback
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // Blink blue at start
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(150);
  leds[0] = CRGB::Black;
  FastLED.show();

  FastLED.clear();
  
  if (msg->data.size > 0 && msg->data.size % 5 == 0) {
    for (size_t i = 0; i < msg->data.size; i += 5) {
      int led_index = msg->data.data[i];
      int r = msg->data.data[i + 1];
      int g = msg->data.data[i + 2];
      int b = msg->data.data[i + 3];
      int brightness = msg->data.data[i + 4];

      // Publish the first index back only once
      if (i == 0) {
        pub_msg.data = led_index;
        rcl_publish(&publisher, &pub_msg, NULL);
      }

      // If index is in range, set that LED
      if (led_index >= 0 && led_index < NUM_LEDS) {
        leds[led_index] = CRGB(r, g, b);
        leds[led_index].nscale8_video(brightness);  // apply brightness scaling
      }
    }
    FastLED.show();
  }

  // Blink cyan at end
  leds[0] = CRGB::Cyan;
  FastLED.show();
  delay(150);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void setup() {
  // Init FastLED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Blink green on startup
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(400);
  leds[0] = CRGB::Black;
  FastLED.show();

  // Init micro-ROS transport
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // Init support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Node
  rclc_node_init_default(&node, "esp32_led_node", "", &support);

  // Subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/LED_strip");

  // Publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/LED_first_element");

  // Allocate memory for sub_msg (sequence types)
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_basic_type_sequence_capacity = 10;  // allow up to 10 ints
  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    &sub_msg,
    conf
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
