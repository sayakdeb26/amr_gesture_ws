#include <micro_ros_arduino.h>
#include <FastLED.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include "esp_log.h"

#define LED_PIN 5
#define NUM_LEDS 100
CRGB leds[NUM_LEDS];

// Declare ROS 2 objects
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while (1) {
        leds[0] = CRGB::Red;  
        FastLED.show();
        delay(200);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(200);
    }
}

// 📌 Blinkt eine LED als Fehleranzeige
void blink_error() {
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(300);
    leds[0] = CRGB::Black;
    FastLED.show();
}

// 📌 Callback-Funktion: Verarbeitet eingehende ROS 2 Nachrichten
void led_callback(const void * msg_in)
{
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msg_in;

    // ✅ Check if data is empty
    if (msg->data.size == 0 || msg->data.data == NULL) {
        blink_error();
        return;
    }

    leds[1] = CRGB::Green;  
    FastLED.show();
    delay(200);
    leds[1] = CRGB::Black;
    FastLED.show();

    size_t maxSize = NUM_LEDS * 5;
    size_t dataSize = std::min(msg->data.size, maxSize);

    for (size_t i = 0; i + 4 < dataSize; i += 5)
    // ✅ Process each set of 5 values (ID, R, G, B, Brightness)
    //for (size_t i = 0; i + 4 < msg->data.size; i += 5)
    {
        int id = msg->data.data[i];
        int r = msg->data.data[i + 1];
        int g = msg->data.data[i + 2];
        int b = msg->data.data[i + 3];
        int brightness = msg->data.data[i + 4];

        if (id >= 0 && id < NUM_LEDS)
        {
            leds[id].setRGB(r, g, b);
        }
        else
        {
          leds[7] = CRGB::Red;  
          FastLED.show();
          delay(200);
          leds[7] = CRGB::Black;
          FastLED.show();
        }
    }

    // ✅ Update LEDs
    leds[1] = CRGB::Blue;  
    FastLED.show();
    delay(200);
    leds[1] = CRGB::Black;
    FastLED.show();
}

// 📌 Setup-Funktion: Initialisiert Micro-ROS und LED-Streifen
void setup()
{
    set_microros_transports();

    // ✅ Initialize the LED strip
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();

    // ✅ Initialize Micro-ROS
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp_node", "", &support));

    // ✅ Initialize msg structure
    memset(&msg, 0, sizeof(std_msgs__msg__Int32MultiArray));
    msg.data.capacity = NUM_LEDS * 5;
    msg.data.data = (int32_t*) malloc(msg.data.capacity * sizeof(int32_t));
    msg.data.size = 0;

    // ✅ Create subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/LED_strip"
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &led_callback, ON_NEW_DATA));

    // ✅ Indicate startup with a LED blink
    leds[0] = CRGB::Blue;  
    FastLED.show();
    delay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
}

// 📌 Hauptschleife: Verarbeitet eingehende Nachrichten und führt sie aus
void loop()
{
    // ✅ Blink status LED
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(100);
    leds[0] = CRGB::Black;
    FastLED.show();

    // ✅ Process incoming messages
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    delay(100);  
}
