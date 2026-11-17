#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <tutorial_interfaces/msg/servos_command2.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example only works with Arduino Serial transport.
#endif

// #define ARRAY_SIZE 3

// ServoEasing
#include "ServoEasing.hpp"
#include "PinDefinitionsAndMore.h"

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

#define START_DEGREE_VALUE 90 // The degree value written to the servo at time of attach.
#define DELAY_BETWEEN_ACTIONS_MILLIS 100

// int32_t received_buffer[ARRAY_SIZE];
// int32_t send_buffer[ARRAY_SIZE];

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
tutorial_interfaces__msg__ServosCommand2 received_msg;
tutorial_interfaces__msg__ServosCommand2 send_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

#define RCCHECK(fn)           \
    {                         \
        rcl_ret_t rc = fn;    \
        if (rc != RCL_RET_OK) \
        {                     \
            while (1)         \
            {                 \
                delay(100);   \
            }                 \
        }                     \
    }

#define RCSOFTCHECK(fn)    \
    {                      \
        rcl_ret_t rc = fn; \
        (void)rc;          \
    }

// servos for neck
int minAngle = -30;
int maxAngle = 30;
int minAngleYaw = -90;
int maxAngleYaw = 90;

// Latest target received from the laptop
int targetYaw = 0;
int targetPitch = 0;
int targetRoll = 0;

// Indicates whether a new target has arrived
bool newTargetAvailable = false;

// Records the last target sent to the servos
int lastYaw = 0;
int lastPitch = 0;
int lastRoll = 0;
bool hasLastTarget = false;

// Servo movement speed
const uint16_t tSpeed = 20;

void neckControl(int yaw, int pitch, int roll);
void subscription_callback(const void *msgin);

void setup()
{
    Serial.begin(115200);

    set_microros_serial_transports(Serial);

    delay(2000);

    // received_msg.data.data = received_buffer;
    // received_msg.data.size = 0;
    // received_msg.data.capacity = ARRAY_SIZE;

    // send_msg.data.data = send_buffer;
    // send_msg.data.size = 0;
    // send_msg.data.capacity = ARRAY_SIZE;

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // a node that can publish and subscribe
    RCCHECK(
        rclc_node_init_default(
            &node,
            "esp32_node",
            "",
            &support));

    RCCHECK(
        rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(tutorial_interfaces, msg, ServosCommand2),
            "servos_laptop2esp32"));

    // Initialize a ROS publisher with the name "micro_ros_platformio_node_publisher" to publish Int32 messages
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tutorial_interfaces, msg, ServosCommand2),
        "resend_esp2host"));

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator));

    RCCHECK(
        rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &received_msg,
            &subscription_callback,
            ON_NEW_DATA));

    Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE);
    Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE);
    Servo3.attach(SERVO3_PIN, START_DEGREE_VALUE);

    delay(4000);
}

void loop()
{
    RCSOFTCHECK(
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(100)));

    // Start a new movement only when the current one has finished
    // if (newTargetAvailable &&
    //     !ServoEasing::areInterruptsActive())
    if (newTargetAvailable &&
        !ServoEasing::areInterruptsActive())
    {
        // Copy the latest target
        int yaw = targetYaw;
        int pitch = targetPitch;
        int roll = targetRoll;

        // Mark this target as handled
        newTargetAvailable = false;

        // Avoid restarting movement for an unchanged target
        if (!hasLastTarget ||
            yaw != lastYaw ||
            pitch != lastPitch ||
            roll != lastRoll)
        {
            // delay(DELAY_BETWEEN_ACTIONS_MILLIS);
            neckControl(yaw, pitch, roll);

            lastYaw = yaw;
            lastPitch = pitch;
            lastRoll = roll;
            hasLastTarget = true;
        }
    }

    // delay(10);
}

void neckControl(int yaw, int pitch, int roll)
{
    // uint16_t tSpeed = 20;

    if (yaw > maxAngleYaw)
    {
        yaw = maxAngleYaw;
    }
    else if (yaw < minAngleYaw)
    {
        yaw = minAngleYaw;
    }

    if (pitch > maxAngle)
    {
        pitch = maxAngle;
    }
    else if (pitch < minAngle)
    {
        pitch = minAngle;
    }

    if (roll > maxAngle)
    {
        roll = maxAngle;
    }
    else if (roll < minAngle)
    {
        roll = minAngle;
    }

    int motorYaw = 90 - round(31.0 / 44.0 * yaw);
    int motorPitch = 90 - round(31.0 / 44.0 * pitch);
    int motorRoll = 90 + round(31.0 / 44.0 * roll);

    /*
     * Move three servos synchronously with interrupt handler
     */
    ServoEasing::ServoEasingNextPositionArray[0] = motorYaw;
    ServoEasing::ServoEasingNextPositionArray[1] = motorPitch;
    ServoEasing::ServoEasingNextPositionArray[2] = motorRoll;
    setEaseToForAllServosSynchronizeAndStartInterrupt(tSpeed); // Set speed and start interrupt here, we check the end with areInterruptsActive()
}

// Callback function
void subscription_callback(const void *msgin)
{
    const tutorial_interfaces__msg__ServosCommand2 *msg =
        (const tutorial_interfaces__msg__ServosCommand2 *)msgin;

    // send_msg.data.size = msg->data.size;
    // for (int i = 0; i < msg->data.size; i++)
    // {
    //     send_msg.data.data[i] = msg->data.data[i];
    // }

    targetYaw = (int)round(msg->new_yaw);
    targetPitch = (int)round(msg->new_pitch);
    targetRoll = (int)round(msg->new_roll);

    // Tell loop() that a new target is available
    newTargetAvailable = true;

    // Publish the received values
    send_msg.new_yaw = msg->new_yaw;
    send_msg.new_pitch = msg->new_pitch;
    send_msg.new_roll = msg->new_roll;
    // send_msg.data.size = 3;
    // for (int i = 0; i < 3; i++)
    // {
    //     send_msg.data.data[i] = msg->data.data[i];
    // }

    RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
}

/*// We include the Arduino library for general Arduino functionality
#include <Arduino.h>
// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <micro_ros_platformio.h>

// These are core ROS2 libraries for creating nodes, publishers, and executors
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// This is a standard ROS2 message type, an integer
#include <std_msgs/msg/int32.h>

// Ensure that the transport layer being used is Arduino Serial.
// If it's not, compilation is stopped and error is printed.
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Define ROS2 objects for a publisher, a message, an executor, support objects, an allocator, a node, and a timer
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Infinite error loop function. If something fails, the device will get stuck here
void error_loop()
{
    while (1)
    {
        delay(100);
    }
}

// This is the function that will be called every time the timer expires
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // We publish our message here
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        // The message contains a single integer that we increment each time
        msg.data++;
    }
}

void setup()
{
    // Start serial communication with a baud rate of 115200
    Serial.begin(115200);
    Serial.println("ESP32 started");
    // Configure Micro-ROS library to use Arduino serial
    set_microros_serial_transports(Serial);
    // Allow some time for everything to start properly
    delay(2000);

    // Get the default memory allocator provided by rcl
    allocator = rcl_get_default_allocator();

    // Initialize rclc_support with default allocator
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Initialize a ROS node with the name "micro_ros_platformio_node"
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // Initialize a ROS publisher with the name "micro_ros_platformio_node_publisher" to publish Int32 messages
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_platformio_node_publisher"));

    // Initialize a timer with a period of 1 second which calls the function timer_callback() every time it expires
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    // Add our timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Initialize our message data to 0
    msg.data = 0;
}

void loop()
{
    // Wait a little bit
    delay(100);
    // Execute pending tasks in the executor. This will handle all ROS communications.
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
*/