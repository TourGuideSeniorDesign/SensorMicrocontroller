//
// Created by Robbie on 11/20/24.
//
#include "microRosFunctions.h"
#include "JoystickFunctions.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <wheelchair_sensor_msgs/msg/sensors.h>
#include <wheelchair_sensor_msgs/msg/ref_speed.h>

//Publisher
// rcl_publisher_t publisher;
// wheelchair_sensor_msgs__msg__Sensors sensorMsg;
// rclc_executor_t pub_executor;

//Subscriber
rcl_subscription_t subscriber;
#ifdef DEBUG
wheelchair_sensor_msgs__msg__Sensors refSpeedMsg;
#else
wheelchair_sensor_msgs__msg__RefSpeed refSpeedMsg;
#endif
rclc_executor_t executor_sub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

// void timer_callback(rcl_timer_t * inputTimer, int64_t last_call_time)
// {
//     RCLC_UNUSED(last_call_time);
//     if (inputTimer != NULL) {
//         RCSOFTCHECK(rcl_publish(&publisher, &sensorMsg, NULL));
//     }
// }

void subscription_callback(const void *msgin)
{
#ifdef DEBUG
    const wheelchair_sensor_msgs__msg__Sensors *msg = (const wheelchair_sensor_msgs__msg__Sensors *)msgin;
    //Try using Serial1 to use a Uart adapter to print this out
    Serial1.print("Left Speed: ");
    Serial1.println(msg->left_speed);
    Serial1.print("Right Speed: ");
    Serial1.println(msg->right_speed);


    if (msg->left_speed == 100 || msg->right_speed == 100) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
#else
    const wheelchair_sensor_msgs__msg__RefSpeed *msg = (const wheelchair_sensor_msgs__msg__RefSpeed *)msgin;
    refSpeedMsg = *msg;
#endif
}

void microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* subTopicName){
    set_microros_serial_transports(Serial);
    delay(2000);
    allocator = rcl_get_default_allocator();

    // Set the domain ID
    const size_t domain_id = 7; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

//    // create publisher
//    RCCHECK(rclc_publisher_init_default(
//            &publisher,
//            &node,
//            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, sensorMsg, Sensors),
//            topicName));

    // Create Subscriber
    RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
            subTopicName));


    // create timer,
    //unsigned int timer_timeout = 1;
    // RCCHECK(rclc_timer_init_default(
    //         &timer,
    //         &support,
    //         RCL_MS_TO_NS(timer_timeout),
    //         timer_callback));

    // create executor
    // RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
    // RCCHECK(rclc_executor_add_timer(&pub_executor, &timer));

    // create sub executor
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &refSpeedMsg, &subscription_callback, ON_NEW_DATA));

    // sensorMsg.left_speed = 0;
    // sensorMsg.right_speed = 0;
}

// void transmitMsg(refSpeed omegaRef){
//     sensorMsg.left_speed = omegaRef.leftSpeed;
//     sensorMsg.right_speed = omegaRef.rightSpeed;
//
//     RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(10)));
// }

void checkSubs() {
    RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}