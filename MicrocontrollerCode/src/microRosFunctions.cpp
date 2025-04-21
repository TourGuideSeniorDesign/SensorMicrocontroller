//
// Created by Robbie on 11/20/24.
//
#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#include "JoystickFunctions.h"
#include "FanFunctions.h"
#include "LightFunctions.h"
#include "LidarFunctions.h"
#include <Arduino.h>
#include <hardware/watchdog.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <wheelchair_sensor_msgs/msg/sensors.h>
#include <wheelchair_sensor_msgs/msg/fingerprint.h>
#include <wheelchair_sensor_msgs/msg/fan_speed.h>
#include <wheelchair_sensor_msgs/msg/light.h>
#include <wheelchair_sensor_msgs/msg/lidar.h>


#ifdef ROS_DEBUG
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#endif


rcl_publisher_t sensorPublisher;
rcl_publisher_t fingerprintPublisher;

rcl_subscription_t fanSubscriber;
rcl_subscription_t lightSubscriber;
rcl_subscription_t lidarSubscriber;

wheelchair_sensor_msgs__msg__FanSpeed fanMsg;
wheelchair_sensor_msgs__msg__Light lightMsg;
wheelchair_sensor_msgs__msg__Lidar lidarMsg;
#ifdef ROS
wheelchair_sensor_msgs__msg__Sensors sensorMsg;
#elif ROS_DEBUG
wheelchair_sensor_msgs__msg__RefSpeed msg;
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

boolean agent_connected = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while (1) {
        delay(100);
    }
}

void timer_callback(rcl_timer_t *inputTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
#ifdef ROS
        RCSOFTCHECK(rcl_publish(&sensorPublisher, &sensorMsg, NULL));
#elif ROS_DEBUG
        RCSOFTCHECK(rcl_publish(&sensorPublisher, &msg, NULL));
#endif
    }
}

//TODO add the fan subscriber
#ifdef ROS
boolean microRosSetup(unsigned int timer_timeout, const char *nodeName, const char *sensorTopicName,
                   const char *fingerprintTopicName) {
#elif ROS_DEBUG
    boolean microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* topicName){
#endif
    set_microros_serial_transports(Serial);
    delay(2000);

//    // Check for agent BEFORE proceeding
//    if (!rmw_uros_ping_agent(100, 3)) { // 100ms timeout, 3 retries
//        printf("Micro-ROS agent not found — skipping setup.\n");
//        return false;
//    }

    allocator = rcl_get_default_allocator();

    // Set the domain ID
    //const size_t domain_id = 7; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

#ifdef ROS
    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
        sensorTopicName));
    // Create fingerprint publisher
    RCCHECK(rclc_publisher_init_default(
        &fingerprintPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Fingerprint),
        fingerprintTopicName));

    // create timer,
    //unsigned int timer_timeout = 1;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

#elif ROS_DEBUG
    // create publisher
RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
        topicName));
#endif

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &fanSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, FanSpeed),
        "fan_duty_cycles"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lightSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Light),
        "light"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lidarSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Lidar),
        "lidar"));


    // create executor
    //Number of handles = # timers + # subscriptions + # clients + # services
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &fanSubscriber, &fanMsg, &fan_subscription_callback, ON_NEW_DATA));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lightSubscriber, &lightMsg, &light_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lidarSubscriber, &lidarMsg, &lidar_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));


#ifdef ROS
    sensorMsg.left_speed = 0;
    sensorMsg.right_speed = 0;
#elif ROS_DEBUG
    msg.left_speed = 0;
    msg.right_speed = 0;
#endif
    return true;
}

void checkConnection() {
    // Try spinning and check if connection is alive
//    if(rmw_uros_ping_agent(500, 10) != RMW_RET_OK){
//        watchdog_enable(1, 1); // 1 ms timeout
//        while (1);
//    }

}

//void reconnectAgent() {
//        rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
//
//    // Clean up old resources
//    RCCHECK(rcl_publisher_fini(&sensorPublisher, &node));
//    RCCHECK(rcl_publisher_fini(&fingerprintPublisher, &node));
//    RCCHECK(rcl_subscription_fini(&fanSubscriber, &node));
//    RCCHECK(rcl_subscription_fini(&lightSubscriber, &node));
//    RCCHECK(rcl_subscription_fini(&lidarSubscriber, &node));
//    RCCHECK(rcl_timer_fini(&timer));
//    RCCHECK(rcl_node_fini(&node));
//    RCCHECK(rclc_executor_fini(&executor));
//    RCCHECK(rclc_support_fini(&support));
//
//
//    const char* nodeName = "sensors_node";
//    const char* sensorTopicName = "sensors";
//    const char* fingerprintTopicName = "fingerprint";
//
//    // Attempt to reinitialize
//    if (microRosSetup(1, nodeName, sensorTopicName, fingerprintTopicName)) {
//        agent_connected = true;
//        Serial.println("Reconnected to Micro-ROS agent.");
//    } else {
//        Serial.println("Reconnection failed.");
//    }
//}

#ifdef ROS
void transmitMsg(RefSpeed omegaRef, USData ultrasonicData, PIRSensors pirSensors, FanSpeeds fanSpeeds,
                 IMUData imuData) {
    sensorMsg.left_speed = omegaRef.leftSpeed;
    sensorMsg.right_speed = omegaRef.rightSpeed;
    sensorMsg.ultrasonic_front_0 = ultrasonicData.us_front_0;
    sensorMsg.ultrasonic_front_1 = ultrasonicData.us_front_1;
    sensorMsg.ultrasonic_back = ultrasonicData.us_back;
    sensorMsg.ultrasonic_left = ultrasonicData.us_left;
    sensorMsg.ultrasonic_right = ultrasonicData.us_right;
    sensorMsg.pir_front = pirSensors.pir0;
    sensorMsg.pir_back = pirSensors.pir1;
    sensorMsg.pir_left = pirSensors.pir2;
    sensorMsg.pir_right = pirSensors.pir3;
    sensorMsg.fan_speed_0 = fanSpeeds.fan_speed_0;
    sensorMsg.fan_speed_1 = fanSpeeds.fan_speed_1;
    sensorMsg.fan_speed_2 = fanSpeeds.fan_speed_2;
    sensorMsg.fan_speed_3 = fanSpeeds.fan_speed_3;
    sensorMsg.linear_acceleration_x = imuData.accel_x;
    sensorMsg.linear_acceleration_y = imuData.accel_y;
    sensorMsg.linear_acceleration_z = imuData.accel_z;
    sensorMsg.angular_velocity_x = imuData.gyro_x;
    sensorMsg.angular_velocity_y = imuData.gyro_y;
    sensorMsg.angular_velocity_z = imuData.gyro_z;
    sensorMsg.magnetic_field_x = imuData.mag_x;
    sensorMsg.magnetic_field_y = imuData.mag_y;
    sensorMsg.magnetic_field_z = imuData.mag_z;

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

void publishFingerprint(uint8_t fingerprintID) {
    wheelchair_sensor_msgs__msg__Fingerprint fingerprintMsg;
    fingerprintMsg.user_id = fingerprintID;
    RCSOFTCHECK(rcl_publish(&fingerprintPublisher, &fingerprintMsg, NULL));
}


static void fan_subscription_callback(const void *msgin) {
    const auto *msg = (const wheelchair_sensor_msgs__msg__FanSpeed *) msgin;
    FanDutyCycles duty_cycles{};
    duty_cycles.fan_0_duty_cycle = msg->fan_percent_0;
    duty_cycles.fan_1_duty_cycle = msg->fan_percent_1;
    duty_cycles.fan_2_duty_cycle = msg->fan_percent_2;
    duty_cycles.fan_3_duty_cycle = msg->fan_percent_3;
    setAllFans(duty_cycles);
}

static void light_subscription_callback(const void *msgin) {
    const auto *msg = (const wheelchair_sensor_msgs__msg__Light *) msgin;
    const int lightState = msg->state;
    setLight(lightState);
}

static void lidar_subscription_callback(const void *msgin){
        const auto *msg = (const wheelchair_sensor_msgs__msg__Lidar *) msgin;
        if(msg->state == 0){
            lidarState(false);
        } else if(msg->state == 1){
            lidarState(true);
        }
    }

#elif ROS_DEBUG

void transmitMsg(RefSpeed omegaRef){
    msg.left_speed = omegaRef.leftSpeed;
    msg.right_speed = omegaRef.rightSpeed;

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

#endif

#endif
