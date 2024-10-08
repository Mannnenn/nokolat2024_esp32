#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <string.h>
#include <Wire.h>

#include <ESP32Servo.h>

#include <geometry_msgs/msg/pose.h>

#include "sbus.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define RCCHECK(fn)              
  {                              
    rcl_ret_t temp_rc = fn;      
    if ((temp_rc != RCL_RET_OK)) 
    {                            
      error_loop();              
    }                            
  }
#define RCSOFTCHECK(fn)          
  {                              
    rcl_ret_t temp_rc = fn;      
    if ((temp_rc != RCL_RET_OK)) 
    {                            
    }                            
  }

// Define the LED pin for error indication
#define LED_PIN 13



/* PWM pin definitions */
#define THROTTLE_PIN 18
#define RUDDER_PIN 19
#define ELEVATOR_PIN 5
#define AILERON_R_PIN 13
#define AILERON_L_PIN 26
#define DROP_PIN 23

// Define the SBUS serial port
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17

// Define the S.BUS array mean to be used
enum SBUS_CHANNELS
{
  SBUS_THROTTLE = 2,
  SBUS_RUDDER = 3,
  SBUS_ELEVATOR = 1,
  SBUS_AILERON_R = 0,
  SBUS_AILERON_L = 5,
  SBUS_DROPPING_DEVICE = 7,
  SBUS_MANUAL_MODE = 6,
  SBUS_AUTOPILOT_MODE = 9,
};

// Define control mode
enum CONTROL_MODE
{
  MANUAL = 0,
  AUTO_TURNING = 1,
  AUTO_RISE_TURNING = 2,
  AUTO_LANDING = 3,
  AUTO_EIGHT = 4,
};

enum MODE_DETECTION
{
  // スイッチが2段階で切り替わる場合
  TOW_TYPE_MINIMUM = 144,
  TOW_TYPE_MAXIMUM = 1904,

  // スイッチが3段階で切り替わる場合
  THREE_TYPE_MINIMUM = 144,
  THREE_TYPE_CENTER = 1024,
  THREE_TYPE_MAXIMUM = 1904,

  // もしかしたらちょっとずれるかもしれないので、幅を持たせてモード切り替え
  WIDTH = 10,
};


// Handle for the multithreading task
TaskHandle_t thp[1];

// micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher_command;

geometry_msgs__msg__Pose msg_command;

rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, 16, 17, true); // invertフラグを追加
bfs::SbusData data;

bool sbus_read_flag = false;


// Control mode detection
CONTROL_MODE control_mode = CONTROL_MODE::MANUAL;

// Function to handle errors
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

  /* Serial to display data */
  Serial.begin(9600);
  while (!Serial)
  {
  }

  // Set pin for error indication
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Set micro-ROS

  set_microros_wifi_transports("NOKOLAT_ZISO", "ZISO2024", "192.168.1.192", 8888);
  // set_microros_wifi_transports("Coron28","masa1222", "192.168.0.244", 8888);

  delay(500);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));


  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_command,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
      "controller"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));


  // Begin the SBUS communication
  sbus_rx.Begin();

  // Create the task that will handle the IMU data
  xTaskCreatePinnedToCore(Core0a, "Core0a", 2048, NULL, 2, &thp[0], 1);
}


void loop()
{
  // Publish the data if it is ready


  if (sbus_read_flag)
  {
    RCSOFTCHECK(rcl_publish(&publisher_command, &msg_command, NULL));
    sbus_read_flag = false;
  }
}

void Core0a(void *args)
{
  while (1)
  {

    if (sbus_rx.Read())
    {
      /* Grab the received data */
      data = sbus_rx.data();

      // If manual mode is selected, the manual mode is set any time
      // スイッチBが手前側の場合は、手動モードに設定
      if (data.ch[SBUS_MANUAL_MODE] <= (MODE_DETECTION::TOW_TYPE_MINIMUM + MODE_DETECTION::WIDTH))
      {
        control_mode = CONTROL_MODE::MANUAL;
      }
      // マニュアルが音だったら必ずマニュアルモードに設定
      else
      {
        // スイッチEが手前側の場合は、自動旋回モードに設定
        if (data.ch[SBUS_AUTOPILOT_MODE] <= (MODE_DETECTION::THREE_TYPE_MINIMUM + MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_TURNING;
        }
        // スイッチEが中央の場合は、自動昇降旋回モードに設定
        else if ((MODE_DETECTION::THREE_TYPE_CENTER - MODE_DETECTION::WIDTH) <= data.ch[SBUS_AUTOPILOT_MODE] && data.ch[SBUS_AUTOPILOT_MODE] <= (MODE_DETECTION::THREE_TYPE_CENTER + MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_RISE_TURNING;
        }
        // スイッチEが奥側の場合は、自動着陸モードに設定
        else if (data.ch[SBUS_AUTOPILOT_MODE] >= (MODE_DETECTION::THREE_TYPE_MAXIMUM - MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_LANDING;
        }
      }

      if (control_mode == CONTROL_MODE::MANUAL)
      {
        msg_command.position.x = data.ch[SBUS_THROTTLE];
        msg_command.position.y = data.ch[SBUS_ELEVATOR];
        msg_command.position.z = data.ch[SBUS_RUDDER];
        msg_command.orientation.x = data.ch[SBUS_AILERON_R];
        msg_command.orientation.y = data.ch[SBUS_AILERON_L];
        msg_command.orientation.z = data.ch[SBUS_DROPPING_DEVICE];
      }

      msg_command.orientation.w = control_mode;

      sbus_read_flag = true;
    }
  }