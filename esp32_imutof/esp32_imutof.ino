#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <string.h>
#include <Wire.h>

#include <ESP32Servo.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/twist.h>

#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <TFLI2C.h>                          // TFLuna-I2C Library v.0.2.0
#include "sbus.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// Define the LED pin for error indication
#define LED_PIN 13
#define LED_TAPE_PIN 33

// Define the BNO08x INT and RST pins
#define BNO08X_INT 36
#define BNO08X_RST 27

#define BNO08x_SDA 21
#define BNO08x_SCL 22

// Define the BNO08x I2C address
#define BNO08X_ADDR 0x4B // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

/* PWM pin definitions */
// ピンの順番:マイコンを上にしてコネクタに向かって左から順に
//         | 05| 23| 19| 18| 26| 13|
// |3V3|3V3| 5V| 5V| 5V| 5V| 5V| 5V|
// |GND|GND|GND|GND|GND|GND|GND|GND| 33|GND|

#define THROTTLE_PIN 5
#define RUDDER_PIN 19
#define ELEVATOR_PIN 18
#define AILERON_R_PIN 26
#define AILERON_L_PIN 23
#define DROP_PIN 13

// Define the SBUS serial port
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17
#define SBUS_RECEIVE_TIMEOUT 100

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
  SBUS_AUTO_LANDING_MODE = 8,
  SBUS_AUTO_TURN_MODE = 9
};

// Define control mode
enum CONTROL_MODE
{
  MANUAL = 0,
  AUTO_TURNING = 1,
  AUTO_RISE_TURNING = 2,
  AUTO_EIGHT_TURNING = 3,
  AUTO_LANDING = 4,
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

// Define the TFLuna-I2C default address and frame rate
int16_t tfAddr = TFL_DEF_ADR; // use this default I2C address or
uint16_t tfFrame = 100;       // default frame rate

// Handle for the multithreading task
TaskHandle_t thp[1];
TaskHandle_t thp2[1];

// micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher_pose;
rcl_publisher_t publisher_command;
rcl_subscription_t subscriber;

geometry_msgs__msg__Pose msg_pose;
geometry_msgs__msg__Pose msg_command;
geometry_msgs__msg__Twist msg_command_recv;

rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// サーボオブジェクトの作成
Servo servo_throttle;
Servo servo_rudder;
Servo servo_elevator;
Servo servo_aileron_r;
Servo servo_aileron_l;
Servo servo_dropping_device;

// BNO08x object
BNO08x myIMU;

// TFLuna-I2C object
TFLI2C tflI2C;
int16_t tfDist; // distance in centimeters

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SBUS_RX_PIN, SBUS_TX_PIN, true); // invertフラグを追加
bfs::SbusData data;

// Flag for publishing data
bool imu_pose_read_flag = false;

bool tof_read_frag = false;

bool sbus_read_flag = false;

// S.BUSの信号で安全を確認するためのフラグ
bool sbus_safe_flag = true;

// S.BUSを読むUARTが何回か受信できない場合、安全のためにサーボを停止する
uint32_t sbus_safe_count = 0;
bool uart_timeout_flag = true;

// Control mode detection
CONTROL_MODE control_mode = CONTROL_MODE::MANUAL;

// Function to handle errors
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    digitalWrite(LED_TAPE_PIN, !digitalRead(LED_TAPE_PIN));
    // Safety for the servos
    servo_throttle.writeMicroseconds(1000);
    delay(100);
  }
}

// twist message cb
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  if (sbus_safe_flag || uart_timeout_flag)
  {
    // Safety for the servos,throttle off
    servo_throttle.writeMicroseconds(1000);
  }
  else
  {
    servo_throttle.writeMicroseconds(map(msg->linear.x, 368, 1680, 1000, 2000));
    servo_rudder.writeMicroseconds(msg->linear.y);
    servo_elevator.writeMicroseconds(msg->linear.z);
    servo_aileron_r.writeMicroseconds(msg->angular.x);
    servo_aileron_l.writeMicroseconds(msg->angular.y);
    servo_dropping_device.writeMicroseconds(msg->angular.z);
  }
}

void setup()
{
  /* Serial to display data */
  Serial.begin(9600);
  while (!Serial)
  {
  }

  // Set pin for error indication
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_TAPE_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_TAPE_PIN, LOW);

  // Set micro-ROS

  set_microros_wifi_transports("NOKOLAT_ZISO", "ZISO2024", "192.168.1.192", 8888);
  // set_microros_wifi_transports("Coron28","masa1222", "192.168.0.244", 8888);

  delay(500);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_pose,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
      "/imu_tof"));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_command,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
      "/controller"));

  // QoS設定をreliableに設定
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // history設定をKEEP_LASTに設定し、バッファサイズを10に設定
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = 10;

  // create subscriber
  RCCHECK(rclc_subscription_init(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/command",
      &qos_profile));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_command_recv, &subscription_callback, ON_NEW_DATA));

  // Initialize the servos
  servo_throttle.attach(THROTTLE_PIN);
  servo_rudder.attach(RUDDER_PIN);
  servo_elevator.attach(ELEVATOR_PIN);
  servo_aileron_r.attach(AILERON_R_PIN);
  servo_aileron_l.attach(AILERON_L_PIN);
  servo_dropping_device.attach(DROP_PIN);

  // Initialize the BN08x
  Wire.begin(BNO08x_SDA, BNO08x_SCL);

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false)
  {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");
  setReports();

  // Initialize the TFLuna-I2C
  Serial.print("Set Frame Rate to: ");
  if (tflI2C.Set_Frame_Rate(tfFrame, tfAddr))
  {
    Serial.println(tfFrame);
  }
  else
    tflI2C.printStatus();

  // Begin the SBUS communication
  sbus_rx.Begin();

  // Create the task that will handle the IMU data
  xTaskCreatePinnedToCore(Core0a, "Core0a", 2048, NULL, 2, &thp[0], 1);
  xTaskCreatePinnedToCore(Core0b, "Core0b", 2048, NULL, 1, &thp2[0], 1);
}

// Get pose, acceleration, and gyro data
void setReports(void)
{
  Serial.println("Setting desired reports");

  if (myIMU.enableRotationVector(1) == true)
  {
    Serial.println(F("Rotation vector enabled"));
  }
  else
  {
    Serial.println("Could not enable rotation vector");
  }
}

void loop()
{
  // Publish the data if it is ready

  if (imu_pose_read_flag && tof_read_frag)
  {
    RCSOFTCHECK(rcl_publish(&publisher_pose, &msg_pose, NULL));
    imu_pose_read_flag = false;
    tof_read_frag = false;
  }

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

    if (myIMU.wasReset())
    {
      Serial.print("sensor was reset ");
      setReports();
    }

    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true)
    {
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
      {
        msg_pose.orientation.x = myIMU.getQuatI();
        msg_pose.orientation.y = myIMU.getQuatJ();
        msg_pose.orientation.z = myIMU.getQuatK();
        msg_pose.orientation.w = myIMU.getQuatReal();
        imu_pose_read_flag = true;
      }
    }

    if (tflI2C.getData(tfDist, tfAddr)) // If read okay...
    {
      msg_pose.position.z = static_cast<float>(tfDist) / 100; // Convert to meters

      tof_read_frag = true;
    }

    if (sbus_rx.Read())
    {
      /* Grab the received data */
      data = sbus_rx.data();

      // ここで、SBUSの受信が止まっているかどうかを確認する.trueなら受信が止まっているのでスロットルを止める
      sbus_safe_flag = data.failsafe;

      // If manual mode is selected, the manual mode is set any time
      // スイッチBが手前側の場合は、手動モードに設定
      if (data.ch[SBUS_MANUAL_MODE] <= (MODE_DETECTION::TOW_TYPE_MINIMUM + MODE_DETECTION::WIDTH))
      {
        control_mode = CONTROL_MODE::MANUAL;
      }
      // マニュアルがonだったら必ずマニュアルモードに設定
      else
      {
        // スイッチEが奥側の場合は、自動旋回モードに設定
        if (data.ch[SBUS_AUTO_TURN_MODE] <= (MODE_DETECTION::THREE_TYPE_MINIMUM + MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_TURNING;
        }
        // スイッチEが中央の場合は、自動昇降旋回モードに設定
        else if ((MODE_DETECTION::THREE_TYPE_CENTER - MODE_DETECTION::WIDTH) <= data.ch[SBUS_AUTO_TURN_MODE] && data.ch[SBUS_AUTO_TURN_MODE] <= (MODE_DETECTION::THREE_TYPE_CENTER + MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_RISE_TURNING;
        }
        // スイッチEが手前側の場合は、自動8字旋回モードに設定
        else if (data.ch[SBUS_AUTO_TURN_MODE] >= (MODE_DETECTION::THREE_TYPE_MAXIMUM - MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_EIGHT_TURNING;
        }

        // スイッチFが奥側の場合は、スイッチEによらず自動着陸モードに設定
        if (data.ch[SBUS_AUTO_LANDING_MODE] <= (MODE_DETECTION::TOW_TYPE_MINIMUM + MODE_DETECTION::WIDTH))
        {
          control_mode = CONTROL_MODE::AUTO_LANDING;
        }
      }

      if (control_mode == CONTROL_MODE::MANUAL)
      {

        // フェイルセーフがかかっている場合は、サーボを0にしておく
        if (sbus_safe_flag || uart_timeout_flag)
        {
          // Safety for the servos,throttle off
          servo_throttle.writeMicroseconds(1000);
        }
        else
        {
          servo_throttle.writeMicroseconds(map(data.ch[SBUS_THROTTLE], 368, 1680, 1000, 2000));
          servo_rudder.writeMicroseconds(data.ch[SBUS_RUDDER]);
          servo_elevator.writeMicroseconds(data.ch[SBUS_ELEVATOR]);
          servo_aileron_r.writeMicroseconds(data.ch[SBUS_AILERON_R]);
          servo_aileron_l.writeMicroseconds(data.ch[SBUS_AILERON_L]);
          servo_dropping_device.writeMicroseconds(data.ch[SBUS_DROPPING_DEVICE]);
        }

        msg_command.position.x = data.ch[SBUS_THROTTLE];
        msg_command.position.y = data.ch[SBUS_RUDDER];
        msg_command.position.z = data.ch[SBUS_ELEVATOR];
        msg_command.orientation.x = data.ch[SBUS_AILERON_R];
        msg_command.orientation.y = data.ch[SBUS_AILERON_L];
        msg_command.orientation.z = data.ch[SBUS_DROPPING_DEVICE];

        // マニュアルモードの場合はLEDを消灯する
        digitalWrite(LED_TAPE_PIN, LOW);
      }
      else
      {
        // マニュアルモード以外ではLEDを点灯する
        digitalWrite(LED_TAPE_PIN, HIGH);
      }

      msg_command.orientation.w = control_mode;

      sbus_read_flag = true;

      // 受信がうまくいっている場合は、カウントをリセットする
      sbus_safe_count = 0;
      uart_timeout_flag = false;
    }
    else
    {
      sbus_safe_count++;
      if (sbus_safe_count > SBUS_RECEIVE_TIMEOUT)
      {
        uart_timeout_flag = true;
      }
    }
  }
}

void Core0b(void *args)
{
  while (1)
  {
    // Publish the data if it is ready

    if (control_mode != CONTROL_MODE::MANUAL)
    {
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    }
    else
    {
      delay(1);
    }
  }
}