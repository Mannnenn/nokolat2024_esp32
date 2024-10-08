#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

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

// パブリッシャーの作成

// micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;
rcl_init_options_t init_options;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// メッセージの作成

// メッセージの値を設定

#define LED_PIN 13

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#include <TFLI2C.h> // TFLuna-I2C Library v.0.2.0

TFLI2C tflI2C;

int16_t tfDist;                 // distance in centimeters
int16_t tfAddr = TFL_DEF_ADR;   // use this default I2C address or
uint16_t tfFrame = TFL_DEF_FPS; // default frame rate

#include <Wire.h>
#include <string.h>

#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
// Define the BNO08x INT and RST pins
#define BNO08X_INT 36
#define BNO08X_RST 26

#define BNO08x_SDA 21
#define BNO08x_SCL 22

#define BNO08X_ADDR 0x4B // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
// #define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// timestamp

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup()
{

  Serial.begin(9600);

  while (!Serial)
    delay(10); // Wait for Serial to become available.
               // Necessary for boards with native USB (like the SAMD51 Thing+).
               // For a final version of a project that does not need serial debug (or a USB cable plugged in),
               // Comment out this while loop, or it will prevent the remaining code from running.

  Serial.print("Set Frame Rate to: ");
  if (tflI2C.Set_Frame_Rate(tfFrame, tfAddr))
  {
    Serial.println(tfFrame);
  }
  else
    tflI2C.printStatus();

  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin(BNO08x_SDA, BNO08x_SCL);

  // if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false)
  {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);

  set_microros_wifi_transports("NOKOLAT_ZISO", "ZISO2024", "192.168.1.192", 8888);
  // set_microros_wifi_transports("Coron28","masa1222", "192.168.0.244", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu"));
}

// Here is where you define the sensor outputs you want to receive
void setReports(void)
{
  Serial.println("Setting desired reports");

  if (myIMU.enableRotationVector() == true)
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

  if (myIMU.wasReset())
  {
    Serial.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true)
  {

    int reportID = myIMU.getSensorEventID();

    switch (reportID)
    {
    case SENSOR_REPORTID_ROTATION_VECTOR:
      msg.orientation.x = myIMU.getQuatI();
      msg.orientation.y = myIMU.getQuatJ();
      msg.orientation.z = myIMU.getQuatK();
      msg.orientation.w = myIMU.getQuatReal();
      break;
    default:
      break;
    }
  }

  if (tflI2C.getData(tfDist, tfAddr)) // If read okay...
  {
    msg.angular_velocity_covariance[0] = static_cast<float>(tfDist) / 100; // Convert to meters
  }

  unsigned long now_micros = micros();

  msg.header.stamp.sec = now_micros / 1000000;
  msg.header.stamp.nanosec = (now_micros % 1000000) * 1000;

  msg.header.frame_id.data = (char *)"imu"; // フレームIDを設定
  msg.header.frame_id.size = strlen(msg.header.frame_id.data);
  msg.header.frame_id.capacity = msg.header.frame_id.size + 1;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}
