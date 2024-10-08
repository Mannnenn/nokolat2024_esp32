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


#include "SparkFun_BNO08x_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
#include <TFLI2C.h>                          // TFLuna-I2C Library v.0.2.0

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

// Define the BNO08x INT and RST pins
#define BNO08X_INT 36
#define BNO08X_RST 26

#define BNO08x_SDA 21
#define BNO08x_SCL 22

// Define the BNO08x I2C address
#define BNO08X_ADDR 0x4B // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B


// Define the TFLuna-I2C default address and frame rate
int16_t tfAddr = TFL_DEF_ADR; // use this default I2C address or
uint16_t tfFrame = 100;       // default frame rate

// Handle for the multithreading task
TaskHandle_t thp[1];

// micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher_pose;

geometry_msgs__msg__Pose msg_pose;

rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// BNO08x object
BNO08x myIMU;

// TFLuna-I2C object
TFLI2C tflI2C;
int16_t tfDist; // distance in centimeters


// Flag for publishing data
bool imu_pose_read_flag = false;

bool tof_read_frag = false;


// Function to handle errors
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
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

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_pose,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
      "imu_tof"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

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

  // Create the task that will handle the IMU data
  xTaskCreatePinnedToCore(Core0a, "Core0a", 2048, NULL, 2, &thp[0], 1);
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

  }
}
