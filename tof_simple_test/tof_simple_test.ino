#include <stdio.h>
#include <string.h>
#include <Wire.h>



#include <TFLI2C.h>                          // TFLuna-I2C Library v.0.2.0

// Define the LED pin for error indication
#define LED_PIN 13

#define BNO08x_SDA 21
#define BNO08x_SCL 22

// Define the TFLuna-I2C default address and frame rate
int16_t tfAddr = TFL_DEF_ADR; // use this default I2C address or
uint16_t tfFrame = 100;       // default frame rate

// TFLuna-I2C object
TFLI2C tflI2C;
int16_t tfDist; // distance in centimeters

float z_h;

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

  // Initialize the BN08x
  Wire.begin(BNO08x_SDA, BNO08x_SCL);


  // Initialize the TFLuna-I2C
  Serial.print("Set Frame Rate to: ");
  if (tflI2C.Set_Frame_Rate(tfFrame, tfAddr))
  {
    Serial.println(tfFrame);
  }
  else
    tflI2C.printStatus();

}

void loop()
{
    if (tflI2C.getData(tfDist, tfAddr)) // If read okay...
    {
      z_h= static_cast<float>(tfDist) / 100; // Convert to meters

      Serial.println(z_h);
    }

}