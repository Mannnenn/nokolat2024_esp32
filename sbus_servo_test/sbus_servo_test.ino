#include <ESP32Servo.h>
#include "sbus.h"

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, 16, 17, true); // invertフラグを追加
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2, 16, 17, true); // invertフラグを追加
/* SBUS data */
bfs::SbusData data;

/* PWM pin definitions */
#define THROTTLE_PIN 5
#define RUDDER_PIN 26
#define ELEVATOR_PIN 14
#define AILERON_R_PIN 19
#define AILERON_L_PIN 39
#define DROP_PIN 13

// サーボオブジェクトの作成
Servo servo_throttle;
Servo servo_yaw;
Servo servo_roll_left;
Servo servo_roll_right;
Servo servo_pitch;
Servo servo_drop;

void setup()
{
  /* Serial to display data */
  Serial.begin(9600);
  while (!Serial)
  {
  }
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  servo_throttle.attach(THROTTLE_PIN);
  servo_yaw.attach(RUDDER_PIN);
  servo_roll_left.attach(AILERON_R_PIN);
  servo_roll_right.attach(AILERON_L_PIN);
  servo_pitch.attach(ELEVATOR_PIN);
  servo_drop.attach(DROP_PIN);
}

void loop()
{
  if (sbus_rx.Read())
  {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    //for (int8_t i = 0; i < data.NUM_CH; i++)
    //{
    //  Serial.print(data.ch[i]);
    //  Serial.print("\t");
    //}
    /* Display lost frames and failsafe data */
    //Serial.print(data.lost_frame);
    //Serial.print("\t");
    //Serial.println(data.failsafe);
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(data);

    /* Output PWM signals */
    int dataH = map(data.ch[2],368,1680,1000,2000);
    Serial.println(dataH);
    servo_throttle.writeMicroseconds(dataH);
    servo_yaw.writeMicroseconds(data.ch[3]);
    servo_roll_left.writeMicroseconds(data.ch[0]);
    servo_roll_right.writeMicroseconds(data.ch[5]);
    servo_pitch.writeMicroseconds(data.ch[1]);
    servo_drop.writeMicroseconds(data.ch[4]);
  }
}