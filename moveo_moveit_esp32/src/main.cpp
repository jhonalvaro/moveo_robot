#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32MultiArray.h>

// Definición de pines para cada articulación
#define JOINT1_ENABLE_PIN  4
#define JOINT1_DIR_PIN     5
#define JOINT1_STEP_PIN    18

#define JOINT2_ENABLE_PIN  19
#define JOINT2_DIR_PIN     21
#define JOINT2_STEP_PIN    22

#define JOINT3_ENABLE_PIN  23
#define JOINT3_DIR_PIN     25
#define JOINT3_STEP_PIN    26

#define JOINT4_ENABLE_PIN  27
#define JOINT4_DIR_PIN     32
#define JOINT4_STEP_PIN    33

#define JOINT5_ENABLE_PIN  12
#define JOINT5_DIR_PIN     13
#define JOINT5_STEP_PIN    14

#define GRIPPER_PIN        15

AccelStepper joint1(1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2(1, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3(1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4(1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
AccelStepper joint5(1, JOINT5_STEP_PIN, JOINT5_DIR_PIN);

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;

ros::NodeHandle nh;

void arm_cb(const std_msgs::Int32MultiArray& arm_steps) {
  joint_status = 1;
  joint_step[0] = arm_steps.data[0];
  joint_step[1] = arm_steps.data[1];
  joint_step[2] = arm_steps.data[2];
  joint_step[3] = arm_steps.data[3];
  joint_step[4] = arm_steps.data[4];
  joint_step[5] = arm_steps.data[5]; // posición de la pinza <0-180>
}

void gripper_cb(const std_msgs::UInt16& cmd_msg) {
  gripper.write(cmd_msg.data); // Establecer ángulo del servo, debe ser de 0-180
  // Alternar LED (comentado porque LED_BUILTIN no está definido)
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

ros::Subscriber<std_msgs::Int32MultiArray> arm_sub("joint_steps", arm_cb);
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb);

void setup() {
  Serial.begin(115200);
  // nh.getHardware()->setBaud(115200); // Eliminado porque no es necesario
  // pinMode(LED_BUILTIN, OUTPUT); // Comentado porque LED_BUILTIN no está definido
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);

  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);

  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  gripper.attach(GRIPPER_PIN);
  
  // digitalWrite(LED_BUILTIN, HIGH); // Comentado porque LED_BUILTIN no está definido
}

void loop() {
  nh.spinOnce();
  delay(1);

  if (joint_status == 1) {
    long positions[5];
    positions[0] = joint_step[0];
    positions[1] = -joint_step[1];
    positions[2] = joint_step[2];
    positions[3] = joint_step[3];
    positions[4] = -joint_step[4];

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition();
    gripper.write(joint_step[5]);
  }
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Comentado porque LED_BUILTIN no está definido
  joint_status = 0;
  
  nh.spinOnce();
  delay(1);
}