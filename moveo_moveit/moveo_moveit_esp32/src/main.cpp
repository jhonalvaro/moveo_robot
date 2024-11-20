/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <WiFi.h>
#include <ros.h>
#include <ArduinoTcpHardware.h>
#include <moveo_moveit/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Joint 1
#define E1_STEP_PIN        18
#define E1_DIR_PIN         5
#define E1_ENABLE_PIN      4

// Joint 2
#define Z_STEP_PIN         22
#define Z_DIR_PIN          21
#define Z_ENABLE_PIN       19

// Joint 3
#define Y_STEP_PIN         26
#define Y_DIR_PIN          25
#define Y_ENABLE_PIN       23

// Joint 4
#define X_STEP_PIN         33
#define X_DIR_PIN          32
#define X_ENABLE_PIN       27

// Joint 5 
#define E0_STEP_PIN        13
#define E0_DIR_PIN         12
#define E0_ENABLE_PIN      14

// Gripper
#define GRIPPER_PIN        15

AccelStepper joint1(1, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;

// Configura los detalles de la red WiFi
const char* ssid = "tured"; // Cambia esto a tu SSID
const char* password = "tucontrasena"; // Cambia esto a tu contraseña

// Configura la dirección IP y el puerto del servidor ROS
IPAddress server_ip(192, 168, 0, 182); // Cambia esto a la IP de tu servidor ROS
const uint16_t server_port = 11411; // Puerto por defecto para rosserial

// Inicializa el nodo ROS
ros::NodeHandle nh;
std_msgs::Int16 msg;

// Define el publicador
ros::Publisher steps_pub("joint_steps_feedback", &msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps) {
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6; //gripper position <0-180>

  // Publica en el topic joint_steps_feedback
  msg.data = joint_step[0]; // Puedes cambiar esto para publicar otros datos
  steps_pub.publish(&msg);
}

void gripper_cb(const std_msgs::UInt16& cmd_msg) {
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH - digitalRead(13));  // Toggle led  
}

// Instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps", arm_cb); // Subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); // Subscribes to gripper position

void setup() {
  // Conecta a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Inicializa el nodo ROS
  nh.getHardware()->setConnection(server_ip, server_port);
  nh.initNode();

  // Configura los pines y suscripciones
  pinMode(13, OUTPUT);
  joint_status = 1;
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  nh.advertise(steps_pub);

  // Configura cada motor paso a paso
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);

  // Añade los motores a MultiStepper para gestionarlos
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  // Configura el servo del gripper
  gripper.attach(11);
  digitalWrite(13, 1); // Enciende el LED
}

void loop() {
  nh.spinOnce();
  delay(1);
}