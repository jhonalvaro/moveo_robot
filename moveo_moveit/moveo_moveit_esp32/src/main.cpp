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
#include <moveo_moveit/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define la versión del protocolo de rosserial
#define ROS_LIB_VERSION "0.9.2" // Asegúrate de que esta versión coincida con la versión de rosserial en tu sistema ROS

const char* ssid     = "equipo";
const char* password = "equipo391136";
// Set the rosserial socket server IP address
IPAddress server(192,168,0,186);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

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

// Pinza
#define GRIPPER_PIN        15

AccelStepper joint1(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

//instantiate publisher (for debugging purposes)
//ros::Publisher steps("joint_steps_feedback",&msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps) {
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6; //gripper position <0-180>
}

void gripper_cb(const std_msgs::UInt16& cmd_msg) {
  gripper.write(cmd_msg.data);
  // Elimina la línea que utiliza el pin 13
  // digitalWrite(13, HIGH-digitalRead(13)); 
}

//instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps",arm_cb); //subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  //Serial.begin(57600);
  // Elimina la configuración del pin 13 como salida
  // pinMode(13,OUTPUT);
  joint_status = 1;

  // Inicializa la comunicación serial
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  //nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  // Configure gripper servo
  gripper.attach(GRIPPER_PIN);
  
  // Elimina la línea que utiliza el pin 13
  // digitalWrite(13, 1); //toggle led
}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = -joint_step[1]; 
    positions[2] = joint_step[2]; 
    positions[3] = joint_step[3]; 
    positions[4] = -joint_step[4]; 

    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    gripper.write(joint_step[5]);  // move gripper after manipulator reaches goal   
  }
  // Elimina la línea que utiliza el pin 13
  // digitalWrite(13, HIGH-digitalRead(13)); //toggle led
  joint_status = 0;
  
  nh.spinOnce();
  delay(1);
}
