// arduino_servo_controller.ino
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Servo.h>

// Servo objects
Servo servo1, servo2, servo3, servo4;
Servo servos[4] = {servo1, servo2, servo3, servo4};
int servo_pins[4] = {3, 5, 6, 9}; // PWM pins for servos

// Joint positions (radians)
float joint_positions[4] = {0, 0, 0, 0};

ros::NodeHandle nh;

void joint_commands_callback(const std_msgs::Float64MultiArray& msg) {
  for (int i = 0; i < 4 && i < msg.data_length; i++) {
    // Convert radians to servo degrees (0-180)
    float degrees = (msg.data[i] * 180.0 / PI) + 90;
    degrees = constrain(degrees, 0, 180);
    
    servos[i].write(degrees);
    joint_positions[i] = msg.data[i];
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub("joint_commands", &joint_commands_callback);

// Joint state publisher
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_states", &joint_state_msg);

void setup() {
  // Initialize servos
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servo_pins[i]);
    servos[i].write(90); // Neutral position
  }
  
  // Initialize ROS
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(joint_state_pub);
  
  // Setup joint state message
  joint_state_msg.name_length = 4;
  joint_state_msg.position_length = 4;
  joint_state_msg.name = (char*[]){"joint1", "joint2", "joint3", "joint4"};
  joint_state_msg.position = joint_positions;
}

void loop() {
  // Update joint state message
  joint_state_msg.header.stamp = nh.now();
  joint_state_pub.publish(&joint_state_msg);
  
  nh.spinOnce();
  delay(50); // 20Hz update rate
}
