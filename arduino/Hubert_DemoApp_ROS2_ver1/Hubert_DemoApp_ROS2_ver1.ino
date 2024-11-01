/*
  Hubert robot test program.

  This sketch control Hubert's servos.
  It implements command line interface via ROS2, for setting the position of the servo using subscribe.
  Also, it publishes the current pwm value through rosserial. 

  Command interface:
  >> ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
  Note: correct port must be selected (ttyACM0, 1 or whatever)
  >> ros2 topic echo /publisher
  >> ros2 topic pub /servo_body std_msgs/UInt16 data:\ <angle>\  
  >> ros2 topic pub /servo_neck_pan std_msgs/UInt16 data:\ <angle>\
  >> ros2 topic pub /servo_neck_tilt std_msgs/UInt16 data:\ <angle>\
  >> ros2 topic pub /servo_shoulder std_msgs/UInt16 data:\ <angle>\
  >> ros2 topic pub /servo_elbow std_msgs/UInt16 data:\ <angle>\
  >> ros2 topic pub /servo_gripper std_msgs/UInt16 data:\ <angle>\

  Replace <angle> with a write.microseconds value in approx. [500,2350], mid pos.: 1350/1420
  NOTE: Go back to NeutralPos. before terminating.
  
  created 23 Jun 2020
  by Krister Wolff
  modified 23 Jun 2020
  by Krister Wolff
  modified 20 September 2024
  Abishek Swaminathan

  This example code is in the public domain.

  http://www.arduino.cc/en/
*/

#include <Servo.h>
// #include <ros2arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>

std_msgs__msg__UInt16 body_msg;
std_msgs__msg__UInt16 neck_pan_msg;
std_msgs__msg__UInt16 neck_tilt_msg;
std_msgs__msg__UInt16 shoulder_msg;
std_msgs__msg__UInt16 elbow_msg; 
std_msgs__msg__UInt16 grip_msg;
std_msgs__msg__UInt16 Pwm;

//Node
rcl_node_t node;

//Allocator
rcl_allocator_t allocator;
rclc_support_t support;

// create executor
rclc_executor_t executor;

// //Publisher
rcl_publisher_t publisher;

// //Subscriber
rcl_subscription_t servo_body_ex;
rcl_subscription_t servo_neck_pan;
rcl_subscription_t servo_neck_tilt;
rcl_subscription_t servo_shoulder;
rcl_subscription_t servo_elbow;
rcl_subscription_t servo_gripper_ex;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} 
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// //Servos
Servo body;
Servo headPan;
Servo headTilt;
Servo shoulder;
Servo elbow;
Servo gripper;

// //Init position of all servos
const int servo_pins[] = {3, 5, 6, 9, 10, 11};

const int pos_init[] = {1420, 1500, 1800, 2200, 1650, 1600}; //{1700, 1500, 2000, 2200, 1650, 1600};
int curr_pos[6];
int new_servo_val[6];

const int pos_min[] = {560, 550, 950, 750, 550, 550};
const int pos_max[] = {2330, 2340, 2400, 2200, 2400, 2150};


//For error handling
void error_loop(){
  while(1){
    delay(100);
  }
}

//ROS-setup
void servo_body_ex_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[0];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	body.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
  RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));
	delay(20);
  }
  curr_pos[0] = now;
  delay(10);

  // node.loginfo("GOT DATA MOVE BODY");
}

void servo_neck_pan_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[1];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	headPan.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
  RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));
	delay(20);
  }
  curr_pos[1] = now;
  delay(10);
  
	// hubert.loginfo("GOT DATA MOVE SERVO NECK PAN");
}

void servo_neck_tilt_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[2];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	headTilt.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
	RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));
	delay(20);
  }
  curr_pos[2] = now;
  delay(10);

	// hubert.loginfo("GOT DATA MOVE SERVO NECK TILT");
}

void servo_shoulder_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[3];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	shoulder.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
	RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));//ZZZ
	delay(20);
  }
  curr_pos[3] = now;
  delay(10);
  
	// hubert.loginfo("GOT DATA MOVE SERVO SHOULDER");
}

void servo_elbow_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[4];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	elbow.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
	RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));//ZZZ
	delay(20);
  }
  curr_pos[4] = now;
  delay(10);

  // hubert.loginfo("GOT DATA MOVE SERVO ELBOW");
}

void servo_gripper_ex_cb(const void * msgin) {

  int diff, steps, now, CurrPwm, NewPwm, delta = 6;
  const std_msgs__msg__UInt16 * cmd_msg = (const std_msgs__msg__UInt16 *)msgin;
  //current servo value
  now = curr_pos[5];
  CurrPwm = now;
  NewPwm = cmd_msg->data;

  /* determine interation "diff" from old to new position */
  diff = (NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	gripper.writeMicroseconds(now);
	//Publishing data
	Pwm.data = now;//ZZZ
	RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));//ZZZ
	delay(20);
  }
  curr_pos[5] = now;
  delay(10);
  
  // hubert.loginfo("GOT DATA MOVE SERVO GRIPPER");
}


// int main()
void setup() {
  
	set_microros_transports();
  delay(100);
  // create init_options
	allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	
	RCCHECK(rclc_node_init_default(&node, "hubert", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"robot_state_publisher"));


	// create subscriber servo_body
  RCCHECK(rclc_subscription_init_default(
		&servo_body_ex,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "servo_body"));

	// create subscriber servo_neck_pan
	RCCHECK(rclc_subscription_init_default(
		&servo_neck_pan,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"servo_neck_pan"));

	// create subscriber servo_neck_tilt
	RCCHECK(rclc_subscription_init_default(
		&servo_neck_tilt,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"servo_neck_tilt"));
	// create subscriber servo_shoulder
	RCCHECK(rclc_subscription_init_default(
		&servo_shoulder,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"servo_shoulder"));
	// create subscriber servo_elbow
	RCCHECK(rclc_subscription_init_default(
		&servo_elbow,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"servo_elbow"));
	
	// create subscriber 6
	RCCHECK(rclc_subscription_init_default(
		&servo_gripper_ex,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
		"servo_gripper"));

	
	
  //Attach each joint servo
	//and write each init position
  body.attach(servo_pins[0]);
  body.writeMicroseconds(pos_init[0]);
  
  headPan.attach(servo_pins[1]);
  headPan.writeMicroseconds(pos_init[1]);
  
  headTilt.attach(servo_pins[2]);
  headTilt.writeMicroseconds(pos_init[2]);

  shoulder.attach(servo_pins[3]);
	shoulder.writeMicroseconds(pos_init[3]);

	elbow.attach(servo_pins[4]);
	elbow.writeMicroseconds(pos_init[4]);
	
	gripper.attach(servo_pins[5]);
  gripper.writeMicroseconds(pos_init[5]);

  // //Initilize curr_pos and new_servo_val vectors
  byte i;
  for (i=0; i<(sizeof(pos_init)/sizeof(int)); i++){
	curr_pos[i] = pos_init[i];
	new_servo_val[i] = curr_pos[i];
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  
	RCCHECK(rclc_executor_add_subscription(&executor, &servo_body_ex, &body_msg, &servo_body_ex_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_neck_pan, &neck_pan_msg, &servo_neck_pan_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_neck_tilt, &neck_tilt_msg, &servo_neck_tilt_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_shoulder, &shoulder_msg, &servo_shoulder_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_elbow, &elbow_msg, &servo_elbow_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_gripper_ex, &grip_msg, &servo_gripper_ex_cb, ON_NEW_DATA));

	
}

void loop() {
  
  // Serial.println("Welcome");
  rclc_executor_spin(&executor);
  // delay(250);
  
}