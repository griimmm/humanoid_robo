/*
  Hubert robot test program.

  This sketch control Hubert's servos.
  It implements command line interface via ROS2, for setting the position of the servo using subscribe.
  Also, it publishes the current pwm value through rosserial. 

  Command interface:
  >> roscore
  >> rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
  Note: correct port must be selected (ttyACM0, 1 or whatever)
  Same baud rate as in Arduino sketch (below)
  >> ros2 topic echo publisher
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
#include <std_msgs/msg/u_int16_multi_array.h>

std_msgs__msg__UInt16MultiArray tester, pwm_tester;

rcl_node_t node;

// //Allocator
rcl_allocator_t allocator;
rclc_support_t support;

// // create executor
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} //printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;
// // #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
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

const int pos_init[] = {1700, 1500, 2000, 2200, 1650, 1600};
int curr_pos[6];
int new_servo_val[6];

const int pos_min[] = {560, 550, 950, 750, 550, 550};
const int pos_max[] = {2330, 2340, 2400, 2200, 2400, 2150};



// //Publisher
rcl_publisher_t publisher_multi;

// //Subscriber
rcl_subscription_t tester_multi;


//ROS-setup
void multi_arr_cb(const void * msgin){

  int delta = 6;
  // uint NewPwm[6];
  uint16_t *NewPwm[6], diff[6], steps[6], now[6], CurrPwm[6];
  const std_msgs__msg__UInt16MultiArray * cmd_msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  

  //current servo value
  for (int i=0; i<6; i++){
    now[i] = curr_pos[i];
    CurrPwm[i] = now[i];
  }
  
  
  NewPwm[0] = cmd_msg->data[0].data;

  /* determine interation "diff" from old to new position */
  diff = (&NewPwm - CurrPwm)/abs(NewPwm - CurrPwm); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
	now = now + delta*diff;
	body.writeMicroseconds(now[0]);
  headPan.writeMicroseconds(now[1]);
  headTilt.writeMicroseconds(now[2]);
  shoulder.writeMicroseconds(now[3])
  elbow.writeMicroseconds(now[4])
  gripper.writeMicroseconds(now[5])

	//Publishing data
	Pwm.data = now;//ZZZ
  RCSOFTCHECK(rcl_publish(&publisher, &Pwm, NULL));
	delay(20);
  }
  curr_pos = now;
  delay(10);

}

//loop or main ?? main replaces setup and loop
//Recheck structure for loop and setup
// int main()
void setup() {
  
  // Serial.begin(9600); // Starts the serial communication
	set_microros_transports();
  // // create init_options
	allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// // create node
	
	RCCHECK(rclc_node_init_default(&node, "hubert", "", &support));

	// // create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_multi,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
		"robot_state_publisher"));

  // Serial.println("Check");
  // Update callbacks according to ros2 conventions
	// create subscriber servo_body
  
	RCCHECK(rclc_subscription_init_default(
		&servo_body_ex,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "servo_body"));

  
	
	RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &tester_multi, &tester, &multi_arr_cb, ON_NEW_DATA));


  // //Attach each joint servo
	// //and write each init position
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

	
}

void loop() {
  
  // Serial.println("Welcome");
  RCCHECK(rclc_executor_spin(&executor));
  delay(250);
  // RCCHECK(rcl_subscription_fini(&servo_body_ex, &node));
  
  // RCCHECK(rcl_publisher_fini(&publisher, &node));

	// RCCHECK(rcl_node_fini(&node));
}