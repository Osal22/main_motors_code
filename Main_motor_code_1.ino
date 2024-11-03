#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <motors_data_msgs_ros1/Motors.h>
#include <motors_data_msgs_ros1/TfFeedback.h>


// Define pins
#define motor_1_pwm D3
#define motor_1_dir1 A5
#define motor_1_dir2 A4
#define ENCODER_PIN_A1 D2
#define ENCODER_PIN_B1 D4

#define motor_2_pwm D5
#define motor_2_dir1 D6
#define motor_2_dir2 D7
#define ENCODER_PIN_A2 D8
#define ENCODER_PIN_B2 D9

#define motor_3_pwm D10
#define motor_3_dir1 D11
#define motor_3_dir2 D12
#define ENCODER_PIN_A3 D13
#define ENCODER_PIN_B3 D14


#define motor_4_pwm D15
#define motor_4_dir1 A0
#define motor_4_dir2 A1
#define ENCODER_PIN_A4 A2
#define ENCODER_PIN_B4 A3

// Variables to store the encoder position and direction
volatile long encoderPosition1 = 0;
volatile int encoderDirection1 = 0;

volatile long encoderPosition2 = 0;
volatile int encoderDirection2 = 0;

volatile long encoderPosition3 = 0;
volatile int encoderDirection3 = 0;

volatile long encoderPosition4 = 0;
volatile int encoderDirection4 = 0;

// ISR to handle encoder interrupt
void encoderISR1();
void encoderISR2();
void encoderISR3();
void encoderISR4();

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
// tf::TransformBroadcaster broadcaster;
const char base_link[] = "/base_link";
const char base_link_[] = "base_link";
const char odom[] = "/odom";
const char odom_[] = "odom";
motors_data_msgs_ros1::TfFeedback odom_tf_data_msg;
ros::Publisher odom_tf_data_pub("odom_tf_data", &odom_tf_data_msg);

// odom related part
const int pulsesPerRevolution = 700;
const double wheelDiameter = 0.06;  // meters
const double distancePerPulse = (PI * wheelDiameter) / pulsesPerRevolution;

void drive_motor(uint8_t pwm, uint8_t dir1, uint8_t dir2, bool dir, uint8_t pwm_val);
void messageCb(const motors_data_msgs_ros1::Motors& motor_msg) {
  drive_motor(motor_1_pwm, motor_1_dir1, motor_1_dir2, motor_msg.dir1, motor_msg.motor1);
  drive_motor(motor_2_pwm, motor_2_dir1, motor_2_dir2, motor_msg.dir2, motor_msg.motor2);
  drive_motor(motor_3_pwm, motor_3_dir1, motor_3_dir2, motor_msg.dir3, motor_msg.motor3);
  drive_motor(motor_4_pwm, motor_4_dir1, motor_4_dir2, motor_msg.dir4, motor_msg.motor4);
}

ros::Subscriber<motors_data_msgs_ros1::Motors> sub("arduino_pub", &messageCb);


void setup() {
  // Set up encoder pins
  pinMode(ENCODER_PIN_A1, INPUT);
  pinMode(ENCODER_PIN_B1, INPUT);
  pinMode(ENCODER_PIN_A2, INPUT);
  pinMode(ENCODER_PIN_B2, INPUT);
  pinMode(ENCODER_PIN_A3, INPUT);
  pinMode(ENCODER_PIN_B3, INPUT);
  pinMode(ENCODER_PIN_A4, INPUT);
  pinMode(ENCODER_PIN_B4, INPUT);

  pinMode(motor_1_pwm, OUTPUT);
  pinMode(motor_1_dir1, OUTPUT);
  pinMode(motor_1_dir2, OUTPUT);
  pinMode(motor_2_pwm, OUTPUT);
  pinMode(motor_2_dir1, OUTPUT);
  pinMode(motor_2_dir2, OUTPUT);
  pinMode(motor_3_pwm, OUTPUT);
  pinMode(motor_3_dir1, OUTPUT);
  pinMode(motor_3_dir2, OUTPUT);
  pinMode(motor_4_pwm, OUTPUT);
  pinMode(motor_4_dir1, OUTPUT);
  pinMode(motor_4_dir2, OUTPUT);

  // Attach interrupt to the encoder A pin (rising change)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A1), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A2), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A3), encoderISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A4), encoderISR4, CHANGE);

  // Initialize Serial Monitor
  Serial.begin(57600);
  nh.initNode();
  // broadcaster.init(nh);
  nh.subscribe(sub);
  nh.advertise(odom_tf_data_pub);
}

void loop() {
  double dist1 = encoderPosition1 * distancePerPulse;
  double dist2 = encoderPosition2 * distancePerPulse;
  double dist3 = encoderPosition3 * distancePerPulse;
  double dist4 = encoderPosition4 * distancePerPulse;

  odom_tf_data_msg.x = (dist4 + dist1 + dist2 + dist3) / 4.0;
  odom_tf_data_msg.y = (0 - dist1 + dist2 + dist3 - dist4) / 4.0;
  odom_tf_data_msg.rot = (0 - dist1 + dist2 - dist3 + dist4) / 4.0;
  odom_tf_data_pub.publish(&odom_tf_data_msg);
  // t.transform.rotation = tf::createQuaternionFromYaw(rot_);
  // t.header.frame_id = odom_;
  // t.child_frame_id = base_link_;
  // t.header.stamp = nh.now();
  // broadcaster.sendTransform(t);
  nh.spinOnce();
}

void encoderISR1() {
  int stateA = digitalRead(ENCODER_PIN_A1);  // Read the A pin state
  int stateB = digitalRead(ENCODER_PIN_B1);  // Read the B pin state

  if (stateA == stateB) {
    encoderPosition1++;     // Clockwise
    encoderDirection1 = 1;  // Clockwise
  } else {
    encoderPosition1--;      // Counterclockwise
    encoderDirection1 = -1;  // Counterclockwise
  }
}

void encoderISR2() {
  int stateA = digitalRead(ENCODER_PIN_A2);  // Read the A pin state
  int stateB = digitalRead(ENCODER_PIN_B2);  // Read the B pin state

  if (stateA == stateB) {
    encoderPosition2++;     // Clockwise
    encoderDirection2 = 1;  // Clockwise
  } else {
    encoderPosition2--;      // Counterclockwise
    encoderDirection2 = -1;  // Counterclockwise
  }
}

void encoderISR3() {
  int stateA = digitalRead(ENCODER_PIN_A3);  // Read the A pin state
  int stateB = digitalRead(ENCODER_PIN_B3);  // Read the B pin state

  if (stateA == stateB) {
    encoderPosition3++;     // Clockwise
    encoderDirection3 = 1;  // Clockwise
  } else {
    encoderPosition3--;      // Counterclockwise
    encoderDirection3 = -1;  // Counterclockwise
  }
}

void encoderISR4() {
  int stateA = digitalRead(ENCODER_PIN_A4);  // Read the A pin state
  int stateB = digitalRead(ENCODER_PIN_B4);  // Read the B pin state

  if (stateA == stateB) {
    encoderPosition4++;     // Clockwise
    encoderDirection4 = 1;  // Clockwise
  } else {
    encoderPosition4--;      // Counterclockwise
    encoderDirection4 = -1;  // Counterclockwise
  }
}

void drive_motor(uint8_t pwm_pin, uint8_t dir1, uint8_t dir2, bool dir, uint8_t pwm_val) {
  if (dir == 1) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  }
  if (dir == 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }
  analogWrite(pwm_pin, pwm_val);
}