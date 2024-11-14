#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define SERVOMIN  90 
#define SERVOMAX  600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BASE 0       
#define SHOULDER 2   
#define ELBOW 4      
#define WRIST 6      

#define GRIP 8       
#define GRIPPER 10   

#define ECHO 8
#define TRIG 9

ros::NodeHandle nh;

float arm_position[6] = {320, 250, 0, 90, 90, 1000};  // 초기 위치값
int distance = 0;

uint16_t angleToPulse1(float angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

uint16_t angleToPulse(float angle) {
  return map(angle, 0, 90, SERVOMIN, SERVOMAX);
}

// nom_x 함수의 값이 너무 커지는 것을 방지하기 위한 조정
float nom_x(float X) {
  float normalized_X = (X - 320) / 320.0;
  float Y = 0.001 * (normalized_X * normalized_X) + 0.001;
  return Y;
}

float BASE_ANGLE = 345.0;

void setArmCallback(const std_msgs::Float32MultiArray& msg) {
  for (int i = 0; i < 6; i++) {
    arm_position[i] = msg.data[i];
  }
  set_arm(arm_position[0], arm_position[1], distance);
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("set_arm", setArmCallback);

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);

  set_arm(arm_position[0], arm_position[1], distance);
}

void loop() {
  distance = ultrasonic();
  nh.spinOnce();  
  delay(2000);     
}

int ultrasonic() {
  int duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = duration * 17 / 1000;

  Serial.println(distance);
  delay(500);

  return distance;
}

void set_arm(float X, float Z , float Y) {
  float GRIP_DEGREE = 90;
  int _GRIPPER = 1000; 
  int de = 1000;

  float BASE_HEIGHT = 0.0;
  float SHOULDER_LENGTH = 110.0;
  float ARM_LENGTH = 130.0;
  float GRIP_LENGTH = 70.0;
  float Pi = 3.14159;

  // BASE_ANGLE 계산시 변화량을 제한하여 갑작스러운 큰 변화 방지

    float delta_angle;
    if (X > 320.0) {
      //delta_angle = -angleToPulse(degrees(atan((X - 320.0) * 0.014))) * 5 * nom_x(X);
      //if (delta_angle < -10) {
      delta_angle = -10;
    //}
    } else if (X < 320.0) {
      //delta_angle = angleToPulse(degrees(atan((320.0 - X) * 0.014))) * 5 * nom_x(X);
      //if (delta_angle > 10) {
      delta_angle = 10;
      //}
    }


    BASE_ANGLE += delta_angle;
  

  float R = sqrt((X * X) + (Y * Y));
  float GRIP_ANGLE = radians(GRIP_DEGREE);
  float RSIN = sin(GRIP_ANGLE) * GRIP_LENGTH;
  float R1 = R - RSIN;
  float ZCOS = cos(GRIP_ANGLE) * GRIP_LENGTH;
  float Z1 = Z - BASE_HEIGHT + (ZCOS * GRIP_LENGTH);
  float A = sqrt((Z1 * Z1) + (R1 * R1)) / 2;
  float ELBOW_ANGLE = asin(A / ARM_LENGTH) * 2;
  float SHOULDER_ANGLE = atan2(Z1, R1) + ((Pi - ELBOW_ANGLE) / 2);
  float WRIST_ANGLE = Pi + GRIP_ANGLE - SHOULDER_ANGLE - ELBOW_ANGLE;

  pwm.setPWM(BASE, 0, BASE_ANGLE);
  pwm.setPWM(SHOULDER, 0, angleToPulse1(35));
  pwm.setPWM(ELBOW, 0, angleToPulse1(90));
  pwm.setPWM(WRIST, 0, angleToPulse1(135));
  pwm.setPWM(GRIP, 0, angleToPulse1(95));
  pwm.setPWM(GRIPPER, 0, angleToPulse1(_GRIPPER));
  delay(de);
}
