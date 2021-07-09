#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>

using namespace webots;
float MAX_SPEED = 10;
float MID_SPEED = 5;
///////// Line follower variables///////////////
float error_weight[8] = {-8,-4,-2,-1,1,2,4,8};
float kp = 1;
float kd = 0.5;
float ki = 0.0001;
float pr_error = 0;
float i_v = 0;
int threshold = 300;

Robot *robot = new Robot();
const int timeStep = (int)robot->getBasicTimeStep();

Motor *l_motor = robot->getMotor("left_motor");
Motor *r_motor = robot->getMotor("right_motor");

PositionSensor *l_enc = robot->getPositionSensor("left_encoder");
PositionSensor *r_enc = robot->getPositionSensor("right_encoder");

DistanceSensor *l3 = robot->getDistanceSensor("l3");
DistanceSensor *l2 = robot->getDistanceSensor("l2");
DistanceSensor *l1 = robot->getDistanceSensor("l1");
DistanceSensor *l0 = robot->getDistanceSensor("l0");
DistanceSensor *r0 = robot->getDistanceSensor("r0");
DistanceSensor *r1 = robot->getDistanceSensor("r1");
DistanceSensor *r2 = robot->getDistanceSensor("r2");
DistanceSensor *r3 = robot->getDistanceSensor("r3");

DistanceSensor *lc = robot->getDistanceSensor("l_corner");
DistanceSensor *rc = robot->getDistanceSensor("r_corner");

DistanceSensor *lft = robot->getDistanceSensor("left_front_tof");
DistanceSensor *lbt = robot->getDistanceSensor("left_back_tof");
DistanceSensor *rft = robot->getDistanceSensor("right_front_tof");
DistanceSensor *rbt = robot->getDistanceSensor("right_back_tof");
DistanceSensor *ct = robot->getDistanceSensor("center_tof");

Gyro *gyro = robot->getGyro("gyro");

void initialize_devices(){
  l_motor->setPosition(INFINITY);
  r_motor->setPosition(-INFINITY);
  l_motor->setVelocity(0.0);
  r_motor->setVelocity(0.0);
  
  l_enc->enable(timeStep);
  r_enc->enable(timeStep);
  
  l3->enable(timeStep);
  l2->enable(timeStep);
  l1->enable(timeStep);
  l0->enable(timeStep);
  r0->enable(timeStep);
  r1->enable(timeStep);
  r2->enable(timeStep);
  r3->enable(timeStep);
  
  lc->enable(timeStep);
  rc->enable(timeStep);
  
  lft->enable(timeStep);
  lbt->enable(timeStep);
  rft->enable(timeStep);
  rbt->enable(timeStep);
  ct->enable(timeStep);
  
  gyro->enable(timeStep);
}

void disable_devices(){
  l_enc->disable();
  r_enc->disable();
  
  l3->disable();
  l2->disable();
  l1->disable();
  l0->disable();
  r0->disable();
  r1->disable();
  r2->disable();
  r3->disable();
  
  lc->disable();
  rc->disable();
  
  lft->disable();
  lbt->disable();
  rft->disable();
  rbt->disable();
  ct->disable();
  
  gyro->disable();
}

void stopRobot(){
  l_motor->setVelocity(0.0);
  r_motor->setVelocity(0.0);
}

void delay(int d){
  while (d--){
    robot->step(timeStep);
  }
}

void moveDistance(double dist){
  double encPos = r_enc->getValue() + l_enc->getValue();
  l_motor->setVelocity(5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() + l_enc->getValue() - encPos < dist){
    robot->step(timeStep);
  }
  stopRobot();
  return;
}

void turnLeft(){
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() - encPos < 5){
    robot->step(timeStep);
  }
  stopRobot();
  return;
}

void turnRight(){
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-5.0);
  l_motor->setVelocity(5.0);
  while (l_enc->getValue() - encPos < 5){
    robot->step(timeStep);
  }
  stopRobot();
  return;
}

void lineFollow(float max_speed, float base_speed){
// how would you handle all black or all white cases - Thiesh
  bool R3 = (threshold < r3 -> getValue());
  bool R2 = (threshold < r2 -> getValue());
  bool R1 = (threshold < r1 -> getValue());
  bool R0 = (threshold < r0 -> getValue());
  bool L0 = (threshold < l0 -> getValue());
  bool L1 = (threshold < l1 -> getValue());
  bool L2 = (threshold < l2 -> getValue());
  bool L3 = (threshold < l3 -> getValue());
    
  float error = R3*error_weight[0] + R2*error_weight[1] + R1*error_weight[2] + R0*error_weight[3] + L0*error_weight[4] + L1*error_weight[5] + L2*error_weight[6] + L3*error_weight[7];
  float d_v = error - pr_error;
  i_v = i_v + error;
  float p_v = error;
  
  float PID = (kp*p_v + kd*d_v + ki*i_v)/2;
  pr_error = error;
  
  float right_v = base_speed - PID;
  float left_v = base_speed + PID;
  
  if(right_v > max_speed){
    right_v = max_speed;
  }
  if(right_v < 0 ){
    right_v = 0;
  }
  if(left_v > max_speed){
    left_v = max_speed;
  }
  if(left_v < 0 ){
    left_v = 0;
  }
  l_motor->setVelocity(left_v);
  r_motor->setVelocity(right_v);
}
void wallFollow(){}

void rampNavigation(bool direction){}

bool pillarCount(){}

void rampPathCorrection(){}

void escapeGates(){}

int main(int argc, char **argv) {

  initialize_devices();

  while (robot->step(timeStep) != -1) {
    
    line_follow(7,5);
    turnLeft();
    std::cout << "Left done" << std::endl;
    delay(256);
    turnRight();
    std::cout << "Right done" << std::endl;
    delay(256);
  };
  
  disable_devices();

  delete robot;
  return 0;
}
