#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>


using namespace webots;

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

//////////////////////////////////////////////////////// YASOD & VIDURA ///////////////////////////////////////////////////////////////
float MAX_speed = 7;
float MID_speed = 5;

float error_weight[8] = {-8,-4,-2,-1,1,2,4,8};
float kp = 1;
float kd = 0.5;
float ki = 0.0001;
float pr_error = 0;
float i_v = 0;
int threshold = 300;

void lineFollow(float MAX_speed, float MID_speed){
  bool R3 = (threshold < r3 -> getValue());
  bool R2 = (threshold < r2 -> getValue());
  bool R1 = (threshold < r1 -> getValue());
  bool R0 = (threshold < r0 -> getValue());
  bool L0 = (threshold < l0 -> getValue());
  bool L1 = (threshold < l1 -> getValue());
  bool L2 = (threshold < l2 -> getValue());
  bool L3 = (threshold < l3 -> getValue());
  
  
  
  float error = R3*error_weight[0] + R2*error_weight[1] + R1*error_weight[2] + R0*error_weight[3] + L0*error_weight[4] + L1*error_weight[5] + L2*error_weight[6] + L3*error_weight[7];
  float p_v = error;
  float d_v = error - pr_error;
  i_v = i_v + error;
  
  float PID = (kp*p_v + kd*d_v + ki*i_v)/2;
  pr_error = error;
  
  float right_v = MID_speed - PID;
  float left_v = MID_speed + PID;
  
  if(right_v > MAX_speed){
    right_v = MAX_speed;
  }
  if(right_v < 0 ){
    right_v = 0;
  }
  if(left_v > MAX_speed){
    left_v = MAX_speed;
  }
  if(left_v < 0 ){
    left_v = 0;
  }
  l_motor->setVelocity(left_v);
  r_motor->setVelocity(right_v);
   
  
  }
int mid_speed = 5;
float left_speed=0;
float right_speed=0 ;
int side;
float left_dist_square, right_dist_square, error, d_error, wall_dist, calc_speed;
float error_scale = 0.005;
float P = 2.1;
  // float I = 0;
float D = 20;
float prev_error = 0;
 

void wallFollow(){
    l_motor->setVelocity(left_speed);
    r_motor->setVelocity(right_speed);
    float l_d2 = lft->getValue() ;
    float l_d1 = lbt->getValue() ;
    float r_d1 = rbt->getValue() ;
    float r_d2 = rft->getValue() ;
    //cout << "******" << r_d1 << " " << r_d2 << " " << l_d1 << endl;
    // cout<<"LF "<< l_d2 <<" LR "<< l_d1 <<" RR "<<r_d1<<" RF "<< r_d2 <<endl;
    l_d1 = l_d1 < 1000 ? l_d1 : 1000;
    l_d2 = l_d2 < 1000 ? l_d2 : 1000;
    r_d1 = r_d1 < 1000 ? r_d1 : 1000;
    r_d2 = r_d2 < 1000 ? r_d2 : 1000;
    // delay(10);

    left_dist_square = sqrt(l_d1*l_d1 + 1.732 * l_d1*l_d2 + l_d2*l_d2)/2;
    right_dist_square = sqrt(r_d1*l_d1 + 1.732 * r_d1*r_d2 + r_d2*r_d2)/2;
    // cout << " left " << left_dist_square << " right " << right_dist_square << endl;

    wall_dist = left_dist_square < right_dist_square ? left_dist_square : right_dist_square;
    //cout << "$$$$$$$$$$$$$$$" << wall_dist << endl;
    side = left_dist_square < right_dist_square ? 0: 1;
    error = (600 - wall_dist) * error_scale;
    d_error = error - prev_error;
    calc_speed = (P * error + D * d_error);
    prev_error = error;
    calc_speed = calc_speed < 5 ? calc_speed: 5;
    calc_speed = calc_speed > -5 ? calc_speed: -5;
    
    if (side == 0){
      left_speed = mid_speed + calc_speed;
      right_speed = mid_speed - calc_speed;
      left_speed = left_speed < 10 ? left_speed: 10;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 10 ? right_speed: 10;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
    if (side == 1){
      left_speed = mid_speed - calc_speed;
      right_speed = mid_speed + calc_speed;
      left_speed = left_speed < 10 ? left_speed: 10;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 10 ? right_speed: 10;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
    //cout << "error " << error << " calc_speed " << calc_speed << " speeds " << left_speed << " " << right_speed << endl;
  
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rampNavigation(bool direction){}

bool pillarCount(){}

void rampPathCorrection(){}

void escapeGates(){}

int main(int argc, char **argv) {

  initialize_devices();

  while (robot->step(timeStep) != -1) {
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
