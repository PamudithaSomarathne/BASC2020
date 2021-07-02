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

void lineFollow(){}

void wallFollow(){}

void rampNavigation(bool direction){}

bool pillarCount(){}

void rampPathCorrection(){}

///////////////////////////////////// THARINDU /////////////////////////////////////////////////
void escapeGates(){
  //this should call only when we come across the first cross line
  
  double thresh1 = 600;  //threshold distance(issarhen) depends on sensor
  double thresh2 = 1500;
  
  double ctread=0;  //front ct reading initialized less than thresh1
  
  
  while (ctread<=thresh1 || thresh2<=ctread){
    ctread = ct->getValue();
    robot -> step(timeStep);
    //just waitwithout goint forward  
  }
  while (thresh1<=ctread && ctread<=thresh2){
    ctread = ct->getValue();
    //line follow forward
    moveDistance(5.0);
    robot -> step(timeStep);
  }
  moveDistance(42);
  //escaping to this level means the second gate just opened
  //line follow forward full speed ahead.
}
////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {

  initialize_devices();

  while (robot->step(timeStep) != -1) {
    if (int(robot->getTime()) > 14){
      escapeGates();
    }
  };
  
  disable_devices();

  delete robot;
  return 0;
}
