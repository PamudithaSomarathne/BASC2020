#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>

using namespace webots;
float MAX_SPEED = 10;
float MID_SPEED = 5;


Robot *robot = new Robot();
const int timeStep = (int)robot->getBasicTimeStep();

Motor *l_motor = robot->getMotor("left_motor");
Motor *r_motor = robot->getMotor("right_motor");
Motor *m_servo = robot->getMotor("main_servo");
Motor *s_servo = robot->getMotor("second_servo");


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

LED *led_1= robot->getLED("led1");
LED *led_2= robot->getLED("led2");

void initialize_devices(){
  l_motor->setPosition(INFINITY);
  r_motor->setPosition(-INFINITY);
  m_servo->setPosition(0);
  s_servo->setPosition(0);
  l_motor->setVelocity(0.0);
  r_motor->setVelocity(0.0);
  m_servo->setVelocity(0.0);
  s_servo->setVelocity(0.0);
  
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

void delay(int d){
  while (d--){
    robot->step(timeStep);
  }
}

void stopRobot(){
  l_motor->setVelocity(0.0);
  r_motor->setVelocity(0.0);
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
  moveDistance(15);
  stopRobot();
  delay(50);
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() - encPos < 7){
    std::cout << "left_turn" << std::endl;
    std::cout << r_enc->getValue() - encPos << std::endl;
    robot->step(timeStep);
  }
  stopRobot();
  delay(50);
  return;
}

void turnRight(){
  moveDistance(15);
  stopRobot();
  delay(50);
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-5.0);
  l_motor->setVelocity(5.0);
  while (l_enc->getValue() - encPos < 7){
    robot->step(timeStep);
    std::cout << "right_turn" << std::endl;
    std::cout << l_enc->getValue() - encPos << std::endl;
  }
  stopRobot();
  delay(50);
  return;
}




float error_weight[8] = {1,2,3,4,5,6,7,8}; //make this positive
float kp = 2;
float kd = 45;
float ki = 0.001;
float pr_error = 0;
float i_v = 0;
int threshold = 900;
void lineFollow(float max_speed, float base_speed){

// how would you handle all black or all white cases - Thiesh
// Handle 90 degrees within this function else call
// Addd all black all white cases
// handle line lost case: can do backwards search
// weight list can be updated with exact distances from robot sensor panel, makes the error real world relatable in centimeters
  bool R3 = (threshold < r3 -> getValue()); //white -0 black -1
  bool R2 = (threshold < r2 -> getValue());
  bool R1 = (threshold < r1 -> getValue());
  bool R0 = (threshold < r0 -> getValue());
  bool L0 = (threshold < l0 -> getValue());
  bool L1 = (threshold < l1 -> getValue());
  bool L2 = (threshold < l2 -> getValue());
  bool L3 = (threshold < l3 -> getValue());
  
  //normalize, ideal error can be 10% of max speed- allows us to have kp ~ 1-10 
  //define variables outside the loop 
  float error = (R3*error_weight[0] + R2*error_weight[1] + R1*error_weight[2] + R0*error_weight[3] 
            + L0*error_weight[4] + L1*error_weight[5] + L2*error_weight[6] + L3*error_weight[7])/
            (R3*1 + R2*1 + R1*1 + R0*1 + L0*1 + L1*1 + L2*1 + L3*1) - 4.5;
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
  std::cout <<l3-> getValue() << ' '<<l2-> getValue() <<' '<<l1-> getValue()<<' '<<l0-> getValue()<<' '<<r0-> getValue()<<' '<<r1-> getValue()<<' '<<r2-> getValue()<<' '<<r3-> getValue()<<' '<< std::endl;
  std::cout <<L3<<L2<<L1<<L0<<R0<<R1<<R2<<R3<< std::endl;
  std::cout << error << std::endl;
  
  bool L_C=(threshold < lc -> getValue());
  bool R_C=(threshold < rc -> getValue());
    
  if ((L0+L1+L2+L3+L_C)<2 && R_C==1){
    turnLeft();
  }
  
  if ((R0+R1+R2+R3+R_C)<2 && L_C==1){
    turnRight();
  }
}

int main(int argc, char **argv) {

  initialize_devices();

  while (robot->step(timeStep) != -1) {
    lineFollow(7,4);
    
    // wallFollow();
    /*
    get_status();
    implement_status(); // a func which calls sevaral acts
    
    
    */
  };
  
  //disable_devices();

  delete robot;
  return 0;
}