#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>

using namespace std;
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

int mid_speed = 6;
float left_speed=0;
float right_speed=0 ;
int side;
float left_dist_square, right_dist_square, error, d_error, wall_dist, calc_speed;
float error_scale = 0.0035;
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
    cout << l_d2<<" "<< l_d1<<" "<< r_d1<<" "<< r_d2 << endl;
    l_d1 = l_d1 < 1000 ? l_d1 : 1000;
    l_d2 = l_d2 < 1000 ? l_d2 : 1000;
    r_d1 = r_d1 < 1000 ? r_d1 : 1000;
    r_d2 = r_d2 < 1000 ? r_d2 : 1000;
    // delay(10);

    left_dist_square = sqrt(l_d1*l_d1 + 1.732 * l_d1*l_d2 + l_d2*l_d2)/2;
    right_dist_square = sqrt(r_d1*l_d1 + 1.732 * r_d1*r_d2 + r_d2*r_d2)/2;
    cout << " left " << left_dist_square << " right " << right_dist_square << endl;
    

    wall_dist = left_dist_square < right_dist_square ? left_dist_square : right_dist_square;
    cout << "$$$$$$$$$$$$$$$" << wall_dist << endl;
    side = left_dist_square < right_dist_square ? 0: 1;
    error = (180 - wall_dist) * error_scale;
    d_error = error - prev_error;
    calc_speed = (P * error + D * d_error);
    prev_error = error;
    calc_speed = calc_speed < 5 ? calc_speed: 5;
    calc_speed = calc_speed > -5 ? calc_speed: -5;
    
    if (side == 0){
      left_speed = mid_speed + calc_speed;
      right_speed = mid_speed - calc_speed;
      left_speed = left_speed < 8 ? left_speed: 8;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 8 ? right_speed: 8;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
    if (side == 1){
      left_speed = mid_speed - calc_speed;
      right_speed = mid_speed + calc_speed;
      left_speed = left_speed < 8 ? left_speed: 8;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 8 ? right_speed: 8;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
}

int main(int argc, char **argv) {

  initialize_devices();

  while (robot->step(timeStep) != -1) {
    wallFollow();
    
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