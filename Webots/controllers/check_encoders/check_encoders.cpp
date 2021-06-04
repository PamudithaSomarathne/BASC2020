#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;

#define TIME_STEP 32
#define AVG_SPEED 9.5

int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Motor *right_back_motor = robot->getMotor("right_back_motor");
  Motor *right_front_motor = robot->getMotor("right_front_motor");
  Motor *left_back_motor = robot->getMotor("left_back_motor");
  Motor *left_front_motor = robot->getMotor("left_front_motor");
  
  PositionSensor *right_back_encoder = robot->getPositionSensor("right_back_encoder");
  PositionSensor *left_back_encoder = robot->getPositionSensor("left_back_encoder");
  
  right_back_encoder->enable(TIME_STEP);
  left_back_encoder->enable(TIME_STEP);
  
  right_back_motor->setPosition(INFINITY);
  right_front_motor->setPosition(INFINITY);
  left_back_motor->setPosition(INFINITY);
  left_front_motor->setPosition(INFINITY);
  
  right_back_motor->setVelocity(0.0);
  right_front_motor->setVelocity(0.0);
  left_back_motor->setVelocity(0.0);
  left_front_motor->setVelocity(0.0);
  
  right_back_motor->setVelocity(AVG_SPEED);
  right_front_motor->setVelocity(AVG_SPEED);
  left_back_motor->setVelocity(AVG_SPEED);
  left_front_motor->setVelocity(AVG_SPEED);  

  while (robot->step(TIME_STEP) != -1) {
  
    std::cout << right_back_encoder->getValue() << '\t' << left_back_encoder->getValue() << std::endl;
   
  };

  delete robot;
  return 0;
}
