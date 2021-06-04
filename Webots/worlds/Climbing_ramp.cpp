#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>

using namespace webots;

#define TIMESTEP 64
#define MAX_SPEED 6.28
#define SCALE 0.6

int main() {
  Robot *robot = new Robot();
  
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  
  Motor *back_left_motor = robot->getMotor("back left wheel");
  Motor *back_right_motor = robot->getMotor("back right wheel");
  Motor *front_left_motor = robot->getMotor("front left wheel");
  Motor *front_right_motor = robot->getMotor("front right wheel");
  
  back_left_motor->setPosition(INFINITY);
  back_right_motor->setPosition(INFINITY);
  front_left_motor->setPosition(INFINITY);
  front_right_motor->setPosition(INFINITY);

  back_left_motor->setVelocity(0.0);
  back_right_motor->setVelocity(0.0);
  front_left_motor->setVelocity(0.0);
  front_right_motor->setVelocity(0.0);
  
  while (robot->step(TIMESTEP) != -1) {
  
    back_left_motor->setVelocity(SCALE * MAX_SPEED);
    back_right_motor->setVelocity(SCALE * MAX_SPEED);
    front_left_motor->setVelocity(SCALE * MAX_SPEED);
    front_right_motor->setVelocity(SCALE * MAX_SPEED);
    
    imu->enable(20);
    
    const double *values = imu->getRollPitchYaw();
    std::cout<<"Angle x :"<<imu->getRollPitchYaw()[0]<<std::endl;
    std::cout<<"Angle y :"<<imu->getRollPitchYaw()[1]<<std::endl;
    std::cout<<"Angle z :"<<imu->getRollPitchYaw()[2]<<std::endl;
    
};
  
  // Clean up
  
  delete robot;
  return 0;
}
