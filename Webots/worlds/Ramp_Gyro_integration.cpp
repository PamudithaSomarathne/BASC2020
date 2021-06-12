
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>

using namespace webots;

#define TIMESTEP 64
#define MAX_SPEED 6.28
#define SCALE 0.6

int main() {
  Robot *robot = new Robot();
  
  Gyro *gy = robot->getGyro("gyro"); //gyro object change
  
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
    
    gy->enable(20);
    
    const double *values = gy->getValues();
    std::cout<<"Angular freq x :"<<values[0]<<std::endl;
    //std::cout<<"Angular freq y :"<<values[1]<<' ';
    //std::cout<<"Angular freq z :"<<values[2]<<std::endl;
    
};
  
  // Clean up
  
  delete robot;
  return 0;
}
