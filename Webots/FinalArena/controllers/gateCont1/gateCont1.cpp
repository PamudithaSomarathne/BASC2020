#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;

int time=0;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *motor = robot->getMotor("rotational motor");
  
  motor->setPosition(0.0);
  motor->setVelocity(5);
  
  int timeStep = (int)robot->getBasicTimeStep();

  while (robot->step(timeStep) != -1) {
    time = (int(robot->getTime()))%20;
    if (time<10) {motor->setPosition(1.2);}
    else if (time<20) {motor->setPosition(0.0);}
  };

  delete robot;
  return 0;
}
