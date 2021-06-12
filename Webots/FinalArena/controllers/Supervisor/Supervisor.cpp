#include <webots/Supervisor.hpp>
#include <cmath>

#define TIME_STEP 32

using namespace webots;

int main() {
  Supervisor *supervisor = new Supervisor();

  Node *robotHandle = supervisor->getFromDef("Team");
  if (robotHandle == NULL) {
    std::cerr << "No robot with given name in the current world" << std::endl;
    exit(1);
  }
  
  double maxSpeed = 0;

  while (supervisor->step(TIME_STEP) != -1) {
    const double *values = robotHandle->getVelocity();
    double speed = 100*sqrt(values[0]*values[0] + values[2]*values[2]);
    if (speed>maxSpeed) maxSpeed = speed;
    std::cout << "Robot's speed: " << speed << "cm/s\tRobot's maximum speed: " << maxSpeed << "cm/s" << std::endl;
    if (speed>15.15) {
      std::cout << "Robot disqualified for exceeding speed limit" << std::endl;
      supervisor->simulationSetMode(supervisor->SIMULATION_MODE_PAUSE);
      break;
    };
  };
      
  delete supervisor;
  return 0;
}