#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#define TIME_STEP 16
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *Secn_m;
  Secn_m=robot->getMotor("slider_motor");
  Secn_m->setPosition(INFINITY);
  
  PositionSensor *Position_S[2];
  Position_S[0]=robot->getPositionSensor("main_encoder");
  Position_S[0]->enable(16);
  
  Position_S[1]=robot->getPositionSensor("encoder_2");
  Position_S[1]->enable(16);
  
  float sec_E_val=0;
  
  
  
  /*PositionSensor *PS;
  PS=robot->getPositionSensor("PS_1");
  PS->enable(TIME_STEP);
  */
  //double position = 0.0;
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  
  while (robot->step(TIME_STEP) != -1) {
       
      if ((sec_E_val>=0) && (sec_E_val<0.75)){
        Secn_m->setVelocity(6.2);
        sec_E_val = Position_S[1]->getValue();
      }
      else if (sec_E_val>0){
        Secn_m->setVelocity(-6.2);
        sec_E_val = Position_S[1]->getValue();
      }
      else {
        Secn_m->setVelocity(0.0);
      }

  }
  return 0;
}
