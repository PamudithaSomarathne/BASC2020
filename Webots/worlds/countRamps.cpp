#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

/*#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>*/

using namespace webots;

#define TIMESTEP 64
#define MAX_SPEED 6.28
#define SCALE 0.6

// int num_ramps = 0, inc = 0, dec = 0;

int main() {
  Robot *robot = new Robot();
  
  /*InertialUnit *imu = robot->getInertialUnit("inertial unit");
  LED *led_incline = robot->getLED("led_green");
  LED *led_decline = robot->getLED("led_red");*/
  
  DistanceSensor *s1 = robot->getDistanceSensor("so");

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
    
    /*imu->enable(20);
    s1->enable(20);
    const double *values = imu->getRollPitchYaw();
    
    if (values[0] > 0.4){
      if (led_incline->get() == 0) inc++;
      led_incline->set(1);
    }
    else if (values[0] < -0.4){
      if (led_decline->get() == 0){
        dec++;
        if (dec == 1 and inc == 0) dec--;
      }
      led_decline->set(1);
      if (inc == 1 and dec == 1){
        num_ramps++; inc = 0; dec = 0;
      }
    }
    else {
      led_incline->set(0);
      led_decline->set(0);
    }*/
    
    double distance = s1->getValue();
    if (distance>900.0){
      back_left_motor->setVelocity(0.0);
      back_right_motor->setVelocity(0.0);
      front_left_motor->setVelocity(0.0);
      front_right_motor->setVelocity(0.0);
      imu->disable();
      s1->disable();
      std::cout << "Number of ramps : " << num_ramps << std::endl;
      break;
    }
  };
  
  // Clean up
  
  delete robot;
  return 0;
}
