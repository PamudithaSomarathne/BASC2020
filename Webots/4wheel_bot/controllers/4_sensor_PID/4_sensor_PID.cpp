#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define great 10

using namespace webots;

double error = 0, P = 0, I = 0, D = 0, pre_error = 0, pre_I = 0;
double kp = 1, kd = 0.09, ki = 0, PID = 0;


int main(int argc, char **argv) {

  Robot *robot = new Robot();
  DistanceSensor *ds[4];
  char dsNames[4][10] = {"left_1", "left_2", "right_1", "right_2"};
  
  for (int i = 0; i < 4; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  while (robot->step(TIME_STEP) != -1) {
    double right_1v = ds[2]->getValue();
    double left_1v = ds[0]->getValue();
    double right_2v = ds[3]->getValue();
    double left_2v = ds[1]->getValue();
    
    std::cout << "right_1 " << right_1v << "right_2 " << right_2v << "left_2 " << left_2v << "left_1 " << left_1v << std::endl;
    
    if (right_1v > 500 && right_2v > 500 && left_2v > 500 && left_1v > 500){
      error = 0;
    }
    else if(right_1v < 500 && right_2v < 500 && left_2v < 500 && left_1v < 500){
      wheels[0]->setVelocity(MAX_SPEED);
      wheels[1]->setVelocity(MAX_SPEED);
      wheels[2]->setVelocity(MAX_SPEED);
      wheels[3]->setVelocity(MAX_SPEED);
    }
    else if(right_1v > 500 && right_2v < 500 && left_2v > 500 && left_1v > 500){
      error = 2;
    }
    else if(right_1v < 500 && right_2v < 500 && left_2v > 500 && left_1v > 500){
      error = 3;
    }
    else if(right_1v < 500 && right_2v > 500 && left_2v > 500 && left_1v > 500){
      error = 4;
    }
    else if(right_1v > 500 && right_2v > 500 && left_2v < 500 && left_1v > 500){
      error = -2;
    }
    else if(right_1v > 500 && right_2v > 500 && left_2v < 500 && left_1v < 500){
      error = -3;
    }
    else if(right_1v > 500 && right_2v > 500 && left_2v > 500 && left_1v < 500){
      error = -4;
    }
    
    P = error;
    D = error - pre_error;
    I = error + pre_I;
    pre_error = error;
    pre_I = I;
    PID = (kp*P) + (kd*D) + (ki*I);
    
    double left_speed_f =  MAX_SPEED + PID;
    double left_speed_b =  MAX_SPEED + PID;
    double right_speed_f =  MAX_SPEED - PID;
    double right_speed_b =  MAX_SPEED - PID;
    
    if (left_speed_f > great){
      left_speed_f = great;
      left_speed_b =  great;
    }
    else if (left_speed_f < 0){
      left_speed_f = 0;
      left_speed_b = 0;
    }
    if (right_speed_f > great){
      right_speed_f = great;
      right_speed_b = great;
    }
    else if (right_speed_f < 0){
      right_speed_f = 0;
      right_speed_b = 0;
    }
    
    
    wheels[0]->setVelocity(left_speed_f);
    wheels[1]->setVelocity(right_speed_f);
    wheels[2]->setVelocity(left_speed_b);
    wheels[3]->setVelocity(right_speed_b);
    
    
  }


  delete robot;
  return 0;
}
