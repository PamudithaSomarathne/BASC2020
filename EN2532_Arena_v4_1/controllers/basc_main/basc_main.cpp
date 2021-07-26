#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>

using namespace webots;

int curr_state;

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

LED *led_1 = robot->getLED("led1");
LED *led_2= robot->getLED("led2");

void initialize_devices(){
  l_motor->setPosition(INFINITY); r_motor->setPosition(-INFINITY);
  m_servo->setPosition(1.9); s_servo->setPosition(-1.5);
  l_motor->setVelocity(0.0); r_motor->setVelocity(0.0);
  
  l_enc->enable(timeStep); r_enc->enable(timeStep);
  
  l3->enable(timeStep); l2->enable(timeStep); l1->enable(timeStep);
  l0->enable(timeStep); r0->enable(timeStep); r1->enable(timeStep);
  r2->enable(timeStep); r3->enable(timeStep);
  
  lc->enable(timeStep); rc->enable(timeStep);
  
  lft->enable(timeStep); lbt->enable(timeStep);
  rft->enable(timeStep); rbt->enable(timeStep); ct->enable(timeStep);
  
  gyro->enable(timeStep);
}

void disable_devices(){
  l_enc->disable(); r_enc->disable();
  
  l3->disable(); l2->disable(); l1->disable(); l0->disable();
  r0->disable(); r1->disable(); r2->disable(); r3->disable();
  
  lc->disable(); rc->disable();
  
  lft->disable(); lbt->disable();
  rft->disable(); rbt->disable();
  ct->disable();
  
  gyro->disable();
}

void stopRobot(){l_motor->setVelocity(0.0); r_motor->setVelocity(0.0);}

void delay(int d){while (d--){robot->step(timeStep);}}

void testLED(){
  led_1->set(1); led_2->set(6); delay(64);
  led_1->set(2); led_2->set(5); delay(64);
  led_1->set(3); led_2->set(4); delay(64);
  led_1->set(4); led_2->set(3); delay(64);
  led_1->set(5); led_2->set(2); delay(64);
  led_1->set(6); led_2->set(1); delay(64);
  led_1->set(0); led_2->set(0); delay(64);
}

void moveDistance(double dist){
  dist = dist/2;
  double encPos = r_enc->getValue() + l_enc->getValue();
  l_motor->setVelocity(5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() + l_enc->getValue() - encPos < dist){
    robot->step(timeStep);
  }
  stopRobot();
  curr_state = 100;
  return;
}

// Move back or scan line

/////////////////////////////////////////////// LINE FOLLOWING ///////////////////////////////////////////////
void turnLeft(){
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() - encPos < 5){
    robot->step(timeStep);
  }
  stopRobot();
  return;
}

void turnRight(){
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-5.0);
  l_motor->setVelocity(5.0);
  while (l_enc->getValue() - encPos < 5){
    robot->step(timeStep);
  }
  stopRobot();
  return;
}

float error_weight[8] = {10,20,30,40,50,60,70,80}; // positive values
// Tune these
float kp = 1;
float kd = 10;
float ki = 0.001;

float pr_error = 0;
float i_v = 0;
int threshold = 900;
// weight list can be updated with exact distances from robot sensor panel, makes the error real world relatable in centimeters

float readRaykha(){
  int R3 = (threshold > r3 -> getValue()); // If this doesn't work with bool check ints
  int R2 = (threshold > r2 -> getValue());
  int R1 = (threshold > r1 -> getValue());
  int R0 = (threshold > r0 -> getValue());
  int L0 = (threshold > l0 -> getValue());
  int L1 = (threshold > l1 -> getValue());
  int L2 = (threshold > l2 -> getValue());
  int L3 = (threshold > l3 -> getValue());
    
  float error = R3*error_weight[0] + R2*error_weight[1] + R1*error_weight[2] + R0*error_weight[3] + L0*error_weight[4] + L1*error_weight[5] + L2*error_weight[6] + L3*error_weight[7];
  int sum_ = (R3 + R2 + R1 + R0 + L0 + L1 + L2 + L3);
  if (sum_ == 0){return -100;} 
  if (sum_ == 8){return 100;}
  error = error/sum_ - 45;
  return error;
}

void pidFollow(float max_speed, float base_speed){
  
  float error = readRaykha();
  if (error == -100){ return true;}
  
  // how would you handle all black or all white cases - Thiesh
  // Addd all black all white cases
  // handle line lost case: can do backwards search
  // If to handle all white and all black
  
  float d_v = error - pr_error;
  i_v = i_v + error;
  float p_v = error;
  
  float PID = (kp*p_v + kd*d_v + ki*i_v)/2;
  pr_error = error;
  
  float right_v = base_speed + PID;
  float left_v = base_speed - PID;
  
  //right_v = max(0, min(right_v, max_speed));
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
  return false;
}

void lineFollow0(float max_speed, float base_speed){
  float d1 = 0;
  float lx = 0;
  float rx = 0;
  lx = lc -> getValue();
  rx = rc -> getValue();
  // l1, r1 for T junction
  //std::cout << d1 <<" " << lx <<" " << rx <<" " << std::endl;
  //d1 > 180
  while (true){
    lx = lc -> getValue();
    rx = rc -> getValue();
    std::cout << d1 <<" " << lx <<" " << rx <<" " << std::endl;
    if ((lx < 900) && (rx > 900)){
      turnLeft();
    }
    else if ((lx > 900) && (rx < 900)){
      turnRight();
    }
    else{
      //pidFollow(max_speed,base_speed);
      if (pidFollow(max_speed, base_speed)){
        d1 = rft -> getValue();
        if (d1 < 180 ){
          stopRobot();
        }
      }
    }
    robot -> step(timeStep);
    
    //d1 = rft -> getValue();
  }
  //if (d1 < 180){
    //stopRobot();}
  
  // Handle 90 degrees within this function else call
  // While (NOT DETECTED WALL)
  //     If corner sensors are active: L&R, L, R
  //     else pidFollow
        //---------------------------robot->step(timeStep);
        // sensor readings for next iteration
        //pidFollow(max_speed,base_speed);
  // if WALL DETECTED: state=1
}
}

void lineFollow1(float max_speed, float base_speed){
  // Handle 90 degrees within this function else call
  // If corner sensors are active: L&R, L, R
  // else pidFollow
}

void lineFollow2(float max_speed, float base_speed){
  // Handle 90 degrees within this function else call
  // If corner sensors are active: L&R, L, R
  // else pidFollow
}

/////////////////////////////////////////////// WALL FOLLOWING ///////////////////////////////////////////////
int mid_speed = 6;
float left_speed=0;
float right_speed=0 ;
int side;
float left_dist_square, right_dist_square, error, d_error, wall_dist, calc_speed;
float error_scale = 0.005;
float P = 3.1;
  // float I = 0;
float D = 30;
float prev_error = 0;


// Turn this to run in a while loop
// Return to main only when both walls are finished
// Update curr_state to next state before returning
// Use robot->step(timeStep); to iterate the while loop
void wallFollow(){
    l_motor->setVelocity(left_speed);
    r_motor->setVelocity(right_speed);
    float l_d2 = lft->getValue() ;
    float l_d1 = lbt->getValue() ;
    float r_d1 = rbt->getValue() ;
    float r_d2 = rft->getValue() ;
    l_d1 = l_d1 < 400 ? l_d1 : 400;
    l_d2 = l_d2 < 400 ? l_d2 : 400;
    r_d1 = r_d1 < 400 ? r_d1 : 400;
    r_d2 = r_d2 < 400 ? r_d2 : 400;
    // delay(10);

    left_dist_square = sqrt(l_d1*l_d1 + 1.732 * l_d1*l_d2 + l_d2*l_d2)/2;
    right_dist_square = sqrt(r_d1*l_d1 + 1.732 * r_d1*r_d2 + r_d2*r_d2)/2;
    // cout << " left " << left_dist_square << " right " << right_dist_square << endl;

    wall_dist = left_dist_square < right_dist_square ? left_dist_square : right_dist_square;
    //cout << "$$$$$$$$$$$$$$$" << wall_dist << endl;
    side = left_dist_square < right_dist_square ? 0: 1;
    error = (160 - wall_dist) * error_scale;
    d_error = error - prev_error;
    calc_speed = (P * error + D * d_error);
    prev_error = error;
    calc_speed = calc_speed < 5 ? calc_speed: 5;
    calc_speed = calc_speed > -5 ? calc_speed: -5;
    
    if (side == 0){
      left_speed = mid_speed + calc_speed;
      right_speed = mid_speed - calc_speed;
      left_speed = left_speed < 10 ? left_speed: 10;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 10 ? right_speed: 10;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
    if (side == 1){
      left_speed = mid_speed - calc_speed;
      right_speed = mid_speed + calc_speed;
      left_speed = left_speed < 10 ? left_speed: 10;
      left_speed = left_speed > 0 ? left_speed: 0.5;
      right_speed = right_speed < 10 ? right_speed: 10;
      right_speed = right_speed > 0 ? right_speed: 0.5;
    }
}

////////////////////////////////////////////// BOX MANIPULATION //////////////////////////////////////////////
void circleNavigation(){}
void boxManipulation(){
  m_servo->setPosition(0);
  robot->step(5000);
  s_servo->setPosition(0);
  robot->step(5000);
  s_servo->setPosition(-1.5);
  robot->step(5000);
  m_servo->setPosition(1.9);
  robot->step(5000);
  curr_state=5;
}
void exitCircle(){}

/////////////////////////////////////////////// RAMP NAVIGATION //////////////////////////////////////////////
double angle = 0.0, gyroThresh = 0.5, gyroThresh2=0.0;
bool inclined=0, top=0, rampTurned=0, direction=0;
void readGyro(){
   double gyroOut = -gyro->getValues()[0];
   if (gyroOut > gyroThresh2 || gyroOut < -gyroThresh2) {angle += gyroOut; }
   if ((not top) && (not inclined) && (angle>0.1)) inclined=true;
   else if ((not top) && (inclined) && (angle>-3.0 && angle<0.1)) {inclined=false; top=true;}
   else if ((top) && not(inclined) && (angle<-3.0)) inclined=true;
   else if ((top) && (inclined) && (angle>-3.0 && angle<0.1)) {inclined=false; top=false;}
}

bool isRampEdge(){
  if (ct->getValue() < 200) return true;
  double gyroOut = -gyro->getValues()[0];
  if (gyroOut > gyroThresh || gyroOut < -gyroThresh) return true;
   return false;
}

void rampNavigation(bool direction){
  readGyro();
  std::cout << angle << " inclined:" << inclined << " top:" << top << std::endl;
  if (not isRampEdge()){
    if ((lc->getValue() < 900) && (rc->getValue() < 900)) {
      if (not rampTurned) {
        std::cout << "HELLO" << std::endl;
        moveDistance(6.0);
        if (direction) turnLeft();
        else turnRight();
        rampTurned = true;
      }
      else moveDistance(0.5);
    }
    else moveDistance(0.5);
  }
  else moveDistance(0.5);
  //std::cout << "Hello" << std::endl;
}

// Left and Right turn encoder values are different ---CHECK


/////////////////////////////////////////////// PILLAR COUNTING //////////////////////////////////////////////
int count=0, i=0;
bool readings[50];

void pillarCount(bool direction){
  if (direction) {
    if (lft->getValue()<900) readings[i]=true;
    i++;
  }
  else {
    if (rft->getValue()<900) readings[i]=true;
    i++;
  }
}

bool evaluatePillars(){
  for (int i=0; i<49; i++){
    if (((not readings[i]) && readings[i+1]) || (readings[i] && (not readings[i+1]))) count++;
  }
  if (count==4) return true;
  else return false;
}

/////////////////////////////////////////////// RAMP CORRECTION //////////////////////////////////////////////
void rampPathCorrection(){}

//////////////////////////////////////////////// ESCAPE GATES ////////////////////////////////////////////////
void lineFollow3(float max_speed, float base_speed){
  // If corner sensors are active: L&R, L, R
  // else pidFollow
}

void escapeGates(){
  //this should call only when we come across the first cross line
  
  double thresh1 = 350;  //threshold distance(a little more than detected) depends on sensor
  double thresh2 = 1100; //threshold distance a little more than second 
  
  double ctread=0;  //front ct reading initialized less than thresh1
  //ctread = ct->getValue();
  //std::cout << ctread << std::endl;
  
  while (ctread<=thresh1 || thresh2<=ctread){
    ctread = ct->getValue();
    robot -> step(timeStep);
    //just waitwithout goint forward  
  }
  if (thresh1<=ctread && ctread<=thresh2){
    //ctread = ct->getValue();
    //line follow forward
    moveDistance(147.0);
    led_1->set(1);
    
    //robot -> step(timeStep);
  }
  
  moveDistance(0.0);
  //escaping to this level means the second gate just opened
  //line follow forward full speed ahead.
  curr_state=10;
}

int main(int argc, char **argv) {
  
  curr_state=21;
  const int end_state=5;
  
  initialize_devices();

  while (robot->step(timeStep) != -1) {
    std::cout << "current state:" << curr_state << ' ' << " | end state:" << end_state << std::endl;
    if (curr_state==end_state) {stopRobot(); break;}
    switch (curr_state){
      case 0: lineFollow0(7, 5); break;   // First line follow upto wall - Vidura & tune turnLeft, turnRight enc values
      case 1: wallFollow(); break;        // Wall - Yasod
      case 2: lineFollow1(7, 5); break;   // Wall to circle line - Vidura
      case 3: circleNavigation(); break;  // Circle - Pamuditha
      case 4: boxManipulation(); break;   // Box - Pamuditha
      case 5: exitCircle(); break;        // Circle - Pamuditha
      case 6: lineFollow2(7, 5); break;   // Dash line - Vidura
      case 7: rampNavigation(0); break;    // Ramp - Yomali
      case 8: pillarCount(0); break;       // Pillar - Yomali
      case 9: escapeGates(); break;       // Gates - Tharindu
      case 10: stopRobot(); break;        // End
      case 21: moveDistance(5); break;
      case 22: testLED(); break;
      default: stopRobot(); break;
    }
  };
  
  disable_devices();

  delete robot;
  return 0;
}
