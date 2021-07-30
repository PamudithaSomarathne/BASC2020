#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>

using namespace webots;

int curr_state;

Robot *robot = new Robot();
const int timeStep = 32;//(int)robot->getBasicTimeStep(); // ADJUST

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

Camera *cam_f = robot->getCamera("front_camera");
Camera *cam_b = robot->getCamera("bottom_camera");

Gyro *gyro = robot->getGyro("gyro");

LED *led_1 = robot->getLED("led1");
LED *led_2= robot->getLED("led2");

void initialize_devices(){
  l_motor->setPosition(INFINITY); r_motor->setPosition(-INFINITY);
  m_servo->setPosition(1.55); s_servo->setPosition(0);
  l_motor->setVelocity(0.0); r_motor->setVelocity(0.0);
  
  l_enc->enable(timeStep); r_enc->enable(timeStep);
  
  l3->enable(timeStep); l2->enable(timeStep); l1->enable(timeStep);
  l0->enable(timeStep); r0->enable(timeStep); r1->enable(timeStep);
  r2->enable(timeStep); r3->enable(timeStep);
  
  lc->enable(timeStep); rc->enable(timeStep);
  
  lft->enable(timeStep); lbt->enable(timeStep);
  rft->enable(timeStep); rbt->enable(timeStep); ct->enable(timeStep);
  
  cam_f->enable(timeStep); cam_b->enable(timeStep);
  
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
  
  cam_f->disable(); cam_b->disable();
  
  gyro->disable();
}

void stopRobot(){l_motor->setVelocity(0.0); r_motor->setVelocity(0.0);}

void delay(int d){while (d--){robot->step(timeStep);}}

void testLED(){
  led_1->set(1); led_2->set(6); delay(8);
  led_1->set(2); led_2->set(5); delay(8);
  led_1->set(3); led_2->set(4); delay(8);
  led_1->set(4); led_2->set(3); delay(8);
  led_1->set(5); led_2->set(2); delay(8);
  led_1->set(6); led_2->set(1); delay(8);
}

const double base_speed = 7.0;

void moveDistance(double dist){
  dist = dist/2;
  double encPos = r_enc->getValue() + l_enc->getValue();
  l_motor->setVelocity(base_speed);
  r_motor->setVelocity(base_speed);
  while (r_enc->getValue() + l_enc->getValue() - encPos < dist){
    robot->step(timeStep);
  }
  stopRobot();
  curr_state = 100;
}

// Move back or scan line

/////////////////////////////////////////////// LINE FOLLOWING ///////////////////////////////////////////////
void turnLeft(){
  moveDistance(15.0);
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-base_speed);
  r_motor->setVelocity(base_speed);
  while (r_enc->getValue() - encPos < 3.5){
    robot->step(timeStep);
  }
  stopRobot();
  delay(25);
  return;
}

void turnRight(){
  moveDistance(15.0);
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-base_speed);
  l_motor->setVelocity(base_speed);
  while (l_enc->getValue() - encPos < 3.5){
    robot->step(timeStep);
  }
  stopRobot();
  delay(25);
  return;
}

void turn180Left(){
  moveDistance(3);
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-base_speed);
  l_motor->setVelocity(base_speed);
  while (l_enc->getValue() - encPos < 6.6){
    robot->step(timeStep);
  }
  stopRobot();
  delay(25);
  return;
}

void turn180Right(){
  moveDistance(3);
  double encPos = r_enc->getValue();
  r_motor->setVelocity(base_speed);
  l_motor->setVelocity(-base_speed);
  while (r_enc->getValue() - encPos < 6.8){
    robot->step(timeStep);
  }
  stopRobot();
  delay(25);
  return;
}

float error_weight[8] = {10,20,30,40,50,60,70,80}; // positive values
// Tune these
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

bool pidFollow(float max_speed, float base_speed, float kp=1.0, float kd=10.0, float ki=0.0){
  
  float error = readRaykha();
  if (error == -100){return true;}
  if (error == 100) {return true;}
  
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

double d1=0, lx, rx;
void lineFollow0(float max_speed, float base_speed){
  lx = lc -> getValue();
  rx = rc -> getValue();
  while (true){
    lx = lc -> getValue();
    rx = rc -> getValue();
    if ((lx < 900) && (rx > 900)){turnLeft();}
    else if ((lx > 900) && (rx < 900)){turnRight();}
    else{
      if (pidFollow(max_speed, base_speed)){
        d1 = rft -> getValue();
        if (d1 < 180 ){stopRobot(); curr_state=1; break;}
      }
    }
    robot -> step(timeStep);
  }
}


void lineFollow1(float max_speed, float base_speed){
  lx = lc -> getValue();
  rx = rc -> getValue();
  while (true){
    lx = lc -> getValue();
    rx = rc -> getValue();
    if ((lx < 900) && (rx > 900)) {turnLeft();}
    else if ((lx > 900) && (rx < 900)) {turnRight();}
    else{
        if(pidFollow(max_speed, base_speed)){ // if the circle is detected switch state to 3
          stopRobot(); curr_state=3; break;
        }
    }
    robot -> step(timeStep);
   }
}

void lineFollow2(float max_speed, float base_speed){
  while(true){ // until the ramp is detected 
    float ramp_dist =  ct -> getValue();
    if (ramp_dist <120){  // if the ramp is detected switch state to 7
      stopRobot(); curr_state=7;
      break;
    }
    else{
      if(pidFollow(max_speed, base_speed)){ // follow the dash lines
        moveDistance(4);}
    }
    robot -> step(timeStep);
  }
}

/////////////////////////////////////////////// WALL FOLLOWING ///////////////////////////////////////////////
double mid_speed = 15, left_speed=0, right_speed=0 ;
bool side;
double left_dist_square, right_dist_square, error, d_error, wall_dist, calc_speed, prev_error = 0;
double l_d2, l_d1, r_d1, r_d2;
double P = 0.3, D = 1.0;

void wallFollow(){
  while (pidFollow(0, 0)){
    l_d1 = lft->getValue() ; l_d2 = lbt->getValue() ;
    r_d2 = rbt->getValue() ; r_d1 = rft->getValue() ;
    
    if (l_d1>1000 && r_d1>1000) {stopRobot(); break;}
    
    side = l_d1 < r_d1;
    
    l_d2 = std::min(l_d2, 170.0); r_d2 = std::min(r_d2, 170.0);

    left_dist_square = sqrt(l_d1*l_d1 + 1.732 * l_d1*l_d2 + l_d2*l_d2)/2;
    left_dist_square = sqrt(l_d1*l_d1 + 1.732 * l_d1*l_d2 + l_d2*l_d2)/2;
    right_dist_square = sqrt(r_d1*l_d1 + 1.732 * r_d1*r_d2 + r_d2*r_d2)/2;

    if (side) wall_dist = left_dist_square;
    else wall_dist = right_dist_square;
    
    error = 140 - wall_dist;    
    d_error = error - prev_error;
    calc_speed = (P * error + D * d_error);
    prev_error = error;
    calc_speed = std::max(-base_speed, std::min(calc_speed, base_speed));
    
    if (side == 0){
      left_speed = std::max(0.5, std::min(mid_speed - calc_speed, 15.0));
      right_speed = std::max(0.5, std::min(mid_speed + calc_speed, 15.0));
    }
    if (side == 1){
      left_speed = std::max(0.5, std::min(mid_speed + calc_speed, 15.0));
      right_speed = std::max(0.5, std::min(mid_speed - calc_speed, 15.0));
    }
    l_motor->setVelocity(left_speed);
    r_motor->setVelocity(right_speed);
    robot->step(timeStep);
  }
  while (pidFollow(0, 0)) {moveDistance(0.5);}
  stopRobot();
  curr_state = 2;
}

////////////////////////////////////////////// BOX MANIPULATION //////////////////////////////////////////////
void turnCircle(){
  moveDistance(16.0);
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() - encPos < 3.9){robot->step(timeStep);}
  stopRobot();
  delay(25);
  return;
}

void turnR_crit(){
  delay(25);
  double encPos = l_enc->getValue();
  r_motor->setVelocity(-5.0);
  l_motor->setVelocity(5.0);
  while (l_enc->getValue() - encPos < 3.0){robot->step(timeStep);}
  stopRobot();
  delay(25);
  return;
}
void turnL_crit(){
  delay(25);
  double encPos = r_enc->getValue();
  l_motor->setVelocity(-5.0);
  r_motor->setVelocity(5.0);
  while (r_enc->getValue() - encPos < 3.0){robot->step(timeStep);}
  stopRobot();
  delay(25);
  return;
}

bool direction=0;
void boxManipulation(){
  stopRobot(); delay(25);
  m_servo->setPosition(0.7); s_servo->setPosition(-0.8); robot->step(1000);
  m_servo->setPosition(0.25); robot->step(1000);
  s_servo->setPosition(0.05); robot->step(2000);
  m_servo->setPosition(1.55); robot->step(2500);
   
  const unsigned char *img_f = cam_f->getImage();
  int r_f = cam_f->imageGetRed(img_f, 3, 1, 1);
  int g_f = cam_f->imageGetGreen(img_f, 3, 1, 1);
  int b_f = cam_f->imageGetBlue(img_f, 3, 1, 1);
  int max_f = std::max(r_f, std::max(g_f, b_f)), col_f;
  if (max_f==r_f) col_f=1;
  else if (max_f==g_f) col_f=2;
  else if (max_f==b_f) col_f=3;

  const unsigned char *img_b = cam_b->getImage();
  int r_b = cam_b->imageGetRed(img_b, 3, 1, 1);
  int g_b = cam_b->imageGetGreen(img_b, 3, 1, 1);
  int b_b = cam_b->imageGetBlue(img_b, 3, 1, 1);
  int max_b = std::max(r_b, std::max(g_b, b_b)), col_b;
  if (max_b==r_b) col_b=1;
  else if (max_b==g_b) col_b=2;
  else if (max_b==b_b) col_b=3;
  
  direction = std::abs(col_f - col_b)%2;
  std::cout << direction << std::endl;
  curr_state=23;
}

void dropBox(){
  m_servo->setPosition(0.3); robot->step(2000);
  s_servo->setPosition(-0.8); robot->step(2000);
  m_servo->setPosition(0.7); robot->step(1000);
  m_servo->setPosition(1.55); s_servo->setPosition(0);
  robot->step(1000);
  curr_state=22;
}

bool check;
void circleNavigation(){
  float lx = lc -> getValue();
  float rx = rc -> getValue();
  if (pidFollow(7,5, 100.0, 0.0)){
    moveDistance(2);
    turnRight();
    curr_state =3;
  }
  if (lx < 900 && rx > 900){
    moveDistance(3.0);
    turnCircle();
    float box1 = ct -> getValue();
    std::cout << box1 <<  std::endl;
    if (box1 < 400){
      check = true;
      turnR_crit();
      curr_state =3;
    }
    else{
      std::cout << 1111 <<  std::endl;
      while(true){
        robot -> step(timeStep);
        if(pidFollow(7,5, 100.0, 0.0)){break;}
        }
      if (check){
        moveDistance(3);
        turnLeft();
        while(true){
          robot -> step(timeStep);
          std::cout << box1 <<  std::endl;
          box1 = ct -> getValue();
          if(box1 < 30){ // BOTTOM LEFT - FIRST CHECK
            led_1->set(1);
            stopRobot();
            boxManipulation();
            moveDistance(15.0);
            dropBox();
            turn180Left();
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            moveDistance(5.0);
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            moveDistance(4.0); turnLeft();
            while (rc->getValue()>900) {pidFollow(7, 5, 100.0, 0.0); robot->step(timeStep);}
            moveDistance(4.0); turnRight();
            curr_state=6; break;
          }
          pidFollow(7,5, 100.0, 0.0);
          
        }
      }
      else{
        moveDistance(14);
        delay(20);
        if (ct -> getValue() < 500){
          while(true){
          robot -> step(timeStep);
          std::cout << box1 <<  std::endl;
          box1 = ct -> getValue();
          if(box1 < 30){ // TOP - RIGHT
            led_1->set(2);
            stopRobot();
            boxManipulation();
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            dropBox(); moveDistance(4.0); turnLeft();
            while (rc->getValue()>900) {pidFollow(7, 5, 100.0, 0.0); robot->step(timeStep);}
            moveDistance(4.0); turnRight();
            curr_state=6; break;
          }
          pidFollow(7, 5, 100.0, 0.0);
          
        }
        }
        else if (rbt -> getValue() < 500){
          moveDistance(5.5);
          turnR_crit();
          while(true){
          robot -> step(timeStep);
          std::cout << box1 <<  std::endl;
          box1 = ct -> getValue();
          if(box1 < 30){
            led_1->set(3);
            stopRobot();
            boxManipulation();
            moveDistance(15.0);
            dropBox();
            turn180Left();
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            moveDistance(5.0);
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            moveDistance(4.0); turnRight();
            while (rc->getValue()>900) {pidFollow(7, 5, 100.0, 0.0); robot->step(timeStep);}
            moveDistance(4.0); turnLeft();
            curr_state=6; break;
          }
          pidFollow(7,5, 100.0, 0.0);
          
        }
        }
        else{
          moveDistance(5);
          turnL_crit();
          while(true){
          robot -> step(timeStep);
          std::cout << box1 <<  std::endl;
          box1 = ct -> getValue();
          if(box1 < 30){
            led_1->set(4);
            stopRobot();
            boxManipulation();
            while (!pidFollow(15, 10, 1.0, 2.5)) {robot->step(timeStep);}
            dropBox(); moveDistance(4.0); turnRight();
            while (rc->getValue()>900) {pidFollow(7, 5, 100.0, 0.0); robot->step(timeStep);}
            moveDistance(4.0); turnLeft();
            curr_state=6; break;
          }
          pidFollow(7,5, 0.5, 100.0, 0.0);    
        }
        }
        
      }
    } 
  }
  pidFollow(7,5);
}

/////////////////////////////////////////////// RAMP NAVIGATION //////////////////////////////////////////////
void reverse(){
  led_1->set(1); 
  double encPos = r_enc->getValue() + l_enc->getValue();
  l_motor->setVelocity(-base_speed);
  r_motor->setVelocity(-base_speed);
  while (encPos - r_enc->getValue() - l_enc->getValue() < 7){robot->step(timeStep);}
  stopRobot();
  led_1->set(0);
  curr_state = 100;
}

double gyroT = 0.5;
bool notRampEdge(){return std::abs(gyro->getValues()[0]) < gyroT;}

void rampNavigation(){
  while (notRampEdge()) {pidFollow(7, 5); robot->step(timeStep);}
  moveDistance(10);
  while (!pidFollow(7, 5)) {robot->step(timeStep);}
  while (pidFollow(7, 5)) {moveDistance(0.5);}
  while (!pidFollow(7, 5)){robot->step(timeStep);}
  if (lc->getValue()<900 && rc->getValue()<900){
    if (direction) turnRight();
    else turnLeft();
  }
  reverse(); delay(10);
  while (!pidFollow(7, 5)) {robot->step(timeStep);}
  while (pidFollow(7, 5)) {moveDistance(0.5);}
  moveDistance(5);
  while (notRampEdge()) {pidFollow(7, 5); robot->step(timeStep);}
  moveDistance(10);
  stopRobot();
  curr_state=8;
}

/////////////////////////////////////////////// RAMP CORRECTION //////////////////////////////////////////////
void lineFollow3(float max_speed, float base_speed){
  led_2->set(1);
  while (lc->getValue()>900 || rc->getValue()>900) {
    if (pidFollow(max_speed, base_speed)) {moveDistance(1);}
    robot->step(timeStep);
  }
  stopRobot(); led_2->set(2); curr_state=9;
}

void rampPathCorrection(){
  led_2->set(6);
  while (notRampEdge()) {pidFollow(15, 10); robot->step(timeStep);}
  moveDistance(10.0);
  while (lc->getValue()>900 && rc->getValue()>900) {
    if (pidFollow(7, 5, 0.07, 0.2)) {moveDistance(3);}
    else robot->step(timeStep);
  }
  moveDistance(10.0);
  led_2->set(5);
  while (notRampEdge()) {pidFollow(7, 5, 0.07, 0.2); robot->step(timeStep);}
  led_2->set(4);
  moveDistance(15.0);
  led_2->set(3);
  while (notRampEdge()) {pidFollow(7, 5, 0.07, 0.2); robot->step(timeStep);}
  led_2->set(2);
  moveDistance(15.0);
  led_2->set(1);
  
  while (lc->getValue()>900 && rc->getValue()>900){
    if (pidFollow(15, 10)) {moveDistance(1);}
    else robot->step(timeStep);
  }
  moveDistance(1);
  if (direction) turnLeft();
  else turnRight();
  lineFollow3(20, 15);
}

/////////////////////////////////////////////// PILLAR COUNTING //////////////////////////////////////////////
const int pillarT=300;
int count=0, stepCount=250;
bool readings[250];

bool evaluatePillars(){
  for (int i=0; i<stepCount-1; i++){
    if (((not readings[i]) && readings[i+1]) || (readings[i] && (not readings[i+1]))) count++;
  }
  if (not(count%4)) return true;
  else return false;
}

void pillarCount(){
  for (int i=0; i<stepCount; i++){
    if (direction) {
      if (rc->getValue()<900) {stepCount=i; break;}
      if (rbt->getValue()<pillarT) {readings[i]=true; led_2->set(6);}
      else led_2->set(0);
    }
    else {
      if (lc->getValue()<900) {stepCount=i; break;}
      if (lbt->getValue()<pillarT) {readings[i]=true; led_2->set(6);}
      else led_2->set(0);
    }
    pidFollow(7,6); robot->step(timeStep);
  }
  if (evaluatePillars()) {
    led_1->set(2);
    moveDistance(1);
    if (direction) turnRight();
    else turnLeft();
    lineFollow3(7, 5);
    stopRobot(); curr_state=9;
  }
  else {
  led_1->set(1);
  if (direction) turn180Right();
  else turn180Left();
  rampPathCorrection();}
}

//////////////////////////////////////////////// ESCAPE GATES ////////////////////////////////////////////////
double thresh1 = 350;  //threshold distance(a little more than detected) depends on sensor
double thresh2 = 1100; //threshold distance a little more than second  
double ctread=0;  //front ct reading initialized less than thresh1
bool white;

void escapeGates(){
  while (ctread<=thresh1 || thresh2<=ctread){ //just waitwithout goint forward  
    ctread = ct->getValue(); robot -> step(timeStep);    
  }
  if (thresh1<=ctread && ctread<=thresh2){
    led_1->set(1); moveDistance(base_speed); // skip the 1st T
    while (true){
      white = pidFollow(20,15); // White cross-line
      if (white){ // Handle cross-line within this function else move forward
        stopRobot(); ctread = ct->getValue();
        // just wait without going forward
        while (ctread <= thresh1) {ctread = ct->getValue(); robot->step(timeStep);}
        // second gate is open
        moveDistance(5.0); white = pidFollow(20, 15);
        if (white) {moveDistance(17.0); stopRobot(); break;} // End square
      }
      robot->step(timeStep);
    }
  }
  curr_state=22;
}

int main(int argc, char **argv) {
  
  curr_state=-1;
  const int end_state=-2;
  
  initialize_devices();
  
  while (robot->step(timeStep) != -1) {
    std::cout << "current state:" << curr_state << ' ' << " | end state:" << end_state << std::endl;
    if (curr_state==end_state) {stopRobot(); break;}
    switch (curr_state){
      case -1: moveDistance(5); curr_state=0; break;
      case 0: lineFollow0(15, 10); break;   // First line follow upto wall - Vidura & tune turnLeft, turnRight enc values
      case 1: wallFollow(); break;        // Wall - Yasod
      case 2: lineFollow1(15, 10); break;   // Wall to circle line - Vidura
      case 3: circleNavigation(); break;  // Circle - Pamuditha
      case 4: boxManipulation(); break;   // Box - Pamuditha
      case 6: lineFollow2(15, 10); break;   // Dash line - Vidura
      case 7: rampNavigation(); break;    // Ramp - Yomali
      case 8: pillarCount(); break;       // Pillar - Yomali
      case 9: escapeGates(); break;       // Gates - Tharindu
      case 10: stopRobot(); break;        // End
      case 21: moveDistance(5); break;
      case 22: testLED(); break;
      case 23: dropBox(); break;
      case 24: rampPathCorrection(); break;
      default: stopRobot(); curr_state=end_state; break;
    }
  };
  
  disable_devices();

  delete robot;
  return 0;
}
