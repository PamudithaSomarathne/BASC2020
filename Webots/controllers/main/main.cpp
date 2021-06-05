#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

#define TIME_STEP 16
#define AVG_SPEED 8
#define MAX_SPEED 9.5

// Initialize all variables
int lsval, lscof[4] = {-20, -10, 10, 20}, lson;
double error = 0, pError = 0, cError = 0, pid = 0;		// Don't change
double kp = 1, ki = 0, kd = 0.09;											// Tune
double leAdj=1, reAdj=1;

Robot *robot = new Robot();
Motor *rbm, *rfm, *lbm, *lfm;
PositionSensor *re, *le;
DistanceSensor *ls[4];

void initialize() {

	// Motors
	*lbm = robot->getMotor("left_back_motor");		// Left back motor
	*lfm = robot->getMotor("left_front_motor");		// Left front motor
	*rbm = robot->getMotor("right_back_motor");		// Right back motor
	*rfm = robot->getMotor("right_front_motor");	// Right front motor
  lbm->setVelocity(0.0);
  lfm->setVelocity(0.0);
  rbm->setVelocity(0.0);
  rfm->setVelocity(0.0);
  lbm->setPosition(INFINITY);
  lfm->setPosition(INFINITY);
  rbm->setPosition(INFINITY);
  rfm->setPosition(INFINITY);

  // Encoders
	*le = robot->getPositionSensor("left_back_encoder");		// Left encoder
	*re = robot->getPositionSensor("right_back_encoder");	// Right encoder
  le->enable(TIME_STEP);
  re->enable(TIME_STEP);

  // Front IR panel
  ls[0] = robot->getDistanceSensor("l2");
  ls[1] = robot->getDistanceSensor("l1");
  ls[2] = robot->getDistanceSensor("r1");
  ls[3] = robot->getDistanceSensor("r2");
  ls[0]->enable(TIME_STEP);
  ls[1]->enable(TIME_STEP);
  ls[2]->enable(TIME_STEP);
  ls[3]->enable(TIME_STEP);
}

double readLine() {
	// Calculate the position of the line and output the pid value for adjustment

	error = 0; lson = 0;
	for (int i=0; i<4; i++){
		lsval = ((ls[i]->getValue())>500);
		error += lscof[i]*lsval;
		lson+=lsval;
	}
	error/=lson;
	pid = kp*error + ki*(error + cError) + kd*(error - pError);
	pError = error; cError += error;
	return max(-1.5, min(pid, 1.5));

}

void moveForward(int left, int right) {
	lbm->setVelocity(AVG_SPEED + left);
	lfm->setVelocity(AVG_SPEED + left);
	rbm->setVelocity(AVG_SPEED + right);
	rfm->setVelocity(AVG_SPEED + right);
}

void moveDistance(int left, int right) {
	double leftPos = le->getValue(), rightPos = re->getValue();
	if (left>=0) {
		if (right>=0) {
			while (true) {
				if ((le->getValue() - leftPos) <= left) {
					lbm->setVelocity(AVG_SPEED);
					lfm->setVelocity(AVG_SPEED);				
				}
				else {
					lbm->setVelocity(0.0);
					lfm->setVelocity(0.0);							
				}
				if ((re->getValue() - rightPos) <= right) {
					rbm->setVelocity(AVG_SPEED);
					rfm->setVelocity(AVG_SPEED);						
				}
				else {
					rbm->setVelocity(0.0);
					rfm->setVelocity(0.0);							
				}
				if (((le->getValue() - leftPos) > left) && ((re->getValue() - rightPos) > right)) break;
			}
		}
		else {
			while (true) {
				if ((le->getValue() - leftPos) <= left) {
					lbm->setVelocity(AVG_SPEED);
					lfm->setVelocity(AVG_SPEED);				
				}
				else {
					lbm->setVelocity(0.0);
					lfm->setVelocity(0.0);							
				}
				if ((re->getValue() - rightPos) >= right) {
					rbm->setVelocity(-AVG_SPEED);
					rfm->setVelocity(-AVG_SPEED);						
				}
				else {
					rbm->setVelocity(0.0);
					rfm->setVelocity(0.0);							
				}
				if (((le->getValue() - leftPos) > left) && ((re->getValue() - rightPos) < right)) break;
			}
		}
	}
	else {
		if (right>=0) {
			while (true) {
				if ((le->getValue() - leftPos) >= left) {
					lbm->setVelocity(-AVG_SPEED);
					lfm->setVelocity(-AVG_SPEED);				
				}
				else {
					lbm->setVelocity(0.0);
					lfm->setVelocity(0.0);							
				}
				if ((re->getValue() - rightPos) <= right) {
					rbm->setVelocity(AVG_SPEED);
					rfm->setVelocity(AVG_SPEED);						
				}
				else {
					rbm->setVelocity(0.0);
					rfm->setVelocity(0.0);							
				}
				if (((le->getValue() - leftPos) < left) && ((re->getValue() - rightPos) > right)) break;
			}
		}
		else {
			while (true) {
				if ((le->getValue() - leftPos) >= left) {
					lbm->setVelocity(-AVG_SPEED);
					lfm->setVelocity(-AVG_SPEED);				
				}
				else {
					lbm->setVelocity(0.0);
					lfm->setVelocity(0.0);							
				}
				if ((re->getValue() - rightPos) >= right) {
					rbm->setVelocity(-AVG_SPEED);
					rfm->setVelocity(-AVG_SPEED);						
				}
				else {
					rbm->setVelocity(0.0);
					rfm->setVelocity(0.0);							
				}
				if (((le->getValue() - leftPos) < left) && ((re->getValue() - rightPos) < right)) break;
			}
		}
	}
}

int main() {

	initialize();

	while (robot->step(TIME_STEP) != -1){
		double line = readLine();
		moveForward(line, -line);
	}

	delete robot;
	return 0;
}