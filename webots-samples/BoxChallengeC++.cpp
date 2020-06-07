#include <iostream>
#include <Math.h>
using namespace std;

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
using namespace webots;

Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left wheel"), *rightMotor = robot->getMotor("right wheel");
Gyro *gyro = robot->getGyro("gyro");
PositionSensor *leftEnc = leftMotor->getPositionSensor(), *rightEnc = rightMotor->getPositionSensor();

const int MAX_VELOCITY = leftMotor->getMaxVelocity();
const double radPerMeter = 10.5; //(100/PI*19)*2*PI

double angle = 0, initAngle;
enum State {movingForward, turn} state;
int numEdgesTravel = 0, targetEnc = 0;

void updateGyro(int timeStep) {
  angle += (timeStep /1000.0) * (gyro->getValues())[1];
}

double getAngle() {

  return angle * 180 / 3.1415;
}

void goForward(double dist) {

  state = movingForward;
  
  targetEnc = leftEnc->getValue() + radPerMeter * dist;
  
  leftMotor->setVelocity(MAX_VELOCITY * .5);
  rightMotor->setVelocity(MAX_VELOCITY * .5);
}

void doTurn() {

  state = turn;
  
  initAngle = getAngle();
  
  leftMotor->setVelocity(MAX_VELOCITY * .1);
  rightMotor->setVelocity(MAX_VELOCITY * -.1);
}

bool ifForward() {

  if(abs(leftEnc->getValue() - targetEnc) < .1) {
  
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    return false;
  }
  
  return true;
}

bool ifTurn() {

  if(abs(getAngle() - initAngle) >= 86.0) {
  
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    return false;
  }
  
  return true;
}


int main() {

  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  
  int timeStep = (int) robot->getBasicTimeStep();
  
  gyro->enable(timeStep);
  leftEnc->enable(timeStep);
  rightEnc->enable(timeStep);
  
  state = movingForward;
  goForward(2.0);
  
  while(robot->step(timeStep) != -1) {
  
    updateGyro(timeStep);
    
    if(state == movingForward && !ifForward()) {
    
      doTurn();
    }
    else if(state == turn && !ifTurn()) {
    
      if(numEdgesTravel < 3) {
      
        numEdgesTravel++;
        goForward(2.0);
      }
    }
  }
  
  return 0;
}