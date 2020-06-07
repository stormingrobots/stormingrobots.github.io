#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gps.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
using namespace webots;

#include <iostream>
#include <string>
using namespace std;

Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left wheel"), *rightMotor = robot->getMotor("right wheel");
Gyro *gyro = robot->getGyro("gyro");
PositionSensor *leftEnc = leftMotor->getPositionSensor(), *rightEnc = rightMotor->getPositionSensor();
GPS *gps = robot->getGPS("gps");
Camera *cam = robot->getCamera("camera");
DistanceSensor *distSensor[8];

double angle = 0, vicAngle, vicDist, initAngle = 0, setAngle = 0, targetEnc = 0;
double victimColor[] = {0,0,0};
int state = 0;
const double radPerMeter = 10.5; //(100/PI*19)*2*PI

double getAngle() {

  return angle * 180 / 3.1415;
}

void updateGyro(int timeStep) {
angle += (timeStep / 1000.0) * (gyro->getValues())[1]; //x, y, z
}

bool arrEquals(double *arr1, double *arr2, int numElem) {
  for (int x = 0; x < numElem; x++)
    if (arr1[x] != arr2[x])
      return false;
  return true;
}

void goForward(double dist) {
 
  targetEnc = leftEnc->getValue() + radPerMeter * dist;
 
  leftMotor->setVelocity(5);
  rightMotor->setVelocity(5);
}

void doTurn(double ang) {
 
  initAngle = getAngle();
 
  setAngle = ang;
 
  leftMotor->setVelocity(1);
  rightMotor->setVelocity(-1);
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

  if(abs(getAngle() - initAngle) >= setAngle) {
 
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    return false;
  }
 
  return true;
}

int main() {

  int timeStep = (int) robot->getBasicTimeStep();
 
  gyro->enable(timeStep);
  leftEnc->enable(timeStep);
  rightEnc->enable(timeStep);
  gps->enable(timeStep);
  cam->enable(timeStep);
  cam->recognitionEnable(timeStep);

  const int CAM_WIDTH = cam->getWidth();
 
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
 
  for (int x = 0; x < 8; x++) {
    distSensor[x] = robot->getDistanceSensor("so" + to_string(x));
    distSensor[x]->enable(timeStep);
  }
 
  while(robot->step(timeStep) != -1) {

  updateGyro(timeStep);
 
    if(state == 0) {
   
      int numObj = cam->getRecognitionNumberOfObjects();
      const CameraRecognitionObject *objects = cam->getRecognitionObjects();
      for(int i = 0; i < numObj; i++) {
     
        if(arrEquals(objects[i].colors, victimColor, 3)) {
       
          cout << "See victim" << endl;
 
          const double* vicPos = objects[i].position;
         
          vicAngle = atan(abs(vicPos[0]) / abs(vicPos[2])) / 3.1415 * 180;
          vicDist = sqrt(vicPos[2] * vicPos[2] + vicPos[0] * vicPos[0]) - 0.3;
         
          cout << vicAngle << endl;
         
          state = 1;
          doTurn(vicAngle);
        }
      }
    }
    else if(state == 1 && !ifTurn()) {
   
     cout << "moving forward" << endl;
      goForward(vicDist);
      state = 2;
    }
    else if(state == 2 && !ifForward()) {
   
      cout << "done" << endl;
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0);
    }
  }
 
  return 0;
}