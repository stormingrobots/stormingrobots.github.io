from controller import Robot
from controller import Gyro

#updates angle variable according to angular velocity from gyro
#angleCurrent = anglePast + integral of angular velocity over one timeStep since last updated angle
#should be called every time main loop repeats
def updateGyro():
    global angle
    angle += (timeStep / 1000.0) * (gyro.getValues())[1]

#returns current angle of robot relative to starting angle
#angle does not drop to 0 after exceeding 360
#angle % 360 will yield relative angle with maximum 360
def getAngle():
    return angle * 180 / 3.1415

#sets motors to go forward a certain distance
def goForward(leftMotor, rightMotor, dist):
    global state, targetEnc
    if encoders.getValue() == encoders.getValue():  #checks if getValue() is not naN
        targetEnc = encoders.getValue() + radPerMeter * dist
    else:
        targetEnc = radPerMeter * dist
    state = forward
    leftMotor.setVelocity(maxVelocity * 0.5)
    rightMotor.setVelocity(maxVelocity * 0.5)

#returns false if robot finished going forward, true otherwise
def ifForward(leftMotor, rightMotor):
    if encoders.getValue() - targetEnc > -0.01:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        return False
    return True

#sets motors to turn
#notes starting angle so ifTurn90() can find degrees turned
def doTurn(leftMotor, rightMotor):
    global state, initAngle
    initAngle = round(getAngle() / 90.0) * 90.0
    state = turn
    leftMotor.setVelocity(maxVelocity * 0.1)
    rightMotor.setVelocity(-maxVelocity * 0.1)

#returns false when robot finished 90 degree turn, true otherwise
def ifTurn90():
    if abs(getAngle() - initAngle) >= 86.0:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        return False
    return True


timeStep = 32
numEdgesTravel = 0
angle = 0
initAngle = 0
targetEnc = 0
radPerMeter = 10.5
state = 0

#state values for specific motor movements
#e.g. state = 1 if robot is currently moving forward
forward = 1
turn = 2

#obtaining all objects needed (robot, motors, sensors)
myRobot = Robot()
leftMotor = myRobot.getMotor('left wheel')
rightMotor = myRobot.getMotor('right wheel')
encoders = leftMotor.getPositionSensor()
gyro = myRobot.getGyro('gyro')

#initialize motors and sensors
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
gyro.enable(timeStep)
encoders.enable(timeStep)
maxVelocity = leftMotor.getMaxVelocity()

#main code: robot will move in a box shape
state = forward
goForward(leftMotor, rightMotor, 2.0)
while myRobot.step(timeStep) != -1:
    updateGyro()
    if state == forward and not ifForward(leftMotor, rightMotor):
        doTurn(leftMotor, rightMotor)
    elif state == turn and not ifTurn90():
        if numEdgesTravel < 3:
            numEdgesTravel += 1
            goForward(leftMotor, rightMotor, 2.0)
        