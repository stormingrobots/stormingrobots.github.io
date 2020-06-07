from controller import Robot
from controller import Camera
import math

#returns list in string form with number rounded to 2 decimals
def roundedList(myList):
    string = "[ "
    for i in myList:
        string += str(round(i, 2)) + ", "
    string += "]"
    return string

recognitionColor = [0, 0, 0]
timeStep = 32
myRobot = Robot()
camera = myRobot.getCamera("camera")

#camera.enable(timeStep)
camera.recognitionEnable(timeStep)

while myRobot.step(timeStep) != -1:
    objects = camera.getRecognitionObjects()
    for obj in objects:
        if obj.get_colors() == recognitionColor:
            print("Recognized object!")
            print("Position: " + roundedList(obj.get_position()))
            break