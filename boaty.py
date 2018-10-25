from turtle import *
import numpy as np
from short_course import run
from random import randint

windDir = 270
boatCurrDir = 90
boatDesDir = 90
targetList = [(0, 200), (200, 0), (0, -200), (-200, 0), (0, 200)]
wind = Turtle()
wind.pu()
wind.setpos(0, 400)
boat = Turtle()
boat.setheading(boatCurrDir)

"""
def angleTo(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return (180* atan2(dy, dx) / pi) % 360

def adjustTo(current, goal):
    change = 1
    if current == goal:
        return goal
    elif current > goal:
        if abs(goal-current) > 180:
            change = -1
        return current + change
    elif current < goal:
        if abs(goal-current) > 180:
            change = -1
        return current - change
"""

for targetPos in targetList:
    print("Heading to ", targetPos)
    while boat.distance(targetPos) > 5:
        windVector = [np.cos(windDir), np.sin(windDir)]
        boatNewDir = get_heading(boatPos, targetPos, windVector)
        #boatCurrDir = adjustTo(boatCurrDir, boatNewDir)

        boat.setheading(boatNewDir)
        boat.forward(2)
        boat.position()

        windDir += randint(-4, 4)
        wind.setheading(windDir)
