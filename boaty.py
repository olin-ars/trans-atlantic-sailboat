from turtle import *
import numpy as np
from time import sleep
from short_course import run
from random import randint

windDir = int(input("wind angle: ")) #90  #wind source direction
boatCurrDir = 0
targetList = [(0, 120), (120, 120), (0, 0), (120, 0), (120, -120), (0, -120), (0, 0)]
wind = Turtle()
wind.pu()
wind.setpos(250*np.cos(windDir*np.pi/180), 250*np.sin(windDir*np.pi/180))
boat = Turtle()
boat.setheading(boatCurrDir)

for targetPos in targetList:
    while boat.distance(targetPos) > 5:
        #windVector = [np.cos(windDir*np.pi/180), np.sin(windDir*np.pi/180)]
        boatNewDir = run(boat.position(), boat.heading(), targetPos, windDir)
        #boatCurrDir = adjustTo(boatCurrDir, boatNewDir)

        boat.setheading(boatNewDir)
        boat.forward(4)
        sleep(0.05)
        boat.position()

        #windDir += randint(-4, 4)
        wind.setheading(windDir+180)
