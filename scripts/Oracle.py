# python_live_plot.py

import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import numpy
import sys


def vectorLength(x1, y1, x2, y2):
    # Vector AB : c
    x = x1 - x2
    y = y1 - y2

    # Length for c
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


# punkt 1 und punkt 2, Punkt 3 ist der dazwischen
def upOrDown(x1, y1, x2, y2, x3, y3):
    print("X1 : ", x1, " Y1 : ", y1)
    print("X2 : ", x2, " Y2 : ", y2)
    print("X3 : ", x3, " Y3 : ", y3)
    p1 = ((x3 * 100) / x1)
    if p1 > 100:
        p1 = p1 - 100
    p2 = 100 - p1
    print(p1)
    print(p2)
    if y1 > y2:
        y = y2 + ((p2 * y2) / 100)
    else:
        y = y2 - ((p2 * y2) / 100)
    print(y)
    return y3 > y1


def calculateAngles(angle, skipp):
    print("Angle : ", angle)
    epsilon = 0.5
    # Centimeter
    # Calculate C
    circleAngle = angle
    circleRadius = 60
    circleCenterX = 0
    circleCenterY = 35
    rad = math.radians(circleAngle)
    # Point to Given rad
    Cx = circleCenterX + circleRadius * math.cos(rad)
    Cy = circleCenterY + circleRadius * math.sin(rad)

    # Get Wrist Position
    cbLength = 8.25
    caLength = 9.47

    cAx = Cx
    cAy = Cy

    rad = math.radians(circleAngle)

    cCx = cAx + cbLength * math.cos(rad)
    cCy = cAy + cbLength * math.sin(rad)

    cBx = cCx + caLength * math.cos(math.radians(circleAngle - 90))
    cBy = cCy + caLength * math.sin(math.radians(circleAngle - 90))

    Cx = cBx
    Cy = cBy

    A = 0
    B = 0

    #Ax ist die distanz zum drehe tisch
    Ax = 62
    Ay = 8.92

    Bx = 0
    By = 0

    # Forearm Length
    cLength = 42.5

    # Upperarm Length
    aLength = 39.2

    bLength = vectorLength(Ax, Ay, Cx, Cy)

    if bLength > cLength + aLength:
        print("Error Arm to short")
        print("Needed length : ", vectorLength(Ax, Ay, Cx, Cy))
        print("Arm length : ", cLength + aLength)
        sys.exit()

    beta = math.acos((math.pow(cLength, 2) + math.pow(aLength, 2) - math.pow(bLength, 2)) / (2 * aLength * cLength))

    elbow_joint = math.radians(180) - beta

    end = False
    for a in range(361):
        rad = math.radians(a)
        Bx = Ax + cLength * math.cos(rad)
        By = Ay + cLength * math.sin(rad)

        for b in range(361):
            # Upperearm head Position
            rad = math.radians(b)
            Cx_maybe = Bx + aLength * math.cos(rad)
            Cy_maybe = By + aLength * math.sin(rad)

            CisEqual = abs(Cx_maybe - Cx) < epsilon and abs(Cy_maybe - Cy) < epsilon

            NoVectorOverOne = abs(vectorLength(Ax, Ay, Bx, By) - cLength) < epsilon and \
                              abs(vectorLength(Bx, By, Cx_maybe, Cy_maybe) - aLength) < epsilon

            if CisEqual and NoVectorOverOne:
                shoulder_joint = math.radians(90) + math.radians(90 - a)
                print("b : ", math.degrees(elbow_joint), "elbow_joint : ", elbow_joint)
                print("a : ", a, "shoulder_lift_joint : ", math.radians(180 - a))
                A = shoulder_joint
                B = elbow_joint
                end = True
                break
        if end:
            break

    if not end:
        print("No Result")

    # Winkel zwischen dreieck und arm :
    dAx = cBx
    dAy = cBy

    dBx = Bx
    dBy = By

    dCx = cCx
    dCy = cCy

    daLength = vectorLength(dBx, dBy, dCx, dCy)
    dbLength = vectorLength(dAx, dAy, dCx, dCy)
    dcLength = vectorLength(dBx, dBy, dAx, dAy)
    #upOrDown(dCx, dCy, dBx, dBy, dAx, dAy)
    dAlpha = math.acos(
        (math.pow(dbLength, 2) + math.pow(dcLength, 2) - math.pow(daLength, 2)) / (2 * dbLength * dcLength))

    print("c : ", math.degrees(dAlpha), "wrist_1_joint : ", math.radians(270) - dAlpha)

    if end and not skipp:
        # plt.style.use('fivethirtyeight')

        # Create figure
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)

        # Create Forearm
        plt.plot([Ax, Bx], [Ay, By], color="red")
        plt.text(Ax, Ay, "A", fontsize=12)

        # Create Upperarm
        plt.plot([Bx, Cx], [By, Cy], color="green")
        plt.text(Bx, By, "B", fontsize=12)

        # Create C
        plt.plot([Ax, Cx], [Ay, Cy], color="blue")
        plt.text(Cx, Cy, "C", fontsize=12)

        plt.plot([cAx, cBx], [cAy, cBy], color="black")

        plt.plot([cBx, cCx], [cBy, cCy], color="black")

        plt.plot([cAx, cCx], [cAy, cCy], color="black")

        # Camera Arm
        plt.plot([circleCenterX, cAx], [circleCenterY, cAy], color="yellow")

        # Assign circles to variables - do not fill the centre circle!
        centreCircle = plt.Circle((circleCenterX, circleCenterY), circleRadius, color="red", fill=True)

        # Draw the circles to our plot
        ax.add_patch(centreCircle)

        plt.show()

    return  [elbow_joint, -(math.radians(180 - a)), -1.63, -(math.radians(270) - dAlpha), -1.63, -0.0]

if __name__ == "__main__":
    for i in numpy.arange(0, 90, 1):
        print(calculateAngles(i, True))
        print("")
