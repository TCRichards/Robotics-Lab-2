#!/usr/bin/env pybricks-micropython

'''
Python submission for COMP 581 Lab 1
Member 1: Thomas Richards; 7301-22201
Member 2: Yasmine Zeffrey; 7301-26635
'''


from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

from collections import deque
import time


PI = 3.14159

# Initialize hardware components
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
bump_front = TouchSensor(Port.S1)   # Use two bump sensors for redundancy
bump_side = TouchSensor(Port.S2)
ultra = UltrasonicSensor(Port.S4)

# Physical information about the robot
r_wheel = 2.5                       # Radius of the wheel in cm
L = 9.15                            # Distance between the two wheels

# Some adjustable variables to mess with
safeSpeed = 300
sprintSpeed = 300


# ====================== Helper Functions ========================================

# Hangs the code until the center gray button on the brick is pressed
def waitForEnter():
    while Button.CENTER not in brick.buttons():
        pass
    return True


# Move a certain distance, using idealized kinematic equations
def moveDistance(dist, pause=False):
    rots = dist / (2 * PI * r_wheel)
    dTheta = rots * 360
    print('In order to move {} cm, the wheel must rotate {} times'.format(dist, rots))

    leftMotor.run_angle(safeSpeed, dTheta, Stop.BRAKE, False) #, stop_type=Stop.BRAKE, wait=False)
    rightMotor.run_angle(safeSpeed, dTheta, Stop.BRAKE, pause) # , stop_type=Stop.BRAKE, wait=False)


# Turn the motors on an move indefinitely
def moveForever(speed):
    leftMotor.run(speed)
    rightMotor.run(speed)


def brake():
    leftMotor.stop(Stop.BRAKE)
    rightMotor.stop(Stop.BRAKE)


def reverseDistance(dist, pause=False):
    rots = dist / (2 * PI * r_wheel)
    dTheta = rots * 360
    print('In order to reverse {} cm, the wheel must rotate {} times'.format(dist, rots))

    moveSpeed = -safeSpeed
    print('Moving at speed {:.2f} deg/s'.format(moveSpeed))
    leftMotor.run_angle(moveSpeed, dTheta, Stop.BRAKE, False) #, stop_type=Stop.BRAKE, wait=False)
    rightMotor.run_angle(moveSpeed, dTheta, Stop.BRAKE, pause) # , stop_type=Stop.BRAKE, wait=False)


# Continue to move forward until the ultrasound sensor tells us we're within finalDist of an obstacle
def approachObstacle(finalDist):
    moveForever(safeSpeed)  # Move slowly enough to stop quickly
    while True:
        dist_cm = ultra.distance() / 10
        if dist_cm < finalDist:
            brake()
            break


# Turn in the given direction at the given speed
def turn(direction, R, omega, sweepAngle, pause=False):
    # Expressions for the necessary angular velocities (assumes counterclockwise)
    u_L = (omega / r_wheel) * (R - L / 2)
    u_R = (omega / r_wheel) * (R + L / 2)
    if direction == Direction.CLOCKWISE:    # Swap u_L and u_R to change direction
        tmp = u_L
        u_L = u_R
        u_R = tmp
    
    # Convert each from radians to degrees
    u_L *= 180 / PI
    u_R *= 180 / PI

    theta_L = (R + L / 2) * sweepAngle / r_wheel   # Angle (degrees) that each wheel should rotate
    theta_R = (R - L / 2) * sweepAngle / r_wheel

    print('Turning {}, {} degrees'.format(theta_L, theta_R))
    leftMotor.run_angle(u_L, theta_L, Stop.BRAKE, False)
    rightMotor.run_angle(u_R, theta_R, Stop.BRAKE, pause)


# Implements PID to determine the magnitude of the control output necessary to make the robot approach the set distance
# Returns the control input u, which corresponds to how much faster the left wheel turns than the right
def PID(x_set, pastQueue):
    queueSize = 5

    k_p = 5
    k_I = 0.05
    k_d = 0.5

    dist_cm = ultra.distance() / 10
    if dist_cm == 255:  # Device error that occurs when too close
        return -1000    # Turn right ASAP! This will be handled explicitly in the traceWall() function

    err = dist_cm - x_set

    if len(pastQueue) >= queueSize:
        pastQueue.pop()
    pastQueue.appendleft(err)


    # Approximate an integral by summing over the most recent measurements
    integral = sum(pastQueue) / len(pastQueue)
    
    if len(pastQueue) > 3:
        # Lastly approximate the derivative by subtracting a recent measurement from the current measurement
        deriv = err - list(pastQueue)[-3]
    else:
        deriv = 0

    # print('P term = {}\nI term = {}\nD term = {}'.format(k_p * err, k_I * integral, k_d * deriv))

    # Positive outputs correspond to being further from the set point than desire

    return k_p * err + k_I * integral + k_d * deriv    # The control input that we want


def traceWall(x_set=25):
    pastQueue = deque()
    endTime = time.time() + 30
    lastResetTime = 0
    minDelay = 1.5
    while time.time() < endTime:
        u = PID(x_set, pastQueue)
        if u is None:
            continue

        scale = 0.5
        correction = scale * u
        # if u > 0:   # Try to avoid overcorrections by capping maximum values
        #     print('Turn Left at {}'.format(correction))
        # else:
        #     print('Turn Right at {}'.format(abs(correction)))

        # This occurs when we're really close to the wall or bumping into it -- especially right at the bend
        if (correction < -400 or bump_front.pressed() or bump_side.pressed()) and time.time() > lastResetTime + minDelay:
            reverseDistance(5, pause=True)
            rotate(-50, reduceAngle=True, pause=True)  # Doesn't seem right
            print('REPOSITIONING TO THE RIGHT')
            lastResetTime = time.time()


        # u = 0 -> continue straight
        # u >> 0 -> turn left
        # u << 0 -> turn right
        leftMotor.run(safeSpeed - correction)
        rightMotor.run(safeSpeed + correction)


# Create a map of distances to the nearest obstacle in each angle
def scanDistances():                           
    totalTime = rotate(360, pause=False)    # Total time for rotation in ms
    dt = totalTime / 360           # Time per angle in s
    dists = {}  # Dict where each entry is {angle : distance (cm)}
    for i in range(360):
        dist_cm = ultra.distance() / 10
        dists.update({i: dist_cm})
        time.sleep(dt)
    return dists
    

# Rotate the robot in place by the given angle
def rotate(angle, reduceAngle=False, pause=False):
    u = 250 # Angular speed
    if reduceAngle:  # Determine the minimum angle that needs to be travelled
        angle %= 360
    
    if angle > 180: # We can more efficiently reach the given angle by going in the opposite direction
        angle = 360 - angle
        u_R = -u
        u_L = u
    else:
        # Angular velocities of the wheels (degrees)
        u_R = u
        u_L = -u
    du = abs(u_R - u_L)
    t = 1000 * L * abs(angle) / (r_wheel * (du))    # Time to run the motors for (ms)
    t *= 1.15    # Fudge factor (Needs to be experimentally tuned)
    leftMotor.run_time(u_L, t, Stop.BRAKE, False)
    rightMotor.run_time(u_R, t, Stop.BRAKE, pause)
    return t / 1000    # Return the total rotation time in s
    

def p1():
    moveForever(400)    # Don't care much about accuracy -- just time
    latestEnd = time.time() + 30    
    # Add timer for redundancy -- Assume this won't last more than 30 s
    while not bump_front.pressed() and time.time() < latestEnd:
        pass
    reverseDistance(20, pause=True)
    rotate(-90, reduceAngle=True, pause=True)

# ======================= Main function to organize behavior ================


if __name__ == '__main__':
    print('Beginning Run.  Press ENTER to start...')
    waitForEnter()

    # scanDistances()

    # Step 1: Move forward (30 - 60 cm) until the robot is within 30 cm of the wall    
    # Step 2: Turn right and allign parallel to the wall
    # p1()

    # Step 3: Move forward and continue tracing the wall as far as it extends
    traceWall(x_set=20)

    # Step 4: Once the wall ends, turn left and leave the wall
    # Use scanDistances() to determine where the wall ends for real, and doing a similar tracing
    # for its entire duration


    # Step 5: Having left the wall, drive exactly 0.7 m away from the original side of the wall and brake
