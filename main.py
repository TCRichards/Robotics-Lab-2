#!/usr/bin/env pybricks-micropython

'''
Python submission for COMP 581 Lab 2
Member 1: Thomas Richards; 7301-22201
Member 2: Yasmine Zeffrey; 7301-26635
'''


from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

from pose import Pose
from collections import deque
import time
from math import pi as PI
from threading import Thread

# Initialize hardware components
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
bump_front = TouchSensor(Port.S1)   # Use two bump sensors for redundancy
bump_side = TouchSensor(Port.S2)
ultra = UltrasonicSensor(Port.S4)

# Physical information about the robot
r_wheel = 2.5            # Radius of the wheels (cm)
L = 9.15                 # Distance between the two wheels (cm)
pose = Pose(r_wheel, L, leftMotor, rightMotor)  # Object storing the robot's pose at any time
updateThread = Thread(target=pose.monitorPose)

# Some adjustable variables to mess with
safeSpeed = 300     # deg / s
sprintSpeed = 300


# ====================== Helper Functions ========================================

def toDegrees(angle):
    return angle * 180 / PI


def toRadians(angle):
    return angle * PI / 180


# Hangs the code until the center gray button on the brick is pressed
def waitForEnter():
    while Button.CENTER not in brick.buttons():
        pass
    return True


# Move a certain distance, using idealized kinematic equations
def moveDistance(dist, pause=False):
    rots = dist / (2 * PI * r_wheel)
    dTheta = rots * 360

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

    moveSpeed = -safeSpeed
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


# Traces an arc of radius R the given angular speed
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

    # print('Turning {}, {} degrees'.format(theta_L, theta_R))
    leftMotor.run_angle(u_L, theta_L, Stop.BRAKE, False)
    rightMotor.run_angle(u_R, theta_R, Stop.BRAKE, pause)


# ======================================== Wall Tracing-Specific Functions =========================================

# Create a map of distances to the nearest obstacle in each angle
def scanDistances(angle=360):
    print('Scanning Distances')
    # Total time for rotation in ms
    totalTime = rotate(angle, reduceAngle=False, pause=False)
    # Thread(target=rotate, args=(angle,), kwargs={'reduceAngle': False, 'pause': True}).start()  # Run the rotation in a separate thread (pause=False not working?)
    dt = totalTime / abs(angle)          # Time per angle in s
    dists = {}  # Dict where each entry is {angle : distance (cm)}
    for i in range(int(abs(angle))):
        dist_cm = ultra.distance() / 10
        dists.update({i: dist_cm})
        time.sleep(dt)
    if angle != 360:    # Unless we made a full rotation, turn back to the original position
        rotate(-angle, pause=True)
    return dists


# TODO: Implement this
def checkWallEnd():
    print('CHecking wall end')
    cutoffDist = 60
    distDict = scanDistances(angle=-80)
    atEnd = True
    for item in distDict.items():           # Entries stored as (Angle : Distance (cm))
        if item[0] < 10:
            distDict.pop(item[0])           # Don't mess with angles < 5 degrees -- may pick up the original obstacle
            continue
        atEnd &= (item[1] > cutoffDist)     # If any of the distances are above the cutoff, this will be false
    if atEnd:   # If we are indeed at the end, exit out
        return True
        print('At the end')
    else:       # If we're not at the end, rotate the robot so that the ultrasound sensor directly faces the obstacle
        turnAngle = -0.4 * min((item[1], item[0]) for item in distDict.items())[1]  # Find the angle corresponding to the minimum distance
        rotate(turnAngle, pause=True)
        return False
     

# Implements PID to determine the magnitude of the control output necessary to make the robot approach the set distance
# Returns the control input u, which corresponds to how much faster the left wheel turns than the right
def PID(x_set, pastQueue, dt):
    queueSize = 100

    # Gains for P, I, and D -- Determine by trial and error
    k_p = 6.0
    k_I = 0.05
    k_d = 2.3

    dist_cm = ultra.distance() / 10
    if dist_cm > 180:  # Device error that occurs when too close or if the sensor is angled to sharply
        return -1000   # Turn right ASAP! This will be handled explicitly in the traceWall() function

    err = dist_cm - x_set

    if len(pastQueue) >= queueSize:
        pastQueue.pop()
    pastQueue.appendleft(err)

    # Approximate an integral by summing over the most recent measurements
    integral = sum(pastQueue) * dt / len(pastQueue)
    
    if len(pastQueue) > 3:
        # Lastly approximate the derivative by subtracting a recent measurement from the current measurement
        deriv = (err - list(pastQueue)[-3]) / dt
    else:
        deriv = 0

    # Positive outputs correspond to being further from the set point than desire
    return k_p * err + k_I * integral + k_d * deriv    # The control input that we want


def traceWall(x_set=20):
    pastQueue = deque()
    endTime = time.time() + 40
    lastResetTime = 0
    lastWallCheckTime = 0
    minDelay = 2
    startTime = time.time()
    lastTime = startTime
    while time.time() < endTime:
        dt = time.time() - lastTime
        u = PID(x_set, pastQueue, dt)
        if u is None:
            continue

        if u == -1000 and (time.time() > lastWallCheckTime + minDelay) and (time.time() > startTime + 3 * minDelay):          # Output given when the distance measured is 255 cm
            lastWallCheckTime = time.time()
            lastResetTime = time.time()
            if checkWallEnd():  # Scans distances and returns if this is really the end of the wall
                return True
            else:
                continue

        # If we're still along the wall, map the input to a correction -> 255 means that we're very close to an obstacle
        correction = u
        # Set bounds for the correction to avoid spinning out of control
        if correction < 0:
            correction = max(-100, correction)
        else:
            correction = min(100, correction)

        # This occurs when we're really close to the wall or bumping into it -- especially right at the bend
        if (correction == -1000 and (time.time() > lastResetTime + minDelay) and (time.time() > startTime + 3 * minDelay)) \
            or bump_front.pressed() or bump_side.pressed():
            print('CORRECTING')
            reverseDistance(12, pause=True)
            rotate(-75, reduceAngle=False, pause=True)  # Doesn't seem right
            lastResetTime = time.time()

        # u = 0 -> continue straight
        # u >> 0 -> turn left
        # u << 0 -> turn right
        leftMotor.run(safeSpeed - correction)
        rightMotor.run(safeSpeed + correction)


# Rotate the robot in place by the given angle
def rotate(angle, reduceAngle=False, pause=True):
    u = 200 # Angular speed
    if reduceAngle:  # Determine the minimum angle that needs to be travelled
        raise Exception('reduceAngle has not been implemented yet!')
    if angle > 0:
        # Angular velocities of the wheels (degrees)
        u_R = u
        u_L = -u
    else:
        u_R = -u
        u_L = u

    du = abs(u_R - u_L)
    t = 1000 * L * abs(angle) / (r_wheel * (du))    # Time to run the motors for (ms)
    t *= 1.37                                      # Fudge factor (Needs to be experimentally tuned)
    leftMotor.run_time(u_L, t, Stop.BRAKE, False)
    rightMotor.run_time(u_R, t, Stop.BRAKE, pause)

    # pose.theta += toRadians(angle)
    return t / 1000    # Return the total rotation time in s
    

def returnToStartTheta():  # YZ
    print('Reached the Wall\'s End: Theta = {} Degrees'.format(toDegrees(pose.theta)))
    rotate(-toDegrees(pose.theta), reduceAngle=False, pause=True)


def leaveWall():  # YZ
    brick.sound.beep(440, 1, 1)
    rotate(-(toDegrees(pose.theta) + 90), reduceAngle=False, pause=True)
    moveDistance(18, pause=True) # Continue a little past the end of the wall to avoid catching it
    time.sleep(0.5)
    returnToStartTheta()  # rotates to original angle
    moveDistance(85, pause=True)  # moves forward 0.85 meters and stops (aiming for 0.7 m past the obstacle)


def bumpAndTurn():
    moveForever(400)    # Don't care much about accuracy -- just time
    latestEnd = time.time() + 30    
    # Add timer for redundancy -- Assume this won't last more than 30 s
    while not bump_front.pressed() and time.time() < latestEnd:
        pass
    reverseDistance(18, pause=True)
    rotate(-95, reduceAngle=False, pause=True)
    pose.theta += toRadians(-95)    # Hard code in this rotation so that we can delay dead reckoning


def calibratePose():
    timeArr = [9e-4]
    results = []
    for t in timeArr:
        pose.theta = 0
        updateThread = Thread(target=pose.monitorPose, args=(t,))
        updateThread.start()
        rotate(360, pause=True)
        results.append(pose.theta)
    print(results)


# ======================= Main function to organize behavior ================

if __name__ == '__main__':

    print('Beginning Run.  Press ENTER to start...')
    # print(checkWallEnd())
    waitForEnter()

    # # # Step 1: Move forward (30 - 60 cm) until the robot is within 30 cm of the wall    
    # # # Step 2: Turn right and allign parallel to the wall
    bumpAndTurn()
    updateThread.start()

    # # # Step 3: Move forward and continue tracing the wall as far as it extends
    traceWall(x_set=18)

    # # # Step 4: Once the wall ends, turn left and leave the wall
    leaveWall()
