from math import cos, sin
from math import pi as PI


class Pose:

    def __init__(self, r_wheel, L_axle, leftMotor, rightMotor):
        self.r = r_wheel
        self.L = L_axle
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor

        self.x = 0
        self.y = 0
        self.theta = 0      # Need to store the angle in radians for kinematics calculations
        self.lastTime = 0
    

    # Updates the pose using the current state of each motor
    def update(self):
        dt = time.time() - self.lastTime()  # Time since last update in s
        
        # Get the angular velocities in radians / sec
        u_L = leftMotor.speed() * PI / 180
        u_R = rightMotor.speed() * PI / 180

        # Linear velocities of the robot's wheels
        V_L = u_L * self.r
        V_R = u_R * self.r

        # Relevant derivatives for calculations
        V = 0.5 * (V_L + V_R)   # Robot's average velocity
        omega = (self.r / self.L) * (u_R - u_L)

        # Update the heading using these derivatives
        self.x += V * cos(self.theta) * dt  # math's cosine function uses radians
        self.y += V * sin(self.theta) * dt
        self.theta += omega * dt

        self.lastTime = time.time()

    # Must call this from a separate thread
    # Call this once from a separate thread in order to continuously update the pose throughout the program
    def monitorPose(self, dt):
        while True:
            update(leftMotor, rightMotor)
            time.sleep(dt)
