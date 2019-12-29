#!/usr/bin/python3
import sys

class chassis_wheel:
    """Information needed to calculate angle and speed for a specific wheel
    Axis orientation conforms to REP103. (+X is forward, +Y is left, +Z is up)
    https://www.ros.org/reps/rep-0103.html"""
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

class chassis_wheel_angle_speed:
    """Results of chassis geometry calculation for the named wheel"""
    def __init__(self, name, angle, velocity):
        self.name = name
        self.angle = angle
        self.velocity = velocity

class chassis_wheel_calculator:
    """Given a overall desired motion for a robot chassis, calculate the
    individual angle and velocity required for each wheel on board the robot
    chassis.
    
    Chassis configuration is given as a list of chassis_wheeel class, one
    for each wheel."""
    def __init__(self, chassis):
        """Initializes an instance of chassis wheel calculator class.
        chassis -- a list of chassis_wheel instances, one for each wheel
                aboard the robot."""
        self.chassis = chassis

    def calculate(self, angular=0, linear=0):
        """Given a desired angular and linear velocity for robot chassis
        center, calculate necessary angle and speed for each wheel on board.
        angular -- desired turning speed for robot chassis in radians/sec
                about the Z axis, counter-clockwise is positive.
        linear -- desired forward speed for robot chassis in meters/sec
                along the X axis. forward is positive."""
        results = list()
        for wheel in self.chassis:
            answer = chassis_wheel_angle_speed(wheel.name,0,0)
            results.append(answer)

        return results

if __name__ == '__main__':
    print("Running under Python " + str(sys.version_info[0]))
    print("This class has no functionality when run directly.")
