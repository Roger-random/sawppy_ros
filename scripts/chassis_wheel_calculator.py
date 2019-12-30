#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Chassis wheel calculator and supporting classes."""


class ChassisWheel(object):
    """Information needed to calculate angle and speed for a specific wheel.

    Axis orientation conforms to REP103. (+X is forward, +Y is left, +Z is up)
    https://www.ros.org/reps/rep-0103.html
    """

    def __init__(self, name, offset_front, offset_left):
        """Initialize a chassis wheel instance with X & Y relative to center.

        Args:
            name : name for this wheel
            offset_front : front/back relative to center, positive forward.
            offset_left : left/right relative to center, positive left.
        """
        self.name = name
        self.offset_front = offset_front
        self.offset_left = offset_left


class ChassisWheelAngleSpeed(object):
    """Results of chassis geometry calculation for the named wheel."""

    def __init__(self, name, angle, velocity):
        """Initialize a chassis wheel with desired angle and velocity.

        Args:
            name : name for this wheel
            angle : steering angle for this wheel in radians.
            velocity : rolling velocity for this wheel in meters/second
        """
        self.name = name
        self.angle = angle
        self.velocity = velocity


class ChassisWheelCalculator(object):
    """Chassis wheel angle and velocity calculator.

    Given a overall desired motion for a robot chassis, calculate the
    individual angle and velocity required for each wheel on board the robot
    chassis.

    Chassis configuration is given as a list of chassis_wheeel class, one
    for each wheel.
    """

    def __init__(self, chassis):
        """Initialize an instance of chassis wheel calculator class.

        Args:
            chassis : a list of ChassisWheel instances
        """
        self.chassis = chassis

    def calculate(self, angular=0, linear=0):
        """Calculate angle and speed of each wheel for desired velocities.

        Args:
            angular : desired turning speed for robot chassis in radians/sec
            linear : desired forward speed for robot chassis in meters/sec

        Returns:
            A list of desired angle and speed for each chassis wheel
        """
        calculated_results = []
        for wheel in self.chassis:
            answer = ChassisWheelAngleSpeed(wheel.name, 0, linear)
            calculated_results.append(answer)

        return calculated_results
