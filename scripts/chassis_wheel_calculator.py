#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Chassis wheel calculator and supporting classes."""

import math


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
            angle, velocity = self._wheel_angle_velocity(
                angular, linear, wheel,
            )
            answer = ChassisWheelAngleSpeed(wheel.name, angle, velocity)
            calculated_results.append(answer)

        return calculated_results

    def _wheel_angle_velocity(self, angular, linear, wheel):
        """Calculate the angle and velocity for a single wheel.

        Args:
            angular : desired angular velocity for robot chassis
            linear : desired linear velocity for robot chassis
            wheel : instance of ChassisWheel

        Returns:
            angle, velocity : desired values for wheel
        """
        if angular == 0:
            # Heading directly forward or backward
            angle = 0
            velocity = linear
        else:
            # Linear velocity is specified in meters/sec. Angular velocity is
            # in radians/sec. How does a simple division between them give us
            # the center point of turn? This deceptively simple calculation is
            # a result of a lot of terms cancelling out when we work through
            # the math long hand.
            #
            # For one example when this works, consider the case when angular
            # velocity is 2*PI, a command for the robot to turn 360 degrees
            # around. The robot's path forms a circle with circumference of
            # linear velocity. The straight division gives us the radius of
            # this circle, which is also the center point of turn.
            #
            # The formula continues to hold for other values of angular.
            # The linear distance traveled is some fraction of a circle, and
            # dividing by the angular velocity returns the center.
            turn_center = linear / angular

            # Dimensions of a triangle representing the wheel relative to
            # center of turn
            opposite = wheel.offset_front
            adjacent = turn_center - wheel.offset_left
            hypotenuse = math.sqrt(pow(opposite, 2)+pow(adjacent, 2))

            # Now we have everything we need to calculate angle and velocity.
            if opposite == 0:
                angle = 0
            else:
                angle = math.atan(opposite/adjacent)

            # Counterintuitively, the 'linear' parameter is not directly used
            # to calculate the desired wheel velocity. Again this was the
            # result of a lot of terms cancelling out when we work through the
            # math long hand. The magnitude of 'linear' is present as part of
            # the 'hypotenuse' value, but we have lost the sign so we have to
            # copy the sign to our desired velocity.
            velocity = math.copysign(angular*hypotenuse, linear)

            # If center of turn is inside the wheel, the pivot point is on
            # the opposite side of normal so we need to reverse direction.
            if self._turn_center_is_inside_wheel(turn_center, wheel):
                velocity = velocity * -1
        return angle, velocity

    def _turn_center_is_inside_wheel(self, turn_center, wheel):
        """Whether the given center point is inside the wheel track.

        Args:
            turn_center : Y-axis coordinate of center of turn.
            wheel : instance of ChassisWheel

        Returns:
            True if the robot is pivoting about a point somewhere inside its
            wheels, which typically means a point under its body.
        """
        if turn_center >= 0:
            if wheel.offset_left > 0 and wheel.offset_left > turn_center:
                return True
        elif wheel.offset_left < 0 and wheel.offset_left < turn_center:
            return True
        return False
