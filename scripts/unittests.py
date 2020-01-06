#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Unit tests for Sawppy logic executed with straight Python (no ROS)."""

import csv
import logging
import math
import sys
import unittest

from chassis_wheel_calculator import ChassisWheel, ChassisWheelCalculator

MAX_ANGLE = 2 * math.pi

# Sawppy chassis geometry in meters
SAWPPY_WHEELBASE_FRONT = 0.285
SAWPPY_WHEELBASE_MID = 0
SAWPPY_WHEELBASE_REAR = -0.257
SAWPPY_TRACK_FRONT = 0.23
SAWPPY_TRACK_MID = 0.26
SAWPPY_TRACK_REAR = 0.23


class TestWheelCalculator(unittest.TestCase):
    """Tests for wheel calculator class."""

    def setUp(self):
        """Configure test chassis with default Sawppy dimensions."""
        self.log = logging.getLogger('TestLog')

        self.test_chassis = []
        self.test_chassis.append(
            ChassisWheel(
                'front_left',  SAWPPY_WHEELBASE_FRONT,  SAWPPY_TRACK_FRONT,
                ),
            )
        self.test_chassis.append(
            ChassisWheel(
                'front_right', SAWPPY_WHEELBASE_FRONT, -SAWPPY_TRACK_FRONT,
                ),
            )
        self.test_chassis.append(
            ChassisWheel(
                'mid_left',      SAWPPY_WHEELBASE_MID,  SAWPPY_TRACK_MID,
                ),
            )
        self.test_chassis.append(
            ChassisWheel(
                'mid_right',     SAWPPY_WHEELBASE_MID, -SAWPPY_TRACK_MID,
                ),
            )
        self.test_chassis.append(
            ChassisWheel(
                'rear_left',    SAWPPY_WHEELBASE_REAR,  SAWPPY_TRACK_REAR,
                ),
            )
        self.test_chassis.append(
            ChassisWheel(
                'rear_right',   SAWPPY_WHEELBASE_REAR, -SAWPPY_TRACK_REAR,
                ),
            )

        self.calculator = ChassisWheelCalculator(self.test_chassis)

    def verify_well_formed_results(self, calculated_results):
        """Verify calculated_results are in expected form.

        Args:
            calculated_results : list of six chassis_wheel_angle_speed.
        """
        self.assertEqual(
            len(calculated_results),
            6,
            'Six wheels in should have resulted in six wheels out',
            )

        test_names_checklist = [
            'front_left',
            'front_right',
            'mid_left',
            'mid_right',
            'rear_left',
            'rear_right',
            ]
        test_names_encountered = []

        for wheel in calculated_results:
            self.assertIn(
                wheel.name,
                test_names_checklist,
                'Encountered name not on the checklist',
                )
            test_names_encountered.append(wheel.name)
            test_names_checklist.remove(wheel.name)
            self.assertGreaterEqual(wheel.angle, -MAX_ANGLE)
            self.assertLessEqual(wheel.angle, MAX_ANGLE)

        self.assertEqual(
            len(test_names_checklist),
            0,
            'Wheel names were expected but never encountered: {0}'.format(
                test_names_checklist,
                ),
            )
        self.assertEqual(
            len(test_names_encountered),
            6,
            'Less than six names on the checklist was encountered',
            )

    def verify_wheel_angle_velocity(self, wheel, test_case):
        """Given a ChassisWheelAngleSpeed, compare against test case data.

        Args:
            wheel : a computed instance of ChassisWheelAnglespeed
            test_case : a row from the test case CSV file
        """
        self.assertAlmostEqual(
            float(wheel.angle),
            float(test_case['{0} {1}'.format(wheel.name, 'angle')]),
            places=4,
            msg='Unexpected angle wheel {0}'.format(wheel.name),
        )
        self.assertAlmostEqual(
            float(wheel.velocity),
            float(test_case['{0} {1}'.format(wheel.name, 'velocity')]),
            places=4,
            msg='Unexpected velocity wheel {0}'.format(wheel.name),
        )

    def perform_wheel_angle_velocity_calculation(self, test_case):
        """Compute and verify results specified by given test case.

        Args:
            test_case : a row from the test case CSV file
        """
        angular_velocity = float(test_case['angular'])
        linear_velocity = float(test_case['linear'])

        with self.subTest(msg='angular={0} linear={1}'.format(
            angular_velocity,
            linear_velocity,
            ),
        ):
            calculated_results = self.calculator.calculate(
                angular_velocity,
                linear_velocity,
            )
            self.verify_well_formed_results(calculated_results)
            for wheel in calculated_results:
                self.verify_wheel_angle_velocity(wheel, test_case)

    def test_angle_velocity_from_csv(self):
        """Run through test data stored as comma separated values file."""
        with open('chassis_wheel_calculator_tests.csv') as csvfile:
            test_case_reader = csv.DictReader(
                csvfile,
                fieldnames=(
                    'angular',
                    'linear',
                    'front_left angle',
                    'front_left velocity',
                    'front_right angle',
                    'front_right velocity',
                    'mid_left angle',
                    'mid_left velocity',
                    'mid_right angle',
                    'mid_right velocity',
                    'rear_left angle',
                    'rear_left velocity',
                    'rear_right angle',
                    'rear_right velocity',
                ),
            )
            for test_case in test_case_reader:
                self.perform_wheel_angle_velocity_calculation(test_case)


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    unittest.main()
