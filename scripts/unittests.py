#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Unit tests for Sawppy logic executed with straight Python (no ROS)."""

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

    def well_formed_results(self, calculated_results):
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

    def test_straight_forward(self):
        """Test simple case of rover going straight forward."""
        calculated_results = self.calculator.calculate(0, 1.0)

        self.well_formed_results(calculated_results)

        for wheel in calculated_results:
            self.assertEqual(
                wheel.angle,
                0,
                'Unexpected angle wheel {0}'.format(wheel.name),
            )
            self.assertEqual(
                wheel.velocity,
                1.0,
                'Unexpected velocity wheel {0}'.format(wheel.name),
            )
            self.log.debug('{0} {1} {2}'.format(
                wheel.name,
                wheel.angle,
                wheel.velocity,
                ),
            )


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    unittest.main()
