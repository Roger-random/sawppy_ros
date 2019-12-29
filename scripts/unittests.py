#!/usr/bin/python3
import chassis_wheel_calculator as cwc
import unittest
import math

max_angle = 2 * math.pi

class TestWheelCalculator(unittest.TestCase):
    def setUp(self):
        """Configure test chassis with default Sawppy dimensions"""
        self.test_chassis = list()

        self.test_chassis.append(cwc.chassis_wheel('front_left',  11.375,   9.125))
        self.test_chassis.append(cwc.chassis_wheel('front_right', 11.375,  -9.125))
        self.test_chassis.append(cwc.chassis_wheel('mid_left',         0,  10.375))
        self.test_chassis.append(cwc.chassis_wheel('mid_right',        0, -10.375))
        self.test_chassis.append(cwc.chassis_wheel('rear_left',      -10,   9))
        self.test_chassis.append(cwc.chassis_wheel('rear_right',     -10,  -9))

        self.calculator = cwc.chassis_wheel_calculator(self.test_chassis)

    def well_formed_results(self, results):
        """Verify calculated results are in expected form"""
        self.assertEqual(len(results), 6, "Six wheels in should have resulted in six wheels out")

        test_names_checklist=list(['front_left', 'front_right', 'mid_left', 'mid_right', 'rear_left', 'rear_right'])
        test_names_encountered=list()

        for wheel in results:
            self.assertIn(wheel.name, test_names_checklist, "Encountered name not on the checklist")
            test_names_encountered.append(wheel.name)
            test_names_checklist.remove(wheel.name)
            self.assertGreaterEqual(wheel.angle, -max_angle)
            self.assertLessEqual(wheel.angle, max_angle)

        self.assertEqual(len(test_names_checklist), 0, "Wheel names were expected but never encountered: {0}".format(test_names_checklist))
        self.assertEqual(len(test_names_encountered), 6, "Less than six names on the checklist was encountered")

    def test_straight_forward(self):
        results = self.calculator.calculate(0, 1.0)

        self.well_formed_results(results)

        for wheel in results:
            print(wheel.name, wheel.angle, wheel.velocity)

if __name__ == '__main__':
    unittest.main()