#!/usr/bin/python3
import chassis_wheel_calculator as cwc
import unittest

class TestWheelCalculator(unittest.TestCase):
    def setUp(self):
        self.test_chassis = list()

        self.test_chassis.append(cwc.chassis_wheel('front_left',  11.375,   9.125))
        self.test_chassis.append(cwc.chassis_wheel('front_right', 11.375,  -9.125))
        self.test_chassis.append(cwc.chassis_wheel('mid_left',         0,  10.375))
        self.test_chassis.append(cwc.chassis_wheel('mid_right',        0, -10.375))
        self.test_chassis.append(cwc.chassis_wheel('rear_left',      -10,   9))
        self.test_chassis.append(cwc.chassis_wheel('rear_right',     -10,  -9))

        self.calculator = cwc.chassis_wheel_calculator(self.test_chassis)

    def test_straight_forward(self):
        results = self.calculator.calculate(0, 1.0)

        self.assertEqual(len(results), 6, "Six wheels in should have resulted in six wheels out")
        for wheel in results:
            print(wheel.name, wheel.angle, wheel.velocity)

if __name__ == '__main__':
    unittest.main()