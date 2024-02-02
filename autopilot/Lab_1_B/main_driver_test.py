# Assuming calculate_turn_and_distance is in a file named vehicle_movement.py
from main_driver import calculate_turn_and_distance
import unittest
import math

class TestVehicleMovement(unittest.TestCase):
    def test_calculate_turn_and_distance(self):
        # Test cases as tuples: (x, y, expected_angle, expected_distance)
        test_cases = [
            (10, 10, 45, math.sqrt(10**2 + 10**2) * 20),  # Diagonal movement in Quadrant I
            (-10, 10, 135, math.sqrt(10**2 + 10**2) * 20),  # Quadrant II
            (-10, -10, -135, math.sqrt(10**2 + 10**2) * 20),  # Quadrant III
            (10, -10, -45, math.sqrt(10**2 + 10**2) * 20),  # Quadrant IV
            (0, 10, 90, 10 * 20),  # Straight up
            (10, 0, 0, 10 * 20),  # Straight right
            (0, -10, -90, 10 * 20),  # Straight down
            (-10, 0, 180, 10 * 20),  # Straight left
        ]

        for x, y, expected_angle, expected_distance in test_cases:
            with self.subTest(x=x, y=y):
                angle, distance = calculate_turn_and_distance(x, y)
                self.assertAlmostEqual(angle, expected_angle, places=1,
                                       msg=f"Failed angle for x={x}, y={y}")
                self.assertAlmostEqual(distance, expected_distance, places=1,
                                       msg=f"Failed distance for x={x}, y={y}")

if __name__ == "__main__":
    unittest.main()