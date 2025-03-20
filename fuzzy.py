import unittest

def f_and(v1: float, v2: float) -> float:
    return min(v1, v2)


def f_or(v1: float, v2: float) -> float:
    return max(v1, v2)


def f_not(value: float) -> float:
    return 1.0 - value


def fuzzify(value: float, start: float, end: float) -> float:
    if value > end:
        return 1.0
    elif value < start:
        return 0.0
    else:
        return (value - start) / (end - start)
    

def triangle(value: float, start: float, peak: float, end: float) -> float:
    if value <= peak:
        return fuzzify(value, start, peak)
    else:
        return f_not(fuzzify(value, peak, end))


def defuzzify(value: float, zero: float, one: float) -> float:
    if zero > one:
        return defuzzify(f_not(value), one, zero)
    else:
        return zero + value * (one - zero)
    

class FuzzyTest(unittest.TestCase):
    def test_and_or_not(self):
        self.assertEqual(0.75, f_and(1.0, 0.75))
        self.assertEqual(1.0, f_or(1.0, 0.75))
        self.assertEqual(0.0, f_not(1.0))
        self.assertEqual(1.0, f_not(0.0))

    def test_fuzzify(self):
        for expected, height in [(1.0, 76), (0.75, 74), (0.5, 72), (0.25, 70), (0.0, 68), (1.0, 80), (0.0, 62)]:
            self.assertEqual(expected, fuzzify(height, 68, 76))

    def test_defuzzify(self):
        for inseam_size, fuzzy_height in [(36, 1.0), (34.5, 0.75), (33, 0.5), (31.5, 0.25), (30, 0.0)]:
            self.assertEqual(inseam_size, defuzzify(fuzzy_height, 30, 36))

    def test_triangle(self):
        for expected, value in [(0, 0), (0.5, 0.5), (1, 1), (0.5, 1.5), (0, 2)]:
            self.assertEqual(expected, triangle(value, 0.0, 1.0, 2.0))


if __name__ == "__main__":
    unittest.main()