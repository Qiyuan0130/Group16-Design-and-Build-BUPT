import math
def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
