import struct
import time

STOP_ZONE_M    = 0.8
CAUTION_ZONE_M = 2.5

def closest_distance(points):
    if not points:
        return float('inf')
    return min((x**2 + y**2 + z**2)**0.5 for x, y, z, v in points)

def zone_decision(distance_m):
    if distance_m < STOP_ZONE_M:
        return 'STOP'
    if distance_m < CAUTION_ZONE_M:
        return 'CAUTION'
    return 'CLEAR'

def run_mock():
    print("DNTD Dynamics — mmWave zone pipeline")
    print("Mock data: simulating object approach\n")
    scenarios = [
        ("5.0m — open space",  [(5.0,  0.1, 0.0, -0.3)]),
        ("3.0m — approaching", [(3.0,  0.1, 0.0, -0.5)]),
        ("2.0m — caution",     [(2.0,  0.1, 0.0, -0.7)]),
        ("1.0m — close",       [(1.0,  0.0, 0.0, -0.9)]),
        ("0.5m — STOP",        [(0.5,  0.0, 0.0, -0.3)]),
        ("no object",          []),
    ]
    for label, points in scenarios:
        d = closest_distance(points)
        decision = zone_decision(d)
        marker = " <--" if decision != "CLEAR" else ""
        print(f"  {label:22s}  {d:5.2f}m  [{decision}]{marker}")
        time.sleep(0.4)
    print("\nPipeline OK — ready for live EVM data")

if __name__ == "__main__":
    run_mock()
