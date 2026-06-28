import os
import time, math
from uart_reader import MmwaveReader

reader = MmwaveReader()
errors = reader.send_config(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'configs', 'profile_AOP.cfg'))
if errors:
    for line, resp in errors: print(f"ERROR: {line}: {resp}")
    raise SystemExit
reader.start()

print("Walk slowly left-to-right. Live angles:")
try:
    while True:
        frame = reader.get_frame(timeout=1.0)
        if frame is None: continue
        # Only print moving points to filter clutter
        moving = [p for p in frame.points if abs(p.velocity) > 0.3]
        if not moving: continue
        for p in moving:
            az = math.degrees(math.atan2(p.x, p.y))
            r = math.hypot(p.x, p.y)
            # ASCII gauge from -90° (left) to +90° (right)
            pos = int((az + 90) / 180 * 60)
            bar = ' ' * pos + '●' + ' ' * (60 - pos)
            print(f"  {az:+5.1f}°  r={r:.2f}m  v={p.velocity:+.2f}  |{bar}|")
except KeyboardInterrupt:
    reader.stop()
