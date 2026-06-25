from uart_reader import MmwaveReader
import time

reader = MmwaveReader()
errors = reader.send_config('/home/nic/mmwave/configs/profile_AOP.cfg')
if errors:
    for line, resp in errors: print(f"ERROR: {line}: {resp}")
    raise SystemExit
reader.start()
print("Dumping raw points (nothing should appear if scene is empty)...")
try:
    while True:
        frame = reader.get_frame(timeout=1.0)
        if frame is None: continue
        if not frame.points: continue
        print(f"--- Frame {frame.frame_number}: {len(frame.points)} pts ---")
        for p in frame.points:
            r = (p.x**2 + p.y**2 + p.z**2) ** 0.5
            print(f"  x={p.x:+.2f} y={p.y:+.2f} z={p.z:+.2f} v={p.velocity:+.2f} snr={p.snr:.1f} range={r:.2f}")
except KeyboardInterrupt:
    reader.stop()
