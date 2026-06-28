# uart_reader.py -- DNTD Dynamics RadarGuard
import serial
import threading
import queue
import time
from tlv_parser import parse_frames, MAGIC

class MmwaveReader:
    def __init__(self, cli_port='/dev/ttyUSB0', data_port='/dev/ttyUSB1',
                 cli_baud=115200, data_baud=921600, queue_size=100):
        self.cli_port = cli_port
        self.data_port = data_port
        self.cli_baud = cli_baud
        self.data_baud = data_baud
        self.frame_queue = queue.Queue(maxsize=queue_size)
        self._running = False
        self._thread = None
        self._buffer = bytearray()

    def send_config(self, cfg_path):
        """Send config file line-by-line over CLI port. Returns list of errors."""
        errors = []
        with serial.Serial(self.cli_port, self.cli_baud, timeout=1) as cli:
            with open(cfg_path) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('%'):
                        continue
                    cli.write((line + '\n').encode())
                    time.sleep(2.0 if line.startswith('sensorStart') else 0.05)
                    resp = cli.read_all().decode(errors='replace')
                    if 'Error' in resp and '0x1ffe' not in resp:
                        errors.append((line, resp.strip()))
        return errors

    def stop_sensor(self):
        with serial.Serial(self.cli_port, self.cli_baud, timeout=1) as cli:
            cli.write(b'sensorStop\n')
            time.sleep(0.2)
            cli.read_all()

    def start(self):
        """Begin background read thread. Call after send_config()."""
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.stop_sensor()

    def _read_loop(self):
        with serial.Serial(self.data_port, self.data_baud, timeout=0.1) as ser:
            while self._running:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                self._buffer.extend(chunk)
                # Parse all complete frames found in buffer
                for frame in parse_frames(bytes(self._buffer)):
                    try:
                        self.frame_queue.put_nowait(frame)
                    except queue.Full:
                        # Drop oldest if downstream is slow
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put_nowait(frame)
                        except queue.Empty:
                            pass
                # Trim buffer — keep only data after last complete frame
                last_magic = self._buffer.rfind(MAGIC)
                if last_magic > 0:
                    self._buffer = self._buffer[last_magic:]

    def get_frame(self, timeout=1.0):
        """Blocking get, returns Frame or None on timeout."""
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None


if __name__ == '__main__':
    reader = MmwaveReader()
    print("Sending config...")
    errors = reader.send_config(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'configs', 'profile_AOP.cfg'))
    if errors:
        for line, resp in errors:
            print(f"ERROR on '{line}': {resp}")
        raise SystemExit(1)
    print("Config sent. Starting reader thread...")
    reader.start()
    try:
        while True:
            frame = reader.get_frame(timeout=1.0)
            if frame is None:
                continue
            moving = [p for p in frame.points if abs(p.velocity) > 0.3]
            if moving:
                print(f"Frame {frame.frame_number}: {len(moving)} moving points")
                for p in moving[:3]:
                    print(f"  ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f}) "
                          f"v={p.velocity:+.2f} m/s")
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()
