import struct
from dataclasses import dataclass

MAGIC = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40

# TLV type constants (mmw demo 3.x)
TLV_DETECTED_POINTS = 1      # x, y, z, velocity per point
TLV_RANGE_PROFILE   = 2
TLV_NOISE_PROFILE   = 3
TLV_AZIMUTH_HEATMAP = 4
TLV_RANGE_DOPPLER   = 5
TLV_STATS           = 6
TLV_SIDE_INFO       = 7      # SNR, noise per point
TLV_TEMPERATURE     = 9

@dataclass
class Point:
    x: float; y: float; z: float; velocity: float
    snr: float = 0.0; noise: float = 0.0

@dataclass
class Frame:
    frame_number: int
    num_objects: int
    points: list

def parse_header(buf, offset):
    """Returns (frame_number, total_packet_len, num_obj, num_tlvs)."""
    h = struct.unpack_from('<8sIIIIIII', buf, offset)
    # h = (magic, version, totalPacketLen, platform,
    #      frameNum, timeCpuCycles, numDetectedObj, numTLVs)
    return h[4], h[2], h[6], h[7]

def parse_detected_points(buf, offset, length, num_points):
    """Each point: 4 floats = 16 bytes (x, y, z, velocity)."""
    points = []
    for i in range(num_points):
        x, y, z, v = struct.unpack_from('<ffff', buf, offset + i * 16)
        points.append(Point(x, y, z, v))
    return points

def parse_side_info(buf, offset, num_points, points):
    """Each point: 2 uint16 = 4 bytes (snr, noise) — units of 0.1 dB."""
    for i in range(num_points):
        snr, noise = struct.unpack_from('<HH', buf, offset + i * 4)
        points[i].snr = snr * 0.1
        points[i].noise = noise * 0.1

def parse_frames(buf):
    """Yield Frame objects from a byte buffer. Skips partial trailing frames."""
    pos = 0
    while True:
        idx = buf.find(MAGIC, pos)
        if idx < 0:
            break
        if idx + HEADER_LEN > len(buf):
            break

        frame_num, total_len, num_obj, num_tlvs = parse_header(buf, idx)

        if idx + total_len > len(buf):
            break  # incomplete frame at tail

        frame = Frame(frame_number=frame_num, num_objects=num_obj, points=[])
        tlv_offset = idx + HEADER_LEN

        for _ in range(num_tlvs):
            if tlv_offset + 8 > idx + total_len:
                break
            tlv_type, tlv_len = struct.unpack_from('<II', buf, tlv_offset)
            payload = tlv_offset + 8

            if tlv_type == TLV_DETECTED_POINTS:
                frame.points = parse_detected_points(buf, payload, tlv_len, num_obj)
            elif tlv_type == TLV_SIDE_INFO and frame.points:
                parse_side_info(buf, payload, num_obj, frame.points)

            tlv_offset = payload + tlv_len

        yield frame
        pos = idx + total_len

if __name__ == '__main__':
    with open('/tmp/capture.bin', 'rb') as f:
        data = f.read()
    for frame in parse_frames(data):
        print(f"Frame {frame.frame_number}: {frame.num_objects} points")
        for p in frame.points[:3]:
            print(f"  ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f}) "
                  f"v={p.velocity:+.2f} m/s  SNR={p.snr:.1f} dB")
