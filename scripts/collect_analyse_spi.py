#!/usr/bin/env python3
"""
radar_analysis_spi — K1 SPI collector + TLV parser → CSV

Firmware: radar_analysis_spi (auto-starts SPI point cloud, no UART shell needed)
Output: msg_id=0xC3, MMW_FRAME_UPLOAD + TL_HEADER + PointCloud3D

Output CSV:
  - analyse_frames_<ts>.csv  : frame summary (frame_idx, n_motion, n_micro, centroid)
  - analyse_points_<ts>.csv  : per-point detail (x, y, z, velocity, snr)

Usage:
    sudo python collect_analyse_spi.py [--speed 8000000] [--burst 3] [--duration 0]
    Ctrl+C to stop.
"""

import time, sys, os, csv, signal, struct, argparse
from datetime import datetime
from collections import deque

import spidev, lgpio as sbc

# ═══════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════
MAGIC = 0xA5
MSG_ID_POINTCLOUD = 0xC3
MMW_TL_TYPE_POINTS = 4
MMW_TL_FLAG_MICRO = 0x02

# Wire format (SPI DMA padding):
#   HIF header: 6 bytes + 1 pad = 7 bytes on wire
#   Check32:    4 bytes + 1 pad = 5 bytes on wire
HDR_WIRE = 7
CHK_WIRE = 5

POINT_STRUCT_SIZE = 12  # PointCloud_Cart / PointCloud_Polar (int16 × 6)

SPI_BUS, SPI_DEV, SPI_HZ = 3, 0, 8_000_000
GPIO_CHIP, INT_GPIO = 0, 49
BURST_PERIOD = 3

# ═══════════════════════════════════════════════════════════════
# HIF Protocol
# ═══════════════════════════════════════════════════════════════

def parse_hif_header(raw6):
    flags = raw6[2] & 0x3F
    return {
        'msg_id': raw6[3],
        'length': raw6[4] | ((raw6[5] & 0x0F) << 8),
        'more':   (flags >> 5) & 1,
        'check':  (flags >> 4) & 1,
    }

def hif_check8_ok(raw6):
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)

def build_poll(poll_type=1, burst_period=BURST_PERIOD):
    """Build HIF POLL frame (poll_type=1 = ACK, triggers data reporting)."""
    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0
    h[2] = 0x15  # type=1 (Host→Device), flags=5 (REQ|Check32)
    h[3] = 0x0C  # msg_id = HOST_READ
    h[4] = 4     # payload length
    h[5] = 0
    h[1] = (~(h[0] + h[2] + h[3] + h[4] + h[5])) & 0xFF

    pl = bytearray([poll_type, 0, burst_period & 0xFF, (burst_period >> 8) & 0xFF])
    # check32 over hdr[2:6] + payload
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')
    return bytes(h) + bytes(pl) + ck

# ═══════════════════════════════════════════════════════════════
# TLV Frame Parser (radar_analysis_spi: MMW_FRAME_UPLOAD)
# ═══════════════════════════════════════════════════════════════

class TlvFrameParser:
    """Accumulates HIF fragments and reassembles TLV point cloud frames."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.frame_idx = None
        self.total_words = 0
        self.words_parsed = 0
        self.motion_points = []
        self.micro_points = []
        self._expect_tl = True
        self._tlv_remaining = 0
        self._pending = b''

    def feed_fragment(self, payload):
        """
        Feed one HIF payload (after HIF header, before check32).
        payload = MMW_FRAME_UPLOAD (12B) + [TL_HEADER (4B)] + point data.

        Returns True if a complete frame was assembled.
        """
        if len(payload) < 12:
            return False

        frame_idx, total_words, offset_words = struct.unpack('<III', payload[:12])
        data = payload[12:]

        self.frame_idx = frame_idx
        self.total_words = total_words

        # offset_words = how many words the firmware sent before this fragment.
        # Should match our words_parsed if everything aligns.
        if offset_words * 4 != self.words_parsed * 4:
            # Lost sync — reset and start fresh
            self.reset()
            self.frame_idx = frame_idx
            self.total_words = total_words

        while data:
            if self._expect_tl:
                if len(data) < 4:
                    break
                tl_type, tl_flag, tl_len = struct.unpack('<BBH', data[:4])
                data = data[4:]
                self.words_parsed += 1
                self._expect_tl = False
                self._tlv_remaining = tl_len
                self._tlv_flag = tl_flag
            else:
                # Read point data
                take = min(len(data), self._tlv_remaining)
                chunk = data[:take]
                data = data[take:]
                self._pending += chunk
                self._tlv_remaining -= take

                # Parse complete points from pending buffer
                while len(self._pending) >= POINT_STRUCT_SIZE and \
                      (self._tlv_remaining > 0 or
                       (self._tlv_remaining == 0 and len(self._pending) >= POINT_STRUCT_SIZE)):
                    point = struct.unpack('<hhhhhh', self._pending[:POINT_STRUCT_SIZE])
                    self._pending = self._pending[POINT_STRUCT_SIZE:]
                    self.words_parsed += 3

                    pt = {
                        'x': point[0], 'y': point[1], 'z': point[2],
                        'vel': point[3], 'snr': point[4],
                    }
                    if self._tlv_flag & MMW_TL_FLAG_MICRO:
                        self.micro_points.append(pt)
                    else:
                        self.motion_points.append(pt)

                if self._tlv_remaining == 0:
                    self._expect_tl = True

        return self.words_parsed >= self.total_words and self.total_words > 0

    def dump(self):
        """Return assembled frame data and reset."""
        result = {
            'frame_idx': self.frame_idx,
            'motion_points': list(self.motion_points),
            'micro_points': list(self.micro_points),
        }
        self.reset()
        return result


# ═══════════════════════════════════════════════════════════════
# SPI Radar Interface
# ═══════════════════════════════════════════════════════════════

running = True

def _stop(sig, frame):
    global running
    running = False
    print("\nStopping...")

signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


class RadarAnalyse:
    def __init__(self, speed=SPI_HZ):
        self.gh = sbc.gpiochip_open(GPIO_CHIP)
        sbc.gpio_claim_input(self.gh, INT_GPIO)
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEV)
        self.spi.mode = 0b00
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = 8
        self.spi.cshigh = False
        self.parser = TlvFrameParser()

    def int_level(self):
        return sbc.gpio_read(self.gh, INT_GPIO)

    def wait_int_high(self, ms=1000):
        d = time.monotonic() + ms / 1000.
        while time.monotonic() < d:
            if sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def wait_int_low(self, ms=1000):
        d = time.monotonic() + ms / 1000.
        while time.monotonic() < d:
            if not sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def poll(self, burst=BURST_PERIOD):
        """
        Send POLL, read all HIF frames, return list of completed frames.
        Each completed frame = {'frame_idx', 'motion_points', 'micro_points'}.
        """
        self.spi.xfer2(list(build_poll(poll_type=1, burst_period=burst)))
        self.wait_int_low(50)
        self.wait_int_high(200)
        time.sleep(0.002)

        chunk = bytes(self.spi.xfer2([0x00] * 4096))
        frames = []
        pos = 0

        while pos < len(chunk) and chunk[pos] == MAGIC:
            if pos + HDR_WIRE > len(chunk):
                break
            raw6 = chunk[pos:pos + 6]
            hdr = parse_hif_header(raw6)
            if not hif_check8_ok(raw6):
                break
            N = hdr['length']
            if N > 4000 or N == 0:
                break

            payload = chunk[pos + HDR_WIRE:pos + HDR_WIRE + N]
            wire_len = HDR_WIRE + N + (CHK_WIRE if hdr['check'] else 0)

            if hdr['msg_id'] == MSG_ID_POINTCLOUD:
                done = self.parser.feed_fragment(payload)
                if done:
                    frames.append(self.parser.dump())

            if hdr['more'] == 0:
                break
            pos += wire_len

        return frames

    def close(self):
        self.spi.close()


# ═══════════════════════════════════════════════════════════════
# CSV Writer
# ═══════════════════════════════════════════════════════════════

FRAMES_HEADER = ["timestamp", "frame_idx", "n_motion", "n_micro",
                 "centroid_x", "centroid_y", "centroid_z"]
POINTS_HEADER = ["timestamp", "frame_idx", "type", "x_cm", "y_cm", "z_cm",
                 "vel_cm_s", "snr_db100"]


def compute_centroid(points):
    if not points:
        return (0, 0, 0)
    n = len(points)
    sx = sum(p['x'] for p in points) / n
    sy = sum(p['y'] for p in points) / n
    sz = sum(p['z'] for p in points) / n
    return (round(sx, 1), round(sy, 1), round(sz, 1))


def timestamp_now():
    t = time.time()
    dt = datetime.fromtimestamp(t)
    return dt.strftime('%Y-%m-%d-%H:%M:%S.') + f"{int(t % 1 * 1000):03d}"


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description='radar_analysis_spi SPI Collector')
    ap.add_argument('--speed', type=int, default=SPI_HZ,
                    help=f'SPI clock Hz (default {SPI_HZ})')
    ap.add_argument('--burst', type=int, default=BURST_PERIOD,
                    help=f'POLL burst_period (default {BURST_PERIOD})')
    ap.add_argument('--duration', type=int, default=0,
                    help='Auto-stop after N seconds (0=forever)')
    ap.add_argument('--output', type=str, default=None,
                    help='Output directory (default: cwd)')
    args = ap.parse_args()

    out_dir = args.output or os.getcwd()
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    frames_path = os.path.join(out_dir, f"analyse_frames_{ts}.csv")
    points_path = os.path.join(out_dir, f"analyse_points_{ts}.csv")

    print(f"radar_analysis_spi → SPI {args.speed/1e6:.1f} MHz")
    print(f"  Frames: {frames_path}")
    print(f"  Points: {points_path}")
    print(f"  Ctrl+C to stop\n")

    radar = RadarAnalyse(speed=args.speed)

    # Open CSV files
    ff = open(frames_path, 'w', newline='', encoding='utf-8-sig')
    fw = csv.writer(ff)
    fw.writerow(FRAMES_HEADER)

    pf = open(points_path, 'w', newline='', encoding='utf-8-sig')
    pw = csv.writer(pf)
    pw.writerow(POINTS_HEADER)

    total_frames = 0
    total_points = 0
    rounds = 0
    deadline = time.monotonic() + args.duration if args.duration > 0 else float('inf')

    try:
        while running and time.monotonic() < deadline:
            rounds += 1

            if not radar.wait_int_high(ms=5000 if rounds == 1 else 2000):
                print(f"[r{rounds}] INT timeout, waiting...")
                continue

            frames = radar.poll(burst=args.burst)

            for frame in frames:
                t = timestamp_now()
                fi = frame['frame_idx']
                motion = frame['motion_points']
                micro = frame['micro_points']
                cx, cy, cz = compute_centroid(motion) if motion else (0, 0, 0)

                # Write frame summary
                fw.writerow([t, fi, len(motion), len(micro), cx, cy, cz])

                # Write motion points
                for pt in motion:
                    pw.writerow([t, fi, 'M', pt['x'], pt['y'], pt['z'],
                                 pt['vel'], pt['snr']])
                    total_points += 1

                # Write micro points
                for pt in micro:
                    pw.writerow([t, fi, 'U', pt['x'], pt['y'], pt['z'],
                                 pt['vel'], pt['snr']])
                    total_points += 1

                total_frames += 1

            # Periodic flush & progress
            if rounds % 30 == 0:
                ff.flush()
                pf.flush()
                elapsed = time.monotonic() - (deadline - args.duration) \
                    if args.duration > 0 else 0
                fps = total_frames / elapsed if elapsed > 0 else 0
                print(f"  [{rounds}] {total_frames} frames, {total_points} pts, "
                      f"~{fps:.1f} fps")

            if not args.duration and rounds == 1:
                print(f"  Receiving data. Ctrl+C to stop.")

    finally:
        radar.close()
        ff.close()
        pf.close()

    elapsed = time.monotonic() - (deadline - args.duration) \
        if args.duration > 0 else 0
    print(f"\nDone. {total_frames} frames, {total_points} points "
          f"in {elapsed:.1f}s")
    print(f"  {frames_path}")
    print(f"  {points_path}")


if __name__ == '__main__':
    main()
