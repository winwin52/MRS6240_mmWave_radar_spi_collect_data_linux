#!/usr/bin/env python3
"""
r3_databox SPI Radar — collect + parse → CSV

Output:
  - radar_data_<timestamp>.csv  : point cloud rows (motion + micro)
  - gain_factor_<timestamp>.csv : per-range-bin gain factors (respiration signal)

Usage:
    sudo python collect_r3_databox.py [--speed 10000000] [--burst 3] [--timeout 0]
    Ctrl+C to stop.
"""

import time, sys, os, csv, signal, struct, argparse
from datetime import datetime

import spidev, lgpio as sbc

# ═══════════════════════════════════════════════════════════════
# HIF Protocol
# ═══════════════════════════════════════════════════════════════
MAGIC = 0xA5

def parse_hif_header(raw6):
    """Parse the 6-byte HIF header."""
    flags = raw6[2] & 0x3F
    return {
        'msg_id': raw6[3],
        'length': raw6[4] | ((raw6[5] & 0x0F) << 8),
        'more':   (flags >> 5) & 1,
        'check':  (flags >> 4) & 1,
    }

def hif_check8_ok(raw6):
    """Verify the HIF header checksum (Check8)."""
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)

def build_poll(poll_type=1, burst_period=3):
    """
    Build a complete HIF POLL frame: header + payload + Check32.
    poll_type=1 (ACK) routes to hif_Msg_ReportEnable() → triggers data.
    poll_type=2 (APP_CUBE) routes to a weak empty handler → DO NOT USE.
    """
    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0  # check8 placeholder
    h[2] = 0x15  # type=1 (Host→Device), flags=5 (REQ|Check32)
    h[3] = 0x0C  # msg_id = HOST_READ
    h[4] = 4     # payload length = 4
    h[5] = 0
    h[1] = (~(h[0] + h[2] + h[3] + h[4] + h[5])) & 0xFF

    pl = bytearray([poll_type, 0, burst_period & 0xFF, (burst_period >> 8) & 0xFF])

    # Check32: one's-complement sum over hdr[2:6] + payload, then NOT
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')
    return bytes(h) + bytes(pl) + ck

# ═══════════════════════════════════════════════════════════════
# PSIC Parser (r3_databox debug protocol)
# ═══════════════════════════════════════════════════════════════

def parse_psic(payload):
    """
    Parse a PSIC debug frame payload.
    Returns (channel_name, dim, raw_bytes).
    dim=2 → 3D points. dim=0 → 1D data (gain_factor).
    """
    if len(payload) < 6:
        return None, None, None
    data_len = payload[2] | (payload[3] << 8)
    dim = ((payload[1] >> 1) & 0x7F) + 1  # stored as dim-1
    name_end = payload.find(b'\x00', 5)
    if name_end < 0:
        return None, None, None
    channel = payload[5:name_end].decode('ascii', errors='replace')
    data_start = name_end + 1
    raw = payload[data_start:data_start + data_len]
    return channel, dim, raw


def unpack_values(raw, channel):
    """
    Unpack raw PSIC data into point list or gain values.
    motion_point_cloud = float32 × 3D
    micro_point_cloud  = int16 × 3D (firmware PSIC header bug: says FLOAT, is S16)
    gain factor        = uint8 values (first 4B = frame_index)
    """
    if channel == 'motion_point_cloud':
        fmt = '<f'; es = 4
    elif channel == 'micro_point_cloud':
        # Firmware PSIC header marks this as FLOAT but data is actually int16
        fmt = '<h'; es = 2
    elif channel == 'gain factor':
        # uint8, first 4 bytes = frame index, rest = gain values per range bin
        return None, list(raw[4:])
    else:
        return None, []

    n = len(raw) // es
    vals = [struct.unpack(fmt, raw[i*es:(i+1)*es])[0] for i in range(n)]
    points = []
    dim = 3
    for i in range(0, len(vals) - dim + 1, dim):
        points.append((vals[i], vals[i+1], vals[i+2]))
    return points, None


# ═══════════════════════════════════════════════════════════════
# SPI / GPIO Hardware Interface
# ═══════════════════════════════════════════════════════════════
SPI_BUS, SPI_DEV, SPI_HZ = 3, 0, 10_000_000
GPIO_CHIP, INT_GPIO = 0, 49

running = True
def _stop(sig, frame):
    global running; running = False; print("\nStopping...")
signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

class Radar:
    def __init__(self, speed=SPI_HZ):
        self.gh = sbc.gpiochip_open(GPIO_CHIP)
        sbc.gpio_claim_input(self.gh, INT_GPIO)
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEV)
        self.spi.mode = 0b00      # CPOL=0, CPHA=0
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = 8
        self.spi.cshigh = False

    def int_level(self):
        return sbc.gpio_read(self.gh, INT_GPIO)

    def wait_int_high(self, ms=1000):
        deadline = time.monotonic() + ms / 1000.
        while time.monotonic() < deadline:
            if sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def wait_int_low(self, ms=1000):
        deadline = time.monotonic() + ms / 1000.
        while time.monotonic() < deadline:
            if not sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def poll(self, burst=3):
        """
        Send POLL, read all HIF frames, return list of {hdr, payload} dicts.

        SPI wire format (DMA padding):
          [magic][check8][msghdr 4B][pad 1B][payload N B][check32 4B][pad 1B]
          ←─── 7 bytes on wire ──→
        """
        self.spi.xfer2(list(build_poll(poll_type=1, burst_period=burst)))
        self.wait_int_low(50)
        self.wait_int_high(200)
        time.sleep(0.002)  # PDF: wait 2ms before reading header

        chunk = bytes(self.spi.xfer2([0x00] * 4096))
        frames = []
        pos = 0
        while pos < len(chunk) and chunk[pos] == 0xA5:
            if pos + 7 > len(chunk):
                break
            raw6 = chunk[pos:pos+6]  # logical header (6 bytes)
            hdr = parse_hif_header(raw6)
            if not hif_check8_ok(raw6):
                break
            N = hdr['length']
            if N > 2000 or N == 0:
                break
            payload = chunk[pos+7:pos+7+N]  # skip 1-byte DMA pad
            wire_len = 7 + N + (5 if hdr['check'] else 0)
            frames.append({'hdr': hdr, 'payload': payload})
            if hdr['more'] == 0:
                break
            pos += wire_len
        return frames

    def close(self):
        self.spi.close()


# ═══════════════════════════════════════════════════════════════
# CSV Writer
# ═══════════════════════════════════════════════════════════════

CSV_HEADER = ["timestamp", "data_type", "frame/seq", "count/length",
              "x", "y", "z", "snr", "velocity"]

GAIN_HEADER = ["timestamp", "frame_idx"] + [f"gain_{i}" for i in range(520)]


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description='r3_databox SPI Radar Collector')
    ap.add_argument('--speed', type=int, default=SPI_HZ,
                    help=f'SPI clock Hz (default {SPI_HZ})')
    ap.add_argument('--burst', type=int, default=3,
                    help='POLL burst_period')
    ap.add_argument('--timeout', type=int, default=0,
                    help='Auto-stop after N seconds (0=forever)')
    ap.add_argument('--dir', type=str, default=None,
                    help='Output directory (default: cwd)')
    args = ap.parse_args()

    out_dir = args.dir or os.getcwd()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"radar_data_{ts}.csv")
    gain_path = os.path.join(out_dir, f"gain_factor_{ts}.csv")

    print(f"r3_databox SPI → {csv_path}")
    print(f"  {args.speed/1e6:.1f} MHz  burst={args.burst}  Ctrl+C to stop\n")

    radar = Radar(speed=args.speed)

    # Open CSV files
    csv_f = open(csv_path, 'w', newline='', encoding='utf-8-sig')
    csv_w = csv.writer(csv_f)
    csv_w.writerow(CSV_HEADER)

    gain_f = open(gain_path, 'w', newline='', encoding='utf-8-sig')
    gain_w = csv.writer(gain_f)
    gain_w.writerow(GAIN_HEADER)

    total_rows = 0
    gain_rows = 0
    rounds = 0
    deadline = time.monotonic() + args.timeout if args.timeout > 0 else float('inf')

    try:
        while running and time.monotonic() < deadline:
            rounds += 1

            if not radar.wait_int_high(ms=5000 if rounds == 1 else 2000):
                print(f"[r{rounds}] INT timeout")
                continue

            frames = radar.poll(burst=args.burst)

            for f in frames:
                hdr = f['hdr']
                if hdr['msg_id'] != 0xC6:   # PSIC debug data only
                    continue

                channel, dim, raw = parse_psic(f['payload'])
                if channel is None:
                    continue

                t = datetime.now().strftime('%Y-%m-%d-%H:%M:%S.') + \
                    f"{int(time.time() % 1 * 1000):03d}"

                if channel == 'gain factor':
                    _, gains = unpack_values(raw, channel)
                    frame_idx = struct.unpack('<I', raw[:4])[0]
                    gain_w.writerow([t, frame_idx] + gains)
                    gain_rows += 1
                    continue

                points, _ = unpack_values(raw, channel)
                if points is None:
                    continue

                n_points = len(points)
                for i, (x, y, z) in enumerate(points):
                    csv_w.writerow([t, channel, i, n_points,
                                    round(x, 2), round(y, 2), round(z, 2), '', ''])
                    total_rows += 1

            if rounds % 20 == 0 or (frames and not frames[-1]['hdr']['more']):
                csv_f.flush()
                gain_f.flush()

            if not args.timeout and rounds == 1:
                print(f"[r{rounds}] Receiving data. Ctrl+C to stop.")

    finally:
        radar.close()
        csv_f.close()
        gain_f.close()

    print(f"\nDone. {rounds} rounds, {total_rows} point rows, {gain_rows} gain rows")
    print(f"  {csv_path}")
    print(f"  {gain_path}")


if __name__ == '__main__':
    main()
