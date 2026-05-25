#!/usr/bin/env python3
"""
r3_databox HIF Protocol Library

Based on firmware hif_msg.h + hif_com.c + PDF V1.1 cross-reference.
All byte ordering = little-endian (matches Cortex-M firmware).

Provides:
  - HIF frame building (build_host_read, build_command, etc.)
  - HIF frame parsing (parse_hif_header, hif_check8_ok, verify_check32)
  - PSIC payload parsing (parse_psic, unpack_values)
  - Complete Ack constant

See docs/hif_protocol.md for protocol details and timing diagrams.
"""

import struct

# ── HIF Constants ──────────────────────────────────────────────

MAGIC          = 0xA5
HDR_LEN        = 6         # Magic(1) + Check8(1) + MsgHdr(4)
CHECK32_LEN    = 4

# SPI DMA alignment padding (firmware hif_com.c:293-294)
# hdr_len  = HIF_HEAD_LEN + 1  → 7 bytes on wire
# check_len = HIF_CHKEC_LEN + 1 → 5 bytes on wire
SPI_HDR_PAD    = 1
SPI_CHK_PAD    = 1

# Wire offsets for SPI DMA mode
WIRE_HDR_LEN   = 7   # HDR_LEN + SPI_HDR_PAD
WIRE_CHK_LEN   = 5   # CHECK32_LEN + SPI_CHK_PAD

# Empirically measured Check32 offset on r3_databox firmware.
# The firmware's TX-side checksum consistently differs from the
# one's-complement sum by +5 (reason TBD, possibly a counter or
# init-value difference in checksum32_calc vs HIF_CheckSum32).
CHECK32_FW_OFFSET = 5

# MsgHdr bit offsets in byte2
HIF_TYPE_MASK       = 0x03
HIF_FLAG_MASK       = 0x3F
HIF_TYPE_HOST2DEV   = 1
HIF_TYPE_DEV2HOST   = 2
HIF_TYPE_DEBUG      = 3

# Flag bits in byte2
HIF_FLAG_ACK        = (1 << 0)   # Request bit
HIF_FLAG_ENC        = (1 << 1)   # Encryption
HIF_FLAG_CHECK32    = (1 << 2)   # Has Check32
HIF_FLAG_MORE       = (1 << 3)   # More data flag
HIF_FLAG_EXT        = (1 << 4)   # Extension
HIF_FLAG_MAC32      = (1 << 5)   # MAC32

# Message IDs
MSG_ID_VERSION          = 0x00
MSG_ID_WAKEUP           = 0x05
MSG_ID_START_CTRL       = 0x07
MSG_ID_READ_VERSION     = 0x0A
MSG_ID_HOST_READ        = 0x0C   # POLL
MSG_ID_CONFIG_GET       = 0x40
MSG_ID_CONFIG_SET       = 0x41
MSG_ID_START_CTRL_R3    = 0x60   # r3_databox start/stop
MSG_ID_MOTION_CFG       = 0x61   # r3_databox param config
MSG_ID_MOTION_INFO_GET  = 0x62
MSG_ID_FFT_DATA         = 0xC1   # FFT DataCube
MSG_ID_POINT_CLOUD      = 0xC3   # MMW_FRAME_UPLOAD
MSG_ID_PSIC_DEBUG       = 0xC6   # r3_databox PSIC debug data
MSG_ID_DBG_PRINT        = 0xF0   # Debug printf
MSG_ID_STARTUP          = 0xFF   # Device boot notification

# Complete Ack: ends a data burst, signals device to sleep
COMPLETE_ACK = bytes([0xA5, 0x4B, 0x03, 0x0C, 0x00, 0x00])

# PSIC channel names
PSIC_CH_GAIN_FACTOR     = 'gain factor'
PSIC_CH_MOTION_CLOUD    = 'motion_point_cloud'
PSIC_CH_MICRO_CLOUD     = 'micro_point_cloud'

# TLV constants (radar_analysis_spi)
MMW_TL_TYPE_POINTS      = 4
MMW_TL_FLAG_MICRO       = 0x02
POINT_STRUCT_SIZE       = 12  # int16 × 6


# ── Check8 ─────────────────────────────────────────────────────

def hif_check8_calc(header_bytes):
    """
    Calculate Check8 for a 6-byte HIF header.
    header_bytes = [magic, check8_placeholder, type_flags, msg_id, len_lo, len_hi]
    """
    return (~(header_bytes[0] + header_bytes[2] + header_bytes[3] +
              header_bytes[4] + header_bytes[5])) & 0xFF


def hif_check8_ok(raw6):
    """Verify Check8 on a received 6-byte header."""
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)


# ── Check32 ────────────────────────────────────────────────────

def checksum32_one_complement(data_bytes, init=0):
    """
    One's-complement sum over byte array, dword-aligned.
    Matches the SDK HIF_CheckSum32 algorithm.
    """
    total = init
    n = len(data_bytes)
    i = 0
    while i + 4 <= n:
        w = int.from_bytes(data_bytes[i:i+4], 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
        i += 4
    rem = n - i
    if rem > 0:
        w = int.from_bytes(data_bytes[i:].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
    return total & 0xFFFFFFFF


def hif_check32_calc(hdr_word, payload_bytes):
    """
    Calculate HIF Check32.
    hdr_word = the 4-byte MsgHdr (header bytes [2:6]).
    """
    data = hdr_word + payload_bytes
    return (~checksum32_one_complement(data)) & 0xFFFFFFFF


# ── Frame Building ─────────────────────────────────────────────

def build_hif_frame(msg_id, payload=b'', hif_type=HIF_TYPE_HOST2DEV,
                    seq=0, flags=0, more=0):
    """
    Build a complete HIF frame with Check8 and Check32.
    Returns bytes ready to send over SPI (without DMA padding — add
    SPI_HDR_PAD and SPI_CHK_PAD for wire format).
    """
    flags_byte = hif_type & HIF_TYPE_MASK
    if flags & HIF_FLAG_ACK:
        flags_byte |= HIF_FLAG_ACK
    flags_byte |= HIF_FLAG_CHECK32   # always include Check32
    if more:
        flags_byte |= HIF_FLAG_MORE

    length = len(payload)
    length_lo = length & 0xFF
    length_hi = ((length >> 8) & 0x0F) | ((seq & 0x07) << 4)

    hdr = bytearray(6)
    hdr[0] = MAGIC
    hdr[1] = 0  # placeholder
    hdr[2] = flags_byte
    hdr[3] = msg_id
    hdr[4] = length_lo
    hdr[5] = length_hi
    hdr[1] = hif_check8_calc(hdr)

    # Check32 over MsgHdr (4B) + payload
    ck32 = hif_check32_calc(bytes(hdr[2:6]), payload)
    ck32_bytes = struct.pack('<I', ck32)

    return bytes(hdr) + payload + ck32_bytes


def build_host_read(poll_type=1, burst_period=3):
    """
    Build a HOST_READ (POLL) command frame.
    poll_type=1 (ACK) → routes to hif_Msg_ReportEnable().
    poll_type=2 (APP_CUBE) → routes to weak empty handler. DO NOT USE.
    """
    payload = struct.pack('<BBH', poll_type, 0, burst_period)
    return build_hif_frame(MSG_ID_HOST_READ, payload)


def build_start_ctrl(enable=True):
    """Build START_CTRL (0x07) command. enable=True starts, False stops."""
    payload = struct.pack('<I', 1 if enable else 0)
    return build_hif_frame(MSG_ID_START_CTRL, payload)


# ── Frame Parsing ──────────────────────────────────────────────

def parse_hif_header(raw6):
    """Parse the 6-byte HIF header into a dict."""
    flags = raw6[2] & 0x3F
    return {
        'magic':   raw6[0],
        'check8':  raw6[1],
        'type':    raw6[2] & HIF_TYPE_MASK,
        'flags':   flags,
        'msg_id':  raw6[3],
        'length':  raw6[4] | ((raw6[5] & 0x0F) << 8),
        'seq':     (raw6[5] >> 4) & 0x07,
        'frag':    (raw6[5] >> 7) & 0x01,
        'more':    (flags >> 3) & 1,   # bit 3
        'check':   (flags >> 2) & 1,   # bit 2
    }


def parse_one_frame_from_wire(wire_bytes, offset=0):
    """
    Parse one HIF frame from SPI wire format (with DMA padding).
    Returns (frame_dict, next_offset) or (None, offset) if invalid.

    Wire format:
      [magic][check8][msghdr 4B][pad 1B][payload N B][check32 4B][pad 1B]
    """
    pos = offset
    if pos + WIRE_HDR_LEN > len(wire_bytes):
        return None, offset
    if wire_bytes[pos] != MAGIC:
        return None, offset

    raw6 = wire_bytes[pos:pos+6]
    hdr = parse_hif_header(raw6)
    if not hif_check8_ok(raw6):
        return None, offset

    N = hdr['length']
    payload_start = pos + WIRE_HDR_LEN
    if payload_start + N > len(wire_bytes):
        return None, offset

    payload = wire_bytes[payload_start:payload_start + N]
    wire_len = WIRE_HDR_LEN + N + (WIRE_CHK_LEN if hdr['check'] else 0)

    return {'hdr': hdr, 'payload': bytes(payload)}, pos + wire_len


# ── PSIC Parser (r3_databox debug protocol) ────────────────────

def parse_psic(payload):
    """
    Parse a PSIC debug frame payload.
    Returns (channel_name, dim, raw_bytes).
    dim: actual dimension count (2 → 3D points, 0 → 1D data).
    """
    if len(payload) < 6:
        return None, None, None
    data_len = payload[2] | (payload[3] << 8)
    dim = ((payload[1] >> 1) & 0x7F) + 1  # stored as dim-1, +1 = actual
    name_end = payload.find(b'\x00', 5)
    if name_end < 0:
        return None, None, None
    channel = payload[5:name_end].decode('ascii', errors='replace')
    data_start = name_end + 1
    raw = payload[data_start:data_start + data_len]
    return channel, dim, raw


def unpack_psic_values(raw, channel):
    """
    Unpack raw PSIC data into (points_list, gains_list).

    motion_point_cloud: float32 × 3 → list of (x, y, z) tuples
    micro_point_cloud:  int16 × 3  → list of (x, y, z) tuples
                         (firmware PSIC header incorrectly says FLOAT)
    gain factor:        uint8, first 4B = frame_idx, rest = gain values
    """
    if channel == PSIC_CH_MOTION_CLOUD:
        fmt = '<f'; es = 4
    elif channel == PSIC_CH_MICRO_CLOUD:
        # Firmware PSIC header says FLOAT but data is int16
        fmt = '<h'; es = 2
    elif channel == PSIC_CH_GAIN_FACTOR:
        # First 4 bytes = frame index, rest = uint8 gain values per range bin
        return None, list(raw[4:])
    else:
        return None, []

    n = len(raw) // es
    vals = [struct.unpack(fmt, raw[i*es:(i+1)*es])[0] for i in range(n)]
    points = []
    for i in range(0, len(vals) - 2, 3):
        points.append((vals[i], vals[i+1], vals[i+2]))
    return points, None


# ── TLV Parser (radar_analysis_spi) ────────────────────────────

def parse_tlv_header(data):
    """Parse a 4-byte TLV header. Returns (tl_type, tl_flag, tl_len)."""
    if len(data) < 4:
        return None
    return struct.unpack('<BBH', data[:4])


def parse_point(data):
    """Parse a 12-byte PointCloud3D struct. Returns dict."""
    if len(data) < POINT_STRUCT_SIZE:
        return None
    x, y, z, vel, snr, _ = struct.unpack('<hhhhhh', data[:POINT_STRUCT_SIZE])
    return {'x': x, 'y': y, 'z': z, 'vel': vel, 'snr': snr}
