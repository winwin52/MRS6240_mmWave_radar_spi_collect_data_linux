#!/usr/bin/env python3
"""
r3_databox 固件 — SPI 雷达数据采集脚本

功能：在 Linux 上通过 SPI 采集正和微芯 MRS6240 毫米波雷达数据。
固件要求：雷达烧录 r3_databox 固件。

输出 CSV：
  - radar_data_<时间戳>.csv  : 点云数据（运动点云 + 微动点云）
  - gain_factor_<时间戳>.csv : 每距离门增益因子（呼吸信号提取用）

用法：
    sudo python collect_r3_databox.py [--speed 10000000] [--burst 3] [--timeout 0]
    按 Ctrl+C 停止采集。

依赖：pip install spidev lgpio
"""

import time, sys, os, csv, signal, struct, argparse
from datetime import datetime

import spidev, lgpio as sbc

# ═══════════════════════════════════════════════════════════════
# HIF 协议常量与工具函数
# ═══════════════════════════════════════════════════════════════
MAGIC = 0xA5  # HIF 帧魔数

def parse_hif_header(raw6):
    """解析 6 字节 HIF 帧头。"""
    flags = raw6[2] & 0x3F
    return {
        'msg_id': raw6[3],                                # 消息 ID
        'length': raw6[4] | ((raw6[5] & 0x0F) << 8),     # payload 长度
        'more':   (flags >> 5) & 1,                       # 还有更多帧标志
        'check':  (flags >> 4) & 1,                       # 是否有 Check32
    }

def hif_check8_ok(raw6):
    """验证 HIF 帧头 Check8 校验。"""
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)

def build_poll(poll_type=1, burst_period=3):
    """
    构造完整 HIF POLL 帧：6 字节头 + 4 字节 payload + 4 字节 Check32。

    poll_type=1 (ACK)：路由到 hif_Msg_ReportEnable()，触发雷达数据上报。★ 正确用法
    poll_type=2 (APP_CUBE)：路由到 cube_report_retry_handler()，空函数。  ★ 错误用法，MISO 全 0xFF
    """
    # ---- 6 字节帧头 ----
    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0               # Check8 占位，稍后计算
    h[2] = 0x15            # type=1 (主机→雷达), flags=5 (REQ|Check32)
    h[3] = 0x0C            # msg_id = HOST_READ (轮询命令)
    h[4] = 4               # payload 长度低字节
    h[5] = 0               # payload 长度高字节 + seq
    h[1] = (~(h[0] + h[2] + h[3] + h[4] + h[5])) & 0xFF

    # ---- 4 字节 payload：poll_type + err_num + burst_num (小端) ----
    pl = bytearray([poll_type, 0, burst_period & 0xFF, (burst_period >> 8) & 0xFF])

    # ---- Check32：反码累加 (hdr[2:6] + payload)，再取反 ----
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)   # 进位折叠
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')

    return bytes(h) + bytes(pl) + ck


# ═══════════════════════════════════════════════════════════════
# PSIC 协议解析（r3_databox 调试数据格式）
# ═══════════════════════════════════════════════════════════════

def parse_psic(payload):
    """
    解析 PSIC 调试帧 payload。
    返回 (通道名, 维度, 原始字节)。
    维度 2 → 3D 点云。维度 0 → 1D 数据（gain_factor）。
    """
    if len(payload) < 6:
        return None, None, None
    data_len = payload[2] | (payload[3] << 8)
    dim = ((payload[1] >> 1) & 0x7F) + 1          # 存储值 = dim-1
    name_end = payload.find(b'\x00', 5)            # 通道名以 NULL 结尾
    if name_end < 0:
        return None, None, None
    channel = payload[5:name_end].decode('ascii', errors='replace')
    data_start = name_end + 1
    raw = payload[data_start:data_start + data_len]
    return channel, dim, raw


def unpack_values(raw, channel):
    """
    将 PSIC 原始字节解析为点列表或增益值。

    motion_point_cloud：float32 × 3D → [(x, y, z), ...]
    micro_point_cloud： int16 × 3D  → [(x, y, z), ...]
                         (固件 PSIC 头错误标记为 FLOAT，实际是 int16)
    gain factor：       uint8，前 4 字节 = frame_idx，剩余 = 各距离门增益值
    """
    if channel == 'motion_point_cloud':
        fmt = '<f'; es = 4
    elif channel == 'micro_point_cloud':
        # 固件 PSIC 头标记为 FLOAT 类型，但实际数据是 int16
        fmt = '<h'; es = 2
    elif channel == 'gain factor':
        # uint8，前 4 字节 = 帧序号，后面是各距离门的增益因子值
        return None, list(raw[4:])
    else:
        return None, []

    n = len(raw) // es
    vals = [struct.unpack(fmt, raw[i*es:(i+1)*es])[0] for i in range(n)]
    points = []
    for i in range(0, len(vals) - 2, 3):
        points.append((vals[i], vals[i+1], vals[i+2]))
    return points, None


# ═══════════════════════════════════════════════════════════════
# SPI / GPIO 硬件接口（K1 MUSE Pi Pro）
# ═══════════════════════════════════════════════════════════════
SPI_BUS, SPI_DEV, SPI_HZ = 3, 0, 10_000_000     # SPI3, CS0, 10MHz
GPIO_CHIP, INT_GPIO = 0, 49                      # gpiochip0, GPIO49 (中断引脚)

running = True
def _stop(sig, frame):
    global running; running = False; print("\n正在停止...")
signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

class Radar:
    """MRS6240 雷达 SPI 通信句柄。"""

    def __init__(self, speed=SPI_HZ):
        # GPIO 初始化：中断引脚（PA6 → K1 GPIO49）
        self.gh = sbc.gpiochip_open(GPIO_CHIP)
        sbc.gpio_claim_input(self.gh, INT_GPIO)
        # SPI 初始化：Mode 0, 8bit
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEV)
        self.spi.mode = 0b00
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = 8
        self.spi.cshigh = False

    def int_level(self):
        return sbc.gpio_read(self.gh, INT_GPIO)

    def wait_int_high(self, ms=1000):
        """等待中断引脚拉高（雷达数据就绪），超时返回 False。"""
        deadline = time.monotonic() + ms / 1000.
        while time.monotonic() < deadline:
            if sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def wait_int_low(self, ms=1000):
        """等待中断引脚拉低。"""
        deadline = time.monotonic() + ms / 1000.
        while time.monotonic() < deadline:
            if not sbc.gpio_read(self.gh, INT_GPIO):
                return True
            time.sleep(0.0001)
        return False

    def poll(self, burst=3):
        """
        发送 POLL 命令，读取所有 HIF 帧，返回 [{'hdr', 'payload'}, ...]。

        SPI 线格式（含 DMA 填充字节）：
          [magic][check8][msghdr 4B][pad 1B][payload N B][check32 4B][pad 1B]
          ←──── 7 字节头 ────→
        """
        # 第一步：发送 POLL 命令
        self.spi.xfer2(list(build_poll(poll_type=1, burst_period=burst)))
        # 第二步：等 INT 跳变（雷达处理命令）
        self.wait_int_low(50)
        self.wait_int_high(200)
        time.sleep(0.002)   # 手册要求等 2ms 再读帧头

        # 第三步：发送 dummy 时钟，读取 MISO 返回的 HIF 数据
        chunk = bytes(self.spi.xfer2([0x00] * 4096))
        frames = []
        pos = 0
        while pos < len(chunk) and chunk[pos] == MAGIC:
            if pos + 7 > len(chunk):
                break
            raw6 = chunk[pos:pos+6]                      # 逻辑帧头 6 字节
            hdr = parse_hif_header(raw6)
            if not hif_check8_ok(raw6):
                break
            N = hdr['length']
            if N > 2000 or N == 0:
                break
            payload = chunk[pos+7:pos+7+N]               # 跳过 1 字节 DMA 填充
            wire_len = 7 + N + (5 if hdr['check'] else 0) # 线总长
            frames.append({'hdr': hdr, 'payload': payload})
            if hdr['more'] == 0:                         # 最后一帧
                break
            pos += wire_len
        return frames

    def close(self):
        self.spi.close()


# ═══════════════════════════════════════════════════════════════
# CSV 输出定义
# ═══════════════════════════════════════════════════════════════

CSV_HEADER = ["时间戳", "数据类型", "帧/序号", "数量/长度",
              "X坐标", "Y坐标", "Z坐标", "信噪比", "速度"]

GAIN_HEADER = ["时间戳", "帧序号"] + [f"增益_{i}" for i in range(520)]


# ═══════════════════════════════════════════════════════════════
# 主循环
# ═══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description='MRS6240 r3_databox SPI 数据采集')
    ap.add_argument('--speed', type=int, default=SPI_HZ,
                    help=f'SPI 时钟频率 Hz (默认 {SPI_HZ})')
    ap.add_argument('--burst', type=int, default=3,
                    help='POLL burst_period 参数')
    ap.add_argument('--timeout', type=int, default=0,
                    help='N 秒后自动停止 (0=无限)')
    ap.add_argument('--dir', type=str, default=None,
                    help='输出目录 (默认当前目录)')
    args = ap.parse_args()

    out_dir = args.dir or os.getcwd()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"雷达点云数据_{ts}.csv")
    gain_path = os.path.join(out_dir, f"雷达增益因子_{ts}.csv")

    print(f"r3_databox SPI → {csv_path}")
    print(f"  {args.speed/1e6:.1f} MHz  burst={args.burst}  按 Ctrl+C 停止\n")

    radar = Radar(speed=args.speed)

    # 打开 CSV 文件
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

            # 等待 INT 拉高（雷达有数据就绪）
            if not radar.wait_int_high(ms=5000 if rounds == 1 else 2000):
                print(f"[第{rounds}轮] INT 超时，等待中...")
                continue

            # 发送 POLL，读取数据帧
            frames = radar.poll(burst=args.burst)

            for f in frames:
                hdr = f['hdr']
                if hdr['msg_id'] != 0xC6:    # 只处理 PSIC 调试数据 (0xC6)
                    continue

                channel, dim, raw = parse_psic(f['payload'])
                if channel is None:
                    continue

                t = datetime.now().strftime('%Y-%m-%d-%H:%M:%S.') + \
                    f"{int(time.time() % 1 * 1000):03d}"

                if channel == 'gain factor':
                    # ---- gain_factor：1D 增益值，用于呼吸信号提取 ----
                    _, gains = unpack_values(raw, channel)
                    frame_idx = struct.unpack('<I', raw[:4])[0]
                    gain_w.writerow([t, frame_idx] + gains)
                    gain_rows += 1
                    continue

                # ---- 点云数据（运动点云 / 微动点云） ----
                points, _ = unpack_values(raw, channel)
                if points is None:
                    continue

                n_points = len(points)
                for i, (x, y, z) in enumerate(points):
                    csv_w.writerow([t, channel, i, n_points,
                                    round(x, 2), round(y, 2), round(z, 2), '', ''])
                    total_rows += 1

            # 定期刷盘
            if rounds % 20 == 0 or (frames and not frames[-1]['hdr']['more']):
                csv_f.flush()
                gain_f.flush()

            if not args.timeout and rounds == 1:
                print(f"[第{rounds}轮] 正在接收数据，按 Ctrl+C 停止。")

    finally:
        radar.close()
        csv_f.close()
        gain_f.close()

    print(f"\n完成。{rounds} 轮，{total_rows} 行点云，{gain_rows} 行增益因子")
    print(f"  点云: {csv_path}")
    print(f"  增益: {gain_path}")


if __name__ == '__main__':
    main()
