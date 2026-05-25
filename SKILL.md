---
name: MRS6240-mmwave-radar-spi
description: >
  教 AI 在 Linux 上通过 SPI 与正和微芯 MRS6240 毫米波雷达通信。
  触发词：雷达 SPI 通信、毫米波雷达 Linux、MRS6240、r3_databox、
  HIF 协议、正和微芯、POSSUMIC radar、radar SPI host、
  SPI 雷达采集、mmwave radar spi、雷达点云 Python、Linux SBC 雷达、
  呼吸心跳雷达数据、写雷达采集脚本。
metadata:
  type: skill
  author: winwin52
  repo: https://github.com/winwin52/MRS6240_mmWave_radar_spi_collect_data_linux
---

# MRS6240 毫米波雷达 SPI 通信 — AI 智能体 Skill

## 适用场景

本 Skill 让 AI 学会"在 Linux 上通过 SPI 直连正和微芯 MRS6240 毫米波雷达，采集原始点云与呼吸信号数据"。AI 获取此 Skill 后，可帮助开发者为自己的 Linux SBC（树莓派、MUSE Pi Pro、旭日 X3 等）写出正确的 SPI 数据采集脚本。

## 安装方式

### Claude Code 用户

```bash
# 从 GitHub 自动安装
claude skill install https://github.com/winwin52/MRS6240_mmWave_radar_spi_collect_data_linux/blob/master/SKILL.md

# 或手动复制到项目目录
cp SKILL.md /path/to/your/project/.claude/skills/
```

安装后，当用户提到"雷达 SPI 采集"、"MRS6240"等关键词时，Claude Code 自动加载本 Skill。

### 其他 AI 工具（ChatGPT、Gemini、通义千问 等）

直接把本文件内容复制粘贴给 AI，然后说：

> "我要在 Linux 上通过 SPI 采集正和微芯 MRS6240 雷达数据，请依据以上 Skill 帮我写采集脚本。我的硬件是 [树莓派 4B / MUSE Pi Pro / 其他]，SPI 总线号是 [0]，GPIO 中断引脚是 [49]。"

### 普通用户（不看 Skill，直接拿脚本跑）

参考 [README.md](README.md)，下载仓库中的 `scripts/collect_r3_databox.py` 按说明运行即可。

---

## 1. 硬件架构（核心认知）

```
┌─────────────────────┐          ┌─────────────────────────┐
│   Linux SBC (主机)   │   SPI    │   MRS6240 雷达 (从机)    │
│   Python spidev      │◄────────►│   FreeRTOS + HIF        │
│   产生 SCLK          │ Mode 0   │   56MHz DMA             │
│   GPIO 读 INT        │          │   r3_databox 固件        │
└─────────────────────┘          └─────────────────────────┘
```

- **主机**：Linux SBC，SPI 主模式，产生 SCLK。脚本用 Python `spidev` 库操作 SPI，`lgpio` 库读 GPIO 中断。
- **从机**：MRS6240 雷达，SPI 从模式 (CPOL=0, CPHA=0)，56MHz。烧录 r3_databox 固件。
- **中断**：雷达 PA6 引脚，高电平有效，通知主机"我有数据准备好了，来读"。

## 2. HIF 通信协议（核心逻辑）

### 2.1 SPI 线格式（含 DMA 填充字节）

```
字节0    1      2        3      4      5      6      7..6+N     ...末2   末1
[MAGIC][CK8][FLAGS][MSG_ID][LEN_LO][LEN_HI][PAD][PAYLOAD N B][CK32 4B][PAD]
 0xA5
 ←────────────── 7 字节帧头（含 1 字节 DMA 填充）──────────→  ←─ 5B 尾 ─→
```

> 固件 `hif_com.c:293` 在 6 字节帧头后和 4 字节 Check32 后各插入 1 字节 DMA 对齐填充。
> 读取时：`raw6 = buffer[pos:pos+6]`（逻辑帧头），`payload = buffer[pos+7:pos+7+N]`（跳过填充）。

### 2.2 通信时序（POLL → 读数据 → ACK）

```
1. while (gpio_read(INT) == LOW) { sleep(0.1ms); }  // 等雷达拉高 INT
2. time.sleep(0.002)                                 // 等待 2ms 稳定
3. spi.xfer2(build_poll(poll_type=1, burst=3))       // 发 POLL 命令
4. wait_until(INT==LOW, timeout=50ms)                // INT 跳低（雷达处理中）
5. wait_until(INT==HIGH, timeout=200ms)              // INT 跳高（雷达处理完）
6. time.sleep(0.002)                                 // 等 2ms 再读帧头
7. raw_data = spi.xfer2([0x00] * 4096)               // 发 dummy 时钟，收 MISO 数据
8. 逐帧解析：
   - 读 raw_data[pos:pos+6]，验证 Check8
   - 读 raw_data[pos+7:pos+7+N]，取出 payload
   - 检查 more 标志：bit3=1 → 继续读下一帧，bit3=0 → 本轮结束
9. （可选）spi.xfer2([0xA5,0x4B,0x03,0x0C,0x00,0x00])  // Complete Ack
```

### 2.3 POLL 命令构建代码（可直接复用）

```python
MAGIC = 0xA5

def build_poll(poll_type=1, burst_period=3):
    # poll_type=1 → ACK 模式 → 路由到 hif_Msg_ReportEnable() → 触发数据上报 ✓
    # poll_type=2 → APP_CUBE → 路由到空函数 → MISO 全 0xFF ✗

    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0                             # Check8 占位
    h[2] = 0x15                          # type=1(Host→Dev), flag=Req|Check32
    h[3] = 0x0C                          # msg_id = HOST_READ
    h[4] = 4                             # payload 长度低字节
    h[5] = 0                             # (长度高 4bit) | (seq 高 4bit)
    h[1] = (~(h[0] + h[2] + h[3] + h[4] + h[5])) & 0xFF

    pl = bytearray([poll_type, 0, burst_period & 0xFF, (burst_period >> 8) & 0xFF])

    # Check32：反码累加 (msg_hdr 4B + payload N B)，结果取反
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total = (total + w) & 0xFFFFFFFF
        total += (total >> 32)           # 进位折叠
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')

    return bytes(h) + bytes(pl) + ck
```

### 2.4 HIF 帧解析

```python
def parse_hif_header(raw6):
    flags = raw6[2] & 0x3F
    return {
        'msg_id': raw6[3],
        'length': raw6[4] | ((raw6[5] & 0x0F) << 8),  # 12bit 长度，最大 4095
        'more':   (flags >> 5) & 1,                     # bit5 = 还有更多帧
        'check':  (flags >> 4) & 1,                     # bit4 = 有 Check32
    }

def check8_ok(raw6):
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)
```

### 2.5 关键消息 ID

| MsgID | 方向 | 含义 |
|-------|------|------|
| 0x0C | 主机→雷达 | HOST_READ (POLL)，触发雷达上报数据 |
| 0xC6 | 雷达→主机 | PSIC 调试数据（r3_databox 主数据通道） |
| 0xC3 | 雷达→主机 | MMW_FRAME_UPLOAD（radar_analysis_spi 固件） |

## 3. 三个致命陷阱（必须避开）

> 这三个坑叠加导致 MISO 全 0xFF，完全无法通信。

| # | 现象 | 根因 | 解决方案 |
|---|------|------|----------|
| 1 | MISO 全 0xFF | **CH347 USB 总线冲突**：开发板 CH347 USB-SPI 桥与排针共享 PA0–PA3 | 断开 USB，改用 K1 3.3V 引脚供电 |
| 2 | MISO 全 0xFF | **poll_type=2** 路由到空函数 `cube_report_retry_handler()` | 改用 `poll_type=1`（路由到 `hif_Msg_ReportEnable()`） |
| 3 | 数据偏移 1 字节 | **SPI DMA 填充**：固件在帧头后和 Check32 后各插入 1 字节填充 | 读 `buffer[0:6]` 取帧头，`buffer[7:7+N]` 取 payload |

## 4. 引脚连接

```
主机 (Linux SBC)                       雷达 (MRS6240 18pin 排针)
├── SPI_MOSI ──────────────────────→ PA2 (Pin 4)
├── SPI_MISO ←────────────────────── PA3 (Pin 5)
├── SPI_SCLK ──────────────────────→ PA1 (Pin 3)
├── SPI_CS   ──────────────────────→ PA0 (Pin 2)
├── GPIO_INT ←────────────────────── PA6 (Pin 8)
├── GND      ─────────────────────── GND (Pin 1)
└── 3.3V     ──────────────────────→ VCC (Pin 12)
```

> K1 MUSE Pi Pro 对应编号：SPI3_MOSI=Pin19, SPI3_MISO=Pin21, SPI3_SCLK=Pin23, SPI3_CS=Pin24, GPIO49=Pin22, GND=Pin6, 3.3V=Pin1。
> 均为 3.3V 电平，无需电平转换。适配其他 SBC 只需修改 SPI 总线号和 GPIO 编号。

## 5. r3_databox 雷达硬件调整

1. **焊接 R1、R4、R6、R8**（0Ω，焊锡短接即可）— 连通 PA0–PA3 到排针
2. **给排针孔焊接排针** — 方便杜邦线连接
3. **断开 USB** — 避免 CH347 总线冲突
4. **拔下 VDD3V3 和 VCC 跳线帽** — 使用外部供电
5. **K1 3.3V → 雷达 VCC** — 独立供电

DIP 开关：5/6/7=ON (SPI→排针)，9=OFF，10=ON (PA6=GPIO 中断)。

## 6. Linux 端配置

```bash
# K1 MUSE Pi Pro：下载预配置 DTB（已添加 spidev 节点）
wget https://archive.spacemit.com/ros2/prebuilt/brdk_libs/spi/k1-x_MUSE-Pi-Pro.dtb
sudo cp k1-x_MUSE-Pi-Pro.dtb /boot/spacemit/6.6.63/
sudo reboot

# 验证 SPI 设备
ls /dev/spidev3.0

# 安装 Python 依赖（pip）
pip install spidev lgpio

# 运行采集（需 sudo，SPI 和 GPIO 需要 root）
sudo python collect_r3_databox.py --speed 10000000
```

> 适配其他 Linux SBC：修改脚本中 `SPI_BUS`、`SPI_DEV`、`GPIO_CHIP`、`INT_GPIO` 四个常量即可。

## 7. r3_databox 数据通道

| 通道名 | 类型 | 维度 | 用途 |
|--------|------|------|------|
| `gain factor` | uint8 | 1D (~520 值) | 每距离门增益因子 → 呼吸检测 |
| `motion_point_cloud` | float32 | 3D (x,y,z cm) | 运动点云 → 姿态检测 |
| `micro_point_cloud` | int16 | 3D (x,y,z cm) | 微动点云 → 心跳检测 |

> 微动点云 PSIC 类型标记为 FLOAT，实际是 int16（固件已知 bug），解析时强制按 int16 读取。

## 8. AI 智能体使用指引

当用户说"帮我写一个采集 MRS6240 雷达数据的脚本"时，按以下步骤操作：

### Step 1：确认硬件

向用户确认：
- Linux SBC 型号
- SPI 总线号和设备号（`ls /dev/spidev*`）
- GPIO 中断引脚号
- 雷达固件类型（r3_databox / radar_analysis_spi）

### Step 2：复用核心逻辑（以下代码不变）

- `build_poll()` — POLL 命令构建，`poll_type=1` 不变
- `parse_hif_header()` — HIF 帧头解析
- `check8_ok()` — Check8 验证
- SPI 线格式处理 — 7 字节帧头，payload 从 offset 7 开始

### Step 3：适配部分（以下随硬件变化）

- `SPI_BUS`、`SPI_DEV`：根据 SBC 的 SPI 控制器编号
- `GPIO_CHIP`、`INT_GPIO`：根据 GPIO 引脚映射
- `SPI_HZ`：建议 8–10 MHz
- 雷达固件类型决定解析路径（r3_databox → PSIC 0xC6，analyse_spi → TLV 0xC3）

### Step 4：验证清单

1. `ls /dev/spidev*` — SPI 设备存在
2. 回环测试（短接 MOSI-MISO，发 0xA5 收 0xA5）— SPI 硬件正常
3. `cat /sys/kernel/debug/gpio \| grep INT_PIN` — INT 引脚有跳变（雷达在运行）
4. 上机实际跑脚本，看 MISO 是否返回 0xA5 帧头

## 参考

- 完整脚本：[scripts/collect_r3_databox.py](scripts/collect_r3_databox.py)
- 协议详解：[docs/hif_protocol.md](docs/hif_protocol.md)
- 项目主页：[README.md](README.md)
- 仓库：[MRS6240_mmWave_radar_spi_collect_data_linux](https://github.com/winwin52/MRS6240_mmWave_radar_spi_collect_data_linux)
