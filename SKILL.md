# RS6x/7x 毫米波雷达 SPI 通信 — AI 智能体 Skill

## 概述

本 Skill 教 AI 智能体掌握"在 Linux 上通过 SPI 与正和微芯 RS6x/7x 毫米波雷达通信"的技术。AI 获取此 Skill 后，可帮助开发者针对其具体硬件平台（Linux SBC + MRS6240/MRS6130 雷达）编写正确的 SPI 数据采集脚本。

## 触发条件

当用户提到以下任意关键词时加载本 Skill：
- "雷达 SPI 通信"、"毫米波雷达 Linux"、"MRS6240 SPI"
- "r3_databox 采集"、"HIF 协议"、"雷达数据 Python"
- "正和微芯"、"POSSUMIC radar"、"radar SPI host"

## 核心知识

### 1. 硬件架构

```
┌─────────────────────┐          ┌─────────────────────────┐
│   Linux SBC (主机)   │   SPI    │   MRS6240 雷达 (从机)    │
│   Python spidev      │◄────────►│   FreeRTOS + HIF        │
│   产生 SCLK          │ Mode 0   │   56MHz DMA             │
│   GPIO 读 INT        │          │   r3_databox 固件        │
└─────────────────────┘          └─────────────────────────┘
```

- **主机**：Linux SBC（MUSE Pi Pro / 树莓派等），SPI 主模式，产生 SCLK
- **从机**：MRS6240 雷达，SPI 从模式 (CPOL=0, CPHA=0)，56MHz
- **中断**：雷达 PA6 引脚，高电平有效，通知主机有数据就绪

### 2. HIF 通信协议（核心）

#### 帧格式（SPI 线格式，含 DMA 填充）

```
字节0    1      2        3      4      5      6      7..6+N    ...末2    末1
[MAGIC][CK8][FLAGS][MSG_ID][LEN_LO][LEN_HI][PAD][PAYLOAD N B][CK32 4B][PAD]
 0xA5
 ←────────────── 7 字节帧头（含 1 字节 DMA 填充）──────────→  ←─ 5B 尾 ─→
```

**关键点：帧头在 SPI 线上是 7 字节（6 帧头 + 1 DMA 填充），读 buffer[0:6] 取逻辑帧头，payload 从 buffer[7] 开始。**

#### 通信流程（正确时序）

```
1. 等 INT 拉高                    # 雷达有数据
2. 等 2ms                        # 稳定
3. 主机发 POLL 命令               # HOST_READ (0x0C), poll_type=1
4. 等 INT 跳变（低→高）           # 雷达处理完成
5. 等 2ms                        # 手册 §6.3.1.2
6. 发送 dummy 时钟读 MISO 数据     # xfer2([0x00]*4096)
7. 逐帧解析：
   - 读 6 字节 (buffer[pos:pos+6])
   - 验证 Check8
   - 读 payload (buffer[pos+7:pos+7+N])
   - 检查 more 标志
   - more=1: 继续读下一帧
   - more=0: 本轮结束
8. （可选）发送 Complete Ack       # A5 4B 03 0C 00 00
9. INT 拉低，雷达可进入低功耗
```

#### POLL 命令构建（关键帧格式）

```python
def build_poll(poll_type=1, burst_period=3):
    h = bytearray(6)
    h[0] = 0xA5                    # HIF 魔数
    h[1] = 0                       # Check8 占位
    h[2] = 0x15                    # type=1(Host→Dev), flag=Req|Check32
    h[3] = 0x0C                    # msg_id = HOST_READ
    h[4] = 4                       # payload 长度低字节
    h[5] = 0                       # payload 长度高字节 (4→低4bit=4)
    h[1] = (~sum(h[0]+h[2:6])) & 0xFF  # Check8

    pl = bytes([poll_type, 0, burst_period & 0xFF, (burst_period>>8) & 0xFF])

    # Check32 = ~one_comp_sum(hdr[2:6] + payload)
    data = bytes(h[2:6]) + pl
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total = (total + w) & 0xFFFFFFFF
        total += (total >> 32)
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')

    return bytes(h) + pl + ck
```

#### 关键消息 ID

| MsgID | 方向 | 含义 |
|-------|------|------|
| 0x0C | 主机→雷达 | HOST_READ (POLL)，触发数据上报 |
| 0xC6 | 雷达→主机 | PSIC 调试数据（r3_databox 固件的主数据通道） |
| 0xC3 | 雷达→主机 | MMW_FRAME_UPLOAD（radar_analysis_spi 固件的点云通道） |

#### Check8 算法

```python
def check8_ok(raw6):
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)
```

### 3. 三个致命陷阱

**陷阱 1：poll_type 必须用 1（非 2）**
- PDF 示例使用 poll_type=2，路由到空的 `cube_report_retry_handler()` → MISO 全 0xFF
- 正确：poll_type=1，路由到 `hif_Msg_ReportEnable()` → 触发数据上报

**陷阱 2：SPI DMA 填充字节**
- 固件 `hif_com.c:293` 在帧头后和 Check32 后各加 1 字节填充
- 必须跳过：读 buffer[0:6] 取帧头，buffer[7:7+N] 取 payload

**陷阱 3：CH347 USB 总线冲突**
- 开发板上的 CH347 USB-SPI 桥与 18pin 排针共享同一组 SPI 引脚
- 必须断开 USB，由 K1 独占 SPI 总线

### 4. 引脚连接模板

```
K1 Pin 19 (SPI3_MOSI)  → 雷达 PA2
K1 Pin 21 (SPI3_MISO)  ← 雷达 PA3
K1 Pin 23 (SPI3_SCLK)  → 雷达 PA1
K1 Pin 24 (SPI3_CS)    → 雷达 PA0
K1 Pin 22 (GPIO49)     ← 雷达 PA6 (INT)
K1 Pin 6  (GND)        — 雷达 GND
K1 Pin 1  (3.3V)       → 雷达 VCC (Pin 12)
```

### 5. Linux 端配置模板

```bash
# K1 MUSE Pi Pro：下载预配置 DTB
wget https://archive.spacemit.com/ros2/prebuilt/brdk_libs/spi/k1-x_MUSE-Pi-Pro.dtb
sudo cp k1-x_MUSE-Pi-Pro.dtb /boot/spacemit/6.6.63/
sudo reboot

# 安装 Python 依赖
pip install spidev lgpio

# 验证
ls /dev/spidev3.0
```

适配其他 Linux SBC：修改 SPI 总线号和 GPIO 编号即可。

### 6. r3_databox 数据通道

| 通道名 (PSIC) | 数据类型 | 维度 | 用途 |
|:---|:---|:---|:---|
| `gain factor` | uint8 | 1D (∼520 值) | 每距离门增益因子 → 呼吸检测 |
| `motion_point_cloud` | float32 | 3D (x,y,z cm) | 运动点云 → 姿态检测 |
| `micro_point_cloud` | int16 | 3D (x,y,z cm) | 微动点云 → 心跳检测 |

> 注意：微动点云 PSIC 类型标记为 FLOAT，实际数据是 int16（固件已知问题），解析时需强制按 int16 读取。

## AI 智能体使用指引

当用户说"帮我写一个采集雷达数据的脚本"时：

1. **确认硬件**：Linux SBC 型号、SPI 总线号、GPIO 中断引脚号
2. **确认固件**：r3_databox 还是 radar_analysis_spi
3. **复用本 Skill 的核心逻辑**：
   - `build_poll()` 函数（poll_type=1 不变）
   - SPI 线格式处理（7 字节帧头，跳过 DMA 填充）
   - PSIC 或 TLV 解析逻辑
   - INT 驱动的轮询循环
4. **适配部分**：
   - `SPI_BUS`、`SPI_DEV`、`GPIO_CHIP`、`INT_GPIO` 常量
   - SPI 时钟频率（建议 8-10 MHz）
5. **验证要点**：
   - 先用 `ls /dev/spidev*` 确认 SPI 设备存在
   - 先用回环测试（短接 MOSI-MISO）验证 SPI 硬件
   - 先用 `cat /sys/kernel/debug/gpio` 确认 INT 引脚有跳变（雷达在运行）

## 参考文件

- 完整采集脚本：`scripts/collect_r3_databox.py`
- 协议详细文档：`docs/hif_protocol.md`
- 项目主页：`README.md`
