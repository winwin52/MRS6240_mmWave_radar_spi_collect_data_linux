# HIF 通信协议 — 正和微芯 RS6x/7x 毫米波雷达 SPI 通信

> 交叉验证来源：SDK 源码 `hif.c`、`hif_com_spi.c`、`hif_msg.c`、`hif_com.c` + PDF 手册（HIF 通信格式 V1.1 + R3 主机驱动 V1.1）。

## 架构

HIF (Host Interface) 是**协议层**，SPI/UART/I2C 是**物理传输层**。

```
┌──────────────────────┐          ┌──────────────────────┐
│  主机 (Linux SBC)     │   SPI    │  雷达 (MRS6240)       │
│  Python spidev+lgpio │◄────────►│  FreeRTOS hif_task   │
│  HIF 帧              │  Mode 0  │  HIF 帧通过 DMA 发送   │
└──────────────────────┘          └──────────────────────┘
```

- 主机：SPI 主机，产生 SCLK
- 雷达：SPI 从机 (Mode 0: CPOL=0, CPHA=0)
- 雷达通过 INT 引脚（PA6，高电平有效）通知主机有数据就绪

## 完整通信时序

```
1. 雷达 INT 拉高                    # 数据就绪
2. 主机等 2ms                      # 稳定
3. 主机发 POLL (HOST_READ 0x0C)    # poll_type=1 (ACK)
4. 主机等 INT 跳变（低→高）         # 雷达处理命令
5. 主机等 2ms                      # 手册 §6.3.1.2
6. 主机发 dummy 时钟读 MISO 数据    # xfer2([0x00]*4096)
7. 逐帧解析 HIF 帧
8. more=1 → 继续读；more=0 → 发送 Complete Ack
9. 雷达 INT 拉低                    # burst 结束
```

**Complete Ack**：`A5 4B 03 0C 00 00`（6 字节，通知雷达本轮数据接收完成，雷达可进入低功耗）

## HIF 帧格式

### 逻辑帧（6 字节头 + payload + Check32）

```
[MAGIC 1B][Check8 1B][MsgHdr 4B][Payload N B][Check32 4B]
```

MsgHdr 位域（4 字节，小端序）：

| 位段 | 字段 | 说明 |
|------|------|------|
| 1:0 | type | 1=主机→雷达, 2=雷达→主机, 3=Debug |
| 7:2 | flag | [bit0=REQ, bit1=Enc, bit2=Check32, bit3=More, bit4=Ext, bit5=MAC32] |
| 15:8 | msg_id | 消息 ID |
| 27:16 | length | Payload 长度 (0–4095) |
| 30:28 | seq | 序列号 (0–7) |
| 31 | frag | 分片标志 |

### SPI DMA 线格式（含填充）

```
[MAGIC][Check8][MsgHdr 4B][PAD 1B][Payload N B][Check32 4B][PAD 1B]
 ←────────── 7 字节 ──────────→                    ←── 5 字节 ──→
```

> 来源：`hif_com.c:293-294`。hdr_len = 6 + 1 = 7，check_len = 4 + 1 = 5。
> **主机必须读 7 字节取前 6 字节帧头，payload 从线偏移 7 开始取值。**

### 重要标志位

| 位 | 名称 | 含义 |
|----|------|------|
| bit3 | **more** | 0=最后一帧, 1=还有更多帧 |

## 校验和

### Check8（帧头校验）

```python
def check8(raw6):
    s = raw6[0] + raw6[2] + raw6[3] + raw6[4] + raw6[5]
    return raw6[1] == ((~s) & 0xFF)
```

### Check32（Payload 校验）

```c
// SDK 算法：反码累加 (hdr[2:6] + payload)，然后取反
uint32_t checksum32(uint32_t init, uint32_t *data, int byte_len) {
    uint32_t sum = init;
    int dwords = byte_len >> 2;
    while (dwords--) sum += *data++;
    int rem = byte_len & 3;
    if (rem) {
        uint32_t mask = 0x00FFFFFF >> ((3 - rem) << 3);
        sum += (*data & mask);
    }
    return sum;
}
// 最终值 = ~checksum32(0, &hdr[2], payload_len + 4)
```

> r3_databox 固件的 Check32 与标准算法有 +5 的偏移（疑似 `checksum32_calc` 与 `HIF_CheckSum32` 之间的计数器/初始值差异）。Check8 100% 可靠，用 Check8 验证数据完整性即可。

## 关键消息 ID

### 主机 → 雷达 (type=1)

| MsgID | 名称 | 说明 |
|-------|------|------|
| 0x00 | VERSION | 获取设备版本 |
| 0x0A | READ_VERSION | 读固件版本 |
| **0x0C** | **HOST_READ / POLL** | **轮询数据，触发雷达上报** |
| 0x60 | START_CTRL | r3_databox 启动/停止 |

### 雷达 → 主机 (type=2)

| MsgID | 名称 | 说明 |
|-------|------|------|
| 0xC1 | FFT_DATA | 原始 FFT DataCube |
| **0xC3** | **MMW_FRAME_UPLOAD** | 点云/跟踪目标（radar_analysis_spi 固件） |
| **0xC6** | **PSIC_DEBUG** | PSIC 调试数据（r3_databox 固件主数据通道） |

## POLL 命令 (0x0C)

### Payload 结构

```
[poll_type 1B] [err_num 1B] [burst_num 2B LE]
```

- **poll_type=1 (ACK)**：路由到 `hif_Msg_ReportEnable()` → **触发数据上报** ← 正确！
- **poll_type=2 (APP_CUBE)**：路由到 `cube_report_retry_handler()` → **空函数，无响应** ← 错误！

> PDF 手册示例使用 poll_type=2，导致 MISO 全 0xFF。必须使用 poll_type=1。

### Python 构建代码

```python
MAGIC = 0xA5

def build_poll(poll_type=1, burst_period=3):
    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0                                 # Check8 占位
    h[2] = 0x15                              # type=1, flag=Req|Check32
    h[3] = 0x0C                              # msg_id = HOST_READ
    h[4] = 4                                 # payload 长度
    h[5] = 0
    h[1] = (~(h[0]+h[2]+h[3]+h[4]+h[5])) & 0xFF

    pl = bytearray([poll_type, 0,
                    burst_period & 0xFF,
                    (burst_period >> 8) & 0xFF])

    # Check32：反码累加 (hdr[2:6] + payload)，取反
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')
    return bytes(h) + bytes(pl) + ck
```

## 时序常数

| 参数 | 值 | 来源 |
|------|-----|------|
| 上电等待 | 6 秒 | PDF 保守建议 |
| POLL 后等读帧头 | 2 ms | PDF §6.3.1.2 |
| 帧头后等读 payload | >200 μs | PDF §6.3.1.2 |
| more=0 后检查 INT | 50 μs | PDF §6.3.1.2 |
| 主机读超时 | 100 ms | `hif.c sendFrameTo` |
| 雷达 SPI 从机速率 | 56 MHz | `prj_config.h` |
| 主机 SPI 速率（实测）| 8–10 MHz | K1 spidev |

## INT 信号逻辑

- **高电平**：`hif_PM_WakeHost()` 拉高 PA6，表示数据就绪
- **低电平**：`hif_PM_SleepHost()` 拉低 PA6，收到 Complete Ack 后
- 正常运行时 INT 约 30Hz 翻转（2× 帧率）

## 三个踩坑记录

### 1. CH347 USB 总线冲突

- **现象**：MISO 全 0xFF，INT 正常跳变
- **原因**：MRS6240-AIP-DEV-V1 上的 CH347 USB-SPI 桥与 18pin 排针共享 PA0–PA3。USB 连接时 CH347 驱动 SPI 总线
- **解决**：断开 USB，用 K1 3.3V 引脚给雷达供电

### 2. poll_type 用错 (2 而非 1)

- **现象**：MISO 全 0xFF
- **原因**：PDF 示例用 poll_type=2，在 `hif.c:893` 路由到 `__weak` 空函数 `cube_report_retry_handler()`
- **解决**：改用 poll_type=1

### 3. SPI DMA 对齐填充

- **现象**：数据能收到但偏移 1 字节，校验失败
- **原因**：`hif_com.c:293-294` 在帧头后加 1 字节填充，Check32 后加 1 字节填充
- **解决**：帧头从线偏移 0 读 6 字节，payload 从线偏移 7 开始

## 雷达固件内部状态机（参考）

```
host_interface_task 主循环（hif.c）：
  1. HIF_MsgHdl_Process()  → 分发收到的命令
  2. hif_Host_CheckReady() → 检查是否满足 TX DMA 触发条件
  3. hif_Msg_ReportSend()  → 将数据入队到 SPI TX DMA
  4. hif_Msg_SendDone()    → 拉低 INT，host_state=SLEEP

TX DMA 触发条件（全部满足）：
  CONFIG_HIF_SEND_DMA == 1
  host_state == ACTIVE
  notifyType == IO
  burst_period > 0
```

### CS 引脚行为

- CS 下降沿：SPI RX 开始，DMA 接收命令
- CS 上升沿：`hif_com_spi_cs_isr()` → 处理命令并准备 TX 数据
- 需要两次独立 CS 周期：一次发 POLL，一次读数据
- DMA 模式下数据在 CS↑ 时完成传输

## 雷达 SPI 引脚

| 雷达引脚 | 信号 | 功能 |
|----------|------|------|
| PA0 | SPI0_CS | 片选（输入） |
| PA1 | SPI0_SCLK | 时钟（输入） |
| PA2 | SPI0_MOSI | 主机出从机入 |
| PA3 | SPI0_MISO | 从机出主机入 |
| PA6 | INT | 中断，高电平有效（输出） |
