# HIF (Host Interface) Protocol — SPI Communication with RS6x/7x Radar

> Cross-referenced from POSSUMIC SDK source code (`hif.c`, `hif_com_spi.c`, `hif_msg.c`, `hif_com.c`) and PDF manuals (HIF Communication Format V1.1, R3 Host Driver V1.1).

## Architecture

HIF is the **protocol layer**. SPI/UART/I2C are the **physical transport layers**.

```
┌──────────────────────┐          ┌──────────────────────┐
│  Host (Linux SBC)    │   SPI    │  Device (Radar)      │
│  Python spidev+lgpio │◄────────►│  FreeRTOS hif_task   │
│  HIF frames          │  Mode 0  │  HIF frames via DMA  │
└──────────────────────┘          └──────────────────────┘
```

- Host is the **SPI master** (generates SCLK)
- Radar is the **SPI slave** (Mode 0: CPOL=0, CPHA=0)
- Radar uses **INT pin** (PA6, active HIGH) to signal data availability

## Communication Sequence

### Phase 1: Wakeup (non-HIF)

```
Host ── [0x55, 0xFF] ──→ Device  (repeat until ACK)
Host ←── [0x79, 0x79] ── Device  (wakeup ACK)
```

- Host sends 0x55 0xFF, waits 2ms, reads 2 bytes
- If response == 0x79 0x79, wakeup successful
- If no response, wait 10ms and retry
- **Note**: The r3_databox firmware auto-wakes — this phase may be skipped.

### Phase 2: Data Report (main collection loop)

```
1. Device:  INT → HIGH (data ready)
2. Host:    Wait 2ms
3. Host:    Send POLL (HOST_READ 0x0C, poll_type=1)
4. Host:    Wait INT LOW → Wait INT HIGH (~200μs)
5. Host:    Wait 2ms
6. Host:    Read HIF frames via SPI (send dummy clocks)
7. Host:    Send Complete Ack (A5 4B 03 0C 00 00)
8. Device:  INT → LOW (burst complete)
```

### Phase 3: Sleep Release (optional)

Send HOST_READ with burst_period=0 to put the device into low-power mode.

## HIF Frame Format

### Logical Frame (6-byte header + payload + Check32)

```
┌──────┬────────┬──────────────────────┬──────────┬──────────┐
│Magic │ Check8 │ MsgHdr (4 bytes)     │ Payload  │ Check32  │
│ 1B   │ 1B     │ type|flag|id|len|seq │ N bytes  │ 4 bytes  │
│ 0xA5 │        │                      │ (0–4095) │ (optional)│
└──────┴────────┴──────────────────────┴──────────┴──────────┘
```

### SPI DMA Wire Format

The firmware's SPI DMA adds alignment padding:

```
┌──────┬────────┬──────────────────────┬─────┬──────────┬──────────┬─────┐
│Magic │ Check8 │ MsgHdr (4 bytes)     │ PAD │ Payload  │ Check32  │ PAD │
│ 1B   │ 1B     │                      │ 1B  │ N bytes  │ 4 bytes  │ 1B  │
└──────┴────────┴──────────────────────┴─────┴──────────┴──────────┴─────┘
│←──────────── 7 bytes on wire ──────────→│           │←─ 5 bytes ─→│
```

> Source: `hif_com.c:293-294`. `hdr_len = HIF_HEAD_LEN + 1 = 7`, `check_len = HIF_CHKEC_LEN + 1 = 5`.

**The host MUST read 7 bytes for the header block but only use the first 6. Payload starts at wire offset 7.**

### MsgHdr Bit Fields (4 bytes, little-endian)

| Bits | Field | Description |
|------|-------|-------------|
| 1:0 | type | 1=Host→Device, 2=Device→Host, 3=Debug/Local |
| 7:2 | flag | bit0=REQ, bit1=Enc, bit2=Check32, bit3=More, bit4=Ext, bit5=MAC32 |
| 15:8 | msg_id | Command/message ID |
| 27:16 | length | Payload length (0–4095) |
| 30:28 | seq | Sequence number (0–7, rolls over) |
| 31 | frag | Fragment flag |

### Key Flag Bits

| Bit | Name | Meaning |
|-----|------|---------|
| 5 | **more** | 0=last frame in burst, 1=more frames coming |

## Checksums

### Check8 (Header)

```python
def check8(header):
    # header = [magic, _, type_flags, msg_id, len_lo, len_hi]
    s = header[0] + header[2] + header[3] + header[4] + header[5]
    header[1] = (~s) & 0xFF
```

### Check32 (Payload)

```c
// SDK algorithm: one's-complement sum over header[2:6] + payload, then NOT
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
// Result: ~checksum32(0, &hdr[2], payload_len + 4)
```

> **Known quirk**: The r3_databox firmware's TX checksum is offset by +5 from the standard calculation. Likely a counter or init-value difference in `checksum32_calc`. Check8 is 100% reliable — use that to verify data integrity.

## Key Message IDs

### Host → Device (type=1)

| MsgID | Name | Description |
|-------|------|-------------|
| 0x00 | VERSION | Get device version info |
| 0x05 | WAKEUP | Wake up device |
| 0x07 | START_CTRL | Start/stop radar data |
| 0x0A | READ_VERSION | Read firmware version |
| **0x0C** | **HOST_READ / POLL** | **Poll for data (triggers report)** |
| 0x60 | START_CTRL | r3_databox specific start/stop |
| 0x61 | MOTION_SENSOR_PARA_CFG | Configure radar params via TLV |

### Device → Host (type=2)

| MsgID | Name | Description |
|-------|------|-------------|
| 0xC1 | FFT_DATA | Raw FFT DataCube |
| **0xC3** | **MMW_FRAME_UPLOAD** | Point cloud / tracking targets |
| **0xC6** | PSIC_DEBUG | r3_databox debug data stream |
| 0xF0 | DBG_PRINT | Debug printf output |
| 0xFF | STARTUP | Device boot notification |

## POLL Command (0x0C)

### Payload Structure

```
[poll_type 1B] [err_num 1B] [burst_num 2B LE]
```

- **poll_type=0 (ACK)**: Routes to `hif_Msg_ReportEnable()` → **enables data reporting** ← USE THIS
- **poll_type=2 (APP_CUBE)**: Routes to `cube_report_retry_handler()` → **weak empty function** ← BROKEN

> **Critical**: Always use `poll_type=1` (ACK mode). The PDF examples use `poll_type=2` which routes to a do-nothing handler.

### Building a POLL in Python

```python
MAGIC = 0xA5

def build_poll(poll_type=1, burst_period=3):
    h = bytearray(6)
    h[0] = MAGIC
    h[1] = 0  # check8 placeholder
    h[2] = 0x15  # type=1(Host→Dev), flag=5(REQ|Check32)
    h[3] = 0x0C  # msg_id = HOST_READ
    h[4] = 4     # payload len = 4
    h[5] = 0
    h[1] = (~(h[0] + h[2] + h[3] + h[4] + h[5])) & 0xFF

    pl = bytearray([poll_type, 0,
                    burst_period & 0xFF,
                    (burst_period >> 8) & 0xFF])

    # Check32 over hdr[2:6] + payload
    data = bytes(h[2:6]) + bytes(pl)
    total = 0
    for i in range(0, len(data), 4):
        w = int.from_bytes(data[i:i+4].ljust(4, b'\x00'), 'little')
        total += w
        total = (total & 0xFFFFFFFF) + (total >> 32)
    ck = (~total & 0xFFFFFFFF).to_bytes(4, 'little')

    return bytes(h) + bytes(pl) + ck
```

## Complete Ack

After reading all frames (when `more=0`), the host sends Complete Ack to end the burst:

```
A5 4B 03 0C 00 00
```

- Magic=0xA5, Check8=0x4B, Type=3(Debug), MsgID=0x0C, Length=0
- The device confirms receipt by pulling INT LOW

## Timing Constants

| Parameter | Value | Source |
|-----------|-------|--------|
| Power-on wait | 6 sec | PDF conservative |
| Wakeup retry interval | 10 ms | PDF §6.3.1.3 |
| Wakeup response wait | 2 ms | PDF §6.3.1.3 |
| POLL → read header wait | 2 ms | PDF §6.3.1.2 |
| Header → payload wait | >200 μs | PDF §6.3.1.2 |
| After more=0, check INT | 50 μs | PDF §6.3.1.2 |
| Host read timeout | 100 ms | `hif.c sendFrameTo` |
| Radar SPI slave speed | 56 MHz | `prj_config.h` |
| Host SPI speed (tested) | 8–10 MHz | K1 spidev |

## INT Signal Logic

- **Active HIGH**: `hif_PM_WakeHost()` sets PA6 HIGH when data is ready
- **Active LOW**: `hif_PM_SleepHost()` sets PA6 LOW after Complete Ack received
- INT toggles ~30 Hz (2× frame rate) — this is normal and indicates the firmware is running

## Three Pitfalls (Why MISO Shows All 0xFF)

During development, we spent a week debugging "MISO all 0xFF" before finding these three root causes:

### 1. CH347 USB Bus Conflict

**Symptom**: MISO reads all 0xFF even though INT is toggling.

**Cause**: The MRS6240-AIP-DEV-V1 has a CH347 USB-SPI bridge that shares PA0–PA3 with the pin header. When USB is connected, the CH347 drives the SPI lines, conflicting with your Linux host.

**Fix**: Unplug USB entirely. Power the radar from your SBC's 3.3V pin instead.

### 2. Wrong poll_type (2 instead of 1)

**Symptom**: MISO all 0xFF after sending POLL command.

**Cause**: The PDF examples use `poll_type=2` (APP_CUBE), but in the firmware source (`hif.c:893`), poll_type=2 routes to `cube_report_retry_handler()` — a `__weak` function with an empty body. No data is queued for transmission.

Poll dispatch logic in `hif.c`:
```c
switch (poll_type) {
    case 0: hif_Msg_ReportEnable(¶m);  break;  // ← triggers data!
    case 2: cube_report_retry_handler(); break;  // ← empty, does nothing
}
```

**Fix**: Use `poll_type=1` (ACK) which routes to `hif_Msg_ReportEnable()`.

### 3. SPI DMA Alignment Padding

**Symptom**: Data appears but is offset by 1 byte — checksums fail, parsing garbage.

**Cause**: The firmware's SPI DMA block (`hif_com.c:293-294`) adds 1 byte of padding after the 6-byte HIF header and 1 byte after the 4-byte Check32. The host was reading exactly 6 header bytes, causing all subsequent parsing to be 1 byte off.

**Fix**: Read 7 bytes for the header block, use only bytes [0:6]. Read payload starting at wire offset 7.

```python
HDR_WIRE = 7    # 6 logical + 1 pad
CHK_WIRE = 5    # 4 logical + 1 pad

raw6 = chunk[pos:pos+6]           # logical header
payload = chunk[pos+7:pos+7+N]    # skip the pad byte
wire_len = 7 + N + 5              # total bytes on wire
```

## Firmware Internals (Reference)

### Key State Machine (`hif.c`)

```
host_interface_task (main loop):
  1. HIF_MsgHdl_Process()  → dispatch received commands
  2. hif_Host_CheckReady()  → check if TX DMA should fire
  3. hif_Msg_ReportSend()   → queue data to SPI TX DMA
  4. hif_Msg_SendDone()     → pull INT LOW, host_state=SLEEP
```

### TX DMA Trigger Conditions

```c
// hif_Host_CheckReady() returns 1 when ALL of:
//   CONFIG_HIF_SEND_DMA == 1
//   host_state == ACTIVE
//   notifyType == IO
//   burst_period > 0
//   frame_link_time < sendFrameTo (100ms timeout)
```

### CS Pin Behavior

- **CS falling edge**: SPI RX starts, command bytes received via DMA
- **CS rising edge**: `hif_com_spi_cs_isr()` → `hif_com_data_recv()` processes the command
- **Two separate CS cycles needed**: one for POLL, one for reading data
- DMA transfer in the data phase completes on CS↑

## Radar SPI Pins

| Radar Pin | Signal | Function |
|-----------|--------|----------|
| PA0 | SPI0_CS | Chip select (input) |
| PA1 | SPI0_SCLK | Serial clock (input) |
| PA2 | SPI0_MOSI | Master-out, slave-in |
| PA3 | SPI0_MISO | Master-in, slave-out |
| PA6 | INT | Interrupt (active HIGH output) |

Firmware config: `SPI_MODE_SLAVE`, CPOL=0, CPHA=0, 56 MHz, 8-bit, DMA enabled.
