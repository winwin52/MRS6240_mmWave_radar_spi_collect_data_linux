# RS6x/7x mmWave Radar — SPI Data Collection on Linux

Collect raw radar data (point clouds, gain factors) from POSSUMIC RS6x/RS7x mmWave radar modules directly via SPI on Linux SBCs (K1 MUSE Pi Pro, Raspberry Pi, etc.) — **no Windows, no CH347 USB bridge, no proprietary GUI tools required**.

## Hardware

- **Radar module**: MRS6240_P2512 on MRS6240-AIP-DEV-V1 (RS6240, 2T4R MIMO)
- **Tested SBC**: K1 MUSE Pi Pro (SpacemiT M1, RISC-V, Bianbu Linux 6.6)
- **Firmware**: `r3_databox` or `radar_analysis_spi` (pre-compiled in SDK)

## Hardware Preparation (5 Steps)

### 1. Solder 0Ω resistors on R1, R4, R6, R8

These 4 resistors connect the SPI pins (PA0–PA3) from the radar chip to the 18-pin header. By default they are unpopulated. Solder bridges (blob of solder) work fine — they're zero-ohm jumpers.

Locations on MRS6240-AIP-DEV-V1 (near the 18-pin header):

| Resistor | Signal | Purpose |
|----------|--------|---------|
| R1 | PA0 (SPI0_CS) | SPI chip select |
| R4 | PA1 (SPI0_CLK) | SPI clock |
| R6 | PA2 (SPI0_MOSI) | SPI MOSI |
| R8 | PA3 (SPI0_MISO) | SPI MISO |

### 2. Disconnect CH347 USB

The CH347 USB-SPI bridge on the dev board shares PA0–PA3 with the pin header. If USB is connected, the CH347 drives the SPI lines and conflicts with your Linux host. **Unplug the USB cable entirely.**

### 3. Power the radar from your SBC's 3.3V pin

Connect your SBC's 3.3V output to the radar board's **VCC** pin on the 18-pin header. The radar module draws ~200 mA which most SBC 3.3V rails can supply.

> On K1 MUSE Pi Pro: use **Pin 1** (3.3V) or **Pin 17** (3.3V).

### 4. Remove VDD3V3 and VCC jumper caps

With external 3.3V power via the pin header, the on-board LDOs are bypassed. Remove the jumper caps on **VDD3V3** and **VCC** to avoid conflicts.

### 5. Solder pin headers to the 18-pin header

Solder a male pin header (2.54mm pitch) onto the radar board's 18-pin connector block for reliable wire connections.

## Wiring

### K1 MUSE Pi Pro 40-pin ↔ Radar 18-pin Header

| K1 Pin | K1 Signal | Dir | Radar Pin | Radar Signal | Notes |
|--------|-----------|-----|-----------|--------------|-------|
| 19 | SPI3_MOSI | → | 4 | PA2 (MOSI) | |
| 21 | SPI3_MISO | ← | 5 | PA3 (MISO) | |
| 23 | SPI3_SCLK | → | 3 | PA1 (SCLK) | |
| 24 | SPI3_CS | → | 2 | PA0 (CS) | |
| 22 | GPIO_49 | ← | 8 | PA6 (INT) | Interrupt signal |
| 6 | GND | — | 1 | GND | Common ground |
| 1 | 3.3V | → | 12 | VCC | Power |

All signals are 3.3V logic — no level shifting needed.

### Radar 18-pin Header Reference

| Pin | Signal | Function |
|-----|--------|----------|
| 1 | GND | Ground |
| 2 | PA0 | SPI0_CS |
| 3 | PA1 | SPI0_CLK |
| 4 | PA2 | SPI0_MOSI |
| 5 | PA3 | SPI0_MISO |
| 8 | PA6 | INT / LED1 |
| 12 | VCC | Power input (3.3V) |
| 13 | VDDIO | IO power |
| 15 | NRST | Reset |

### MODE SEL DIP Switches

Set on the radar dev board:

| Switch | State | Purpose |
|--------|-------|---------|
| 5, 6, 7 | **ON** | Route SPI to pin header (not Type-C) |
| 9 | OFF | Disconnect PA6 from USB |
| 10 | ON | PA6 = GPIO (INT output mode) |

## Firmware Comparison

Two firmware options. Choose one:

| | r3_databox | radar_analysis_spi |
|---|---|---|
| **Data output** | gain_factor + motion cloud + micro cloud | motion cloud + micro cloud (Cartesian/Polar) |
| **Protocol** | PSIC (0xC6) | TLV MMW_FRAME_UPLOAD (0xC3) |
| **Frame rate** | 20 fps (50ms period) | 20 fps (50ms period) |
| **Best for** | Respiration (gain_factor per range bin) | Clean point clouds (x/y/z/vel/snr) |
| **Shell** | No | No (auto-start) |
| **CSV output** | `radar_data_*.csv` + `gain_factor_*.csv` | `analyse_frames_*.csv` + `analyse_points_*.csv` |

Both communicate via the exact same HIF protocol over SPI.

## Quick Start

### 1. Enable spidev on K1

See [k1_config/spi_device_tree.md](k1_config/spi_device_tree.md) for detailed steps. Summary:

```bash
# After modifying device tree & rebooting:
ls /dev/spidev3.0   # should exist
```

### 2. Install Python packages

```bash
sudo apt update
sudo apt install python3-spidev python3-libgpiod python3-gpiod
```

### 3. Run the collector

For **r3_databox** firmware (gain factor + point clouds):

```bash
sudo python scripts/collect_r3_databox.py --speed 10000000
```

For **radar_analysis_spi** firmware (TLV point clouds with snr/velocity):

```bash
sudo python scripts/collect_analyse_spi.py --speed 8000000
```

Press `Ctrl+C` to stop. CSV files are written to the current directory.

### Script Options

| Option | Default | Description |
|--------|---------|-------------|
| `--speed` | 10,000,000 (r3) / 8,000,000 (analyse) | SPI clock rate in Hz |
| `--burst` | 3 | POLL burst_period |
| `--timeout 300` | 0 (forever) | Auto-stop after N seconds |

## How It Works

```
┌──────────────┐  SPI (Mode 0, 8-10 MHz)   ┌──────────────────┐
│  Linux SBC   │◄──────────────────────────►│  MRS6240 Radar   │
│  (K1 / Pi)   │   CS, SCLK, MOSI, MISO    │  (SPI Slave)     │
│              │◄───────────────────────────│  INT (PA6/GPIO)  │
│  Python      │   Interrupt (active HIGH)  │  FreeRTOS + HIF  │
│  spidev +    │                            │  56 MHz SPI      │
│  lgpio       │                            │  DMA enabled     │
└──────────────┘                            └──────────────────┘
```

1. Radar raises INT when data is ready
2. Host sends POLL (HOST_READ 0x0C) command via SPI
3. Radar responds with HIF frames containing point cloud data
4. Host sends Complete Ack to end the burst
5. Script parses PSIC or TLV payload, writes CSV

## Protocol Details

See [docs/hif_protocol.md](docs/hif_protocol.md) for the complete HIF communication protocol, timing diagrams, frame formats, and known pitfalls.

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| MISO all 0xFF | CH347 bus conflict | Unplug USB cable |
| MISO all 0xFF | Wrong poll_type | Use poll_type=1 (ACK), not 2 |
| Data offset / garbage | DMA padding not handled | Read 7-byte header, skip pad byte |
| INT never goes HIGH | Wrong MODE SEL switch | DIP 5/6/7 ON, 9 OFF, 10 ON |
| "No such device" | spidev not configured | Check device tree (see k1_config/) |
| Script imports fail | Missing packages | `apt install python3-spidev python3-libgpiod` |

## License

MIT — see LICENSE file.

## Credits

Developed through reverse-engineering the POSSUMIC HIF protocol, cross-referencing the SDK source code (`hif_com_spi.c`, `hif.c`, `hif_msg.c`) against the official PDF documentation, and a week of hardware debugging on the K1 MUSE Pi Pro.
