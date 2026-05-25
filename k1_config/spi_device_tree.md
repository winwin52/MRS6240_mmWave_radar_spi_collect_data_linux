# K1 MUSE Pi Pro — SPI3 spidev Configuration

Enable `/dev/spidev3.0` on the K1 MUSE Pi Pro (SpacemiT M1, Bianbu Linux 6.6) for radar SPI communication.

## Background

The K1's SPI3 controller is at `spi@d401c000` using pins GPIO_75–78. By default, the device tree only has the controller node — you need to add a `spidev` child node to expose it as a userspace device.

The relevant pinctrl configuration (`k1-x_pinctrl.dtsi`):

```dts
pinctrl_ssp3_0: ssp3_0_grp {
    pinctrl-single,pins = <
        K1X_PADCONF(GPIO_75, MUX_MODE2, (EDGE_NONE | PULL_DIS | PAD_3V_DS4))  /* ssp3_sclk */
        K1X_PADCONF(GPIO_76, MUX_MODE2, (EDGE_NONE | PULL_UP  | PAD_3V_DS4))  /* ssp3_frm (CS) */
        K1X_PADCONF(GPIO_77, MUX_MODE2, (EDGE_NONE | PULL_DIS | PAD_3V_DS4))  /* ssp3_txd (MOSI) */
        K1X_PADCONF(GPIO_78, MUX_MODE2, (EDGE_NONE | PULL_DIS | PAD_3V_DS4))  /* ssp3_rxd (MISO) */
    >;
};
```

## Step-by-Step: Add spidev

### 1. Decompile current DTB

```bash
sudo dtc -I dtb -O dts \
  /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb \
  -o /tmp/k1-x_MUSE-Pi-Pro.dts
```

### 2. Find & edit the SPI3 node

Look for `spi@d401c000` in the DTS. It should have `k1x,ssp-id = <3>;`.

Add a `spidev@0` child node:

```dts
&spi3 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_ssp3_0>;
    k1x,ssp-disable-dma;
    status = "okay";
    k1x,ssp-clock-rate = <25600000>;

    spidev@0 {
        compatible = "rohm,dh2228fv";
        reg = <0>;
        spi-max-frequency = <10000000>;
    };
};
```

### 3. Recompile DTB

```bash
sudo dtc -I dts -O dtb \
  /tmp/k1-x_MUSE-Pi-Pro.dts \
  -o /boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb
```

### 4. Reboot & verify

```bash
sudo reboot
# After reboot:
ls /dev/spidev3.0   # should exist
```

## Install Python Packages

```bash
sudo apt update
sudo apt install python3-spidev python3-libgpiod python3-gpiod
```

The scripts import `spidev` and `lgpio`:
- `spidev` — from `python3-spidev`
- `lgpio` — from `python3-libgpiod` (provides `lgpio` module for `/dev/gpiochip*` access)

## GPIO Reference

| GPIO Number | K1 Pin | Function |
|-------------|--------|----------|
| 49 | 22 | INT input (radar PA6) |
| 75 | 23 | SPI3 SCLK |
| 76 | 24 | SPI3 CS |
| 77 | 19 | SPI3 MOSI |
| 78 | 21 | SPI3 MISO |

GPIO chip: `/dev/gpiochip0`

## Verification Commands

```bash
# Check SPI controllers
ls /sys/class/spi_master/

# Check SPI devices
ls /sys/bus/spi/devices/

# Confirm SPI3 ID
cat /proc/device-tree/soc/spi@d401c000/k1x,ssp-id

# Monitor GPIO state (INT pin should toggle ~30 Hz when radar is running)
sudo cat /sys/kernel/debug/gpio | grep GPIO_49

# Test SPI loopback (short MOSI & MISO on K1 pins 19 & 21)
python3 -c "
import spidev
spi = spidev.SpiDev()
spi.open(3, 0)
spi.mode = 0
spi.max_speed_hz = 1000000
result = spi.xfer2([0xA5, 0x5A, 0x00])
print('Loopback:', [hex(b) for b in result])
spi.close()
"
```

## Alternative SBCs (Raspberry Pi, etc.)

The same approach works on any Linux SBC with an SPI controller:

1. Enable SPI in `raspi-config` or device tree
2. Install the same Python packages
3. Adjust `SPI_BUS` and `SPI_DEV` in the scripts (e.g., `0, 0` for Raspberry Pi SPI0)
4. Adjust `GPIO_CHIP` and `INT_GPIO` for your board's GPIO numbering
