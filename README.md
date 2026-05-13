# uflash — Unisoc Flash Tool

`uflash` is a lightweight, high-performance Unisoc flashing tool and firmware dumper built against the local `upac` PAC parsing library. It is designed to reliably navigate the Unisoc BootROM/FDL handshakes and provide robust flashing for modern devices (like the Meig SLM500S module, Android 10).

## Key Features

- **Protocol Support**: Understands legacy `0x7E` handshakes and modern `0xAE` Host Protocol connections.
- **Auto-Repartitioning**: Automatically detects `0x96` or `0xFE` mismatch errors from `ExecNandInit` and dynamically generates a `BSL_CMD_REPARTITION` payload to format the `8GB+` eMMC layouts correctly.
- **Fast Flashing (MTK Style)**: Utilizes the `BSL_CMD_DISABLE_TRANSCODE (0x21)` command to bypass HDLC byte-stuffing, unlocking massive transfer speeds (`25+ MiB/s`) for large images like `super.img`. Includes a robust 60s cooldown tolerance for heavy TLC cache flushes.
- **Firmware Dumping**: Can read directly from the eMMC via `BSL_CMD_READ_START` to extract live partitions without needing a pre-existing scatter file.
- **Smart NV Backup**: Safely backs up critical calibration NV partitions before repartitioning.
- **Adaptive USB Chunking**: Modulates USB `libusb_bulk_transfer` sizes to prevent eMMC controller exhaustion, automatically gracefully backing off when FDL2 signals timeouts.

## Build Requirements

`uflash` depends on:
- Valid `libusb-1.0` headers
- The local `upac` PAC library

```bash
mkdir build && cd build
cmake ..
make -j8
```

Linux note:
`uflash` talks to the device through `libusb`, so production installs should either run with elevated privileges or ship a udev rule that grants access to Unisoc bootrom devices such as `1782:4d00`.

## Usage

```bash
./uflash <firmware.pac> [OPTIONS]
```

### Core Operations

**Standard Full Flash:** (Will auto-repartition if needed)
```bash
./uflash firmware.pac
```

**Force Full Flash:** (Explicitly triggers comprehensive flashing modes)
```bash
./uflash firmware.pac --full-flash
```

**Preserve Current Layout:** (Skips sensitive bootloader data and avoids repartitioning)
```bash
./uflash firmware.pac --preserve-layout
```

**Fast Flashing Mode:** (Bypasses byte-stuffing overhead for massive performance gains)
```bash
./uflash firmware.pac --disable-transcode
```

### Selective Actions

**Flash a Single Partition:**
```bash
./uflash firmware.pac --partition boot
```

**Skip Specific Partitions:**
```bash
./uflash firmware.pac --skip userdata --skip system
```

**Skip NV Calibration Backup:** (Useful if the partitions are already severely corrupted)
```bash
./uflash firmware.pac --skip-nv-backup
```

**Dump Firmware from Device:** (Extracts the device's partitions matching the PAC XML into a local folder)
```bash
./uflash firmware.pac --dump-firmware ./device_dump/
```

### Utility & Debugging

**Restart Device in Normal Mode:**
```bash
./uflash --reset-only
```

**Print Embedded PAC XML:**
```bash
./uflash firmware.pac --dump-xml
```

**Debug FDL Timeline:**
```bash
./uflash firmware.pac --fdl2-settle-ms 5000     # Introduce a delay before FDL2 initiates
./uflash firmware.pac --debug-fast-lane         # Print detailed fast-lane pipeline heuristics
./uflash firmware.pac --debug-protocol          # Hex-dump every USB packet transmitted
```

## Notes & Findings
- **0xFE Repartition Rejection**: If `ExecNandInit` returns `0xFE`, it typically indicates that `szID1` and `szID2` are unsynchronized in the partition schema, or the `userdata` size isn't set to auto-fill (`0x00` size) for modern FDL2s.
- **Named Downloads**: File downloads directed at `0x00` base addresses require `DownloadByID`. `uflash` will automatically pad out unaligned named-download trails (e.g. `gnssmodem.bin`) to prevent CRC timeouts.
- **The Fast Lane (`DA_INFO_T`)**: For `disable-transcode` to work, the `uflash` protocol engine parses the hidden `DA_INFO_T` struct returned by modern Unisoc BootROMs (even on failure packets) to toggle the `bDisableTransCode` flag.
- **Linux Transport Robustness**: The USB transport now retains its own `libusb` context, scans all alternate settings for bulk endpoints, and claims the discovered interface before the first handshake. This is important because Linux tends to be less forgiving than macOS when the active configuration or interface selection is wrong.
