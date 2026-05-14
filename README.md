# uflash

Open-source USB flash tool for Unisoc / Spreadtrum devices. Reads standard `.pac` firmware archives and programs them over USB bulk using the BSL (Boot Serial Loader) protocol.

Two binaries are built:

| Binary | Description |
|---|---|
| `uflash` | Command-line tool — scriptable, stdout-safe |
| `uflash-tui` | Interactive TUI (FTXUI) — partition picker + live progress gauge |

---

## Requirements

| Dependency | Notes |
|---|---|
| C++17 compiler | GCC 9+ or Clang 10+ |
| CMake 3.14+ | |
| libusb-1.0 | `brew install libusb` / `apt install libusb-1.0-0-dev` |
| [upac](../upac) | Sibling directory — PAC reader library |
| [FTXUI](https://github.com/ArthurSonzogni/ftxui) v5 | Fetched automatically by CMake (for `uflash-tui` only) |

### macOS

The USB interface is held by IOKit's `AppleUSBCDCACMData` driver when the device enumerates as CDC-ACM. `uflash` calls `set_configuration(1)` to briefly evict the IOKit driver before claiming the interface, then issues `SET_CONTROL_LINE_STATE` to assert DTR+RTS — required for the BROM bulk OUT endpoint to accept data.

### Linux

Grant access without `sudo` via a udev rule:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1782", MODE="0666"
```

Reload: `udevadm control --reload && udevadm trigger`

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

Both `uflash` and `uflash-tui` land in `build/`. FTXUI is fetched automatically on the first build.

---

## Entering Download Mode

Hold **Vol Down** (or the board's BOOT/BROM pin) while plugging in USB. The device enumerates as VID `0x1782`, PID `0x4d00` / `0x5d00` / `0x3d00`.

If FDL1/FDL2 is already running from a previous partial flash, `uflash` detects this automatically and skips straight to the flash phase.

---

## CLI Usage

```
uflash <firmware.pac> [options]
```

### Flash modes

| Flag | Behaviour |
|---|---|
| *(default)* | Full flash — repartitions eMMC from PAC XML, backs up NV first |
| `--full-flash` | Same as default, explicit |
| `--preserve-layout` | Skips repartition; skips protected partitions (`UserData`, `fixnv*`, `prodnv`, etc.) — safe for OTA-style updates |

### Partition selection

| Flag | Behaviour |
|---|---|
| `--partition <id>` | Flash only this partition (repeatable) |
| `--skip <id>` | Skip this partition (repeatable) |

`<id>` matches against both the XML `id` field and the `id_name` / block-device name.

### Transfer options

| Flag | Description |
|---|---|
| `--disable-transcode` | Enable vendor fast-lane mode (`BSL_CMD_DISABLE_TRANSCODE`). Skips HDLC byte-stuffing — roughly 2× throughput on supported FDL2 builds |
| `--debug-fast-lane` | Print per-packet diagnostics for the pipeline path |
| `--debug-protocol` | Dump every BSL frame send/receive to stdout |

### Other options

| Flag | Description |
|---|---|
| `--dump-firmware <dir>` | Read all partitions from the device into `<dir>/*.bin` |
| `--skip-nv-backup` | Skip NV backup before repartition (faster, destructive) |
| `--fdl2-settle-ms <ms>` | Extra settle delay after FDL1 executes before the FDL2 handshake |
| `--reset-only` | Send `BSL_CMD_NORMAL_RESET` and exit |
| `--dump-xml` | Print the embedded PAC XML config and exit |
| `--dump-descriptors` | Print USB descriptors of all detected Unisoc devices and exit |

### Examples

```bash
# Full flash (repartitions, backs up NV)
build/uflash firmware.pac

# Preserve layout + fast-lane (typical OTA update)
build/uflash firmware.pac --preserve-layout --disable-transcode

# Flash only the Super partition
build/uflash firmware.pac --preserve-layout --disable-transcode --partition Super

# Dump firmware to disk for backup
build/uflash firmware.pac --dump-firmware ./backup

# Soft-reset a device that is stuck in download mode
build/uflash --reset-only
```

---

## TUI Usage

```bash
build/uflash-tui <firmware.pac>
```

Opens a full-screen partition selector, then launches `uflash` as a subprocess and renders its output as a live progress view with an animated gauge, speed, and ETA.

`uflash-tui` expects the `uflash` binary to be in the same directory as itself.

**Selector keys:**

| Key | Action |
|---|---|
| `j` / `k` / `↑↓` | Navigate list |
| `Space` | Toggle partition |
| `a` | Toggle select-all |
| `p` / `f` | Preserve-layout / Full-flash mode |
| `t` | Toggle fast-lane (`--disable-transcode`) |
| `g` | Toggle protocol debug |
| `Enter` | Start flash |
| `q` | Quit |

---

## Supported Devices

Any Unisoc / Spreadtrum SoC enumerating with VID `0x1782` and PID `0x4d00`, `0x5d00`, or `0x3d00` in BROM or BSL mode. Tested on UMS512 / SC9863A based platforms.

---

## Project Layout

```
uflash/
├── src/
│   ├── main.cpp               — entry point, banner
│   ├── cli_options.cpp        — argument parsing
│   ├── flash_workflow.cpp     — flash orchestration, partition loop
│   ├── bsl_protocol.cpp       — BSL framing, commands, handshake
│   ├── usb_device.cpp         — libusb abstraction, endpoint discovery
│   ├── transfer_pipeline.cpp  — background frame-encoder thread
│   └── tui_app.cpp            — FTXUI interactive frontend
├── include/uflash/
│   ├── bsl_protocol.h
│   ├── usb_device.h
│   ├── transfer_pipeline.h
│   ├── flash_workflow.h
│   └── cli_options.h
└── CMakeLists.txt
```

See [IMPLEMENTATION.md](IMPLEMENTATION.md) for a full description of the protocol and internal architecture.
