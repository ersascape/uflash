# uflash — Implementation Reference

This document describes how `uflash` works end-to-end: USB enumeration, the BSL protocol, the FDL boot sequence, the flash phase, and the throughput optimisations.

---

## Table of Contents

1. [USB Layer](#1-usb-layer)
2. [BSL Protocol Framing](#2-bsl-protocol-framing)
3. [Handshake Sequence](#3-handshake-sequence)
4. [FDL Boot Sequence](#4-fdl-boot-sequence)
5. [Flash Phase](#5-flash-phase)
6. [Named vs Address Downloads](#6-named-vs-address-downloads)
7. [Fast-Lane Mode (DisableTransCode)](#7-fast-lane-mode-disabletranscode)
8. [Transfer Pipeline](#8-transfer-pipeline)
9. [Adaptive Chunk Sizing](#9-adaptive-chunk-sizing)
10. [Repartition](#10-repartition)
11. [NV Backup](#11-nv-backup)
12. [Firmware Dump](#12-firmware-dump)
13. [PAC File Format](#13-pac-file-format)
14. [Command Reference](#14-command-reference)

---

## 1. USB Layer

**Source:** `src/usb_device.cpp`, `include/uflash/usb_device.h`

### Device detection

`uflash` scans all USB devices for VID `0x1782` (Spreadtrum Communications) with one of three PIDs:

| PID | Mode |
|---|---|
| `0x4d00` | BROM / early BSL |
| `0x5d00` | FDL1 running |
| `0x3d00` | Variant / engineering |

For each matching device, `find_bulk_endpoints()` walks all interface alternate settings and selects the first that exposes both a bulk-IN and a bulk-OUT endpoint.

### Interface claim sequence

After opening the device handle, `prepare_handle_for_interface()` runs the following steps in order:

1. **`libusb_set_auto_detach_kernel_driver`** — enables automatic driver detachment on `claim_interface`. Required on Linux.
2. **`libusb_detach_kernel_driver`** — explicit detach for Linux. On macOS this returns `NOT_SUPPORTED` and is silently ignored.
3. **`libusb_set_configuration(1)`** — even when already active, this causes IOKit on macOS to temporarily release any bound interface drivers (e.g. `AppleUSBCDCACMData`), opening a window for `claim_interface` to succeed. Errors are ignored.
4. **`libusb_claim_interface`** — claims the discovered interface.
5. **`libusb_set_interface_alt_setting`** — initialises the IOKit interface service on macOS before bulk transfers can be issued.
6. **`SET_CONTROL_LINE_STATE` (DTR|RTS)** — class control transfer (`bmRequestType=0x21`, `bRequest=34`, `wValue=0x0003`). Unisoc BROMs that enumerate as CDC-ACM require this before the bulk OUT endpoint will accept data. Vendor-class interfaces will NAK or STALL this control transfer; errors are silently ignored.
7. **500 ms settle delay** — allows the device time to finish enumeration before the first bulk transfer.

### Write paths

There are two write paths:

**`write()`** — slices data into 4 KB chunks and calls `libusb_bulk_transfer` for each. Up to 3 retries with escalating timeouts on `LIBUSB_ERROR_TIMEOUT`. This is used for all normal BSL command traffic and the transcoded (HDLC) data path.

**`write_fast()`** — sends the entire buffer in a single `libusb_bulk_transfer` call. Up to 2 retries. Used exclusively for the fast-lane (no-transcode) data path where the frame has already been fully encoded and large atomic writes improve throughput.

---

## 2. BSL Protocol Framing

**Source:** `src/bsl_protocol.cpp`

### Packet structure (pre-frame)

Every BSL packet is constructed as:

```
[TYPE:2] [LENGTH:2] [PAYLOAD:N] [CHECKSUM:2]
```

All fields are big-endian.

- **TYPE** — 16-bit command or response code.
- **LENGTH** — number of payload bytes (not including type, length, or checksum).
- **CHECKSUM** — either CRC-16/CCITT or a 16-bit ones-complement sum, depending on the FDL session. After FDL2 loads and `set_use_checksum(true)` is called, the ones-complement sum is used. The receiver accepts either variant.

### CRC-16 / CCITT

```
poly = 0x1021, init = 0x0000
```

Applied over `TYPE + LENGTH + PAYLOAD` (the pre-frame bytes, before the checksum field is appended).

### Ones-complement checksum

```
sum all big-endian 16-bit words of (TYPE + LENGTH + PAYLOAD)
fold carry bits into the low 16 bits
return ~sum
```

### HDLC framing (transcoded mode)

The raw packet is wrapped in HDLC-style framing:

```
[0x7E] [ESCAPED_BODY] [0x7E]
```

Byte-stuffing rules:
- `0x7E` → `0x7D 0x5E`
- `0x7D` → `0x7D 0x5D`

All other bytes are transmitted as-is.

### No-transcode mode (fast-lane)

When `BSL_CMD_DISABLE_TRANSCODE` has been accepted by FDL2, byte-stuffing is disabled. The raw packet body is placed between two `0x7E` delimiters without escaping. This eliminates the worst-case 2× size expansion and the per-byte conditional that drives the encoder loop.

### Receiving packets

`receive_packet()` reads from the bulk-IN endpoint in 8 KB chunks with a 100 ms per-call timeout, accumulating bytes until a complete HDLC frame is found. In no-transcode mode it counts expected bytes from the LENGTH field to avoid waiting for the closing `0x7E` unnecessarily.

The `DA_INFO_T` detection hook runs on every successfully parsed packet: if the payload is ≥ 8 bytes and the first DWORD is a known DA version (1, 2, or 4), the second DWORD is read as the `bDisableTransCode` capability flag. This surfaces the capability even from error responses (e.g. the `0x96` incompatible-partition reply from `ExecNandInit`).

---

## 3. Handshake Sequence

**Source:** `src/bsl_protocol.cpp` — `handshake()`, `host_handshake()`

### Legacy 0x7E handshake

1. Flush the IN endpoint (two passes with 20 ms timeouts).
2. Sleep 200 ms.
3. Send a 32-byte burst of `0x7E` characters.
4. Wait up to 5 s for any response that contains a `0x7E` byte followed by at least one more byte — interpreted as the start of a BSL version frame.
5. Retry every 200 ms until timeout.

The response, if any, is stored and later returned from `get_last_handshake_response()`. If the printable portion contains the string `"Spreadtrum"`, the device is already running FDL1/FDL2 and the FDL load sequence is skipped.

### Modern 0xAE Host Protocol handshake

Used as a fallback when the `0x7E` burst receives no reply.

The host sends a 20-byte packet:

```
PacketHeader (8 bytes):
  tag       = 0xAE
  data_size = sizeof(DataHeader) in big-endian = 0x0000000C
  flow_id   = 0xFF
  reserved  = 0x0000

DataHeader (12 bytes):
  cmd_type = 0x00000000  (CONNECT)
  addr     = 0x00000000
  size     = 0x00000000
```

A valid response starts with `0xAE`. The host retries every 200 ms until the timeout.

---

## 4. FDL Boot Sequence

**Source:** `src/flash_workflow.cpp` — `run_with_connected_device()`, `load_fdl2()`

If the handshake string does not contain `"Spreadtrum"` (i.e. the device is in raw BROM mode), `uflash` loads two bootloader stages before flashing can begin.

```
BROM → FDL1 load → FDL1 executes → re-enumerate → FDL2 load → FDL2 executes
```

### Stage 1 — FDL1 (BROM download agent)

1. Send `BSL_CMD_CONNECT` and discard the response.
2. Locate `fdl1` by case-insensitive filename search in the extracted PAC temp directory.
3. Look up the FDL1 load address from the PAC XML `<base_address>` field, then from the PAC file-info `addr[]` array.
4. `START_DATA(address, size, checksum)` — address-mode, 8-byte payload, no checksum field.
5. Stream FDL1 in 512-byte `MIDST_DATA` chunks.
6. `END_DATA` → `EXEC_DATA(address)`.
7. Reset and release the `UsbDevice` handle.
8. Poll `UsbDevice::find_any()` for up to 8 s waiting for re-enumeration.

### Stage 2 — FDL2 (flash download loader)

After FDL1 re-enumerates, `uflash` repeats the handshake (5 s, trying both `0x7E` and `0xAE`), then:

1. Call `set_use_checksum(true)` — FDL2 sessions use the ones-complement checksum variant.
2. `BSL_CMD_CONNECT` — if it ACKs, parse the `DA_INFO_T` struct in the reply payload to learn whether `DisableTransCode` is supported.
3. `BSL_CMD_CHANGE_BAUD` — sends the vendor default baud rate (`115200`). On USB bulk this is a logical channel step; the host does not retune USB line coding. ACK failure is non-fatal.
4. Load FDL2 in 1 KB `MIDST_DATA` chunks (same address-mode download as FDL1).
5. `EXEC_DATA(fdl2_address)`.
6. Sleep 2 s for FDL2 to start up.
7. Call `ExecNandInit` (see below).

---

## 5. Flash Phase

**Source:** `src/flash_workflow.cpp` — `init_flash_and_repartition()`, `run_flash_phase()`

### ExecNandInit

`uflash` re-sends `BSL_CMD_EXEC_DATA` (no address payload) to trigger FDL2's `ExecNandInit` routine. Three response codes are significant:

| Code | Meaning |
|---|---|
| `0x80` (`BSL_REP_ACK`) | eMMC partition layout matches; flash phase can proceed |
| `0x96` | Layout mismatch — repartition required |
| anything else | Warning logged; execution continues |

If `0x96` is received and `--disable-transcode` was requested, `uflash` attempts `DisableTransCode` here (before repartition) so that the repartition payload itself benefits from fast-lane framing.

### Partition loop

`run_flash_phase()` iterates the partition list from the PAC XML `<Files>` section, skipping:

- `FDL` and `FDL2` type entries (already loaded).
- Any partition not in `--partition` list, if one was specified.
- Any partition in the `--skip` list.
- Protected partitions (`UserData`, `fixnv*`, `runtimenv*`, `prodnv`, `miscdata`) when `--preserve-layout` is active.

For each partition to flash, `execute_download()` runs the following state machine:

```
START_DATA → [MIDST_DATA × N] → END_DATA
```

---

## 6. Named vs Address Downloads

Whether to use named-download (`DownloadByID`) or address-download depends on the partition:

```cpp
use_named_download = !fc.id_name.empty() &&
                     (fc.type.find("2") != npos || fc.base_address == 0)
```

In practice: partitions with `base_address == 0` (eMMC named partitions like `Super`, `boot`, `vbmeta`) always use named downloads. NAND-backed images (`CODE2`, `YAFFS_IMG2`) use named downloads because their type string ends in `2`.

### Address-mode `START_DATA` payload

```
[ADDR:4 big-endian] [SIZE:4 big-endian] [CHECKSUM:4 big-endian, if non-zero]
```

### Named-mode `START_DATA` payload (DownloadByID)

```
[NAME:72 bytes UTF-16LE, zero-padded] [SIZE:4 LE]         [CHECKSUM:4 big-endian]
```

For files ≥ 4 GiB (DownloadByIDEx):

```
[NAME:72 bytes UTF-16LE]              [SIZE:8 big-endian] [RESERVED:8] [CHECKSUM:4 big-endian]
```

Error responses from `START_DATA`:
- `0x96` — partition size in firmware does not match device partition table.
- `0xFE` — partition name not found in FDL2's internal table (device not repartitioned yet, or wrong id_name).

### Tail padding

For `CODE2` / `YAFFS_IMG2` / `CHECK_NV2` named downloads, if the file size is not a multiple of 4 bytes, `uflash` pads the buffer with zero bytes to align it. This prevents the FDL2 CRC check from stalling on a final partial-word chunk.

---

## 7. Fast-Lane Mode (DisableTransCode)

**Command:** `BSL_CMD_DISABLE_TRANSCODE = 0x21`

When FDL2 supports it, the host sends this command and FDL2 replies `ACK`. From that point on:

- The host sets `disable_transcode_ = true` in `BslProtocol`.
- `frame_packet()` no longer byte-stuffs; it emits `[0x7E][raw body][0x7E]` directly.
- `receive_packet()` switches to the length-aware no-transcode parser.
- `write_fast()` is used instead of `write()` — the full frame is submitted as a single `libusb_bulk_transfer` rather than 4 KB slices.

The capability is advertised through the `DA_INFO_T` struct embedded in certain FDL2 replies (notably the `0x96` ExecNandInit response). The struct layout:

```
offset 0: uint32 version    — 1, 2, or 4 are recognised
offset 4: uint32 flags      — non-zero means DisableTransCode is supported
```

`transcode_supported_` is set to `true` when this struct is successfully parsed from any received packet.

---

## 8. Transfer Pipeline

**Source:** `src/transfer_pipeline.cpp`, `include/uflash/transfer_pipeline.h`

For large transfers (`is_large_transfer_profile`: named download AND file size ≥ 8 MB), `uflash` runs a `PacketPipeline` alongside the main flash loop.

### Purpose

HDLC encoding (byte-stuffing) takes non-trivial CPU time for 32–64 KB chunks. Without pipelining, the main thread must encode the next frame while waiting for the previous ACK — a sequential bottleneck. The pipeline runs the encoder concurrently with the USB round-trip so the next frame is ready to send the moment the ACK arrives.

### Design

```
Worker thread:
  while buffer has data:
    wait until queue depth < max_depth
    encode chunk → FramedPacket { offset, size, frame }
    push to queue

Main thread:
  pop FramedPacket from queue (blocking wait)
  send frame via write_fast()
  wait for ACK
```

The queue depth is 6 frames when fast-lane is active, 4 otherwise.

`pop()` returns `false` (and the main thread falls back to `midst_data()`) when:
- The pipeline is disabled (`enabled_ = false`).
- The current chunk size does not match the pipeline's configured chunk size (e.g. during tail-taper or after a backoff).

`reset(offset, chunk_size)` drains and restarts the worker from a given offset with a new chunk size — used after an ACK failure forces a chunk-size reduction.

---

## 9. Adaptive Chunk Sizing

Large transfers use a multi-tier strategy to maximise throughput while surviving eMMC erase-boundary stalls.

### Partition type tiers

| Partition type | Storage | Max chunk |
|---|---|---|
| `CODE2`, `YAFFS_IMG2`, `CODE264`, `YAFFS_IMG264` | NAND | 32 KB (0x8000) |
| All other named partitions (Super, boot, etc.) | eMMC | 64 KB (0x10000) |

The starting chunk size is `min(kLargeImageStartChunkSize, base_chunk_size)`, ensuring NAND partitions never open at the eMMC chunk size.

### Tail taper

For named downloads, the chunk size and inter-chunk delay are progressively reduced as the remaining bytes approach zero:

| Remaining | Max chunk | Delay |
|---|---|---|
| ≤ 128 B | 32 B | 8 ms |
| ≤ 512 B | 64 B | 7 ms |
| ≤ 2 KB | 128 B | 6 ms |
| ≤ 4 KB | 256 B | 5 ms |
| ≤ 16 KB | 512 B | 4 ms |
| ≤ 32 KB | 1 KB | 3 ms |
| ≤ 128 KB | 2 KB | 2 ms |

This prevents the final tail chunk from being smaller than `kMinNamedTailChunkSize` (32 B), which some FDL2 builds reject.

### Backoff on ACK failure

When `send_framed_packet_fast()` or `midst_data()` returns failure:

1. Pipeline falls back to non-pipelined `midst_data()` for the current chunk.
2. If that also fails and `chunk_size > kLargeImageMinChunkSize` (8 KB), step down the chunk size using `prev_large_transfer_chunk()` through the ladder `{8 KB, 16 KB, 32 KB, 64 KB}`.
3. Increment `chunk_delay_ms` by 2 ms.
4. Set `freeze_recovery = true` — recovery ramp-up is suppressed until 16 MB of stable progress has been made past the backoff point.

### Recovery ramp-up

After `recovery_window` (16) consecutive successful chunks past the `kLargeImageRecoveryDistance` (16 MB) threshold, `uflash` attempts one step up the chunk-size ladder via `next_large_transfer_chunk()`. This is attempted only once per transfer (`large_probe_attempted`), and only when:
- No prior backoff has occurred.
- Fast-lane did not fall back.
- 256 MB of stable progress has elapsed (`kLargeImageProbeDistance`).

### ACK timeouts

| Path | Timeout |
|---|---|
| `midst_data()` fallback | 120 s |
| `send_framed_packet_fast()` fast-lane | 120 s |
| `end_data()` — large transfer | 180 s |
| `end_data()` — default | 30 s |

The 120 s mid-transfer timeout absorbs eMMC erase-boundary stalls (large-block erase before the next write window).

---

## 10. Repartition

**Source:** `src/flash_workflow.cpp` — `init_flash_and_repartition()`, `build_repartition_table()`

When `ExecNandInit` returns `0x96` and flash mode is `Full`, `uflash` sends `BSL_CMD_REPARTITION` with a partition table built from the PAC XML.

### Partition table format

Two formats exist, selected automatically:

**Standard (76 bytes per entry):**
```
[NAME:72 bytes UTF-16LE] [SIZE:4 LE in MB]
```

**Extended (152 bytes per entry)** — used when any entry has a non-empty `id2` or non-zero `type`:
```
[NAME1:72 bytes UTF-16LE] [NAME2:72 bytes UTF-16LE] [SIZE:4 LE in MB] [TYPE:1] [PAD:3]
```

`uflash` detects which format to use by checking whether any parsed partition entry has `id2 != ""` or `type != 0`.

---

## 11. NV Backup

Before repartition, `uflash` reads and saves a copy of calibration and runtime NV partitions to a local directory (`nv_backup_<timestamp>/`). The partitions backed up are:

```
l_fixnv1, l_fixnv2, l_runtimenv1, l_runtimenv2, prodnv, miscdata
```

Each partition's size is looked up from the XML partition table (in MB). Partitions with size `0` or `0xFFFFFFFF` (variable/unbounded) are skipped.

If any single backup read fails, the remaining reads are abandoned, the USB endpoint state is drained and cleared, and repartition proceeds regardless — the goal is best-effort preservation, not blocking the flash.

`--skip-nv-backup` bypasses this entirely.

---

## 12. Firmware Dump

**Source:** `src/flash_workflow.cpp` — `run_dump_phase()`
**Source:** `src/bsl_protocol.cpp` — `read_partition()`

`--dump-firmware <dir>` reads every partition listed in the PAC XML using:

```
BSL_CMD_READ_START → [BSL_CMD_READ_MIDST × N] → BSL_CMD_READ_END
```

### READ_START payload (ReadFlash2)

```
[NAME:72 bytes UTF-16LE] [SIZE:4 LE]
```

For partitions ≥ 4 GiB (ReadFlash2_64):

```
[NAME:72 bytes UTF-16LE] [SIZE:8 big-endian] [RESERVED:8]
```

### READ_MIDST payload

```
[CHUNK_SIZE:4 LE] [OFFSET:4 LE]
```

FDL2 replies with response type `0x93` (REP_READ_FLASH) carrying the chunk data. The loop reads in 4 KB chunks until all bytes are received, then sends `READ_END`.

Partitions with XML size `0` or `> 1024 MB` are skipped.

---

## 13. PAC File Format

**Source:** `../upac` (sibling library)

A `.pac` file is a container holding:
- An XML configuration block describing all partitions, their types, base addresses, and file mappings.
- Binary blobs for each partition image (FDL1, FDL2, boot, system, super, etc.).

`upac::PacReader::open()` parses the container header, and `extract_all()` writes each blob to a temp directory (`/tmp/uflash_extract/`) so `uflash` can stream them file-by-file.

`upac::parse_xml_config()` parses the XML into `XmlProductConfig`, providing the ordered partition list (`XmlFileConfig` entries) with fields:
- `id` — internal identifier (e.g. `Super`, `FDL2`, `NV_WLTE`)
- `id_name` — block device name (e.g. `super`, `boot`, `l_fixnv1`)
- `type` — partition type string (e.g. `CODE2`, `YAFFS_IMG2`, `EXT4_IMG`)
- `base_address` — load/flash address (`0` for named eMMC partitions)
- `operations` — ordered list of BSL operations for this entry

---

## 14. Command Reference

| Value | Name | Description |
|---|---|---|
| `0x00` | `BSL_CMD_CONNECT` | Ping FDL; parses `DA_INFO_T` from ACK payload |
| `0x01` | `BSL_CMD_START_DATA` | Begin a download — address or named mode |
| `0x02` | `BSL_CMD_MIDST_DATA` | Send one data chunk |
| `0x03` | `BSL_CMD_END_DATA` | End download; FDL verifies checksum |
| `0x04` | `BSL_CMD_EXEC_DATA` | Execute at address (also triggers ExecNandInit in FDL2) |
| `0x05` | `BSL_CMD_NORMAL_RESET` | Soft-reset device to normal boot |
| `0x06` | `BSL_CMD_ERASE_FLASH` | Erase by address range |
| `0x09` | `BSL_CMD_CHANGE_BAUD` | Logical baud-rate step (USB: no-op on host) |
| `0x0B` | `BSL_CMD_REPARTITION` | Send partition table to FDL2 |
| `0x10` | `BSL_CMD_READ_START` | Begin partition read (ReadFlash2) |
| `0x11` | `BSL_CMD_READ_MIDST` | Request next chunk |
| `0x12` | `BSL_CMD_READ_END` | End partition read |
| `0x1A` | `BSL_CMD_READ_CHIP_UID` | Read chip UID |
| `0x21` | `BSL_CMD_DISABLE_TRANSCODE` | Switch to no-transcode framing |
| `0x7E` | `BSL_CMD_CHECK_BAUD` | Legacy 0x7E handshake byte |
| `0xAE` | `BSL_CMD_HOST_CONNECT` | Modern Host Protocol connect |

| Value | Name | Description |
|---|---|---|
| `0x80` | `BSL_REP_ACK` | Success |
| `0x81` | `BSL_REP_BSL_VER` | BSL version string |
| `0x82` | `BSL_REP_INVALID_CMD` | Unrecognised command |
| `0x83` | `BSL_REP_UNKWN_CHIP` | Unknown chip ID |
| `0x85` | `BSL_REP_VERIFY_SUCCESS` | Checksum verified OK |
| `0x8B` | `BSL_REP_VERIFY_ERROR` | Checksum mismatch |
| `0x93` | *(unnamed)* | Read data chunk response |
| `0x96` | *(unnamed)* | Partition layout mismatch (ExecNandInit / START_DATA) |
| `0xFE` | *(unnamed)* | Partition name not found in FDL2 table |
