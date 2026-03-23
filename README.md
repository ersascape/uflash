# uflash

`uflash` is a lightweight Unisoc flashing tool built against the local `upac` PAC parser.

It is focused on practical flashing workflows:
- load FDL1/FDL2 from a PAC
- initialize flash / optionally repartition
- flash PAC images by named partition or address
- preserve layout when desired
- flash a single partition for debugging

## Build

`uflash` depends on:
- local `../upac`
- `libusb-1.0`

Build with:

```bash
cmake -S . -B build
cmake --build build
```

Binary:

```bash
./build/uflash
```

## Common Usage

Full flash:

```bash
./build/uflash firmware.pac --full-flash
```

Preserve current partition layout and skip sensitive partitions:

```bash
./build/uflash firmware.pac --preserve-layout
```

Flash only one partition:

```bash
./build/uflash firmware.pac --partition GPS_GL
./build/uflash firmware.pac --partition gpsgl
```

Skip specific targets:

```bash
./build/uflash firmware.pac --skip NV_WLTE --skip GPS_BD
```

Tune the FDL1 -> FDL2 settle delay:

```bash
./build/uflash firmware.pac --fdl2-settle-ms 5000
```

Dump embedded PAC XML:

```bash
./build/uflash firmware.pac --dump-xml
```

## Notes

- `--preserve-layout` is the safer mode when you do not want an automatic repartition.
- NV-style partitions are not treated like normal raw images; some devices/FDLs require backup/merge logic.
- Named `CODE2`-style downloads now auto-pad tiny unaligned tails, which fixes end-of-file failures seen on images like `gnssmodem.bin`.
- `--partition` is useful for debugging problematic partitions without rerunning the full PAC sequence.
