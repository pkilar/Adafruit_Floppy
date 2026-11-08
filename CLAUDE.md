# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Adafruit_Floppy is an Arduino library for driving PC (Shugart) and Apple II floppy drives directly from SAMD51 and RP2040/RP2350 microcontrollers: raw flux capture/write, MFM/FM track codecs, an SdFat block device, Greaseweazle-compatible firmware, and read-only disk-image catalogs. Supported targets need >128KB SRAM and >100MHz so a full track of flux fits in RAM (README recommends SAMD51 @180MHz "Fastest", RP2040 @200MHz `-O3`, TinyUSB stack, Philhower core on RP2).

## Commands

### Host-side tests (no hardware) — run from `host_src/`

CI does **not** run these; run them locally when touching `src/mfm_impl.h`, `src/mfm_bpb.h`, or `src/Adafruit_FloppyImage.*`.

```sh
make check-files     # test_mfm_files + test_mfm_bpb against real-drive fixtures (cc only)
make check-layout    # encoded track layouts vs committed layout_fixtures/*.bits (binary in /tmp)
make check-catalog   # builds image_catalog.cpp + ../src/Adafruit_FloppyImage.cpp (g++ -Werror), runs test_image_catalog.py
make check-fm        # FM decoder + encoder sector_order/skip_iam round trips (speed skew, jitter, damage)
make check-atari-track  # Atari SD/ED/DD track codec: fit per RPM, inversion, interleave, density detect
make check-atari-gw  # Atari tracks cross-checked both ways against Greaseweazle's IBM codec (needs the submodule)
make check-atr       # ATR/XFD image backend over a memory file
make check-sio       # SIO drive protocol engine against a simulated Atari (virtual clock, baud-aware wire)
make check-atx       # ATX (copy-protected) images: parser, writer, 1050/810 timing model, weak bits
make check-atari-prot  # protected-track codec: scan every ID with its angular position, merge revolutions, master tracks
make                 # = check + checkfm: synthetic flux decode via main/main_fm + check_flux.py
```

- Each `check-*` target is one suite; for a single binary: `make test_mfm_bpb && ./test_mfm_bpb`.
- Plain `make` needs the Greaseweazle submodule (`make_flux.py` generates `test_flux.h` from it). Run `git submodule update --init` first, otherwise it fails in `host_src/greaseweazle`.
- C tests compile with `-iquote ../src -Wall -Werror`; assertions must stay enabled (`fixtures/README.md`).
- `test_scp.cpp` has no Makefile target.

### Firmware builds, formatting, docs (what CI checks)

CI (`.github/workflows/githubci.yml`) clones `adafruit/ci-arduino` into `ci/` and runs:

```sh
bash ci/actions_install.sh
python3 ci/build_platform.py feather_m4_express_tinyusb   # also: feather_rp2040_tinyusb, floppsy_rp2040_tinyusb, feather_rp2350_tinyusb
python3 ci/run-clang-format.py -e "ci/*" -e "bin/*" -e "*.pio.h" -r .
bash ci/doxy_gen_and_deploy.sh                            # doxygen over src/ (Doxyfile INPUT = src/)
```

- CI also clones `adafruit/SdFat` into the Arduino libraries dir; the library depends on the "SdFat - Adafruit Fork".
- Every example is compiled for every platform. ci-arduino marker files in an example folder control this: `.<platform>.test.only` restricts it (only `mfm_emu`), `.<platform>.generate` keeps the UF2 artifact (`greaseweazle`, `04_msd_test`, `mfm_emu`).
- The Doxyfile warns on undocumented members and parameters (`WARN_IF_UNDOCUMENTED`, `WARN_NO_PARAMDOC`). Document new public API in `src/` so the doxygen step stays warning-free.

### On-hardware bench tests — `extras/hw_tests/`

Floppsy RP2350 sketches (240MHz, TinyUSB). `floppsy_write_verify` is driven from the host by `bench.py` (needs `pyserial`); see its README. Writes require `--destructive` and a disposable 1.44MB HD disk; `--output` must be a new directory. `floppsy_board_test`, `floppsy_flush_failure` and `mfm_format_settings` are non-destructive.

## Architecture

**Entry point:** `src/Adafruit_Floppy.h` defines nearly everything.
- `Adafruit_FloppyBase` (abstract) owns the flux operations: `capture_track`, `write_track`, `erase_track`, `decode_track_mfm`, `encode_track_mfm`. Drive control is pure virtual.
- `Adafruit_Floppy` (Shugart/PC, 13 pins, `step()`) and `Adafruit_Apple2Floppy` (4 phase pins, quarter-track stepping) implement it.
- `Adafruit_MFM_Floppy` is *not* a FloppyBase. It is an SdFat `FsBlockDeviceInterface` wrapping an `Adafruit_Floppy*` so FatFs / USB MSC can mount the disk.

**Per-chip backends, selected by preprocessor (no vtable):**
- `arch_rp2.cpp`: `PICO_BOARD || __RP2040__ || ARDUINO_ARCH_RP2040`. PIO programs for flux read/write/Apple II write are hand-assembled `uint16_t` arrays in this file, sampling at 24MHz. The write path subtracts `OVERHEAD=20` ticks per pulse to compensate for PIO overhead.
- `arch_samd51.cpp`: `__SAMD51__`. TC timer ISRs, sampling at 48MHz/`g_timing_div` (default 24MHz).
- Anything else falls back to GPIO bit-banging in `Adafruit_Floppy.cpp`, which requires `index_wait_ms`.
- Each backend provides `init_capture` / `start_polled_capture` / `disable_capture` (SAMD51 also has the `*_generate` write family).
- The timing-critical paths are `optimize("O3")` and run with interrupts disabled. Don't add work inside them.

**Read path:** `capture_track` fills a byte-per-flux-interval buffer (optionally Greaseweazle-packed via `greasepack.h`). `decode_track_mfm` then turns it into sectors plus a per-sector validity array. `Adafruit_MFM_Floppy::readTrack` caches one whole track (up to 18×512) and retries up to 5 times.

**Write path:** `writeSector` only updates the track cache and marks it dirty (write-back). `syncDevice` encodes the track and writes it. It refuses if any sector in the track is invalid or the disk is write-protected. A failed flush keeps the data dirty, and `readTrack` flushes first and aborts on failure.

**Shared plain-C codecs:** `src/mfm_impl.h` (header-only, all `static`) and `src/mfm_bpb.h` (boot-sector parsing) are compiled both into the Arduino build and by `host_src` tests. In the Arduino build, `mfm_impl.h` is included only from `Adafruit_Floppy.cpp`. Keep them plain C with libc only; use the `DEBUG_PRINTF` / `DEBUG_ASSERT` hooks, never Arduino APIs.

**Formats:** `_format_info` / `flux_rates` in `Adafruit_MFM_Floppy.cpp` (bit cells of 2000/1667/1000/833ns) must stay in the same order as the `adafruit_floppy_disk_t` enum; a `static_assert` checks the length. `AUTODETECT` parses the BPB on `inserted()`, tries each flux rate, and sets double-stepping for 40-track media in 80-track drives.

**Greaseweazle parity:** IBM MFM/FM track layouts (gaps, IAM) deliberately match Greaseweazle's codec, and `layout_fixtures/*.bits` were produced from Greaseweazle commit `26690f8`. To regenerate them, run `host_src/make_layout_fixtures.py` with Greaseweazle's `src` on `PYTHONPATH`. When editing real-drive fixtures in `host_src/fixtures/`, update the SHA-256s in `manifest.json`. `.flux24` is one byte per flux interval at 24MHz, not SCP or KryoFlux.

**Other modules:**
- `Adafruit_Floppsy`: board helper for the Floppsy RP2350. It owns `drive0`/`drive1`, the level-shifter OE/DIR lines, and an INA3221 rail monitor. The header `#error`s unless `ARDUINO_ADAFRUIT_FLOPPSY_RP2350` is defined. `begin()` leaves the cable isolated; `enable(true)` connects it.
- `Adafruit_FloppyFluxCapture` (RP2350 only): 25ns timestamps into PSRAM using two synchronized PIO state machines plus DMA, at 120 or 240MHz. Its PIO comes from `src/flux_capture.pio`; regenerate `flux_capture.pio.h` with pioasm rather than editing it (clang-format excludes `*.pio.h`).
- `Adafruit_FloppySCP`: converts those timestamps to SCP files.
- `Adafruit_FloppyImage`: read-only filesystem identification and root-directory listing over a caller-supplied byte reader. It covers FAT12 (PC/Atari ST), Amiga ADF, C64 D64, Apple DOS 3.3 and ProDOS. It never touches hardware and is host-tested by `check-catalog`.

`examples/greaseweazle` is the Greaseweazle-compatible USB firmware, with per-board pin `#if` blocks. `examples/mfm_emu` emulates a drive (Xerox 820 by default).

### Atari 8-bit support (SIO drive emulation + physical Atari disks)

`examples/atari_sio_drive` makes the board act as Atari drives D1–D4 on the SIO bus, backed by ATR/XFD/ATX images on SD or by a real Atari floppy in a PC drive. Its README covers wiring and the console commands. The layers:

- **`atari_disk.h`** (plain C): geometry (128/256/512-byte sectors, 1 or 2 sides, 40 or 80 tracks, or one-track "hard disk" images up to 65535 sectors), the size → geometry table (`atari_geometry_for_sectors`), logical sector ↔ cylinder/head/slot and the WD status bits.
  - On 256-byte-sector disks, sectors 1–3 are 256 bytes on disk but 128 bytes over SIO and in ATRs (`atari_sector_bytes`). With 512-byte sectors (SpartaDOS X) every sector is full size.
  - Double-sided disks follow the XF551 order: side 1 is walked backwards, so sector 721 is track 39 ID 18 and sector 1440 is track 0 ID 1.
- **`Adafruit_AtariDisk`**: the abstract disk the SIO engine talks to. `ready()`, `geometry()` and `writeProtected()` must never block, because the engine calls them before its ACK and the Atari expects the ACK within about 16 ms.
  - The engine reads through `readSectorTimed()`, which lets a disk report the FDC status, send a bad sector's data with the ERROR response, and delay the answer the way drive mechanics would (ATX timing).
- **`Adafruit_AtariSIO`** (pure C++; transport is `Adafruit_AtariSIOPort`, Arduino adaptor `Adafruit_AtariSIOUart.h`): commands R/W/P/S/!/"/N/O/?, US Doubler Ultra Speed (divisor via `?`, autobaud by toggling rate on a bad command frame) and XF551 high speed (command | 0x80). All timing constants are at the top of the .cpp.
- **`Adafruit_AtariATR`** over `Adafruit_AtariFile` (SdFat adaptor in `Adafruit_AtariSdFile.h`; host tests use a memory file). Handles ATR, XFD, the full and short double-density layouts and 512-byte sectors. Formatting with a new PERCOM geometry resizes the image.
- **Copy protection:**
  - **`atari_atx.h`** (plain C): the ATX layout and `atari_prot_track_t`, a protected track as a list of sector entries with angular position (26042 units per revolution from the index), ATX status bits and data. All three protection modules exchange this type.
  - **`Adafruit_AtariATX`**: read-only ATX disk. It picks the duplicate sector nearest ahead of a virtual 288 RPM head, adds seek, rotation and retry delays (1050 or 810 constants), randomizes weak bytes and reports Altirra's status values. One static track cache is shared by all instances. `Adafruit_AtariATXWriter` writes ATX files.
  - **`atari_prot_track.h`** (plain C over `atari_track.h`): scans every sector ID of a captured revolution with its position, merges several revolutions (bytes that differ become weak), and encodes a protected track for mastering. Weak-bit mastering writes deliberately ambiguous flux and is untested on hardware.
- **`atari_track.h`** (plain C over `mfm_impl.h`) and **`Adafruit_AtariFloppy`**:
  - Track layouts: SD = FM 18×128, ED = MFM 26×128, DD = MFM 18×256, no index address mark. Disks can be single or double sided (side byte = head; side 1 reads also accept 0) with 40 or 80 tracks; an 80-track drive detects 40-track media from the ID cylinder numbers and double-steps.
  - Sector data is stored one's-complemented on disk; IDs, address marks and CRCs are not (US Doubler/1050 ROM, inherited from the 810's inverted FD1771 bus).
  - Cells are scaled by the measured revolution time, so a 300 or 360 RPM PC drive writes the same physical layout as a 288 RPM Atari drive.
  - Writes rewrite a whole track and refuse tracks with unreadable sectors.
  - `idle()` stops the motor and marks the disk for lazy re-detection, since PC drives have no usable disk-change line.
- Don't include `mfm_impl.h` directly where `atari_track.h` or `atari_prot_track.h` is included: `mfm_impl.h` has no include guard.
