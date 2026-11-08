# Atari SIO drive

Turns a supported board into Atari 8-bit disk drives D1: to D15: on the SIO bus
(SIO device IDs $31 to $3F, SpartaDOS X drives A to O). Four drives are emulated
by default; `units` changes that. Each unit is virtual (an ATR/XFD/ATX image
from a microSD card), physical (a real Atari floppy in the PC (Shugart) drive on
the board) or bypass (left to another drive on the SIO chain). Everything is
configured over the USB serial console at 115200 baud.

## Drive modes

Each unit has a mode, set with `mode` (or `mount <n> floppy`, the config file or
the menu):

* **virtual**: an emulated drive served from an image on the SD card. With no
  image mounted it stays silent, like a switched-off drive, so an Atari with
  nothing in D1: goes on to BASIC instead of repeating BOOT ERROR.
* **physical**: served by the floppy drive on the board. There is one, so only
  one unit can be physical; setting a second one fails and names the unit that
  has it. With no disk inserted the unit answers "not ready".
* **bypass**: the board never answers this ID, so another drive on the SIO chain
  with that number works. A bypassed unit refuses mounts, so it cannot shadow
  the real drive by accident.

Units beyond `units` behave like bypass. Lowering `units` detaches the units
beyond the new count and resets them to virtual. If a high-speed drive on the
chain uses a different speed than `speed`, the board may need one retry to
re-sync its autobaud after that drive's traffic.

**Status: not tested on a real Atari.** It was validated only by host-side
simulation of the SIO protocol and by compile checks on the CI platforms.
Expect to debug wiring and timing.

## Supported boards

| Board | Floppy pins | SIO UART | COMMAND | microSD |
|---|---|---|---|---|
| Feather M4 Express | as `greaseweazle` | Serial1 (TX/RX pins) | SDA | SPI breakout, CS = SCL |
| Feather RP2040 / RP2350 HSTX | as `greaseweazle` | Serial1 (TX/RX pins) | D4 (GPIO6) | SPI breakout, CS = SCL |
| Floppsy RP2040 | built in | UART0 on GPIO16 (TX) / GPIO17 (RX), the STEMMA QT pins | GPIO26 (SENSE1 JST) | on-board, CS = `PIN_CARD_CS` |
| Floppsy RP2350 | built in | **unverified**, copied from the RP2040 | **unverified** | SDIO |
| Custom RP2350B board (below) | IDC pins, see below | UART0 on GPIO16 (TX) / GPIO17 (RX) | GPIO26 | SDIO |

The Floppsy RP2350 board package was not released when this was written; its
SIO pins are guesses and `setup()` reports an error if the UART rejects them.
All pins are in one block at the top of the sketch. Select the Adafruit TinyUSB
stack under Tools > USB Stack.

## Custom RP2350B board (OLED and encoder)

For the `floppy-rp2350b-pcb` board: RP2350B, microSD on SDIO, a 1.3" SH1106
OLED and a rotary encoder for a local user interface. Pins (all in the sketch
and `ui_config.h`):

| Function | GPIO |
|---|---|
| Floppy DENSITY, INDEX, SELECT, MOTOR, DIR, STEP | 8, 2, 9, 10, 11, 12 |
| Floppy WRDATA, WRGATE, TRK0, PROT, READ, SIDE, READY | 13, 14, 4, 5, 3, 15, 6 |
| SIO UART0 TX / RX, COMMAND | 16 / 17, 26 |
| microSD SDIO CLK, CMD, DAT0 (DAT1-3 = 37-39), card detect | 34, 35, 36, 40 |
| OLED SDA / SCL (I2C0, address 0x3C) | 28 / 29 |
| Encoder A, B, switch (active low, external pull-ups) | 22, 23, 24 |
| VBUS sense (ADC, 100k/27k), PD power good, status LED | 41, 43, 21 |
| PSRAM chip select (QMI CS1) | 47 |

**Selecting the board.** In the Arduino IDE choose Tools > Board > Raspberry Pi
Pico/RP2040/RP2350 > **Generic RP2350**, then set Variant chip = RP2350B,
PSRAM CS = GPIO 47, Flash size to match the board's W25Q128 (16MB), USB Stack =
Adafruit TinyUSB. That board defines `ARDUINO_GENERIC_RP2350`, which selects
the pin block automatically. With another board package, build with
`-DFLOPPY_SIO_RP2350B_REVA`. The FQBN used for the compile checks is
`rp2040:rp2040:generic_rp2350:variantchip=RP2350B,psramcs=GPIO47,flash=16777216_0,freq=150,arch=arm,dbgport=Disabled,dbglvl=None,usbstack=tinyusb`.
The user interface needs the Adafruit SH110X library (and Adafruit GFX).
The status LED (GPIO21) blinks on SIO traffic; the SD card is started before
anything claims a PIO, because SdFat's PIO SDIO needs to move the PIO's GPIO
base to 16 and the floppy code only uses PIOs with base 0.

**Display options.** The board has an I2C OLED header (SDA 28, SCL 29, 0x3C) and a
7-pin SPI OLED socket (SPI1: SCK 30, MOSI 31, CS 32 in software, DC 33,
RES 27). Choose one with `OLED_TYPE` in `ui_config.h` (or build with
`-DOLED_TYPE=n`): `OLED_SH1106_I2C` (1, default), `OLED_SH1106_SPI` (2),
`OLED_SSD1306_SPI` (3), `OLED_SSD1309_SPI` (4, 2.42" modules). SPI runs at
8 MHz on hardware SPI1. SSD1309 is driven by Adafruit_SSD1306, whose command
set it shares; this was not tried on a real module. Needs the Adafruit SH110X
and Adafruit SSD1306 libraries.

**User interface.** Other boards have no UI and behave as before. It runs on
core 1, so the SIO engine on core 0 is not delayed by the display. Core 1 never
touches the SD card or the mounts: it sends requests (list a folder, mount,
eject, save) that core 0 serves between SIO polls (folders a few entries per
pass), and reads a status snapshot. Mounting from the UI is the same code as the
`mount` console command, so the same rules apply; mounting the floppy spins the
drive up and can take a few seconds, during which the Atari is not served.

| Control | Action |
|---|---|
| Turn | scroll |
| Press | select; on the status screen, open the menu |
| Hold (0.6 s) | back: leaves the menu, browser or file menu, one level at a time |

Screens:

1. **Status**: four units at a time with the file name (`ATX` marks protected
   images), `FLOPPY`, `bypass` or `--` (empty virtual unit); turn the knob to
   scroll when more are emulated (`^` and `v` at the right edge show there are
   more). Below: the last SIO command (`>` while active) and, when the floppy
   is mounted, its type (SD/ED/DD, SS/DS, tracks), current track and side, and
   MOTOR / BUSY. The last line shows the PD voltage (VBUS sense).
2. **Menu**: Browse SD card, one item per unit (`D1 SDX.ATR`, `D2 FLOPPY`,
   `D3 bypass`, `D4 --`), `Drives: N`, Save settings (writes `/ATARISIO.CFG`
   like `save`), Back.
   * A **unit screen** opens for a unit: Virtual, Physical (floppy), Bypass,
     Eject, Back. `*` marks the current mode. Hold to go back.
   * The **Drives screen** changes the number of drives: turn for 1 to 15,
     press to apply, hold to cancel.
3. **Browser**: folders and `.atr`/`.xfd`/`.atx` files, `..` to go up. Names
   starting with `.` or longer than 62 characters are not listed, and a folder
   shows at most 160 entries.
4. **File menu**: Mount on each unit that is not set to bypass (`*` = unit in
   use, it is replaced), Cancel.

While core 0 is inside a long command (`dump`, `write`, `format`, floppy
mount) the status screen and the waiting screen show "Busy: ..."; hold the
button to leave the waiting screen (the request is withdrawn, or its late
result is ignored). A `+` at the end of the browser title means the folder has
more than 160 entries.

Errors such as "Cannot open ..." appear as a message for a few seconds or until
a press. Menus return to the status screen after a minute without input.

## Building with PlatformIO

`platformio.ini` in this folder builds the same firmware without the Arduino
IDE: run `pio run` here (the first build downloads the platforms and
libraries). The library itself is this repository (`symlink://../..`), the
other libraries are pinned registry versions, and the RP2 environments use
Earle Philhower's core through `maxgerhardt/platform-raspberrypi` pinned to a
commit. The TinyUSB stack is selected with `-DUSE_TINYUSB`. PlatformIO builds
`pio/main.cpp`, which includes the sketch and `ui.cpp`, because its own
`.ino` conversion mishandles the board `#if` chain. Arduino ignores that folder.

| Environment | Board |
|---|---|
| `floppy_sio_rp2350b` (default) | custom RP2350B board, SH1106 on I2C (`OLED_TYPE=1`), 150 MHz, 16 MB flash, PSRAM CS GPIO47 |
| `floppy_sio_rp2350b_sh1106_spi` / `_ssd1306_spi` / `_ssd1309_spi` | the same with an SPI display (`OLED_TYPE` 2, 3, 4) |
| `feather_rp2040`, `floppsy_rp2040`, `feather_rp2350` | Adafruit boards, 120 MHz, no UI |
| `feather_m4_express` | Feather M4 Express, 120 MHz (uses the core's built-in TinyUSB) |

The Floppsy RP2350 has no PlatformIO board definition, so it is not included.
The custom board uses the stock `generic_rp2350` board with
`-D__PICO_RP2350A=0` (RP2350B) and `-DRP2350_PSRAM_CS=47`.

```
pio run -e floppy_sio_rp2350b                 # build
pio run -e floppy_sio_rp2350b -t upload       # upload
pio device monitor -e floppy_sio_rp2350b      # serial console, 115200 baud
```

RP2 boards upload with picotool: hold BOOTSEL while plugging in the USB cable
(or let PlatformIO reset a running firmware with the 1200 bps touch), then run
the upload. The M4 uses the SAM-BA bootloader (double-tap reset).

## SIO wiring

Atari SIO pins used: 3 DATA IN, 4 GND, 5 DATA OUT, 7 COMMAND. The names are the
Atari's point of view, so the board **receives** on DATA OUT and **transmits**
on DATA IN.

| Atari SIO | Board |
|---|---|
| 5 DATA OUT | UART RX, through a 5 V to 3.3 V level shifter |
| 7 COMMAND | COMMAND pin, through a 5 V to 3.3 V level shifter |
| 3 DATA IN | UART TX, through a 1N4148 diode (below) |
| 4 GND | GND |

The Atari signals are 5 V and these MCUs are not 5 V tolerant. From AtariSIO's
`README.RPi` (the Raspberry Pi wiring):

* DATA OUT and COMMAND need level shifting down to 3.3 V. A resistor divider or
  a down-shifting FET shifter works; shifting up is not needed.
* DATA IN is a shared bus line. Put a 1N4148 in series with the board's TX, anode
  to the SIO pin and cathode to the board, so the board can only pull the line
  low and the Atari's pull-up makes the high level. Without it a 3.3 V push-pull
  output fights the other SIO devices. An open-drain buffer that is high-Z when
  not transmitting does the same job. This is only optional if the board is the
  one and only device on the bus.

Not taken from that documentation (general practice, check it yourself): the
divider values (for example 2.2k/3.3k for DATA OUT and COMMAND), and checking
the DATA IN waveform at the Ultra Speed rate, since the diode slows the rising
edge. The Atari's +5 V on SIO pin 10 is not used.

## Floppy drive

* Best: a 5.25" 40-track 360K drive (300 RPM), which is what Atari disks were
  written for. It needs 12 V as well as 5 V.
* A 5.25" 1.2MB drive turns at 360 RPM and has 80 tracks: set `DRIVE_RPM` to 360
  and `DRIVE_TRACKS` to 80 at the top of the sketch. The library drives it at
  its low density setting and tells 40-track from 80-track media by the
  cylinder numbers in the sector IDs of physical track 2, stepping twice per
  track for 40-track disks.
* 3.5" drives cannot read 5.25" Atari disks. 80-track double-sided disks
  written by 3.5"-modified drives may work with `DRIVE_TRACKS` 80; this is
  untested, and the side-1 order is assumed to follow the XF551.
* Double-sided XF551 disks need a drive with a side-select line, which every PC
  drive has. Side 1 of an XF551 disk is read as logical sectors 721 to 1440,
  walking the tracks backwards (sector 721 is track 39, 1440 is track 0).

The measured spindle speed is printed when the floppy is mounted.

## Formats

| Format | Sectors | Image (ATR/XFD) | Physical floppy |
|---|---|---|---|
| SD, FM, 18 x 128, 40 tracks | 720 (90K) | yes | yes |
| ED, MFM, 26 x 128, 40 tracks | 1040 (130K) | yes | yes |
| DD, MFM, 18 x 256, 40 tracks | 720 (180K) | yes | yes |
| DS DD (XF551), 2 x 40 tracks | 1440 (360K) | yes | yes |
| DS DD, 80 tracks | 2880 (720K) | yes | 80-track drive only |
| 512-byte sectors, 720 / 1440 / 2880 sectors | SpartaDOS X | yes | no |
| 128, 256 or 512 bytes, up to 65535 sectors | SpartaDOS X "hard disk" | yes | no |
| ATX (protected, 40 tracks, one side) | any | read-only | dump and master |

Format tokens, used by `new` and `format`: `[ds](sd|ed|dd)[80]`, for example
`sd`, `dd`, `dsdd` (XF551 360K), `dsdd80` (720K) and `dd80`. `new` also takes
`<count>x<128|256|512>`, for example `65535x512` for a 32 MB SpartaDOS X image.
The resulting geometry is printed.

## Console commands

| Command | What it does |
|---|---|
| `help` | List the commands |
| `ls [dir]` | List folders and `.atr` / `.xfd` / `.atx` files with sizes |
| `units [1-15]` | Show or set the number of drives (default 4). Lowering it detaches the units beyond it and resets them to virtual |
| `mode <1-15> [virtual\|physical\|bypass]` | Show or set a unit's mode (`floppy` also means physical). `virtual` ejects the floppy of a physical unit or turns a bypassed unit into an empty virtual one, and keeps the image of a virtual unit; `physical` fails if another unit has the floppy; `bypass` detaches whatever is mounted |
| `mount <unit> <path>\|floppy` | Attach an image (read-write if possible, otherwise read-only; `.atx` files and files starting with `AT8X` are ATX and always read-only) or the physical floppy, which spins up and prints the detected density and measured RPM. Refused for bypassed units and units beyond `units`. Units are written `1` to `15`, `D1` to `D15` or `D1:` |
| `umount <unit>` | Detach a unit; it stays virtual and empty (a physical unit ejects the floppy) |
| `status` | Per unit up to `units`: mode, source, "ATX" for protected images, geometry (sector size, tracks, sides), write protection; ATX drive model; SIO statistics; Ultra Speed setting |
| `new <path> <fmt>` | Create a blank ATR (`<fmt>` as in Formats); refuses to overwrite |
| `dump <path>` | `.atr`: copy the floppy to a new ATR with the detected geometry (both sides, 40 or 80 tracks). Prints progress per track side and the unreadable sectors, which are stored as zeros. `.atx`: a protected dump of a single-sided 40-track floppy; per track it prints the entry count and the duplicate, CRC, missing, deleted and weak counts. Refused for double-sided or 80-track media |
| `write <path> yes` | `.atr`: copy an image onto the floppy, one verified track side at a time; the drive must be able to format that geometry. `.atx`: master the protected image (see Copy protection); prints entries, the largest push in angular units and failures per track. Needs the literal `yes` |
| `format <fmt> yes` | Format the floppy; refused for geometries the drive cannot do (80 tracks need an 80-track drive). Needs the literal `yes` |
| `drive 810\|1050` | Drive model whose timing ATX mounts use (default 1050) |
| `speed <divisor>\|off` | Ultra Speed POKEY divisor (default 10, about 38400 baud) |
| `save` | Write the drive count, modes and mounts to `/ATARISIO.CFG` |

`mount` and the config file take the rest of the line as the path, so those
paths may contain spaces (no quoting; leading and trailing blanks are dropped).
The other commands take a single word as the path. `dump`, `write` and `format` block for seconds to
minutes; the floppy is taken off the SIO bus meanwhile and the Atari cannot use
the drives until they finish (a floppy mounted on a unit is reattached
afterwards).

## Config file

`/ATARISIO.CFG` on the SD card is read at startup and written by `save`. One
setting per line, `#` starts a comment line. `units N` sets the drive count
(wherever it appears; the default is 4). A `Dn` line sets a unit: `FLOPPY` (or
`PHYSICAL`), `BYPASS`, `VIRTUAL` (empty), or the path of an image (the rest of
the line, which may contain spaces). Lines for units beyond `units` are ignored
with a message. Files with only `D1` to `D4` lines still work.

```
# SpartaDOS X on D1, the real floppy on D2, a real 1050 as D3
units 6
D1 /DOS/SDX.ATR
D2 FLOPPY
D3 BYPASS
D5 /games/mule.atr
```

## Copy protection

ATX images keep what a sector copy loses: several sectors with the same number,
data CRC errors, sectors whose ID has no data field, deleted data marks, weak
(read differently every time) sectors and their rotational positions. A mounted
ATX is served with a timing model of an Atari 1050 (WD2793) or 810 (FD1771),
chosen with `drive`:

* The sector read is the next ID of that number the head reaches from its
  current angular position, so duplicate sectors come back in the order and
  with the delays a real drive gives, and a missing sector takes the drive's
  full retry time to fail.
* CRC error, missing and deleted status is reported as the controller does
  (the SIO engine sends the data frame with the error status when there is
  data).
* Weak sectors return different bytes from the weak offset on at each read.
* ATX mounts are read-only. Writes are refused.

`dump <path>.atx` reads a real protected floppy: each track is captured about
five times, every sector ID is scanned (so duplicates, CRC errors, missing and
deleted sectors are found) and the revolutions are compared to find weak
sectors. Tracks with nothing readable are stored as empty.

`write <path>.atx yes` masters an ATX onto a disk, track by track. This is
experimental: each track is encoded with every entry at its recorded angular
position (pushed later if sectors would overlap, which is reported; if a
sector would straddle the index the whole track is rotated so the index falls
in a gap, which Atari drives cannot notice), an
unformatted track is erased, and the track is read back and compared. Weak
sectors are written as flux out of specification so that they read differently
each time; this has not been tried on a real drive. Whether a mastered disk
passes its protection check depends on the drive's timing and is unproven.

A real protected floppy mounted live (`mount 1 floppy`) also serves simple
bad-sector protections: when a sector cannot be read cleanly but its ID is on
the disk with a data field that has a bad CRC, the SIO engine sends that data
with the CRC error status instead of reporting the sector as missing. Timing
tricks, duplicate sectors and weak bits need an ATX dump.

## Limitations

* Not tested on a real Atari or with real protected disks. The 1050/810 timing
  constants are approximate.
* Physical disks: SD, ED and DD (128/256-byte sectors, 40 tracks, or 80 tracks
  on an 80-track drive), one or two sides. 512-byte and large SpartaDOS X
  images work as image files only.
* ATX is single-sided, 40 tracks. Sectors with a CRC error in the ID field
  cannot be represented. Long sectors keep only the first 128/256 bytes.
* Weak-bit mastering is untested on hardware.
* The side byte of the sector IDs on side 1 of an XF551 disk is not known for
  certain, so reads accept 0 or 1.
* Writing to the physical floppy rewrites whole tracks and is refused for tracks
  with unreadable sectors: `format` the disk first.
* Ultra Speed and XF551 high-speed commands are supported. XF551 mode changes
  the UART speed between the ACK and the Atari's data frame; that switch has
  not been timed on real hardware yet.
* PC drives have no usable disk-change signal. A swap is noticed once the drive
  has been idle for 3 seconds (motor off); until the next read, `status` still
  reports the previous disk's density. Don't swap disks while the drive is busy.
* A live floppy does not report the "deleted" status of a sector (it is read as
  normal data); only an ATX dump does.
