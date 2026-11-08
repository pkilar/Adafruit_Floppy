// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// Atari 8-bit disk drives D1: to D4: on the SIO bus. Each unit serves either an
// ATR/XFD/ATX image from the microSD card or a real Atari floppy in a PC drive.
// Configure everything over the USB serial console (type "help").
//
// Validated only by host-side simulation and compile checks; not yet tried on a
// real Atari. See README.md for wiring.
#include <Adafruit_AtariATR.h>
#include <Adafruit_AtariATX.h>
#include <Adafruit_AtariFloppy.h>
#include <Adafruit_AtariSIO.h>
#include <Adafruit_AtariSIOUart.h>
#include <Adafruit_AtariSdFile.h>
#include <Adafruit_Floppy.h>
#include <Adafruit_TinyUSB.h>
#include <SdFat.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

// Speed of the floppy drive's spindle. A 5.25" 360K drive turns at 300 RPM and
// is what Atari disks were written for. 1.2MB drives turn at 360 RPM and have
// 80 tracks: set DRIVE_RPM to 360 and DRIVE_TRACKS to 80 (3.5" drives are
// 300 RPM, 80 tracks). The library then detects 40- or 80-track media itself.
const uint16_t DRIVE_RPM = 300;
const uint8_t DRIVE_TRACKS = 40;

// Mounts are loaded from this file at startup, and written by "save".
const char CONFIG_PATH[] = "/ATARISIO.CFG";

// ---------------------------------------------------------------------------
// Board configuration: floppy pins, SIO pins and the SD card. Change the pins
// here if you wire things differently.
//
//   SIO_COMMAND_PIN  SIO COMMAND (pin 7), input, active low
//   Serial1 RX       SIO DATA OUT (pin 5)
//   Serial1 TX       SIO DATA IN (pin 3), through the diode described in the
//                    README
// ---------------------------------------------------------------------------
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS)
#define DENSITY_PIN A1 // IDC 2
#define INDEX_PIN A5   // IDC 8
#define SELECT_PIN A0  // IDC 12
#define MOTOR_PIN A2   // IDC 16
#define DIR_PIN A3     // IDC 18
#define STEP_PIN A4    // IDC 20
#define WRDATA_PIN 13  // IDC 22
#define WRGATE_PIN 12  // IDC 24
#define TRK0_PIN 10    // IDC 26
#define PROT_PIN 11    // IDC 28
#define READ_PIN 9     // IDC 30
#define SIDE_PIN 6     // IDC 32
#define READY_PIN 5    // IDC 34
// Serial1 is on the TX and RX pins already.
#define SIO_COMMAND_PIN SDA
#define SD_CS_PIN SCL // SPI microSD breakout
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) ||                              \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2350_HSTX)
#define DENSITY_PIN A1    // IDC 2
#define INDEX_PIN 25      // IDC 8
#define SELECT_PIN A0     // IDC 12
#define MOTOR_PIN A2      // IDC 16
#define DIR_PIN A3        // IDC 18
#define STEP_PIN 24       // IDC 20
#define WRDATA_PIN 13     // IDC 22
#define WRGATE_PIN 12     // IDC 24
#define TRK0_PIN 10       // IDC 26
#define PROT_PIN 11       // IDC 28
#define READ_PIN 9        // IDC 30
#define SIDE_PIN 8        // IDC 32
#define READY_PIN 7       // IDC 34
// Serial1 is on the TX and RX pins already.
#define SIO_COMMAND_PIN 6 // D4 = GPIO6
#define SD_CS_PIN SCL     // SPI microSD breakout
#elif defined(ARDUINO_ADAFRUIT_FLOPPSY_RP2040)
// The floppy pins (DENSITY_PIN ... READY_PIN) come from the board package.
#define SIO_UART_TX_PIN 16 // UART0 on the STEMMA QT connector pins
#define SIO_UART_RX_PIN 17
#define SIO_COMMAND_PIN 26 // SENSE1 JST connector
#define SD_CS_PIN PIN_CARD_CS
#elif defined(ARDUINO_ADAFRUIT_FLOPPSY_RP2350)
#include <Adafruit_Floppsy.h>
// !!! UNVERIFIED !!! The Floppsy RP2350 board package is not released yet, so
// these SIO pins are copied from the RP2040 Floppsy and have not been checked
// against the real board. setup() reports an error if the UART refuses them;
// a wrong COMMAND pin fails silently. Verify against the schematic.
#define SIO_UART_TX_PIN 16
#define SIO_UART_RX_PIN 17
#define SIO_COMMAND_PIN 26
// The microSD card is on SDIO.
#else
#error "Please set up pin definitions!"
#endif

#if defined(ARDUINO_ADAFRUIT_FLOPPSY_RP2350)
Adafruit_Floppsy floppsy;
#define FLOPPY_DRIVE (&floppsy.drive1)
#define SD_CONFIG SdioConfig(PIN_SD_CLK, PIN_SD_CMD_MOSI, PIN_SD_DAT0_MISO, 3.0)
#else
#if defined(READ_PIN)
Adafruit_Floppy pcfloppy(DENSITY_PIN, INDEX_PIN, SELECT_PIN, MOTOR_PIN, DIR_PIN,
                         STEP_PIN, WRDATA_PIN, WRGATE_PIN, TRK0_PIN, PROT_PIN,
                         READ_PIN, SIDE_PIN, READY_PIN);
#define FLOPPY_DRIVE (&pcfloppy)
#endif
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12))
#endif

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

const uint8_t UNITS = 4;
const size_t PATH_LEN = 96;
const size_t LINE_LEN = 128;
const uint8_t DEFAULT_SPEED_DIVISOR = 10;

enum Source : uint8_t { SRC_NONE, SRC_IMAGE, SRC_ATX, SRC_FLOPPY };

// Everything one image needs; the FsFile and its adaptor must stay put while
// the ATR is mounted, so these are globals and never copied.
struct Mount {
  FsFile file;
  Adafruit_AtariSdFile adaptor;
  Adafruit_AtariATR atr;
  Adafruit_AtariATX atx; // protected images; read-only
  Source source;
  char path[PATH_LEN];
  Mount() : adaptor(file), source(SRC_NONE) { path[0] = 0; }
};

SdFs sd;
bool sdReady = false;
Adafruit_AtariFloppy atariFloppy(FLOPPY_DRIVE); // one ~66 KB flux buffer
Adafruit_AtariSIOUart sioPort(Serial1, SIO_COMMAND_PIN);
Adafruit_AtariSIO sio(&sioPort);
Mount mounts[UNITS];
Mount copyImage;              // scratch image for new / dump / write
atari_prot_track_t protTrack; // scratch for protected dump / master
atari_drive_model_t driveModel = ATARI_DRIVE_1050; // timing of ATX mounts
uint8_t speedDivisor = DEFAULT_SPEED_DIVISOR;

char line[LINE_LEN];
size_t lineLen = 0;
bool lineOverflow = false;

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const char *resultName(atari_disk_result_t r) {
  switch (r) {
  case ATARI_DISK_OK:
    return "ok";
  case ATARI_DISK_NOT_READY:
    return "not ready";
  case ATARI_DISK_WRITE_PROTECTED:
    return "write protected";
  case ATARI_DISK_NOT_FOUND:
    return "sector not found";
  case ATARI_DISK_CRC_ERROR:
    return "CRC error";
  case ATARI_DISK_UNSUPPORTED:
    return "unsupported";
  default:
    return "I/O error";
  }
}

static void printGeometry(const atari_geometry_t &g) {
  static const char *const names[] = {"SD", "ED", "DD"};
  atari_density_t d = atari_track_density(&g);
  if (d != ATARI_DENSITY_OTHER && g.tracks >= 40)
    out("%s ", names[d]);
  out("%u sectors x %u bytes", (unsigned)g.sector_count,
      (unsigned)g.sector_size);
  if (g.tracks > 1)
    out(", %u tracks, %s", (unsigned)g.tracks,
        g.sides == 2 ? "double sided" : "single sided");
}

static void printRpm() {
  uint32_t us = atariFloppy.revolutionMicros();
  if (us)
    out("measured %lu RPM\n", (unsigned long)(60000000UL / us));
}

static bool parseDensity(const char *s, atari_density_t *d) {
  if (!strcasecmp(s, "sd"))
    *d = ATARI_DENSITY_SD;
  else if (!strcasecmp(s, "ed"))
    *d = ATARI_DENSITY_ED;
  else if (!strcasecmp(s, "dd"))
    *d = ATARI_DENSITY_DD;
  else
    return false;
  return true;
}

// A format token: [ds](sd|ed|dd)[80], e.g. sd, dd, dsdd (XF551 360K), dd80.
static bool parseFormat(const char *s, atari_geometry_t *g) {
  uint8_t sides = 1;
  if (!strncasecmp(s, "ds", 2)) {
    sides = 2;
    s += 2;
  }
  char dens[3] = {s[0], s[0] ? s[1] : (char)0, 0};
  atari_density_t d;
  if (strlen(s) < 2 || !parseDensity(dens, &d))
    return false;
  s += 2;
  uint8_t tracks = 40;
  if (!strcmp(s, "80"))
    tracks = 80;
  else if (*s)
    return false;
  *g = atari_geometry_make(d, tracks, sides);
  return true;
}

// "<count>x<128|256|512>", count 1..65535: a large image of one track.
static bool parseSectors(const char *s, atari_geometry_t *g) {
  char *end;
  unsigned long count = strtoul(s, &end, 10);
  if (end == s || (*end != 'x' && *end != 'X') || count < 1 || count > 65535)
    return false;
  unsigned long size = strtoul(end + 1, &end, 10);
  if (*end || (size != 128 && size != 256 && size != 512))
    return false;
  *g = atari_geometry_for_sectors((uint16_t)size, (uint16_t)count);
  return true;
}

// Logical sector at (track, head, slot) of a floppy geometry, 0 if none.
static uint16_t sectorFor(const atari_geometry_t &g, uint8_t t, uint8_t h,
                          uint16_t i) {
  uint32_t per = (uint32_t)g.tracks * g.sectors_per_track;
  uint32_t idx = (uint32_t)t * g.sectors_per_track + i;
  uint32_t cand =
      h == 0 ? idx + 1 : per + (per - 1 - idx) + 1; // side 1 reversed
  atari_geometry_t gg = g;
  uint8_t c, hh;
  uint16_t sl;
  if (cand <= g.sector_count &&
      atari_sector_location(&gg, (uint16_t)cand, &c, &hh, &sl) && c == t &&
      hh == h && sl == i)
    return (uint16_t)cand;
  for (uint32_t k = 1; k <= g.sector_count; k++)
    if (atari_sector_location(&gg, (uint16_t)k, &c, &hh, &sl) && c == t &&
        hh == h && sl == i)
      return (uint16_t)k;
  return 0;
}

// "1".."4" to a unit index 0..3, or -1.
static int parseUnit(const char *s) {
  if (s[0] >= '1' && s[0] <= '0' + UNITS && s[1] == 0)
    return s[0] - '1';
  return -1;
}

static bool hasExtension(const char *name, const char *ext) {
  size_t n = strlen(name);
  return n > 4 && !strcasecmp(name + n - 4, ext);
}

static bool hasImageExtension(const char *name) {
  return hasExtension(name, ".atr") || hasExtension(name, ".xfd") ||
         hasExtension(name, ".atx");
}

// An ATX image: the .atx extension or the AT8X signature at the start.
static bool isAtxFile(FsFile &f, const char *path) {
  if (hasExtension(path, ".atx"))
    return true;
  char sig[4];
  f.seekSet(0);
  bool is = f.read(sig, 4) == 4 && !memcmp(sig, ATARI_ATX_SIGNATURE, 4);
  f.seekSet(0);
  return is;
}

static bool isFileSource(Source s) { return s == SRC_IMAGE || s == SRC_ATX; }

static bool isBlank(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

// Split on blanks (including CR and LF) in place; returns the number of words
// found.
static int split(char *s, char **words, int max) {
  int n = 0;
  while (*s && n < max) {
    while (isBlank(*s))
      s++;
    if (!*s)
      break;
    words[n++] = s;
    while (*s && !isBlank(*s))
      s++;
    if (*s)
      *s++ = 0;
  }
  return n;
}

static int floppyUnit() {
  for (uint8_t i = 0; i < UNITS; i++)
    if (mounts[i].source == SRC_FLOPPY)
      return i;
  return -1;
}

// The floppy operations below block for seconds to minutes and own the drive,
// so take the floppy off the SIO bus meanwhile and put it back afterwards.
class FloppyBusy {
public:
  FloppyBusy() : unit_(floppyUnit()) {
    if (unit_ >= 0)
      sio.detach(unit_ + 1);
    Serial.println("The Atari cannot use the drives until this finishes.");
  }
  ~FloppyBusy() {
    if (unit_ >= 0)
      sio.attach(unit_ + 1, &atariFloppy);
  }

private:
  int unit_;
};

// ---------------------------------------------------------------------------
// Mounting
// ---------------------------------------------------------------------------

static bool unmountUnit(uint8_t u) {
  Mount &m = mounts[u];
  if (m.source == SRC_NONE)
    return false;
  sio.detach(u + 1);
  if (m.source == SRC_IMAGE) {
    m.atr.flush();
    m.atr.unmount();
    m.file.close();
  } else if (m.source == SRC_ATX) {
    m.atx.unmount();
    m.file.close();
  } else {
    atariFloppy.eject();
  }
  m.source = SRC_NONE;
  m.path[0] = 0;
  return true;
}

static bool mountImage(uint8_t u, const char *path) {
  if (!sdReady) {
    Serial.println("No SD card.");
    return false;
  }
  if (strlen(path) >= PATH_LEN) {
    Serial.println("Path too long.");
    return false;
  }
  // Two units on one file would each cache its header; a format through one
  // would leave the other reading at stale offsets.
  for (uint8_t i = 0; i < UNITS; i++) {
    if (i != u && isFileSource(mounts[i].source) &&
        !strcasecmp(mounts[i].path, path)) {
      out("%s is already mounted as D%u:\n", path, i + 1);
      return false;
    }
  }
  unmountUnit(u);
  Mount &m = mounts[u];
  m.file = sd.open(path, O_RDWR);
  if (!m.file)
    m.file = sd.open(path, O_RDONLY);
  if (!m.file) {
    out("Cannot open %s\n", path);
    return false;
  }
  if (isAtxFile(m.file, path)) {
    if (!m.atx.mount(&m.adaptor)) {
      out("%s is not a usable ATX image\n", path);
      m.file.close();
      return false;
    }
    m.atx.setDriveModel(driveModel);
    snprintf(m.path, sizeof(m.path), "%s", path);
    m.source = SRC_ATX;
    sio.attach(u + 1, &m.atx);
    out("D%u: %s, ATX ", u + 1, path);
    printGeometry(m.atx.geometry());
    out(", read-only, %s timing\n",
        driveModel == ATARI_DRIVE_810 ? "810" : "1050");
    return true;
  }
  if (!m.atr.mount(&m.adaptor)) {
    out("%s is not a usable ATR/XFD image\n", path);
    m.file.close();
    return false;
  }
  snprintf(m.path, sizeof(m.path), "%s", path);
  m.source = SRC_IMAGE;
  sio.attach(u + 1, &m.atr);
  out("D%u: %s, ", u + 1, path);
  printGeometry(m.atr.geometry());
  out(", %s\n", m.atr.writeProtected() ? "read-only" : "read-write");
  return true;
}

static bool mountFloppy(uint8_t u) {
  int other = floppyUnit();
  if (other >= 0 && other != u) {
    out("The floppy is already D%d:\n", other + 1);
    return false;
  }
  unmountUnit(u);
  Mount &m = mounts[u];
  m.source = SRC_FLOPPY;
  sio.attach(u + 1, &atariFloppy);
  Serial.println("Spinning up the floppy drive...");
  if (atariFloppy.insert()) {
    out("D%u: floppy, ", u + 1);
    printGeometry(atariFloppy.geometry());
    out(", %s, ",
        atariFloppy.writeProtected() ? "write protected" : "writable");
    printRpm();
  } else {
    out("D%u: floppy, no readable disk (rechecked on every access)\n", u + 1);
  }
  return true;
}

static bool mountUnit(uint8_t u, const char *what) {
  if (!strcasecmp(what, "floppy"))
    return mountFloppy(u);
  return mountImage(u, what);
}

// ---------------------------------------------------------------------------
// Config file: lines of "D1 FLOPPY" or "D2 /path/file.atr" (.xfd and .atx
// too, chosen by signature); '#' starts a
// comment line.
// ---------------------------------------------------------------------------

static void loadConfig() {
  FsFile f = sd.open(CONFIG_PATH, O_RDONLY);
  if (!f) {
    out("No %s, nothing mounted.\n", CONFIG_PATH);
    return;
  }
  char buf[LINE_LEN];
  while (f.fgets(buf, sizeof(buf)) > 0) {
    char *w[2];
    int n = split(buf, w, 2);
    if (n == 0 || w[0][0] == '#')
      continue;
    if (n == 2 && (w[0][0] == 'D' || w[0][0] == 'd') &&
        parseUnit(w[0] + 1) >= 0) {
      mountUnit(parseUnit(w[0] + 1), w[1]);
    } else {
      out("%s: ignored line \"%s\"\n", CONFIG_PATH, w[0]);
    }
  }
  f.close();
}

static void cmdSave() {
  if (!sdReady) {
    Serial.println("No SD card.");
    return;
  }
  FsFile f = sd.open(CONFIG_PATH, O_WRONLY | O_CREAT | O_TRUNC);
  if (!f) {
    out("Cannot write %s\n", CONFIG_PATH);
    return;
  }
  for (uint8_t i = 0; i < UNITS; i++) {
    char buf[PATH_LEN + 8];
    int n = 0;
    if (mounts[i].source == SRC_FLOPPY)
      n = snprintf(buf, sizeof(buf), "D%u FLOPPY\n", i + 1);
    else if (isFileSource(mounts[i].source))
      n = snprintf(buf, sizeof(buf), "D%u %s\n", i + 1, mounts[i].path);
    if (n > 0)
      f.write(buf, n);
  }
  f.close();
  out("Saved %s\n", CONFIG_PATH);
}

// ---------------------------------------------------------------------------
// Console commands
// ---------------------------------------------------------------------------

static void cmdHelp() {
  Serial.println(
      "ls [dir]                  list .atr/.xfd/.atx files and folders\n"
      "mount <1-4> <path>|floppy attach an image (ATR, XFD, ATX) or the "
      "floppy\n"
      "umount <1-4>              detach a unit\n"
      "status                    units, drive model, SIO statistics\n"
      "new <path> <fmt>          create a blank ATR; fmt as below, or\n"
      "                          <count>x<128|256|512>, e.g. 65535x512\n"
      "dump <path>               copy the floppy to a new .atr, or\n"
      "                          to a new .atx keeping copy protection\n"
      "write <path> yes          copy an .atr (or master an .atx) onto the "
      "floppy\n"
      "format <fmt> yes          format the floppy (erases it)\n"
      "                          fmt: [ds](sd|ed|dd)[80], e.g. sd, dsdd, "
      "dd80\n"
      "drive 810|1050            drive timing model for ATX images\n"
      "speed <divisor>|off       Ultra Speed POKEY divisor (default 10)\n"
      "save                      store the mounts in "
      "/ATARISIO.CFG");
}

static void cmdLs(const char *dir) {
  if (!sdReady) {
    Serial.println("No SD card.");
    return;
  }
  FsFile d = sd.open(dir);
  if (!d || !d.isDir()) {
    out("%s is not a folder\n", dir);
    return;
  }
  FsFile e;
  char name[64];
  while (e.openNext(&d, O_RDONLY)) {
    e.getName(name, sizeof(name));
    if (e.isDir())
      out("           %s/\n", name);
    else if (hasImageExtension(name))
      out("%10lu %s\n", (unsigned long)e.fileSize(), name);
    e.close();
  }
}

static void cmdMount(char **w, int n) {
  int u = n == 3 ? parseUnit(w[1]) : -1;
  if (u < 0)
    Serial.println("usage: mount <1-4> <path>|floppy");
  else
    mountUnit(u, w[2]);
}

static void cmdUmount(char **w, int n) {
  int u = n == 2 ? parseUnit(w[1]) : -1;
  if (u < 0)
    Serial.println("usage: umount <1-4>");
  else if (unmountUnit(u))
    out("D%d: detached\n", u + 1);
  else
    out("D%d: was empty\n", u + 1);
}

static void cmdStatus() {
  for (uint8_t i = 0; i < UNITS; i++) {
    Mount &m = mounts[i];
    out("D%u: ", i + 1);
    if (m.source == SRC_NONE) {
      Serial.println("empty");
      continue;
    }
    out("%s ", m.source == SRC_FLOPPY ? "floppy" : m.path);
    Adafruit_AtariDisk *disk = &m.atr;
    if (m.source == SRC_FLOPPY)
      disk = &atariFloppy;
    else if (m.source == SRC_ATX)
      disk = &m.atx;
    if (!disk->ready()) {
      Serial.println("(no disk)");
      continue;
    }
    if (m.source == SRC_ATX)
      out("ATX ");
    printGeometry(disk->geometry());
    out(", %s\n", disk->writeProtected() ? "write protected" : "writable");
  }
  out("ATX drive model: %s\n", driveModel == ATARI_DRIVE_810 ? "810" : "1050");
  const Adafruit_AtariSIO::Stats &s = sio.stats();
  out("SIO: %lu frames, %lu bad checksums, %lu bad frames, %lu NAK, %lu "
      "errors\n",
      (unsigned long)s.frames, (unsigned long)s.badChecksums,
      (unsigned long)s.badFrames, (unsigned long)s.naks,
      (unsigned long)s.errors);
  if (s.frames)
    out("Last command: '%c' D%u aux %u, reply '%c'\n", s.lastCommand,
        s.lastUnit, s.lastAux, s.lastResult);
  if (speedDivisor)
    out("High speed: Ultra Speed divisor %u (%lu baud), currently %s\n",
        speedDivisor, (unsigned long)Adafruit_AtariSIO::pokeyBaud(speedDivisor),
        s.highSpeed ? "active" : "idle");
  else
    Serial.println("High speed: off");
}

static void cmdSpeed(char **w, int n) {
  if (n != 2) {
    Serial.println("usage: speed <divisor 1-255>|off");
    return;
  }
  int d = strcasecmp(w[1], "off") ? atoi(w[1]) : 0;
  if (d < 0 || d > 255 || (d == 0 && strcasecmp(w[1], "off"))) {
    Serial.println("The divisor must be 1-255, or off.");
    return;
  }
  speedDivisor = d;
  sio.setUltraSpeed(speedDivisor);
  Serial.println("OK");
}

static void cmdNew(char **w, int n) {
  atari_geometry_t g;
  if (n != 3 || !(parseFormat(w[2], &g) || parseSectors(w[2], &g))) {
    Serial.println("usage: new <path> [ds](sd|ed|dd)[80] | <count>x<size>");
    return;
  }
  if (!sdReady || sd.exists(w[1])) {
    Serial.println("No SD card, or the file already exists.");
    return;
  }
  Mount &c = copyImage;
  c.file = sd.open(w[1], O_RDWR | O_CREAT | O_EXCL);
  bool opened = c.file;
  bool ok = opened && Adafruit_AtariATR::create(&c.adaptor, g);
  c.file.close();
  if (opened && !ok)
    sd.remove(w[1]); // we created it: don't leave a partial image behind
  if (ok) {
    out("Created %s: ", w[1]);
    printGeometry(g);
    Serial.println();
  } else {
    out("Could not create %s\n", w[1]);
  }
}

// Copies the floppy to a new ATR; unreadable sectors stay zero.
static void dumpFloppy(const char *path) {
  Serial.println("Reading the floppy...");
  if (!atariFloppy.insert()) {
    Serial.println("No readable Atari disk found.");
    return;
  }
  atari_geometry_t g = atariFloppy.geometry();
  printGeometry(g);
  Serial.print(", ");
  printRpm();

  Mount &c = copyImage;
  c.file = sd.open(path, O_RDWR | O_CREAT | O_EXCL);
  if (!c.file || !Adafruit_AtariATR::create(&c.adaptor, g) ||
      !c.atr.mount(&c.adaptor)) {
    out("Cannot create %s\n", path);
    bool created = c.file;
    c.file.close();
    if (created)
      sd.remove(path);
    return;
  }
  uint8_t buf[256];
  uint16_t bad = 0;
  bool ok = true;
  // One group of sectors per track side, in logical order
  for (uint16_t first = 1; ok && first <= g.sector_count;
       first += g.sectors_per_track) {
    atari_geometry_t gg = g;
    uint8_t cyl = 0, head = 0;
    uint16_t slot;
    atari_sector_location(&gg, first, &cyl, &head, &slot);
    out("track %2u side %u:", cyl, head);
    bool trackBad = false;
    for (uint16_t i = 0; i < g.sectors_per_track; i++) {
      uint16_t sector = first + i;
      atari_disk_result_t r = atariFloppy.readSector(sector, buf);
      if (r == ATARI_DISK_NOT_READY || r == ATARI_DISK_IO_ERROR) {
        out("\nStopped: %s (disk removed?)\n", resultName(r));
        ok = false;
        break;
      }
      if (r != ATARI_DISK_OK) {
        out(" %u", sector);
        trackBad = true;
        bad++;
      } else if (c.atr.writeSector(sector, buf, false) != ATARI_DISK_OK) {
        ok = false;
        break;
      }
    }
    Serial.println(trackBad ? " unreadable (stored as zeros)" : " ok");
  }
  c.atr.flush();
  c.atr.unmount();
  c.file.close();
  if (ok) {
    out("Wrote %s, %u unreadable sectors\n", path, bad);
  } else {
    sd.remove(path);
    Serial.println("Failed (disk removed or SD error); partial file deleted.");
  }
}

// Copies a single-sided 40-track floppy to a new ATX, keeping duplicate,
// bad-CRC, missing and deleted sectors and (by comparing revolutions) weak
// sectors.
static void dumpFloppyAtx(const char *path) {
  Serial.println("Reading the floppy...");
  if (!atariFloppy.insert()) {
    Serial.println("No readable Atari disk found.");
    return;
  }
  atari_geometry_t g = atariFloppy.geometry();
  atari_density_t density = atari_track_density(&g);
  if (g.sides != 1 || g.tracks != 40 || density == ATARI_DENSITY_OTHER) {
    Serial.println("ATX holds single-sided 40-track disks only; use .atr.");
    return;
  }
  printGeometry(g);
  Serial.print(", ");
  printRpm();
  Mount &c = copyImage;
  c.file = sd.open(path, O_RDWR | O_CREAT | O_EXCL);
  Adafruit_AtariATXWriter writer;
  if (!c.file || !writer.begin(&c.adaptor, density)) {
    out("Cannot create %s\n", path);
    bool created = c.file;
    c.file.close();
    if (created)
      sd.remove(path);
    return;
  }
  bool ok = true;
  unsigned totDup = 0, totCrc = 0, totMissing = 0, totDeleted = 0, totWeak = 0;
  for (uint8_t t = 0; ok && t < g.tracks; t++) {
    unsigned overflow = 0;
    atari_disk_result_t r =
        atariFloppy.readProtectedTrack(t, &protTrack, 5, &overflow);
    if (r != ATARI_DISK_OK) {
      out("track %2u: %s\n", t, resultName(r));
      ok = false;
      break;
    }
    unsigned dup = 0, crc = 0, missing = 0, deleted = 0, weak = 0;
    for (uint16_t i = 0; i < protTrack.sector_count; i++) {
      const atari_prot_sector_t &e = protTrack.sectors[i];
      for (uint16_t j = 0; j < i; j++)
        if (protTrack.sectors[j].number == e.number) {
          dup++;
          break;
        }
      if (e.weak_offset != ATARI_PROT_NONE)
        weak++;
      else if (e.status & ATARI_ATX_CRC_ERROR)
        crc++;
      if (e.status & ATARI_ATX_MISSING)
        missing++;
      if (e.status & ATARI_ATX_DELETED)
        deleted++;
    }
    out("track %2u: %2u entries, %u duplicate, %u CRC, %u missing, %u deleted, "
        "%u weak\n",
        t, (unsigned)protTrack.sector_count, dup, crc, missing, deleted, weak);
    if (overflow)
      out("  WARNING: track %u overflowed the entry or data limit; %u entries "
          "lost, the dump of this track is incomplete\n",
          t, overflow);
    totDup += dup;
    totCrc += crc;
    totMissing += missing;
    totDeleted += deleted;
    totWeak += weak;
    ok = writer.addTrack(protTrack);
    if (!ok)
      Serial.println("Cannot write the ATX (SD error).");
  }
  ok = ok && writer.finish();
  c.file.close();
  if (ok) {
    out("Wrote %s: %u duplicate, %u CRC, %u missing, %u deleted, %u weak\n",
        path, totDup, totCrc, totMissing, totDeleted, totWeak);
  } else {
    sd.remove(path);
    Serial.println("Failed; partial file deleted.");
  }
}

static void cmdDump(char **w, int n) {
  if (n != 2 || !sdReady || sd.exists(w[1])) {
    Serial.println(
        "usage: dump <new .atr|.atx path>   (needs an SD card; will not "
        "overwrite; .atx keeps copy protection)");
    return;
  }
  FloppyBusy busy;
  if (hasExtension(w[1], ".atx"))
    dumpFloppyAtx(w[1]);
  else if (hasExtension(w[1], ".atr"))
    dumpFloppy(w[1]);
  else
    Serial.println("dump writes .atr or .atx files only.");
}

// Copies an ATR onto the floppy, one whole track of one side at a time.
static void writeFloppy(const char *path) {
  Mount &c = copyImage;
  c.file = sd.open(path, O_RDONLY);
  if (!c.file || !c.atr.mount(&c.adaptor)) {
    out("%s is not a usable ATR/XFD image\n", path);
    c.file.close();
    return;
  }
  atari_geometry_t g = c.atr.geometry();
  if (!atariFloppy.canFormat(g)) {
    Serial.println("This drive cannot write that image: only SD, ED and DD "
                   "128/256-byte images of 40 tracks (80 with an 80-track "
                   "drive), one or two sides.");
  } else {
    static uint8_t trackData[ATARI_FLOPPY_MAX_BYTES];
    Serial.println("Writing the floppy...");
    for (uint8_t t = 0; t < g.tracks; t++) {
      for (uint8_t h = 0; h < g.sides; h++) {
        // Slots are full-size and zero padded; boot sectors of a DD image only
        // fill the first 128 bytes.
        memset(trackData, 0, sizeof(trackData));
        for (uint16_t i = 0; i < g.sectors_per_track; i++) {
          uint16_t sector = sectorFor(g, t, h, i);
          atari_disk_result_t r =
              sector ? c.atr.readSector(sector, trackData + i * g.sector_size)
                     : ATARI_DISK_NOT_FOUND;
          if (r != ATARI_DISK_OK) {
            out("Cannot read sector %u of the image: %s\n", sector,
                resultName(r));
            goto done;
          }
        }
        atari_disk_result_t r =
            atariFloppy.writeTrack(t, h, g, trackData, true);
        out("track %2u side %u: %s\n", t, h, resultName(r));
        if (r != ATARI_DISK_OK)
          goto done;
      }
    }
    Serial.println("Done.");
  }
done:
  c.atr.unmount();
  c.file.close();
}

// Masters an ATX onto the floppy: every track is flux-encoded with its
// duplicates, errors and weak sectors (experimental).
static void masterFloppy(const char *path) {
  Mount &c = copyImage;
  c.file = sd.open(path, O_RDONLY);
  if (!c.file || !c.atx.mount(&c.adaptor)) {
    out("%s is not a usable ATX image\n", path);
    c.file.close();
    return;
  }
  atari_geometry_t g = c.atx.geometry();
  Serial.println("Mastering the floppy (experimental)...");
  unsigned failed = 0;
  uint32_t worst = 0;
  for (uint8_t t = 0; t < g.tracks; t++) {
    if (!c.atx.loadTrack(t, &protTrack)) {
      out("track %2u: cannot read the image\n", t);
      failed++;
      break;
    }
    uint32_t moved = 0;
    atari_disk_result_t r =
        atariFloppy.writeProtectedTrack(&protTrack, true, &moved);
    out("track %2u: %2u entries, moved %lu units, %s\n", t,
        (unsigned)protTrack.sector_count, (unsigned long)moved, resultName(r));
    if (moved > worst)
      worst = moved;
    if (r != ATARI_DISK_OK) {
      failed++;
      if (r == ATARI_DISK_WRITE_PROTECTED)
        break;
    }
  }
  out("Done: %u failed tracks, largest push %lu units (26042 per turn)\n",
      failed, (unsigned long)worst);
  c.atx.unmount();
  c.file.close();
}

static void cmdWrite(char **w, int n) {
  if (n != 3 || strcmp(w[2], "yes") || !sdReady) {
    Serial.println("usage: write <path> yes   (erases the floppy)");
    return;
  }
  FloppyBusy busy;
  FsFile f = sd.open(w[1], O_RDONLY);
  bool atx = f && isAtxFile(f, w[1]);
  f.close();
  if (atx)
    masterFloppy(w[1]);
  else
    writeFloppy(w[1]);
}

static void cmdFormat(char **w, int n) {
  atari_geometry_t g;
  if (n != 3 || !parseFormat(w[1], &g) || strcmp(w[2], "yes")) {
    Serial.println(
        "usage: format [ds](sd|ed|dd)[80] yes   (erases the floppy)");
    return;
  }
  if (!atariFloppy.canFormat(g)) {
    Serial.println("This drive cannot format that (80 tracks need an 80-track "
                   "drive).");
    return;
  }
  FloppyBusy busy;
  Serial.println("Formatting...");
  out("%s\n", resultName(atariFloppy.format(g)));
}

static void cmdDrive(char **w, int n) {
  if (n != 2 || (strcmp(w[1], "810") && strcmp(w[1], "1050"))) {
    Serial.println("usage: drive 810|1050");
    return;
  }
  driveModel = strcmp(w[1], "810") ? ATARI_DRIVE_1050 : ATARI_DRIVE_810;
  for (uint8_t i = 0; i < UNITS; i++)
    mounts[i].atx.setDriveModel(driveModel);
  copyImage.atx.setDriveModel(driveModel);
  Serial.println("OK");
}

static void runCommand(char *s) {
  char *w[4];
  int n = split(s, w, 4);
  if (n == 0)
    return;
  if (!strcasecmp(w[0], "help"))
    cmdHelp();
  else if (!strcasecmp(w[0], "ls"))
    cmdLs(n > 1 ? w[1] : "/");
  else if (!strcasecmp(w[0], "mount"))
    cmdMount(w, n);
  else if (!strcasecmp(w[0], "umount"))
    cmdUmount(w, n);
  else if (!strcasecmp(w[0], "status"))
    cmdStatus();
  else if (!strcasecmp(w[0], "new"))
    cmdNew(w, n);
  else if (!strcasecmp(w[0], "dump"))
    cmdDump(w, n);
  else if (!strcasecmp(w[0], "write"))
    cmdWrite(w, n);
  else if (!strcasecmp(w[0], "format"))
    cmdFormat(w, n);
  else if (!strcasecmp(w[0], "drive"))
    cmdDrive(w, n);
  else if (!strcasecmp(w[0], "speed"))
    cmdSpeed(w, n);
  else if (!strcasecmp(w[0], "save"))
    cmdSave();
  else
    Serial.println("Unknown command. Type help.");
}

// Collects a line without blocking, so the SIO bus keeps being served.
static void pollConsole() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineOverflow)
        Serial.println("Line too long.");
      else if (lineLen) {
        line[lineLen] = 0;
        runCommand(line);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if ((c == '\b' || c == 0x7f) && lineLen) {
      lineLen--;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // The drives work without a USB host; only wait briefly for a console.
  while (!Serial && millis() < 3000)
    delay(10);
  Serial.println("Atari SIO drive emulator");

#if defined(ARDUINO_ADAFRUIT_FLOPPSY_RP2350)
  if (!floppsy.begin() || !floppsy.enable(true))
    Serial.println("Floppsy drive setup failed.");
#endif
#if defined(FLOPPY_DIRECTION_PIN)
  pinMode(FLOPPY_DIRECTION_PIN, OUTPUT);
  digitalWrite(FLOPPY_DIRECTION_PIN, HIGH);
#endif
#if defined(FLOPPY_ENABLE_PIN)
  pinMode(FLOPPY_ENABLE_PIN, OUTPUT);
  digitalWrite(FLOPPY_ENABLE_PIN, LOW); // after setting the direction
#endif
  if (!FLOPPY_DRIVE->begin() || !atariFloppy.begin(DRIVE_RPM, DRIVE_TRACKS))
    Serial.println("Floppy drive setup failed; the floppy is unavailable.");

#if defined(SIO_UART_TX_PIN)
  if (!Serial1.setTX(SIO_UART_TX_PIN) || !Serial1.setRX(SIO_UART_RX_PIN))
    Serial.println("ERROR: these SIO UART pins are not valid for Serial1.");
#endif
  sio.begin();
  sio.setUltraSpeed(speedDivisor);

#if defined(ARDUINO_ADAFRUIT_FLOPPSY_RP2040)
  // Keep the on-board display deselected so it never answers on the SD bus.
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
#endif
  sdReady = sd.begin(SD_CONFIG);
  if (sdReady)
    loadConfig();
  else
    Serial.println("Could not mount the microSD card; images are unavailable.");
  Serial.println("Type help for commands.");
}

void loop() {
  sio.poll();
  atariFloppy.idle();
  pollConsole();
}
