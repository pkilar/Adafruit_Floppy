// Host tests for the Atari SIO protocol engine (src/Adafruit_AtariSIO.cpp).
// A simulated serial port with a virtual clock plays the Atari side.
#include "Adafruit_AtariSIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

static int g_pass = 0, g_fail = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (c) {                                                                   \
      g_pass++;                                                                \
    } else {                                                                   \
      g_fail++;                                                                \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                      \
    }                                                                          \
  } while (0)

// Independent checksum implementation (end-around carry).
static uint8_t sum8(const uint8_t *p, size_t n) {
  unsigned s = 0;
  for (size_t i = 0; i < n; i++) {
    s += p[i];
    s = (s & 0xFF) + (s >> 8);
  }
  return (uint8_t)s;
}

struct RxByte {
  uint32_t t;
  uint8_t b;
  uint32_t baud;
};
struct TxByte {
  uint32_t start, end;
  uint8_t b;
  uint32_t baud;
};
struct Interval {
  uint32_t a, b;
};

static uint32_t bitDur10(uint32_t baud) {
  return (10000000u + baud / 2) / baud;
}

// Simulated wire: bytes are delivered intact only if the receiver's baud is
// within 3% of the sender's, otherwise garbage.
struct SimPort : Adafruit_AtariSIOPort {
  uint32_t now = 1000, baud = 0, txBusy = 0;
  std::vector<RxByte> rx;
  size_t rxPos = 0;
  std::vector<Interval> cmd;
  std::vector<TxByte> tx;
  std::vector<uint32_t> bauds;

  bool commandAsserted() override {
    for (size_t i = 0; i < cmd.size(); i++)
      if (now >= cmd[i].a && now < cmd[i].b)
        return true;
    return false;
  }
  int read() override {
    if (rxPos < rx.size() && rx[rxPos].t <= now) {
      const RxByte &r = rx[rxPos++];
      uint32_t d = baud > r.baud ? baud - r.baud : r.baud - baud;
      return d * 33 <= r.baud ? r.b : (r.b ^ 0x55);
    }
    now += 1;
    return -1;
  }
  void write(const uint8_t *d, size_t n) override {
    for (size_t i = 0; i < n; i++) {
      TxByte t;
      t.start = now > txBusy ? now : txBusy;
      t.end = t.start + bitDur10(baud);
      t.b = d[i];
      t.baud = baud;
      txBusy = t.end;
      tx.push_back(t);
    }
  }
  void flushTx() override {
    if (txBusy > now)
      now = txBusy;
  }
  void clearRx() override {
    while (rxPos < rx.size() && rx[rxPos].t <= now)
      rxPos++;
  }
  void setBaud(uint32_t b) override {
    baud = b;
    bauds.push_back(b);
  }
  uint32_t micros() override { return now; }
  void delayMicros(uint32_t us) override { now += us; }
};

// RAM-backed disk with injectable failures.
struct RamDisk : Adafruit_AtariDisk {
  static const unsigned kStride = 512; // bytes reserved per sector in `data`
  atari_geometry_t g;
  // readSectorTimed hook: sector `timedSector` returns timedResult with the
  // bad data in the buffer and the given info; `lastNow` records tAck.
  uint16_t timedSector = 0;
  atari_disk_result_t timedResult = ATARI_DISK_CRC_ERROR;
  atari_read_info_t timedInfo = {0, false, 0};
  uint32_t lastNow = 0;
  int timedCalls = 0, plainCalls = 0;
  std::vector<uint8_t> data;
  bool isReady = true, wp = false;
  uint16_t errSector = 0;
  atari_disk_result_t errResult = ATARI_DISK_NOT_FOUND;
  int writes = 0, formats = 0;
  bool lastVerify = false;
  uint16_t lastWritten = 0;

  explicit RamDisk(const atari_geometry_t &geo) { setGeo(geo); }
  void setGeo(const atari_geometry_t &geo) {
    g = geo;
    data.assign((size_t)geo.sector_count * kStride, 0);
    for (unsigned s = 1; s <= geo.sector_count; s++)
      for (unsigned i = 0; i < kStride; i++)
        data[(size_t)(s - 1) * kStride + i] = pat(s, i);
  }
  static uint8_t pat(unsigned s, unsigned i) { return (uint8_t)(s * 7 + i); }
  bool ready() override { return isReady; }
  atari_geometry_t geometry() override { return g; }
  bool writeProtected() override { return wp; }
  atari_disk_result_t readSectorTimed(uint16_t s, uint8_t *buf, uint32_t now_us,
                                      atari_read_info_t *info) override {
    timedCalls++;
    lastNow = now_us;
    if (s == timedSector) {
      memset(buf, 0xE5, atari_sector_bytes(&g, s)); // "bad" data
      *info = timedInfo;
      return timedResult;
    }
    return Adafruit_AtariDisk::readSectorTimed(s, buf, now_us, info);
  }
  atari_disk_result_t readSector(uint16_t s, uint8_t *buf) override {
    plainCalls++;
    if (!isReady)
      return ATARI_DISK_NOT_READY;
    if (s == errSector)
      return errResult;
    memcpy(buf, &data[(size_t)(s - 1) * kStride], atari_sector_bytes(&g, s));
    return ATARI_DISK_OK;
  }
  atari_disk_result_t writeSector(uint16_t s, const uint8_t *buf,
                                  bool verify) override {
    if (wp)
      return ATARI_DISK_WRITE_PROTECTED;
    memcpy(&data[(size_t)(s - 1) * kStride], buf, atari_sector_bytes(&g, s));
    writes++;
    lastVerify = verify;
    lastWritten = s;
    return ATARI_DISK_OK;
  }
  bool canFormat(const atari_geometry_t &f) override {
    if (f.sector_size == 512)
      return f.mfm &&
             f.tracks * f.sides * f.sectors_per_track == f.sector_count;
    return f.tracks == 40 && f.sides == 1 &&
           ((f.sector_size == 128 &&
             (f.sectors_per_track == 18 || f.sectors_per_track == 26)) ||
            (f.sector_size == 256 && f.sectors_per_track == 18));
  }
  atari_disk_result_t format(const atari_geometry_t &f) override {
    if (!isReady)
      return ATARI_DISK_NOT_READY;
    if (wp)
      return ATARI_DISK_WRITE_PROTECTED;
    if (!canFormat(f))
      return ATARI_DISK_UNSUPPORTED;
    g = f;
    data.assign((size_t)f.sector_count * kStride, 0);
    formats++;
    return ATARI_DISK_OK;
  }
};

struct Fix {
  SimPort port;
  RamDisk disk;
  Adafruit_AtariSIO sio;
  uint32_t deassert = 0;

  explicit Fix(atari_density_t d = ATARI_DENSITY_SD)
      : disk(atari_geometry_for_density(d)), sio(&port) {
    sio.attach(1, &disk);
    sio.begin();
    port.tx.clear();
    port.bauds.clear();
  }

  // Schedule a command frame: COMMAND low, first byte after T0 = 900 us,
  // COMMAND high T1 = 850 us after the last stop bit.
  void command(uint8_t cmd, uint16_t aux, uint32_t baud = 19040,
               uint8_t unit = 0x31, int chk = -1, uint32_t gap = 3000) {
    uint8_t f[5] = {unit, cmd, (uint8_t)aux, (uint8_t)(aux >> 8), 0};
    f[4] = chk < 0 ? sum8(f, 4) : (uint8_t)chk;
    uint32_t ta = port.now + gap, d = bitDur10(baud);
    for (int i = 0; i < 5; i++) {
      RxByte r = {ta + 900 + (uint32_t)(i + 1) * d, f[i], baud};
      port.rx.push_back(r);
    }
    deassert = ta + 900 + 5 * d + 850;
    Interval iv = {ta, deassert};
    port.cmd.push_back(iv);
  }
  // Schedule an Atari data frame starting at absolute time t0.
  void dataFrame(uint32_t t0, const uint8_t *p, size_t n, uint32_t baud,
                 bool badSum = false) {
    uint32_t d = bitDur10(baud);
    for (size_t i = 0; i <= n; i++) {
      uint8_t b = i < n ? p[i] : (uint8_t)(sum8(p, n) + (badSum ? 1 : 0));
      RxByte r = {t0 + (uint32_t)(i + 1) * d, b, baud};
      port.rx.push_back(r);
    }
  }
  void run(uint32_t us = 60000) {
    uint32_t end = port.now + us;
    while ((int32_t)(end - port.now) > 0) {
      sio.poll();
      port.now += 10;
    }
  }
  size_t n() const { return port.tx.size(); }
  uint8_t b(size_t i) const { return i < port.tx.size() ? port.tx[i].b : 0; }
  // Gap in us between the stop bit of tx[i-1] and the start of tx[i].
  uint32_t gap(size_t i) const {
    return i == 0 ? port.tx[0].start - deassert
                  : port.tx[i].start - port.tx[i - 1].end;
  }
  // tx[from..from+len) followed by a correct checksum?
  bool frameOk(size_t from, size_t len) const {
    if (n() < from + len + 1)
      return false;
    uint8_t tmp[520];
    for (size_t i = 0; i < len; i++)
      tmp[i] = b(from + i);
    return sum8(tmp, len) == b(from + len);
  }
  bool contiguous(size_t from, size_t to) const {
    for (size_t i = from + 1; i < to; i++)
      if (port.tx[i].start != port.tx[i - 1].end)
        return false;
    return true;
  }
};

// Standard A, C, data+chk sequence with minimum gaps.
static void expectReadReply(Fix &f, size_t len, const char *what) {
  (void)what;
  CHECK(f.n() == 2 + len + 1);
  if (f.n() != 2 + len + 1)
    return;
  CHECK(f.b(0) == 'A' && f.b(1) == 'C');
  // Command-to-ACK gap, AtariSIO driver/atarisio.c.
  CHECK(f.gap(0) >= 100 && f.gap(0) < 10000); // min 100 us
  CHECK(f.gap(1) >= 300);                     // ACK -> COMPLETE
  CHECK(f.gap(2) >= 150);                     // COMPLETE -> data
  CHECK(f.contiguous(2, f.n()));              // data frame back to back
  CHECK(f.frameOk(2, len));
}

// Expect `len` bytes of the sector pattern for sector `sec` at tx[2..].
static bool dataIs(const Fix &f, uint16_t sec, size_t len) {
  for (size_t i = 0; i < len; i++)
    if (f.b(2 + i) != RamDisk::pat(sec, (unsigned)i))
      return false;
  return true;
}

static void test_checksum_and_framing() {
  // Sector 176: 0x31+0x52+0xB0 = 0x133 -> end-around 0x34 (plain mod 256 =
  // 0x33)
  const uint8_t ones[2] = {0xFF, 0x01};
  Fix f;
  f.command('R', 176, 19040, 0x31, 0x34);
  f.run();
  CHECK(f.n() > 0 && f.b(0) == 'A');
  Fix g;
  g.command('R', 176, 19040, 0x31, 0x33);
  g.run();
  CHECK(g.n() == 0);
  CHECK(g.sio.stats().badChecksums == 1);
  CHECK(sum8(ones, 2) == 0x01);

  Fix w;
  w.command('R', 1, 19040, 0x32);
  w.run();
  CHECK(w.n() == 0); // wrong unit
  Fix u;
  u.command('R', 1, 19040, 0x31, 0x99 /* bad sum */);
  u.run();
  CHECK(u.n() == 0);
}

static void test_status() {
  const atari_density_t dens[3] = {ATARI_DENSITY_SD, ATARI_DENSITY_ED,
                                   ATARI_DENSITY_DD};
  const uint8_t exp0[3] = {0x10, 0x90, 0x30};
  for (int i = 0; i < 3; i++) {
    Fix f(dens[i]);
    f.command('S', 0);
    f.run();
    expectReadReply(f, 4, "status");
    CHECK(f.b(2) == exp0[i] && f.b(3) == 0xFF && f.b(4) == 0xE0 && f.b(5) == 0);
  }
  Fix p;
  p.disk.wp = true;
  p.command('S', 0);
  p.run();
  CHECK(p.b(2) == 0x18);

  Fix e; // after a failed read
  e.disk.errSector = 5;
  e.command('R', 5);
  e.run();
  e.port.tx.clear();
  e.command('S', 0);
  e.run();
  CHECK(e.n() == 7 && e.b(2) == 0x14 && e.b(3) == 0xEF);
  e.disk.errResult = ATARI_DISK_CRC_ERROR;
  e.port.tx.clear();
  e.command('R', 5);
  e.run();
  e.port.tx.clear();
  e.command('S', 0);
  e.run();
  CHECK(e.b(3) == 0xF7);
  e.disk.isReady = false;
  e.port.tx.clear();
  e.command('S', 0);
  e.run();
  CHECK(e.b(3) == 0x7F);
}

static void test_read() {
  Fix f;
  f.command('R', 5);
  f.run();
  expectReadReply(f, 128, "read");
  bool match = true;
  for (unsigned i = 0; i < 128; i++)
    match = match && f.b(2 + i) == RamDisk::pat(5, i);
  CHECK(match);
  CHECK(f.sio.stats().frames == 1 && f.sio.stats().lastCommand == 'R' &&
        f.sio.stats().lastUnit == 1 && f.sio.stats().lastAux == 5);

  Fix d(ATARI_DENSITY_DD);
  d.command('R', 1);
  d.run();
  expectReadReply(d, 128, "dd boot");
  Fix d4(ATARI_DENSITY_DD);
  d4.command('R', 4);
  d4.run();
  expectReadReply(d4, 256, "dd 4");

  Fix o;
  o.command('R', 721);
  o.run();
  CHECK(o.n() == 1 && o.b(0) == 'N' && o.gap(0) >= 100);
  CHECK(o.sio.stats().naks == 1);
  Fix z;
  z.command('R', 0);
  z.run();
  CHECK(z.n() == 1 && z.b(0) == 'N');

  Fix x; // unsupported command
  x.command(0x99, 0);
  x.run();
  CHECK(x.n() == 1 && x.b(0) == 'N');

  Fix er; // read error: ACK, E, then full frame
  er.disk.errSector = 9;
  er.command('R', 9);
  er.run();
  CHECK(er.n() == 2 + 128 + 1 && er.b(0) == 'A' && er.b(1) == 'E');
  CHECK(er.gap(1) >= 300 && er.frameOk(2, 128));
  CHECK(er.sio.stats().errors == 1);
}

static void test_write() {
  uint8_t pay[128];
  for (int i = 0; i < 128; i++)
    pay[i] = (uint8_t)(0xA0 + i);

  Fix f;
  f.command('W', 12);
  f.dataFrame(f.deassert + 3000, pay, 128, 19040);
  f.run(120000);
  CHECK(f.n() == 3 && f.b(0) == 'A' && f.b(1) == 'A' && f.b(2) == 'C');
  CHECK(f.disk.writes == 1 && f.disk.lastWritten == 12 && f.disk.lastVerify);
  CHECK(memcmp(&f.disk.data[11 * RamDisk::kStride], pay, 128) == 0);
  CHECK(f.gap(1) >= 450 && f.gap(2) >= 300);

  Fix p; // put: no verify
  p.command('P', 12);
  p.dataFrame(p.deassert + 3000, pay, 128, 19040);
  p.run(120000);
  CHECK(p.n() == 3 && p.b(2) == 'C' && p.disk.writes == 1 &&
        !p.disk.lastVerify);

  Fix b; // bad data checksum
  b.command('W', 12);
  b.dataFrame(b.deassert + 3000, pay, 128, 19040, true);
  b.run(120000);
  CHECK(b.n() == 2 && b.b(0) == 'A' && b.b(1) == 'N' && b.disk.writes == 0);
  CHECK(b.gap(1) >= 450);
  CHECK(b.sio.stats().badChecksums == 1);

  Fix w; // write protected
  w.disk.wp = true;
  w.command('W', 12);
  w.dataFrame(w.deassert + 3000, pay, 128, 19040);
  w.run(120000);
  CHECK(w.n() == 3 && w.b(1) == 'A' && w.b(2) == 'E' && w.disk.writes == 0);

  Fix d(ATARI_DENSITY_DD); // 256-byte write, sector 10
  uint8_t big[256];
  for (int i = 0; i < 256; i++)
    big[i] = (uint8_t)i;
  d.command('W', 10);
  d.dataFrame(d.deassert + 3000, big, 256, 19040);
  d.run(120000);
  CHECK(d.n() == 3 && d.b(2) == 'C' &&
        memcmp(&d.disk.data[9 * RamDisk::kStride], big, 256) == 0);

  Fix t; // data-frame wait: COMMAND re-asserts, then a status command works
  t.command('W', 12);
  t.command('S', 0, 19040, 0x31, -1, t.deassert + 5000 - t.port.now);
  t.run(80000);
  CHECK(t.n() == 1 + 2 + 4 + 1 && t.b(0) == 'A' && t.b(1) == 'A' &&
        t.b(2) == 'C');
  CHECK(t.disk.writes == 0);
}

static void test_format() {
  Fix f;
  f.disk.data[0] = 0x77;
  f.command('!', 0);
  f.run();
  expectReadReply(f, 128, "format");
  bool ff = true;
  for (int i = 0; i < 128; i++)
    ff = ff && f.b(2 + i) == 0xFF;
  CHECK(ff && f.disk.formats == 1 && f.disk.data[0] == 0);

  Fix d(ATARI_DENSITY_DD);
  d.command('!', 0);
  d.run();
  expectReadReply(d, 256, "format dd");
  CHECK(d.disk.formats == 1 && d.disk.g.sector_size == 256);

  Fix e;
  e.command('"', 0);
  e.run();
  expectReadReply(e, 128, "format ed");
  CHECK(e.disk.formats == 1 && e.disk.g.sector_count == 1040 && e.b(2) == 0xFF);

  Fix w;
  w.disk.wp = true;
  w.command('!', 0);
  w.run();
  CHECK(w.n() == 2 + 128 + 1 && w.b(0) == 'A' && w.b(1) == 'E' &&
        w.frameOk(2, 128) && w.disk.formats == 0);

  Fix nr; // not ready: defaults to SD
  nr.disk.isReady = false;
  nr.command('!', 0);
  nr.run();
  CHECK(nr.n() > 1 && nr.b(1) == 'E');
}

static void test_percom() {
  const uint8_t sd[12] = {0x28, 0, 0, 0x12, 0, 0, 0, 0x80, 0xFF, 0, 0, 0};
  const uint8_t ed[12] = {0x28, 0, 0, 0x1A, 0, 4, 0, 0x80, 0xFF, 0, 0, 0};
  const uint8_t dd[12] = {0x28, 0, 0, 0x12, 0, 4, 1, 0x00, 0xFF, 0, 0, 0};
  const uint8_t *exp[3] = {sd, ed, dd};
  const atari_density_t dens[3] = {ATARI_DENSITY_SD, ATARI_DENSITY_ED,
                                   ATARI_DENSITY_DD};
  for (int i = 0; i < 3; i++) {
    Fix f(dens[i]);
    f.command('N', 0);
    f.run();
    expectReadReply(f, 12, "percom");
    bool m = f.n() == 15;
    for (int k = 0; m && k < 12; k++)
      m = f.b(2 + k) == exp[i][k];
    CHECK(m);
  }
  Fix big(ATARI_DENSITY_SD); // non-floppy image
  atari_geometry_t g = {256, 5000, 1, 1, 5000, true};
  big.disk.setGeo(g);
  big.command('N', 0);
  big.run();
  const uint8_t hd[12] = {1, 0, 0x13, 0x88, 0, 4, 1, 0, 0xFF, 0, 0, 0};
  bool m = big.n() == 15;
  for (int k = 0; m && k < 12; k++)
    m = big.b(2 + k) == hd[k];
  CHECK(m);

  // Accept: put ED PERCOM, then N and '!' use it.
  Fix a;
  a.command('O', 0);
  a.dataFrame(a.deassert + 3000, ed, 12, 19040);
  a.run(100000);
  CHECK(a.n() == 3 && a.b(0) == 'A' && a.b(1) == 'A' && a.b(2) == 'C');
  a.port.tx.clear();
  a.command('N', 0);
  a.run();
  bool m2 = a.n() == 15;
  for (int k = 0; m2 && k < 12; k++)
    m2 = a.b(2 + k) == ed[k];
  CHECK(m2);
  a.port.tx.clear();
  a.command('S', 0);
  a.run();
  CHECK(a.b(2) == 0x90); // status reports the pending format geometry
  a.port.tx.clear();
  a.command('!', 0);
  a.run();
  CHECK(a.disk.g.sector_count == 1040 && a.disk.formats == 1 && a.n() == 131);

  // Reject: 80 tracks.
  uint8_t bad[12];
  memcpy(bad, sd, 12);
  bad[0] = 80;
  Fix r;
  r.command('O', 0);
  r.dataFrame(r.deassert + 3000, bad, 12, 19040);
  r.run(100000);
  CHECK(r.n() == 3 && r.b(1) == 'A' && r.b(2) == 'E');
  r.port.tx.clear();
  r.command('N', 0);
  r.run();
  CHECK(r.b(2) == 0x28 && r.b(5) == 0x12); // geometry unchanged
}

static void test_status_geometries() {
  // Status byte 0: bit 4 motor, bit 5 non-128 sectors, bit 6 double-sided
  // (XF551, Altirra HRM table 79), bit 7 enhanced density.
  struct {
    uint16_t size, count;
    uint8_t exp;
  } cases[] = {{128, 720, 0x10},  {128, 1040, 0x90}, {256, 720, 0x30},
               {256, 1440, 0x70}, {256, 2880, 0x70}, {512, 1440, 0x70},
               {512, 720, 0x30},  {128, 1440, 0x50}};
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    Fix f;
    f.disk.setGeo(atari_geometry_for_sectors(cases[i].size, cases[i].count));
    f.command('S', 0);
    f.run();
    expectReadReply(f, 4, "status geo");
    CHECK(f.b(2) == cases[i].exp);
  }
}

static void test_percom_geometries() {
  struct {
    uint16_t size, count;
    uint8_t exp[8]; // tracks, 0, spt hi, spt lo, sides-1, density, size hi/lo
  } cases[] = {
      {256, 1440, {40, 0, 0, 18, 1, 4, 1, 0}},
      {256, 2880, {80, 0, 0, 18, 1, 4, 1, 0}},
      {512, 1440, {80, 0, 0, 9, 1, 4, 2, 0}},
      {512, 720, {80, 0, 0, 9, 0, 4, 2, 0}},
      {512, 2880, {80, 0, 0, 18, 1, 4, 2, 0}},
      {512, 65535, {1, 0, 0xFF, 0xFF, 0, 4, 2, 0}},
  };
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    Fix f;
    f.disk.setGeo(atari_geometry_for_sectors(cases[i].size, cases[i].count));
    f.command('N', 0);
    f.run();
    expectReadReply(f, 12, "percom geo");
    bool m = f.n() == 15;
    for (int k = 0; m && k < 8; k++)
      m = f.b(2 + k) == cases[i].exp[k];
    CHECK(m);
    CHECK(f.b(10) == 0xFF);
  }
}

static void test_512() {
  Fix f;
  f.disk.setGeo(atari_geometry_for_sectors(512, 1440));
  // Sectors 1-3 are full 512-byte sectors too.
  for (uint16_t s : {1, 3, 4, 1440}) {
    f.port.tx.clear();
    f.command('R', s);
    f.run();
    expectReadReply(f, 512, "512 read");
    CHECK(dataIs(f, s, 512));
  }
  f.port.tx.clear();
  f.command('R', 1441);
  f.run();
  CHECK(f.n() == 1 && f.b(0) == 'N');

  // Write sector 2 (512 bytes) and sector 1440.
  uint8_t pay[512];
  for (int i = 0; i < 512; i++)
    pay[i] = (uint8_t)(0x30 + i * 3);
  for (uint16_t s : {2, 1440}) {
    f.port.tx.clear();
    f.command('W', s);
    f.dataFrame(f.deassert + 3000, pay, 512, 19040);
    f.run(400000);
    CHECK(f.n() == 3 && f.b(0) == 'A' && f.b(1) == 'A' && f.b(2) == 'C');
    CHECK(f.disk.lastWritten == s &&
          memcmp(&f.disk.data[(s - 1u) * RamDisk::kStride], pay, 512) == 0);
  }

  // PERCOM round trip: get, put the same block, get again.
  f.port.tx.clear();
  f.command('N', 0);
  f.run();
  uint8_t pc[12];
  for (int k = 0; k < 12; k++)
    pc[k] = f.b(2 + k);
  CHECK(pc[6] == 2 && pc[7] == 0 && pc[0] == 80 && pc[3] == 9 && pc[4] == 1);
  f.port.tx.clear();
  f.command('O', 0);
  f.dataFrame(f.deassert + 3000, pc, 12, 19040);
  f.run(100000);
  CHECK(f.n() == 3 && f.b(2) == 'C');
  // Format with the pending 512-byte geometry: reply is 512 bytes of $FF.
  f.port.tx.clear();
  f.command('!', 0);
  f.run(400000);
  expectReadReply(f, 512, "format 512");
  CHECK(f.disk.formats == 1 && f.disk.g.sector_size == 512 &&
        f.disk.g.sector_count == 1440);
  bool ff = true;
  for (int i = 0; i < 512; i++)
    ff = ff && f.b(2 + i) == 0xFF;
  CHECK(ff);

  // Put a 720K 512-byte PERCOM on an SD disk, then status shows it.
  Fix s;
  uint8_t p2[12] = {80, 0, 0, 9, 1, 4, 2, 0, 0xFF, 0, 0, 0};
  s.command('O', 0);
  s.dataFrame(s.deassert + 3000, p2, 12, 19040);
  s.run(100000);
  CHECK(s.n() == 3 && s.b(2) == 'C');
  s.port.tx.clear();
  s.command('S', 0);
  s.run();
  CHECK(s.b(2) == 0x70);
  // 1024-byte sectors are rejected.
  Fix r;
  uint8_t p3[12] = {80, 0, 0, 9, 1, 4, 4, 0, 0xFF, 0, 0, 0};
  r.command('O', 0);
  r.dataFrame(r.deassert + 3000, p3, 12, 19040);
  r.run(100000);
  CHECK(r.n() == 3 && r.b(2) == 'E');
  // Bad sides byte rejected.
  p3[6] = 2;
  p3[4] = 5;
  Fix r2;
  r2.command('O', 0);
  r2.dataFrame(r2.deassert + 3000, p3, 12, 19040);
  r2.run(100000);
  CHECK(r2.n() == 3 && r2.b(2) == 'E');
}

static void test_protected_reads() {
  // Plain disks go through readSectorTimed with tAck = end of our ACK.
  Fix p;
  p.command('R', 5);
  p.run();
  expectReadReply(p, 128, "timed plain");
  CHECK(p.disk.timedCalls == 1 && p.disk.plainCalls == 1);
  CHECK(p.disk.lastNow == p.port.tx[0].end ||
        p.disk.lastNow >= p.port.tx[0].end);
  CHECK(p.disk.lastNow - p.port.tx[0].end < 200);

  // CRC error with send_data: 'E' then the disk's own data, status ~0x08.
  Fix c;
  c.disk.timedSector = 7;
  c.disk.timedInfo.fdc_status = ATARI_FDC_CRC_ERROR;
  c.disk.timedInfo.send_data = true;
  c.command('R', 7);
  c.run();
  CHECK(c.n() == 2 + 128 + 1 && c.b(0) == 'A' && c.b(1) == 'E');
  bool bad = true;
  for (int i = 0; i < 128; i++)
    bad = bad && c.b(2 + i) == 0xE5;
  CHECK(bad && c.frameOk(2, 128));
  CHECK(c.gap(1) >= 1000);
  c.port.tx.clear();
  c.command('S', 0);
  c.run();
  CHECK(c.n() == 7 && c.b(3) == 0xF7 && (c.b(2) & 0x04));

  // Same error without send_data: zeros as before.
  Fix z;
  z.disk.timedSector = 7;
  z.disk.timedInfo.fdc_status = ATARI_FDC_CRC_ERROR;
  z.command('R', 7);
  z.run();
  bool zero = z.n() == 131;
  for (int i = 0; zero && i < 128; i++)
    zero = z.b(2 + i) == 0;
  CHECK(zero);

  // Record-not-found style failure with a 200 ms mechanical delay.
  Fix d;
  d.disk.timedSector = 9;
  d.disk.timedResult = ATARI_DISK_NOT_FOUND;
  d.disk.timedInfo.fdc_status = ATARI_FDC_NOT_FOUND;
  d.disk.timedInfo.delay_us = 200000;
  d.command('R', 9);
  d.run(600000);
  CHECK(d.n() == 131 && d.b(0) == 'A' && d.b(1) == 'E');
  CHECK(d.gap(1) >= 200000 && d.gap(1) < 205000);
  CHECK(d.port.tx[1].start >= d.port.tx[0].end + 200000 - 1);
  d.port.tx.clear();
  d.command('S', 0);
  d.run();
  CHECK(d.b(3) == 0xEF);

  // Successful read with a delay still completes late and sends the data.
  Fix s;
  s.disk.timedSector = 3;
  s.disk.timedResult = ATARI_DISK_OK;
  s.disk.timedInfo.fdc_status = 0;
  s.disk.timedInfo.delay_us = 50000;
  s.command('R', 3);
  s.run(300000);
  CHECK(s.n() == 131 && s.b(1) == 'C' && s.gap(1) >= 50000);
  CHECK(s.b(2) == 0xE5);

  // COMMAND re-asserted during a long delay aborts the wait and a following
  // status command is served.
  Fix a;
  a.disk.timedSector = 9;
  a.disk.timedResult = ATARI_DISK_NOT_FOUND;
  a.disk.timedInfo.fdc_status = ATARI_FDC_NOT_FOUND;
  a.disk.timedInfo.delay_us = 3000000; // 3 s
  a.command('R', 9);
  uint32_t ackAt = a.deassert;
  a.command('S', 0, 19040, 0x31, -1, ackAt + 100000 - a.port.now);
  a.run(400000);
  CHECK(a.n() == 1 + 2 + 4 + 1 && a.b(0) == 'A' && a.b(1) == 'A' &&
        a.b(2) == 'C');
  CHECK(a.sio.stats().errors == 0);
  CHECK(a.port.tx[1].start < ackAt + 200000);

  // Not-ready disk: status byte 1 is $7F via the default wrapper.
  Fix n;
  n.disk.isReady = false;
  n.command('S', 0);
  n.run();
  CHECK(n.b(3) == 0x7F);

  // I/O error: ~(LOST_DATA | NOT_FOUND) = $EB.
  Fix io;
  io.disk.errSector = 4;
  io.disk.errResult = ATARI_DISK_IO_ERROR;
  io.command('R', 4);
  io.run();
  io.port.tx.clear();
  io.command('S', 0);
  io.run();
  CHECK(io.b(3) == 0xEB);
}

static void test_highspeed() {
  // '?' returns the divisor (default 10) as a one-byte frame.
  Fix f;
  f.command('?', 0);
  f.run();
  CHECK(f.n() == 4 && f.b(0) == 'A' && f.b(1) == 'C' && f.b(2) == 10 &&
        f.b(3) == 10);
  // The next command at 52640 baud: first frame is lost (device still at
  // standard speed), autobaud toggles, the retry is served at high speed.
  f.port.tx.clear();
  f.command('S', 0, 52640);
  f.run();
  CHECK(f.n() == 0 && f.sio.stats().highSpeed);
  f.command('S', 0, 52640);
  f.run();
  CHECK(f.n() == 7 && f.b(0) == 'A' && f.b(1) == 'C' && f.frameOk(2, 4));
  CHECK(f.port.tx[0].baud == 52640 && f.port.tx[6].baud == 52640);
  CHECK(f.gap(0) >= 100 && f.gap(1) >= 300);
  // Atari back to standard speed (reset): one frame lost, then fine.
  f.port.tx.clear();
  f.command('S', 0, 19040);
  f.run();
  CHECK(f.n() == 0 && !f.sio.stats().highSpeed);
  f.command('S', 0, 19040);
  f.run();
  CHECK(f.n() == 7 && f.port.tx[0].baud == 19040);

  Fix off; // Ultra Speed disabled: NAK '?', no autobaud
  off.sio.setUltraSpeed(0);
  off.command('?', 0);
  off.run();
  CHECK(off.n() == 1 && off.b(0) == 'N');
  off.port.tx.clear();
  off.command('S', 0, 52640);
  off.run();
  off.command('S', 0, 19040);
  off.run();
  CHECK(off.n() == 7);
}

static void test_xf551() {
  Fix f;
  f.command(0xD2, 5, 19040);
  f.run();
  expectReadReply(f, 128, "xf551 read");
  CHECK(f.port.tx[0].baud == 19040 && f.port.tx[1].baud == 38908 &&
        f.port.tx[2].baud == 38908 && f.port.tx[130].baud == 38908);
  CHECK(f.gap(1) >= 1000);
  CHECK(f.port.baud == 19040); // restored
  bool match = true;
  for (unsigned i = 0; i < 128; i++)
    match = match && f.b(2 + i) == RamDisk::pat(5, i);
  CHECK(match);

  Fix s;
  s.command(0xD3, 0);
  s.run();
  CHECK(s.n() == 7 && s.b(4) == 0xFE);

  uint8_t pay[128];
  memset(pay, 0x5A, sizeof(pay));
  Fix w; // XF551 write: data frame arrives at the high rate
  w.command(0xD7, 3);
  w.dataFrame(w.deassert + 3000, pay, 128, 38908);
  w.run(120000);
  CHECK(w.n() == 3 && w.b(2) == 'C' && w.disk.writes == 1);
  CHECK(w.port.tx[0].baud == 19040 && w.port.tx[1].baud == 38908);
  CHECK(w.gap(2) >= 1000 && w.port.baud == 19040);

  Fix off;
  off.sio.setXF551HighSpeed(false);
  off.command(0xD2, 5);
  off.run();
  CHECK(off.n() == 1 && off.b(0) == 'N');
}

static void test_rx_buffer_handling() {
  // Bytes on DATA OUT while COMMAND is high (another device's traffic) are
  // dropped and do not corrupt our next frame.
  Fix f;
  for (int i = 0; i < 4; i++) {
    RxByte r = {f.port.now + 200 + (uint32_t)i * 530, (uint8_t)(0xA0 + i),
                19040};
    f.port.rx.push_back(r);
  }
  f.command('S', 0); // COMMAND asserts 3000 us from now, after the strays
  f.run();
  CHECK(f.n() > 0 && f.b(0) == 'A');

  // poll() entered late, with part of the frame already buffered: those
  // bytes belong to the frame and must be kept.
  Fix g;
  g.command('S', 0);
  g.port.now = g.deassert - 850 - 2 * bitDur10(19040); // 3 of 5 bytes in
  g.run();
  CHECK(g.n() > 0 && g.b(0) == 'A');
}

int main() {
  test_checksum_and_framing();
  test_rx_buffer_handling();
  test_status();
  test_read();
  test_write();
  test_format();
  test_percom();
  test_status_geometries();
  test_percom_geometries();
  test_512();
  test_protected_reads();
  test_highspeed();
  test_xf551();
  printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
