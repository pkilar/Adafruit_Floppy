// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
// Host tests for Adafruit_AtariATX / Adafruit_AtariATXWriter over a
// memory-backed file.
#include "Adafruit_AtariATX.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

static int failures = 0;
static int checks = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    checks++;                                                                  \
    if (!(c)) {                                                                \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                      \
      failures++;                                                              \
    }                                                                          \
  } while (0)

class MemFile : public Adafruit_AtariFile {
public:
  std::vector<uint8_t> data;
  size_t pos;
  bool writable;
  int syncs;
  explicit MemFile(bool w = true) : pos(0), writable(w), syncs(0) {}
  bool seek(uint32_t p) override {
    pos = p;
    return true;
  }
  int read(void *buf, size_t len) override {
    if (pos >= data.size())
      return 0;
    size_t n = data.size() - pos;
    if (n > len)
      n = len;
    memcpy(buf, &data[pos], n);
    pos += n;
    return (int)n;
  }
  int write(const void *buf, size_t len) override {
    if (!writable)
      return -1;
    if (pos + len > data.size())
      data.resize(pos + len, 0);
    memcpy(&data[pos], buf, len);
    pos += len;
    return (int)len;
  }
  uint32_t size() override { return (uint32_t)data.size(); }
  bool truncate(uint32_t len) override {
    if (!writable)
      return false;
    data.resize(len, 0);
    return true;
  }
  bool sync() override {
    syncs++;
    return true;
  }
  bool isWritable() override { return writable; }
};

typedef std::vector<uint8_t> Bytes;
static void p8(Bytes &b, unsigned v) { b.push_back((uint8_t)v); }
static void p16(Bytes &b, unsigned v) {
  p8(b, v);
  p8(b, v >> 8);
}
static void p32(Bytes &b, uint32_t v) {
  p16(b, v & 0xFFFF);
  p16(b, v >> 16);
}
static void pz(Bytes &b, size_t n) { b.resize(b.size() + n, 0); }

static bool near(uint32_t a, double b, double tol) {
  double d = (double)a - b;
  return d < tol && d > -tol;
}

static uint8_t dat(int k, int j, int variant) {
  return (uint8_t)(k * 37 + j * 3 + 5 + variant);
}
static void sector_data(int k, int variant, uint8_t *out, int n) {
  for (int j = 0; j < n; j++)
    out[j] = dat(k, j, variant);
}

// Offsets inside the hand-built image.
static const uint32_t T0 = 48;      // track 0 record
static const uint32_t T0_LIST = 32; // sector list chunk, relative to T0
static const uint32_t T0_LEN = 912;
static const uint32_t T1 = 980;  // track 1 record
static const uint32_t T3 = 1172; // track 3 record (empty)

// A single-density image built byte by byte following spec_atx.md section A,
// independent of the writer. Track 0 holds seven entries in file (not
// position) order; sector 1 (list index 0/1) exists twice.
static Bytes build_sd(int variant) {
  Bytes b;
  // File header, 48 bytes.
  b.push_back('A');
  b.push_back('T');
  b.push_back('8');
  b.push_back('X');
  p16(b, 1);    // version
  p16(b, 1);    // min version
  p16(b, 0);    // creator
  p16(b, 0);    // creator version
  p32(b, 0);    // flags
  p16(b, 0);    // image type
  p8(b, 0);     // density: single
  p8(b, 0);     // reserved
  p32(b, 0);    // image id
  p16(b, 0);    // image version
  p16(b, 0);    // reserved
  p32(b, 0x30); // start
  p32(b, 1212); // end
  pz(b, 12);
  CHECK(b.size() == T0);

  // Track 0 record.
  p32(b, T0_LEN);
  p16(b, 0x0000);
  p16(b, 0);
  p8(b, 0);  // track
  p8(b, 0);  // reserved
  p16(b, 7); // sector count
  p16(b, 0); // rate
  p16(b, 0); // reserved
  p32(b, 0); // flags
  p32(b, 0x20);
  pz(b, 8);
  // Sector list chunk.
  p32(b, 8 + 7 * 8);
  p8(b, 0x01);
  p8(b, 0);
  p16(b, 0);
  struct E {
    uint8_t num, st;
    uint16_t pos;
    int k; // data block, -1 none
  };
  const E e[7] = {{1, 0x00, 100, 0},   {1, 0x00, 13100, 1}, {2, 0x08, 3000, 2},
                  {3, 0x10, 5000, -1}, {4, 0x48, 8000, 3},  {5, 0x20, 9000, 4},
                  {6, 0x44, 10000, 5}};
  for (int i = 0; i < 7; i++) {
    p8(b, e[i].num);
    p8(b, e[i].st);
    p16(b, e[i].pos);
    p32(b, e[i].k < 0 ? 0 : 120 + 128 * e[i].k);
  }
  // Unknown chunk (skipped).
  p32(b, 16);
  p8(b, 0x55);
  p8(b, 0);
  p16(b, 0);
  pz(b, 8);
  CHECK(b.size() == T0 + 112);
  // Sector data chunk.
  p32(b, 8 + 6 * 128);
  p8(b, 0x00);
  p8(b, 0);
  p16(b, 0);
  for (int k = 0; k < 6; k++)
    for (int j = 0; j < 128; j++)
      p8(b, dat(k, j, variant));
  // Weak chunk for list index 4 (sector 4): bytes from 100 are random.
  p32(b, 8);
  p8(b, 0x10);
  p8(b, 4);
  p16(b, 100);
  // Extended chunk for list index 6 (sector 6): size code 1.
  p32(b, 8);
  p8(b, 0x11);
  p8(b, 6);
  p16(b, 1);
  pz(b, 8); // terminator
  CHECK(b.size() == T0 + T0_LEN);

  // Host record (skipped).
  p32(b, 20);
  p16(b, 0x0100);
  p16(b, 0);
  for (int i = 0; i < 12; i++)
    p8(b, 0xEE);
  CHECK(b.size() == T1);

  // Track 1: sector 19 = ID 1.
  p32(b, 192);
  p16(b, 0);
  p16(b, 0);
  p8(b, 1);
  p8(b, 0);
  p16(b, 1);
  p16(b, 0);
  p16(b, 0);
  p32(b, 0);
  p32(b, 0x20);
  pz(b, 8);
  p32(b, 16);
  p8(b, 0x01);
  p8(b, 0);
  p16(b, 0);
  p8(b, 1);
  p8(b, 0);
  p16(b, 0);
  p32(b, 56);
  p32(b, 8 + 128);
  p8(b, 0x00);
  p8(b, 0);
  p16(b, 0);
  for (int j = 0; j < 128; j++)
    p8(b, dat(6, j, variant));
  pz(b, 8);
  CHECK(b.size() == T3);

  // Track 3: empty (no sectors, terminator only).
  p32(b, 40);
  p16(b, 0);
  p16(b, 0);
  p8(b, 3);
  p8(b, 0);
  p16(b, 0);
  p16(b, 0);
  p16(b, 0);
  p32(b, 0);
  p32(b, 0x20);
  pz(b, 8);
  pz(b, 8);
  CHECK(b.size() == 1212);
  return b;
}

static MemFile *make_file(const Bytes &b) {
  MemFile *f = new MemFile();
  f->data = b;
  return f;
}

static atari_disk_result_t rd(Adafruit_AtariATX &a, uint16_t sector,
                              uint32_t now, uint8_t *buf,
                              atari_read_info_t *info) {
  memset(buf, 0xEE, 256);
  return a.readSectorTimed(sector, buf, now, info);
}

static void test_hand_image() {
  Bytes img = build_sd(0);
  MemFile *f = make_file(img);
  Adafruit_AtariATX a;
  CHECK(!a.ready() && !a.mounted());
  CHECK(a.mount(f));
  CHECK(a.ready() && a.mounted() && a.writeProtected());
  atari_geometry_t g = a.geometry();
  CHECK(atari_density_of(&g) == ATARI_DENSITY_SD);
  CHECK(!a.canFormat(g));
  uint8_t w[128] = {0};
  CHECK(a.writeSector(1, w, true) == ATARI_DISK_WRITE_PROTECTED);
  CHECK(a.format(g) == ATARI_DISK_WRITE_PROTECTED);

  atari_prot_track_t *t = new atari_prot_track_t;
  CHECK(a.loadTrack(0, t));
  CHECK(t->sector_count == 7 && t->density == ATARI_DENSITY_SD &&
        t->track == 0);
  const uint8_t nums[7] = {1, 2, 3, 4, 5, 6, 1};
  const uint16_t poss[7] = {100, 3000, 5000, 8000, 9000, 10000, 13100};
  const uint8_t sts[7] = {0x00, 0x08, 0x10, 0x48, 0x20, 0x44, 0x00};
  const int ks[7] = {0, 2, -1, 3, 4, 5, 1};
  for (int i = 0; i < 7; i++) {
    const atari_prot_sector_t &s = t->sectors[i];
    CHECK(s.number == nums[i]);
    CHECK(s.position == poss[i]);
    CHECK(s.status == sts[i]);
    CHECK(s.size_code == (nums[i] == 6 ? 1 : 0));
    CHECK(s.weak_offset == (nums[i] == 4 ? 100 : ATARI_PROT_NONE));
    if (ks[i] < 0) {
      CHECK(s.data_offset == ATARI_PROT_NONE);
    } else {
      uint8_t exp[128];
      sector_data(ks[i], 0, exp, 128);
      CHECK(s.data_offset != ATARI_PROT_NONE &&
            memcmp(t->data + s.data_offset, exp, 128) == 0);
    }
  }
  CHECK(t->data_bytes == 6 * 128);
  CHECK(a.loadTrack(1, t) && t->sector_count == 1 && t->sectors[0].number == 1);
  CHECK(a.loadTrack(3, t) && t->sector_count == 0);
  CHECK(!a.loadTrack(2, t));
  CHECK(!a.loadTrack(80, t));
  delete t;

  // Duplicate selection by time. At now = 0 the head is at unit ~402 when the
  // command reaches the FDC: the copy at 13100 is next. One revolution minus
  // the latency later the head is at unit 0: the copy at 100 is next.
  uint8_t buf[256], da[128], db[128];
  sector_data(0, 0, da, 128);
  sector_data(1, 0, db, 128);
  atari_read_info_t info;
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    CHECK(rd(x, 1, 0, buf, &info) == ATARI_DISK_OK);
    CHECK(memcmp(buf, db, 128) == 0 && info.fdc_status == 0 && !info.send_data);
    // request + wait (12698 units of 8 us) + one slot + CRC
    CHECK(near(info.delay_us,
               3220 + 12698 * 208333.0 / 26042 + 208333.0 / 18 + 2000, 20));
  }
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    CHECK(rd(x, 1, ATARI_ATX_REV_US - 3220, buf, &info) == ATARI_DISK_OK);
    CHECK(memcmp(buf, da, 128) == 0);
    CHECK(near(info.delay_us,
               3220 + 100 * 208333.0 / 26042 + 208333.0 / 18 + 2000, 20));
  }
  // The same sector read again later by the same unit follows the clock:
  // 10 ms after the first read the head is still before the copy at 13100.
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    CHECK(rd(x, 1, 0, buf, &info) == ATARI_DISK_OK);
    CHECK(rd(x, 1, 105000, buf, &info) == ATARI_DISK_OK); // head ~ half way
    CHECK(memcmp(buf, db, 128) == 0 || memcmp(buf, da, 128) == 0);
    CHECK(rd(x, 1, 105000 + 100000, buf, &info) == ATARI_DISK_OK);
  }

  // CRC error: data still sent, status $08, a firmware retry adds a revolution
  // on a 1050 and three on an 810.
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    uint8_t d2[128];
    sector_data(2, 0, d2, 128);
    CHECK(rd(x, 2, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(info.fdc_status == 0x08 && info.send_data);
    CHECK(memcmp(buf, d2, 128) == 0);
    CHECK(near(info.delay_us,
               3220 + 2598 * 208333.0 / 26042 + 208333.0 / 18 + 2000 + 208333.0,
               20));
    Adafruit_AtariATX y;
    y.setDriveModel(ATARI_DRIVE_810);
    CHECK(y.mount(f));
    CHECK(rd(y, 2, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(near(info.delay_us,
               3220 + 2598 * 208333.0 / 26042 + 208333.0 / 18 + 2000 +
                   3 * 208333.0,
               20));
  }
  // Deleted: $20 on the 1050 (status byte $DF), $60 on the 810 ($9F).
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    uint8_t d5[128];
    sector_data(4, 0, d5, 128);
    CHECK(rd(x, 5, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(info.fdc_status == 0x20 && info.send_data &&
          memcmp(buf, d5, 128) == 0);
    // Deleted marks are not retried: no extra revolution.
    CHECK(near(info.delay_us,
               3220 + 3598 * 208333.0 / 26042 + 208333.0 / 18 + 2000, 20) ||
          info.delay_us < 3220 + ATARI_ATX_REV_US + 208333 / 18 + 2000);
    Adafruit_AtariATX y;
    y.setDriveModel(ATARI_DRIVE_810);
    CHECK(y.mount(f));
    CHECK(rd(y, 5, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(info.fdc_status == 0x60);
    // Long sector: first 128 bytes, lost data + DRQ on the 1050.
    CHECK(rd(x, 6, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(info.fdc_status == 0x06 && info.send_data);
  }
  // Weak bits: bytes before the offset are stable, the rest change per read.
  {
    Adafruit_AtariATX x;
    x.seedRandom(1234);
    CHECK(x.mount(f));
    uint8_t s4[128], r1[256], r2[256];
    sector_data(3, 0, s4, 128);
    CHECK(rd(x, 4, 0, r1, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(info.fdc_status == 0x08 && info.send_data);
    CHECK(rd(x, 4, 400000, r2, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(memcmp(r1, s4, 100) == 0 && memcmp(r2, s4, 100) == 0);
    CHECK(memcmp(r1 + 100, r2 + 100, 28) != 0);
    CHECK(memcmp(r1 + 100, s4 + 100, 28) != 0);
    int same = 0;
    for (int i = 100; i < 128; i++)
      same += r1[i] == r2[i];
    CHECK(same < 10); // independent per-byte randomness
  }
  // Record not found: previous data returned, a long delay.
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    uint8_t d19[128];
    sector_data(6, 0, d19, 128);
    CHECK(rd(x, 19, 0, buf, &info) == ATARI_DISK_OK);
    CHECK(memcmp(buf, d19, 128) == 0);
    // seek (1 track) + settle + a wait + slot + CRC
    CHECK(info.delay_us >= 3220 + 12410 + 40000 + 11574 + 2000);
    CHECK(info.delay_us <
          3220 + 12410 + 40000 + 11574 + 2000 + ATARI_ATX_REV_US);
    // Track 2 does not exist: seek one track, give up after 2 x 5 rev.
    CHECK(rd(x, 37, 5000000, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.fdc_status == ATARI_FDC_NOT_FOUND && info.send_data);
    CHECK(memcmp(buf, d19, 128) == 0);
    CHECK(info.delay_us ==
          3220 + 12410 + 40000 + 10 * (uint32_t)ATARI_ATX_REV_US);
    // Sector 3 has an ID but no data field: same.
    CHECK(rd(x, 3, 9000000, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.send_data && memcmp(buf, d19, 128) == 0);
    CHECK(info.delay_us ==
          3220 + 2 * 12410 + 40000 + 10 * (uint32_t)ATARI_ATX_REV_US);
    // Same track again: no seek or settle.
    CHECK(rd(x, 3, 20000000, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.delay_us == 3220 + 10 * (uint32_t)ATARI_ATX_REV_US);
    Adafruit_AtariATX y;
    y.setDriveModel(ATARI_DRIVE_810);
    CHECK(y.mount(f));
    CHECK(rd(y, 37, 0, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.delay_us == 3220 + 2 * 5300 + 20 * (uint32_t)ATARI_ATX_REV_US);
  }
  // Seek timing without a settle on the 810, and no seek on the same track.
  {
    Adafruit_AtariATX y;
    y.setDriveModel(ATARI_DRIVE_810);
    CHECK(y.mount(f));
    CHECK(rd(y, 19, 0, buf, &info) == ATARI_DISK_OK);
    CHECK(info.delay_us >= 3220 + 5300 + 11574 + 2000);
    CHECK(info.delay_us < 3220 + 5300 + 11574 + 2000 + ATARI_ATX_REV_US);
    CHECK(rd(y, 19, 3000000, buf, &info) == ATARI_DISK_OK);
    CHECK(info.delay_us >= 3220 + 11574 + 2000);
    CHECK(info.delay_us < 3220 + 11574 + 2000 + ATARI_ATX_REV_US);
  }
  // Out of range sectors fail fast.
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    CHECK(rd(x, 0, 0, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.delay_us == 0 && !info.send_data);
    CHECK(rd(x, 721, 0, buf, &info) == ATARI_DISK_NOT_FOUND);
    CHECK(info.delay_us == 0 && !info.send_data);
  }
  // readSector() with the internal clock.
  {
    Adafruit_AtariATX x;
    CHECK(x.mount(f));
    uint8_t d19[128];
    sector_data(6, 0, d19, 128);
    CHECK(x.readSector(19, buf) == ATARI_DISK_OK);
    CHECK(memcmp(buf, d19, 128) == 0);
    CHECK(x.readSector(3, buf) == ATARI_DISK_NOT_FOUND);
    CHECK(x.readSector(2, buf) == ATARI_DISK_CRC_ERROR);
    CHECK(!x.ready() == false);
    x.unmount();
    CHECK(!x.ready());
    CHECK(x.readSector(1, buf) == ATARI_DISK_NOT_READY);
  }
  delete f;
}

static void test_clock_wrap() {
  MemFile *f = make_file(build_sd(0));
  Adafruit_AtariATX a, b;
  CHECK(a.mount(f) && b.mount(f));
  uint32_t a0 = 0xFFFFF000u;
  uint32_t b0 = a0 % ATARI_ATX_REV_US; // same phase, no wrap
  uint8_t ba[256], bb[256];
  atari_read_info_t ia, ib;
  CHECK(rd(a, 1, a0, ba, &ia) == ATARI_DISK_OK);
  CHECK(rd(b, 1, b0, bb, &ib) == ATARI_DISK_OK);
  CHECK(ia.delay_us == ib.delay_us && memcmp(ba, bb, 128) == 0);
  // 50 ms later a's 32-bit clock has wrapped (0xFFFFF000 + 50000 > 2^32).
  uint32_t a1 = a0 + 50000;
  CHECK(a1 < a0);
  CHECK(rd(a, 1, a1, ba, &ia) == ATARI_DISK_OK);
  CHECK(rd(b, 1, b0 + 50000, bb, &ib) == ATARI_DISK_OK);
  CHECK(ia.delay_us == ib.delay_us && memcmp(ba, bb, 128) == 0);
  for (int i = 0; i < 6; i++) {
    a1 += 70001;
    b0 += 70001;
    CHECK(rd(a, 1, a1, ba, &ia) == ATARI_DISK_OK);
    CHECK(rd(b, 1, b0 + 50000, bb, &ib) == ATARI_DISK_OK);
    CHECK(ia.delay_us == ib.delay_us && memcmp(ba, bb, 128) == 0);
  }
  delete f;
}

static void test_shared_cache() {
  MemFile *f1 = make_file(build_sd(0));
  MemFile *f2 = make_file(build_sd(50));
  Adafruit_AtariATX a, b;
  CHECK(a.mount(f1) && b.mount(f2));
  uint8_t d1[128], d2[128], buf[256];
  sector_data(2, 0, d1, 128);
  sector_data(2, 50, d2, 128);
  atari_read_info_t info;
  for (int i = 0; i < 4; i++) {
    CHECK(rd(a, 2, 1000000u * i, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(memcmp(buf, d1, 128) == 0);
    CHECK(rd(b, 2, 1000000u * i, buf, &info) == ATARI_DISK_CRC_ERROR);
    CHECK(memcmp(buf, d2, 128) == 0);
  }
  // Track 1 through the cache from both, then back to track 0.
  sector_data(6, 0, d1, 128);
  sector_data(6, 50, d2, 128);
  CHECK(rd(a, 19, 9000000, buf, &info) == ATARI_DISK_OK);
  CHECK(memcmp(buf, d1, 128) == 0);
  CHECK(rd(b, 19, 9000000, buf, &info) == ATARI_DISK_OK);
  CHECK(memcmp(buf, d2, 128) == 0);
  // loadTrack uses the same cache.
  atari_prot_track_t *ta = new atari_prot_track_t, *tb = new atari_prot_track_t;
  CHECK(a.loadTrack(0, ta) && b.loadTrack(0, tb));
  CHECK(ta->sector_count == 7 && tb->sector_count == 7);
  CHECK(ta->data[ta->sectors[0].data_offset] == dat(0, 0, 0));
  CHECK(tb->data[tb->sectors[0].data_offset] == dat(0, 0, 50));
  delete ta;
  delete tb;
  // Remounting the same unit on another file drops its cached track.
  a.unmount();
  CHECK(a.mount(f2));
  sector_data(2, 50, d2, 128);
  CHECK(rd(a, 2, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
  CHECK(memcmp(buf, d2, 128) == 0);
  delete f1;
  delete f2;
}

static void test_malformed() {
  Bytes good = build_sd(0);
  atari_prot_track_t *t = new atari_prot_track_t;
  uint8_t buf[256];
  atari_read_info_t info;
  {
    Adafruit_AtariATX x;
    MemFile *f = make_file(good);
    CHECK(x.mount(f));
    delete f;
  }
  for (int kind = 0; kind < 12; kind++) {
    Bytes b = good;
    bool expect = false;
    switch (kind) {
    case 0: // bad signature
      b[0] = 'X';
      break;
    case 1: // version 0
      b[4] = 0;
      break;
    case 2: // minimum version 2
      b[6] = 2;
      break;
    case 3: // unknown density
      b[0x12] = 3;
      break;
    case 4: // truncated in the last record
      b.resize(b.size() - 10);
      break;
    case 5: // shorter than the header
      b.resize(20);
      break;
    case 6: // start past the end of the file
      b[0x1C] = 0xFF;
      b[0x1D] = 0xFF;
      break;
    case 7: // record length below the record header
      b[T0] = 4;
      b[T0 + 1] = 0;
      break;
    case 8: // record length reaching past the end of the file
      b[T0 + 2] = 0x10;
      break;
    case 9: // header only: no tracks
      b.resize(48);
      b[0x1C] = 0x30;
      break;
    case 10: // zero-length record would loop forever
      b[T1] = b[T1 + 1] = 0;
      break;
    case 11: // trailing bytes shorter than a record header are ignored
      b.push_back(0);
      b.push_back(1);
      expect = true;
      break;
    }
    MemFile *f = make_file(b);
    Adafruit_AtariATX x;
    bool ok = x.mount(f);
    if (ok != expect)
      printf("malformed case %d mounted=%d\n", kind, ok);
    CHECK(ok == expect);
    CHECK(x.ready() == expect);
    if (!ok)
      CHECK(x.readSectorTimed(1, buf, 0, &info) == ATARI_DISK_NOT_READY);
    delete f;
  }

  // Mountable files whose track 0 record is damaged: the track is unreadable
  // (every sector not found) but nothing crashes and other tracks still work.
  struct Mut {
    uint32_t off;
    uint8_t v0, v1;
    bool track_ok;
  };
  const Mut muts[] = {
      {T0 + 8 + 2, 49, 0, false},       // too many sectors
      {T0 + 8 + 12, 0xFF, 0xFF, false}, // header size past the record
      {T0 + 8 + 12, 0x10, 0, false},    // header size inside the header
      {T0 + T0_LIST + 8 + 4, 0xFF, 0xFF, false}, // sector 1 data offset huge
      {T0 + T0_LIST + 8 + 4, 0x10, 0, false}, // data offset inside the header
      {T0 + T0_LIST + 8 + 8 * 6 + 4, 0xE8, 0x03,
       false},                           // last data offset past the record end
      {T0 + T0_LIST, 0xFF, 0xFF, false}, // list chunk longer than record
      {T0 + T0_LIST, 0x08, 0x00, false}, // list chunk too short for 7
      {T0 + 8, 9, 0, false},             // track number mismatch
      {T0 + 112 + 24 * 0 + 8 + 768 + 5, 200, 0, true}, // weak idx out of range
  };
  for (unsigned m = 0; m < sizeof(muts) / sizeof(muts[0]); m++) {
    Bytes b = good;
    b[muts[m].off] = muts[m].v0;
    b[muts[m].off + 1] = muts[m].v1;
    MemFile *f = make_file(b);
    Adafruit_AtariATX x;
    bool mounted = x.mount(f);
    if (mounted) {
      bool ok = x.loadTrack(0, t);
      if (ok != muts[m].track_ok)
        printf("mutation %u loadTrack=%d\n", m, ok);
      CHECK(ok == muts[m].track_ok);
      if (!muts[m].track_ok) {
        CHECK(x.readSectorTimed(1, buf, 0, &info) == ATARI_DISK_NOT_FOUND);
        CHECK(info.send_data && info.fdc_status == ATARI_FDC_NOT_FOUND);
        CHECK(x.lastTrackBad() == (m != 8)); // 8: track simply absent
        CHECK(x.loadTrack(1, t));            // track 1 unaffected
      }
    } else {
      CHECK(mounted);
    }
    delete f;
  }
  delete t;
}

static void fill_track(atari_prot_track_t *t, uint8_t track, atari_density_t d,
                       int variant) {
  const uint16_t bytes = atari_prot_sector_bytes(d);
  const uint8_t nat = d == ATARI_DENSITY_DD ? 1 : 0;
  atari_prot_track_init(t, track, 0, d);
  struct S {
    uint8_t num, st;
    uint16_t pos, weak;
    uint8_t code;
  };
  const S s[6] = {
      {1, 0x00, 500, ATARI_PROT_NONE, nat},
      {2, 0x08, 4000, 10, nat},                              // weak
      {3, 0x10, 9000, ATARI_PROT_NONE, nat},                 // ID without data
      {2, 0x00, 15000, ATARI_PROT_NONE, nat},                // duplicate
      {4, 0x04, 20000, ATARI_PROT_NONE, (uint8_t)(nat + 1)}, // long
      {26, 0x20, 25000, ATARI_PROT_NONE, nat},               // deleted, odd ID
  };
  for (int i = 0; i < 6; i++) {
    atari_prot_sector_t e;
    e.number = s[i].num;
    e.status = s[i].st;
    e.position = s[i].pos;
    e.weak_offset = s[i].weak;
    e.size_code = s[i].code;
    if (s[i].st & ATARI_ATX_MISSING) {
      e.data_offset = ATARI_PROT_NONE;
    } else {
      e.data_offset = t->data_bytes;
      for (int j = 0; j < bytes; j++)
        t->data[t->data_bytes + j] = dat(i + track, j, variant);
      t->data_bytes = (uint16_t)(t->data_bytes + bytes);
    }
    t->sectors[t->sector_count++] = e;
  }
}

static void roundtrip(atari_density_t d) {
  const uint16_t bytes = atari_prot_sector_bytes(d);
  atari_prot_track_t *t0 = new atari_prot_track_t, *t5 = new atari_prot_track_t,
                     *te = new atari_prot_track_t, *r = new atari_prot_track_t;
  fill_track(t0, 0, d, 3);
  fill_track(t5, 5, d, 9);
  atari_prot_track_init(te, 1, 0, d);

  MemFile f;
  Adafruit_AtariATXWriter w;
  CHECK(w.begin(&f, d));
  CHECK(w.addTrack(*t0));
  CHECK(w.addTrack(*te));
  CHECK(w.addTrack(*t5));
  CHECK(w.finish());
  CHECK(f.syncs >= 1);
  const Bytes &b = f.data;
  CHECK(memcmp(&b[0], "AT8X", 4) == 0);
  CHECK(b[4] == 1 && b[5] == 0 && b[6] == 1 && b[7] == 0);
  CHECK(b[0x12] == (uint8_t)d);
  CHECK(b[0x1C] == 0x30 && b[0x1D] == 0 && b[0x1E] == 0 && b[0x1F] == 0);
  uint32_t end =
      b[0x20] | (b[0x21] << 8) | (b[0x22] << 16) | ((uint32_t)b[0x23] << 24);
  CHECK(end == b.size());
  // First record: header size 0x20, MFM flag only on enhanced density.
  CHECK(b[0x30 + 4] == 0 && b[0x30 + 5] == 0); // track record
  CHECK(b[0x30 + 8] == 0 && b[0x30 + 10] == 6);
  CHECK(b[0x30 + 16] == (d == ATARI_DENSITY_ED ? 2 : 0));
  CHECK(b[0x30 + 20] == 0x20);
  // Sector list entry 1 (weak) carries the extended bit; entry 0 does not.
  CHECK(b[0x30 + 32 + 8 + 1] == 0x00);
  CHECK(b[0x30 + 32 + 8 + 8 + 1] == (0x08 | 0x40));
  CHECK(b[0x30 + 32 + 8 + 4 * 8 + 1] == (0x04 | 0x40)); // long sector

  Adafruit_AtariATX a;
  CHECK(a.mount(&f));
  atari_geometry_t g = a.geometry();
  CHECK(atari_density_of(&g) == d);
  atari_prot_track_t *src[2] = {t0, t5};
  uint8_t tn[2] = {0, 5};
  for (int k = 0; k < 2; k++) {
    CHECK(a.loadTrack(tn[k], r));
    CHECK(r->track == tn[k] && r->density == d);
    CHECK(r->sector_count == src[k]->sector_count);
    for (int i = 0; i < r->sector_count; i++) {
      const atari_prot_sector_t &x = r->sectors[i];
      const atari_prot_sector_t &y = src[k]->sectors[i];
      CHECK(x.number == y.number && x.position == y.position);
      CHECK((x.status & 0x3F) == y.status);
      CHECK(x.weak_offset == y.weak_offset && x.size_code == y.size_code);
      CHECK((x.data_offset == ATARI_PROT_NONE) ==
            (y.data_offset == ATARI_PROT_NONE));
      if (y.data_offset != ATARI_PROT_NONE)
        CHECK(memcmp(r->data + x.data_offset, src[k]->data + y.data_offset,
                     bytes) == 0);
    }
  }
  CHECK(a.loadTrack(1, r) && r->sector_count == 0 && r->data_bytes == 0);
  CHECK(!a.loadTrack(2, r));

  // Reading through the SIO view: ID 1 of track 0 is sector 1 (one entry).
  uint8_t buf[256], exp[256];
  atari_read_info_t info;
  CHECK(rd(a, 1, 0, buf, &info) == ATARI_DISK_OK);
  sector_data(0 + 0, 3, exp, bytes);
  CHECK(memcmp(buf, exp, atari_sector_bytes(&g, 1)) == 0);
  if (d == ATARI_DENSITY_DD)
    CHECK(atari_sector_bytes(&g, 1) == 128);
  // Duplicate ID 2: two different copies depending on time. Entries 1 (weak,
  // CRC error at 4000) and 2 (clean at 15000).
  int clean = 0, bad = 0;
  for (int i = 0; i < 20; i++) {
    Adafruit_AtariATX x;
    CHECK(x.mount(&f));
    atari_disk_result_t res = rd(x, 2, (uint32_t)i * 10000u, buf, &info);
    if (res == ATARI_DISK_OK)
      clean++;
    else if (res == ATARI_DISK_CRC_ERROR)
      bad++;
  }
  CHECK(clean > 0 && bad > 0 && clean + bad == 20);
  // Sector 3 (ID without data), the long sector (ID 4) and ID 26's track.
  CHECK(rd(a, 3, 0, buf, &info) == ATARI_DISK_NOT_FOUND);
  CHECK(rd(a, 4, 0, buf, &info) == ATARI_DISK_CRC_ERROR);
  CHECK(info.fdc_status == (ATARI_FDC_LOST_DATA | ATARI_FDC_DRQ));
  // Track 5 sector with ID 1.
  uint16_t spt = a.geometry().sectors_per_track;
  CHECK(rd(a, (uint16_t)(5 * spt + 1), 0, buf, &info) == ATARI_DISK_OK);
  sector_data(0 + 5, 9, exp, bytes);
  CHECK(memcmp(buf, exp, bytes) == 0);

  // Writer refusals.
  atari_prot_track_t bad_t = *t0;
  Adafruit_AtariATXWriter w2;
  MemFile f2;
  CHECK(w2.begin(&f2, d));
  bad_t.sector_count = ATARI_PROT_MAX_SECTORS + 1;
  CHECK(!w2.addTrack(bad_t));
  bad_t = *t0;
  bad_t.density = d == ATARI_DENSITY_SD ? ATARI_DENSITY_DD : ATARI_DENSITY_SD;
  CHECK(!w2.addTrack(bad_t));
  bad_t = *t0;
  bad_t.sectors[0].data_offset = ATARI_PROT_NONE; // data expected
  CHECK(!w2.addTrack(bad_t));
  bad_t = *t0;
  bad_t.track = 80;
  CHECK(!w2.addTrack(bad_t));

  delete t0;
  delete t5;
  delete te;
  delete r;
}

static void test_writer_misc() {
  MemFile ro(false);
  Adafruit_AtariATXWriter w;
  CHECK(!w.begin(&ro, ATARI_DENSITY_SD));
  CHECK(!w.finish());
  MemFile f;
  CHECK(!w.begin(&f, ATARI_DENSITY_OTHER));
  CHECK(!w.begin(nullptr, ATARI_DENSITY_SD));
  atari_prot_track_t *t = new atari_prot_track_t;
  atari_prot_track_init(t, 0, 0, ATARI_DENSITY_SD);
  CHECK(!w.addTrack(*t)); // no file
  CHECK(w.begin(&f, ATARI_DENSITY_SD));
  CHECK(w.addTrack(*t)); // empty track
  CHECK(w.finish());
  CHECK(f.size() == 48 + 40);
  Adafruit_AtariATX a;
  CHECK(a.mount(&f));
  atari_prot_track_t *r = new atari_prot_track_t;
  CHECK(a.loadTrack(0, r) && r->sector_count == 0);
  delete t;
  delete r;
}

int main() {
  test_hand_image();
  test_clock_wrap();
  test_shared_cache();
  test_malformed();
  roundtrip(ATARI_DENSITY_SD);
  roundtrip(ATARI_DENSITY_ED);
  roundtrip(ATARI_DENSITY_DD);
  test_writer_misc();
  if (failures) {
    printf("%d of %d checks failed\n", failures, checks);
    return 1;
  }
  printf("test_atari_atx: %d checks passed\n", checks);
  return 0;
}
