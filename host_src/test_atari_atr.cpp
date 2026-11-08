// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
// Host tests for Adafruit_AtariATR over a memory-backed file.
#include "Adafruit_AtariATR.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

static int failures = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
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

static void fill(uint8_t *b, size_t n, unsigned seed) {
  for (size_t i = 0; i < n; i++)
    b[i] = (uint8_t)(seed * 31 + i * 7 + 1);
}

static void test_density(atari_density_t d, uint32_t file_size) {
  atari_geometry_t g = atari_geometry_for_density(d);
  MemFile f;
  CHECK(Adafruit_AtariATR::create(&f, g));
  CHECK(f.size() == file_size);
  CHECK(f.data[0] == 0x96 && f.data[1] == 0x02);
  Adafruit_AtariATR atr;
  CHECK(!atr.ready());
  CHECK(atr.mount(&f));
  CHECK(atr.ready());
  atari_geometry_t m = atr.geometry();
  CHECK(atari_density_of(&m) == d);
  CHECK(!atr.writeProtected());
  uint16_t last = g.sector_count;
  uint16_t list[4] = {1, 3, 4, last};
  for (int i = 0; i < 4; i++) {
    uint8_t w[256], r[256];
    uint16_t n = atari_sector_bytes(&g, list[i]);
    fill(w, sizeof(w), list[i]);
    CHECK(atr.writeSector(list[i], w, true) == ATARI_DISK_OK);
    memset(r, 0xEE, sizeof(r));
    CHECK(atr.readSector(list[i], r) == ATARI_DISK_OK);
    CHECK(memcmp(r, w, n) == 0);
    CHECK(r[n] == 0xEE || n == 256);
  }
  // Byte-exact offsets.
  uint8_t w[256];
  fill(w, 256, 4);
  if (g.sector_size == 256) {
    CHECK(memcmp(&f.data[16 + 3 * 128], w, 256) == 0);
    fill(w, 256, last);
    CHECK(memcmp(&f.data[16 + 384 + (last - 4) * 256u], w, 256) == 0);
  } else {
    CHECK(memcmp(&f.data[16 + 3 * 128], w, 128) == 0);
  }
  fill(w, 256, 3);
  CHECK(memcmp(&f.data[16 + 2 * 128], w, 128) == 0);
  // Out of range.
  uint8_t r[256];
  CHECK(atr.readSector(0, r) == ATARI_DISK_NOT_FOUND);
  CHECK(atr.readSector(last + 1, r) == ATARI_DISK_NOT_FOUND);
  CHECK(atr.writeSector(last + 1, w, false) == ATARI_DISK_NOT_FOUND);
  CHECK(atr.flush() == ATARI_DISK_OK);
  CHECK(f.syncs > 0);
}

static void put_hdr(std::vector<uint8_t> &v, uint32_t data, uint16_t ss,
                    uint8_t f8 = 0, uint8_t f15 = 0) {
  v.assign(16, 0);
  uint32_t p = data >> 4;
  v[0] = 0x96;
  v[1] = 0x02;
  v[2] = p & 0xFF;
  v[3] = (p >> 8) & 0xFF;
  v[4] = ss & 0xFF;
  v[5] = ss >> 8;
  v[6] = (p >> 16) & 0xFF;
  v[7] = (p >> 24) & 0xFF;
  v[8] = f8;
  v[15] = f15;
}

// 512-byte (SpartaDOS X) images, double-sided 256-byte images, resizing format.
static void test_sdx() {
  // 512-byte image created, remounted and accessed: every sector is 512 bytes.
  {
    atari_geometry_t g = atari_geometry_for_sectors(512, 1440); // 720K
    CHECK(g.tracks == 80 && g.sides == 2 && g.sectors_per_track == 9);
    MemFile f;
    CHECK(Adafruit_AtariATR::create(&f, g));
    CHECK(f.size() == 16 + 1440u * 512u);
    CHECK(f.data[4] == 0x00 && f.data[5] == 0x02); // sector size 512
    uint32_t paras = (1440u * 512u) >> 4;
    CHECK(f.data[2] == (paras & 0xFF) && f.data[3] == ((paras >> 8) & 0xFF));
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t m = atr.geometry();
    CHECK(m.sector_size == 512 && m.sector_count == 1440 && m.tracks == 80 &&
          m.sides == 2 && m.sectors_per_track == 9 && m.mfm);
    uint8_t w[512], r[512];
    for (uint16_t s : {1, 2, 3, 4, 1440}) {
      fill(w, 512, s);
      CHECK(atr.writeSector(s, w, true) == ATARI_DISK_OK);
      CHECK(memcmp(&f.data[16 + (s - 1u) * 512u], w, 512) == 0);
      memset(r, 0, sizeof(r));
      CHECK(atr.readSector(s, r) == ATARI_DISK_OK && memcmp(r, w, 512) == 0);
    }
    CHECK(atr.readSector(1441, r) == ATARI_DISK_NOT_FOUND);
    CHECK(atari_sector_bytes(&m, 1) == 512);
  }
  // 1.44M-ish: 2880 x 512 = 80 x 2 x 18.
  {
    atari_geometry_t g = atari_geometry_for_sectors(512, 2880);
    MemFile f;
    CHECK(Adafruit_AtariATR::create(&f, g));
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    CHECK(atr.geometry().sectors_per_track == 18 && atr.geometry().sides == 2);
  }
  // Double-sided 256-byte images (XF551 / quad).
  {
    atari_geometry_t g = atari_geometry_for_sectors(256, 1440);
    CHECK(g.tracks == 40 && g.sides == 2 && g.sectors_per_track == 18);
    MemFile f;
    CHECK(Adafruit_AtariATR::create(&f, g));
    CHECK(f.size() == 16 + 1440u * 256u - 384u);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t m = atr.geometry();
    CHECK(m.tracks == 40 && m.sides == 2 && m.sector_count == 1440 && m.mfm);
    uint8_t w[256], r[256];
    fill(w, 256, 4);
    CHECK(atr.writeSector(1440, w, true) == ATARI_DISK_OK);
    CHECK(atr.readSector(1440, r) == ATARI_DISK_OK && memcmp(r, w, 256) == 0);

    atari_geometry_t q = atari_geometry_for_sectors(256, 2880);
    CHECK(q.tracks == 80 && q.sides == 2 && q.sectors_per_track == 18);
    MemFile f2;
    CHECK(Adafruit_AtariATR::create(&f2, q));
    Adafruit_AtariATR atr2;
    CHECK(atr2.mount(&f2));
    CHECK(atr2.geometry().tracks == 80 && atr2.geometry().sides == 2);
  }
  // Formatting a different geometry rewrites the image to the new size.
  {
    MemFile f;
    Adafruit_AtariATR::create(&f, atari_geometry_for_density(ATARI_DENSITY_DD));
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t g720 = atari_geometry_for_sectors(512, 1440);
    CHECK(atr.canFormat(g720));
    CHECK(atr.format(g720) == ATARI_DISK_OK);
    CHECK(f.size() == 16 + 1440u * 512u);
    CHECK(atr.geometry().sector_size == 512);
    Adafruit_AtariATR again;
    CHECK(again.mount(&f) && again.geometry().sector_count == 1440);
    // 360K 5.25" style: 40 x 2 x 9 x 512.
    atari_geometry_t g360 = {512, 720, 40, 2, 9, true};
    CHECK(atr.canFormat(g360));
    CHECK(atr.format(g360) == ATARI_DISK_OK && f.size() == 16 + 720u * 512u);
    // Inconsistent / unsupported geometries.
    atari_geometry_t bad = g360;
    bad.sectors_per_track = 10;
    CHECK(!atr.canFormat(bad));
    bad = g360;
    bad.mfm = false; // FM needs 128-byte sectors
    CHECK(!atr.canFormat(bad));
    bad = g360;
    bad.sector_count = 0;
    CHECK(!atr.canFormat(bad));
    // Read-only file: format refused.
    MemFile ro(false);
    ro.data = f.data;
    Adafruit_AtariATR rr;
    CHECK(rr.mount(&ro));
    CHECK(rr.format(g720) == ATARI_DISK_WRITE_PROTECTED);
  }
  // 65535 sectors x 512 bytes: header and offset math without a 32 MB file.
  {
    atari_geometry_t g = atari_geometry_for_sectors(512, 65535);
    CHECK(g.tracks == 1 && g.sectors_per_track == 65535);
    MemFile f;
    uint32_t data = 65535u * 512u; // 33553920
    put_hdr(f.data, data, 512);
    f.data.resize(16 + data, 0);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    CHECK(atr.geometry().sector_count == 65535 &&
          atr.geometry().sector_size == 512);
    uint8_t w[512], r[512];
    fill(w, 512, 9);
    CHECK(atr.writeSector(65535, w, false) == ATARI_DISK_OK);
    CHECK(memcmp(&f.data[16 + 65534u * 512u], w, 512) == 0);
    CHECK(atr.readSector(65535, r) == ATARI_DISK_OK && memcmp(r, w, 512) == 0);
    CHECK(atr.canFormat(g));
  }
  // 512-byte image with a size that is not a multiple of 512 is rejected.
  {
    MemFile f;
    put_hdr(f.data, 512 * 10 + 128, 512);
    f.data.resize(16 + 512 * 10 + 128, 0);
    Adafruit_AtariATR atr;
    CHECK(!atr.mount(&f));
  }
}

// APE CRC byte vs protect flag, and headerless XFD formatting.
static void test_ape_and_xfd() {
  {
    MemFile f; // byte 8 bit 5 set but it is APE CRC data (byte 15 bit 1)
    put_hdr(f.data, 128 * 720, 128, 0x20, 0x02);
    f.data.resize(16 + 128 * 720, 0);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f) && !atr.writeProtected());
    MemFile g; // same without the CRC-valid bit: protected
    put_hdr(g.data, 128 * 720, 128, 0x20, 0x00);
    g.data.resize(16 + 128 * 720, 0);
    CHECK(atr.mount(&g) && atr.writeProtected());
    MemFile h; // APE read-only still wins
    put_hdr(h.data, 128 * 720, 128, 0x00, 0x03);
    h.data.resize(16 + 128 * 720, 0);
    CHECK(atr.mount(&h) && atr.writeProtected());
  }
  {
    MemFile f;
    f.data.assign(128 * 720, 0x55);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t sd = atari_geometry_for_density(ATARI_DENSITY_SD);
    CHECK(atr.canFormat(sd));
    CHECK(!atr.canFormat(atari_geometry_for_density(ATARI_DENSITY_DD)));
    CHECK(!atr.canFormat(atari_geometry_for_density(ATARI_DENSITY_ED)));
    CHECK(atr.format(atari_geometry_for_density(ATARI_DENSITY_DD)) ==
          ATARI_DISK_UNSUPPORTED);
    CHECK(f.size() == 128u * 720u && f.data[0] == 0x55);
    CHECK(atr.format(sd) == ATARI_DISK_OK);
    CHECK(f.size() == 128u * 720u && f.data[0] == 0 && f.data[0x96] == 0 &&
          f.data.back() == 0);
    uint8_t r[128];
    CHECK(atr.readSector(720, r) == ATARI_DISK_OK && r[127] == 0);
  }
}

int main() {
  test_density(ATARI_DENSITY_SD, 16 + 92160);
  test_density(ATARI_DENSITY_ED, 16 + 133120);
  test_density(ATARI_DENSITY_DD, 16 + 183936);

  // Write-protected file.
  {
    MemFile f;
    Adafruit_AtariATR::create(&f, atari_geometry_for_density(ATARI_DENSITY_SD));
    f.writable = false;
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    CHECK(atr.writeProtected());
    uint8_t b[128] = {1};
    CHECK(atr.writeSector(1, b, false) == ATARI_DISK_WRITE_PROTECTED);
    CHECK(atr.readSector(1, b) == ATARI_DISK_OK);
    CHECK(atr.format(atari_geometry_for_density(ATARI_DENSITY_DD)) ==
          ATARI_DISK_WRITE_PROTECTED);
    atr.unmount();
    CHECK(!atr.ready());
    CHECK(atr.readSector(1, b) == ATARI_DISK_NOT_READY);
  }
  // Header protect flags (SIO2PC byte 8 bit 5, APE byte 15 bit 0).
  for (int k = 0; k < 2; k++) {
    MemFile f;
    put_hdr(f.data, 128 * 720, 128, k == 0 ? 0x20 : 0, k == 1 ? 1 : 0);
    f.data.resize(16 + 128 * 720, 0);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    CHECK(atr.writeProtected());
  }
  // XFD: SD, ED, DD (short).
  {
    uint32_t sizes[3] = {92160, 133120, 183936};
    atari_density_t ds[3] = {ATARI_DENSITY_SD, ATARI_DENSITY_ED,
                             ATARI_DENSITY_DD};
    for (int i = 0; i < 3; i++) {
      MemFile f;
      f.data.assign(sizes[i], 0);
      fill(&f.data[0], 384, 9);
      Adafruit_AtariATR atr;
      CHECK(atr.mount(&f));
      atari_geometry_t m = atr.geometry();
      CHECK(atari_density_of(&m) == ds[i]);
      uint8_t r[256], w[256];
      CHECK(atr.readSector(2, r) == ATARI_DISK_OK);
      CHECK(memcmp(r, &f.data[128], 128) == 0);
      fill(w, 256, 5);
      CHECK(atr.writeSector(m.sector_count, w, true) == ATARI_DISK_OK);
      uint32_t off = (m.sector_size == 256) ? 384 + (m.sector_count - 4) * 256u
                                            : (m.sector_count - 1) * 128u;
      CHECK(memcmp(&f.data[off], w, m.sector_size) == 0);
    }
  }
  // 720 x 256-byte XFD: double density with full 256-byte boot sector slots.
  {
    MemFile f;
    f.data.assign(720u * 256u, 0);
    fill(&f.data[256], 128, 11); // sector 2
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t m = atr.geometry();
    CHECK(atari_density_of(&m) == ATARI_DENSITY_DD);
    uint8_t r[256], w[256];
    CHECK(atr.readSector(2, r) == ATARI_DISK_OK);
    CHECK(memcmp(r, &f.data[256], 128) == 0);
    fill(w, 256, 12);
    CHECK(atr.writeSector(720, w, true) == ATARI_DISK_OK);
    CHECK(memcmp(&f.data[719u * 256u], w, 256) == 0);
  }
  // 256-byte "full" DD variant: every sector in a 256-byte slot.
  {
    MemFile f;
    put_hdr(f.data, 256 * 720, 256);
    f.data.resize(16 + 256 * 720, 0);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t m = atr.geometry();
    CHECK(atari_density_of(&m) == ATARI_DENSITY_DD);
    uint8_t w[256], r[256];
    fill(w, 256, 7);
    CHECK(atr.writeSector(2, w, true) == ATARI_DISK_OK);
    CHECK(memcmp(&f.data[16 + 256], w, 128) == 0);
    CHECK(f.data[16 + 256 + 128] == 0);
    CHECK(atr.writeSector(4, w, true) == ATARI_DISK_OK);
    CHECK(memcmp(&f.data[16 + 3 * 256], w, 256) == 0);
    CHECK(atr.readSector(4, r) == ATARI_DISK_OK && memcmp(r, w, 256) == 0);
    // Formatting converts to the standard layout.
    CHECK(atr.format(atari_geometry_for_density(ATARI_DENSITY_DD)) ==
          ATARI_DISK_OK);
    CHECK(f.size() == 16 + 183936);
  }
  // Non-standard ("hard disk") images.
  {
    MemFile f;
    put_hdr(f.data, 256 * 2000 - 384, 256);
    f.data.resize(16 + 256 * 2000 - 384, 0);
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    atari_geometry_t m = atr.geometry();
    CHECK(m.sector_size == 256 && m.sector_count == 2000 && m.tracks == 1 &&
          m.sectors_per_track == 2000 && m.mfm);
    CHECK(atari_density_of(&m) == ATARI_DENSITY_OTHER);
    CHECK(atr.canFormat(m));
    CHECK(atr.canFormat(atari_geometry_for_density(ATARI_DENSITY_SD)));
    atari_geometry_t o = m;
    o.sector_count = 1999;
    o.sectors_per_track = 1999;
    CHECK(atr.canFormat(o)); // any one-track hard-disk geometry
    o.tracks = 40;           // tracks * sides * spt != count
    o.sectors_per_track = 18;
    CHECK(!atr.canFormat(o));
    uint8_t w[256], r[256];
    fill(w, 256, 2);
    CHECK(atr.writeSector(2000, w, true) == ATARI_DISK_OK);
    CHECK(atr.format(m) == ATARI_DISK_OK);
    CHECK(atr.readSector(2000, r) == ATARI_DISK_OK && r[0] == 0 && r[255] == 0);
  }
  // Format SD -> DD resizes and zeroes.
  {
    MemFile f;
    Adafruit_AtariATR::create(&f, atari_geometry_for_density(ATARI_DENSITY_SD));
    Adafruit_AtariATR atr;
    CHECK(atr.mount(&f));
    uint8_t w[128];
    fill(w, 128, 1);
    CHECK(atr.writeSector(720, w, false) == ATARI_DISK_OK);
    atari_geometry_t dd = atari_geometry_for_density(ATARI_DENSITY_DD);
    CHECK(atr.canFormat(dd));
    CHECK(atr.format(dd) == ATARI_DISK_OK);
    CHECK(f.size() == 16 + 183936);
    atari_geometry_t m = atr.geometry();
    CHECK(atari_density_of(&m) == ATARI_DENSITY_DD);
    uint8_t r[256];
    CHECK(atr.readSector(720, r) == ATARI_DISK_OK && r[0] == 0 && r[255] == 0);
    CHECK(atr.readSector(1, r) == ATARI_DISK_OK && r[0] == 0 && r[127] == 0);
    // And shrink back; remount from the file agrees.
    CHECK(atr.format(atari_geometry_for_density(ATARI_DENSITY_SD)) ==
          ATARI_DISK_OK);
    CHECK(f.size() == 16 + 92160);
    Adafruit_AtariATR again;
    CHECK(again.mount(&f));
    m = again.geometry();
    CHECK(atari_density_of(&m) == ATARI_DENSITY_SD);
    atari_geometry_t bad = dd;
    bad.sector_size = 1024;
    CHECK(!atr.canFormat(bad));
    CHECK(atr.format(bad) == ATARI_DISK_UNSUPPORTED);
  }
  // Rejects.
  {
    Adafruit_AtariATR atr;
    MemFile f;
    f.data.assign(16 + 92160, 0); // bad magic, size not a valid XFD? 92176
    f.data[0] = 0x12;
    CHECK(!atr.mount(&f)); // 92176 & 0x7F == 16
    CHECK(!atr.ready());
    MemFile e; // empty
    CHECK(!atr.mount(&e));
    CHECK(!atr.mount(0));
    MemFile t; // truncated ATR
    Adafruit_AtariATR::create(&t, atari_geometry_for_density(ATARI_DENSITY_SD));
    t.data.resize(t.data.size() - 128);
    CHECK(!atr.mount(&t));
    MemFile s; // bad sector size
    put_hdr(s.data, 1024 * 100, 1024);
    s.data.resize(16 + 1024 * 100, 0);
    CHECK(!atr.mount(&s));
    MemFile d; // DD data size neither short nor full
    put_hdr(d.data, 256 * 100 + 64, 256);
    d.data.resize(16 + 256 * 100 + 64, 0);
    CHECK(!atr.mount(&d));
  }
  test_sdx();
  test_ape_and_xfd();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("test_atari_atr: all passed\n");
  return 0;
}
