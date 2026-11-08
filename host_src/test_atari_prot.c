// SPDX-License-Identifier: MIT
//
// Host tests for src/atari_prot_track.h: scanning, merging and mastering of
// copy-protected Atari tracks at the flux level.
#include <stdio.h>
#include <stdlib.h>

#include "atari_prot_track.h"

enum { sample_freq = 24000000, n_pulses = 66000, jitter_pulses = 2 * n_pulses };

static int failures, passes;

static void report(const char *name, bool ok) {
  if (ok)
    passes++;
  else
    failures++;
  printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
}

static uint32_t rng_state = 24681357;
static uint32_t rng(void) {
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 8;
}

static const char *const density_name[] = {"SD", "ED", "DD"};
static const atari_density_t densities[3] = {ATARI_DENSITY_SD, ATARI_DENSITY_ED,
                                             ATARI_DENSITY_DD};
static const unsigned rpms[2] = {300, 360};

static uint8_t pulses[jitter_pulses];
static uint8_t noisy[jitter_pulses];
static atari_prot_track_t src, got, other, work;

typedef struct {
  atari_density_t d;
  unsigned rpm;
  uint16_t cell;
  uint32_t rev;
  double upb; // angular units per byte of flux
} drive_t;

static drive_t make_drive(atari_density_t d, unsigned rpm) {
  drive_t v;
  v.d = d;
  v.rpm = rpm;
  v.cell = atari_track_cell_ticks(d, sample_freq, 60000000u / rpm);
  v.rev = (uint32_t)((uint64_t)sample_freq * 60u / rpm);
  v.upb = 16.0 * v.cell * ATARI_ATX_REV_UNITS / v.rev;
  return v;
}

static uint8_t code_of(atari_density_t d) { return d == ATARI_DENSITY_DD; }

// Append an entry with random data; returns its index
static size_t add(atari_prot_track_t *t, uint8_t number, uint8_t status,
                  uint16_t position, uint8_t size_code) {
  uint16_t sb = atari_prot_sector_bytes(t->density);
  atari_prot_sector_t *s = &t->sectors[t->sector_count];
  s->number = number;
  s->status = status;
  s->position = position;
  s->size_code = size_code;
  s->weak_offset = ATARI_PROT_NONE;
  s->data_offset = ATARI_PROT_NONE;
  if (!(status & ATARI_ATX_MISSING)) {
    s->data_offset = t->data_bytes;
    for (uint16_t i = 0; i < sb; i++)
      t->data[t->data_bytes + i] = (uint8_t)rng();
    t->data_bytes = (uint16_t)(t->data_bytes + sb);
  }
  return t->sector_count++;
}

// A protected track: nnormal sectors 1.., a duplicate of 5 at the end of the
// list (far from the first 5), a CRC error, a deleted and a missing-data entry
static void build(atari_prot_track_t *t, const drive_t *v, unsigned nnormal) {
  atari_prot_track_init(t, 7, 0, v->d);
  uint8_t code = code_of(v->d);
  double pos = 200;
  uint8_t nums[40];
  uint8_t stat[40];
  unsigned n = 0;
  for (unsigned i = 0; i < nnormal; i++, n++) {
    nums[n] = (uint8_t)(i + 1);
    stat[n] = 0;
  }
  nums[n] = 5, stat[n++] = 0;
  nums[n] = (uint8_t)(nnormal + 1), stat[n++] = ATARI_ATX_CRC_ERROR;
  nums[n] = (uint8_t)(nnormal + 2), stat[n++] = ATARI_ATX_DELETED;
  nums[n] = (uint8_t)(nnormal + 3), stat[n++] = ATARI_ATX_MISSING;
  for (unsigned i = 0; i < n; i++) {
    add(t, nums[i], stat[i], (uint16_t)pos, code);
    pos += atari_prot_pitch_bytes(v->d, code, !(stat[i] & ATARI_ATX_MISSING)) *
               v->upb +
           10;
  }
}

static uint16_t last_shift;

static size_t encode(const atari_prot_track_t *t, const drive_t *v,
                     uint32_t *moved) {
  return atari_prot_encode(t, pulses, n_pulses, v->cell, v->rev, moved, NULL,
                           &last_shift);
}

static size_t scan(atari_prot_track_t *t, const drive_t *v, const uint8_t *p,
                   size_t n) {
  atari_prot_track_init(t, 7, 0, v->d);
  return atari_prot_scan(t, p, n, v->cell, v->rev, 0, NULL);
}

static const uint8_t *data_of(const atari_prot_track_t *t, size_t i) {
  return &t->data[t->sectors[i].data_offset];
}

// Entries match: same list, statuses (without EXTENDED), size codes, data and
// positions within tol units
static bool same_track(const atari_prot_track_t *a, const atari_prot_track_t *b,
                       unsigned tol, bool check_data) {
  if (a->sector_count != b->sector_count)
    return false;
  uint16_t sb = atari_prot_sector_bytes(a->density);
  for (size_t i = 0; i < a->sector_count; i++) {
    const atari_prot_sector_t *x = &a->sectors[i], *y = &b->sectors[i];
    if (x->number != y->number || x->size_code != y->size_code ||
        (x->status & ~ATARI_ATX_EXTENDED) != (y->status & ~ATARI_ATX_EXTENDED))
      return false;
    if (atari_prot_distance(x->position, y->position) > tol)
      return false;
    if ((x->data_offset == ATARI_PROT_NONE) !=
        (y->data_offset == ATARI_PROT_NONE))
      return false;
    if (check_data && x->data_offset != ATARI_PROT_NONE &&
        memcmp(data_of(a, i), data_of(b, i), sb))
      return false;
  }
  return true;
}

static void jitter(const uint8_t *p, size_t n, uint8_t *out, int amp) {
  for (size_t i = 0; i < n; i++) {
    int v = p[i] + (int)(rng() % (2 * amp + 1)) - amp;
    out[i] = (uint8_t)(v < 1 ? 1 : v > 255 ? 255 : v);
  }
}

static void test_roundtrip(void) {
  static const unsigned nnormal[3] = {14, 22, 14};
  for (int di = 0; di < 3; di++) {
    for (int ri = 0; ri < 2; ri++) {
      drive_t v = make_drive(densities[di], rpms[ri]);
      char name[96];
      build(&src, &v, nnormal[di]);
      uint32_t moved = 99999;
      size_t n = encode(&src, &v, &moved);
      snprintf(name, sizeof(name),
               "%s %u RPM protected track encodes (moved %u)", density_name[di],
               v.rpm, (unsigned)moved);
      report(name, n > 0 && moved == 0);
      if (!n)
        continue;
      uint32_t ticks = atari_pulse_ticks(pulses, n);
      snprintf(name, sizeof(name), "%s %u RPM encoded length within revolution",
               density_name[di], v.rpm);
      report(name, ticks <= v.rev && ticks > v.rev - v.rev / 20);

      size_t found = scan(&got, &v, pulses, n);
      snprintf(name, sizeof(name), "%s %u RPM scan finds %u entries (want %u)",
               density_name[di], v.rpm, (unsigned)found,
               (unsigned)src.sector_count);
      report(name, found == src.sector_count);
      snprintf(name, sizeof(name),
               "%s %u RPM scan matches statuses, data and positions",
               density_name[di], v.rpm);
      report(name, same_track(&src, &got, 6, true));

      jitter(pulses, n, noisy, 2);
      scan(&got, &v, noisy, n);
      snprintf(name, sizeof(name), "%s %u RPM round trip with +-2 tick jitter",
               density_name[di], v.rpm);
      report(name, same_track(&src, &got, 10, true));
    }
  }
}

static void test_weak(void) {
  static const atari_density_t ds[2] = {ATARI_DENSITY_SD, ATARI_DENSITY_DD};
  for (int di = 0; di < 2; di++) {
    for (int ri = 0; ri < 2; ri++) {
      drive_t v = make_drive(ds[di], rpms[ri]);
      char name[96];
      const char *wname = ds[di] == ATARI_DENSITY_DD ? "DD" : "SD";
      build(&src, &v, 10);
      uint16_t want = ds[di] == ATARI_DENSITY_DD ? 100 : 40;
      src.sectors[2].weak_offset = want;
      src.sectors[2].status |= ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED;
      size_t n = encode(&src, &v, NULL);
      snprintf(name, sizeof(name), "%s %u RPM weak track encodes", wname,
               v.rpm);
      report(name, n > 0);
      if (!n)
        continue;
      // Two reads with different noise (about +-8% of a cell)
      int amp = v.cell / 12;
      jitter(pulses, n, noisy, amp);
      scan(&got, &v, noisy, n);
      jitter(pulses, n, noisy, amp);
      scan(&other, &v, noisy, n);
      snprintf(name, sizeof(name), "%s %u RPM weak sector reads as CRC error",
               wname, v.rpm);
      report(name, got.sector_count == src.sector_count &&
                       (got.sectors[2].status & ATARI_ATX_CRC_ERROR) &&
                       other.sectors[2].number == 3);
      bool pre = got.sector_count > 2 &&
                 !memcmp(data_of(&got, 2), data_of(&src, 2), want);
      snprintf(name, sizeof(name),
               "%s %u RPM bytes before the weak offset read", wname, v.rpm);
      report(name, pre);
      size_t added = 99;
      bool ok = atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, &added);
      const atari_prot_sector_t *w = &got.sectors[2];
      snprintf(name, sizeof(name),
               "%s %u RPM merge marks weak sector (offset %u, want %u)", wname,
               v.rpm, (unsigned)w->weak_offset, (unsigned)want);
      report(name, ok && added == 0 && w->weak_offset >= want &&
                       w->weak_offset <= want + 30 &&
                       (w->status & ATARI_ATX_EXTENDED) &&
                       (w->status & ATARI_ATX_CRC_ERROR));
      unsigned flagged = 0;
      for (size_t i = 0; i < got.sector_count; i++)
        if (got.sectors[i].weak_offset != ATARI_PROT_NONE)
          flagged++;
      snprintf(name, sizeof(name), "%s %u RPM no other sector marked weak",
               wname, v.rpm);
      report(name, flagged == 1);
    }
  }
}

static void test_merge(void) {
  drive_t v = make_drive(ATARI_DENSITY_SD, 300);
  build(&src, &v, 14);
  size_t n = encode(&src, &v, NULL);
  scan(&got, &v, pulses, n);
  memcpy(&work, &got, sizeof(work));
  jitter(pulses, n, noisy, 1);
  scan(&other, &v, noisy, n);
  size_t added = 99;
  bool ok = atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, &added);
  report("merge of identical scans adds nothing", ok && added == 0);
  bool unchanged = got.sector_count == work.sector_count &&
                   got.data_bytes == work.data_bytes &&
                   !memcmp(got.sectors, work.sectors,
                           sizeof(got.sectors[0]) * got.sector_count) &&
                   !memcmp(got.data, work.data, got.data_bytes);
  report("merge of identical scans leaves the track unchanged", unchanged);

  // Entries seen in only one revolution are kept, in position order
  memcpy(&other, &work, sizeof(other));
  size_t drop = 3;
  other.sector_count--; // simulate a revolution that missed the last entry
  memcpy(&got, &work, sizeof(got));
  ok = atari_prot_merge(&other, &got, ATARI_PROT_MERGE_TOL, &added);
  report("merge keeps an entry only the other scan saw",
         ok && added == 1 && other.sector_count == work.sector_count);
  ok = same_track(&other, &work, 0, true);
  report("merged entries are in position order with their data", ok);
  // Remove an entry in the middle
  memcpy(&other, &work, sizeof(other));
  for (size_t i = drop; i + 1 < other.sector_count; i++)
    other.sectors[i] = other.sectors[i + 1];
  other.sector_count--;
  ok = atari_prot_merge(&other, &work, ATARI_PROT_MERGE_TOL, &added);
  report("merge inserts an entry back in the middle",
         ok && added == 1 &&
             other.sectors[drop].number == work.sectors[drop].number &&
             other.sectors[drop].position == work.sectors[drop].position);
  // Same number at a distant position is a different entry
  memcpy(&other, &work, sizeof(other));
  other.sectors[0].position =
      (uint16_t)(other.sectors[0].position + ATARI_ATX_REV_UNITS / 2);
  memcpy(&got, &work, sizeof(got));
  ok = atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, &added);
  report("merge treats a far position as another entry",
         ok && added == 1 && got.sector_count == work.sector_count + 1);
  // Mismatched tracks, overflow and bad arguments
  memcpy(&got, &work, sizeof(got));
  other.track = 9;
  report("merge rejects a different track",
         !atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, NULL));
  report("merge rejects NULL and self",
         !atari_prot_merge(&got, NULL, ATARI_PROT_MERGE_TOL, NULL) &&
             !atari_prot_merge(&got, &got, ATARI_PROT_MERGE_TOL, NULL));
  memcpy(&got, &work, sizeof(got));
  memcpy(&other, &work, sizeof(other));
  got.sector_count = ATARI_PROT_MAX_SECTORS;
  for (size_t i = 0; i < other.sector_count; i++)
    other.sectors[i].number = (uint8_t)(100 + i);
  report("merge fails without changing dst when full",
         !atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, NULL) &&
             got.sector_count == ATARI_PROT_MAX_SECTORS);
}

static void test_normal_tracks(void) {
  for (int di = 0; di < 3; di++) {
    for (int ri = 0; ri < 2; ri++) {
      drive_t v = make_drive(densities[di], rpms[ri]);
      atari_geometry_t g = atari_geometry_for_density(v.d);
      static uint8_t sec[ATARI_TRACK_MAX_BYTES];
      for (size_t i = 0; i < sizeof(sec); i++)
        sec[i] = (uint8_t)rng();
      size_t n = atari_encode_track(v.d, 7, sec, pulses, n_pulses, v.cell,
                                    v.rev, NULL);
      size_t found = n ? scan(&got, &v, pulses, n) : 0;
      char name[96];
      snprintf(name, sizeof(name), "%s %u RPM stock track scans to %u entries",
               density_name[di], v.rpm, g.sectors_per_track);
      report(name, found == g.sectors_per_track);
      bool ok = found == g.sectors_per_track;
      const uint8_t *order = atari_track_order(v.d);
      uint32_t gap_sum = 0;
      for (size_t i = 0; ok && i < found; i++) {
        const atari_prot_sector_t *s = &got.sectors[i];
        ok = s->status == 0 && s->number == order[i] + 1 &&
             s->size_code == code_of(v.d) &&
             !memcmp(data_of(&got, i), &sec[order[i] * g.sector_size],
                     atari_prot_sector_bytes(v.d) < g.sector_size
                         ? atari_prot_sector_bytes(v.d)
                         : g.sector_size);
        if (i)
          gap_sum += s->position - got.sectors[i - 1].position;
      }
      // Positions rise, start after the lead-in and are evenly spaced
      uint32_t mean = found > 1 ? gap_sum / (uint32_t)(found - 1) : 0;
      for (size_t i = 1; ok && i < found; i++) {
        uint32_t d = got.sectors[i].position - got.sectors[i - 1].position;
        ok = d + d / 8 > mean && d < mean + mean / 8;
      }
      ok = ok && got.sectors[0].position > 100 &&
           got.sectors[0].position < 1500 &&
           got.sectors[found - 1].position < ATARI_ATX_REV_UNITS;
      snprintf(name, sizeof(name),
               "%s %u RPM stock track entries clean, in order, evenly spaced",
               density_name[di], v.rpm);
      report(name, ok);
    }
  }
}

static void test_edges(void) {
  drive_t v = make_drive(ATARI_DENSITY_SD, 300);

  // Long sector: size code above the density's keeps the first 128 bytes
  atari_prot_track_init(&src, 7, 0, v.d);
  add(&src, 3, 0, 2000, 1);
  add(&src, 4, 0, 6000, 0);
  size_t n = encode(&src, &v, NULL);
  scan(&got, &v, pulses, n);
  report("long sector scans as LOST_DATA with its first 128 bytes",
         got.sector_count == 2 && got.sectors[0].size_code == 1 &&
             (got.sectors[0].status & ATARI_ATX_LOST_DATA) &&
             !(got.sectors[0].status & ATARI_ATX_CRC_ERROR) &&
             !memcmp(data_of(&got, 0), data_of(&src, 0), 128) &&
             got.sectors[1].status == 0 && got.sectors[1].size_code == 0);

  // Entries too close are pushed later and reported
  atari_prot_track_init(&src, 7, 0, v.d);
  add(&src, 1, 0, 300, 0);
  add(&src, 2, 0, 310, 0);
  uint32_t moved = 0;
  n = encode(&src, &v, &moved);
  scan(&got, &v, pulses, n);
  uint32_t pitch = (uint32_t)(atari_prot_pitch_bytes(v.d, 0, true) * v.upb);
  report("overlapping entry is pushed later and the move reported",
         n && moved > pitch / 2 && moved < 2 * pitch && got.sector_count == 2 &&
             got.sectors[1].position > got.sectors[0].position + pitch / 2);

  // Wrong cylinder is invisible; side byte is not checked
  atari_prot_track_init(&src, 7, 1, v.d);
  add(&src, 1, 0, 400, 0);
  n = encode(&src, &v, NULL);
  atari_prot_track_init(&got, 7, 0, v.d);
  size_t seen = atari_prot_scan(&got, pulses, n, v.cell, v.rev, 0, NULL);
  report("side byte is not checked", seen == 1);
  atari_prot_track_init(&got, 8, 0, v.d);
  seen = atari_prot_scan(&got, pulses, n, v.cell, v.rev, 0, NULL);
  report("IDs of another cylinder are skipped", seen == 0);

  // Index offset: junk before the index shifts nothing
  build(&src, &v, 14);
  n = encode(&src, &v, NULL);
  memmove(noisy + 1, pulses, n);
  noisy[0] = 200;
  atari_prot_track_init(&got, 7, 0, v.d);
  atari_prot_scan(&got, noisy, n + 1, v.cell, v.rev, 200, NULL);
  report("index_ticks offsets positions", same_track(&src, &got, 6, true));

  // A second revolution in the buffer is not read again
  memcpy(pulses + n, pulses, n);
  uint32_t ticks = atari_pulse_ticks(pulses, n);
  atari_prot_track_init(&got, 7, 0, v.d);
  seen = atari_prot_scan(&got, pulses, 2 * n, v.cell, ticks, 0, NULL);
  report("IDs past one revolution are ignored", seen == src.sector_count);

  // Entries past the capacity are dropped and counted
  unsigned dropped = 0;
  atari_prot_track_init(&got, 7, 0, v.d);
  got.sector_count = 0;
  build(&src, &v, 14);
  n = encode(&src, &v, NULL);
  atari_prot_track_init(&got, 7, 0, v.d);
  seen = atari_prot_scan(&got, pulses, n, v.cell, v.rev, 0, &dropped);
  report("scan reports no drops when everything fits",
         seen == src.sector_count && dropped == 0);

  // Bad arguments
  atari_prot_track_init(&got, 7, 0, v.d);
  report("scan rejects bad arguments",
         atari_prot_scan(NULL, pulses, n, v.cell, v.rev, 0, NULL) == 0 &&
             atari_prot_scan(&got, pulses, n, v.cell, 0, 0, NULL) == 0 &&
             atari_prot_scan(&got, pulses, n, 0, v.rev, 0, NULL) == 0 &&
             atari_prot_scan(&got, NULL, n, v.cell, v.rev, 0, NULL) == 0);
  report("encode rejects bad arguments",
         atari_prot_encode(NULL, pulses, n_pulses, v.cell, v.rev, NULL, NULL,
                           NULL) == 0 &&
             atari_prot_encode(&src, NULL, n_pulses, v.cell, v.rev, NULL, NULL,
                               NULL) == 0 &&
             atari_prot_encode(&src, pulses, n_pulses, 0, v.rev, NULL, NULL,
                               NULL) == 0 &&
             atari_prot_encode(&src, pulses, n_pulses, v.cell, 0, NULL, NULL,
                               NULL) == 0);
  report("encode fails when the pulse buffer is too small",
         atari_prot_encode(&src, pulses, 2000, v.cell, v.rev, NULL, NULL,
                           NULL) == 0);
  src.sectors[0].data_offset = (uint16_t)(src.data_bytes);
  report("encode fails when an entry's data is not in the track",
         encode(&src, &v, NULL) == 0);

  // Too many sectors for one revolution
  drive_t dd = make_drive(ATARI_DENSITY_DD, 300);
  atari_prot_track_init(&src, 7, 0, dd.d);
  for (unsigned i = 0; i < 30; i++)
    add(&src, (uint8_t)(i + 1), 0, (uint16_t)(200 + i * 800), 1);
  moved = 0;
  report("encode fails when the track does not fit in a revolution",
         encode(&src, &dd, &moved) == 0);
}

static void test_index_crossing(void) {
  drive_t v = make_drive(ATARI_DENSITY_SD, 300);
  static const uint16_t crossing[2] = {25900, 26000};
  for (int k = 0; k < 2; k++) {
    char name[96];
    atari_prot_track_init(&src, 7, 0, v.d);
    add(&src, 1, 0, 2000, 0);
    add(&src, 2, 0, 4000, 0);
    add(&src, 3, ATARI_ATX_DELETED, 8000, 0);
    add(&src, 4, 0, 13000, 0);
    add(&src, 5, 0, crossing[k], 0);
    uint32_t moved = 99;
    size_t n = encode(&src, &v, &moved);
    snprintf(name, sizeof(name),
             "entry at %u crosses the index and encodes (shifted)",
             crossing[k]);
    report(name, n > 0 && moved == 0);
    if (!n)
      continue;
    uint32_t ticks = atari_pulse_ticks(pulses, n);
    report("shifted flux keeps the end margin",
           ticks <= v.rev - v.rev / 50 + 1 && last_shift != 0);
    // Expected: every entry at (position - shift) mod a revolution, sorted
    memcpy(&work, &src, sizeof(work));
    for (size_t i = 0; i < work.sector_count; i++)
      work.sectors[i].position = (uint16_t)((work.sectors[i].position +
                                             ATARI_ATX_REV_UNITS - last_shift) %
                                            ATARI_ATX_REV_UNITS);
    for (size_t i = 1; i < work.sector_count; i++)
      for (size_t j = i;
           j > 0 && work.sectors[j - 1].position > work.sectors[j].position;
           j--) {
        atari_prot_sector_t tmp = work.sectors[j];
        work.sectors[j] = work.sectors[j - 1];
        work.sectors[j - 1] = tmp;
      }
    size_t found = scan(&got, &v, pulses, n);
    snprintf(name, sizeof(name), "entry at %u scans back at shifted positions",
             crossing[k]);
    report(name,
           found == src.sector_count && same_track(&work, &got, 10, true));
  }
  // A track that is too full still fails after trying a shift
  drive_t dd = make_drive(ATARI_DENSITY_DD, 300);
  atari_prot_track_init(&src, 7, 0, dd.d);
  for (unsigned i = 0; i < 20; i++)
    add(&src, (uint8_t)(i + 1), 0, (uint16_t)(1300 * i + 100), 1);
  report("shifting does not rescue a track that is too full",
         encode(&src, &dd, NULL) == 0);
}

static void test_merge_crc(void) {
  drive_t v = make_drive(ATARI_DENSITY_SD, 300);
  build(&src, &v, 14);
  size_t n = encode(&src, &v, NULL);
  scan(&got, &v, pulses, n);
  memcpy(&work, &got, sizeof(work));
  // A noisy revolution: sector 3 reads with wrong data and a CRC error
  memcpy(&other, &got, sizeof(other));
  other.data[other.sectors[2].data_offset + 20] ^= 0x55;
  other.sectors[2].status |= ATARI_ATX_CRC_ERROR;
  bool ok = atari_prot_merge(&got, &other, ATARI_PROT_MERGE_TOL, NULL);
  report("clean then corrupted revolution stays clean, not weak",
         ok && got.sectors[2].weak_offset == ATARI_PROT_NONE &&
             !(got.sectors[2].status &
               (ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED)) &&
             !memcmp(data_of(&got, 2), data_of(&work, 2), 128));
  // The other way round: switches to the good copy
  memcpy(&got, &other, sizeof(got));
  ok = atari_prot_merge(&got, &work, ATARI_PROT_MERGE_TOL, NULL);
  report("corrupted then clean revolution switches to the clean copy",
         ok && got.sectors[2].weak_offset == ATARI_PROT_NONE &&
             !(got.sectors[2].status &
               (ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED)) &&
             !memcmp(data_of(&got, 2), data_of(&work, 2), 128));
  // Two different bad reads are weak
  memcpy(&got, &other, sizeof(got));
  memcpy(&work, &other, sizeof(work));
  work.data[work.sectors[2].data_offset + 30] ^= 0x33;
  ok = atari_prot_merge(&got, &work, ATARI_PROT_MERGE_TOL, NULL);
  report("two different CRC-error reads are weak at the first difference",
         ok && got.sectors[2].weak_offset == 30 &&
             (got.sectors[2].status & ATARI_ATX_EXTENDED));
}

int main(int argc, char **argv) {
  if (argc > 1) // optional seed, to vary the noise
    rng_state = (uint32_t)atoi(argv[1]);
  test_roundtrip();
  test_weak();
  test_merge();
  test_normal_tracks();
  test_edges();
  test_index_crossing();
  test_merge_crc();
  printf("%d passed, %d failed\n", passes, failures);
  return failures ? 1 : 0;
}
