// SPDX-License-Identifier: MIT
//
// Host tests for decode_track_fm and the encoder's sector_order / skip_iam
// extensions in mfm_impl.h.

#include <stdio.h>
#include <stdlib.h>

#include "mfm_impl.h"

enum { max_sectors = 26, max_sector_size = 256, n_pulses = 80000 };

static int failures;

static void report(const char *name, bool ok) {
  printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok)
    failures++;
}

static uint32_t rng_state = 12345;
static uint32_t rng(void) {
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 8;
}

typedef struct {
  mfm_io_t io;
  uint8_t pulses[n_pulses];
  uint8_t data[max_sectors * max_sector_size];
  uint8_t out[max_sectors * max_sector_size];
  uint8_t validity[max_sectors];
} track_t;

// Atari-like tight FM layout without an index mark
static const mfm_io_settings_t atari_fm = {
    16, 11, {12, 0, 0, 0, 0, 0, 0, 0}, 0, 6, 0xff, true, true,
};

static const uint8_t interleave18[18] = {0, 2, 4, 6, 8, 10, 12, 14, 16,
                                         1, 3, 5, 7, 9, 11, 13, 15, 17};

static void encode(track_t *t, const mfm_io_settings_t *settings,
                   size_t n_sectors, uint8_t n, uint16_t t1,
                   const uint8_t *order, uint8_t cylinder) {
  memset(t, 0, sizeof(*t));
  t->io.settings = settings;
  t->io.T1_nom = t1;
  t->io.pulses = t->pulses;
  t->io.n_pulses = n_pulses;
  t->io.sectors = t->data;
  t->io.n_sectors = n_sectors;
  t->io.n = n;
  t->io.cylinder = cylinder;
  t->io.sector_order = order;
  for (size_t i = 0; i < n_sectors * (128u << n); i++)
    t->data[i] = rng();
  encode_track_mfm(&t->io);
}

static void setup_decode(track_t *t, uint8_t cylinder) {
  memset(t->out, 0, sizeof(t->out));
  memset(t->validity, 0, sizeof(t->validity));
  t->io.sectors = t->out;
  t->io.sector_validity = t->validity;
  t->io.cylinder = cylinder;
  t->io.validate_id = false;
  t->io.cylinder_ptr = NULL;
}

// Number of sectors decoded with the right content (0 if any is wrong data
// marked valid)
static size_t check_sectors(track_t *t, bool *content_ok) {
  size_t size = 128u << t->io.n, valid = 0;
  *content_ok = true;
  for (size_t i = 0; i < t->io.n_sectors; i++) {
    if (!t->validity[i])
      continue;
    valid++;
    if (memcmp(t->out + i * size, t->data + i * size, size) != 0)
      *content_ok = false;
  }
  return valid;
}

static bool decodes_all(track_t *t, uint8_t cylinder) {
  setup_decode(t, cylinder);
  size_t n = decode_track_fm(&t->io);
  bool content_ok;
  size_t valid = check_sectors(t, &content_ok);
  return n == t->io.n_sectors && valid == n && content_ok;
}

static void scale_pulses(track_t *t, double scale, double jitter) {
  for (size_t i = 0; i < n_pulses; i++) {
    double j = jitter ? 1 + jitter * ((rng() % 2001) / 1000.0 - 1) : 1;
    double v = t->pulses[i] * scale * j + 0.5;
    t->pulses[i] = v > 255 ? 255 : (uint8_t)v;
  }
}

// Swap the first two unequal adjacent pulses at or after `from`; this changes
// cells without moving anything after them.
static bool corrupt_from(track_t *t, size_t from) {
  for (size_t i = from; i < from + 32; i++) {
    if (t->pulses[i] != t->pulses[i + 1]) {
      uint8_t tmp = t->pulses[i];
      t->pulses[i] = t->pulses[i + 1];
      t->pulses[i + 1] = tmp;
      return true;
    }
  }
  return false;
}

static void test_round_trip(void) {
  track_t *t = malloc(sizeof(*t));
  rng_state = 1;
  encode(t, &atari_fm, 18, 0, 80, interleave18, 5);
  report("a. FM 18x128 interleaved, skip_iam round trip", decodes_all(t, 5));

  // The physical order follows sector_order: with the flux cut short, the
  // first sectors to decode are slots 0 and 2, never slot 1.
  setup_decode(t, 5);
  t->io.n_pulses = 3000;
  decode_track_fm(&t->io);
  bool first = t->validity[0] && !t->validity[1] && !t->validity[2];
  setup_decode(t, 5);
  t->io.n_pulses = 5000;
  decode_track_fm(&t->io);
  bool second = t->validity[0] && t->validity[2] && !t->validity[1];
  t->io.n_pulses = n_pulses;
  report("a. sector_order sets the physical sector order", first && second);

  uint8_t cyl = 0;
  setup_decode(t, 5);
  t->io.cylinder_ptr = &cyl;
  decode_track_fm(&t->io);
  report("a. cylinder_ptr reports the IDAM cylinder", cyl == 5);

  // already-valid sectors are counted and kept (early exit)
  size_t n_before = t->io.n_valid;
  bool again = decode_track_fm(&t->io) == n_before;
  report("a. second pass with all sectors valid returns immediately", again);
  free(t);
}

static void test_speed_jitter(void) {
  static const double scales[] = {0.92, 1.0, 1.04, 1.08};
  track_t *t = malloc(sizeof(*t));
  track_t *orig = malloc(sizeof(*t));
  rng_state = 2;
  encode(orig, &atari_fm, 18, 0, 80, interleave18, 5);
  char name[96];
  for (size_t s = 0; s < sizeof(scales) / sizeof(scales[0]); s++) {
    memcpy(t, orig, sizeof(*t));
    scale_pulses(t, scales[s], 0);
    snprintf(name, sizeof(name), "b. speed x%.2f", scales[s]);
    report(name, decodes_all(t, 5));
  }
  for (int trial = 0; trial < 20; trial++) {
    memcpy(t, orig, sizeof(*t));
    scale_pulses(t, 1.0, 0.06);
    snprintf(name, sizeof(name), "b. +/-6%% jitter trial %d", trial);
    if (!decodes_all(t, 5))
      report(name, false);
  }
  report("b. +/-6% jitter x20 trials", true);
  for (int trial = 0; trial < 20; trial++) {
    memcpy(t, orig, sizeof(*t));
    scale_pulses(t, 1.04, 0.04);
    snprintf(name, sizeof(name), "b. x1.04 with +/-4%% jitter trial %d", trial);
    if (!decodes_all(t, 5))
      report(name, false);
  }
  report("b. x1.04 with +/-4% jitter x20 trials", true);

  // Saturated 255 values (longer than any real FM interval) in the leading
  // gap must not stop later sectors from decoding.
  memcpy(t, orig, sizeof(*t));
  t->pulses[2] = t->pulses[40] = t->pulses[100] = 255;
  report("b. saturated 255 pulses in gap", decodes_all(t, 5));
  free(t);
  free(orig);
}

static void test_standard_fm(void) {
  track_t *t = malloc(sizeof(*t));
  rng_state = 3;
  encode(t, &standard_fm, 26, 0, 80, NULL, 7);
  report("c. standard_fm 26x128 with IAM round trip", decodes_all(t, 7));
  free(t);
}

static void test_corruption(void) {
  track_t *t = malloc(sizeof(*t));
  track_t *alt = malloc(sizeof(*t));
  rng_state = 4;
  encode(t, &atari_fm, 18, 0, 80, interleave18, 5);

  // Data: re-encode with one byte of slot 5 changed; the first differing
  // pulse lies in that sector's data field.
  memcpy(alt, t, sizeof(*t));
  alt->data[5 * 128 + 10] ^= 0x5a;
  alt->io.pulses = alt->pulses;
  alt->io.sectors = alt->data;
  encode_track_mfm(&alt->io);
  size_t d = 0;
  while (d < n_pulses && t->pulses[d] == alt->pulses[d])
    d++;
  bool ok = corrupt_from(t, d);
  setup_decode(t, 5);
  size_t n = decode_track_fm(&t->io);
  bool content_ok;
  size_t valid = check_sectors(t, &content_ok);
  ok = ok && n == 17 && valid == 17 && content_ok && !t->validity[5];
  report("d. corrupted data byte invalidates only that sector", ok);

  // IDAM: the first differing pulse between cylinder 5 and 6 is in the first
  // physical IDAM (slot 0).
  rng_state = 4;
  encode(t, &atari_fm, 18, 0, 80, interleave18, 5);
  memcpy(alt, t, sizeof(*t));
  alt->io.pulses = alt->pulses;
  alt->io.sectors = alt->data;
  alt->io.cylinder = 6;
  encode_track_mfm(&alt->io);
  d = 0;
  while (d < n_pulses && t->pulses[d] == alt->pulses[d])
    d++;
  ok = corrupt_from(t, d);
  setup_decode(t, 5);
  n = decode_track_fm(&t->io);
  valid = check_sectors(t, &content_ok);
  ok = ok && n == 17 && valid == 17 && content_ok && !t->validity[0];
  report("d. corrupted IDAM invalidates only that sector", ok);
  free(t);
  free(alt);
}

static void test_validate_id(void) {
  track_t *t = malloc(sizeof(*t));
  rng_state = 5;
  encode(t, &atari_fm, 18, 0, 80, interleave18, 5);
  setup_decode(t, 6);
  t->io.validate_id = true;
  size_t wrong = decode_track_fm(&t->io);
  setup_decode(t, 5);
  t->io.validate_id = true;
  size_t right = decode_track_fm(&t->io);
  setup_decode(t, 5);
  t->io.validate_id = true;
  t->io.head = 1;
  size_t wrong_head = decode_track_fm(&t->io);
  report("e. validate_id rejects wrong cylinder/head, accepts right one",
         wrong == 0 && right == 18 && wrong_head == 0);
  free(t);
}

static void test_mfm_sanity(void) {
  track_t *t = malloc(sizeof(*t));
  rng_state = 6;
  mfm_io_settings_t s = standard_mfm;
  s.skip_iam = true;
  encode(t, &s, 18, 1, 48, interleave18, 9);
  t->io.T2_max = 120; // 2.5 * T1_nom, as Adafruit_Floppy::set_timings
  t->io.T3_max = 168; // 3.5 * T1_nom
  setup_decode(t, 9);
  t->io.validate_id = true;
  size_t n = decode_track_mfm(&t->io);
  bool content_ok;
  size_t valid = check_sectors(t, &content_ok);
  report("f. MFM 18x256 interleaved round trip via decode_track_mfm",
         n == 18 && valid == 18 && content_ok);
  free(t);
}

int main(void) {
  test_round_trip();
  test_speed_jitter();
  test_standard_fm();
  test_corruption();
  test_validate_id();
  test_mfm_sanity();
  return failures != 0;
}
