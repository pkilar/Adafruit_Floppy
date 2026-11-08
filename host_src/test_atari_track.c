// SPDX-License-Identifier: MIT
//
// Host tests for src/atari_track.h: Atari physical track encode/decode,
// timing scaling for 288/300/360 RPM drives, interleave and density
// detection.

#include <stdio.h>
#include <stdlib.h>

#include "atari_track.h"

enum { sample_freq = 24000000, n_pulses = 66000 };

static int failures, passes;

static void report(const char *name, bool ok) {
  if (ok)
    passes++;
  else
    failures++;
  printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
}

static uint32_t rng_state = 987654321;
static uint32_t rng(void) {
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 8;
}

static const char *const density_name[] = {"SD", "ED", "DD", "none"};
static const atari_density_t densities[3] = {ATARI_DENSITY_SD, ATARI_DENSITY_ED,
                                             ATARI_DENSITY_DD};
static const unsigned rpms[3] = {288, 300, 360};

static uint8_t pulses[n_pulses];
static uint8_t scaled[n_pulses];
static uint8_t data[ATARI_TRACK_MAX_BYTES];
static uint8_t out[ATARI_TRACK_MAX_BYTES];
static uint8_t validity[ATARI_TRACK_MAX_SECTORS];

static uint32_t rev_us(unsigned rpm) { return 60000000u / rpm; }
static uint32_t rev_ticks(unsigned rpm) {
  return (uint32_t)((uint64_t)sample_freq * 60u / rpm);
}

static void fill_data(atari_density_t d) {
  atari_geometry_t g = atari_geometry_for_density(d);
  for (size_t i = 0; i < (size_t)g.sectors_per_track * g.sector_size; i++)
    data[i] = rng();
}

// Encode a full track for a drive at rpm; returns pulse count
static size_t encode(atari_density_t d, unsigned rpm, uint32_t *content) {
  uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(rpm));
  return atari_encode_track(d, 7, data, pulses, n_pulses, cell, rev_ticks(rpm),
                            content);
}

static bool all_match(atari_density_t d, size_t n_valid) {
  atari_geometry_t g = atari_geometry_for_density(d);
  if (n_valid != g.sectors_per_track)
    return false;
  for (size_t s = 0; s < g.sectors_per_track; s++)
    if (!validity[s] || memcmp(out + s * g.sector_size,
                               data + s * g.sector_size, g.sector_size))
      return false;
  return true;
}

static size_t decode(atari_density_t d, const uint8_t *p, size_t n,
                     unsigned rpm) {
  uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(rpm));
  memset(out, 0, sizeof(out));
  return atari_decode_track(d, 7, p, n, cell, out, validity, true);
}

// Scale every interval of a track encoded for 288 RPM to what a drive at
// rpm would read (the same disk spins faster, so intervals shrink)
static void scale_pulses(const uint8_t *p, size_t n, unsigned rpm) {
  for (size_t i = 0; i < n; i++)
    scaled[i] = (uint8_t)((p[i] * 288u + rpm / 2) / rpm);
}

static void test_density(atari_density_t d) {
  char name[96];
  const char *dn = density_name[d];
  atari_geometry_t g = atari_geometry_for_density(d);

  for (int r = 0; r < 3; r++) {
    unsigned rpm = rpms[r];
    fill_data(d);
    uint32_t content = 0;
    size_t n = encode(d, rpm, &content);
    double pct = 100.0 * content / rev_ticks(rpm);
    double total_pct = 100.0 * atari_pulse_ticks(pulses, n) / rev_ticks(rpm);
    printf("  %s @%u RPM: cell %u ticks, content %.2f ms (%.1f%%), track "
           "%.2f ms (%.1f%%) of %.2f ms rev\n",
           dn, rpm, atari_track_cell_ticks(d, sample_freq, rev_us(rpm)),
           content * 1000.0 / sample_freq, pct,
           atari_pulse_ticks(pulses, n) * 1000.0 / sample_freq, total_pct,
           rev_us(rpm) / 1000.0);

    snprintf(name, sizeof(name), "%s %u RPM fits in a revolution with margin",
             dn, rpm);
    report(name, n > 0 && content <= rev_ticks(rpm) - rev_ticks(rpm) / 50 &&
                     atari_pulse_ticks(pulses, n) <= rev_ticks(rpm));

    size_t nv = decode(d, pulses, n, rpm);
    snprintf(name, sizeof(name), "%s %u RPM round-trips all sectors", dn, rpm);
    report(name, all_match(d, nv));

    // The raw codec sees inverted data
    mfm_io_t io;
    memset(&io, 0, sizeof(io));
    uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(rpm));
    io.T1_nom = cell;
    io.T2_max = (5 * cell + 1) / 2;
    io.T3_max = (7 * cell + 1) / 2;
    io.pulses = pulses;
    io.n_pulses = n;
    io.sectors = out;
    io.n_sectors = g.sectors_per_track;
    io.sector_validity = validity;
    io.n = g.sector_size == 256;
    io.cylinder = 7;
    io.validate_id = true;
    memset(validity, 0, sizeof(validity));
    if (g.mfm)
      decode_track_mfm(&io);
    else
      decode_track_fm(&io);
    bool inverted = io.n_valid == g.sectors_per_track;
    for (size_t i = 0;
         inverted && i < (size_t)g.sectors_per_track * g.sector_size; i++)
      inverted = out[i] == (uint8_t)~data[i];
    snprintf(name, sizeof(name), "%s %u RPM on-disk data is inverted", dn, rpm);
    report(name, inverted);

    // data buffer must be restored after encoding
    uint8_t first = data[0];
    (void)encode(d, rpm, NULL);
    snprintf(name, sizeof(name), "%s %u RPM encode restores caller data", dn,
             rpm);
    report(name, data[0] == first);
  }

  // Track made for a 288 RPM drive read on 300 and 360 RPM drives
  fill_data(d);
  size_t n = encode(d, 288, NULL);
  for (int r = 1; r < 3; r++) {
    scale_pulses(pulses, n, rpms[r]);
    size_t nv = decode(d, scaled, n, rpms[r]);
    snprintf(name, sizeof(name), "%s 288 RPM track reads on %u RPM drive", dn,
             rpms[r]);
    report(name, all_match(d, nv));
  }

  // Interleave: truncate the flux to see which sector completes first
  n = encode(d, 288, NULL);
  const uint8_t *order = atari_track_order(d);
  uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(288));
  uint8_t seen[ATARI_TRACK_MAX_SECTORS];
  memset(seen, 0, sizeof(seen));
  bool order_ok = true;
  for (size_t k = 0; k < g.sectors_per_track; k++) {
    size_t lo = 0, hi = n;
    while (lo < hi) { // smallest prefix with k + 1 valid sectors
      size_t mid = (lo + hi) / 2;
      size_t nv =
          atari_decode_track(d, 7, pulses, mid, cell, out, validity, true);
      if (nv > k)
        hi = mid;
      else
        lo = mid + 1;
    }
    atari_decode_track(d, 7, pulses, lo, cell, out, validity, true);
    int fresh = -1;
    for (size_t s = 0; s < g.sectors_per_track; s++)
      if (validity[s] && !seen[s]) {
        fresh = (int)s;
        seen[s] = 1;
      }
    if (fresh != order[k])
      order_ok = false;
  }
  snprintf(name, sizeof(name), "%s physical sector order matches interleave",
           dn);
  report(name, order_ok);

  // +-5% per-pulse jitter
  for (int r = 0; r < 3; r++) {
    n = encode(d, rpms[r], NULL);
    for (size_t i = 0; i < n; i++) {
      int v = pulses[i];
      int j = (int)(rng() % 1001) - 500; // -5.00% .. +5.00%
      scaled[i] = (uint8_t)(v + (v * j + (j < 0 ? -5000 : 5000)) / 10000);
    }
    size_t nv = decode(d, scaled, n, rpms[r]);
    snprintf(name, sizeof(name), "%s %u RPM decodes with +-5%% jitter", dn,
             rpms[r]);
    report(name, all_match(d, nv));
  }
}

static void test_detect(void) {
  char name[96];
  uint8_t scratch[ATARI_TRACK_MAX_BYTES];
  for (int i = 0; i < 3; i++) {
    for (int r = 0; r < 3; r++) {
      fill_data(densities[i]);
      size_t n = encode(densities[i], rpms[r], NULL);
      size_t nv = 0;
      atari_density_t got = atari_detect_density(
          pulses, n, 7, sample_freq, rev_us(rpms[r]), scratch, validity, &nv);
      snprintf(name, sizeof(name), "detect %s at %u RPM", density_name[i],
               rpms[r]);
      report(name, got == densities[i] &&
                       nv == atari_geometry_for_density(got).sectors_per_track);
    }
  }
  // garbage
  for (size_t i = 0; i < 40000; i++)
    pulses[i] = 24 + rng() % 200;
  report("detect garbage -> none",
         atari_detect_density(pulses, 40000, 7, sample_freq, rev_us(300),
                              scratch, validity, NULL) == ATARI_DENSITY_OTHER);
  report("detect empty -> none",
         atari_detect_density(pulses, 0, 7, sample_freq, rev_us(300), scratch,
                              validity, NULL) == ATARI_DENSITY_OTHER);
  // wrong cylinder is not accepted
  fill_data(ATARI_DENSITY_DD);
  size_t n = encode(ATARI_DENSITY_DD, 300, NULL);
  report("detect wrong cylinder -> none",
         atari_detect_density(pulses, n, 8, sample_freq, rev_us(300), scratch,
                              validity, NULL) == ATARI_DENSITY_OTHER);
}

// Encode for a head, decode for another; returns valid sector count
static size_t cross_decode(atari_density_t d, unsigned rpm, uint8_t enc_head,
                           uint8_t dec_head, uint8_t enc_cyl, uint8_t dec_cyl) {
  uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(rpm));
  size_t n = atari_encode_track_head(d, enc_cyl, enc_head, data, pulses,
                                     n_pulses, cell, rev_ticks(rpm), NULL);
  memset(out, 0, sizeof(out));
  return n ? atari_decode_track_head(d, dec_cyl, dec_head, pulses, n, cell, out,
                                     validity, true)
           : 0;
}

static void test_head1(atari_density_t d) {
  char name[96];
  atari_geometry_t g = atari_geometry_for_density(d);
  for (int r = 1; r < 3; r++) {
    unsigned rpm = rpms[r];
    fill_data(d);
    size_t v = cross_decode(d, rpm, 1, 1, 9, 9);
    snprintf(name, sizeof(name), "%s head 1 round trip at %u RPM",
             density_name[d], rpm);
    report(name, all_match(d, v));
    v = cross_decode(d, rpm, 0, 1, 9, 9);
    snprintf(name, sizeof(name), "%s head 1 accepts side byte 0 at %u RPM",
             density_name[d], rpm);
    report(name, all_match(d, v));
  }
  report("head 0 rejects side byte 1", cross_decode(d, 300, 1, 0, 9, 9) == 0);
  report("head 1 rejects wrong cylinder",
         cross_decode(d, 300, 1, 1, 9, 10) == 0);
  report("head 2 is refused",
         atari_encode_track_head(d, 0, 2, data, pulses, n_pulses, 48,
                                 rev_ticks(300), NULL) == 0);
  (void)g;
}

static bool loc(const atari_geometry_t *g, uint16_t sector, int cyl, int head,
                int slot) {
  uint8_t c, h;
  uint16_t sl;
  return atari_sector_location(g, sector, &c, &h, &sl) && c == cyl &&
         h == head && sl == slot;
}

static void test_geometry_ds(void) {
  atari_geometry_t g = atari_geometry_make(ATARI_DENSITY_DD, 40, 2);
  report("DS 40x2x18 sector count", g.sector_count == 1440);
  report("sector 720 is track 39 ID 18 head 0", loc(&g, 720, 39, 0, 17));
  report("sector 721 is track 39 ID 18 head 1", loc(&g, 721, 39, 1, 17));
  report("sector 722 is track 39 ID 17 head 1", loc(&g, 722, 39, 1, 16));
  report("sector 1440 is track 0 ID 1 head 1", loc(&g, 1440, 0, 1, 0));
  report("sector 1441 is out of range", !loc(&g, 1441, 0, 1, 0));
  atari_geometry_t g80 = atari_geometry_make(ATARI_DENSITY_DD, 80, 2);
  report("DS 80x2x18 sector count", g80.sector_count == 2880);
  report("sector 2880 is track 0 ID 1 head 1", loc(&g80, 2880, 0, 1, 0));
  report("sector 1441 is track 79 ID 18 head 1", loc(&g80, 1441, 79, 1, 17));
  report("track density of DS/80 geometry",
         atari_track_density(&g80) == ATARI_DENSITY_DD);
}

// A head 1 track of a DS or 80-track disk still fits the revolution
static void test_fit_ds(void) {
  char name[96];
  for (int i = 0; i < 3; i++)
    for (int r = 1; r < 3; r++) {
      atari_density_t d = densities[i];
      fill_data(d);
      uint16_t cell = atari_track_cell_ticks(d, sample_freq, rev_us(rpms[r]));
      size_t n = atari_encode_track_head(d, 79, 1, data, pulses, n_pulses, cell,
                                         rev_ticks(rpms[r]), NULL);
      size_t v = n ? atari_decode_track_head(d, 79, 1, pulses, n, cell, out,
                                             validity, true)
                   : 0;
      snprintf(name, sizeof(name), "%s track 79 head 1 fits at %u RPM",
               density_name[d], rpms[r]);
      report(name, n && all_match(d, v));
    }
}

static void test_cell_ticks(void) {
  report("cell ticks 24 MHz 288 RPM",
         atari_track_cell_ticks(ATARI_DENSITY_SD, sample_freq, rev_us(288)) ==
                 96 &&
             atari_track_cell_ticks(ATARI_DENSITY_DD, sample_freq,
                                    rev_us(288)) == 48);
  report("cell ticks overflow guard",
         atari_track_cell_ticks(ATARI_DENSITY_SD, 48000000, rev_us(288)) == 0 &&
             atari_track_cell_ticks(ATARI_DENSITY_OTHER, sample_freq,
                                    rev_us(288)) == 0);
}

int main(void) {
  test_cell_ticks();
  for (int i = 0; i < 3; i++)
    test_density(densities[i]);
  test_detect();
  for (int i = 0; i < 3; i++)
    test_head1(densities[i]);
  test_geometry_ds();
  test_fit_ds();
  printf("%d passed, %d failed\n", passes, failures);
  return failures ? 1 : 0;
}
