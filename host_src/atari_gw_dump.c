// SPDX-License-Identifier: MIT
//
// Plumbing for check_atari_gw.py: cross-checks src/atari_track.h against
// Greaseweazle's Python IBM codec.
//
//   atari_gw_dump encode DIR   encode one track per density (cylinder 7,
//                              random SIO data; plus DD on head 1 as "dd1") for
//                              a 300 RPM drive at 24 MHz. Writes
//                              DIR/enc_{sd,ed,dd,dd1}.{pulses,sio,meta}.
//   atari_gw_dump decode DIR   decode DIR/gw_{sd,ed,dd,dd1}.pulses (flux made
//                              by Greaseweazle; .meta holds "density cylinder
//                              head")
//                              and write DIR/gw_*.out: one validity byte per
//                              sector followed by the SIO data.
//
// pulses files hold raw uint8 flux intervals; enc meta is the text line
// "density cylinder cell_ticks rev_ticks n_pulses head".

#include <stdio.h>
#include <stdlib.h>

#include "atari_track.h"

enum { sample_freq = 24000000, rev_us = 200000 }; // 300 RPM
#define CYLINDER 7

enum { n_tracks = 4 };
static const char *const names[n_tracks] = {"sd", "ed", "dd", "dd1"};
static const atari_density_t densities[n_tracks] = {
    ATARI_DENSITY_SD, ATARI_DENSITY_ED, ATARI_DENSITY_DD, ATARI_DENSITY_DD};
static const uint8_t heads[n_tracks] = {0, 0, 0, 1};

static uint8_t pulses[ATARI_TRACK_MAX_PULSES + 1000000 / 10];
static uint8_t data[ATARI_TRACK_MAX_BYTES];
static uint8_t out[ATARI_TRACK_MAX_BYTES];
static uint8_t validity[ATARI_TRACK_MAX_SECTORS];

static uint32_t rng_state = 20260101;
static uint32_t rng(void) {
  rng_state = rng_state * 1664525u + 1013904223u;
  return rng_state >> 8;
}

static int write_file(const char *dir, const char *prefix, const char *name,
                      const char *ext, const void *buf, size_t len) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s_%s.%s", dir, prefix, name, ext);
  FILE *f = fopen(path, "wb");
  if (!f || fwrite(buf, 1, len, f) != len) {
    perror(path);
    return -1;
  }
  fclose(f);
  return 0;
}

static long read_file(const char *dir, const char *prefix, const char *name,
                      const char *ext, void *buf, size_t max) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s_%s.%s", dir, prefix, name, ext);
  FILE *f = fopen(path, "rb");
  if (!f) {
    perror(path);
    return -1;
  }
  size_t n = fread(buf, 1, max, f);
  fclose(f);
  return (long)n;
}

static int do_encode(const char *dir) {
  for (int i = 0; i < n_tracks; i++) {
    atari_geometry_t g = atari_geometry_for_density(densities[i]);
    size_t total = (size_t)g.sectors_per_track * g.sector_size;
    for (size_t b = 0; b < total; b++)
      data[b] = (uint8_t)rng();
    uint16_t cell = atari_track_cell_ticks(densities[i], sample_freq, rev_us);
    uint32_t rev_ticks = (uint32_t)((uint64_t)sample_freq * rev_us / 1000000u);
    size_t n =
        atari_encode_track_head(densities[i], CYLINDER, heads[i], data, pulses,
                                sizeof(pulses), cell, rev_ticks, NULL);
    if (!n) {
      fprintf(stderr, "encode %s failed\n", names[i]);
      return 1;
    }
    char meta[128];
    int ml =
        snprintf(meta, sizeof(meta), "%d %d %u %u %zu %d\n", (int)densities[i],
                 CYLINDER, cell, rev_ticks, n, (int)heads[i]);
    if (write_file(dir, "enc", names[i], "pulses", pulses, n) ||
        write_file(dir, "enc", names[i], "sio", data, total) ||
        write_file(dir, "enc", names[i], "meta", meta, (size_t)ml))
      return 1;
  }
  return 0;
}

static int do_decode(const char *dir) {
  int rc = 0;
  for (int i = 0; i < n_tracks; i++) {
    char meta[128] = {0};
    long ml = read_file(dir, "gw", names[i], "meta", meta, sizeof(meta) - 1);
    long n = read_file(dir, "gw", names[i], "pulses", pulses, sizeof(pulses));
    if (ml < 0 || n < 0)
      return 1;
    int d, cyl, head;
    if (sscanf(meta, "%d %d %d", &d, &cyl, &head) != 3 ||
        d != (int)densities[i] || head != (int)heads[i]) {
      fprintf(stderr, "bad meta for %s\n", names[i]);
      return 1;
    }
    atari_geometry_t g = atari_geometry_for_density(densities[i]);
    size_t total = (size_t)g.sectors_per_track * g.sector_size;
    uint16_t cell = atari_track_cell_ticks(densities[i], sample_freq, rev_us);
    memset(out, 0, sizeof(out));
    size_t valid =
        atari_decode_track_head(densities[i], (uint8_t)cyl, (uint8_t)head,
                                pulses, (size_t)n, cell, out, validity, true);
    printf("%s: %zu/%u sectors valid\n", names[i], valid,
           (unsigned)g.sectors_per_track);
    uint8_t buf[ATARI_TRACK_MAX_SECTORS + ATARI_TRACK_MAX_BYTES];
    memcpy(buf, validity, g.sectors_per_track);
    memcpy(buf + g.sectors_per_track, out, total);
    if (write_file(dir, "gw", names[i], "out", buf,
                   g.sectors_per_track + total))
      rc = 1;
  }
  return rc;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "usage: %s encode|decode DIR\n", argv[0]);
    return 2;
  }
  if (!strcmp(argv[1], "encode"))
    return do_encode(argv[2]);
  if (!strcmp(argv[1], "decode"))
    return do_decode(argv[2]);
  fprintf(stderr, "unknown mode %s\n", argv[1]);
  return 2;
}
