// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file atari_track.h
 *
 * Atari 8-bit physical track layout on top of the mfm_impl.h codecs: gap
 * settings, sector interleave, data inversion, timing scaling for PC drives
 * and density detection. Plain C with libc only so host_src can test it.
 *
 * On-disk layout (US Doubler 1050 ROM, WRITETRACK): single side, ID side byte
 * 0 (head number on side 1), IDAM FE, DAM FB, no index address mark. Sector
 * data is stored one's complemented (the drive ROM XORs it with 0xFF); IDs,
 * marks and CRCs are not. Callers of this file always see the SIO (true) data;
 * inversion is done here.
 *
 * Gaps are re-budgeted from the ROM's WRITETRACK parameters, whose byte
 * streams (3300 / 6598 / 6728 bytes) are longer than one 288 RPM revolution
 * (3255 FM / 6510 MFM bytes). Only the lead-in is shortened (the sector
 * gaps match the ROM):
 *   SD: gap 0xFF, presync 6, gap2 11, gap3 8,  lead-in 40  -> 3074 bytes
 *   ED: gap 0x4E, presync 12, gap2 22, gap3 49, lead-in 30 -> 6195 bytes
 *   DD: gap 0x4E, presync 12, gap2 22, gap3 22, lead-in 30 -> 6128 bytes
 *
 * Do not include this together with mfm_impl.h (which has no include guard).
 */
#ifndef ATARI_TRACK_H
#define ATARI_TRACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "atari_disk.h"
#include "mfm_impl.h"

/** Most sectors on an Atari track (enhanced density). */
#define ATARI_TRACK_MAX_SECTORS 26
/** Most sector bytes on an Atari track (18 x 256, double density). */
#define ATARI_TRACK_MAX_BYTES 4608
/**
 * Pulses needed to hold the worst-case encoded track: an FM or MFM track is
 * under one revolution of bytes (3256 FM at 16 pulses per byte, 6511 MFM at
 * 8 pulses per byte), so about 52100 pulses.
 */
#define ATARI_TRACK_MAX_PULSES 52100
/** The encoded track is cut at 1 - 1/ATARI_TRACK_MARGIN_DIV of a revolution. */
#define ATARI_TRACK_MARGIN_DIV 50

/// @cond false

// Fields: gap_1 (lead-in), gap_2, gap_3[n], gap_4a, presync, gap byte, is_fm,
// skip_iam. Presync, gap2 and gap3 counts are the US Doubler 1050 ROM tables.
MFM_MAYBE_UNUSED
static const mfm_io_settings_t atari_settings_sd = {
    40, 11, {8, 8, 8, 8, 8, 8, 8, 8}, 0, 6, 0xff, true, true,
};
MFM_MAYBE_UNUSED
static const mfm_io_settings_t atari_settings_ed = {
    30, 22, {49, 49, 49, 49, 49, 49, 49, 49}, 0, 12, 0x4e, false, true,
};
MFM_MAYBE_UNUSED
static const mfm_io_settings_t atari_settings_dd = {
    30, 22, {22, 22, 22, 22, 22, 22, 22, 22}, 0, 12, 0x4e, false, true,
};

// Physical order of sectors as 0-based slots (ID number - 1); US Doubler 1050
// ROM tables and the stock 1050 (AtariSIO tools/Atari1050Model.cpp sector
// positions): SD 1,3,..17,2,4,..18; ED 1,3,..25,2,4,..26; DD (US Doubler)
// 18,17,..1.
MFM_MAYBE_UNUSED
static const uint8_t atari_order_sd[18] = {0, 2, 4, 6, 8, 10, 12, 14, 16,
                                           1, 3, 5, 7, 9, 11, 13, 15, 17};
MFM_MAYBE_UNUSED
static const uint8_t atari_order_ed[26] = {0,  2,  4,  6,  8,  10, 12, 14, 16,
                                           18, 20, 22, 24, 1,  3,  5,  7,  9,
                                           11, 13, 15, 17, 19, 21, 23, 25};
MFM_MAYBE_UNUSED
static const uint8_t atari_order_dd[18] = {17, 16, 15, 14, 13, 12, 11, 10, 9,
                                           8,  7,  6,  5,  4,  3,  2,  1,  0};

/// @endcond

/*!
 * @brief Codec settings for a density.
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @return The settings, or NULL for any other density.
 */
MFM_MAYBE_UNUSED
static const mfm_io_settings_t *atari_track_settings(atari_density_t density) {
  switch (density) {
  case ATARI_DENSITY_SD:
    return &atari_settings_sd;
  case ATARI_DENSITY_ED:
    return &atari_settings_ed;
  case ATARI_DENSITY_DD:
    return &atari_settings_dd;
  default:
    return NULL;
  }
}

/*!
 * @brief Physical sector order of a density.
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @return Array of sectors_per_track 0-based slots in physical order, or NULL.
 */
MFM_MAYBE_UNUSED
static const uint8_t *atari_track_order(atari_density_t density) {
  switch (density) {
  case ATARI_DENSITY_SD:
    return atari_order_sd;
  case ATARI_DENSITY_ED:
    return atari_order_ed;
  case ATARI_DENSITY_DD:
    return atari_order_dd;
  default:
    return NULL;
  }
}

/*!
 * @brief Flux ticks of one codec cell on this drive (mfm_io T1_nom).
 *
 * A cell is 4 us (FM) or 2 us (MFM) at the Atari's 288 RPM. The disk always
 * holds the same number of cells per revolution (52083.3 FM, 104166.7 MFM;
 * scaled from the 288 RPM Atari layout), so the cell is the revolution time
 * divided by that count. Pass the measured revolution time where possible.
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @param sample_freq_hz Flux sample rate in Hz.
 * @param rev_time_us Revolution time in microseconds (60000000 / RPM when not
 * measured).
 * @return Cell ticks, or 0 for a bad argument or when the longest interval
 * (2 cells FM, 4 cells MFM) would not fit a uint8_t pulse.
 */
MFM_MAYBE_UNUSED
static uint16_t atari_track_cell_ticks(atari_density_t density,
                                       uint32_t sample_freq_hz,
                                       uint32_t rev_time_us) {
  if (density > ATARI_DENSITY_DD || !sample_freq_hz || !rev_time_us)
    return 0;
  bool fm = density == ATARI_DENSITY_SD;
  uint64_t cells_x3 = fm ? 156250u : 312500u; // cells per revolution x 3
  uint64_t num = (uint64_t)sample_freq_hz * rev_time_us * 3u;
  uint64_t den = 1000000u * cells_x3;
  uint64_t ticks = (num + den / 2) / den;
  if (ticks == 0 || ticks * (fm ? 2u : 4u) > 255u)
    return 0;
  return (uint16_t)ticks;
}

/*!
 * @brief Sum of flux ticks.
 * @param pulses Flux intervals.
 * @param n Number of intervals to add.
 * @return Total ticks.
 */
MFM_MAYBE_UNUSED
static uint32_t atari_pulse_ticks(const uint8_t *pulses, size_t n) {
  uint32_t sum = 0;
  for (size_t i = 0; i < n; i++)
    sum += pulses[i];
  return sum;
}

/*!
 * @brief One's complement sector data (SIO data <-> on-disk data).
 * @param sectors Sector data, sectors_per_track * sector_size bytes.
 * @param validity Per-sector flags; only sectors flagged are inverted. NULL
 * inverts every sector.
 * @param n_sectors Number of sectors.
 * @param sector_size Bytes per sector.
 */
MFM_MAYBE_UNUSED
static void atari_invert_sectors(uint8_t *sectors, const uint8_t *validity,
                                 size_t n_sectors, size_t sector_size) {
  for (size_t s = 0; s < n_sectors; s++) {
    if (validity && !validity[s])
      continue;
    for (size_t i = 0; i < sector_size; i++)
      sectors[s * sector_size + i] ^= 0xff;
  }
}

/*!
 * @brief Encode a full Atari track as flux.
 *
 * The sector data is inverted in place for encoding and restored before
 * returning. The encoder fills the rest of the buffer with gap bytes; the
 * result is cut so its duration is rev_ticks minus 1/ATARI_TRACK_MARGIN_DIV.
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @param cylinder Physical track number written into the sector IDs (both
 * sides carry the physical cylinder).
 * @param head Side, 0 or 1; written as the ID side byte.
 * @param sectors SIO (non-inverted) data, sectors_per_track slots in ID order
 * (slot i has ID i + 1), each atari_geometry_for_density() sector_size bytes.
 * @param pulses Receives flux intervals.
 * @param max_pulses Capacity of pulses; at least ATARI_TRACK_MAX_PULSES.
 * @param cell_ticks From atari_track_cell_ticks().
 * @param rev_ticks Length of one revolution in ticks.
 * @param content_ticks If not NULL, receives the ticks up to the end of the
 * last sector's CRC.
 * @return Number of pulses, or 0 if the sectors do not fit in the margin or
 * an argument is bad.
 */
MFM_MAYBE_UNUSED
static size_t atari_encode_track_head(atari_density_t density, uint8_t cylinder,
                                      uint8_t head, uint8_t *sectors,
                                      uint8_t *pulses, size_t max_pulses,
                                      uint16_t cell_ticks, uint32_t rev_ticks,
                                      uint32_t *content_ticks) {
  const mfm_io_settings_t *settings = atari_track_settings(density);
  if (!settings || !cell_ticks || head > 1 ||
      max_pulses < ATARI_TRACK_MAX_PULSES)
    return 0;
  atari_geometry_t g = atari_geometry_for_density(density);

  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.settings = settings;
  io.T1_nom = cell_ticks;
  io.pulses = pulses;
  io.n_pulses = max_pulses;
  io.sectors = sectors;
  io.n_sectors = g.sectors_per_track;
  io.n = g.sector_size == 256 ? 1 : 0;
  io.cylinder = cylinder;
  io.head = head;
  io.sector_order = atari_track_order(density);

  size_t n_sectors = g.sectors_per_track;
  atari_invert_sectors(sectors, NULL, n_sectors, g.sector_size);
  size_t data_end = encode_track_mfm(&io);
  atari_invert_sectors(sectors, NULL, n_sectors, g.sector_size);

  uint32_t limit = rev_ticks - rev_ticks / ATARI_TRACK_MARGIN_DIV;
  uint32_t sum = atari_pulse_ticks(pulses, data_end);
  if (content_ticks)
    *content_ticks = sum;
  if (sum > limit)
    return 0;
  size_t n = data_end;
  while (n < max_pulses && sum + pulses[n] <= limit)
    sum += pulses[n++];
  return n;
}

/*!
 * @brief Encode a full side 0 Atari track; see atari_encode_track_head().
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @param cylinder Track number written into the sector IDs.
 * @param sectors SIO (non-inverted) data in ID order.
 * @param pulses Receives flux intervals.
 * @param max_pulses Capacity of pulses.
 * @param cell_ticks From atari_track_cell_ticks().
 * @param rev_ticks Length of one revolution in ticks.
 * @param content_ticks If not NULL, receives the ticks of the content.
 * @return Number of pulses, or 0 on failure.
 */
MFM_MAYBE_UNUSED
static size_t atari_encode_track(atari_density_t density, uint8_t cylinder,
                                 uint8_t *sectors, uint8_t *pulses,
                                 size_t max_pulses, uint16_t cell_ticks,
                                 uint32_t rev_ticks, uint32_t *content_ticks) {
  return atari_encode_track_head(density, cylinder, 0, sectors, pulses,
                                 max_pulses, cell_ticks, rev_ticks,
                                 content_ticks);
}

/*!
 * @brief Decode a captured Atari track.
 *
 * Sectors found are returned as SIO (un-inverted) data. Sector IDs must
 * match the cylinder, the side and the density's size code. On head 1 the ID
 * side byte may be 0 or 1 (a real XF551 does not compare it and its value is
 * undocumented; the US Doubler writes 0); on head 0 it must be 0.
 * @param density ATARI_DENSITY_SD (FM), _ED or _DD (MFM).
 * @param cylinder Physical track number expected in the sector IDs.
 * @param head Side being read, 0 or 1.
 * @param pulses Captured flux intervals.
 * @param n Number of intervals.
 * @param cell_ticks From atari_track_cell_ticks().
 * @param sectors Receives sectors_per_track * sector_size bytes of SIO data.
 * @param validity Receives one flag per sector (ID order).
 * @param clear_validity true to start empty; false to keep sectors already
 * valid from an earlier capture and only add missing ones.
 * @return Number of valid sectors after decoding (0 for bad arguments).
 */
MFM_MAYBE_UNUSED
static size_t atari_decode_track_head(atari_density_t density, uint8_t cylinder,
                                      uint8_t head, const uint8_t *pulses,
                                      size_t n, uint16_t cell_ticks,
                                      uint8_t *sectors, uint8_t *validity,
                                      bool clear_validity) {
  const mfm_io_settings_t *settings = atari_track_settings(density);
  if (!settings || !cell_ticks || head > 1)
    return 0;
  atari_geometry_t g = atari_geometry_for_density(density);
  size_t n_sectors = g.sectors_per_track;

  // The codec works on on-disk bytes, so earlier results go back to that form
  if (clear_validity)
    memset(validity, 0, n_sectors);
  else
    atari_invert_sectors(sectors, validity, n_sectors, g.sector_size);

  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.settings = settings;
  io.T1_nom = cell_ticks;
  io.T2_max = (uint16_t)((5u * cell_ticks + 1u) / 2u); // 2.5 cells
  io.T3_max = (uint16_t)((7u * cell_ticks + 1u) / 2u); // 3.5 cells
  io.pulses = (uint8_t *)pulses;
  io.n_pulses = n;
  io.sectors = sectors;
  io.n_sectors = n_sectors;
  io.sector_validity = validity;
  io.n = g.sector_size == 256 ? 1 : 0;
  io.cylinder = cylinder;
  io.head = head;
  io.validate_id = true;
  io.any_head = true; // XF551 side 1 ID side byte is 0 or 1

  if (settings->is_fm)
    decode_track_fm(&io);
  else
    decode_track_mfm(&io);

  atari_invert_sectors(sectors, validity, n_sectors, g.sector_size);
  return io.n_valid;
}

/*!
 * @brief Decode a captured side 0 Atari track; see atari_decode_track_head().
 * @param density ATARI_DENSITY_SD (FM), _ED or _DD (MFM).
 * @param cylinder Track number expected in the sector IDs.
 * @param pulses Captured flux intervals.
 * @param n Number of intervals.
 * @param cell_ticks From atari_track_cell_ticks().
 * @param sectors Receives sectors_per_track * sector_size bytes of SIO data.
 * @param validity Receives one flag per sector (ID order).
 * @param clear_validity true to start empty.
 * @return Number of valid sectors after decoding.
 */
MFM_MAYBE_UNUSED
static size_t atari_decode_track(atari_density_t density, uint8_t cylinder,
                                 const uint8_t *pulses, size_t n,
                                 uint16_t cell_ticks, uint8_t *sectors,
                                 uint8_t *validity, bool clear_validity) {
  return atari_decode_track_head(density, cylinder, 0, pulses, n, cell_ticks,
                                 sectors, validity, clear_validity);
}

/*!
 * @brief Work out which Atari density a captured track of a side holds.
 *
 * Tries ED (MFM, 26 x 128), DD (MFM, 18 x 256) and SD (FM, 18 x 128) on the
 * same flux. The density with the most valid sectors wins.
 * @param pulses Captured flux intervals.
 * @param n Number of intervals.
 * @param cylinder Track number expected in the sector IDs.
 * @param head Side, 0 or 1.
 * @param sample_freq_hz Flux sample rate in Hz.
 * @param rev_time_us Revolution time in microseconds.
 * @param scratch Work buffer of ATARI_TRACK_MAX_BYTES bytes.
 * @param validity Work buffer of ATARI_TRACK_MAX_SECTORS bytes.
 * @param n_valid If not NULL, receives the winner's valid sector count.
 * @return The density, or ATARI_DENSITY_OTHER if nothing decoded or two
 * densities tie.
 */
MFM_MAYBE_UNUSED
static atari_density_t
atari_detect_density_head(const uint8_t *pulses, size_t n, uint8_t cylinder,
                          uint8_t head, uint32_t sample_freq_hz,
                          uint32_t rev_time_us, uint8_t *scratch,
                          uint8_t *validity, size_t *n_valid) {
  static const atari_density_t candidates[3] = {
      ATARI_DENSITY_ED, ATARI_DENSITY_DD, ATARI_DENSITY_SD};
  size_t best = 0;
  bool tie = false;
  atari_density_t found = ATARI_DENSITY_OTHER;
  for (int i = 0; i < 3; i++) {
    uint16_t cell =
        atari_track_cell_ticks(candidates[i], sample_freq_hz, rev_time_us);
    size_t count =
        atari_decode_track_head(candidates[i], cylinder, head, pulses, n, cell,
                                scratch, validity, true);
    if (count > best) {
      best = count;
      found = candidates[i];
      tie = false;
    } else if (count == best && count > 0) {
      tie = true;
    }
  }
  if (n_valid)
    *n_valid = best;
  return tie ? ATARI_DENSITY_OTHER : found;
}

/*!
 * @brief Side 0 density detection; see atari_detect_density_head().
 * @param pulses Captured flux intervals.
 * @param n Number of intervals.
 * @param cylinder Track number expected in the sector IDs.
 * @param sample_freq_hz Flux sample rate in Hz.
 * @param rev_time_us Revolution time in microseconds.
 * @param scratch Work buffer of ATARI_TRACK_MAX_BYTES bytes.
 * @param validity Work buffer of ATARI_TRACK_MAX_SECTORS bytes.
 * @param n_valid If not NULL, receives the winner's valid sector count.
 * @return The density or ATARI_DENSITY_OTHER.
 */
MFM_MAYBE_UNUSED
static atari_density_t
atari_detect_density(const uint8_t *pulses, size_t n, uint8_t cylinder,
                     uint32_t sample_freq_hz, uint32_t rev_time_us,
                     uint8_t *scratch, uint8_t *validity, size_t *n_valid) {
  return atari_detect_density_head(pulses, n, cylinder, 0, sample_freq_hz,
                                   rev_time_us, scratch, validity, n_valid);
}

#endif // ATARI_TRACK_H
