// SPDX-FileCopyrightText: 2026 Adafruit Industries
//
// SPDX-License-Identifier: MIT

/*!
 * @file atari_prot_track.h
 *
 * Copy-protected Atari tracks at the flux level: scan every sector ID of a
 * captured revolution (duplicates, bad CRCs, missing data, deleted marks,
 * long sectors), merge several revolutions to find weak sectors, and master
 * an atari_prot_track_t back to flux. Plain C with libc only so host_src can
 * test it; built on the static codecs of mfm_impl.h and atari_track.h.
 * Do not include this together with mfm_impl.h (which has no include guard);
 * this file includes it through atari_track.h.
 *
 * Capture timing assumed: the flux buffer starts at an index pulse (as from
 * Adafruit_FloppyBase::capture_track, which waits for the index first), time
 * 0 being the first interval. The caller passes the measured index-to-index
 * time in ticks (the sum of the first falling_index_offset intervals). The
 * buffer may extend past one revolution (usually 1.25): data fields that
 * straddle the end of the first revolution are read in full, while IDs found
 * at or after one revolution are duplicates of the first revolution and are
 * ignored.
 *
 * A position is the angular start of an entry's ID address mark (the first
 * A1 of the three-A1 sync for MFM, the FE mark byte for FM; the preceding
 * zero bytes are not counted), in ATARI_ATX_REV_UNITS per revolution.
 */

#ifndef ATARI_PROT_TRACK_H
#define ATARI_PROT_TRACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "atari_track.h"

#include "atari_atx.h"

/** Gap bytes always left between the end of one entry and the next. */
#define ATARI_PROT_MIN_GAP 8
/** Gap bytes before the first entry, after the index. */
#define ATARI_PROT_LEAD_GAP 4
/** Largest data field read or written, in bytes (size code 3). */
#define ATARI_PROT_MAX_FIELD 1024
/** Default merge tolerance, in angular units (about 1.5% of a revolution). */
#define ATARI_PROT_MERGE_TOL 400

/*!
 * @brief One sector ID found on a track, with the data field that follows.
 */
typedef struct atari_prot_found {
  uint8_t number;      ///< ID sector number
  uint8_t track;       ///< ID cylinder (equals the one asked for)
  uint8_t side;        ///< ID side byte (not checked)
  uint8_t size_code;   ///< ID size code, 0..3
  uint32_t ticks;      ///< Flux ticks from the buffer start to the ID mark
  uint8_t data_mark;   ///< 0xFB normal, 0xF8 deleted, 0 = no data field
  bool data_crc_ok;    ///< Data CRC was good (false when data_mark is 0)
  uint16_t data_len;   ///< Bytes in data: 128 << size_code (0 if no field)
  const uint8_t *data; ///< Data in SIO form (un-inverted); valid in callback
} atari_prot_found_t;

/**
 * @brief Callback for atari_prot_walk().
 * @param ctx The context passed to atari_prot_walk().
 * @param found The sector; data is only valid during the call.
 * @return true to continue, false to stop the walk.
 */
typedef bool (*atari_prot_found_cb)(void *ctx, const atari_prot_found_t *found);

/// @cond false
// Running sum of flux ticks with a movable cursor, so the walkers can look up
// the time of a pulse index cheaply and in either direction.
typedef struct {
  const uint8_t *p;
  size_t n;
  size_t pos;
  uint32_t ticks;
} atari_prot_clock_t;

typedef struct {
  atari_prot_found_cb cb;
  void *ctx;
  atari_prot_clock_t clock;
  bool have; // an ID is waiting for its data field
  bool stop; // the callback asked to stop
  size_t count;
  atari_prot_found_t f;
  uint8_t buf[ATARI_PROT_MAX_FIELD];
} atari_prot_walk_t;

MFM_MAYBE_UNUSED
static uint32_t atari_prot_clock_at(atari_prot_clock_t *c, size_t pos) {
  if (pos > c->n)
    pos = c->n;
  while (c->pos < pos)
    c->ticks += c->p[c->pos++];
  while (c->pos > pos)
    c->ticks -= c->p[--c->pos];
  return c->ticks;
}

// Deliver the pending ID (data mark 0 means no data field was found).
MFM_MAYBE_UNUSED
static void atari_prot_walk_flush(atari_prot_walk_t *w) {
  if (!w->have)
    return;
  w->have = false;
  w->count++;
  if (!w->cb(w->ctx, &w->f))
    w->stop = true;
}

// Start a pending ID from a good ID field; mark_end is the time its mark
// ended and mark_ticks how long the mark (with its sync) lasts.
MFM_MAYBE_UNUSED
static void atari_prot_walk_start(atari_prot_walk_t *w, const uint8_t *id,
                                  uint32_t mark_end, uint32_t mark_ticks) {
  w->f.track = id[0];
  w->f.side = id[1];
  w->f.number = id[2];
  w->f.size_code = id[3];
  w->f.ticks = mark_end > mark_ticks ? mark_end - mark_ticks : 0;
  w->f.data_mark = 0;
  w->f.data_crc_ok = false;
  w->f.data_len = 0;
  w->f.data = w->buf;
  w->have = true;
}

MFM_MAYBE_UNUSED
static void atari_prot_walk_data(atari_prot_walk_t *w, uint8_t mark,
                                 uint16_t crc) {
  uint16_t n = (uint16_t)(128u << w->f.size_code);
  for (uint16_t i = 0; i < n; i++)
    w->buf[i] ^= 0xff;
  w->f.data_mark = mark;
  w->f.data_len = n;
  w->f.data_crc_ok = crc == 0;
  atari_prot_walk_flush(w);
}

MFM_MAYBE_UNUSED
static void atari_prot_walk_mfm(mfm_io_t *io, atari_prot_walk_t *w,
                                uint8_t cylinder) {
  uint8_t mark, id[mfm_io_idam_size], crcb[mfm_io_crc_size];
  uint32_t sync_ticks = 48u * io->T1_nom; // three A1 bytes of 16 cells
  while (!w->stop && !mfm_io_eof(io)) {
    if (!skip_triple_sync_mark(io))
      break;
    size_t end_pos = io->pos;
    int16_t end_err = io->phase_error;
    receive_crc(io, &mark, (size_t)1, NULL);
    if (mark == MFM_IO_IDAM) {
      io->pos = end_pos;
      io->phase_error = end_err;
      uint16_t crc = receive_crc(io, &mark, (size_t)1, id, sizeof(id), crcb,
                                 sizeof(crcb), NULL);
      atari_prot_walk_flush(w);
      if (w->stop || crc != 0 || id[3] > 3 || id[0] != cylinder)
        continue;
      atari_prot_walk_start(w, id, atari_prot_clock_at(&w->clock, end_pos),
                            sync_ticks);
    } else if ((mark == MFM_IO_DAM || mark == 0xf8) && w->have) {
      uint16_t n = (uint16_t)(128u << w->f.size_code);
      io->pos = end_pos;
      io->phase_error = end_err;
      uint16_t crc = receive_crc(io, &mark, (size_t)1, w->buf, (size_t)n, crcb,
                                 sizeof(crcb), NULL);
      atari_prot_walk_data(w, mark, crc);
    }
  }
  atari_prot_walk_flush(w);
}

MFM_MAYBE_UNUSED
static void atari_prot_walk_fm(mfm_io_t *io, atari_prot_walk_t *w,
                               uint8_t cylinder) {
  uint8_t id[mfm_io_idam_size + mfm_io_crc_size], crcb[mfm_io_crc_size];
  uint32_t cells = 0;
  unsigned run = 0;
  while (!w->stop) {
    int cell = mfm_io_fm_read_cell(io, &run);
    if (cell < 0)
      break;
    cells = (cells << 1) | (uint32_t)cell;
    uint8_t mark;
    switch (cells & 0xffff) {
    case mfm_io_fm_idam_cells:
      mark = MFM_IO_IDAM;
      break;
    case mfm_io_fm_dam_cells:
      mark = MFM_IO_DAM;
      break;
    case mfm_io_fm_ddam_cells:
      mark = 0xf8;
      break;
    default:
      continue;
    }
    cells = 0;
    uint32_t mark_end = atari_prot_clock_at(&w->clock, io->pos);
    uint32_t back = run * io->T1_nom; // cells of this interval not consumed
    mark_end = mark_end > back ? mark_end - back : 0;
    uint16_t crc = mfm_io_crc16(&mark, 1, 0xffff);
    if (mark == MFM_IO_IDAM) {
      atari_prot_walk_flush(w);
      if (w->stop || !mfm_io_fm_read_bytes(io, &run, id, sizeof(id), &crc))
        break;
      if (crc != 0 || id[3] > 3 || id[0] != cylinder)
        continue;
      atari_prot_walk_start(w, id, mark_end, 16u * io->T1_nom);
    } else if (w->have) {
      uint16_t n = (uint16_t)(128u << w->f.size_code);
      if (!mfm_io_fm_read_bytes(io, &run, w->buf, n, &crc) ||
          !mfm_io_fm_read_bytes(io, &run, crcb, sizeof(crcb), &crc))
        break;
      atari_prot_walk_data(w, mark, crc);
    }
  }
  atari_prot_walk_flush(w);
}
/// @endcond

/*!
 * @brief Walk one captured track and report every sector ID found.
 * Unlike atari_decode_track() this reports duplicate IDs, sectors whose data
 * CRC is bad, deleted data marks, long sectors and IDs with no data field,
 * all in flux order. An ID with a bad CRC, a size code above 3 or a cylinder
 * other than the one asked for is skipped (an Atari drive cannot see it
 * either); the side byte is not checked. A data field is "missing" when the
 * next ID mark (or the end of the flux) comes before any data mark; that next
 * ID is still reported. Sector data is returned un-inverted (SIO form).
 * @param density ATARI_DENSITY_SD (FM), _ED or _DD (MFM).
 * @param cylinder Cylinder the IDs must carry.
 * @param pulses Captured flux intervals in ticks.
 * @param n Number of intervals.
 * @param cell_ticks From atari_track_cell_ticks().
 * @param cb Called for each sector found.
 * @param ctx Passed to cb.
 * @return Number of sectors reported, 0 for a bad argument.
 */
MFM_MAYBE_UNUSED
static size_t atari_prot_walk(atari_density_t density, uint8_t cylinder,
                              const uint8_t *pulses, size_t n,
                              uint16_t cell_ticks, atari_prot_found_cb cb,
                              void *ctx) {
  const mfm_io_settings_t *settings = atari_track_settings(density);
  if (!settings || !pulses || !cb || !cell_ticks)
    return 0;
  atari_prot_walk_t w;
  memset(&w, 0, sizeof(w));
  w.cb = cb;
  w.ctx = ctx;
  w.clock.p = pulses;
  w.clock.n = n;
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.settings = settings;
  io.T1_nom = cell_ticks;
  io.T2_max = (uint16_t)((5u * cell_ticks + 1u) / 2u); // 2.5 cells
  io.T3_max = (uint16_t)((7u * cell_ticks + 1u) / 2u); // 3.5 cells
  io.pulses = (uint8_t *)pulses;
  io.n_pulses = n;
  if (settings->is_fm)
    atari_prot_walk_fm(&io, &w, cylinder);
  else
    atari_prot_walk_mfm(&io, &w, cylinder);
  return w.count;
}

/// @cond false
typedef struct {
  atari_prot_track_t *t;
  uint32_t index_ticks;
  uint32_t rev_ticks;
  uint8_t size_code; // the density's own size code
  unsigned dropped;
} atari_prot_scan_ctx_t;

MFM_MAYBE_UNUSED
static bool atari_prot_scan_cb(void *vctx, const atari_prot_found_t *f) {
  atari_prot_scan_ctx_t *c = (atari_prot_scan_ctx_t *)vctx;
  atari_prot_track_t *t = c->t;
  if (f->ticks < c->index_ticks)
    return true;
  uint32_t rel = f->ticks - c->index_ticks;
  if (rel >= c->rev_ticks)
    return true; // the repeat of the first revolution
  uint16_t sb = atari_prot_sector_bytes(t->density);
  bool has_data = f->data_mark != 0;
  if (t->sector_count >= ATARI_PROT_MAX_SECTORS ||
      (has_data && (uint32_t)t->data_bytes + sb > ATARI_PROT_MAX_DATA)) {
    c->dropped++;
    return true;
  }
  atari_prot_sector_t *s = &t->sectors[t->sector_count++];
  s->number = f->number;
  s->size_code = f->size_code;
  s->position =
      (uint16_t)(((uint64_t)rel * ATARI_ATX_REV_UNITS) / c->rev_ticks);
  s->weak_offset = ATARI_PROT_NONE;
  s->data_offset = ATARI_PROT_NONE;
  s->status = 0;
  if (!has_data) {
    s->status = ATARI_ATX_MISSING;
    return true;
  }
  if (f->data_mark == 0xf8)
    s->status |= ATARI_ATX_DELETED;
  if (!f->data_crc_ok)
    s->status |= ATARI_ATX_CRC_ERROR;
  if (f->size_code > c->size_code)
    s->status |= ATARI_ATX_LOST_DATA;
  uint16_t keep = f->data_len < sb ? f->data_len : sb;
  s->data_offset = t->data_bytes;
  memcpy(&t->data[t->data_bytes], f->data, keep);
  memset(&t->data[t->data_bytes + keep], 0, (size_t)(sb - keep));
  t->data_bytes = (uint16_t)(t->data_bytes + sb);
  return true;
}
/// @endcond

/*!
 * @brief Scan one captured revolution into a protected track.
 * Every ID of the right cylinder is an entry, in position order. Status
 * bits: ATARI_ATX_CRC_ERROR (data CRC bad), ATARI_ATX_MISSING (no data
 * field, no data stored), ATARI_ATX_DELETED (data mark F8) and
 * ATARI_ATX_LOST_DATA (ID size code above the density's: a long sector, of
 * which only the first 128/256 bytes are kept, best effort). A sector with a
 * smaller size code than the density's is stored zero padded. t->track and
 * t->density say what to expect; t->side is kept; the entries and data are
 * replaced. Entries past ATARI_PROT_MAX_SECTORS or ATARI_PROT_MAX_DATA are
 * dropped and counted. weak_offset is always ATARI_PROT_NONE here; see
 * atari_prot_merge(). A sector whose ID is cut by the end of the first
 * revolution is read through the overlap if the buffer has one.
 * @param t The track to fill; set track and density first.
 * @param pulses Captured flux intervals in ticks, starting at the index.
 * @param n Number of intervals (a revolution plus overlap is fine).
 * @param cell_ticks From atari_track_cell_ticks().
 * @param rev_ticks Measured index-to-index time in ticks.
 * @param index_ticks Tick offset of the index pulse from the buffer start
 * (0 when the buffer starts at the index).
 * @param dropped If not NULL, receives the number of entries dropped for lack
 * of space.
 * @return Number of entries in t, or 0 for a bad argument or an empty track.
 */
MFM_MAYBE_UNUSED
static size_t atari_prot_scan(atari_prot_track_t *t, const uint8_t *pulses,
                              size_t n, uint16_t cell_ticks, uint32_t rev_ticks,
                              uint32_t index_ticks, unsigned *dropped) {
  if (dropped)
    *dropped = 0;
  if (!t || !rev_ticks || t->density > ATARI_DENSITY_DD)
    return 0;
  t->sector_count = 0;
  t->data_bytes = 0;
  atari_prot_scan_ctx_t c;
  c.t = t;
  c.index_ticks = index_ticks;
  c.rev_ticks = rev_ticks;
  c.size_code = t->density == ATARI_DENSITY_DD ? 1 : 0;
  c.dropped = 0;
  if (!atari_prot_walk(t->density, t->track, pulses, n, cell_ticks,
                       atari_prot_scan_cb, &c))
    return 0;
  if (dropped)
    *dropped = c.dropped;
  return t->sector_count;
}

/*!
 * @brief Angular distance between two positions, around the revolution.
 * @param a First position.
 * @param b Second position.
 * @return Distance in units, at most half a revolution.
 */
MFM_MAYBE_UNUSED
static uint16_t atari_prot_distance(uint16_t a, uint16_t b) {
  uint32_t d = a > b ? (uint32_t)(a - b) : (uint32_t)(b - a);
  if (d > ATARI_ATX_REV_UNITS / 2)
    d = ATARI_ATX_REV_UNITS - d;
  return (uint16_t)d;
}

/*!
 * @brief Merge a scan of another revolution into the first one.
 * Entries match when their numbers are equal and their positions are within
 * tol_units (circular). For a matched pair with data: if either copy read
 * with a good CRC the sector is not weak; the good copy's data is kept (or
 * switched to, clearing ATARI_ATX_CRC_ERROR and ATARI_ATX_EXTENDED), so one
 * noisy revolution cannot spoil a normal sector. Only when both copies have
 * CRC errors and differ does the first differing byte become the entry's
 * weak_offset (the earlier of it and any existing one) and the status get
 * ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED, like an ATX weak sector. The
 * first revolution's data and other status bits are otherwise kept. Entries
 * seen only in src are inserted in position order and kept (a duplicate ID
 * that a single revolution happened to miss, or one noisy misread; callers
 * wanting a vote can scan more revolutions and compare).
 * dst is left unchanged if the result would not fit.
 * @param dst Track holding earlier revolutions; updated.
 * @param src Scan of one more revolution of the same track and density.
 * @param tol_units Position tolerance, e.g. ATARI_PROT_MERGE_TOL.
 * @param added If not NULL, receives the number of entries inserted.
 * @return true on success; false for a bad argument, mismatched track or
 * density, or if the entries or data would not fit.
 */
MFM_MAYBE_UNUSED
static bool atari_prot_merge(atari_prot_track_t *dst,
                             const atari_prot_track_t *src, uint16_t tol_units,
                             size_t *added) {
  if (added)
    *added = 0;
  if (!dst || !src || dst == src || dst->density != src->density ||
      dst->track != src->track || dst->density > ATARI_DENSITY_DD)
    return false;
  uint16_t sb = atari_prot_sector_bytes(dst->density);
  int match[ATARI_PROT_MAX_SECTORS];
  uint64_t taken = 0; // dst entries already matched
  size_t unmatched = 0, new_data = 0;
  for (size_t i = 0; i < src->sector_count; i++) {
    match[i] = -1;
    const atari_prot_sector_t *s = &src->sectors[i];
    uint16_t best = 0xffff;
    for (size_t j = 0; j < dst->sector_count; j++) {
      const atari_prot_sector_t *d = &dst->sectors[j];
      if (d->number != s->number || (taken >> j) & 1)
        continue;
      uint16_t dist = atari_prot_distance(d->position, s->position);
      if (dist <= tol_units && dist < best) {
        best = dist;
        match[i] = (int)j;
      }
    }
    if (match[i] >= 0) {
      taken |= (uint64_t)1 << match[i];
    } else {
      unmatched++;
      if (s->data_offset != ATARI_PROT_NONE)
        new_data += sb;
    }
  }
  if (dst->sector_count + unmatched > ATARI_PROT_MAX_SECTORS ||
      (size_t)dst->data_bytes + new_data > ATARI_PROT_MAX_DATA)
    return false;

  for (size_t i = 0; i < src->sector_count; i++) {
    if (match[i] < 0)
      continue;
    const atari_prot_sector_t *s = &src->sectors[i];
    atari_prot_sector_t *d = &dst->sectors[match[i]];
    if (d->data_offset == ATARI_PROT_NONE || s->data_offset == ATARI_PROT_NONE)
      continue;
    bool d_good = !(d->status & ATARI_ATX_CRC_ERROR);
    bool s_good = !(s->status & ATARI_ATX_CRC_ERROR);
    uint8_t *a = &dst->data[d->data_offset];
    const uint8_t *b = &src->data[s->data_offset];
    if (d_good)
      continue; // a good read stands, whatever a noisy one says
    if (s_good) {
      // Switch to the good copy
      memcpy(a, b, sb);
      d->status &= (uint8_t)~(ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED);
      d->weak_offset = ATARI_PROT_NONE;
      continue;
    }
    // Both copies have CRC errors: weak where they first differ
    uint16_t weak = s->weak_offset;
    for (uint16_t k = 0; k < sb; k++) {
      if (a[k] != b[k]) {
        if (k < weak)
          weak = k;
        break;
      }
    }
    if (weak != ATARI_PROT_NONE) {
      if (d->weak_offset == ATARI_PROT_NONE || weak < d->weak_offset)
        d->weak_offset = weak;
      d->status |= ATARI_ATX_CRC_ERROR | ATARI_ATX_EXTENDED;
    }
  }

  size_t inserted = 0;
  for (size_t i = 0; i < src->sector_count; i++) {
    if (match[i] >= 0)
      continue;
    const atari_prot_sector_t *s = &src->sectors[i];
    atari_prot_sector_t e = *s;
    if (s->data_offset != ATARI_PROT_NONE) {
      e.data_offset = dst->data_bytes;
      memcpy(&dst->data[dst->data_bytes], &src->data[s->data_offset], sb);
      dst->data_bytes = (uint16_t)(dst->data_bytes + sb);
    }
    size_t at = dst->sector_count;
    while (at > 0 && dst->sectors[at - 1].position > e.position) {
      dst->sectors[at] = dst->sectors[at - 1];
      at--;
    }
    dst->sectors[at] = e;
    dst->sector_count++;
    inserted++;
  }
  if (added)
    *added = inserted;
  return true;
}

/*!
 * @brief Bytes an entry takes on disk when packed at the minimum gap: the
 * gap and presync before it, its ID field, gap 2 and (if it has data) the
 * data field. Multiply by 16 cells to get the cells.
 * @param density ATARI_DENSITY_SD, _ED or _DD.
 * @param size_code ID size code of the entry, 0..3.
 * @param has_data false for a missing data field.
 * @return Bytes, or 0 for a bad density.
 */
MFM_MAYBE_UNUSED
static uint32_t atari_prot_pitch_bytes(atari_density_t density,
                                       uint8_t size_code, bool has_data) {
  const mfm_io_settings_t *s = atari_track_settings(density);
  if (!s)
    return 0;
  uint32_t sync = s->is_fm ? 1 : 4; // address mark with its sync bytes
  uint32_t n = ATARI_PROT_MIN_GAP + s->gap_presync + sync + 4 + 2;
  if (has_data)
    n += s->gap_2 + s->gap_presync + sync + (128u << size_code) + 2;
  return n;
}

/// @cond false
// Cells of flux emitted so far, including zeros waiting for the next pulse.
MFM_MAYBE_UNUSED
static inline uint32_t atari_prot_cells(const mfm_io_t *io) {
  return (uint32_t)io->time + io->pulse_len;
}

MFM_MAYBE_UNUSED
static inline uint32_t atari_prot_rand(uint32_t *state) {
  *state = *state * 1664525u + 1013904223u;
  return *state >> 16;
}

// Emit `region_cells` of out-of-spec flux in the place of data bytes. The
// intervals sit on the edge between two valid cell counts (about 1.5 or 2.5
// cells for FM, 2.5 or 3.5 for MFM), one tick either way, so a data
// separator reads them as one or the other depending on noise and phase.
// Their total length matches the data they replace so later positions hold.
MFM_MAYBE_UNUSED
static void atari_prot_put_weak(mfm_io_t *io, uint32_t region_cells,
                                uint32_t *rng) {
  uint32_t cell = io->T1_nom;
  uint32_t lo, hi;
  if (io->settings->is_fm) {
    lo = cell;
    hi = (13u * cell) / 5u; // 2.6 cells
  } else {
    lo = 2u * cell;
    hi = (39u * cell) / 10u; // 3.9 cells
  }
  if (hi > 255)
    hi = 255;
  uint32_t b1 = io->settings->is_fm ? (3u * cell - 1u) / 2u : (5u * cell) / 2u;
  uint32_t b2 = io->settings->is_fm ? (5u * cell - 1u) / 2u : (7u * cell) / 2u;
  if (b2 + 1u > 255u)
    b2 = 254u;
  uint32_t min_tail = (3u * cell) / 2u;
  // Close the last good byte with an exact interval one cell into the region
  mfm_io_flux_put(io, (uint8_t)((io->pulse_len + 1u) * cell));
  io->time += io->pulse_len + region_cells;
  uint32_t remaining = (region_cells - 1u) * cell;
  io->pulse_len = 0;
  io->y = 0;
  while (remaining > 0) {
    uint32_t r = atari_prot_rand(rng);
    uint32_t v = lo + (r >> 4) % (hi - lo + 1u);
    if (r & 3u) // most intervals sit on a decision limit, the rest spread
      v = (((r >> 2) & 1u) ? b2 : b1) + ((r >> 3) & 1u);
    if (remaining <= v) {
      v = remaining;
    } else if (remaining - v < min_tail) {
      v = remaining / 2;
    }
    mfm_io_flux_put(io, (uint8_t)v);
    remaining -= v;
  }
}

// Encode an address mark: sync run (already emitted by the caller for MFM
// via mfm_io_encode_gap_and_sync) then the mark byte, starting the CRC.
MFM_MAYBE_UNUSED
static void atari_prot_put_mark(mfm_io_t *io, uint8_t mark) {
  mfm_io_crc_preload(io);
  if (io->settings->is_fm)
    mfm_io_encode_fm_sync_crc(io, mark, fm_default_sync_clk);
  else
    mfm_io_encode_byte_crc(io, mark);
}

MFM_MAYBE_UNUSED
static void atari_prot_put_bad_crc(mfm_io_t *io) {
  unsigned crc = io->crc;
  mfm_io_encode_byte(io, (uint8_t)((crc >> 8) ^ 0xff));
  mfm_io_encode_byte(io, (uint8_t)((crc & 0xff) ^ 0xff));
}
/// @endcond

/// @cond false
// Encode the entries from index `first` on, as one stream whose time 0 is the
// angle `shift` (units). Pads with gap bytes up to limit_cells.
MFM_MAYBE_UNUSED
static size_t atari_prot_encode_core(const atari_prot_track_t *t, size_t first,
                                     uint32_t shift, uint32_t limit_cells,
                                     uint8_t *pulses, size_t max_pulses,
                                     uint16_t cell_ticks, uint32_t rev_ticks,
                                     uint32_t *moved_units,
                                     uint32_t *content_ticks) {
  if (moved_units)
    *moved_units = 0;
  if (content_ticks)
    *content_ticks = 0;
  if (!t || !pulses || !cell_ticks || rev_ticks < 1000 ||
      t->sector_count > ATARI_PROT_MAX_SECTORS || t->density > ATARI_DENSITY_DD)
    return 0;
  const mfm_io_settings_t *settings = atari_track_settings(t->density);
  uint16_t sb = atari_prot_sector_bytes(t->density);
  mfm_io_t io;
  memset(&io, 0, sizeof(io));
  io.settings = settings;
  io.T1_nom = cell_ticks;
  io.T2_max = (uint16_t)((5u * cell_ticks + 1u) / 2u);
  io.T3_max = (uint16_t)((7u * cell_ticks + 1u) / 2u);
  io.pulses = pulses;
  io.n_pulses = max_pulses;
  io.flux_byte = mfm_io_flux_byte;
  io.encode_raw =
      settings->is_fm ? mfm_io_encode_raw_fm : mfm_io_encode_raw_mfm;

  uint32_t max_moved = 0, end_cells = 0;
  uint8_t field[ATARI_PROT_MAX_FIELD];
  for (size_t k = 0; k < t->sector_count; k++) {
    size_t i = (first + k) % t->sector_count;
    const atari_prot_sector_t *s = &t->sectors[i];
    bool has_data = !(s->status & ATARI_ATX_MISSING);
    uint32_t n = 128u << (s->size_code & 3);
    if (s->size_code > 3)
      return 0;
    if (has_data && ((uint32_t)s->data_offset + sb > t->data_bytes ||
                     s->data_offset == ATARI_PROT_NONE))
      return 0;
    // Wanted start of the ID mark, in cells
    uint32_t rel = s->position >= shift
                       ? s->position - shift
                       : s->position + ATARI_ATX_REV_UNITS - shift;
    uint64_t want_ticks =
        ((uint64_t)rel * rev_ticks + ATARI_ATX_REV_UNITS / 2) /
        ATARI_ATX_REV_UNITS;
    uint32_t want = (uint32_t)((want_ticks + cell_ticks / 2) / cell_ticks);
    uint32_t cur = atari_prot_cells(&io);
    uint32_t presync = settings->gap_presync;
    uint32_t min_gap = k == 0 ? ATARI_PROT_LEAD_GAP : ATARI_PROT_MIN_GAP;
    uint32_t gap = min_gap;
    if (want > cur + 16u * (min_gap + presync)) {
      gap = (want - cur - 16u * presync + 8u) / 16u;
    } else if (cur + 16u * (min_gap + presync) > want) {
      // Pushed later than wanted (rounding to a byte does not count)
      uint64_t moved = ((uint64_t)(cur + 16u * (min_gap + presync) - want) *
                        cell_ticks * ATARI_ATX_REV_UNITS) /
                       rev_ticks;
      if (moved > max_moved)
        max_moved = (uint32_t)moved;
    }
    uint32_t start = cur + 16u * (gap + presync);
    if (start > limit_cells || io.pos + 64 >= max_pulses)
      return 0;

    // ID field
    mfm_io_encode_gap_and_sync(&io, gap);
    atari_prot_put_mark(&io, MFM_IO_IDAM);
    uint8_t id[4] = {t->track, t->side, s->number, s->size_code};
    mfm_io_encode_buf_crc(&io, id, sizeof(id));
    mfm_io_encode_crc(&io);
    end_cells = atari_prot_cells(&io);
    if (end_cells > limit_cells)
      return 0;
    if (!has_data)
      continue;

    // Data field
    mfm_io_encode_gap_and_sync(&io, settings->gap_2);
    atari_prot_put_mark(&io,
                        (s->status & ATARI_ATX_DELETED) ? 0xf8 : MFM_IO_DAM);
    if (n > sizeof(field))
      return 0;
    uint32_t have = n < sb ? n : sb;
    for (uint32_t k = 0; k < n; k++)
      field[k] = (k < have ? t->data[s->data_offset + k] : 0) ^ 0xff;
    uint32_t weak = s->weak_offset;
    if (weak != ATARI_PROT_NONE && weak < n) {
      mfm_io_encode_buf_crc(&io, field, weak);
      uint32_t rng = 0x9e3779b9u ^ ((uint32_t)s->position << 8) ^ s->number;
      atari_prot_put_weak(&io, 16u * (n - weak + 2u), &rng);
    } else {
      mfm_io_encode_buf_crc(&io, field, n);
      if (s->status & ATARI_ATX_CRC_ERROR)
        atari_prot_put_bad_crc(&io);
      else
        mfm_io_encode_crc(&io);
    }
    end_cells = atari_prot_cells(&io);
    if (end_cells > limit_cells || io.pos + 64 >= max_pulses)
      return 0;
  }
  if (moved_units)
    *moved_units = max_moved;
  if (content_ticks)
    *content_ticks = end_cells * cell_ticks;
  // Fill the rest of the revolution with gap bytes
  while (atari_prot_cells(&io) + 16u <= limit_cells && io.pos + 64 < max_pulses)
    mfm_io_encode_byte(&io, settings->gap_byte);
  if (atari_prot_cells(&io) > limit_cells || io.pos + 1 >= max_pulses)
    return 0;
  return io.pos;
}
/// @endcond

/*!
 * @brief Master a protected track as flux for one revolution.
 * The result starts at the index. Each entry's ID address mark starts at its
 * position (scaled to rev_ticks) rounded to a whole byte, so positions come
 * back within about one byte (4 units MFM, 8 units FM at the nominal rate)
 * when scanned. Between entries the gap byte of the density is written
 * (MFM 4E, FM FF), every mark is preceded by its sync run (MFM 12 x 00 and
 * A1 A1 A1 with the missing clock, FM 6 x 00 and a mark with clock C7), the
 * ID carries t->track, t->side, the entry's number and size code, then gap 2
 * and the data mark (FB, F8 for ATARI_ATX_DELETED; no data mark, data or CRC
 * at all for ATARI_ATX_MISSING), the data (inverted on disk, zero padded to
 * the size code) and its CRC (deliberately wrong with ATARI_ATX_CRC_ERROR).
 * Long sectors (size code above the density's) are written in full, the
 * bytes beyond those stored being zero.
 *
 * Weak entries (weak_offset below the data length) are experimental: from
 * that byte to the end of the data field, CRC included, the flux is
 * out-of-spec. After one exact interval that closes the last good byte, it
 * is a pseudo-random run of intervals, most of them one tick either side of
 * a decision limit between two valid cell counts (about 1.5/2.5 cells for
 * FM, 2.5/3.5 for MFM) and the rest spread over the valid range, with the
 * same total length as the data they replace. A data separator reads such
 * flux differently depending on noise and phase and the CRC fails. Real
 * drives have not been tried. A decoder's phase correction can make the
 * first few bytes of the region read the same every time, so a merged
 * weak_offset comes out a few bytes (usually under 10) after the encoded
 * one. Every interval stays at most 255 ticks.
 *
 * Angles are only meaningful relative to each other (Atari drives never
 * locate sectors by the index), so when the entries do not fit as laid out
 * from the index, e.g. one crosses it, the whole track is shifted: the flux is
 * one stream that starts in the middle of the largest gap between the
 * entries (assumed in position order) and is written from the index as is,
 * with the usual margin at the end, so both the index and the write splice
 * fall inside that gap. Every entry then lands at (position - shift) mod
 * ATARI_ATX_REV_UNITS, with shift reported in shift_units.
 *
 * An entry whose ID would start before the previous field ends plus
 * ATARI_PROT_MIN_GAP bytes (or too early for its sync run after the index)
 * is pushed later. The result is padded with gap bytes up to 1 -
 * 1/ATARI_TRACK_MARGIN_DIV of the revolution so it can be written with the
 * index as in atari_encode_track().
 * @param t The track; entries should be in position order.
 * @param pulses Receives flux intervals in ticks.
 * @param max_pulses Capacity of pulses (a full MFM track needs about
 * ATARI_TRACK_MAX_PULSES).
 * @param cell_ticks From atari_track_cell_ticks().
 * @param rev_ticks Revolution time in ticks of the drive that will write it.
 * @param moved_units If not NULL, receives the largest distance, in units,
 * any entry was pushed later than its position.
 * @param content_ticks If not NULL, receives the ticks up to the end of the
 * last entry.
 * @param shift_units If not NULL, receives the angular shift applied to every
 * entry (0 when the plain layout from the index fits).
 * @return Number of pulses, or 0 for a bad argument (including an entry with
 * data that t does not hold), a buffer that is too small, or entries that do
 * not fit in one revolution.
 */
MFM_MAYBE_UNUSED
static size_t atari_prot_encode(const atari_prot_track_t *t, uint8_t *pulses,
                                size_t max_pulses, uint16_t cell_ticks,
                                uint32_t rev_ticks, uint32_t *moved_units,
                                uint32_t *content_ticks,
                                uint16_t *shift_units) {
  if (moved_units)
    *moved_units = 0;
  if (content_ticks)
    *content_ticks = 0;
  if (shift_units)
    *shift_units = 0;
  if (!t || !pulses || !cell_ticks || rev_ticks < 1000 ||
      t->sector_count > ATARI_PROT_MAX_SECTORS || t->density > ATARI_DENSITY_DD)
    return 0;
  uint32_t limit =
      (rev_ticks - rev_ticks / ATARI_TRACK_MARGIN_DIV) / cell_ticks;
  size_t n =
      atari_prot_encode_core(t, 0, 0, limit, pulses, max_pulses, cell_ticks,
                             rev_ticks, moved_units, content_ticks);
  if (n || t->sector_count == 0)
    return n;

  // Something crosses the index: start the stream in the largest gap
  const int32_t rev_units = (int32_t)ATARI_ATX_REV_UNITS;
  int32_t best = -1, best_end = 0;
  size_t best_i = 0;
  uint32_t presync = atari_track_settings(t->density)->gap_presync;
  for (size_t i = 0; i < t->sector_count; i++) {
    size_t j = (i + 1) % t->sector_count;
    const atari_prot_sector_t *a = &t->sectors[i], *b = &t->sectors[j];
    if (a->size_code > 3 || b->size_code > 3)
      return 0;
    uint32_t pre = ATARI_PROT_MIN_GAP + presync;
    uint32_t pa = atari_prot_pitch_bytes(t->density, a->size_code,
                                         !(a->status & ATARI_ATX_MISSING));
    int32_t end_a = (int32_t)a->position +
                    (int32_t)((uint64_t)(pa - pre) * 16u * cell_ticks *
                              ATARI_ATX_REV_UNITS / rev_ticks);
    int32_t start_b =
        (int32_t)b->position - (int32_t)((uint64_t)pre * 16u * cell_ticks *
                                         ATARI_ATX_REV_UNITS / rev_ticks);
    int32_t gap = start_b - end_a;
    if (j <= i)
      gap += rev_units; // wraps past the index
    if (gap > best) {
      best = gap;
      best_i = j;
      best_end = end_a;
    }
  }
  if (best <= 0)
    return 0;
  int32_t shift = (best_end + best / 2) % rev_units;
  if (shift < 0)
    shift += rev_units;
  n = atari_prot_encode_core(t, best_i, (uint32_t)shift, limit, pulses,
                             max_pulses, cell_ticks, rev_ticks, moved_units,
                             content_ticks);
  if (n && shift_units)
    *shift_units = (uint16_t)shift;
  return n;
}

#endif // ATARI_PROT_TRACK_H
