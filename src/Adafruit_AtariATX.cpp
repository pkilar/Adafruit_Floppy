// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariATX.cpp
 *
 * ATX image reader (with rotation and drive timing emulation) and writer.
 */
#include "Adafruit_AtariATX.h"
#include <string.h>

namespace {

// Drive timing, microseconds. Request latency, step and settle times are the
// values published ATX-capable emulators use (FujiNet, SDrive-MAX); the CRC /
// transfer allowance is theirs too. Retry counts and the not-found timeout are
// approximations: firmware retries differ per ROM, and a WD controller gives
// up after five index pulses per attempt (Altirra Hardware Reference Manual).
const uint32_t REQUEST_US = 3220;
const uint32_t CRC_US = 2000;
const uint32_t STEP_810_US = 5300;
const uint32_t STEP_1050_US = 12410;
const uint32_t SETTLE_1050_US = 40000;
const uint32_t GIVE_UP_REVS = 5;
const uint32_t RNF_ATTEMPTS_810 = 4;
const uint32_t RNF_ATTEMPTS_1050 = 2;
const uint32_t BAD_RETRIES_810 = 3;
const uint32_t BAD_RETRIES_1050 = 1;

const uint8_t NO_CODE = 0xFF;

// One parsed track shared by all mounted images.
struct TrackCache {
  atari_prot_track_t track;
  const void *owner; // nullptr: empty
  uint8_t number;
};
TrackCache g_cache;

uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
void wr16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}
void wr32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

// Size code of the ID field for a density's normal sectors.
uint8_t natural_code(atari_density_t d) {
  return d == ATARI_DENSITY_DD ? 1 : 0;
}

} // namespace

Adafruit_AtariATX::Adafruit_AtariATX()
    : file_(nullptr), density_(ATARI_DENSITY_SD), model_(ATARI_DRIVE_1050),
      head_track_(0), clock_valid_(false), last_now_(0), clock_us_(0),
      internal_us_(0), prng_(0x2545F491u), seeded_(false), track_bad_(false) {
  geom_ = atari_geometry_for_density(ATARI_DENSITY_SD);
  memset(offsets_, 0, sizeof(offsets_));
  memset(last_data_, 0, sizeof(last_data_));
}

Adafruit_AtariATX::~Adafruit_AtariATX() { unmount(); }

bool Adafruit_AtariATX::readAt(uint32_t pos, void *buf, size_t len) {
  if (!file_->seek(pos))
    return false;
  return file_->read(buf, len) == (int)len;
}

bool Adafruit_AtariATX::mount(Adafruit_AtariFile *file) {
  unmount();
  if (!file)
    return false;
  uint32_t fsize = file->size();
  uint8_t h[ATARI_ATX_FILE_HEADER];
  if (fsize < sizeof(h) || !file->seek(0) ||
      file->read(h, sizeof(h)) != (int)sizeof(h))
    return false;
  if (memcmp(h, ATARI_ATX_SIGNATURE, 4) != 0)
    return false;
  // Version 1 files only; min_version is what a reader must understand.
  if (rd16(h + 4) < 1 || rd16(h + 6) != 1)
    return false;
  if (h[0x12] > ATARI_ATX_DENSITY_DD)
    return false;
  uint32_t start = rd32(h + 0x1C);
  uint32_t end = rd32(h + 0x20);
  if (start < sizeof(h) || start > fsize)
    return false;
  if (end < start || end > fsize)
    end = fsize; // trust the real size over a wrong header value

  file_ = file;
  density_ = (atari_density_t)h[0x12];
  geom_ = atari_geometry_for_density(density_);
  memset(offsets_, 0, sizeof(offsets_));
  memset(last_data_, 0, sizeof(last_data_));
  head_track_ = 0;
  clock_valid_ = false;
  internal_us_ = 0;

  int tracks = 0;
  uint32_t off = start;
  while (off + ATARI_ATX_RECORD_HEADER <= end) {
    uint8_t r[ATARI_ATX_TRACK_HEADER];
    uint32_t want =
        off + sizeof(r) <= end ? sizeof(r) : ATARI_ATX_RECORD_HEADER;
    if (!readAt(off, r, want)) {
      unmount();
      return false;
    }
    uint32_t len = rd32(r);
    if (len < ATARI_ATX_RECORD_HEADER || len > end - off) {
      unmount(); // truncated or corrupt record chain
      return false;
    }
    if (rd16(r + 4) == ATARI_ATX_RECORD_TRACK && want == sizeof(r) &&
        len >= sizeof(r)) {
      uint8_t n = r[8];
      if (n < MAX_TRACKS && offsets_[n] == 0) {
        offsets_[n] = off;
        tracks++;
      }
    }
    off += len;
  }
  if (tracks == 0) {
    unmount();
    return false;
  }
  return true;
}

void Adafruit_AtariATX::unmount() {
  if (g_cache.owner == this)
    g_cache.owner = nullptr;
  file_ = nullptr;
}

bool Adafruit_AtariATX::parseTrack(uint8_t track, atari_prot_track_t *t) {
  uint32_t off = offsets_[track];
  uint8_t h[ATARI_ATX_TRACK_HEADER];
  if (!off || !readAt(off, h, sizeof(h)))
    return false;
  uint32_t len = rd32(h);
  uint32_t fsize = file_->size();
  if (rd16(h + 4) != ATARI_ATX_RECORD_TRACK || len < sizeof(h) ||
      len > fsize - off || h[8] != track)
    return false;
  uint16_t count = rd16(h + 10);
  uint32_t hsize = rd32(h + 20);
  if (count > ATARI_PROT_MAX_SECTORS || hsize < sizeof(h) || hsize > len)
    return false;

  uint32_t start[ATARI_PROT_MAX_SECTORS];
  uint16_t weak[ATARI_PROT_MAX_SECTORS];
  uint8_t code[ATARI_PROT_MAX_SECTORS];
  uint8_t num[ATARI_PROT_MAX_SECTORS];
  uint8_t stat[ATARI_PROT_MAX_SECTORS];
  uint16_t pos[ATARI_PROT_MAX_SECTORS];
  for (int i = 0; i < ATARI_PROT_MAX_SECTORS; i++) {
    weak[i] = ATARI_PROT_NONE;
    code[i] = NO_CODE;
  }

  bool have_list = false;
  uint32_t p = hsize;
  while (p + ATARI_ATX_CHUNK_HEADER <= len) {
    uint8_t c[ATARI_ATX_CHUNK_HEADER];
    if (!readAt(off + p, c, sizeof(c)))
      return false;
    uint32_t clen = rd32(c);
    if (clen == 0)
      break; // terminator
    if (clen < sizeof(c) || clen > len - p)
      return false;
    uint8_t type = c[4];
    uint8_t idx = c[5];
    uint16_t hd = rd16(c + 6);
    if (type == ATARI_ATX_CHUNK_LIST && !have_list) {
      if (clen - sizeof(c) < (uint32_t)count * ATARI_ATX_SECTOR_ENTRY)
        return false;
      for (uint16_t i = 0; i < count; i++) {
        uint8_t e[ATARI_ATX_SECTOR_ENTRY];
        if (!readAt(off + p + sizeof(c) + i * sizeof(e), e, sizeof(e)))
          return false;
        num[i] = e[0];
        stat[i] = e[1] & 0x7F;
        pos[i] = rd16(e + 2);
        if (pos[i] >= ATARI_ATX_REV_UNITS)
          pos[i] = 0;
        start[i] = rd32(e + 4);
      }
      have_list = true;
    } else if (type == ATARI_ATX_CHUNK_WEAK && idx < count) {
      weak[idx] = hd;
    } else if (type == ATARI_ATX_CHUNK_EXTENDED && idx < count) {
      code[idx] = (uint8_t)hd;
    } // data chunks need no handling: sectors point into the record
    p += clen;
  }
  if (count && !have_list)
    return false;

  atari_prot_track_init(t, track, 0, density_);
  const uint16_t bytes = atari_prot_sector_bytes(density_);
  for (uint16_t i = 0; i < count; i++) {
    atari_prot_sector_t s;
    s.number = num[i];
    s.status = stat[i];
    s.position = pos[i];
    s.weak_offset = weak[i];
    s.size_code = code[i] == NO_CODE ? natural_code(density_) : code[i];
    if (stat[i] & ATARI_ATX_MISSING) {
      s.data_offset = ATARI_PROT_NONE;
    } else {
      // Data must lie inside this record, after the track header.
      if (start[i] < hsize || start[i] > len || len - start[i] < bytes ||
          t->data_bytes + bytes > ATARI_PROT_MAX_DATA)
        return false;
      if (!readAt(off + start[i], t->data + t->data_bytes, bytes))
        return false;
      s.data_offset = t->data_bytes;
      t->data_bytes = (uint16_t)(t->data_bytes + bytes);
    }
    // Insertion sort by position keeps duplicates in file order.
    uint16_t j = t->sector_count++;
    while (j > 0 && t->sectors[j - 1].position > s.position) {
      t->sectors[j] = t->sectors[j - 1];
      j--;
    }
    t->sectors[j] = s;
  }
  return true;
}

const atari_prot_track_t *Adafruit_AtariATX::cachedTrack(uint8_t track) {
  if (!file_ || track >= MAX_TRACKS)
    return nullptr;
  if (g_cache.owner == this && g_cache.number == track)
    return &g_cache.track;
  g_cache.owner = nullptr;
  if (!parseTrack(track, &g_cache.track))
    return nullptr;
  g_cache.owner = this;
  g_cache.number = track;
  return &g_cache.track;
}

bool Adafruit_AtariATX::loadTrack(uint8_t track, atari_prot_track_t *out) {
  const atari_prot_track_t *t = cachedTrack(track);
  if (!t || !out)
    return false;
  *out = *t;
  return true;
}

uint64_t Adafruit_AtariATX::advanceClock(uint32_t now_us) {
  if (!clock_valid_) {
    clock_us_ = now_us;
    clock_valid_ = true;
  } else {
    clock_us_ += (uint32_t)(now_us - last_now_); // modulo 2^32 delta
  }
  last_now_ = now_us;
  return clock_us_;
}

uint32_t Adafruit_AtariATX::random32() {
  prng_ ^= prng_ << 13;
  prng_ ^= prng_ >> 17;
  prng_ ^= prng_ << 5;
  return prng_;
}

atari_disk_result_t Adafruit_AtariATX::readSector(uint16_t sector,
                                                  uint8_t *buf) {
  atari_read_info_t info;
  atari_disk_result_t r = readSectorTimed(sector, buf, internal_us_, &info);
  internal_us_ += info.delay_us + 1000;
  return r;
}

atari_disk_result_t
Adafruit_AtariATX::readSectorTimed(uint16_t sector, uint8_t *buf,
                                   uint32_t now_us, atari_read_info_t *info) {
  info->send_data = false;
  info->delay_us = 0;
  if (!file_) {
    info->fdc_status = ATARI_FDC_NOT_READY;
    return ATARI_DISK_NOT_READY;
  }
  const uint16_t bytes = atari_sector_bytes(&geom_, sector);
  if (bytes == 0) {
    info->fdc_status = ATARI_FDC_NOT_FOUND;
    return ATARI_DISK_NOT_FOUND;
  }
  const bool m810 = model_ == ATARI_DRIVE_810;
  if (!seeded_) { // vary weak bytes across boots, once
    prng_ ^= now_us * 2654435761u;
    if (!prng_)
      prng_ = 0x2545F491u;
    seeded_ = true;
  }
  const uint64_t now = advanceClock(now_us);
  const uint8_t track = (uint8_t)((sector - 1) / geom_.sectors_per_track);
  const uint8_t id = (uint8_t)((sector - 1) % geom_.sectors_per_track + 1);

  // 1. Command latency, then the seek if the head has to move.
  uint32_t t = REQUEST_US;
  if (track != head_track_) {
    uint32_t steps =
        track > head_track_ ? track - head_track_ : head_track_ - track;
    t += steps * (m810 ? STEP_810_US : STEP_1050_US);
    if (!m810)
      t += SETTLE_1050_US;
    head_track_ = track;
  }

  // 2. Nearest matching sector ID ahead of the head.
  const atari_prot_track_t *tr = cachedTrack(track);
  track_bad_ = !tr && offsets_[track] != 0;
  const atari_prot_sector_t *hit = nullptr;
  if (tr) {
    uint32_t phase = (uint32_t)((now + t) % ATARI_ATX_REV_US);
    uint32_t angle =
        (uint32_t)((uint64_t)phase * ATARI_ATX_REV_UNITS / ATARI_ATX_REV_US);
    uint32_t best = 0;
    for (uint16_t i = 0; i < tr->sector_count; i++) {
      const atari_prot_sector_t &s = tr->sectors[i];
      if (s.number != id || (s.status & ATARI_ATX_MISSING))
        continue;
      uint32_t dist =
          (s.position + ATARI_ATX_REV_UNITS - angle) % ATARI_ATX_REV_UNITS;
      if (!hit || dist < best) {
        hit = &s;
        best = dist;
      }
    }
    if (hit) {
      // 4. Wait for the sector, read it, compute its CRC.
      uint32_t wait =
          (uint32_t)((uint64_t)best * ATARI_ATX_REV_US / ATARI_ATX_REV_UNITS);
      t += wait + ATARI_ATX_REV_US / geom_.sectors_per_track + CRC_US;
    }
  }

  if (!hit) {
    // 3. Record not found: the controller gives up after several index
    // pulses per attempt. The previous buffer contents go out.
    t += (m810 ? RNF_ATTEMPTS_810 : RNF_ATTEMPTS_1050) * GIVE_UP_REVS *
         ATARI_ATX_REV_US;
    memcpy(buf, last_data_, bytes);
    info->fdc_status = ATARI_FDC_NOT_FOUND;
    info->send_data = true;
    info->delay_us = t;
    return ATARI_DISK_NOT_FOUND;
  }

  const uint8_t status = hit->status;
  if (status & (ATARI_ATX_LOST_DATA | ATARI_ATX_CRC_ERROR))
    t += (m810 ? BAD_RETRIES_810 : BAD_RETRIES_1050) * ATARI_ATX_REV_US;

  // 5. Data; weak bytes are fresh for this (final) attempt.
  memcpy(buf, tr->data + hit->data_offset, bytes);
  if (hit->weak_offset != ATARI_PROT_NONE && hit->weak_offset < bytes) {
    for (uint16_t i = hit->weak_offset; i < bytes; i++)
      buf[i] = (uint8_t)(random32() >> 11);
  }
  memcpy(last_data_, buf, bytes);

  // 6. Status.
  info->fdc_status = atari_prot_fdc_status(status, model_);
  info->delay_us = t;
  if (info->fdc_status == 0)
    return ATARI_DISK_OK;
  info->send_data = true;
  return ATARI_DISK_CRC_ERROR;
}

atari_disk_result_t Adafruit_AtariATX::writeSector(uint16_t sector,
                                                   const uint8_t *buf,
                                                   bool verify) {
  (void)sector;
  (void)buf;
  (void)verify;
  return ATARI_DISK_WRITE_PROTECTED;
}

bool Adafruit_AtariATX::canFormat(const atari_geometry_t &g) {
  (void)g;
  return false;
}

atari_disk_result_t Adafruit_AtariATX::format(const atari_geometry_t &g) {
  (void)g;
  return ATARI_DISK_WRITE_PROTECTED;
}

// ---------------------------------------------------------------------------

Adafruit_AtariATXWriter::Adafruit_AtariATXWriter()
    : file_(nullptr), density_(ATARI_DENSITY_SD), pos_(0), failed_(false) {}

bool Adafruit_AtariATXWriter::put(const void *buf, size_t len) {
  if (!file_ || failed_)
    return false;
  if (file_->write(buf, len) != (int)len) {
    failed_ = true;
    return false;
  }
  pos_ += (uint32_t)len;
  return true;
}

bool Adafruit_AtariATXWriter::begin(Adafruit_AtariFile *file,
                                    atari_density_t density) {
  file_ = nullptr;
  failed_ = false;
  if (!file || !file->isWritable() || density > ATARI_DENSITY_DD)
    return false;
  if (!file->truncate(0) || !file->seek(0))
    return false;
  file_ = file;
  density_ = density;
  pos_ = 0;
  uint8_t h[ATARI_ATX_FILE_HEADER];
  memset(h, 0, sizeof(h));
  memcpy(h, ATARI_ATX_SIGNATURE, 4);
  wr16(h + 4, 1);      // version
  wr16(h + 6, 1);      // minimum version
  wr16(h + 8, 0xAF10); // creator
  wr16(h + 10, 1);     // creator version
  h[0x12] = (uint8_t)density;
  wr32(h + 0x1C, sizeof(h)); // first record
  wr32(h + 0x20, sizeof(h)); // end, patched by finish()
  return put(h, sizeof(h));
}

bool Adafruit_AtariATXWriter::addTrack(const atari_prot_track_t &t) {
  if (!file_ || failed_ || t.density != density_ || t.side != 0 ||
      t.track >= Adafruit_AtariATX::MAX_TRACKS ||
      t.sector_count > ATARI_PROT_MAX_SECTORS)
    return false;
  const uint16_t n = t.sector_count;
  const uint16_t bytes = atari_prot_sector_bytes(density_);
  uint8_t extra[ATARI_PROT_MAX_SECTORS]; // weak/extended chunk wanted
  uint32_t with_data = 0, chunks = 0;
  for (uint16_t i = 0; i < n; i++) {
    const atari_prot_sector_t &s = t.sectors[i];
    bool ext = s.size_code != natural_code(density_);
    bool wk = s.weak_offset != ATARI_PROT_NONE;
    extra[i] = (uint8_t)(ext ? 2 : 0) | (uint8_t)(wk ? 1 : 0);
    chunks += (ext ? 1 : 0) + (wk ? 1 : 0);
    if (!(s.status & ATARI_ATX_MISSING)) {
      if (s.data_offset == ATARI_PROT_NONE ||
          (uint32_t)s.data_offset + bytes > t.data_bytes ||
          t.data_bytes > ATARI_PROT_MAX_DATA)
        return false;
      with_data++;
    }
  }

  const uint32_t list_len = n ? ATARI_ATX_CHUNK_HEADER + n * 8u : 0;
  const uint32_t data_len =
      with_data ? ATARI_ATX_CHUNK_HEADER + with_data * bytes : 0;
  const uint32_t rec_len = ATARI_ATX_TRACK_HEADER + list_len + data_len +
                           chunks * ATARI_ATX_CHUNK_HEADER +
                           ATARI_ATX_CHUNK_HEADER;
  const uint32_t data_start =
      ATARI_ATX_TRACK_HEADER + list_len + ATARI_ATX_CHUNK_HEADER;

  uint8_t h[ATARI_ATX_TRACK_HEADER];
  memset(h, 0, sizeof(h));
  wr32(h, rec_len);
  wr16(h + 4, ATARI_ATX_RECORD_TRACK);
  h[8] = t.track;
  wr16(h + 10, n);
  wr32(h + 16, density_ == ATARI_DENSITY_ED ? ATARI_ATX_TRACK_MFM : 0);
  wr32(h + 20, ATARI_ATX_TRACK_HEADER);
  bool ok = put(h, sizeof(h));

  if (n && ok) {
    uint8_t c[ATARI_ATX_CHUNK_HEADER];
    memset(c, 0, sizeof(c));
    wr32(c, list_len);
    c[4] = ATARI_ATX_CHUNK_LIST;
    ok = put(c, sizeof(c));
    uint32_t k = 0; // data entries so far
    for (uint16_t i = 0; i < n && ok; i++) {
      const atari_prot_sector_t &s = t.sectors[i];
      uint8_t e[ATARI_ATX_SECTOR_ENTRY];
      bool missing = (s.status & ATARI_ATX_MISSING) != 0;
      e[0] = s.number;
      e[1] = (uint8_t)((s.status & 0x3F) | (extra[i] ? ATARI_ATX_EXTENDED : 0));
      wr16(e + 2, (uint16_t)(s.position % ATARI_ATX_REV_UNITS));
      wr32(e + 4, missing ? 0 : data_start + k++ * bytes);
      ok = put(e, sizeof(e));
    }
  }
  if (with_data && ok) {
    uint8_t c[ATARI_ATX_CHUNK_HEADER];
    memset(c, 0, sizeof(c));
    wr32(c, data_len);
    c[4] = ATARI_ATX_CHUNK_DATA;
    ok = put(c, sizeof(c));
    for (uint16_t i = 0; i < n && ok; i++) {
      const atari_prot_sector_t &s = t.sectors[i];
      if (!(s.status & ATARI_ATX_MISSING))
        ok = put(t.data + s.data_offset, bytes);
    }
  }
  for (uint16_t i = 0; i < n && ok; i++) {
    const atari_prot_sector_t &s = t.sectors[i];
    for (int w = 0; w < 2 && ok; w++) {
      uint8_t c[ATARI_ATX_CHUNK_HEADER];
      memset(c, 0, sizeof(c));
      if (w == 0 && (extra[i] & 1)) {
        c[4] = ATARI_ATX_CHUNK_WEAK;
        wr16(c + 6, s.weak_offset);
      } else if (w == 1 && (extra[i] & 2)) {
        c[4] = ATARI_ATX_CHUNK_EXTENDED;
        wr16(c + 6, s.size_code);
      } else {
        continue;
      }
      wr32(c, ATARI_ATX_CHUNK_HEADER);
      c[5] = (uint8_t)i;
      ok = put(c, sizeof(c));
    }
  }
  if (ok) {
    const uint8_t term[ATARI_ATX_CHUNK_HEADER] = {0};
    ok = put(term, sizeof(term));
  }
  return ok;
}

bool Adafruit_AtariATXWriter::finish() {
  if (!file_ || failed_)
    return false;
  uint8_t e[4];
  wr32(e, pos_);
  bool ok = file_->seek(0x20) && file_->write(e, 4) == 4 && file_->sync();
  file_ = nullptr;
  return ok;
}
