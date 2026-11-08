// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
#include "Adafruit_AtariFloppy.h"

#include "atari_prot_track.h"

static_assert(ATARI_FLOPPY_MAX_BYTES == ATARI_TRACK_MAX_BYTES,
              "track cache size mismatch");
static_assert(ATARI_FLOPPY_MAX_SECTORS == ATARI_TRACK_MAX_SECTORS,
              "sector count mismatch");

// Capture 1.25 revolutions; the worst case (FM of all 0xFF, 16 pulses per
// byte, 3255 bytes per revolution) needs 65.1 KB.
static uint8_t g_flux[ATARI_FLOPPY_FLUX_BYTES];
// Scratch for merging and verifying protected tracks
static atari_prot_track_t g_scan;

static const uint32_t IDLE_MS = 3000;      // motor off after this long idle
static const uint8_t CAPTURE_TRIES = 3;    // captures merged per track read
static const uint8_t DETECT_TRIES = 3;     // captures tried to detect density
static const uint32_t INDEX_WAIT_MS = 250; // longest wait for an index pulse
static const uint32_t REDETECT_MS = 3000;  // min gap between empty-drive probes

static bool sameGeometry(const atari_geometry_t &a, const atari_geometry_t &b) {
  return a.sector_size == b.sector_size && a.sector_count == b.sector_count &&
         a.tracks == b.tracks && a.sides == b.sides &&
         a.sectors_per_track == b.sectors_per_track && a.mfm == b.mfm;
}

Adafruit_AtariFloppy::Adafruit_AtariFloppy(Adafruit_Floppy *drive)
    : _drive(drive), _geom(atari_geometry_for_density(ATARI_DENSITY_SD)) {}

bool Adafruit_AtariFloppy::begin(uint16_t drive_rpm, uint8_t drive_tracks) {
  if (drive_rpm == 0 || (drive_tracks != 40 && drive_tracks != 80))
    return false;
  _rpm = drive_rpm;
  _drive_tracks = drive_tracks;
  _double_step = false;
  // Atari media is 48 tpi double-density type: even a 1.2M drive stays in
  // low-density (write current) mode. The speed difference is absorbed by the
  // measured revolution time.
  _drive->set_density(false);
  return true;
}

uint32_t Adafruit_AtariFloppy::revUs() const {
  if (_rev_ticks) {
    return (uint32_t)((uint64_t)_rev_ticks * 1000000u /
                      _drive->getSampleFrequency());
  }
  return 60000000u / _rpm;
}

uint32_t Adafruit_AtariFloppy::revTicks() const {
  if (_rev_ticks)
    return _rev_ticks;
  return (uint32_t)((uint64_t)_drive->getSampleFrequency() * 60u / _rpm);
}

uint16_t Adafruit_AtariFloppy::cellTicks(atari_density_t density) const {
  return atari_track_cell_ticks(density, _drive->getSampleFrequency(), revUs());
}

bool Adafruit_AtariFloppy::allValid() const {
  atari_geometry_t g = atari_geometry_for_density(_density);
  for (uint16_t i = 0; i < g.sectors_per_track; i++)
    if (!_valid[i])
      return false;
  return true;
}

bool Adafruit_AtariFloppy::motorOn() {
  _last_access_ms = millis();
  if (!_drive->drive_is_selected() || !_drive->motor_is_spinning()) {
    _drive->select(true);
    if (!_drive->spin_motor(true))
      return false;
  }
  _write_protected = _drive->get_write_protect();
  return true;
}

bool Adafruit_AtariFloppy::seek(uint8_t cylinder) {
  return _drive->goto_track(_double_step ? 2 * cylinder : cylinder);
}

// Capture 1.25 revolutions starting at an index pulse and, because the
// second index falling edge is recorded in the flux, refresh the measured
// revolution time from it.
size_t Adafruit_AtariFloppy::capture() {
  int32_t index_offset = -1;
  uint32_t capture_ms = 75000u / _rpm; // 1.25 * 60000 / rpm
  size_t n = _drive->capture_track(g_flux, sizeof(g_flux), &index_offset, false,
                                   capture_ms, INDEX_WAIT_MS);
  _last_pulses = n;
  if (n && index_offset > 0 && (size_t)index_offset <= n) {
    uint32_t ticks = atari_pulse_ticks(g_flux, (size_t)index_offset);
    uint32_t nominal =
        (uint32_t)((uint64_t)_drive->getSampleFrequency() * 60u / _rpm);
    // Only trust a revolution within 15% of nominal
    if (ticks > nominal - nominal / 7 && ticks < nominal + nominal / 7)
      _rev_ticks = ticks;
  }
  if (debug)
    debug->printf("atari: captured %u pulses, rev %u us\r\n", (unsigned)n,
                  (unsigned)revUs());
  return n;
}

// Media track count on an 80-track drive: 40-track media is stepped twice per
// track, so physical track 2 carries ID cylinder 1; 80-track media has
// cylinder 2 there. A 40-track drive only ever holds 40-track media. Nothing
// readable at track 2 counts as 40 (only track 0 may be formatted).
uint8_t Adafruit_AtariFloppy::detectMediaTracks(atari_density_t density) {
  if (_drive_tracks < 80)
    return 40;
  uint8_t result = 40;
  if (!_drive->goto_track(2))
    return result;
  for (uint8_t i = 0; i < DETECT_TRIES; i++) {
    size_t n = capture();
    if (!n)
      continue;
    uint16_t cell = cellTicks(density);
    size_t c1 = atari_decode_track_head(density, 1, 0, g_flux, n, cell, _data,
                                        _valid, true);
    size_t c2 = atari_decode_track_head(density, 2, 0, g_flux, n, cell, _data,
                                        _valid, true);
    if (debug)
      debug->printf("atari: track 2 has %u sectors as cylinder 1, %u as 2\r\n",
                    (unsigned)c1, (unsigned)c2);
    if (c1 || c2) {
      result = c2 > c1 ? 80 : 40;
      break;
    }
  }
  return result;
}

// A second side exists if head 1 of track 0 holds sectors of the same layout
// with cylinder 0. The head is left on side 0.
bool Adafruit_AtariFloppy::detectSecondSide(atari_density_t density) {
  bool found = false;
  if (!_drive->goto_track(0) || !_drive->side(1)) {
    _drive->side(0);
    return false;
  }
  for (uint8_t i = 0; i < DETECT_TRIES && !found; i++) {
    size_t n = capture();
    if (!n)
      continue;
    found =
        atari_decode_track_head(density, 0, 1, g_flux, n, cellTicks(density),
                                _data, _valid, true) > 0;
  }
  _drive->side(0);
  return found;
}

bool Adafruit_AtariFloppy::insert() {
  _wanted = true;
  _stale = false;
  _ready = false;
  _cache_cyl = NO_TRACK;
  _rev_ticks = 0;
  _double_step = false;
  bool spinning = motorOn() && _drive->side(0) && seek(0);
  for (uint8_t i = 0; spinning && i < DETECT_TRIES && !_ready; i++) {
    size_t n = capture();
    if (!n)
      continue;
    size_t found = 0;
    atari_density_t d =
        atari_detect_density(g_flux, n, 0, _drive->getSampleFrequency(),
                             revUs(), _data, _valid, &found);
    if (debug)
      debug->printf("atari: density %d, %u sectors on track 0\r\n", (int)d,
                    (unsigned)found);
    if (d != ATARI_DENSITY_OTHER) {
      uint8_t sides = detectSecondSide(d) ? 2 : 1;
      uint8_t tracks = detectMediaTracks(d);
      _density = d;
      _geom = atari_geometry_make(d, tracks, sides);
      _double_step = _drive_tracks == 80 && tracks == 40;
      _ready = true;
      if (debug)
        debug->printf("atari: %u tracks, %u sides\r\n", tracks, sides);
    }
  }
  _last_detect_ms = millis();
  if (!_ready) {
    _drive->spin_motor(false);
    _drive->select(false);
  }
  return _ready;
}

void Adafruit_AtariFloppy::eject() {
  _ready = false;
  _stale = false;
  _wanted = false;
  _cache_cyl = NO_TRACK;
  _drive->spin_motor(false);
  _drive->select(false);
}

void Adafruit_AtariFloppy::idle() {
  if (_drive->motor_is_spinning() && millis() - _last_access_ms > IDLE_MS) {
    _drive->spin_motor(false);
    _drive->select(false); // also turns off the drive's activity LED
    // The disk may be swapped while idle (no disk-change line): forget the
    // cache and detect again on the next access.
    _cache_cyl = NO_TRACK;
    _stale = _ready;
  }
}

// Redo density detection if the disk may have changed since idle() stopped
// the motor.
void Adafruit_AtariFloppy::detectIfStale() {
  if (_stale)
    insert();
}

// Like detectIfStale(), but also probe an empty drive again (rate-limited, so
// Atari retries against an empty drive fail fast). Only sector reads and
// writes do this; formatting a blank disk must not keep re-probing it.
//
// Returns false if detection changed the geometry the caller already sized its
// transfer for: the SIO engine takes the length from geometry() (single
// density while not ready) before the command. That command must then fail,
// and the Atari's retry uses the new geometry.
bool Adafruit_AtariFloppy::detectIfNeeded() {
  if (!_stale &&
      !(!_ready && _wanted && millis() - _last_detect_ms >= REDETECT_MS))
    return true;
  atari_geometry_t before =
      _ready ? _geom : atari_geometry_for_density(ATARI_DENSITY_SD);
  insert();
  return !_ready || sameGeometry(_geom, before);
}

// ready(), geometry() and writeProtected() never touch the drive: the SIO
// engine calls them before it sends ACK.
bool Adafruit_AtariFloppy::ready() { return _ready; }

atari_geometry_t Adafruit_AtariFloppy::geometry() { return _geom; }

bool Adafruit_AtariFloppy::writeProtected() { return _write_protected; }

// Read the cylinder into the cache, merging up to CAPTURE_TRIES captures.
// Sectors that never decode stay flagged invalid in _valid.
bool Adafruit_AtariFloppy::loadTrack(uint8_t cylinder, uint8_t head) {
  if (_cache_cyl == cylinder && _cache_head == head)
    return true;
  _cache_cyl = NO_TRACK;
  if (!motorOn() || !seek(cylinder) || !_drive->side(head))
    return false;
  atari_geometry_t g = atari_geometry_for_density(_density);
  memset(_valid, 0, g.sectors_per_track);
  bool captured = false;
  for (uint8_t i = 0; i < CAPTURE_TRIES && !allValid(); i++) {
    size_t n = capture();
    if (!n)
      continue;
    captured = true;
    atari_decode_track_head(_density, cylinder, head, g_flux, n,
                            cellTicks(_density), _data, _valid, false);
  }
  if (captured) {
    _cache_cyl = cylinder;
    _cache_head = head;
  }
  return captured;
}

atari_disk_result_t Adafruit_AtariFloppy::readSector(uint16_t sector,
                                                     uint8_t *buf) {
  if (!detectIfNeeded() || !_ready)
    return ATARI_DISK_NOT_READY;
  atari_geometry_t g = _geom;
  uint8_t cylinder, head;
  uint16_t slot;
  if (!atari_sector_location(&g, sector, &cylinder, &head, &slot))
    return ATARI_DISK_NOT_FOUND;
  bool cached = _cache_cyl == cylinder && _cache_head == head;
  if (!loadTrack(cylinder, head))
    return ATARI_DISK_IO_ERROR;
  if (!_valid[slot] && cached) {
    // It failed before; this is an Atari retry, so really read again.
    _cache_cyl = NO_TRACK;
    if (!loadTrack(cylinder, head))
      return ATARI_DISK_IO_ERROR;
  }
  if (!_valid[slot])
    return ATARI_DISK_NOT_FOUND;
  memcpy(buf, _data + (size_t)slot * g.sector_size,
         atari_sector_bytes(&g, sector));
  return ATARI_DISK_OK;
}

// Encode the cached track and write it in one revolution from the index.
bool Adafruit_AtariFloppy::writeCached() {
  uint32_t rev_ticks = revTicks();
  size_t n = atari_encode_track_head(_density, _cache_cyl, _cache_head, _data,
                                     g_flux, sizeof(g_flux),
                                     cellTicks(_density), rev_ticks, nullptr);
  if (debug)
    debug->printf("atari: write track %u side %u, %u pulses\r\n", _cache_cyl,
                  _cache_head, (unsigned)n);
  return n && _drive->write_track(g_flux, n, false, true);
}

// Re-read the track into the cache and compare one sector with expect.
bool Adafruit_AtariFloppy::verifyTrack(const uint8_t *expect, size_t slot,
                                       size_t len) {
  atari_geometry_t g = _geom;
  size_t n = capture();
  if (!n)
    return false;
  atari_decode_track_head(_density, _cache_cyl, _cache_head, g_flux, n,
                          cellTicks(_density), _data, _valid, true);
  return _valid[slot] && memcmp(_data + slot * g.sector_size, expect, len) == 0;
}

atari_disk_result_t Adafruit_AtariFloppy::writeSector(uint16_t sector,
                                                      const uint8_t *buf,
                                                      bool verify) {
  // A geometry change here would mean buf has the wrong length for the disk.
  if (!detectIfNeeded() || !_ready)
    return ATARI_DISK_NOT_READY;
  if (!motorOn())
    return ATARI_DISK_IO_ERROR;
  if (_write_protected)
    return ATARI_DISK_WRITE_PROTECTED;
  atari_geometry_t g = _geom;
  uint8_t cylinder, head;
  uint16_t slot;
  if (!atari_sector_location(&g, sector, &cylinder, &head, &slot))
    return ATARI_DISK_NOT_FOUND;
  // Rewriting the track needs every other sector intact. A cached track with
  // holes is read again once before giving up.
  bool cached = _cache_cyl == cylinder && _cache_head == head;
  if (!loadTrack(cylinder, head))
    return ATARI_DISK_IO_ERROR;
  if (!allValid() && cached) {
    _cache_cyl = NO_TRACK;
    if (!loadTrack(cylinder, head))
      return ATARI_DISK_IO_ERROR;
  }
  if (!allValid())
    return ATARI_DISK_IO_ERROR;
  size_t len = atari_sector_bytes(&g, sector);
  memcpy(_data + (size_t)slot * g.sector_size, buf, len);
  if (!writeCached()) {
    _cache_cyl = NO_TRACK;
    return ATARI_DISK_IO_ERROR;
  }
  if (verify) {
    bool ok = verifyTrack(buf, slot, len);
    // The cache now holds the re-read track, which may have holes
    if (!ok || !allValid()) {
      _cache_cyl = NO_TRACK;
      if (!ok)
        return ATARI_DISK_IO_ERROR;
    }
  }
  return ATARI_DISK_OK;
}

bool Adafruit_AtariFloppy::canFormat(const atari_geometry_t &g) {
  if (atari_track_density(&g) == ATARI_DENSITY_OTHER)
    return false;
  if (g.tracks != 40 && !(g.tracks == 80 && _drive_tracks == 80))
    return false;
  if (g.sides != 1 && g.sides != 2)
    return false;
  return g.sector_count == (uint32_t)g.tracks * g.sides * g.sectors_per_track;
}

atari_disk_result_t Adafruit_AtariFloppy::format(const atari_geometry_t &g) {
  atari_disk_result_t result = ATARI_DISK_OK;
  for (uint8_t track = 0; track < g.tracks && result == ATARI_DISK_OK; track++)
    for (uint8_t head = 0; head < g.sides && result == ATARI_DISK_OK; head++)
      result = writeTrackImpl(track, head, g, nullptr, true);
  return result;
}

atari_disk_result_t Adafruit_AtariFloppy::writeTrack(uint8_t track,
                                                     uint8_t head,
                                                     const atari_geometry_t &g,
                                                     const uint8_t *sectors,
                                                     bool verify) {
  return sectors ? writeTrackImpl(track, head, g, sectors, verify)
                 : ATARI_DISK_UNSUPPORTED;
}

uint32_t Adafruit_AtariFloppy::revolutionMicros() { return revUs(); }

// Write (and optionally read back) one whole track of one side. Write
// protection and encoding failures are detected before the head starts
// writing. sectors == nullptr writes zeros. The geometry only changes once the
// last track of the last side is written.
atari_disk_result_t
Adafruit_AtariFloppy::writeTrackImpl(uint8_t track, uint8_t head,
                                     const atari_geometry_t &g,
                                     const uint8_t *sectors, bool verify) {
  detectIfStale();
  if (!canFormat(g) || track >= g.tracks || head >= g.sides)
    return ATARI_DISK_UNSUPPORTED;
  if (!motorOn())
    return ATARI_DISK_IO_ERROR;
  if (_write_protected)
    return ATARI_DISK_WRITE_PROTECTED;

  atari_density_t previous = _density;
  atari_geometry_t previous_geom = _geom;
  bool previous_step = _double_step;
  _density = atari_track_density(&g);
  _double_step = _drive_tracks == 80 && g.tracks == 40;
  size_t track_bytes = (size_t)g.sectors_per_track * g.sector_size;
  if (sectors)
    memcpy(_data, sectors, track_bytes);
  else
    memset(_data, 0, track_bytes);
  memset(_valid, 1, g.sectors_per_track);
  _cache_cyl = track;
  _cache_head = head;

  atari_disk_result_t result = ATARI_DISK_OK;
  if (!seek(track) || !_drive->side(head) || !writeCached()) {
    result = ATARI_DISK_IO_ERROR;
  } else if (verify) {
    // Read it back: every sector must decode to what was written
    memset(_valid, 0, g.sectors_per_track);
    for (uint8_t i = 0; i < CAPTURE_TRIES && !allValid(); i++) {
      size_t n = capture();
      if (n)
        atari_decode_track_head(_density, track, head, g_flux, n,
                                cellTicks(_density), _data, _valid, false);
    }
    bool ok = allValid();
    for (size_t i = 0; ok && i < track_bytes; i++)
      ok = _data[i] == (sectors ? sectors[i] : 0);
    if (!ok)
      result = ATARI_DISK_IO_ERROR;
  }
  _cache_cyl = NO_TRACK;
  if (result == ATARI_DISK_OK && track == g.tracks - 1 && head == g.sides - 1) {
    _geom = g;
    _ready = true;
    _stale = false;
  } else {
    _density = previous;
    _geom = previous_geom;
    _double_step = previous_step;
  }
  return result;
}

namespace {
struct BadSectorCtx {
  uint8_t number;
  uint8_t cylinder;
  uint16_t size;
  uint16_t copy;
  uint8_t *buf;
  bool found;
};

// Take the first bad-CRC data field of the wanted ID
bool badSectorCb(void *ctx, const atari_prot_found_t *f) {
  BadSectorCtx *c = (BadSectorCtx *)ctx;
  if (f->number != c->number || f->track != c->cylinder || !f->data_mark ||
      f->data_crc_ok || f->data_len != c->size)
    return true;
  memcpy(c->buf, f->data, c->copy);
  c->found = true;
  return false;
}
} // namespace

atari_disk_result_t Adafruit_AtariFloppy::readSectorTimed(
    uint16_t sector, uint8_t *buf, uint32_t now_us, atari_read_info_t *info) {
  (void)now_us;
  info->send_data = false;
  info->delay_us = 0;
  atari_disk_result_t r = readSector(sector, buf);
  if (r == ATARI_DISK_NOT_FOUND) {
    // The flux of the last capture is still in g_flux if the track is cached
    atari_geometry_t g = _geom;
    uint8_t cylinder, head;
    uint16_t slot;
    if (atari_sector_location(&g, sector, &cylinder, &head, &slot) &&
        _cache_cyl == cylinder && _cache_head == head && _last_pulses) {
      BadSectorCtx c = {(uint8_t)(slot + 1),
                        cylinder,
                        g.sector_size,
                        (uint16_t)atari_sector_bytes(&g, sector),
                        buf,
                        false};
      atari_prot_walk(_density, cylinder, g_flux, _last_pulses,
                      cellTicks(_density), badSectorCb, &c);
      if (c.found) {
        info->fdc_status = ATARI_FDC_CRC_ERROR;
        info->send_data = true;
        return ATARI_DISK_CRC_ERROR;
      }
    }
  }
  info->fdc_status = atari_fdc_status_for(r);
  return r;
}

atari_disk_result_t
Adafruit_AtariFloppy::readProtectedTrack(uint8_t track, atari_prot_track_t *out,
                                         uint8_t revolutions,
                                         unsigned *overflow) {
  if (overflow)
    *overflow = 0;
  detectIfStale();
  if (!_ready)
    return ATARI_DISK_NOT_READY;
  if (!out || track >= _geom.tracks)
    return ATARI_DISK_UNSUPPORTED;
  if (revolutions < 1)
    revolutions = 1;
  if (revolutions > 8)
    revolutions = 8;
  _cache_cyl = NO_TRACK;
  if (!motorOn() || !seek(track) || !_drive->side(0))
    return ATARI_DISK_IO_ERROR;
  uint8_t got = 0;
  for (uint8_t i = 0; i < revolutions; i++) {
    size_t n = capture();
    if (!n)
      continue;
    atari_prot_track_t *dst = got ? &g_scan : out;
    dst->track = track;
    dst->side = 0;
    dst->density = _density;
    unsigned dropped = 0;
    atari_prot_scan(dst, g_flux, n, cellTicks(_density), revTicks(), 0,
                    &dropped);
    if (overflow && !got)
      *overflow += dropped;
    if (got && !atari_prot_merge(out, &g_scan, ATARI_PROT_MERGE_TOL, nullptr) &&
        overflow)
      *overflow += 1;
    got++;
  }
  _cache_cyl = NO_TRACK;
  return got ? ATARI_DISK_OK : ATARI_DISK_IO_ERROR;
}

// Whether the track just written reads back as t. The encoder may have
// shifted the whole track (so the index falls in a gap) and pushed crowded
// entries later; positions are compared after the shift with that much extra
// tolerance.
bool Adafruit_AtariFloppy::verifyProtected(const atari_prot_track_t *t,
                                           uint16_t shift, uint32_t moved) {
  size_t n = capture();
  if (!n)
    return false;
  g_scan.track = t->track;
  g_scan.side = 0;
  g_scan.density = t->density;
  atari_prot_scan(&g_scan, g_flux, n, cellTicks(t->density), revTicks(), 0,
                  nullptr);
  uint16_t sb = atari_prot_sector_bytes(t->density);
  uint8_t natural = t->density == ATARI_DENSITY_DD ? 1 : 0;
  uint32_t tolerance = ATARI_PROT_MERGE_TOL + moved;
  bool used[ATARI_PROT_MAX_SECTORS] = {false};
  for (uint16_t i = 0; i < t->sector_count; i++) {
    const atari_prot_sector_t *a = &t->sectors[i];
    uint16_t expect = (uint16_t)((a->position + ATARI_ATX_REV_UNITS - shift) %
                                 ATARI_ATX_REV_UNITS);
    bool weak = a->weak_offset != ATARI_PROT_NONE;
    bool ok = false;
    for (uint16_t j = 0; j < g_scan.sector_count && !ok; j++) {
      const atari_prot_sector_t *b = &g_scan.sectors[j];
      if (used[j] || b->number != a->number ||
          atari_prot_distance(expect, b->position) > tolerance)
        continue;
      if (!weak) {
        uint8_t mask =
            ATARI_ATX_CRC_ERROR | ATARI_ATX_MISSING | ATARI_ATX_DELETED;
        if (a->size_code > natural || b->size_code > natural)
          mask |= ATARI_ATX_LOST_DATA;
        if (((a->status ^ b->status) & mask) || a->size_code != b->size_code ||
            (a->data_offset == ATARI_PROT_NONE) !=
                (b->data_offset == ATARI_PROT_NONE))
          continue;
        if (a->data_offset != ATARI_PROT_NONE &&
            memcmp(&t->data[a->data_offset], &g_scan.data[b->data_offset],
                   sb) != 0)
          continue;
      }
      used[j] = true;
      ok = true;
    }
    if (!ok)
      return false;
  }
  return true;
}

atari_disk_result_t
Adafruit_AtariFloppy::writeProtectedTrack(const atari_prot_track_t *t,
                                          bool verify, uint32_t *moved_units) {
  if (moved_units)
    *moved_units = 0;
  detectIfStale();
  if (!t || t->density > ATARI_DENSITY_DD || t->side != 0 || t->track >= 40)
    return ATARI_DISK_UNSUPPORTED;
  if (!motorOn())
    return ATARI_DISK_IO_ERROR;
  if (_write_protected)
    return ATARI_DISK_WRITE_PROTECTED;

  atari_density_t previous = _density;
  atari_geometry_t previous_geom = _geom;
  bool previous_step = _double_step;
  _density = t->density;
  _double_step = _drive_tracks == 80;
  _cache_cyl = NO_TRACK;

  atari_disk_result_t result = ATARI_DISK_OK;
  if (!seek(t->track) || !_drive->side(0)) {
    result = ATARI_DISK_IO_ERROR;
  } else if (t->sector_count == 0) {
    // An unformatted track
    if (!_drive->erase_track(revUs() + revUs() / 10))
      result = ATARI_DISK_IO_ERROR;
  } else {
    if (!_rev_ticks)
      capture(); // measure the revolution before encoding
    uint32_t moved = 0;
    uint16_t shift = 0;
    size_t n = atari_prot_encode(t, g_flux, sizeof(g_flux), cellTicks(_density),
                                 revTicks(), &moved, nullptr, &shift);
    if (debug)
      debug->printf("atari: master track %u, %u pulses, moved %u, shift %u\r\n",
                    t->track, (unsigned)n, (unsigned)moved, (unsigned)shift);
    if (moved_units)
      *moved_units = moved;
    if (!n)
      result = ATARI_DISK_UNSUPPORTED;
    else if (!_drive->write_track(g_flux, n, false, true))
      result = ATARI_DISK_IO_ERROR;
    else if (verify) {
      bool ok = false;
      for (uint8_t i = 0; i < CAPTURE_TRIES && !ok; i++)
        ok = verifyProtected(t, shift, moved);
      if (!ok)
        result = ATARI_DISK_IO_ERROR;
    }
  }
  _cache_cyl = NO_TRACK;
  if (result == ATARI_DISK_OK && t->track == 39) {
    _geom = atari_geometry_make(_density, 40, 1);
    _ready = true;
    _stale = false;
  } else {
    _density = previous;
    _geom = previous_geom;
    _double_step = previous_step;
  }
  return result;
}
