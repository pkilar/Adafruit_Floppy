// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
#include "Adafruit_AtariATR.h"
#include <string.h>

// Format references: AtariSIO tools/AtrImage.cpp (SetFormatFromATRHeader,
// CreateATRHeaderFromFormat, CalculateOffset) and AtrMemoryImage.cpp
// (ReadImageFromXfdFile).
namespace {
const uint32_t ATR_HEADER_BYTES = 16;
const uint8_t ATR_MAGIC0 = 0x96;
const uint8_t ATR_MAGIC1 = 0x02;
const uint8_t ATR_FLAG_PROTECT = 0x20;  // header byte 8 (SIO2PC/AtariSIO)
const uint8_t ATR_APE_READONLY = 0x01;  // header byte 15 (APE)
const uint8_t ATR_APE_CRC_VALID = 0x02; // header byte 15: bytes 8-11 are a CRC

} // namespace

Adafruit_AtariATR::Adafruit_AtariATR()
    : _file(0), _header(0), _full(false), _header_protect(false) {
  memset(&_geom, 0, sizeof(_geom));
}

bool Adafruit_AtariATR::mount(Adafruit_AtariFile *file) {
  unmount();
  if (!file)
    return false;
  uint32_t fsize = file->size();
  uint8_t hdr[ATR_HEADER_BYTES];
  uint32_t header = 0;
  uint32_t data = 0; // bytes of sector data
  uint16_t sector_size = 128;
  bool full = false;
  bool protect = false;

  bool atr = false;
  if (fsize >= ATR_HEADER_BYTES && file->seek(0) &&
      file->read(hdr, sizeof(hdr)) == (int)sizeof(hdr) &&
      hdr[0] == ATR_MAGIC0 && hdr[1] == ATR_MAGIC1)
    atr = true;

  if (atr) {
    header = ATR_HEADER_BYTES;
    sector_size = hdr[4] | ((uint16_t)hdr[5] << 8);
    if (sector_size != 128 && sector_size != 256 && sector_size != 512)
      return false;
    uint64_t paras = hdr[2] | ((uint32_t)hdr[3] << 8) |
                     ((uint32_t)hdr[6] << 16) | ((uint32_t)hdr[7] << 24);
    uint64_t bytes = paras << 4;
    if (bytes + header > fsize)
      return false; // truncated
    data = (uint32_t)bytes;
    // APE keeps an authentication CRC in bytes 8-11 (valid when byte 15 bit 1
    // is set), so byte 8 is only the SIO2PC protect flag without it.
    bool ape_crc = (hdr[15] & ATR_APE_CRC_VALID) != 0;
    protect = (!ape_crc && (hdr[8] & ATR_FLAG_PROTECT)) ||
              (hdr[15] & ATR_APE_READONLY);
    if (sector_size == 256) {
      if (data <= 384)
        return false;
      if ((data & 0xFF) == 128)
        full = false;
      else if ((data & 0xFF) == 0)
        full = true;
      else
        return false;
    } else if (data % sector_size != 0) {
      return false;
    }
  } else {
    // XFD: raw dump, AtariSIO rule (AtariSIO tools/AtrMemoryImage.cpp),
    // except that 720 x 256 bytes is the usual double-density XFD with full
    // 256-byte boot sectors, not 1440 single-density sectors.
    if (fsize < 384 || (fsize & 0x7F) != 0)
      return false;
    data = fsize;
    sector_size = (fsize & 0x80) ? 256 : 128;
    if (fsize == 720u * 256u) {
      sector_size = 256;
      full = true;
    }
  }

  uint32_t count;
  if (sector_size == 256) {
    count = full ? data / 256 : (data - 384) / 256 + 3;
  } else {
    count = data / sector_size; // 128 or 512, every sector full size
  }
  if (count == 0 || count > 0xFFFF)
    return false;

  _geom = atari_geometry_for_sectors(sector_size, (uint16_t)count);
  _header = header;
  _full = full;
  _header_protect = protect;
  _file = file;
  return true;
}

void Adafruit_AtariATR::unmount() {
  _file = 0;
  _header = 0;
  _full = false;
  _header_protect = false;
  memset(&_geom, 0, sizeof(_geom));
}

bool Adafruit_AtariATR::ready() { return _file != 0; }

atari_geometry_t Adafruit_AtariATR::geometry() { return _geom; }

bool Adafruit_AtariATR::writeProtected() {
  return !_file || _header_protect || !_file->isWritable();
}

bool Adafruit_AtariATR::offsetOf(uint16_t sector, uint32_t *offset) const {
  if (sector == 0 || sector > _geom.sector_count)
    return false;
  uint32_t n = sector - 1u;
  if (_geom.sector_size != 256)
    *offset = _header + n * _geom.sector_size; // 128 or 512: all sectors full
  else if (_full)
    *offset = _header + n * 256u;
  else if (sector <= 3)
    *offset = _header + n * 128u;
  else
    *offset = _header + 384u + (n - 3u) * 256u;
  return true;
}

atari_disk_result_t Adafruit_AtariATR::readSector(uint16_t sector,
                                                  uint8_t *buf) {
  if (!_file)
    return ATARI_DISK_NOT_READY;
  uint32_t off;
  if (!offsetOf(sector, &off))
    return ATARI_DISK_NOT_FOUND;
  int len = atari_sector_bytes(&_geom, sector);
  if (!_file->seek(off) || _file->read(buf, len) != len)
    return ATARI_DISK_IO_ERROR;
  return ATARI_DISK_OK;
}

atari_disk_result_t Adafruit_AtariATR::writeSector(uint16_t sector,
                                                   const uint8_t *buf,
                                                   bool verify) {
  if (!_file)
    return ATARI_DISK_NOT_READY;
  uint32_t off;
  if (!offsetOf(sector, &off))
    return ATARI_DISK_NOT_FOUND;
  if (writeProtected())
    return ATARI_DISK_WRITE_PROTECTED;
  int len = atari_sector_bytes(&_geom, sector);
  if (!_file->seek(off) || _file->write(buf, len) != len)
    return ATARI_DISK_IO_ERROR;
  if (verify) {
    uint8_t back[512];
    if (!_file->seek(off) || _file->read(back, len) != len ||
        memcmp(back, buf, len) != 0)
      return ATARI_DISK_IO_ERROR;
  }
  return ATARI_DISK_OK;
}

// Total sector-data bytes of the standard (short) layout for a geometry.
bool Adafruit_AtariATR::layoutBytes(const atari_geometry_t &g,
                                    uint32_t *data_bytes) {
  if (g.sector_count == 0 ||
      (g.sector_size != 128 && g.sector_size != 256 && g.sector_size != 512))
    return false;
  if (g.sector_size == 256) {
    if (g.sector_count < 4)
      return false;
    *data_bytes = (uint32_t)g.sector_count * 256u - 384u;
  } else {
    *data_bytes = (uint32_t)g.sector_count * g.sector_size;
  }
  return true;
}

bool Adafruit_AtariATR::canFormat(const atari_geometry_t &g) {
  if (!_file)
    return false;
  uint32_t unused;
  if (!layoutBytes(g, &unused))
    return false;
  // FM is only defined for 128-byte sectors; 128-byte MFM only as enhanced
  // density (26 sectors per track) so that PERCOM data stays unambiguous.
  if (!g.mfm && g.sector_size != 128)
    return false;
  if (g.mfm && g.sector_size == 128 && g.sectors_per_track != 26)
    return false;
  if (g.tracks == 0 || g.sides < 1 || g.sides > 2 || g.sectors_per_track == 0)
    return false;
  // A headerless XFD can only be zeroed in place, never resized.
  if (_header == 0)
    return g.sector_size == _geom.sector_size &&
           g.sector_count == _geom.sector_count;
  // Floppy layout; a one-track hard-disk style image satisfies it too.
  return (uint32_t)g.tracks * g.sides * g.sectors_per_track == g.sector_count;
}

bool Adafruit_AtariATR::create(Adafruit_AtariFile *file,
                               const atari_geometry_t &g) {
  uint32_t data;
  if (!file || !file->isWritable() || !layoutBytes(g, &data))
    return false;
  uint8_t hdr[ATR_HEADER_BYTES];
  memset(hdr, 0, sizeof(hdr));
  uint32_t paras = data >> 4;
  hdr[0] = ATR_MAGIC0;
  hdr[1] = ATR_MAGIC1;
  hdr[2] = paras & 0xFF;
  hdr[3] = (paras >> 8) & 0xFF;
  hdr[4] = g.sector_size & 0xFF;
  hdr[5] = g.sector_size >> 8;
  hdr[6] = (paras >> 16) & 0xFF;
  hdr[7] = (paras >> 24) & 0xFF;
  // Truncating to 0 then growing guarantees zeroed sector data.
  if (!file->truncate(0) || !file->truncate(ATR_HEADER_BYTES + data) ||
      !file->seek(0) || file->write(hdr, sizeof(hdr)) != (int)sizeof(hdr))
    return false;
  return file->sync();
}

atari_disk_result_t Adafruit_AtariATR::format(const atari_geometry_t &g) {
  if (!_file)
    return ATARI_DISK_NOT_READY;
  if (writeProtected())
    return ATARI_DISK_WRITE_PROTECTED;
  if (!canFormat(g))
    return ATARI_DISK_UNSUPPORTED;
  if (_header == 0) { // XFD: zero the data in place, keep it headerless
    uint8_t z[128];
    memset(z, 0, sizeof(z));
    uint32_t n = _file->size();
    bool ok = _file->seek(0);
    for (uint32_t i = 0; ok && i < n; i += sizeof(z))
      ok = _file->write(z, sizeof(z)) == (int)sizeof(z);
    if (!ok || !_file->sync()) {
      unmount();
      return ATARI_DISK_IO_ERROR;
    }
    return ATARI_DISK_OK;
  }
  if (!create(_file, g)) {
    // The file may be half-written; do not keep serving it.
    unmount();
    return ATARI_DISK_IO_ERROR;
  }
  _geom = g;
  _header = ATR_HEADER_BYTES;
  _full = false;
  _header_protect = false;
  return ATARI_DISK_OK;
}

atari_disk_result_t Adafruit_AtariATR::flush() {
  if (!_file)
    return ATARI_DISK_NOT_READY;
  return _file->sync() ? ATARI_DISK_OK : ATARI_DISK_IO_ERROR;
}
