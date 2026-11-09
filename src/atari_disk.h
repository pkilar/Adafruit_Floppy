/*!
 * @file atari_disk.h
 *
 * Atari 8-bit disk geometry shared by the SIO drive emulator, ATR images and
 * physical Atari floppies. Plain C with libc only, so the host tests in
 * host_src can compile it without Arduino.
 *
 * Atari logical sectors are numbered from 1. On a floppy, sector L lives on
 * track (L - 1) / sectors_per_track with ID sector number
 * (L - 1) % sectors_per_track + 1. Double-density media keep 256-byte sectors
 * on disk, but sectors 1-3 (the boot sectors) are transferred over SIO and
 * stored in ATR images as 128 bytes.
 */
#ifndef ATARI_DISK_H
#define ATARI_DISK_H

#include <stdbool.h>
#include <stdint.h>

/** Standard Atari floppy densities. */
typedef enum {
  ATARI_DENSITY_SD = 0, ///< Single: FM, 18 x 128-byte sectors, 40 tracks (90K)
  ATARI_DENSITY_ED = 1, ///< Enhanced/medium: MFM, 26 x 128 bytes (130K)
  ATARI_DENSITY_DD = 2, ///< Double: MFM, 18 x 256 bytes (180K)
  ATARI_DENSITY_OTHER = 3, ///< Anything else (large images, DS/DD, ...)
} atari_density_t;

/** Geometry of an Atari disk or disk image. */
typedef struct atari_geometry {
  uint16_t sector_size;       ///< Bytes per sector: 128, 256 or 512
  uint16_t sector_count;      ///< Total logical sectors (SIO limit 65535)
  uint8_t tracks;             ///< Tracks per side (40, 80; 1 for hard disks)
  uint8_t sides;              ///< 1 or 2
  uint16_t sectors_per_track; ///< 18 (SD, DD) or 26 (ED); any for images
  bool mfm;                   ///< true for MFM media (ED, DD), false for FM
} atari_geometry_t;

/** @name WD179x/WD279x status bits (not inverted) after a sector read or
 * write. Atari drives report the inverted byte in SIO status byte 1. ATX
 * images store these bits per sector. */
///@{
#define ATARI_FDC_BUSY 0x01          ///< Command in progress
#define ATARI_FDC_DRQ 0x02           ///< Data request
#define ATARI_FDC_LOST_DATA 0x04     ///< Data not taken in time
#define ATARI_FDC_CRC_ERROR 0x08     ///< ID or data field CRC failed
#define ATARI_FDC_NOT_FOUND 0x10     ///< Record (sector ID) not found
#define ATARI_FDC_RECORD_TYPE 0x20   ///< Deleted data mark (279x read)
#define ATARI_FDC_WRITE_PROTECT 0x40 ///< Write-protected (write commands)
#define ATARI_FDC_NOT_READY 0x80     ///< Drive not ready
///@}

/** Tracks on a standard Atari 5.25" floppy. */
#define ATARI_FLOPPY_TRACKS 40
/** Nominal rotational speed of Atari 810/1050 drives, in RPM. */
#define ATARI_NOMINAL_RPM 288

/*!
 * @brief Geometry of a standard Atari floppy density.
 * @param density ATARI_DENSITY_SD, ATARI_DENSITY_ED or ATARI_DENSITY_DD;
 * anything else returns the single-density geometry.
 * @return The geometry.
 */
static inline atari_geometry_t
atari_geometry_for_density(atari_density_t density) {
  atari_geometry_t g;
  g.tracks = ATARI_FLOPPY_TRACKS;
  g.sides = 1;
  switch (density) {
  case ATARI_DENSITY_ED:
    g.sector_size = 128;
    g.sectors_per_track = 26;
    g.mfm = true;
    break;
  case ATARI_DENSITY_DD:
    g.sector_size = 256;
    g.sectors_per_track = 18;
    g.mfm = true;
    break;
  default:
    g.sector_size = 128;
    g.sectors_per_track = 18;
    g.mfm = false;
    break;
  }
  g.sector_count = (uint16_t)(g.tracks * g.sides * g.sectors_per_track);
  return g;
}

/*!
 * @brief Classify a geometry as one of the standard floppy densities.
 * @param g The geometry.
 * @return The matching density, or ATARI_DENSITY_OTHER.
 */
static inline atari_density_t atari_density_of(const atari_geometry_t *g) {
  for (int d = ATARI_DENSITY_SD; d <= ATARI_DENSITY_DD; d++) {
    atari_geometry_t s = atari_geometry_for_density((atari_density_t)d);
    if (g->sector_size == s.sector_size && g->sector_count == s.sector_count &&
        g->sides == s.sides && g->sectors_per_track == s.sectors_per_track &&
        g->tracks == s.tracks && g->mfm == s.mfm)
      return (atari_density_t)d;
  }
  return ATARI_DENSITY_OTHER;
}

/*!
 * @brief Classify the layout of one track: which standard density's
 * encoding, sector count and sector size it uses. Double-sided and 80-track
 * disks use the same track layouts as single-sided 40-track ones.
 * @param g The geometry.
 * @return ATARI_DENSITY_SD, _ED or _DD, or ATARI_DENSITY_OTHER.
 */
static inline atari_density_t atari_track_density(const atari_geometry_t *g) {
  for (int d = ATARI_DENSITY_SD; d <= ATARI_DENSITY_DD; d++) {
    atari_geometry_t s = atari_geometry_for_density((atari_density_t)d);
    if (g->sector_size == s.sector_size &&
        g->sectors_per_track == s.sectors_per_track && g->mfm == s.mfm)
      return (atari_density_t)d;
  }
  return ATARI_DENSITY_OTHER;
}

/*!
 * @brief Build a floppy geometry from a standard track layout.
 * @param density Track layout: ATARI_DENSITY_SD, _ED or _DD.
 * @param tracks Tracks per side (40, or 80 for "quad" density drives).
 * @param sides 1, or 2 for XF551-style double-sided disks.
 * @return The geometry.
 */
static inline atari_geometry_t
atari_geometry_make(atari_density_t density, uint8_t tracks, uint8_t sides) {
  atari_geometry_t g = atari_geometry_for_density(density);
  g.tracks = tracks;
  g.sides = sides;
  g.sector_count = (uint16_t)((uint32_t)tracks * sides * g.sectors_per_track);
  return g;
}

/*!
 * @brief Guess an image's geometry from its sector size and count, as
 * AtariSIO and SIO2SD do (Altirra Hardware Reference Manual, ATR section).
 *
 * 128 and 256-byte sectors: 720 = 40 x 1 x 18 (SD/DD), 1040 = 40 x 1 x 26
 * (ED), 1440 = 40 x 2 x 18 (XF551 DS), 2880 = 80 x 2 x 18, 5760 = 80 x 2 x 36.
 * 512-byte sectors: 720 = 80 x 1 x 9, 1440 = 80 x 2 x 9, 2880 = 80 x 2 x 18.
 * Anything else is a "hard disk": one track holding every sector.
 * @param sector_size 128, 256 or 512.
 * @param sector_count Number of sectors, 1-65535.
 * @return The geometry.
 */
static inline atari_geometry_t
atari_geometry_for_sectors(uint16_t sector_size, uint16_t sector_count) {
  atari_geometry_t g;
  g.sector_size = sector_size;
  g.sector_count = sector_count;
  g.mfm = sector_size > 128;
  g.tracks = 1;
  g.sides = 1;
  g.sectors_per_track = sector_count;
  if (sector_size == 128 && sector_count == 1040) {
    g.mfm = true; // ED is the only 128-byte MFM layout
    g.tracks = 40;
    g.sectors_per_track = 26;
    return g;
  }
  static const struct {
    uint16_t count;
    uint8_t tracks, sides;
    uint16_t spt;
  } small[] = {{720, 40, 1, 18},
               {1440, 40, 2, 18},
               {2880, 80, 2, 18},
               {5760, 80, 2, 36}},
    large[] = {{720, 80, 1, 9}, {1440, 80, 2, 9}, {2880, 80, 2, 18}};
  const bool big = sector_size >= 512;
  const unsigned n =
      big ? sizeof(large) / sizeof(large[0]) : sizeof(small) / sizeof(small[0]);
  for (unsigned i = 0; i < n; i++) {
    const uint16_t count = big ? large[i].count : small[i].count;
    if (count != sector_count)
      continue;
    g.tracks = big ? large[i].tracks : small[i].tracks;
    g.sides = big ? large[i].sides : small[i].sides;
    g.sectors_per_track = big ? large[i].spt : small[i].spt;
    break;
  }
  return g;
}

/*!
 * @brief Number of bytes transferred over SIO (and stored in an ATR) for a
 * logical sector.
 *
 * Only 256-byte-sector disks shorten sectors 1-3 to 128 bytes. With 512-byte
 * sectors every sector, boot sectors included, is full size (SpartaDOS X 4.40
 * addendum; Altirra Hardware Reference Manual).
 * @param g The geometry.
 * @param sector Logical sector, 1-based.
 * @return 128 for boot sectors 1-3 of a 256-byte-sector disk, otherwise the
 * sector size. 0 for an out-of-range sector.
 */
static inline uint16_t atari_sector_bytes(const atari_geometry_t *g,
                                          uint16_t sector) {
  if (sector == 0 || sector > g->sector_count)
    return 0;
  if (g->sector_size == 256 && sector <= 3)
    return 128;
  return g->sector_size;
}

/*!
 * @brief Physical location of a logical sector on a floppy.
 *
 * Side 0 holds the first tracks * sectors_per_track sectors in order. Side 1
 * follows the XF551 convention (Altirra Hardware Reference Manual, XF551
 * section): it is walked backwards, so the first sector after side 0 is the
 * last ID of the last track. For 40 x 18: sector 721 is track 39 ID 18,
 * sector 722 is track 39 ID 17, sector 1440 is track 0 ID 1.
 * @param g The geometry.
 * @param sector Logical sector, 1-based.
 * @param cylinder Receives the cylinder (0-based).
 * @param head Receives the side, 0 or 1.
 * @param slot Receives the 0-based position in the track's sector table; the
 * on-disk ID sector number is slot + 1.
 * @return false if the sector is out of range or the geometry is unusable.
 */
static inline bool atari_sector_location(const atari_geometry_t *g,
                                         uint16_t sector, uint8_t *cylinder,
                                         uint8_t *head, uint16_t *slot) {
  if (sector == 0 || sector > g->sector_count || g->sectors_per_track == 0 ||
      g->tracks == 0 || g->sides == 0 || g->sides > 2)
    return false;
  uint32_t index = sector - 1u;
  uint32_t per_side = (uint32_t)g->tracks * g->sectors_per_track;
  if (index < per_side) {
    *head = 0;
    *cylinder = (uint8_t)(index / g->sectors_per_track);
    *slot = (uint16_t)(index % g->sectors_per_track);
    return true;
  }
  if (g->sides != 2)
    return false;
  index -= per_side;
  *head = 1;
  *cylinder = (uint8_t)(g->tracks - 1u - index / g->sectors_per_track);
  *slot = (uint16_t)(g->sectors_per_track - 1u - index % g->sectors_per_track);
  return true;
}

#endif // ATARI_DISK_H
