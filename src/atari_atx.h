/*!
 * @file atari_atx.h
 *
 * Copy-protected Atari tracks: the ATX ("VAPI") image layout and the
 * in-memory track description shared by the ATX image backend
 * (Adafruit_AtariATX), the protected-disk reader and the mastering writer
 * (Adafruit_AtariFloppy). Plain C with libc only, so host_src tests can use
 * it.
 *
 * Layout sources: FujiNet lib/media/atari/diskTypeAtx.h, SDrive-MAX atx.h,
 * Altirra Hardware Reference Manual (disk drive chapters). All multi-byte
 * fields are little-endian.
 *
 * A protected track is a list of sector entries in the order they pass the
 * head. An entry carries the ID sector number, its angular position, the
 * status a drive's controller reports for it and its data. Copy protection
 * uses duplicate IDs (which copy is read depends on timing), CRC errors,
 * missing data fields, deleted data marks, weak bits (bytes that read
 * differently every time) and long sectors.
 */
#ifndef ATARI_ATX_H
#define ATARI_ATX_H

#include "atari_disk.h"
#include <stdbool.h>
#include <stdint.h>

/** @name ATX file layout */
///@{
#define ATARI_ATX_SIGNATURE "AT8X" ///< First four bytes of the file
#define ATARI_ATX_FILE_HEADER 48   ///< File header bytes
#define ATARI_ATX_RECORD_HEADER 8  ///< Record header: length u32, type u16
#define ATARI_ATX_TRACK_HEADER 32  ///< Record header + track header
#define ATARI_ATX_CHUNK_HEADER                                                 \
  8                                   ///< Chunk header: length u32, type,
                                      ///< sector index, header data u16
#define ATARI_ATX_SECTOR_ENTRY 8      ///< Sector list entry bytes
#define ATARI_ATX_RECORD_TRACK 0x0000 ///< Record type: track
#define ATARI_ATX_RECORD_HOST 0x0100  ///< Record type: host data (skipped)
#define ATARI_ATX_CHUNK_DATA 0x00     ///< Chunk: concatenated sector data
#define ATARI_ATX_CHUNK_LIST 0x01     ///< Chunk: sector list
#define ATARI_ATX_CHUNK_WEAK 0x10     ///< Chunk: weak bits from header_data
#define ATARI_ATX_CHUNK_EXTENDED                                               \
  0x11                                 ///< Chunk: long sector, header_data =
                                       ///< size code on disk
#define ATARI_ATX_TRACK_MFM 0x0002     ///< Track flag: MFM (enhanced density)
#define ATARI_ATX_TRACK_NO_SKEW 0x0100 ///< Track flag: skew unknown
#define ATARI_ATX_DENSITY_SD 0         ///< File header density: single
#define ATARI_ATX_DENSITY_ED 1         ///< File header density: enhanced
#define ATARI_ATX_DENSITY_DD 2         ///< File header density: double
///@}

/** @name ATX sector status bits (stored per sector entry). Bits 2-5 equal
 * the WD179x/279x status bits of the same name. */
///@{
#define ATARI_ATX_LOST_DATA 0x04 ///< Long sector: data overran 128 bytes
#define ATARI_ATX_CRC_ERROR 0x08 ///< Data field CRC error (also weak sectors)
#define ATARI_ATX_MISSING 0x10   ///< ID present but no data field stored
#define ATARI_ATX_DELETED 0x20   ///< Deleted data mark
#define ATARI_ATX_EXTENDED 0x40  ///< Weak or extended-size chunk follows
///@}

/** Angular units per revolution: 8 us each at 288 RPM. Positions run from
 * the index pulse. */
#define ATARI_ATX_REV_UNITS 26042u
/** Microseconds per revolution of an Atari 810/1050 (288 RPM). */
#define ATARI_ATX_REV_US 208333u

/** Most sector entries (including duplicates) one protected track holds. */
#define ATARI_PROT_MAX_SECTORS 48
/** Data bytes one protected track holds: 28 x 256, room for a double-density
 * track plus ten duplicate sectors. */
#define ATARI_PROT_MAX_DATA 7168
/** "None" value for atari_prot_sector_t data_offset and weak_offset. */
#define ATARI_PROT_NONE 0xFFFFu

/** Drive whose controller behavior a protected image is replayed as. */
typedef enum {
  ATARI_DRIVE_1050 = 0, ///< Atari 1050 (WD2793), the default
  ATARI_DRIVE_810 = 1,  ///< Atari 810 (FD1771)
} atari_drive_model_t;

/** One sector entry of a protected track. */
typedef struct atari_prot_sector {
  uint8_t number;    ///< ID field sector number (any value, duplicates allowed)
  uint8_t status;    ///< ATARI_ATX_* status bits
  uint16_t position; ///< Angular position of the sector's ID field, in
                     ///< ATARI_ATX_REV_UNITS per revolution from the index
  uint16_t data_offset; ///< Offset of the data in atari_prot_track_t data,
                        ///< or ATARI_PROT_NONE (missing data field)
  uint16_t weak_offset; ///< First byte that reads differently every time,
                        ///< or ATARI_PROT_NONE
  uint8_t size_code;    ///< ID field size code on disk (0 = 128 ... 3 = 1024).
                        ///< Larger than the density's code for a long sector.
} atari_prot_sector_t;

/*!
 * @brief One protected track. Data is stored as the SIO sees it (not
 * inverted), atari_prot_sector_bytes() bytes per entry that has data.
 */
typedef struct atari_prot_track {
  uint8_t track;           ///< Cylinder
  uint8_t side;            ///< Side (ATX images are single-sided: 0)
  atari_density_t density; ///< Track layout: ATARI_DENSITY_SD, _ED or _DD
  uint16_t sector_count;   ///< Entries used in sectors[]
  uint16_t data_bytes;     ///< Bytes used in data[]
  atari_prot_sector_t sectors[ATARI_PROT_MAX_SECTORS]; ///< In position order
  uint8_t data[ATARI_PROT_MAX_DATA];                   ///< Sector data
} atari_prot_track_t;

/*!
 * @brief Data bytes stored per sector entry for a density: 128 for SD and
 * ED, 256 for DD. (ATX keeps DD boot sectors at 256 bytes; the SIO transfer
 * of sectors 1-3 uses the first 128.)
 * @param density Track layout.
 * @return Bytes per entry.
 */
static inline uint16_t atari_prot_sector_bytes(atari_density_t density) {
  return density == ATARI_DENSITY_DD ? 256 : 128;
}

/*!
 * @brief Reset a track to empty.
 * @param t The track.
 * @param track Cylinder.
 * @param side Side.
 * @param density Track layout.
 */
static inline void atari_prot_track_init(atari_prot_track_t *t, uint8_t track,
                                         uint8_t side,
                                         atari_density_t density) {
  t->track = track;
  t->side = side;
  t->density = density;
  t->sector_count = 0;
  t->data_bytes = 0;
}

/*!
 * @brief WD status bits a drive reports after reading a sector entry
 * (Altirra Hardware Reference Manual, table of SIO status byte 1 values per
 * drive). Status byte 1 is the inverse: e.g. CRC error $F7, missing $EF,
 * 1050 deleted $DF, 810 deleted $9F, 1050 long sector $F9.
 * @param status ATARI_ATX_* bits of the entry.
 * @param model Drive being emulated.
 * @return ATARI_FDC_* bits, 0 for a clean sector.
 */
static inline uint8_t atari_prot_fdc_status(uint8_t status,
                                            atari_drive_model_t model) {
  if (status & ATARI_ATX_MISSING)
    return ATARI_FDC_NOT_FOUND;
  uint8_t fdc = status & ATARI_ATX_CRC_ERROR;
  if (status & ATARI_ATX_LOST_DATA) {
    if (model == ATARI_DRIVE_810)
      return ATARI_FDC_DRQ | ATARI_FDC_BUSY;    // stops after 129 bytes ($FC)
    fdc |= ATARI_FDC_LOST_DATA | ATARI_FDC_DRQ; // $F9 / $F1 with CRC
  }
  if (status & ATARI_ATX_DELETED)
    // The 1771's record-type field is two bits wide ($9F); the 2793 has one.
    fdc |= model == ATARI_DRIVE_810 ? 0x60 : ATARI_FDC_RECORD_TYPE;
  return fdc;
}

#endif // ATARI_ATX_H
