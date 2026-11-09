/*!
 * @file Adafruit_AtariDisk.h
 *
 * Abstract Atari disk used by the SIO drive emulator. Implementations are
 * ATR/XFD images (Adafruit_AtariATR) and physical Atari floppies read through
 * a PC drive (Adafruit_AtariFloppy).
 */
#ifndef ADAFRUIT_ATARIDISK_H
#define ADAFRUIT_ATARIDISK_H

#include "atari_disk.h"

/** Result of an Atari disk operation. */
typedef enum {
  ATARI_DISK_OK = 0,          ///< Success
  ATARI_DISK_NOT_READY,       ///< No media / image not mounted
  ATARI_DISK_WRITE_PROTECTED, ///< Media or image is write-protected
  ATARI_DISK_NOT_FOUND,       ///< Sector ID not found (or out of range)
  ATARI_DISK_CRC_ERROR,       ///< Sector found but its data failed CRC
  ATARI_DISK_IO_ERROR,        ///< Any other failure (file I/O, flux, verify)
  ATARI_DISK_UNSUPPORTED,     ///< Geometry or operation not supported
} atari_disk_result_t;

/*!
 * @brief WD279x status bits a real drive's controller would report for a
 * result.
 * @param r The result.
 * @return ATARI_FDC_* bits, not inverted; 0 for ATARI_DISK_OK.
 */
static inline uint8_t atari_fdc_status_for(atari_disk_result_t r) {
  switch (r) {
  case ATARI_DISK_OK:
    return 0;
  case ATARI_DISK_NOT_FOUND:
    return ATARI_FDC_NOT_FOUND;
  case ATARI_DISK_CRC_ERROR:
    return ATARI_FDC_CRC_ERROR;
  case ATARI_DISK_WRITE_PROTECTED:
    return ATARI_FDC_WRITE_PROTECT;
  case ATARI_DISK_NOT_READY:
    return ATARI_FDC_NOT_READY;
  default:
    return ATARI_FDC_LOST_DATA | ATARI_FDC_NOT_FOUND;
  }
}

/*!
 * @brief Details of one sector read, for media that reproduce copy
 * protection (ATX images, protected physical disks).
 */
typedef struct atari_read_info {
  /** WD279x status bits of the read, not inverted (ATARI_FDC_*; 0 = clean).
   * The SIO engine reports the inverted value in status byte 1. */
  uint8_t fdc_status;
  /** On a failed read, send buf as the data frame anyway, as a real drive
   * does for a sector with a CRC error. false sends zeros. */
  bool send_data;
  /** Microseconds after now_us before the drive would answer COMPLETE or
   * ERROR (seek, rotation, retries). 0 means answer as soon as possible. */
  uint32_t delay_us;
} atari_read_info_t;

/*!
 * @brief One Atari disk: a mounted image or a physical floppy.
 *
 * Sector buffers always hold atari_sector_bytes(geometry, sector) bytes, so
 * boot sectors 1-3 of a double-density disk are 128 bytes.
 */
class Adafruit_AtariDisk {
public:
  virtual ~Adafruit_AtariDisk() {}

  /*!
   * @brief Read one logical sector with the details a copy-protected disk
   * needs: FDC status, whether to send the data of a bad sector, and how long
   * the drive mechanics would take. The SIO engine calls this, not
   * readSector(). The default wraps readSector() with no extra delay.
   * @param sector Logical sector, 1-based.
   * @param buf Receives atari_sector_bytes() bytes.
   * @param now_us SIO clock (Adafruit_AtariSIOPort::micros()) when the
   * command was acknowledged; rotational models measure the head angle from
   * it.
   * @param info Receives the details. Always filled in.
   * @return ATARI_DISK_OK or the failure.
   */
  virtual atari_disk_result_t readSectorTimed(uint16_t sector, uint8_t *buf,
                                              uint32_t now_us,
                                              atari_read_info_t *info) {
    (void)now_us;
    atari_disk_result_t r = readSector(sector, buf);
    info->fdc_status = atari_fdc_status_for(r);
    info->send_data = false;
    info->delay_us = 0;
    return r;
  }

  /*!
   * @brief Whether media is present and its geometry is known.
   * @return true when sectors can be accessed.
   */
  virtual bool ready() = 0;

  /*!
   * @brief Current geometry. Only meaningful while ready().
   * @return The geometry.
   */
  virtual atari_geometry_t geometry() = 0;

  /*!
   * @brief Whether writes and formats are refused.
   * @return true if write-protected.
   */
  virtual bool writeProtected() = 0;

  /*!
   * @brief Read one logical sector.
   * @param sector Logical sector, 1-based.
   * @param buf Receives atari_sector_bytes() bytes.
   * @return ATARI_DISK_OK or the failure.
   */
  virtual atari_disk_result_t readSector(uint16_t sector, uint8_t *buf) = 0;

  /*!
   * @brief Write one logical sector.
   * @param sector Logical sector, 1-based.
   * @param buf atari_sector_bytes() bytes of data.
   * @param verify Read the data back after writing (SIO 'W'); 'P' passes
   * false.
   * @return ATARI_DISK_OK or the failure.
   */
  virtual atari_disk_result_t writeSector(uint16_t sector, const uint8_t *buf,
                                          bool verify) = 0;

  /*!
   * @brief Whether format() can produce a geometry (used to accept or refuse
   * an SIO PERCOM write).
   * @param g Requested geometry.
   * @return true if supported.
   */
  virtual bool canFormat(const atari_geometry_t &g) = 0;

  /*!
   * @brief Format the whole disk with a new geometry; all sectors read back as
   * zeros afterwards.
   * @param g Geometry to format, normally one of atari_geometry_for_density().
   * @return ATARI_DISK_OK or the failure.
   */
  virtual atari_disk_result_t format(const atari_geometry_t &g) = 0;

  /*!
   * @brief Commit any buffered writes. Default does nothing.
   * @return ATARI_DISK_OK or the failure.
   */
  virtual atari_disk_result_t flush() { return ATARI_DISK_OK; }
};

#endif // ADAFRUIT_ATARIDISK_H
