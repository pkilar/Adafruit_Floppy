// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariATR.h
 *
 * Atari ATR / XFD disk image implementing Adafruit_AtariDisk over an
 * Adafruit_AtariFile. Plain C++ with no Arduino or SdFat dependency.
 *
 * Supported layouts (AtariSIO tools/AtrImage.cpp, AtrMemoryImage.cpp):
 *  - 128-byte sectors: sector N at (N-1)*128.
 *  - 256-byte sectors, "short" (standard): sectors 1-3 are 128 bytes, sector
 *    N >= 4 at 384 + (N-4)*256.
 *  - 512-byte sectors (SpartaDOS X): every sector, boot sectors included, is
 *    512 bytes at (N-1)*512.
 *  - 256-byte sectors, "full": every sector, including 1-3, occupies a
 *    256-byte slot at (N-1)*256 (only 128 bytes used for N <= 3).
 * The ATR layouts are preceded by a 16-byte header; XFD files have none.
 */
#ifndef ADAFRUIT_ATARIATR_H
#define ADAFRUIT_ATARIATR_H

#include "Adafruit_AtariDisk.h"
#include "Adafruit_AtariFile.h"

/*!
 * @brief An ATR or XFD image mounted from an Adafruit_AtariFile.
 */
class Adafruit_AtariATR : public Adafruit_AtariDisk {
public:
  /*! @brief Create an unmounted image. */
  Adafruit_AtariATR();

  /*!
   * @brief Parse and mount an ATR or XFD file. The file must outlive the
   * mount.
   * @param file Open image file.
   * @return true if it is a valid, complete image; false leaves it unmounted.
   */
  bool mount(Adafruit_AtariFile *file);

  /*! @brief Detach from the file (does not flush). */
  void unmount();

  /*!
   * @brief Whether an image is mounted.
   * @return true when mounted.
   */
  bool ready() override;

  /*!
   * @brief Geometry of the mounted image.
   * @return The geometry.
   */
  atari_geometry_t geometry() override;

  /*!
   * @brief Whether the file is read-only or the header marks it protected
   * (SIO2PC flag byte 8 bit 5, or APE flag byte 15 bit 0).
   * @return true if protected.
   */
  bool writeProtected() override;

  /*!
   * @brief Read one logical sector.
   * @param sector Logical sector, 1-based.
   * @param buf Receives atari_sector_bytes() bytes.
   * @return ATARI_DISK_OK, ATARI_DISK_NOT_READY, ATARI_DISK_NOT_FOUND or
   * ATARI_DISK_IO_ERROR.
   */
  atari_disk_result_t readSector(uint16_t sector, uint8_t *buf) override;

  /*!
   * @brief Write one logical sector.
   * @param sector Logical sector, 1-based.
   * @param buf atari_sector_bytes() bytes of data.
   * @param verify Re-read and compare after writing.
   * @return ATARI_DISK_OK or the failure (including
   * ATARI_DISK_WRITE_PROTECTED).
   */
  atari_disk_result_t writeSector(uint16_t sector, const uint8_t *buf,
                                  bool verify) override;

  /*!
   * @brief Whether format() accepts a geometry: 128, 256 or 512-byte sectors,
   * 1-65535 sectors, tracks * sides * sectors per track equal to the count
   * (or a one-track hard-disk geometry), FM only with 128-byte sectors.
   * Formatting a different geometry resizes the image file.
   * @param g Requested geometry.
   * @return true if supported.
   */
  bool canFormat(const atari_geometry_t &g) override;

  /*!
   * @brief Rewrite the header, resize the file to the exact size for the
   * geometry, zero all sectors and sync. The result uses the standard
   * (short) layout; 512-byte images have no short layout.
   * @param g Geometry to format.
   * @return ATARI_DISK_OK or the failure.
   */
  atari_disk_result_t format(const atari_geometry_t &g) override;

  /*!
   * @brief Sync the file.
   * @return ATARI_DISK_OK or ATARI_DISK_IO_ERROR.
   */
  atari_disk_result_t flush() override;

  /*!
   * @brief Write a new blank ATR (header plus zeroed sectors) to a writable
   * file, discarding its contents. Uses the standard (short) layout.
   * @param file Writable file.
   * @param g Geometry of the new image.
   * @return true on success.
   */
  static bool create(Adafruit_AtariFile *file, const atari_geometry_t &g);

private:
  bool offsetOf(uint16_t sector, uint32_t *offset) const;
  static bool layoutBytes(const atari_geometry_t &g, uint32_t *data_bytes);

  Adafruit_AtariFile *_file;
  atari_geometry_t _geom;
  uint32_t _header;     // 16 for ATR, 0 for XFD
  bool _full;           // 256-byte slots for sectors 1-3
  bool _header_protect; // protect flag found in the header
};

#endif // ADAFRUIT_ATARIATR_H
