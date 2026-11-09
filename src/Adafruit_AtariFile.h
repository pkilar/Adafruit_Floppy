// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariFile.h
 *
 * Tiny abstract random-access file used by Adafruit_AtariATR, so the ATR code
 * has no SdFat dependency and can be tested on the host.
 */
#ifndef ADAFRUIT_ATARIFILE_H
#define ADAFRUIT_ATARIFILE_H

#include <stddef.h>
#include <stdint.h>

/*!
 * @brief Random-access file backing an Atari disk image.
 */
class Adafruit_AtariFile {
public:
  virtual ~Adafruit_AtariFile() {}

  /*!
   * @brief Move the file position.
   * @param pos Absolute byte offset from the start of the file.
   * @return true on success.
   */
  virtual bool seek(uint32_t pos) = 0;

  /*!
   * @brief Read at the current position, advancing it.
   * @param buf Receives the data.
   * @param len Maximum number of bytes to read.
   * @return Bytes read (0 at end of file), or -1 on error.
   */
  virtual int read(void *buf, size_t len) = 0;

  /*!
   * @brief Write at the current position, advancing it.
   * @param buf Data to write.
   * @param len Number of bytes to write.
   * @return Bytes written, or -1 on error.
   */
  virtual int write(const void *buf, size_t len) = 0;

  /*!
   * @brief Current file size.
   * @return Size in bytes.
   */
  virtual uint32_t size() = 0;

  /*!
   * @brief Set the file size, shrinking or growing it. Bytes added by growing
   * read as zero.
   * @param len New size in bytes.
   * @return true on success.
   */
  virtual bool truncate(uint32_t len) = 0;

  /*!
   * @brief Commit buffered writes to the medium.
   * @return true on success.
   */
  virtual bool sync() = 0;

  /*!
   * @brief Whether the file was opened for writing.
   * @return true if writes are allowed.
   */
  virtual bool isWritable() = 0;
};

#endif // ADAFRUIT_ATARIFILE_H
