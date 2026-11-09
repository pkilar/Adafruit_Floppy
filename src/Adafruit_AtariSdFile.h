// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariSdFile.h
 *
 * Adafruit_AtariFile implementation over an SdFat FsFile.
 */
#ifndef ADAFRUIT_ATARISDFILE_H
#define ADAFRUIT_ATARISDFILE_H

#include "Adafruit_AtariFile.h"
#include <SdFat.h>
#include <string.h>

/*!
 * @brief Adafruit_AtariFile over an already-opened SdFat FsFile.
 */
class Adafruit_AtariSdFile : public Adafruit_AtariFile {
public:
  /*!
   * @brief Wrap an open file. The adaptor keeps a reference; the FsFile must
   * outlive it.
   * @param file An open file (O_RDWR for a writable image).
   */
  explicit Adafruit_AtariSdFile(FsFile &file) : _file(file) {}

  /*!
   * @brief Move the file position.
   * @param pos Absolute byte offset.
   * @return true on success.
   */
  bool seek(uint32_t pos) override { return _file.seekSet(pos); }

  /*!
   * @brief Read at the current position.
   * @param buf Receives the data.
   * @param len Maximum bytes to read.
   * @return Bytes read, or -1 on error.
   */
  int read(void *buf, size_t len) override { return _file.read(buf, len); }

  /*!
   * @brief Write at the current position.
   * @param buf Data to write.
   * @param len Bytes to write.
   * @return Bytes written, or -1 on error.
   */
  int write(const void *buf, size_t len) override {
    size_t n = _file.write(buf, len);
    return (n == len) ? (int)n : -1;
  }

  /*!
   * @brief Current file size.
   * @return Size in bytes (clamped to 32 bits).
   */
  uint32_t size() override {
    uint64_t s = _file.fileSize();
    return (s > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)s;
  }

  /*!
   * @brief Shrink or grow the file; grown bytes are written as zeros because
   * SdFat's truncate cannot extend a file.
   * @param len New size in bytes.
   * @return true on success.
   */
  bool truncate(uint32_t len) override {
    uint32_t cur = size();
    if (len < cur)
      return _file.truncate(len);
    if (len == cur)
      return true;
    uint8_t zeros[512]; // one SD block per write
    memset(zeros, 0, sizeof(zeros));
    if (!_file.seekSet(cur))
      return false;
    while (cur < len) {
      uint32_t n = len - cur;
      if (n > sizeof(zeros))
        n = sizeof(zeros);
      if (_file.write(zeros, n) != n)
        return false;
      cur += n;
    }
    return true;
  }

  /*!
   * @brief Flush buffered data and the directory entry to the card.
   * @return true on success.
   */
  bool sync() override { return _file.sync(); }

  /*!
   * @brief Whether the file was opened for writing.
   * @return true if writable.
   */
  bool isWritable() override { return _file.isWritable(); }

private:
  FsFile &_file;
};

#endif // ADAFRUIT_ATARISDFILE_H
