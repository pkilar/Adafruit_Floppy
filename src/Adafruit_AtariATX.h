// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariATX.h
 *
 * Read-only Atari ATX ("VAPI") disk images, which preserve copy protection
 * (duplicate sector IDs, CRC errors, missing data, deleted marks, weak bits,
 * long sectors, sector timing), plus a writer that creates ATX files.
 * Plain C++ with no Arduino or SdFat dependency.
 *
 * Image layout and status semantics follow the public descriptions in the
 * FujiNet and SDrive-MAX ATX readers and the Altirra Hardware Reference
 * Manual; see atari_atx.h. No code is shared with those projects.
 */
#ifndef ADAFRUIT_ATARIATX_H
#define ADAFRUIT_ATARIATX_H

#include "Adafruit_AtariDisk.h"
#include "Adafruit_AtariFile.h"
#include "atari_atx.h"

/*!
 * @brief An ATX image mounted from an Adafruit_AtariFile, served as a
 * read-only 40-track single-sided disk. Reads replay the drive mechanics
 * (seek, rotation, retries) on a virtual 288 RPM rotation so duplicate-sector
 * protections see the timing a real drive would give.
 *
 * Only one parsed track is cached, in a buffer shared by every instance, so
 * several mounted units cost one 7 KB cache in total. The file must outlive
 * the mount.
 */
class Adafruit_AtariATX : public Adafruit_AtariDisk {
public:
  /*! @brief Most tracks an ATX image can hold (cylinders 0-79). */
  enum { MAX_TRACKS = 80 };

  /*! @brief Create an unmounted image. */
  Adafruit_AtariATX();
  ~Adafruit_AtariATX() override;

  /*!
   * @brief Parse the file header and index the track records. Validates the
   * "AT8X" signature, version, density and the record chain; host records and
   * unknown record types are skipped, missing tracks are tolerated. Track
   * contents are checked lazily when a track is first read.
   * @param file Open ATX file (read access is enough).
   * @return true if it is a usable image with at least one track record;
   * false leaves it unmounted.
   */
  bool mount(Adafruit_AtariFile *file);

  /*! @brief Detach from the file. */
  void unmount();

  /*!
   * @brief Whether an image is mounted.
   * @return true when mounted.
   */
  bool mounted() const { return file_ != nullptr; }

  /*!
   * @brief Copy one parsed track (sector entries sorted by position, data not
   * inverted). Used to master an ATX onto a real floppy.
   * @param track Cylinder, 0-79.
   * @param out Receives the track (about 7.4 KB).
   * @return false if the track is absent from the image or its record is
   * malformed (offsets outside the record, too many sectors or data bytes).
   */
  bool loadTrack(uint8_t track, atari_prot_track_t *out);

  /*!
   * @brief Choose the drive whose controller and firmware timing is replayed.
   * @param model ATARI_DRIVE_1050 (default) or ATARI_DRIVE_810.
   */
  void setDriveModel(atari_drive_model_t model) { model_ = model; }

  /*!
   * @brief Seed the generator that fills weak bits (for repeatable tests).
   * @param seed Any value.
   */
  void seedRandom(uint32_t seed) {
    prng_ = seed ? seed : 0x2545F491u;
    seeded_ = true;
  }

  /*!
   * @brief Whether the most recent read found its track record malformed (as
   * opposed to simply absent), in which case every sector on it read as not
   * found.
   * @return true if that track could not be parsed.
   */
  bool lastTrackBad() const { return track_bad_; }

  /*!
   * @brief Whether an image is mounted.
   * @return true when mounted.
   */
  bool ready() override { return file_ != nullptr; }

  /*!
   * @brief Geometry from the image density: 40 x 18 x 128 (SD), 40 x 26 x 128
   * (ED) or 40 x 18 x 256 (DD).
   * @return The geometry.
   */
  atari_geometry_t geometry() override { return geom_; }

  /*!
   * @brief ATX images are read-only.
   * @return Always true.
   */
  bool writeProtected() override { return true; }

  /*!
   * @brief Read a sector using an internal clock that advances by each read's
   * delay. Prefer readSectorTimed() with the real SIO clock.
   * @param sector Logical sector, 1-based.
   * @param buf Receives atari_sector_bytes() bytes.
   * @return ATARI_DISK_OK or the failure; a failed read still fills buf.
   */
  atari_disk_result_t readSector(uint16_t sector, uint8_t *buf) override;

  /*!
   * @brief Read a sector as the emulated drive would. Models the command
   * latency, head seek and settle, the rotational wait for the nearest
   * matching sector ID after that (so duplicate IDs return different copies
   * depending on now_us), the sector transfer time and firmware retries of
   * bad sectors. Weak bytes are random on every read. A sector that is not
   * found returns the previous read's data, as a real drive does not clear
   * its buffer, after the controller's multi-revolution timeout. Do not mix
   * with readSector() on the same instance.
   * @param sector Logical sector, 1-based.
   * @param buf Receives atari_sector_bytes() bytes (also on failure).
   * @param now_us Free-running microsecond clock when the command arrived;
   * 32-bit wraparound is handled.
   * @param info Receives FDC status, send_data and the delay in microseconds.
   * @return ATARI_DISK_OK, CRC_ERROR (any bad status) or NOT_FOUND.
   */
  atari_disk_result_t readSectorTimed(uint16_t sector, uint8_t *buf,
                                      uint32_t now_us,
                                      atari_read_info_t *info) override;

  /*!
   * @brief Writes are refused.
   * @param sector Unused.
   * @param buf Unused.
   * @param verify Unused.
   * @return ATARI_DISK_WRITE_PROTECTED.
   */
  atari_disk_result_t writeSector(uint16_t sector, const uint8_t *buf,
                                  bool verify) override;

  /*!
   * @brief Formatting is not supported.
   * @param g Unused.
   * @return Always false.
   */
  bool canFormat(const atari_geometry_t &g) override;

  /*!
   * @brief Formatting is refused.
   * @param g Unused.
   * @return ATARI_DISK_WRITE_PROTECTED.
   */
  atari_disk_result_t format(const atari_geometry_t &g) override;

private:
  bool readAt(uint32_t pos, void *buf, size_t len);
  bool parseTrack(uint8_t track, atari_prot_track_t *t);
  const atari_prot_track_t *cachedTrack(uint8_t track);
  uint64_t advanceClock(uint32_t now_us);
  uint32_t random32();

  Adafruit_AtariFile *file_;
  atari_geometry_t geom_;
  atari_density_t density_;
  atari_drive_model_t model_;
  uint32_t offsets_[MAX_TRACKS]; // record offset per track, 0 = absent
  uint8_t head_track_;           // where the emulated head is
  bool clock_valid_;
  uint32_t last_now_; // previous now_us
  uint64_t clock_us_; // monotonic virtual time
  uint32_t internal_us_;
  uint32_t prng_;
  bool seeded_;            // seedRandom() called
  bool track_bad_;         // last read's track was malformed
  uint8_t last_data_[256]; // bytes of the previous read
};

/*!
 * @brief Writes an ATX file from protected tracks, for example a dump of a
 * copy-protected floppy. Layout choices: file version 1 / minimum version 1,
 * creator 0xAF10 version 1 (readers ignore them), track rate field 0, track
 * flag ATARI_ATX_TRACK_MFM only on enhanced-density tracks (SDrive-MAX
 * rejects it on 18-sector images), sector list, one data chunk, weak and
 * extended chunks, 8-byte terminator per track.
 */
class Adafruit_AtariATXWriter {
public:
  /*! @brief Create a writer with no file. */
  Adafruit_AtariATXWriter();

  /*!
   * @brief Truncate the file and write the 48-byte header.
   * @param file Writable file.
   * @param density ATARI_DENSITY_SD, _ED or _DD.
   * @return true on success.
   */
  bool begin(Adafruit_AtariFile *file, atari_density_t density);

  /*!
   * @brief Append one track record. Entries are written in the given order.
   * Entries with status ATARI_ATX_MISSING store no data; every other entry
   * must have data inside atari_prot_track_t::data. The extended status bit
   * is set for entries with a weak offset or a size code other than the
   * density's own.
   * @param t The track; its density must equal the writer's and its side be 0.
   * @return false on a bad track or write error.
   */
  bool addTrack(const atari_prot_track_t &t);

  /*!
   * @brief Patch the end offset in the header and sync the file.
   * @return true on success.
   */
  bool finish();

private:
  bool put(const void *buf, size_t len);

  Adafruit_AtariFile *file_;
  atari_density_t density_;
  uint32_t pos_;
  bool failed_;
};

#endif // ADAFRUIT_ATARIATX_H
