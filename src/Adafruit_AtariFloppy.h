// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
/*!
 * @file Adafruit_AtariFloppy.h
 *
 * Physical Atari 8-bit floppies (SD / ED / DD; 40 or 80 tracks; one or two
 * sides, including the XF551 double-sided layout) read
 * and written through a PC/Shugart drive on an Adafruit_Floppy, exposed as an
 * Adafruit_AtariDisk. Track layout and timing live in atari_track.h.
 *
 * RAM: one static flux buffer of ATARI_FLOPPY_FLUX_BYTES (66000) bytes is
 * shared by all instances (use a single drive), plus a static scratch
 * atari_prot_track_t (about 7.7 KB) for readProtectedTrack() and its verify,
 * plus about 4.6 KB of track cache inside each object. Put the object at global
 * scope.
 */
#ifndef ADAFRUIT_ATARIFLOPPY_H
#define ADAFRUIT_ATARIFLOPPY_H

#include <Adafruit_Floppy.h>

#include "Adafruit_AtariDisk.h"
#include "atari_atx.h"

/** Sectors on the largest Atari track (ED). */
#define ATARI_FLOPPY_MAX_SECTORS 26
/** Bytes on the largest Atari track (DD, 18 x 256). */
#define ATARI_FLOPPY_MAX_BYTES 4608
/** Flux buffer size: 1.25 revolutions of the worst case (FM, all 0xFF). */
#define ATARI_FLOPPY_FLUX_BYTES 66000

/*!
 * @brief An Atari floppy in a PC drive.
 *
 * Sector writes are write-through: the whole track is rewritten, so the disk
 * must have been formatted (all sectors readable) first.
 *
 * ready(), geometry() and writeProtected() never touch the drive, so they can
 * be called before the SIO ACK. They report the last detected state: ready()
 * is true after a successful insert() until eject() or a failed re-detect,
 * geometry() is the last detected geometry and writeProtected() the state
 * seen the last time the motor was on. After idle() stops the motor, the next
 * readSector(), writeSector(), format() or writeTrack() spins up and detects
 * the density again. A different density can make a sector out of range
 * (ATARI_DISK_NOT_FOUND); no disk gives ATARI_DISK_NOT_READY and ready()
 * turns false. Until eject(), an empty drive is probed again by readSector()
 * and writeSector() at most every few seconds, so a disk inserted later is
 * picked up like on a real Atari drive. Limitation: after a disk swap,
 * geometry() keeps reporting the old density until the first read.
 */
class Adafruit_AtariFloppy : public Adafruit_AtariDisk {
public:
  /*!
   * @brief Create an instance.
   * @param drive The PC/Shugart drive; must outlive this object.
   */
  Adafruit_AtariFloppy(Adafruit_Floppy *drive);

  /*!
   * @brief Configure the drive speed and track count.
   * @param drive_rpm Nominal drive speed: 300 (360K drive) or 360 (1.2M
   * drive). Used until the real speed is measured. The drive is always put in
   * low-density mode: Atari media is 48 tpi double-density type.
   * @param drive_tracks Tracks per side the drive can reach: 40 (360K 5.25")
   * or 80 (1.2M 5.25", 3.5"). An 80-track drive reads the cylinder number in
   * the sector IDs of physical track 2 on insert to tell 40-track media
   * (stepped twice per track) from 80-track media (stepped once).
   * @return true on success; false for a zero RPM or a track count other than
   * 40 or 80.
   */
  bool begin(uint16_t drive_rpm = 300, uint8_t drive_tracks = 40);

  /*!
   * @brief Spin up, measure the revolution time and detect the density from
   * track 0, then the number of sides (a second side with the same layout on
   * track 0) and, on an 80-track drive, the number of media tracks.
   * @return true if a disk was recognized (same as ready()).
   */
  bool insert();

  /*! @brief Forget the disk and stop the motor. */
  void eject();

  /*!
   * @brief Stop the motor after about 3 s without access. Call from loop().
   * Because the disk may be swapped meanwhile, this also drops the track cache
   * and the next readSector(), writeSector(), format() or writeTrack() re-runs
   * density detection first.
   */
  void idle();

  bool ready() override;
  atari_geometry_t geometry() override;
  bool writeProtected() override;
  atari_disk_result_t readSector(uint16_t sector, uint8_t *buf) override;
  atari_disk_result_t writeSector(uint16_t sector, const uint8_t *buf,
                                  bool verify) override;
  bool canFormat(const atari_geometry_t &g) override;
  atari_disk_result_t format(const atari_geometry_t &g) override;

  /*!
   * @brief Read a sector for the SIO engine. Like readSector(), but when the
   * sector cannot be read cleanly and its ID is on the track with a data field
   * whose CRC is bad, returns ATARI_DISK_CRC_ERROR with that data and
   * info->send_data set, so a simple bad-sector protection works straight from
   * a real disk. The normal path costs nothing extra.
   * @param sector Logical sector number.
   * @param buf Receives the SIO data.
   * @param now_us Current time in microseconds (unused).
   * @param info Receives fdc_status, send_data and delay_us (always 0).
   * @return As readSector(), or ATARI_DISK_CRC_ERROR for a bad-CRC sector.
   */
  atari_disk_result_t readSectorTimed(uint16_t sector, uint8_t *buf,
                                      uint32_t now_us,
                                      atari_read_info_t *info) override;

  /*!
   * @brief Capture one track of side 0 as a protected track: every sector ID
   * with its status (duplicates, CRC errors, missing and deleted data) and,
   * from comparing revolutions, weak sectors. Uses the detected density. A
   * track with nothing readable gives an empty track (sector_count 0), not an
   * error, because protections use unformatted tracks.
   * @param track Cylinder, 0 to the media's last track.
   * @param out Receives the track.
   * @param revolutions Revolutions to capture and merge (1 to 8; default 5).
   * @param overflow If not NULL, receives the number of entries lost because
   * the track exceeded ATARI_PROT_MAX_SECTORS or ATARI_PROT_MAX_DATA (a failed
   * merge counts as one); 0 means nothing was lost.
   * @return ATARI_DISK_OK, ATARI_DISK_NOT_READY (no disk),
   * ATARI_DISK_UNSUPPORTED (track out of range) or ATARI_DISK_IO_ERROR
   * (seek failed or no flux captured).
   */
  atari_disk_result_t readProtectedTrack(uint8_t track, atari_prot_track_t *out,
                                         uint8_t revolutions = 5,
                                         unsigned *overflow = nullptr);

  /*!
   * @brief Master a protected track onto side 0 of the disk (experimental;
   * weak-bit mastering is untested on hardware). The track is encoded with the
   * drive's cell timing from the measured revolution and written from the
   * index. If a sector would straddle the index, the whole track is shifted
   * so the index (and the write splice) fall in the largest gap; Atari drives
   * do not use the index, so only the angles between sectors matter. The disk
   * need not be formatted. After writing track 39 the disk
   * counts as a single-sided 40-track disk of the track's density. Write
   * protection is checked before anything is written.
   * @param t The track, with t->density and t->track set.
   * @param verify Read it back and check that every non-weak entry is found
   * with the same number, status and data and a position (after any shift)
   * within ATARI_PROT_MERGE_TOL plus the push distance; weak entries only
   * have to be found.
   * @param moved_units If not NULL, receives the largest distance (in
   * ATARI_ATX_REV_UNITS) any entry had to be pushed later than its position.
   * @return ATARI_DISK_OK, ATARI_DISK_WRITE_PROTECTED,
   * ATARI_DISK_UNSUPPORTED (bad track, or the entries do not fit on a track)
   * or ATARI_DISK_IO_ERROR (including a failed verify).
   */
  atari_disk_result_t writeProtectedTrack(const atari_prot_track_t *t,
                                          bool verify,
                                          uint32_t *moved_units = nullptr);

  /*!
   * @brief Write one whole track of one side, formatting it for geometry g.
   * Used to copy an image onto a disk, or on a blank disk. The disk need not
   * be formatted or recognized. After the last track of the last side
   * (g.tracks - 1, g.sides - 1) is written, the disk's geometry becomes g and
   * ready() is true. Write protection is checked before anything is written.
   * @param track Track (cylinder) number, 0 to g.tracks - 1.
   * @param head Side, 0 to g.sides - 1.
   * @param g SD, ED or DD track layout (see canFormat()).
   * @param sectors SIO (non-inverted) data: sectors_per_track slots in
   * sector ID order, each the full on-disk sector_size bytes. For DD the boot
   * sectors 1-3 are 256-byte slots of which only the first 128 bytes are
   * meaningful; the caller pads the rest.
   * @param verify Read the track back and compare it.
   * @return ATARI_DISK_OK, ATARI_DISK_WRITE_PROTECTED, ATARI_DISK_UNSUPPORTED
   * or ATARI_DISK_IO_ERROR.
   */
  atari_disk_result_t writeTrack(uint8_t track, uint8_t head,
                                 const atari_geometry_t &g,
                                 const uint8_t *sectors, bool verify);

  /*!
   * @brief Write one whole track of side 0; same as writeTrack(track, 0, g,
   * sectors, verify).
   * @param track Track (cylinder) number, 0 to g.tracks - 1.
   * @param g SD, ED or DD track layout (see canFormat()).
   * @param sectors SIO data, see the other writeTrack().
   * @param verify Read the track back and compare it.
   * @return As the other writeTrack().
   */
  atari_disk_result_t writeTrack(uint8_t track, const atari_geometry_t &g,
                                 const uint8_t *sectors, bool verify) {
    return writeTrack(track, 0, g, sectors, verify);
  }

  /*!
   * @brief Revolution time used for timing, for diagnostics.
   * @return Microseconds: the last measurement, or the nominal time for the
   * configured RPM if none has been taken.
   */
  uint32_t revolutionMicros();

  /** Optional stream for diagnostics. */
  Stream *debug = nullptr;

private:
  static constexpr uint8_t NO_TRACK = 0xff;

  void detectIfStale();
  bool detectIfNeeded();
  bool motorOn();
  bool seek(uint8_t cylinder);
  size_t capture();
  uint8_t detectMediaTracks(atari_density_t density);
  bool detectSecondSide(atari_density_t density);
  bool loadTrack(uint8_t cylinder, uint8_t head);
  bool writeCached();
  atari_disk_result_t writeTrackImpl(uint8_t track, uint8_t head,
                                     const atari_geometry_t &g,
                                     const uint8_t *sectors, bool verify);
  bool verifyTrack(const uint8_t *expect, size_t slot, size_t len);
  uint32_t revUs() const;
  uint32_t revTicks() const;
  bool verifyProtected(const atari_prot_track_t *t, uint16_t shift,
                       uint32_t moved);
  uint16_t cellTicks(atari_density_t density) const;
  bool allValid() const;

  Adafruit_Floppy *_drive;
  uint16_t _rpm = 300;
  uint8_t _drive_tracks = 40; // tracks per side the drive reaches
  bool _double_step = false;  // 40-track media in an 80-track drive
  atari_geometry_t _geom;     // last detected geometry
  bool _ready = false;
  bool _write_protected = false; // as of the last time the motor was on
  bool _stale = false;           // idle() ran: redetect the disk on next access
  bool _wanted = false;          // insert() called and no eject() since
  uint32_t _last_detect_ms = 0;  // when insert() last finished
  atari_density_t _density = ATARI_DENSITY_SD;
  uint32_t _rev_ticks = 0; // last measured revolution, 0 = unknown
  uint32_t _last_access_ms = 0;
  size_t _last_pulses = 0; // pulses in g_flux from the last capture()
  uint8_t _cache_cyl = NO_TRACK;
  uint8_t _cache_head = 0;
  uint8_t _data[ATARI_FLOPPY_MAX_BYTES]; // SIO data of the cached track
  uint8_t _valid[ATARI_FLOPPY_MAX_SECTORS];
};

#endif // ADAFRUIT_ATARIFLOPPY_H
