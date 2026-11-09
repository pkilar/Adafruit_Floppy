/*!
 * @file Adafruit_AtariSIO.h
 *
 * Atari SIO disk-drive protocol engine. Receives command frames from an Atari
 * 8-bit computer through an abstract serial transport and serves them from
 * Adafruit_AtariDisk objects (ATR images or physical floppies). Pure C++ with
 * no Arduino dependency so host tests can compile it.
 */
#ifndef ADAFRUIT_ATARISIO_H
#define ADAFRUIT_ATARISIO_H

#include "Adafruit_AtariDisk.h"
#include <stddef.h>
#include <stdint.h>

/*!
 * @brief Serial transport used by Adafruit_AtariSIO (8N1, LSB first).
 *
 * Implemented for Arduino by Adafruit_AtariSIOUart and by a simulator in the
 * host tests.
 */
class Adafruit_AtariSIOPort {
public:
  virtual ~Adafruit_AtariSIOPort() {}

  /*!
   * @brief State of the SIO COMMAND line.
   * @return true while COMMAND is asserted (electrically low).
   */
  virtual bool commandAsserted() = 0;

  /*!
   * @brief Read one received byte without blocking.
   * @return The byte 0-255, or -1 if none is available.
   */
  virtual int read() = 0;

  /*!
   * @brief Transmit bytes. May block until they are queued.
   * @param data Bytes to send.
   * @param len Number of bytes.
   */
  virtual void write(const uint8_t *data, size_t len) = 0;

  /*!
   * @brief Wait until the last stop bit of everything written has left the
   * wire.
   */
  virtual void flushTx() = 0;

  /*! @brief Discard any received but unread bytes. */
  virtual void clearRx() = 0;

  /*!
   * @brief Change the baud rate (8N1). Pending RX data may be lost.
   * @param baud New rate in bits per second.
   */
  virtual void setBaud(uint32_t baud) = 0;

  /*!
   * @brief Free-running microsecond clock (wraps at 2^32).
   * @return Microseconds.
   */
  virtual uint32_t micros() = 0;

  /*!
   * @brief Busy-wait or sleep.
   * @param us Microseconds to wait.
   */
  virtual void delayMicros(uint32_t us) = 0;
};

/*!
 * @brief Atari SIO disk drives D1:-D15: on top of an Adafruit_AtariSIOPort.
 *
 * Call attach() for each emulated unit, begin(), then poll() from loop().
 * Unit n answers to SIO device ID $30 + n ($31-$3F). A unit with nothing
 * attached never answers, so a real drive with that number elsewhere on the
 * SIO chain can be used (bypass).
 * Supports R, W, P, S, !, ", N, O (128, 256 and 512-byte sectors), the Ultra
 * Speed '?' probe with autobaud switching, and the XF551 high-speed command
 * variants.
 */
class Adafruit_AtariSIO {
public:
  /** Highest unit number: D15: is SIO device ID $3F. */
  static const uint8_t MAX_UNITS = 15;

  /** Read-only diagnostic counters. */
  struct Stats {
    uint32_t frames;       ///< Valid command frames addressed to our units
    uint32_t badChecksums; ///< Command or data frames with a bad checksum
    uint32_t badFrames;    ///< Command frames that were short, long or late
    uint32_t naks;         ///< NAK bytes sent
    uint32_t errors;       ///< ERROR ('E') results sent
    uint8_t lastCommand;   ///< Command byte of the last valid frame
    uint8_t lastUnit;      ///< Unit (1-15) of the last valid frame
    uint16_t lastAux;      ///< AUX1 | AUX2 << 8 of the last valid frame
    uint8_t lastResult;    ///< Last response byte (A, N, C or E)
    bool highSpeed;        ///< true while listening at the Ultra Speed rate
  };

  /*!
   * @brief Create the engine.
   * @param port Transport; must outlive this object.
   */
  explicit Adafruit_AtariSIO(Adafruit_AtariSIOPort *port);

  /*! @brief Select the standard 19040 baud rate and reset state. */
  void begin();

  /*!
   * @brief Attach a disk to a unit.
   * @param unit Unit number 1-15 (D1: to D15:).
   * @param disk The disk; must outlive this object.
   * @return false if unit is out of range or disk is NULL.
   */
  bool attach(uint8_t unit, Adafruit_AtariDisk *disk);

  /*!
   * @brief Remove a unit; it then ignores all commands.
   * @param unit Unit number 1-15.
   */
  void detach(uint8_t unit);

  /*!
   * @brief Configure Ultra Speed ('?' command and autobaud).
   * @param pokey_divisor POKEY divisor of the high rate (baud =
   * 1789773 / (2 * (divisor + 7))); 0 disables Ultra Speed. Default 10.
   */
  void setUltraSpeed(uint8_t pokey_divisor);

  /*!
   * @brief Enable XF551 high-speed command variants (command | 0x80).
   * @param enable true to accept them (default), false to NAK them.
   */
  void setXF551HighSpeed(bool enable);

  /*!
   * @brief Service the bus. Call often from loop().
   *
   * Returns immediately unless COMMAND is asserted, in which case it receives
   * and completely handles one command (blocking, including disk access).
   */
  void poll();

  /*!
   * @brief Diagnostic counters.
   * @return Reference to the live counters.
   */
  const Stats &stats() const { return stats_; }

  /*!
   * @brief Baud rate for a POKEY divisor.
   * @param divisor POKEY AUDF3/4 divisor.
   * @return Bits per second.
   */
  static uint32_t pokeyBaud(uint8_t divisor);

private:
  struct Unit {
    Adafruit_AtariDisk *disk; ///< Attached disk or NULL
    atari_geometry_t pending; ///< Geometry from the last PERCOM write
    bool hasPending;          ///< pending is valid
    uint8_t hwStatus;         ///< Inverted WD279x status of last operation
    bool dataFrameBad;        ///< Last data frame had a bad checksum
  };

  enum RxResult { RX_OK, RX_BAD_SUM, RX_TIMEOUT, RX_ABORT };

  bool receiveCommand(uint8_t *frame);
  static uint8_t checksum(const uint8_t *data, size_t len);
  void handleCommand(const uint8_t *frame);
  void dispatch(Unit &u, uint8_t cmd, uint16_t aux, bool hs);

  void cmdRead(Unit &u, uint16_t sector);
  void cmdWrite(Unit &u, uint16_t sector, bool verify);
  void cmdStatus(Unit &u, bool xf);
  void cmdFormat(Unit &u, bool enhanced);
  void cmdGetPercom(Unit &u);
  void cmdPutPercom(Unit &u);
  void cmdSpeedIndex();

  atari_geometry_t effectiveGeometry(const Unit &u) const;
  atari_geometry_t diskGeometry(const Unit &u) const;
  bool noteResult(Unit &u, atari_disk_result_t r);

  bool cmdActive();
  bool waitSince(uint32_t since, uint32_t us);
  bool sendByte(uint8_t b);
  bool sendAck();
  bool sendNak();
  bool sendComplete(bool ok, uint32_t extra_us = 0);
  bool sendDataFrame(uint16_t len);
  bool replyWithData(bool ok, uint16_t len, uint32_t extra_us = 0,
                     bool keep_data = false);
  RxResult receiveDataFrame(uint16_t len);
  bool acceptDataFrame(Unit &u, uint16_t len);
  void toggleBaud();
  void setBaud(uint32_t baud);

  Adafruit_AtariSIOPort *port_; ///< Transport
  Unit units_[MAX_UNITS];       ///< Units D1:-D15:
  uint8_t ultraDivisor_;        ///< 0 = Ultra Speed disabled
  bool xf551_;                  ///< Accept XF551 variants
  bool highBaud_;               ///< Listening at the Ultra Speed rate
  bool keepRx_;                 ///< Do not clear RX at the next command
  uint32_t baud_;               ///< Rate currently programmed
  uint32_t frameBaud_;          ///< Rate of the command frame being handled
  uint32_t tLast_;   ///< micros() when the last byte moved on the wire
  uint32_t tAck_;    ///< micros() when our last ACK finished
  uint32_t tIdle_;   ///< micros() when poll() last saw COMMAND high
  bool hs_;          ///< Current command uses XF551 high speed
  Stats stats_;      ///< Diagnostic counters
  uint8_t buf_[513]; ///< Sector data (up to 512) plus checksum byte
};

#endif // ADAFRUIT_ATARISIO_H
