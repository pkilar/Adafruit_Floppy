/*!
 * @file Adafruit_FUSB302Sink.h
 *
 * USB Power Delivery sink on an onsemi FUSB302 Type-C/PD controller: finds
 * the active CC line, answers the source's Source_Capabilities with a Request
 * for one fixed voltage (5 V if that voltage is not offered) and keeps the
 * contract alive. It never sends a Hard Reset, because a hard reset turns
 * VBUS off and would reset a board powered from that port.
 *
 * Plain C++ without Arduino calls: registers are reached through
 * Adafruit_FUSB302Bus and the caller passes the time in, so host tests can
 * run it against a model of the chip and a PD source. The FUSB302's INT_N pin
 * is not used; call poll() every few milliseconds instead (a source waits
 * about 25 ms for a Request).
 */
#ifndef ADAFRUIT_FUSB302SINK_H
#define ADAFRUIT_FUSB302SINK_H

#include <stddef.h>
#include <stdint.h>

/*!
 * @brief Register access to one FUSB302.
 *
 * Reads and writes start at a register address; a multi-byte access walks
 * consecutive registers, except at the FIFOS register (0x43), where every byte
 * goes to or comes from the FIFO.
 */
class Adafruit_FUSB302Bus {
public:
  virtual ~Adafruit_FUSB302Bus() {}
  /*!
   * @brief Read registers.
   * @param reg First register.
   * @param buf Destination.
   * @param len Byte count.
   * @return true on success.
   */
  virtual bool read(uint8_t reg, uint8_t *buf, size_t len) = 0;
  /*!
   * @brief Write registers.
   * @param reg First register.
   * @param buf Bytes to write.
   * @param len Byte count.
   * @return true on success.
   */
  virtual bool write(uint8_t reg, const uint8_t *buf, size_t len) = 0;
};

/*!
 * @brief USB-PD sink policy and protocol layer for the FUSB302.
 *
 * Usage: begin(), then poll() every 1-5 ms (from a timer interrupt is fine as
 * long as nothing else uses the bus). The accessors may be read from another
 * context; each value is updated atomically, though a PDO list read during a
 * negotiation can mix two Source_Capabilities.
 */
class Adafruit_FUSB302Sink {
public:
  /*! @brief Where the sink is in the negotiation. */
  enum State : uint8_t {
    PD_OFF = 0,    ///< begin() not called or the chip did not answer
    PD_DETACHED,   ///< no VBUS on the port
    PD_ATTACHING,  ///< measuring CC1/CC2 to find the cable orientation
    PD_WAIT_CAPS,  ///< waiting for Source_Capabilities
    PD_SOFT_RESET, ///< sent Soft_Reset, waiting for Accept
    PD_REQUESTED,  ///< sent Request, waiting for Accept
    PD_WAIT_RETRY, ///< the source said Wait; asking again shortly
    PD_TRANSITION, ///< Accepted, waiting for PS_RDY
    PD_READY,      ///< explicit contract in place
    PD_NO_PD       ///< VBUS present but the source does not talk PD
  };

  static const uint8_t MAX_PDOS = 7; ///< data objects in one PD message

  /*!
   * @brief Create the sink.
   * @param bus Register access to the chip.
   */
  explicit Adafruit_FUSB302Sink(Adafruit_FUSB302Bus &bus);

  /*!
   * @brief Choose what to ask for. Call before begin(); the default is
   *        12 V at 3 A.
   * @param millivolts Fixed supply voltage to request when the source offers
   *        it (otherwise the sink asks for 5 V).
   * @param milliamps Operating current to request, capped at what the
   *        source offers.
   */
  void setTarget(uint16_t millivolts, uint16_t milliamps);

  /*!
   * @brief Reset and configure the chip. The CC pull-downs stay on, so a
   *        source keeps VBUS at 5 V meanwhile.
   * @param nowMs Current time in milliseconds.
   * @return false if the chip does not answer.
   */
  bool begin(uint32_t nowMs);

  /*!
   * @brief Run the state machine: read status, handle received messages,
   *        timeouts and finished transmissions.
   * @param nowMs Current time in milliseconds (wraps).
   */
  void poll(uint32_t nowMs);

  /*!
   * @brief Negotiation state.
   * @return One of State.
   */
  State state() const { return state_; }

  /*!
   * @brief Voltage of the explicit contract.
   * @return Millivolts, 0 without a contract.
   */
  uint16_t contractMillivolts() const { return contractMv_; }

  /*!
   * @brief Current of the explicit contract.
   * @return Milliamps, 0 without a contract.
   */
  uint16_t contractMilliamps() const { return contractMa_; }

  /*!
   * @brief Active CC line.
   * @return 1 or 2, 0 while detached or unknown.
   */
  uint8_t ccLine() const { return cc_; }

  /*!
   * @brief PD revision the source uses.
   * @return 2 or 3, 0 before the first message.
   */
  uint8_t sourceRevision() const { return srcRev_ ? srcRev_ + 1 : 0; }

  /*!
   * @brief Number of PDOs in the last Source_Capabilities.
   * @return 0..MAX_PDOS.
   */
  uint8_t sourcePdoCount() const { return pdoCount_; }

  /*!
   * @brief One PDO of the last Source_Capabilities.
   * @param i Index, 0-based.
   * @return Raw 32-bit power data object, 0 if out of range.
   */
  uint32_t sourcePdo(uint8_t i) const { return i < pdoCount_ ? pdos_[i] : 0; }

  /*!
   * @brief Hard resets the source sent (each one also turned VBUS off).
   * @return Count since begin().
   */
  uint16_t hardResets() const { return hardResets_; }

  /*!
   * @brief Soft resets sent or received.
   * @return Count since begin().
   */
  uint16_t softResets() const { return softResets_; }

  /*!
   * @brief Name of a state, for logs.
   * @param s State.
   * @return Short name.
   */
  static const char *stateName(State s);

private:
  // Message bookkeeping
  void reset(State next, uint32_t nowMs);
  void attachStep(uint8_t bcLvl, uint32_t nowMs);
  void readMessages(uint32_t nowMs);
  void handle(uint16_t header, const uint32_t *obj, uint8_t n, uint32_t nowMs);
  void handleSourceCaps(const uint32_t *obj, uint8_t n, uint32_t nowMs);
  void notSupported(uint32_t nowMs);
  bool send(uint8_t type, const uint32_t *obj, uint8_t n, uint32_t nowMs);
  void txDone(bool ok, uint32_t nowMs);
  void sendRequest(uint32_t nowMs);
  void sendSoftReset(uint32_t nowMs);
  void enter(State s, uint32_t nowMs);
  bool writeReg(uint8_t reg, uint8_t value);

  Adafruit_FUSB302Bus *bus_;
  uint16_t targetMv_, targetMa_;
  volatile State state_;
  uint32_t since_;      // time state_ was entered
  uint8_t attachPhase_; // ATTACHING: 0 = measuring CC1, 1 = CC2
  uint8_t bc1_;         // BC_LVL seen on CC1
  volatile uint8_t cc_; // 1, 2 or 0
  uint8_t srcRev_; // header revision field of the source (1 = 2.0, 2 = 3.0)
  int8_t rxId_;    // last MessageID received, -1 = none
  uint8_t txId_;   // MessageIDCounter
  bool txBusy_;    // a message is in flight
  uint8_t txType_; // its type (data messages | 0x80)
  uint32_t txAt_;
  uint8_t pendType_; // answer queued while a message was in flight
  uint8_t pendCount_;
  uint32_t pendObj_[MAX_PDOS];
  uint8_t softTries_;  // Soft_Resets sent without getting capabilities
  uint8_t reqResends_; // Requests resent after a failed transmission
  bool resendRequest_; // send the Request again on the next poll
  uint8_t reqPos_;     // object position of the outstanding Request
  uint16_t reqMv_, reqMa_;
  bool fallback5V_; // the source rejected the target voltage
  volatile uint16_t contractMv_, contractMa_;
  volatile uint8_t pdoCount_;
  uint32_t pdos_[MAX_PDOS];
  volatile uint16_t hardResets_, softResets_;
};

#endif // ADAFRUIT_FUSB302SINK_H
