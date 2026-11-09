/*!
 * @file Adafruit_FUSB302Sink.cpp
 *
 * USB-PD sink on the FUSB302. Register numbers and FIFO tokens are from the
 * onsemi FUSB302B datasheet (Rev 1.4); message layouts and timers are from
 * the USB Power Delivery 2.0 specification. The sink always speaks PD 2.0: a
 * PD 3.0 source then uses 2.0 as well, and 12 V fixed supplies exist in both.
 */
#include "Adafruit_FUSB302Sink.h"

namespace {

// ---- FUSB302 registers ------------------------------------------------------
const uint8_t REG_DEVICE_ID = 0x01;
const uint8_t REG_SWITCHES0 = 0x02;
const uint8_t REG_SWITCHES1 = 0x03;
const uint8_t REG_CONTROL0 = 0x06;
const uint8_t REG_CONTROL1 = 0x07;
const uint8_t REG_CONTROL3 = 0x09;
const uint8_t REG_MASK = 0x0A;
const uint8_t REG_POWER = 0x0B;
const uint8_t REG_RESET = 0x0C;
const uint8_t REG_MASKA = 0x0E;
const uint8_t REG_MASKB = 0x0F;
const uint8_t REG_INTERRUPTA = 0x3E; // then INTERRUPTB, STATUS0, STATUS1,
                                     // INTERRUPT
const uint8_t REG_STATUS0 = 0x40;    // then STATUS1
const uint8_t REG_FIFOS = 0x43;

const uint8_t SW0_PDWN1 = 0x01;    // Rd on CC1
const uint8_t SW0_PDWN2 = 0x02;    // Rd on CC2
const uint8_t SW0_MEAS_CC1 = 0x04; // measure block / BC_LVL on CC1
const uint8_t SW0_MEAS_CC2 = 0x08;
const uint8_t SW1_TXCC1 = 0x01; // BMC transmitter on CC1
const uint8_t SW1_TXCC2 = 0x02;
const uint8_t SW1_AUTO_CRC = 0x04;      // answer received packets with GoodCRC
const uint8_t SW1_SPECREV_20 = 0x20;    // GoodCRC header: revision 2.0
const uint8_t CTL0_HOST_CUR_USB = 0x04; // reset value without INT_MASK
const uint8_t CTL0_TX_FLUSH = 0x40;
const uint8_t CTL1_RX_FLUSH = 0x04;
const uint8_t CTL3_RETRIES = 0x07; // AUTO_RETRY, N_RETRIES = 3
const uint8_t POWER_ALL = 0x0F;    // bandgap, receiver, measure, oscillator
const uint8_t RESET_SW = 0x01;
const uint8_t RESET_PD = 0x02;

const uint8_t IA_HARDRST = 0x01;   // INTERRUPTA: hard reset received
const uint8_t IA_TXSENT = 0x04;    // GoodCRC received for our packet
const uint8_t IA_RETRYFAIL = 0x10; // no GoodCRC after the retries
const uint8_t S0_VBUSOK = 0x80;    // STATUS0
const uint8_t S0_ACTIVITY = 0x40;  // BMC transitions on the CC line right now
const uint8_t S0_BC_LVL = 0x03;
const uint8_t S1_RX_EMPTY = 0x20; // STATUS1
const uint8_t I_COLLISION = 0x02; // INTERRUPT

// TX FIFO tokens; RX packets start with a token whose top 3 bits are the SOP
// type (111 = SOP, the only one enabled).
const uint8_t TX_SOP1 = 0x12;
const uint8_t TX_SOP2 = 0x13;
const uint8_t TX_PACKSYM = 0x80;
const uint8_t TX_JAM_CRC = 0xFF;
const uint8_t TX_EOP = 0x14;
const uint8_t TX_OFF = 0xFE;
const uint8_t TX_ON = 0xA1;
const uint8_t RX_TOKEN_MASK = 0xE0;
const uint8_t RX_TOKEN_SOP = 0xE0;

// ---- PD 2.0 messages
// ---------------------------------------------------------
const uint8_t CTRL_GOODCRC = 1;
const uint8_t CTRL_GOTOMIN = 2;
const uint8_t CTRL_ACCEPT = 3;
const uint8_t CTRL_REJECT = 4;
const uint8_t CTRL_PING = 5;
const uint8_t CTRL_PS_RDY = 6;
const uint8_t CTRL_GET_SINK_CAP = 8;
const uint8_t CTRL_WAIT = 12;
const uint8_t CTRL_SOFT_RESET = 13;
const uint8_t DATA_SOURCE_CAPS = 1;
const uint8_t DATA_REQUEST = 2;
const uint8_t DATA_SINK_CAPS = 4;
const uint8_t TYPE_DATA = 0x80; // marks a data message in txType_
const uint8_t REV_20 = 1;       // header Specification Revision field

// ---- Timers (ms; PD 2.0 values with some margin)
// -----------------------------
const uint32_t CAPS_WAIT_MS = 650;    // tTypeCSinkWaitCap max 620
const uint32_t CAPS_RESET_MS = 1000;  // after our Soft_Reset was accepted
const uint32_t RESPONSE_MS = 35;      // tSenderResponse 24-30
const uint32_t TRANSITION_MS = 650;   // tPSTransition 450-550
const uint32_t SINK_REQUEST_MS = 120; // tSinkRequest 100 (after Wait)
const uint32_t TX_TIMEOUT_MS = 20;    // retries finish in ~5 ms
const uint32_t MEASURE_MS = 2;        // BC_LVL settles in < 1 ms
const uint8_t MAX_SOFT_RESETS = 3;    // then assume a non-PD source
const uint8_t MAX_REQUEST_RESENDS = 2;

uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

bool isFixed(uint32_t pdo) { return (pdo >> 30) == 0; }
uint16_t fixedMv(uint32_t pdo) { return ((pdo >> 10) & 0x3FF) * 50; }
uint16_t fixedMa(uint32_t pdo) { return (pdo & 0x3FF) * 10; }

} // namespace

Adafruit_FUSB302Sink::Adafruit_FUSB302Sink(Adafruit_FUSB302Bus &bus)
    : bus_(&bus), targetMv_(12000), targetMa_(3000), state_(PD_OFF), since_(0),
      attachPhase_(0), bc1_(0), cc_(0), srcRev_(0), rxId_(-1), txId_(0),
      txBusy_(false), txType_(0), txAt_(0), pendType_(0), pendCount_(0),
      softTries_(0), reqResends_(0), resendRequest_(false), reqPos_(0),
      reqMv_(0), reqMa_(0), fallback5V_(false), contractMv_(0), contractMa_(0),
      pdoCount_(0), hardResets_(0), softResets_(0) {
  for (uint8_t i = 0; i < MAX_PDOS; i++)
    pdos_[i] = pendObj_[i] = 0;
}

void Adafruit_FUSB302Sink::setTarget(uint16_t millivolts, uint16_t milliamps) {
  targetMv_ = millivolts;
  targetMa_ = milliamps;
}

const char *Adafruit_FUSB302Sink::stateName(State s) {
  switch (s) {
  case PD_OFF:
    return "off";
  case PD_DETACHED:
    return "detached";
  case PD_ATTACHING:
    return "attaching";
  case PD_WAIT_CAPS:
    return "wait caps";
  case PD_SOFT_RESET:
    return "soft reset";
  case PD_REQUESTED:
    return "requested";
  case PD_WAIT_RETRY:
    return "wait";
  case PD_TRANSITION:
    return "transition";
  case PD_READY:
    return "ready";
  case PD_NO_PD:
    return "no PD";
  }
  return "?";
}

bool Adafruit_FUSB302Sink::writeReg(uint8_t reg, uint8_t value) {
  return bus_->write(reg, &value, 1);
}

bool Adafruit_FUSB302Sink::begin(uint32_t nowMs) {
  uint8_t id = 0;
  state_ = PD_OFF;
  if (!bus_->read(REG_DEVICE_ID, &id, 1) || id == 0 || id == 0xFF)
    return false;
  // A full reset leaves SWITCHES0 at PDWN1|PDWN2: the Rd the source sees never
  // goes away, so VBUS stays on.
  if (!writeReg(REG_RESET, RESET_SW))
    return false;
  writeReg(REG_POWER, POWER_ALL);
  writeReg(REG_CONTROL3, CTL3_RETRIES);
  // Unmask everything: the interrupt bits are polled (INT_N is not wired).
  writeReg(REG_MASK, 0x00);
  writeReg(REG_MASKA, 0x00);
  writeReg(REG_MASKB, 0x00);
  hardResets_ = softResets_ = 0;
  reset(PD_DETACHED, nowMs);
  return true;
}

void Adafruit_FUSB302Sink::enter(State s, uint32_t nowMs) {
  state_ = s;
  since_ = nowMs;
}

void Adafruit_FUSB302Sink::reset(State next, uint32_t nowMs) {
  writeReg(REG_RESET, RESET_PD);
  writeReg(REG_CONTROL0, CTL0_HOST_CUR_USB | CTL0_TX_FLUSH);
  writeReg(REG_CONTROL1, CTL1_RX_FLUSH);
  rxId_ = -1;
  txId_ = 0;
  txBusy_ = false;
  pendCount_ = pendType_ = 0;
  softTries_ = 0;
  resendRequest_ = false;
  contractMv_ = contractMa_ = 0;
  if (next == PD_DETACHED) {
    writeReg(REG_SWITCHES1, SW1_SPECREV_20); // no GoodCRC, no transmitter
    writeReg(REG_SWITCHES0, SW0_PDWN1 | SW0_PDWN2);
    cc_ = 0;
    srcRev_ = 0;
    pdoCount_ = 0;
  }
  enter(next, nowMs);
}

void Adafruit_FUSB302Sink::poll(uint32_t nowMs) {
  if (state_ == PD_OFF)
    return;
  // INTERRUPTA, INTERRUPTB, STATUS0, STATUS1, INTERRUPT (reading clears the
  // interrupt bits)
  uint8_t st[5];
  if (!bus_->read(REG_INTERRUPTA, st, sizeof(st)))
    return;
  uint8_t ia = st[0], s0 = st[2], s1 = st[3], irq = st[4];

  if (!(s0 & S0_VBUSOK)) {
    if (state_ != PD_DETACHED)
      reset(PD_DETACHED, nowMs);
    return;
  }
  if (state_ == PD_DETACHED) {
    attachPhase_ = 0;
    writeReg(REG_SWITCHES0, SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC1);
    enter(PD_ATTACHING, nowMs);
    return;
  }
  if (state_ == PD_ATTACHING) {
    if (!(s0 & S0_ACTIVITY)) // BC_LVL is wrong while a packet is on the line
      attachStep(s0 & S0_BC_LVL, nowMs);
    return;
  }
  if (ia & IA_HARDRST) {
    // The source turns VBUS off and back on; start over at 5 V.
    hardResets_++;
    reset(PD_WAIT_CAPS, nowMs);
    return;
  }
  if (txBusy_) {
    if (ia & IA_TXSENT)
      txDone(true, nowMs);
    else if ((ia & IA_RETRYFAIL) || (irq & I_COLLISION) ||
             nowMs - txAt_ > TX_TIMEOUT_MS)
      txDone(false, nowMs);
  }
  // While a packet is arriving the FIFO holds part of it: read only on a
  // quiet line, when everything in the FIFO is complete.
  if (!(s1 & S1_RX_EMPTY) && !(s0 & S0_ACTIVITY))
    readMessages(nowMs);

  uint32_t age = nowMs - since_;
  switch (state_) {
  case PD_WAIT_CAPS:
    // A source sends its capabilities right after attach. If none come, the
    // contract is older than this firmware (a reboot): a Soft_Reset makes the
    // source send them again without touching VBUS.
    if (age >= (softTries_ ? CAPS_RESET_MS : CAPS_WAIT_MS)) {
      if (softTries_ < MAX_SOFT_RESETS)
        sendSoftReset(nowMs);
      else
        enter(PD_NO_PD, nowMs);
    }
    break;
  case PD_SOFT_RESET:
    if (!txBusy_ && age >= RESPONSE_MS)
      enter(PD_WAIT_CAPS, nowMs); // counted as a try
    break;
  case PD_REQUESTED:
    if (resendRequest_ && !txBusy_) {
      resendRequest_ = false;
      sendRequest(nowMs);
    } else if (!txBusy_ && age >= RESPONSE_MS) {
      sendSoftReset(nowMs);
    }
    break;
  case PD_WAIT_RETRY:
    if (age >= SINK_REQUEST_MS)
      sendRequest(nowMs);
    break;
  case PD_TRANSITION:
    if (age >= TRANSITION_MS)
      sendSoftReset(nowMs);
    break;
  default:
    break;
  }
}

void Adafruit_FUSB302Sink::attachStep(uint8_t bcLvl, uint32_t nowMs) {
  if (nowMs - since_ < MEASURE_MS)
    return;
  if (attachPhase_ == 0) {
    bc1_ = bcLvl;
    attachPhase_ = 1;
    writeReg(REG_SWITCHES0, SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC2);
    since_ = nowMs;
    return;
  }
  if (bc1_ == 0 && bcLvl == 0) {
    // No Rp seen on either line yet: measure again.
    attachPhase_ = 0;
    writeReg(REG_SWITCHES0, SW0_PDWN1 | SW0_PDWN2 | SW0_MEAS_CC1);
    since_ = nowMs;
    return;
  }
  cc_ = bcLvl > bc1_ ? 2 : 1;
  writeReg(REG_SWITCHES0,
           SW0_PDWN1 | SW0_PDWN2 | (cc_ == 1 ? SW0_MEAS_CC1 : SW0_MEAS_CC2));
  writeReg(REG_SWITCHES1,
           (cc_ == 1 ? SW1_TXCC1 : SW1_TXCC2) | SW1_AUTO_CRC | SW1_SPECREV_20);
  reset(PD_WAIT_CAPS, nowMs);
}

void Adafruit_FUSB302Sink::readMessages(uint32_t nowMs) {
  for (uint8_t k = 0; k < 4; k++) {
    uint8_t head[3]; // token, header
    if (!bus_->read(REG_FIFOS, head, sizeof(head)))
      return;
    uint16_t header = head[1] | ((uint16_t)head[2] << 8);
    uint8_t n = (header >> 12) & 7;
    uint8_t body[MAX_PDOS * 4 + 4]; // data objects + CRC
    if (!bus_->read(REG_FIFOS, body, n * 4 + 4))
      return;
    if ((head[0] & RX_TOKEN_MASK) == RX_TOKEN_SOP) {
      uint32_t obj[MAX_PDOS];
      for (uint8_t i = 0; i < n; i++)
        obj[i] = le32(body + 4 * i);
      handle(header, obj, n, nowMs);
    }
    uint8_t st[2]; // STATUS0, STATUS1
    if (!bus_->read(REG_STATUS0, st, 2) || (st[1] & S1_RX_EMPTY) ||
        (st[0] & S0_ACTIVITY))
      return;
  }
}

void Adafruit_FUSB302Sink::handle(uint16_t header, const uint32_t *obj,
                                  uint8_t n, uint32_t nowMs) {
  uint8_t type = header & 0x1F;
  int8_t id = (header >> 9) & 7;
  if (n == 0 && type == CTRL_GOODCRC)
    return; // the chip already matched it to our packet
  if (n == 0 && type == CTRL_SOFT_RESET) {
    // Both sides restart their MessageID counters; the power level stays.
    softResets_++;
    rxId_ = -1;
    txId_ = 0;
    txBusy_ = false;
    pendCount_ = pendType_ = 0;
    softTries_ = 0;
    send(CTRL_ACCEPT, 0, 0, nowMs);
    enter(PD_WAIT_CAPS, nowMs);
    return;
  }
  if (id == rxId_)
    return; // a retry of a message we already acknowledged
  rxId_ = id;
  uint8_t rev = (header >> 6) & 3;
  if (rev)
    srcRev_ = rev;

  if (n > 0) {
    if (type == DATA_SOURCE_CAPS)
      handleSourceCaps(obj, n, nowMs);
    // PD 2.0: other data messages (BIST, VDMs, Sink_Capabilities) need no
    // answer from a sink without those features.
    return;
  }
  switch (type) {
  case CTRL_ACCEPT:
    if (state_ == PD_REQUESTED)
      enter(PD_TRANSITION, nowMs);
    else if (state_ == PD_SOFT_RESET)
      enter(PD_WAIT_CAPS, nowMs);
    break;
  case CTRL_REJECT:
    if (state_ == PD_REQUESTED) {
      if (reqPos_ != 1) {
        fallback5V_ = true; // ask for 5 V instead
        sendRequest(nowMs);
      } else {
        enter(PD_READY, nowMs); // keep the implicit 5 V contract
      }
    }
    break;
  case CTRL_WAIT:
    if (state_ == PD_REQUESTED)
      enter(PD_WAIT_RETRY, nowMs);
    break;
  case CTRL_PS_RDY:
    if (state_ == PD_TRANSITION) {
      contractMv_ = reqMv_;
      contractMa_ = reqMa_;
      enter(PD_READY, nowMs);
    }
    break;
  case CTRL_GET_SINK_CAP: {
    // 5 V and the target voltage, 3 A max; "higher capability" set because
    // the board wants more than vSafe5V.
    uint32_t caps[2];
    caps[0] = (1UL << 29) | (100UL << 10) | 300;
    caps[1] = ((uint32_t)(targetMv_ / 50) << 10) | (targetMa_ / 10);
    send(DATA_SINK_CAPS, caps, targetMv_ > 5000 ? 2 : 1, nowMs);
    break;
  }
  case CTRL_GOODCRC:
  case CTRL_GOTOMIN: // only sent when GiveBack is set, which it never is
  case CTRL_PING:
    break;
  default:
    // Get_Source_Cap, the swaps and anything newer: a sink-only PD 2.0 port
    // answers Reject.
    send(CTRL_REJECT, 0, 0, nowMs);
    break;
  }
}

void Adafruit_FUSB302Sink::handleSourceCaps(const uint32_t *obj, uint8_t n,
                                            uint32_t nowMs) {
  pdoCount_ = 0;
  for (uint8_t i = 0; i < n && i < MAX_PDOS; i++)
    pdos_[i] = obj[i];
  pdoCount_ = n;
  softTries_ = 0;
  reqResends_ = 0;
  fallback5V_ = false;
  sendRequest(nowMs);
}

void Adafruit_FUSB302Sink::sendRequest(uint32_t nowMs) {
  if (!pdoCount_)
    return;
  // PDO 1 is always vSafe5V; take the target voltage if a fixed PDO has it.
  uint8_t pos = 1;
  uint32_t pdo = pdos_[0];
  bool mismatch = true;
  for (uint8_t i = 0; i < pdoCount_ && !fallback5V_; i++) {
    if (isFixed(pdos_[i]) && fixedMv(pdos_[i]) == targetMv_) {
      pos = i + 1;
      pdo = pdos_[i];
      mismatch = false;
      break;
    }
  }
  uint16_t ma = fixedMa(pdo) < targetMa_ ? fixedMa(pdo) : targetMa_;
  reqPos_ = pos;
  reqMv_ = fixedMv(pdo);
  reqMa_ = ma;
  // Fixed RDO: object position, capability mismatch, no USB suspend,
  // operating and maximum current in 10 mA units. USB communication is not
  // flagged: J1 carries power only.
  uint32_t rdo = ((uint32_t)pos << 28) | ((uint32_t)mismatch << 26) |
                 (1UL << 24) | ((uint32_t)(ma / 10) << 10) | (ma / 10);
  send(DATA_REQUEST | TYPE_DATA, &rdo, 1, nowMs);
  enter(PD_REQUESTED, nowMs);
}

void Adafruit_FUSB302Sink::sendSoftReset(uint32_t nowMs) {
  // The sender restarts its counters and uses MessageID 0.
  softResets_++;
  softTries_++;
  rxId_ = -1;
  txId_ = 0;
  txBusy_ = false;
  pendCount_ = pendType_ = 0;
  writeReg(REG_CONTROL0, CTL0_HOST_CUR_USB | CTL0_TX_FLUSH);
  send(CTRL_SOFT_RESET, 0, 0, nowMs);
  enter(PD_SOFT_RESET, nowMs);
}

bool Adafruit_FUSB302Sink::send(uint8_t type, const uint32_t *obj, uint8_t n,
                                uint32_t nowMs) {
  if (txBusy_) {
    // One message in flight; keep the latest answer and send it next.
    pendType_ = type;
    pendCount_ = n;
    for (uint8_t i = 0; i < n; i++)
      pendObj_[i] = obj[i];
    return false;
  }
  uint16_t header = (type & 0x1F) | (REV_20 << 6) | ((uint16_t)txId_ << 9) |
                    ((uint16_t)n << 12); // power role sink, data role UFP
  uint8_t buf[4 + 1 + 2 + MAX_PDOS * 4 + 4];
  uint8_t k = 0;
  buf[k++] = TX_SOP1;
  buf[k++] = TX_SOP1;
  buf[k++] = TX_SOP1;
  buf[k++] = TX_SOP2;
  buf[k++] = TX_PACKSYM | (2 + 4 * n);
  buf[k++] = header & 0xFF;
  buf[k++] = header >> 8;
  for (uint8_t i = 0; i < n; i++) {
    buf[k++] = obj[i] & 0xFF;
    buf[k++] = (obj[i] >> 8) & 0xFF;
    buf[k++] = (obj[i] >> 16) & 0xFF;
    buf[k++] = obj[i] >> 24;
  }
  buf[k++] = TX_JAM_CRC;
  buf[k++] = TX_EOP;
  buf[k++] = TX_OFF;
  buf[k++] = TX_ON;
  if (!bus_->write(REG_FIFOS, buf, k))
    return false;
  txBusy_ = true;
  txType_ = type;
  txAt_ = nowMs;
  return true;
}

void Adafruit_FUSB302Sink::txDone(bool ok, uint32_t nowMs) {
  txBusy_ = false;
  txId_ = (txId_ + 1) & 7; // the counter moves on either way
  if (txType_ == (DATA_REQUEST | TYPE_DATA) && state_ == PD_REQUESTED) {
    if (ok) {
      since_ = nowMs; // the source's answer is due tSenderResponse from now
    } else if (reqResends_ < MAX_REQUEST_RESENDS) {
      // The source hard-resets if no Request arrives in time: send it again
      // on the next poll (after a collision the line is still busy now).
      reqResends_++;
      resendRequest_ = true;
    }
  }
  if (pendCount_ || pendType_) {
    uint8_t type = pendType_, n = pendCount_;
    pendType_ = pendCount_ = 0;
    send(type, pendObj_, n, nowMs);
  }
}
