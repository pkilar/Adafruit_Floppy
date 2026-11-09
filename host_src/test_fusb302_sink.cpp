// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// Adafruit_FUSB302Sink against a register model of the FUSB302 and a USB-PD
// source that behaves like the PD 2.0 policy engine: it repeats
// Source_Capabilities until they are acknowledged, hard-resets (VBUS off) when
// a Request is late, and answers Soft_Reset. Every scenario must end in the
// expected contract without a single hard reset.
#include "Adafruit_FUSB302Sink.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <vector>

static int failures = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                    \
      failures++;                                                              \
    }                                                                          \
  } while (0)

struct Msg {
  uint16_t header;
  uint8_t n;
  uint32_t obj[7];
  uint8_t type() const { return header & 0x1F; }
  uint8_t id() const { return (header >> 9) & 7; }
};

static uint32_t fixedPdo(uint32_t mv, uint32_t ma) {
  return ((mv / 50) << 10) | (ma / 10);
}

// ---- PD source
// ---------------------------------------------------------------
struct Source {
  // configuration
  std::vector<uint32_t> pdos;
  uint8_t cc = 1;               // CC line the cable connects
  uint8_t rp = 3;               // BC_LVL the sink sees on that line (3 A Rp)
  uint8_t rev = 2;              // header revision it starts with (2 = PD 3.0)
  bool pd = true;               // false: a plain 5 V Type-C supply
  bool waitOnce = false;        // answer the first Request with Wait
  bool rejectHigh = false;      // reject anything but PDO 1
  bool startInContract = false; // already at a contract (sink rebooted)
  uint32_t startMv = 12000;

  // state
  enum { OFF, CAPS, WAIT_REQ, ACCEPTED, READY, HARD_RESET } pe = OFF;
  uint32_t vbusMv = 0;
  uint32_t t = 0, timer = 0, capsSent = 0;
  uint8_t txId = 0;
  int rxId = -1;
  bool waited = false;
  int hardResets = 0, requests = 0, sinkCapsReplies = 0, rejects = 0;
  uint32_t lastRdo = 0, contractMv = 0;
  uint8_t sinkRev = 0;
  std::deque<std::pair<uint32_t, Msg>> outbox; // (due time, message)

  Msg make(uint8_t type, const uint32_t *obj, uint8_t n) {
    Msg m;
    m.header = type | (rev << 6) | (1 << 8) | (1 << 5) | (txId << 9) |
               (n << 12); // source, DFP
    m.n = n;
    for (uint8_t i = 0; i < n; i++)
      m.obj[i] = obj[i];
    return m;
  }
  void queue(uint32_t delay, uint8_t type, const uint32_t *obj = 0,
             uint8_t n = 0) {
    outbox.push_back(std::make_pair(t + delay, make(type, obj, n)));
  }
  void sendCaps(uint32_t delay) {
    queue(delay, 1, pdos.data(), (uint8_t)pdos.size());
  }
  void attach(uint32_t now) {
    t = now;
    if (startInContract) {
      vbusMv = startMv;
      contractMv = startMv;
      pe = READY;
      txId = 5; // whatever the previous conversation left
      rxId = 3;
      return;
    }
    vbusMv = 5000;
    pe = CAPS;
    txId = 0;
    rxId = -1;
    timer = now + 50; // tFirstSourceCap
    capsSent = 0;
  }
  void hardReset() {
    hardResets++;
    pe = HARD_RESET;
    vbusMv = 0;
    timer = t + 800; // tSrcRecover + VBUS on
    outbox.clear();
    txId = 0;
    rxId = -1;
    contractMv = 0;
  }
  // Called by the chip model when the sink transmits; returns the GoodCRC.
  bool receive(const Msg &m) {
    if (!pd || pe == HARD_RESET || pe == OFF)
      return false;
    CHECK(!(m.header & (1 << 8))); // sink power role
    CHECK(!(m.header & (1 << 5))); // UFP
    sinkRev = (m.header >> 6) & 3;
    if (m.n == 0 && m.type() == 13) { // Soft_Reset
      txId = 0;
      rxId = -1;
      outbox.clear();
      queue(2, 3); // Accept
      sendCaps(6);
      pe = CAPS;
      timer = t + 1000000; // the queued caps do it
      return true;
    }
    if ((int)m.id() == rxId)
      return true; // duplicate
    rxId = m.id();
    if (sinkRev < rev)
      rev = sinkRev;                 // fall back to the sink's revision
    if (m.n == 1 && m.type() == 2) { // Request
      requests++;
      lastRdo = m.obj[0];
      uint8_t pos = (m.obj[0] >> 28) & 7;
      CHECK(pe == WAIT_REQ || pe == READY);
      if (pos == 0 || pos > pdos.size() || (rejectHigh && pos != 1)) {
        rejects++;
        queue(3, 4); // Reject
        pe = READY;
        return true;
      }
      if (waitOnce && !waited) {
        waited = true;
        queue(3, 12); // Wait
        pe = READY;
        return true;
      }
      queue(3, 3);  // Accept
      queue(40, 6); // PS_RDY after the transition
      pe = ACCEPTED;
      timer = t + 40;
      contractMv = ((pdos[pos - 1] >> 10) & 0x3FF) * 50;
      return true;
    }
    if (m.n > 0 && m.type() == 4) { // Sink_Capabilities
      sinkCapsReplies++;
      CHECK(((m.obj[0] >> 10) & 0x3FF) * 50 == 5000);
      return true;
    }
    return true;
  }
  // The chip acknowledged one of our messages (GoodCRC from the sink).
  void acked(const Msg &m) {
    txId = (txId + 1) & 7;
    if (m.n > 0 && m.type() == 1) { // Source_Capabilities
      pe = WAIT_REQ;
      timer = t + 27; // tSenderResponse
    }
  }
  void nacked(const Msg &m) {
    txId = (txId + 1) & 7;
    (void)m;
  }
};

// ---- FUSB302 model
// -------------------------------------------------------------
struct Chip : public Adafruit_FUSB302Bus {
  uint8_t r[256];
  std::deque<uint8_t> rx;
  Source *src;
  uint32_t t = 0, activityUntil = 0;
  int collideNext = 0; // collide with this many coming sink transmissions
  int writes = 0;
  bool sawHardResetToken = false, droppedRd = false, overflow = false;

  explicit Chip(Source *s) : src(s) { powerOn(); }
  void powerOn() {
    memset(r, 0, sizeof(r));
    r[0x01] = 0x91;
    r[0x02] = 0x03;
    r[0x03] = 0x20;
    r[0x04] = 0x31;
    r[0x06] = 0x24;
    r[0x08] = 0x02;
    r[0x09] = 0x06;
    r[0x0B] = 0x01;
    rx.clear();
  }
  uint8_t cc() const {
    uint8_t sw1 = r[0x03];
    if (!(sw1 & 0x04))
      return 0; // AUTO_CRC off: nothing acknowledged
    return (sw1 & 0x01) ? 1 : (sw1 & 0x02) ? 2 : 0;
  }
  bool vbusOk() const { return src->vbusMv >= 4000; }
  uint8_t status0() const {
    uint8_t s = vbusOk() ? 0x80 : 0;
    if (t < activityUntil)
      s |= 0x40;
    uint8_t meas = (r[0x02] >> 2) & 3;
    if (vbusOk() &&
        ((meas == 1 && src->cc == 1) || (meas == 2 && src->cc == 2)))
      s |= src->rp;
    return s;
  }
  bool read(uint8_t reg, uint8_t *buf, size_t len) override {
    for (size_t i = 0; i < len; i++) {
      uint8_t a = reg == 0x43 ? 0x43 : (uint8_t)(reg + i);
      switch (a) {
      case 0x40:
        buf[i] = status0();
        break;
      case 0x41:
        buf[i] = rx.empty() ? 0x28 : 0x08;
        break;
      case 0x3E:
      case 0x3F:
      case 0x42:
        buf[i] = r[a];
        r[a] = 0; // read clears
        break;
      case 0x43:
        if (rx.empty()) {
          buf[i] = 0;
        } else {
          buf[i] = rx.front();
          rx.pop_front();
        }
        break;
      default:
        buf[i] = r[a];
      }
    }
    return true;
  }
  bool write(uint8_t reg, const uint8_t *buf, size_t len) override {
    writes++;
    if (reg == 0x43)
      return transmit(buf, len);
    for (size_t i = 0; i < len; i++) {
      uint8_t a = reg + i, v = buf[i];
      if (a == 0x0C) {
        if (v & 0x01)
          powerOn();
        if (v & 0x02)
          rx.clear(); // PD logic reset
        continue;
      }
      if (a == 0x06 && (v & 0x40))
        v &= ~0x40; // TX_FLUSH self-clears
      if (a == 0x07 && (v & 0x04)) {
        rx.clear();
        v &= ~0x04;
      }
      if (a == 0x09 && (v & 0x50))
        sawHardResetToken = true; // SEND_HARD_RESET or AUTO_HARDRESET
      if (a == 0x02 && (v & 0x03) != 0x03)
        droppedRd = true;
      r[a] = v;
    }
    return true;
  }
  void pushPacket(const Msg &m) {
    if (rx.size() + 7 + 4 * m.n > 80) {
      overflow = true;
      return;
    }
    rx.push_back(0xE0);
    rx.push_back(m.header & 0xFF);
    rx.push_back(m.header >> 8);
    for (uint8_t k = 0; k < m.n; k++)
      for (int b = 0; b < 4; b++)
        rx.push_back((m.obj[k] >> (8 * b)) & 0xFF);
    for (int b = 0; b < 4; b++)
      rx.push_back(0xA5); // CRC
  }
  bool transmit(const uint8_t *b, size_t len) {
    // SOP1 SOP1 SOP1 SOP2 PACKSYM|N <N bytes> JAM_CRC EOP TXOFF TXON. A hard
    // reset would start with RESET1 (0x15) tokens instead of SOP1.
    if (len && (b[0] == 0x15 || b[0] == 0x16))
      sawHardResetToken = true;
    assert(len >= 11);
    assert(b[0] == 0x12 && b[1] == 0x12 && b[2] == 0x12 && b[3] == 0x13);
    uint8_t n = b[4] & 0x1F;
    assert((b[4] & 0xE0) == 0x80 && n >= 2 && len == 5 + n + 4u);
    assert(b[5 + n] == 0xFF && b[6 + n] == 0x14 && b[7 + n] == 0xFE &&
           b[8 + n] == 0xA1);
    Msg m;
    m.header = b[5] | (b[6] << 8);
    m.n = (m.header >> 12) & 7;
    assert(n == 2 + 4 * m.n);
    for (uint8_t k = 0; k < m.n; k++)
      m.obj[k] = b[7 + 4 * k] | (b[8 + 4 * k] << 8) |
                 ((uint32_t)b[9 + 4 * k] << 16) |
                 ((uint32_t)b[10 + 4 * k] << 24);
    if (collideNext > 0) {
      collideNext--;
      r[0x42] |= 0x02; // COLLISION: not sent
      return true;
    }
    bool ok = cc() && cc() == src->cc && src->receive(m);
    if (ok) {
      Msg g;
      g.header = 1 | (src->rev << 6) | (1 << 8) | (1 << 5) | (m.id() << 9);
      g.n = 0;
      pushPacket(g);   // the peer's GoodCRC lands in the RX FIFO
      r[0x3E] |= 0x04; // TXSENT
    } else {
      r[0x3E] |= 0x10; // RETRYFAIL
    }
    return true;
  }
  // The source puts a message on the line at time t. ackLost: the sink got it
  // but its GoodCRC did not reach the source, which will send it again.
  void fromSource(const Msg &m, bool ackLost = false) {
    activityUntil = t + 1;
    if (cc() && cc() == src->cc) {
      pushPacket(m);
      r[0x3F] |= 0x01; // GCRCSENT
      if (!ackLost)
        src->acked(m);
    } else {
      src->nacked(m);
    }
  }
};

// ---- Simulation
// ------------------------------------------------------------------
struct Sim {
  Source src;
  Chip chip;
  Adafruit_FUSB302Sink sink;
  uint32_t t = 0;
  int pollMs = 3;
  int duplicateAt = -1; // resend the next message with the same ID once

  Sim() : chip(&src), sink(chip) {}
  void run(uint32_t until) {
    for (; t < until; t++) {
      chip.t = src.t = t;
      // source timers
      if (src.pe == Source::OFF && t == 5)
        src.attach(t);
      if (src.pe == Source::HARD_RESET && t >= src.timer) {
        src.attach(t);
      }
      if (src.pd && src.pe == Source::CAPS && t >= src.timer &&
          src.capsSent < 50) {
        src.capsSent++;
        src.sendCaps(0);
        src.timer = t + 150; // tSendSourceCap
      }
      if (src.pe == Source::WAIT_REQ && t >= src.timer)
        src.hardReset(); // no Request in time
      if (src.pe == Source::ACCEPTED && t >= src.timer) {
        src.pe = Source::READY;
        src.vbusMv = src.contractMv;
      }
      while (!src.outbox.empty() && src.outbox.front().first <= t) {
        Msg m = src.outbox.front().second;
        src.outbox.pop_front();
        if (src.pe == Source::HARD_RESET)
          continue;
        // MessageID and revision are fixed when the message goes out
        m.header = (m.header & ~((7 << 9) | (3 << 6))) | (src.txId << 9) |
                   (src.rev << 6);
        if (duplicateAt >= 0 && (uint32_t)duplicateAt <= t) {
          duplicateAt = -1;
          chip.fromSource(m, true); // GoodCRC lost: the same ID comes again
        }
        chip.fromSource(m);
      }
      if (t % pollMs == 0)
        sink.poll(t);
    }
  }
};

static void expect(Sim &s, uint16_t mv, const char *name) {
  printf("%-44s state=%-10s contract=%5u mV  requests=%d softresets=%u\n", name,
         Adafruit_FUSB302Sink::stateName(s.sink.state()),
         s.sink.contractMillivolts(), s.src.requests, s.sink.softResets());
  CHECK(s.src.hardResets == 0);
  CHECK(!s.chip.sawHardResetToken);
  CHECK(!s.chip.droppedRd);
  CHECK(!s.chip.overflow);
  CHECK(s.sink.contractMillivolts() == mv);
  CHECK(s.src.vbusMv == (mv ? mv : 5000));
  if (mv)
    CHECK(s.sink.state() == Adafruit_FUSB302Sink::PD_READY);
  CHECK(s.src.sinkRev == 0 || s.src.sinkRev == 1); // the sink speaks PD 2.0
}

static std::vector<uint32_t> caps(std::initializer_list<uint32_t> mv) {
  std::vector<uint32_t> v;
  for (uint32_t x : mv)
    v.push_back(fixedPdo(x, 3000) | (x == 5000 ? (1UL << 26) : 0));
  return v;
}

int main() {
  for (int poll = 1; poll <= 5; poll += 2) {
    for (uint8_t cc = 1; cc <= 2; cc++) {
      Sim s;
      s.pollMs = poll;
      s.src.cc = cc;
      s.src.pdos = caps({5000, 9000, 12000, 15000, 20000});
      CHECK(s.sink.begin(0));
      s.run(3000);
      char name[64];
      snprintf(name, sizeof(name), "12 V offered, CC%u, poll %d ms", cc, poll);
      expect(s, 12000, name);
      CHECK(s.sink.ccLine() == cc);
      CHECK(((s.src.lastRdo >> 28) & 7) == 3);
      CHECK(((s.src.lastRdo >> 10) & 0x3FF) == 300); // 3 A
    }
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 9000, 15000, 20000});
    s.sink.begin(0);
    s.run(3000);
    expect(s, 5000, "no 12 V PDO: asks for 5 V");
    CHECK(((s.src.lastRdo >> 28) & 7) == 1);
    CHECK(s.src.lastRdo & (1UL << 26)); // capability mismatch
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.src.startInContract = true;
    s.sink.begin(0);
    s.run(4000);
    expect(s, 12000, "rebooted under a 12 V contract");
    CHECK(s.sink.softResets() >= 1);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.src.startInContract = true;
    s.src.startMv = 5000;
    s.sink.begin(0);
    s.run(4000);
    expect(s, 12000, "rebooted under a 5 V contract");
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.src.waitOnce = true;
    s.sink.begin(0);
    s.run(3000);
    expect(s, 12000, "source says Wait once");
    CHECK(s.src.requests == 2);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.src.rejectHigh = true;
    s.sink.begin(0);
    s.run(3000);
    expect(s, 5000, "source rejects 12 V: falls back to 5 V");
    CHECK(s.src.rejects == 1);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.sink.begin(0);
    s.run(2000);
    s.src.queue(0, 8); // Get_Sink_Cap
    s.run(2100);
    s.src.pdos = caps({5000, 9000, 12000});
    s.src.sendCaps(0); // capabilities change: Request due within 27 ms
    s.run(3500);
    expect(s, 12000, "Get_Sink_Cap, then new capabilities");
    CHECK(s.src.sinkCapsReplies == 1);
    CHECK(((s.src.lastRdo >> 28) & 7) == 3);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.duplicateAt = 0; // the first Source_Capabilities arrives twice
    s.sink.begin(0);
    s.run(3000);
    expect(s, 12000, "duplicate message ignored");
    CHECK(s.src.requests == 1);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.chip.collideNext = 1; // the first Request collides
    s.sink.begin(0);
    s.run(3000);
    expect(s, 12000, "Request collides once, resent in time");
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.src.rev = 1; // PD 2.0 source
    s.sink.begin(0);
    s.run(3000);
    expect(s, 12000, "PD 2.0 source");
  }
  {
    Sim s;
    s.src.pd = false;
    s.src.rp = 1; // default USB Rp
    s.sink.begin(0);
    s.run(6000);
    expect(s, 0, "plain 5 V supply, no PD");
    CHECK(s.sink.state() == Adafruit_FUSB302Sink::PD_NO_PD);
  }
  {
    Sim s;
    s.src.pdos = caps({5000, 12000});
    s.sink.begin(0);
    s.run(2000);
    s.src.vbusMv = 0; // unplugged
    s.src.pe = Source::OFF;
    s.run(2100);
    CHECK(s.sink.state() == Adafruit_FUSB302Sink::PD_DETACHED);
    CHECK(s.sink.contractMillivolts() == 0);
    s.src.attach(s.t); // plugged in again
    s.run(4000);
    expect(s, 12000, "unplug and plug in again");
  }
  printf(failures ? "FAILED (%d)\n" : "all FUSB302 sink tests passed\n",
         failures);
  return failures ? 1 : 0;
}
