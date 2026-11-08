/*!
 * @file Adafruit_AtariSIO.cpp
 *
 * Atari SIO disk-drive protocol engine. Timing constants cite sio2sd (sio.c)
 * and AtariSIO (driver/atarisio.c, tools/AtrSIOHandler.cpp).
 */
#include "Adafruit_AtariSIO.h"
#include <string.h>

namespace {

// Response bytes (sio2sd sio.c; AtariSIO driver/atarisio.c).
const uint8_t kAck = 'A';
const uint8_t kNak = 'N';
const uint8_t kComplete = 'C';
const uint8_t kError = 'E';

// Commands (sio2sd sio.c; AtariSIO tools/AtrSIOHandler.cpp).
const uint8_t kCmdRead = 0x52;
const uint8_t kCmdWrite = 0x57;
const uint8_t kCmdPut = 0x50;
const uint8_t kCmdStatus = 0x53;
const uint8_t kCmdFormat = 0x21;
const uint8_t kCmdFormatEd = 0x22;
const uint8_t kCmdGetPercom = 0x4E;
const uint8_t kCmdPutPercom = 0x4F;
const uint8_t kCmdSpeedIndex = 0x3F;
const uint8_t kXF551Bit = 0x80;

// POKEY clock used for exact baud rates (sio2sd sio.c; AtariSIO
// driver/atarisio.c).
const uint32_t kPokeyHz = 1789773;   // NTSC POKEY clock (3.579545 MHz / 2)
const uint8_t kStandardDivisor = 40; // 19040 baud
const uint8_t kXF551Divisor = 16;    // 38908 baud (AtariSIO driver/atarisio.h)
const uint8_t kDefaultUltraDivisor = 10;

// Sources: AtariSIO driver/atarisio.c, except where marked sio2sd sio.c.
// Timing, microseconds. "min" values are gaps between the end of the previous
// stop bit (or COMMAND rising) and the start bit of ours.
const uint32_t kAckDelayUs =
    150; // COMMAND high -> ACK/NAK: min 100 (AtariSIO driver/atarisio.c)
const uint32_t kAckToCompleteUs = 300; // ACK -> COMPLETE: min 300 (atarisio.c)
const uint32_t kAckToCompleteSlowUs = 1000; // XF551 or error (atarisio.c)
const uint32_t kCompleteToDataUs = 150; // COMPLETE -> data frame (atarisio.c)
const uint32_t kDataToAckUs = 450; // Atari data frame -> our ACK/NAK (AtariSIO)
const uint32_t kByteTimeoutUs = 17000; // wait for Atari bytes (sio2sd sio.c)
const uint32_t kCommandRiseUs =
    1500; // COMMAND must rise after frame (sio2sd sio.c, 1 ms + margin)
// The Atari waits t0 >= 750 us between asserting COMMAND and the first frame
// byte. If we saw COMMAND high less than this long ago, nothing in the RX
// buffer can belong to the new frame, so it is safe to clear.
const uint32_t kStaleRxWindowUs = 300;

// Drive status byte 0 bits (sio2sd sio.c; AtariSIO tools/AtrSIOHandler.cpp).
const uint8_t kStatusMotor = 0x10;
const uint8_t kStatusDD = 0x20; // any sector size other than 128 (AtariSIO)
const uint8_t kStatusDS = 0x40; // XF551 double-sided (Altirra HRM, table 79)
const uint8_t kStatusED = 0x80;
const uint8_t kStatusWriteProt = 0x08;
const uint8_t kStatusDataBad = 0x02;
const uint8_t kStatusFdcError = 0x04;
const uint8_t kFormatTimeout = 0xE0;   // status byte 2, 810/1050 (sio2sd sio.c)
const uint8_t kFormatTimeoutXF = 0xFE; // XF551

// Inverted WD279x status for status byte 1 (sio2sd sio.c; AtariSIO
// tools/AtrSIOHandler.cpp). The per-result values come from
// atari_fdc_status_for(): write protect gives $BF and not ready $7F as on real
// drives. I/O failures give ~(LOST_DATA | NOT_FOUND) = $EB; AtariSIO's
// AtrSIOHandler.cpp uses $B0 for a failed write (under a "write protected"
// comment) and $EF for a failed read, with no documented meaning for $B0.
const uint8_t kFdcOk = 0xFF;
const uint8_t kFdcNotReady = 0x7F;

// Longest single sleep while waiting, so COMMAND is noticed during the
// multi-second delays of protected-disk emulation.
const uint32_t kWaitSliceUs = 1000;

} // namespace

uint32_t Adafruit_AtariSIO::pokeyBaud(uint8_t divisor) {
  return kPokeyHz / (2u * ((uint32_t)divisor + 7u));
}

uint8_t Adafruit_AtariSIO::checksum(const uint8_t *data, size_t len) {
  // 8-bit sum with end-around carry (sio2sd sio.c; AtariSIO driver/atarisio.c).
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
    if (sum >= 0x100)
      sum = (uint16_t)((sum & 0xFF) + 1);
  }
  return (uint8_t)sum;
}

Adafruit_AtariSIO::Adafruit_AtariSIO(Adafruit_AtariSIOPort *port)
    : port_(port), ultraDivisor_(kDefaultUltraDivisor), xf551_(true),
      highBaud_(false), keepRx_(false), baud_(0), frameBaud_(0), tLast_(0),
      tAck_(0), tIdle_(0), hs_(false) {
  memset(units_, 0, sizeof(units_));
  memset(&stats_, 0, sizeof(stats_));
  memset(buf_, 0, sizeof(buf_));
}

void Adafruit_AtariSIO::begin() {
  highBaud_ = false;
  keepRx_ = false;
  stats_.highSpeed = false;
  baud_ = pokeyBaud(kStandardDivisor);
  port_->setBaud(baud_);
  port_->clearRx();
}

bool Adafruit_AtariSIO::attach(uint8_t unit, Adafruit_AtariDisk *disk) {
  if (unit < 1 || unit > MAX_UNITS || !disk)
    return false;
  Unit &u = units_[unit - 1];
  memset(&u, 0, sizeof(u));
  u.disk = disk;
  u.hwStatus = kFdcOk;
  return true;
}

void Adafruit_AtariSIO::detach(uint8_t unit) {
  if (unit >= 1 && unit <= MAX_UNITS)
    memset(&units_[unit - 1], 0, sizeof(Unit));
}

void Adafruit_AtariSIO::setUltraSpeed(uint8_t pokey_divisor) {
  ultraDivisor_ = pokey_divisor;
  if (highBaud_) {
    if (pokey_divisor == 0) {
      highBaud_ = false;
      stats_.highSpeed = false;
      setBaud(pokeyBaud(kStandardDivisor));
    } else {
      setBaud(pokeyBaud(pokey_divisor));
    }
  }
}

void Adafruit_AtariSIO::setXF551HighSpeed(bool enable) { xf551_ = enable; }

void Adafruit_AtariSIO::setBaud(uint32_t baud) {
  if (baud != baud_) {
    port_->setBaud(baud);
    baud_ = baud;
  }
}

void Adafruit_AtariSIO::toggleBaud() {
  // Ultra Speed autobaud (AtariSIO driver/atarisio.c): a bad command frame may
  // simply mean the Atari changed speed, so try the other rate for the retry.
  if (ultraDivisor_ == 0)
    return;
  highBaud_ = !highBaud_;
  stats_.highSpeed = highBaud_;
  setBaud(pokeyBaud(highBaud_ ? ultraDivisor_ : kStandardDivisor));
}

// ---------------------------------------------------------------- bus helpers

bool Adafruit_AtariSIO::cmdActive() {
  if (port_->commandAsserted()) {
    keepRx_ = true; // the new frame may already be arriving
    return true;
  }
  return false;
}

// Wait until us microseconds have passed since `since`; false if COMMAND
// asserted meanwhile.
bool Adafruit_AtariSIO::waitSince(uint32_t since, uint32_t us) {
  for (;;) {
    uint32_t elapsed = port_->micros() - since;
    if (elapsed >= us)
      break;
    uint32_t left = us - elapsed;
    port_->delayMicros(left < kWaitSliceUs ? left : kWaitSliceUs);
    if (left > kWaitSliceUs && cmdActive())
      return false;
  }
  return !cmdActive();
}

bool Adafruit_AtariSIO::sendByte(uint8_t b) {
  if (cmdActive())
    return false;
  port_->write(&b, 1);
  port_->flushTx();
  tLast_ = port_->micros();
  stats_.lastResult = b;
  return true;
}

bool Adafruit_AtariSIO::sendAck() {
  if (!waitSince(tLast_, kAckDelayUs) || !sendByte(kAck))
    return false;
  tAck_ = tLast_;
  if (hs_)
    setBaud(pokeyBaud(kXF551Divisor)); // XF551: rest of command at high speed
  return true;
}

bool Adafruit_AtariSIO::sendNak() {
  if (!waitSince(tLast_, kAckDelayUs) || !sendByte(kNak))
    return false;
  stats_.naks++;
  return true;
}

bool Adafruit_AtariSIO::sendComplete(bool ok, uint32_t extra_us) {
  uint32_t gap = (hs_ || !ok) ? kAckToCompleteSlowUs : kAckToCompleteUs;
  if (extra_us > gap)
    gap = extra_us;
  if (!waitSince(tAck_, gap) || !sendByte(ok ? kComplete : kError))
    return false;
  if (!ok)
    stats_.errors++;
  return true;
}

bool Adafruit_AtariSIO::sendDataFrame(uint16_t len) {
  if (cmdActive())
    return false;
  buf_[len] = checksum(buf_, len);
  port_->write(buf_, (size_t)len + 1);
  port_->flushTx();
  tLast_ = port_->micros();
  return true;
}

// COMPLETE (or ERROR) followed by the data frame held in buf_. After an error
// a zeroed full-length frame still follows (AtariSIO tools/AtrSIOHandler.cpp),
// unless the disk asked for its own (bad) data to be sent, as real drives do
// for copy-protected sectors.
bool Adafruit_AtariSIO::replyWithData(bool ok, uint16_t len, uint32_t extra_us,
                                      bool keep_data) {
  if (!ok && !keep_data)
    memset(buf_, 0, len);
  return sendComplete(ok, extra_us) && waitSince(tLast_, kCompleteToDataUs) &&
         sendDataFrame(len);
}

Adafruit_AtariSIO::RxResult Adafruit_AtariSIO::receiveDataFrame(uint16_t len) {
  size_t want = (size_t)len + 1;
  size_t n = 0;
  uint32_t t0 = port_->micros();
  while (n < want) {
    if (cmdActive())
      return RX_ABORT;
    int c = port_->read();
    if (c >= 0) {
      buf_[n++] = (uint8_t)c;
      t0 = port_->micros();
    } else if (port_->micros() - t0 > kByteTimeoutUs) {
      return RX_TIMEOUT;
    }
  }
  tLast_ = port_->micros();
  return checksum(buf_, len) == buf_[len] ? RX_OK : RX_BAD_SUM;
}

// Receive an Atari data frame and answer it with ACK (good) or NAK (bad).
// Returns true only when the frame was good and ACKed.
bool Adafruit_AtariSIO::acceptDataFrame(Unit &u, uint16_t len) {
  RxResult r = receiveDataFrame(len);
  if (r == RX_ABORT || r == RX_TIMEOUT)
    return false;
  if (r == RX_BAD_SUM) {
    stats_.badChecksums++;
    u.dataFrameBad = true;
    if (waitSince(tLast_, kDataToAckUs) && sendByte(kNak))
      stats_.naks++;
    return false;
  }
  u.dataFrameBad = false;
  if (!waitSince(tLast_, kDataToAckUs))
    return false;
  return sendAck();
}

// -------------------------------------------------------------- command frame

bool Adafruit_AtariSIO::receiveCommand(uint8_t *f) {
  uint32_t t0 = port_->micros();
  size_t n = 0;
  while (n < 5) {
    int c = port_->read();
    if (c >= 0) {
      f[n++] = (uint8_t)c;
      continue;
    }
    if (!port_->commandAsserted()) {
      c = port_->read(); // late byte
      if (c < 0)
        break;
      f[n++] = (uint8_t)c;
      continue;
    }
    if (port_->micros() - t0 > kByteTimeoutUs)
      break;
  }
  bool good = (n == 5);
  if (good) {
    // COMMAND must rise promptly, with no extra characters (wrong baud
    // typically yields too many or too few characters; AtariSIO
    // driver/atarisio.c).
    uint32_t t1 = port_->micros();
    while (port_->commandAsserted()) {
      if (port_->read() >= 0)
        good = false;
      if (port_->micros() - t1 > kCommandRiseUs) {
        good = false;
        break;
      }
    }
    if (port_->read() >= 0)
      good = false;
  }
  if (!good) {
    stats_.badFrames++;
    toggleBaud();
    return false;
  }
  tLast_ = port_->micros(); // COMMAND went high
  if (checksum(f, 4) != f[4]) {
    stats_.badChecksums++;
    toggleBaud();
    return false;
  }
  return true;
}

void Adafruit_AtariSIO::poll() {
  if (!port_->commandAsserted()) {
    // Bytes seen while COMMAND is high are other devices' traffic. Drop them
    // here, so a late poll() never has to clear bytes of a frame in progress.
    port_->clearRx();
    tIdle_ = port_->micros();
    keepRx_ = false;
    return;
  }
  if (!keepRx_ && port_->micros() - tIdle_ < kStaleRxWindowUs)
    port_->clearRx();
  keepRx_ = false;

  uint8_t frame[5];
  if (!receiveCommand(frame))
    return;
  frameBaud_ = baud_;
  handleCommand(frame);
  setBaud(frameBaud_); // undo any XF551 speed change
}

void Adafruit_AtariSIO::handleCommand(const uint8_t *f) {
  uint8_t unit = (uint8_t)(f[0] - 0x30);
  if (f[0] < 0x31 || unit > MAX_UNITS || !units_[unit - 1].disk)
    return; // not ours: stay silent
  Unit &u = units_[unit - 1];
  uint16_t aux = (uint16_t)(f[2] | (f[3] << 8));
  stats_.frames++;
  stats_.lastCommand = f[1];
  stats_.lastUnit = unit;
  stats_.lastAux = aux;

  uint8_t cmd = f[1];
  hs_ = false;
  if (cmd & kXF551Bit) {
    cmd &= (uint8_t)~kXF551Bit;
    switch (cmd) {
    case kCmdRead:
    case kCmdWrite:
    case kCmdPut:
    case kCmdStatus:
    case kCmdGetPercom:
    case kCmdPutPercom:
      hs_ = true; // XF551: 0xD2 0xD7 0xD0 0xD3 0xCE 0xCF
      break;
    // Format commands run at standard speed (AtariSIO tools/AtrSIOHandler.cpp).
    case kCmdFormat:
    case kCmdFormatEd: // 0xA1 0xA2 run at standard speed
      break;
    default:
      cmd = 0; // unsupported
      break;
    }
    if (!xf551_)
      cmd = 0;
  }
  dispatch(u, cmd, aux, hs_);
}

void Adafruit_AtariSIO::dispatch(Unit &u, uint8_t cmd, uint16_t aux, bool hs) {
  switch (cmd) {
  case kCmdRead:
    cmdRead(u, aux);
    break;
  case kCmdWrite:
    cmdWrite(u, aux, true);
    break;
  case kCmdPut:
    cmdWrite(u, aux, false);
    break;
  case kCmdStatus:
    cmdStatus(u, hs);
    break;
  case kCmdFormat:
    cmdFormat(u, false);
    break;
  case kCmdFormatEd:
    cmdFormat(u, true);
    break;
  case kCmdGetPercom:
    cmdGetPercom(u);
    break;
  case kCmdPutPercom:
    cmdPutPercom(u);
    break;
  case kCmdSpeedIndex:
    cmdSpeedIndex();
    break;
  default:
    sendNak();
    break;
  }
}

// ------------------------------------------------------------------- helpers

atari_geometry_t Adafruit_AtariSIO::diskGeometry(const Unit &u) const {
  if (u.disk->ready())
    return u.disk->geometry();
  return atari_geometry_for_density(ATARI_DENSITY_SD);
}

atari_geometry_t Adafruit_AtariSIO::effectiveGeometry(const Unit &u) const {
  return u.hasPending ? u.pending : diskGeometry(u);
}

bool Adafruit_AtariSIO::noteResult(Unit &u, atari_disk_result_t r) {
  u.hwStatus = (uint8_t)~atari_fdc_status_for(r);
  return r == ATARI_DISK_OK;
}

// ------------------------------------------------------------------ commands

void Adafruit_AtariSIO::cmdRead(Unit &u, uint16_t sector) {
  atari_geometry_t g = diskGeometry(u);
  uint16_t len = atari_sector_bytes(&g, sector);
  if (len == 0) {
    // Source: sio2sd sio.c; AtariSIO tools/AtrSIOHandler.cpp.
    sendNak(); // out of range: NAK before ACK
    return;
  }
  if (!sendAck())
    return;
  atari_read_info_t info;
  memset(&info, 0, sizeof(info));
  atari_disk_result_t r = u.disk->readSectorTimed(sector, buf_, tAck_, &info);
  bool ok = r == ATARI_DISK_OK;
  uint8_t fdc = info.fdc_status;
  if (!ok && fdc == 0)
    fdc = atari_fdc_status_for(r); // disk reported failure without a status
  u.hwStatus = (uint8_t)~fdc;
  replyWithData(ok, len, info.delay_us, info.send_data);
}

void Adafruit_AtariSIO::cmdWrite(Unit &u, uint16_t sector, bool verify) {
  atari_geometry_t g = diskGeometry(u);
  uint16_t len = atari_sector_bytes(&g, sector);
  if (len == 0) {
    sendNak();
    return;
  }
  if (!sendAck() || !acceptDataFrame(u, len))
    return;
  atari_disk_result_t r;
  if (u.disk->writeProtected())
    r = ATARI_DISK_WRITE_PROTECTED;
  else
    r = u.disk->writeSector(sector, buf_, verify);
  bool ok = noteResult(u, r);
  if (ok)
    u.disk->flush();
  sendComplete(ok);
}

void Adafruit_AtariSIO::cmdStatus(Unit &u, bool xf) {
  if (!sendAck())
    return;
  atari_geometry_t g = effectiveGeometry(u);
  uint8_t hw = u.disk->ready() ? u.hwStatus : kFdcNotReady;
  uint8_t s = kStatusMotor;
  if (g.sector_size != 128)
    s |= kStatusDD;
  if (g.sides == 2)
    s |= kStatusDS;
  if (atari_density_of(&g) == ATARI_DENSITY_ED)
    s |= kStatusED;
  if (u.disk->writeProtected())
    s |= kStatusWriteProt;
  if (u.dataFrameBad)
    s |= kStatusDataBad;
  if (hw != kFdcOk)
    s |= kStatusFdcError;
  buf_[0] = s;
  buf_[1] = hw;
  buf_[2] = xf ? kFormatTimeoutXF : kFormatTimeout;
  buf_[3] = 0;
  replyWithData(true, 4);
}

void Adafruit_AtariSIO::cmdFormat(Unit &u, bool enhanced) {
  atari_geometry_t g = enhanced ? atari_geometry_for_density(ATARI_DENSITY_ED)
                                : effectiveGeometry(u);
  uint16_t len = g.sector_size; // AtariSIO: buflen = sector length
  if (!sendAck())
    return;
  atari_disk_result_t r;
  if (u.disk->writeProtected())
    r = ATARI_DISK_WRITE_PROTECTED;
  else
    r = u.disk->format(g);
  bool ok = noteResult(u, r);
  // A PERCOM write only applies to the format that follows it. Keeping it
  // after a failure would make status report a density the disk lacks.
  u.hasPending = false;
  if (ok) {
    u.disk->flush();
    // Source: sio2sd sio.c; AtariSIO tools/AtrSIOHandler.cpp.
    memset(buf_, 0xFF, len); // no bad sectors
  }
  replyWithData(ok, len);
}

void Adafruit_AtariSIO::cmdGetPercom(Unit &u) {
  if (!sendAck())
    return;
  atari_geometry_t g = effectiveGeometry(u);
  uint16_t tracks = g.tracks, spt = g.sectors_per_track, sides = g.sides;
  bool floppy = tracks > 0 && spt > 0 && sides > 0 &&
                (uint32_t)tracks * sides * spt == g.sector_count;
  // PERCOM layout: sio2sd sio.c; AtariSIO tools/AtrSIOHandler.cpp.
  if (!floppy) { // hard-disk style image
    tracks = 1;
    spt = g.sector_count;
    sides = 1;
  }
  buf_[0] = (uint8_t)tracks;
  buf_[1] = 0;
  buf_[2] = (uint8_t)(spt >> 8);
  buf_[3] = (uint8_t)spt;
  buf_[4] = (uint8_t)(sides - 1);
  buf_[5] = (!g.mfm && g.sector_size == 128) ? 0 : 4;
  buf_[6] = (uint8_t)(g.sector_size >> 8);
  buf_[7] = (uint8_t)g.sector_size;
  buf_[8] = 0xFF;
  buf_[9] = buf_[10] = buf_[11] = 0;
  replyWithData(true, 12);
}

void Adafruit_AtariSIO::cmdPutPercom(Unit &u) {
  if (!sendAck() || !acceptDataFrame(u, 12))
    return;
  atari_geometry_t g;
  g.tracks = buf_[0];
  g.sectors_per_track = (uint16_t)((buf_[2] << 8) | buf_[3]);
  g.sides = (uint8_t)(buf_[4] + 1);
  g.sector_size = (uint16_t)((buf_[6] << 8) | buf_[7]);
  g.mfm = g.sector_size >= 256 || (buf_[5] & 4);
  uint32_t total = (uint32_t)g.tracks * g.sides * g.sectors_per_track;
  g.sector_count = (uint16_t)total;
  bool ok =
      (g.sector_size == 128 || g.sector_size == 256 || g.sector_size == 512) &&
      buf_[4] <= 1 && total >= 1 && total <= 65535 && u.disk->canFormat(g);
  if (ok) {
    u.pending = g;
    u.hasPending = true;
  }
  sendComplete(ok);
}

void Adafruit_AtariSIO::cmdSpeedIndex() {
  if (ultraDivisor_ == 0) {
    sendNak();
    return;
  }
  if (!sendAck())
    return;
  buf_[0] = ultraDivisor_;
  replyWithData(true, 1);
}
