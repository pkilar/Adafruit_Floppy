/*!
 * @file Adafruit_FUSB302Wire.h
 *
 * Arduino I2C transport for Adafruit_FUSB302Sink.
 */
#ifndef ADAFRUIT_FUSB302WIRE_H
#define ADAFRUIT_FUSB302WIRE_H

#include "Adafruit_FUSB302Sink.h"
#include <Arduino.h>
#include <Wire.h>

/*!
 * @brief Adafruit_FUSB302Bus over a TwoWire bus.
 *
 * Pin selection, clock (up to 1 MHz on the FUSB302, 400 kHz is plenty) and
 * begin() are the sketch's job. If Adafruit_FUSB302Sink::poll() runs from an
 * interrupt, nothing else may use this bus.
 */
class Adafruit_FUSB302Wire : public Adafruit_FUSB302Bus {
public:
  /*!
   * @brief Create the transport.
   * @param wire I2C bus the FUSB302 is on.
   * @param address 7-bit address: 0x22 for the FUSB302BMPX (B01: 0x23, B10:
   *        0x24, B11: 0x25).
   */
  Adafruit_FUSB302Wire(TwoWire &wire, uint8_t address = 0x22)
      : wire_(&wire), addr_(address) {}

  /*!
   * @brief Read registers (repeated start between address and data).
   * @param reg First register.
   * @param buf Destination.
   * @param len Byte count.
   * @return true on success.
   */
  bool read(uint8_t reg, uint8_t *buf, size_t len) override {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0)
      return false;
    if ((size_t)wire_->requestFrom(addr_, len, true) != len)
      return false;
    for (size_t i = 0; i < len; i++)
      buf[i] = wire_->read();
    return true;
  }

  /*!
   * @brief Write registers in one transaction (a whole packet into the TX
   *        FIFO, so the transmitter never runs dry).
   * @param reg First register.
   * @param buf Bytes.
   * @param len Byte count.
   * @return true on success.
   */
  bool write(uint8_t reg, const uint8_t *buf, size_t len) override {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(buf, len);
    return wire_->endTransmission() == 0;
  }

private:
  TwoWire *wire_;
  uint8_t addr_;
};

#endif // ADAFRUIT_FUSB302WIRE_H
