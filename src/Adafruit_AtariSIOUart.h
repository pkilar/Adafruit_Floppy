/*!
 * @file Adafruit_AtariSIOUart.h
 *
 * Arduino transport for Adafruit_AtariSIO over a HardwareSerial plus a COMMAND
 * input pin.
 */
#ifndef ADAFRUIT_ATARISIOUART_H
#define ADAFRUIT_ATARISIOUART_H

#include "Adafruit_AtariSIO.h"
#include <Arduino.h>

/*!
 * @brief Adafruit_AtariSIOPort over a hardware UART.
 *
 * Wiring: Atari DATA OUT (SIO pin 5) to the UART RX, UART TX to DATA IN (pin
 * 3), COMMAND (pin 7) to the command pin, all level-shifted to 3.3 V. Choosing
 * the UART pins (for example Serial1.setTX()/setRX() on the RP2040 core) is
 * the sketch's job and must be done before begin().
 */
class Adafruit_AtariSIOUart : public Adafruit_AtariSIOPort {
public:
  /*!
   * @brief Create the transport.
   * @param serial UART connected to the SIO data lines.
   * @param commandPin Pin wired to SIO COMMAND (INPUT_PULLUP, active LOW).
   */
  Adafruit_AtariSIOUart(HardwareSerial &serial, int commandPin)
      : serial_(&serial), commandPin_(commandPin), pinReady_(false) {}

  /*! @brief Configure the COMMAND pin. Also done lazily by setBaud(). */
  void begin() {
    if (!pinReady_) {
      pinMode(commandPin_, INPUT_PULLUP);
      pinReady_ = true;
    }
  }

  /*!
   * @brief State of COMMAND.
   * @return true while the pin is low.
   */
  bool commandAsserted() override { return digitalRead(commandPin_) == LOW; }

  /*!
   * @brief Read one byte.
   * @return Byte or -1.
   */
  int read() override { return serial_->read(); }

  /*!
   * @brief Send bytes.
   * @param data Bytes.
   * @param len Count.
   */
  void write(const uint8_t *data, size_t len) override {
    serial_->write(data, len);
  }

  /*! @brief Wait for the transmitter to finish. */
  void flushTx() override { serial_->flush(); }

  /*! @brief Drop unread received bytes. */
  void clearRx() override {
    while (serial_->available())
      serial_->read();
  }

  /*!
   * @brief Restart the UART at a new rate, 8N1.
   * @param baud Bits per second.
   */
  void setBaud(uint32_t baud) override {
    begin();
    serial_->end();
    serial_->begin(baud, SERIAL_8N1);
  }

  /*!
   * @brief Microsecond clock.
   * @return micros().
   */
  uint32_t micros() override { return ::micros(); }

  /*!
   * @brief Delay.
   * @param us Microseconds.
   */
  void delayMicros(uint32_t us) override {
    while (us > 10000) {
      delayMicroseconds(10000);
      us -= 10000;
    }
    delayMicroseconds(us);
  }

private:
  HardwareSerial *serial_; ///< UART
  int commandPin_;         ///< COMMAND input pin
  bool pinReady_;          ///< Pin already configured
};

#endif // ADAFRUIT_ATARISIOUART_H
