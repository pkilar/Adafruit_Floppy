// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// USB Power Delivery on the RP2350B board: a FUSB302 on I2C1 asks the charger
// on the PD port for 12 V (for the drive's 12 V pin) and stays at 5 V when the
// charger has no 12 V. Built only for boards that define PD_SDA_PIN.
#ifndef ATARI_SIO_USB_PD_H
#define ATARI_SIO_USB_PD_H

#include "ui_config.h"

#if defined(PD_SDA_PIN)
#include <Arduino.h>

/** Start the PD sink; call from setup1() so it polls on core 1. */
void pdSetup1();
/** Short power status for the display, e.g. "PD 12V", "5V", "no PD port". */
void pdShortStatus(char *buf, size_t len);
/** Print the negotiation state and the charger's offers. */
void pdPrint(Print &out);
#endif

#endif // ATARI_SIO_USB_PD_H
