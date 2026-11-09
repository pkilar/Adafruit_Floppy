// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// Board selection and the pins of the local user interface (OLED and rotary
// encoder). Included by the sketch and by ui.cpp; a board without OLED_SDA_PIN
// builds without the user interface.
#ifndef ATARI_SIO_UI_CONFIG_H
#define ATARI_SIO_UI_CONFIG_H

// The custom RP2350B board (floppy-rp2350b-pcb). Select it either by choosing
// the "Generic RP2350" board in the Arduino IDE (which defines
// ARDUINO_GENERIC_RP2350; also pick RP2350B and PSRAM CS GPIO 47), or by
// building with -DFLOPPY_SIO_RP2350B_REVA for another board package.
#if defined(FLOPPY_SIO_RP2350B_REVA) || defined(ARDUINO_GENERIC_RP2350)
#define BOARD_FLOPPY_SIO_RP2350B
#endif

#if defined(BOARD_FLOPPY_SIO_RP2350B)
// Display: exactly one of these, chosen with OLED_TYPE (or -DOLED_TYPE=...).
//   OLED_SH1106_I2C    1.3" SH1106 128x64 on I2C0 (the default)
//   OLED_SH1106_SPI    SH1106 128x64 on the 7-pin SPI socket
//   OLED_SSD1306_SPI   SSD1306 128x64 on the SPI socket
//   OLED_SSD1309_SPI   SSD1309 128x64 (2.42" modules) on the SPI socket;
//                      driven by Adafruit_SSD1306, which uses a compatible
//                      command set (not verified on a real module)
#define OLED_SH1106_I2C 1
#define OLED_SH1106_SPI 2
#define OLED_SSD1306_SPI 3
#define OLED_SSD1309_SPI 4
#ifndef OLED_TYPE
#define OLED_TYPE OLED_SH1106_I2C
#endif
// I2C0, address 0x3C
#define OLED_SDA_PIN 44
#define OLED_SCL_PIN 45
#define OLED_I2C_ADDR 0x3C
// SPI1 (software chip select)
#define OLED_SPI_SCK_PIN 42
#define OLED_SPI_MOSI_PIN 43
#define OLED_SPI_CS_PIN 31
#define OLED_SPI_DC_PIN 30
#define OLED_SPI_RES_PIN 29
#define OLED_SPI_HZ 8000000
// EC11 rotary encoder; A, B and the switch are active low with external
// pull-ups.
#define ENCODER_A_PIN 40
#define ENCODER_B_PIN 39
#define ENCODER_SW_PIN 41
// VBUS_PD through 100k/27k, 12 V present when above about 1.9 V
#define VBUS_SENSE_PIN 46
#define STATUS_LED_PIN 21
#endif

#endif // ATARI_SIO_UI_CONFIG_H
