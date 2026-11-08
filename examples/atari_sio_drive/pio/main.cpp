// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// PlatformIO only: builds the sketch as a plain C++ file. (PlatformIO's own
// .ino conversion preprocesses the board #if chain without evaluating it and
// prints "redefined" warnings; the Arduino IDE and arduino-cli ignore this
// folder.) ui.cpp is included too because PlatformIO builds only src_dir.
#include <Arduino.h>

#include "../atari_sio_drive.ino"
#include "../ui.cpp"
