// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// Local user interface (OLED + encoder): the data shared between core 0 (SIO
// engine, SD card, mounts, console) and core 1 (display and encoder).
//
// Rules: core 1 never touches SdFat, the mounts or the floppy. It posts one
// request at a time (UiRequest) and reads a status snapshot that core 0
// publishes. All shared data sits behind one mutex held only for short
// copies, so neither core waits long.
#ifndef ATARI_SIO_UI_H
#define ATARI_SIO_UI_H

#include "ui_config.h"
#include <stdint.h>

#if defined(OLED_SDA_PIN)

#define UI_MAX_UNITS 15    // D1..D15, same as MAX_UNITS in the sketch
#define UI_PATH_LEN 96     // same as PATH_LEN in the sketch
#define UI_NAME_LEN 64     // longest file name the browser shows
#define UI_MSG_LEN 64      // result text for the display
#define UI_MAX_ENTRIES 160 // entries of one folder

enum UiSource : uint8_t {
  UI_SRC_NONE,
  UI_SRC_IMAGE,
  UI_SRC_ATX,
  UI_SRC_FLOPPY
};

// Same order as Mode in the sketch.
enum UiMode : uint8_t { UI_MODE_VIRTUAL, UI_MODE_PHYSICAL, UI_MODE_BYPASS };

struct UiUnitInfo {
  uint8_t source; // UiSource
  uint8_t mode;   // UiMode
  char name[24];  // file name without the folder, cut off
};

struct UiStatus {
  uint8_t unitCount; // drives emulated or bypassed, 1..UI_MAX_UNITS
  UiUnitInfo unit[UI_MAX_UNITS];
  // SIO
  uint32_t frames;     // valid command frames so far
  uint8_t lastCommand; // character
  uint8_t lastUnit;    // 1..15
  uint16_t lastAux;
  uint8_t lastResult; // character
  // Physical floppy
  bool floppyMounted;
  bool floppyReady; // a disk was recognized
  bool floppyMotor;
  bool floppyActive; // the drive was used in the last fraction of a second
  uint8_t density;   // 0 SD, 1 ED, 2 DD, 3 other
  uint8_t sides;
  uint8_t tracks;
  uint8_t cylinder; // 0xFF unknown
  uint8_t head;
  // Core 0 is inside a long command (dump, write, format, floppy mount)
  bool busy;
  char busyText[24];
};

enum UiRequestKind : uint8_t {
  UI_REQ_NONE,
  UI_REQ_LIST,      // list folder `path`
  UI_REQ_MOUNT,     // mount image `path` on unit
  UI_REQ_UMOUNT,    // detach unit
  UI_REQ_SET_MODE,  // set unit to the UiMode in `arg`
  UI_REQ_SET_UNITS, // set the number of drives to `arg`
  UI_REQ_SAVE       // write the config file
};

struct UiRequest {
  uint8_t kind;
  uint8_t unit; // 0..14
  uint8_t arg;  // mode or drive count
  uint32_t id;
  char path[UI_PATH_LEN];
};

struct UiDirEntry {
  char name[UI_NAME_LEN];
  bool isDir;
};

// ---- Core 0 side ---------------------------------------------------------

/** Publish a new status snapshot. */
void uiPublishStatus(const UiStatus &s);
/** Take the pending request, if any; true if r was filled. */
bool uiTakeRequest(UiRequest *r);
/** Finish a mount, umount, mode, drive count or save request; msg is shown on
 * the display. */
void uiRequestDone(uint32_t id, bool ok, const char *msg);
/** Add one entry of the folder being listed; false if the list is full. */
bool uiListAdd(uint32_t id, const char *name, bool isDir);
/** The listing is complete (ok) or failed. */
void uiListEnd(uint32_t id, bool ok, const char *msg);

// ---- Core 1 side (call from setup1() / loop1()) ---------------------------

void uiSetup1();
void uiLoop1();

#endif // OLED_SDA_PIN
#endif // ATARI_SIO_UI_H
