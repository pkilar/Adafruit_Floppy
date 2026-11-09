// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// OLED + rotary encoder user interface. Everything from uiSetup1() on runs on
// core 1; the functions declared "Core 0 side" in ui.h run on core 0 and only
// copy data under g_lock. See ui.h for the protocol.
#include "ui.h"

#if defined(OLED_SDA_PIN)

#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#if OLED_TYPE == OLED_SH1106_I2C || OLED_TYPE == OLED_SH1106_SPI
#include <Adafruit_SH110X.h>
#else
#include <Adafruit_SSD1306.h>
#endif
#include <pico/mutex.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "usb_pd.h"

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

// Quadrature transitions the encoder makes per detent. EC11 types have 4 (the
// resting position is both contacts open); some have 2. One step is reported
// per detent.
static const int8_t ENC_TRANSITIONS_PER_DETENT = 4;
// Set to true if turning clockwise should scroll up.
static const bool ENC_REVERSE = false;

static const uint32_t DEBOUNCE_MS = 20;
static const uint32_t LONG_PRESS_MS = 600;
static const uint32_t STATUS_COPY_MS = 50;
static const uint32_t STATUS_REDRAW_MS = 150;
static const uint32_t ACTIVITY_MS = 400;
static const uint32_t MESSAGE_MS = 4000;
static const uint32_t IDLE_BACK_MS = 60000; // menus return to the status screen

// ---------------------------------------------------------------------------
// Shared state (g_lock protects everything in this block)
// ---------------------------------------------------------------------------

auto_init_mutex(g_lock);

namespace {
struct Lock {
  Lock() { mutex_enter_blocking(&g_lock); }
  ~Lock() { mutex_exit(&g_lock); }
};

enum ListState : uint8_t { LIST_IDLE, LIST_RUNNING, LIST_DONE, LIST_ERROR };

UiStatus g_status;
UiRequest g_req;
bool g_reqPending = false; // posted, core 0 has not taken it yet
bool g_inFlight = false;   // posted and not finished
uint32_t g_nextId = 0;
uint32_t g_resultId = 0;
bool g_resultOk = false;
char g_resultMsg[UI_MSG_LEN];

struct {
  uint32_t id;
  ListState state;
  uint16_t count;
  bool truncated;
  char msg[UI_MSG_LEN];
  UiDirEntry entry[UI_MAX_ENTRIES];
} g_list;
} // namespace

// ---- Core 0 side ----------------------------------------------------------

void uiPublishStatus(const UiStatus &s) {
  Lock l;
  g_status = s;
}

bool uiTakeRequest(UiRequest *r) {
  Lock l;
  if (!g_reqPending)
    return false;
  *r = g_req;
  g_reqPending = false;
  if (r->kind == UI_REQ_LIST) {
    g_list.id = r->id;
    g_list.state = LIST_RUNNING;
    g_list.count = 0;
    g_list.truncated = false;
  }
  return true;
}

void uiRequestDone(uint32_t id, bool ok, const char *msg) {
  Lock l;
  g_resultId = id;
  g_resultOk = ok;
  snprintf(g_resultMsg, sizeof(g_resultMsg), "%s", msg);
  g_inFlight = false;
}

bool uiListAdd(uint32_t id, const char *name, bool isDir) {
  Lock l;
  if (g_list.id != id || g_list.state != LIST_RUNNING)
    return false;
  if (g_list.count >= UI_MAX_ENTRIES) {
    g_list.truncated = true;
    return false;
  }
  UiDirEntry &e = g_list.entry[g_list.count++];
  snprintf(e.name, sizeof(e.name), "%s", name);
  e.isDir = isDir;
  return true;
}

void uiListEnd(uint32_t id, bool ok, const char *msg) {
  Lock l;
  if (g_list.id == id) {
    g_list.state = ok ? LIST_DONE : LIST_ERROR;
    snprintf(g_list.msg, sizeof(g_list.msg), "%s", msg);
  }
  g_inFlight = false;
}

// ---------------------------------------------------------------------------
// Core 1: encoder
// ---------------------------------------------------------------------------

// Change in position for (previous state << 2) | state, states being
// A << 1 | B. Impossible jumps (a missed edge) count as 0.
static const int8_t QUAD_DELTA[16] = {0,  +1, -1, 0,  -1, 0,  0,  +1,
                                      +1, 0,  0,  -1, 0,  -1, +1, 0};

static volatile int32_t g_encSteps = 0;
static volatile uint8_t g_encState = 3;
static int8_t g_encAcc = 0;

static void encoderIsr() {
  uint8_t s = (digitalRead(ENCODER_A_PIN) ? 2 : 0) |
              (digitalRead(ENCODER_B_PIN) ? 1 : 0);
  uint8_t prev = g_encState;
  g_encState = s;
  g_encAcc += QUAD_DELTA[(prev << 2) | s];
  if (g_encAcc >= ENC_TRANSITIONS_PER_DETENT) {
    g_encSteps = g_encSteps + 1;
    g_encAcc = 0;
  } else if (g_encAcc <= -ENC_TRANSITIONS_PER_DETENT) {
    g_encSteps = g_encSteps - 1;
    g_encAcc = 0;
  } else if (s == 3) {
    g_encAcc = 0; // back at rest without completing a detent
  }
}

static int takeSteps() {
  noInterrupts();
  int32_t v = g_encSteps;
  g_encSteps = 0;
  interrupts();
  return ENC_REVERSE ? -(int)v : (int)v;
}

enum Event : uint8_t { EVT_NONE, EVT_SHORT, EVT_LONG };

// Debounced switch: a short press is reported on release, a long press as soon
// as LONG_PRESS_MS have passed.
static Event pollSwitch(uint32_t now) {
  static bool stable = false, cand = false, longFired = false;
  static uint32_t tChange = 0, tPress = 0;
  bool raw = digitalRead(ENCODER_SW_PIN) == LOW;
  if (raw != cand) {
    cand = raw;
    tChange = now;
  } else if (cand != stable && now - tChange >= DEBOUNCE_MS) {
    stable = cand;
    if (stable) {
      tPress = now;
      longFired = false;
    } else if (!longFired) {
      return EVT_SHORT;
    }
  }
  if (stable && !longFired && now - tPress >= LONG_PRESS_MS) {
    longFired = true;
    return EVT_LONG;
  }
  return EVT_NONE;
}

// ---------------------------------------------------------------------------
// Core 1: UI state
// ---------------------------------------------------------------------------

namespace {

enum Screen : uint8_t {
  S_STATUS,
  S_MENU,
  S_BROWSE,
  S_ACTION,
  S_UNIT,
  S_DRIVES,
  S_WAIT,
  S_MSG
};

// The drawing code only uses Adafruit_GFX (through `gfx`); the few calls that
// differ between the driver classes are in this block.
#define WHITE_PIXEL 1
#define BLACK_PIXEL 0
#if OLED_TYPE == OLED_SH1106_I2C
Adafruit_SH1106G drv(128, 64, &Wire, -1);
bool dispBegin() { return drv.begin(OLED_I2C_ADDR, true); }
#elif OLED_TYPE == OLED_SH1106_SPI
Adafruit_SH1106G drv(128, 64, &SPI1, OLED_SPI_DC_PIN, OLED_SPI_RES_PIN,
                     OLED_SPI_CS_PIN, OLED_SPI_HZ);
bool dispBegin() { return drv.begin(0, true); }
#else // SSD1306 and SSD1309
Adafruit_SSD1306 drv(128, 64, &SPI1, OLED_SPI_DC_PIN, OLED_SPI_RES_PIN,
                     OLED_SPI_CS_PIN, OLED_SPI_HZ);
bool dispBegin() { return drv.begin(SSD1306_SWITCHCAPVCC, 0, true, true); }
#endif
Adafruit_GFX &disp = drv;
bool haveDisplay = false;

Screen screen = S_STATUS;
bool dirty = true;
uint32_t lastInput = 0;
UiStatus st; // core 1's copy of the status
uint32_t prevFrames = 0, tActivity = 0;
char vbusText[12] = "";

// status screen: first unit shown
int statusTop = 0;

// menus
int menuSel = 0, menuTop = 0;
int actionSel = 0, actionTop = 0;
uint8_t actionUnit[UI_MAX_UNITS]; // unit of each "Mount on" row
int actionCount = 0;              // rows before Cancel
int unitSel = 0, unitTop = 0;
int curUnit = 0;     // unit shown on the unit screen
int drivesValue = 4; // value being edited on the drives screen

// browser
char dirPath[UI_PATH_LEN] = "/";
char prevPath[UI_PATH_LEN] = "/";
UiDirEntry view[UI_MAX_ENTRIES];
int viewCount = 0;
bool viewTruncated = false;
bool loading = false;
uint32_t listId = 0;
int browseSel = 0, browseTop = 0;
char pickPath[UI_PATH_LEN];
char pickName[UI_NAME_LEN];

// requests
bool haveWant = false;
UiRequest want;
uint32_t waitId = 0;
uint8_t waitKind = UI_REQ_NONE;
char waitText[22] = "";

// message screen
char msgText[UI_MSG_LEN] = "";
Screen msgNext = S_STATUS;
uint32_t msgSince = 0;

const int STATUS_UNIT_ROWS = 4; // units shown at once on the status screen
const int UNIT_ITEMS = 5;       // Virtual, Physical, Bypass, Eject, Back
const int ROWS = 7;             // list rows below the title

// Browse, one per unit, Drives, Save settings, Back
int menuItems() { return st.unitCount + 4; }

// ---- helpers --------------------------------------------------------------

void gotoScreen(Screen s) {
  screen = s;
  dirty = true;
}

// Give up on the request being waited for: withdraw it if core 0 has not taken
// it yet; otherwise its result is discarded (the id no longer matches).
void cancelWait() {
  haveWant = false;
  if (waitId) {
    Lock l;
    if (g_reqPending && g_req.id == waitId) {
      g_reqPending = false;
      g_inFlight = false;
    }
  }
  waitId = 0;
  if (loading) {
    loading = false;
    snprintf(dirPath, sizeof(dirPath), "%s", prevPath);
  }
}

void showMessage(const char *text, Screen next, uint32_t now) {
  snprintf(msgText, sizeof(msgText), "%s", text);
  msgNext = next;
  msgSince = now;
  gotoScreen(S_MSG);
}

// Post `want` once core 0 is free to take it; returns the request id, or 0.
uint32_t tryPost() {
  Lock l;
  if (g_reqPending || g_inFlight)
    return 0;
  g_req = want;
  g_req.id = ++g_nextId;
  if (g_req.id == 0)
    g_req.id = ++g_nextId;
  g_reqPending = true;
  g_inFlight = true;
  return g_req.id;
}

void request(uint8_t kind, uint8_t unit, uint8_t arg, const char *path,
             const char *text) {
  memset(&want, 0, sizeof(want));
  want.kind = kind;
  want.unit = unit;
  want.arg = arg;
  if (path)
    snprintf(want.path, sizeof(want.path), "%s", path);
  haveWant = true;
  waitId = 0;
  waitKind = kind;
  snprintf(waitText, sizeof(waitText), "%s", text);
  if (kind != UI_REQ_LIST)
    gotoScreen(S_WAIT);
}

void listFolder(const char *path) {
  snprintf(prevPath, sizeof(prevPath), "%s", dirPath);
  snprintf(dirPath, sizeof(dirPath), "%s", path);
  loading = true;
  request(UI_REQ_LIST, 0, 0, path, "");
  dirty = true;
}

int cmpEntries(const UiDirEntry &a, const UiDirEntry &b) {
  if (a.isDir != b.isDir)
    return a.isDir ? -1 : 1;
  return strcasecmp(a.name, b.name);
}

// Folders first, then by name (insertion sort: at most UI_MAX_ENTRIES).
void sortView() {
  for (int i = 1; i < viewCount; i++) {
    UiDirEntry key = view[i];
    int j = i - 1;
    while (j >= 0 && cmpEntries(view[j], key) > 0) {
      view[j + 1] = view[j];
      j--;
    }
    view[j + 1] = key;
  }
}

bool atRoot() { return !strcmp(dirPath, "/"); }

int browseItems() { return viewCount + (atRoot() ? 0 : 1); }

// Child path of the current folder; false if too long.
bool childPath(const char *name, char *out, size_t n) {
  int len = snprintf(out, n, "%s%s%s", dirPath, atRoot() ? "" : "/", name);
  return len > 0 && (size_t)len < n;
}

void parentPath(char *out, size_t n) {
  snprintf(out, n, "%s", dirPath);
  char *s = strrchr(out, '/');
  if (s == out)
    out[1] = 0;
  else if (s)
    *s = 0;
}

// What a unit holds, for a row with `room` characters left.
void unitText(int u, char *buf, size_t n, int room) {
  const UiUnitInfo &ui = st.unit[u];
  if (ui.mode == UI_MODE_BYPASS)
    snprintf(buf, n, "bypass");
  else if (ui.source == UI_SRC_NONE)
    snprintf(buf, n, "--");
  else if (ui.source == UI_SRC_FLOPPY)
    snprintf(buf, n, "FLOPPY");
  else
    snprintf(buf, n, "%.*s", room, ui.name);
}

void menuLabel(int i, char *buf, size_t n) {
  int units = st.unitCount;
  if (i == 0) {
    snprintf(buf, n, "Browse SD card");
  } else if (i <= units) {
    char t[24];
    unitText(i - 1, t, sizeof(t), 20 - (i >= 10 ? 4 : 3));
    snprintf(buf, n, "D%d %s", i, t);
  } else if (i == units + 1) {
    snprintf(buf, n, "Drives: %d", units);
  } else if (i == units + 2) {
    snprintf(buf, n, "Save settings");
  } else {
    snprintf(buf, n, "Back");
  }
}

void actionLabel(int i, char *buf, size_t n) {
  if (i < actionCount)
    snprintf(buf, n, "Mount on D%d%s", actionUnit[i] + 1,
             st.unit[actionUnit[i]].source != UI_SRC_NONE ? " *" : "");
  else
    snprintf(buf, n, "Cancel");
}

void unitLabel(int i, char *buf, size_t n) {
  static const char *const names[UNIT_ITEMS] = {"Virtual", "Physical (floppy)",
                                                "Bypass", "Eject", "Back"};
  bool current = i < 3 && st.unit[curUnit].mode == i;
  snprintf(buf, n, "%s%s", names[i], current ? " *" : "");
}

void browseLabel(int i, char *buf, size_t n) {
  if (!atRoot()) {
    if (i == 0) {
      snprintf(buf, n, "..");
      return;
    }
    i--;
  }
  snprintf(buf, n, "%.20s%s", view[i].name, view[i].isDir ? "/" : "");
}

void clampScroll(int count, int *sel, int *top) {
  if (count <= 0) {
    *sel = *top = 0;
    return;
  }
  *sel = (*sel % count + count) % count;
  if (*sel < *top)
    *top = *sel;
  if (*sel >= *top + ROWS)
    *top = *sel - ROWS + 1;
  if (*top < 0)
    *top = 0;
}

void moveSel(int steps, int count, int *sel, int *top) {
  *sel += steps;
  clampScroll(count, sel, top);
}

// ---- input ----------------------------------------------------------------

void onMenuSelect(uint32_t now) {
  int units = st.unitCount;
  if (menuSel == 0) {
    gotoScreen(S_BROWSE);
    listFolder(dirPath); // always reread: the card may have changed
  } else if (menuSel <= units) {
    curUnit = menuSel - 1;
    unitSel = unitTop = 0;
    gotoScreen(S_UNIT);
  } else if (menuSel == units + 1) {
    drivesValue = units < 1 ? 1 : units;
    gotoScreen(S_DRIVES);
  } else if (menuSel == units + 2) {
    request(UI_REQ_SAVE, 0, 0, nullptr, "Saving");
  } else {
    gotoScreen(S_STATUS);
  }
  (void)now;
}

// Units a file can be mounted on: all but those set to bypass.
void buildActionUnits() {
  actionCount = 0;
  for (int u = 0; u < st.unitCount && u < UI_MAX_UNITS; u++)
    if (st.unit[u].mode != UI_MODE_BYPASS)
      actionUnit[actionCount++] = (uint8_t)u;
}

void onUnitSelect() {
  char t[32];
  snprintf(t, sizeof(t), "D%d", curUnit + 1);
  if (unitSel < 3)
    request(UI_REQ_SET_MODE, curUnit, (uint8_t)unitSel, nullptr, t);
  else if (unitSel == 3)
    request(UI_REQ_UMOUNT, curUnit, 0, nullptr, t);
  else
    gotoScreen(S_MENU);
}

void onBrowseSelect(uint32_t now) {
  int n = browseItems();
  if (loading || n == 0)
    return;
  int i = browseSel;
  if (!atRoot()) {
    if (i == 0) {
      char p[UI_PATH_LEN];
      parentPath(p, sizeof(p));
      browseSel = browseTop = 0;
      listFolder(p);
      return;
    }
    i--;
  }
  char p[UI_PATH_LEN];
  if (!childPath(view[i].name, p, sizeof(p))) {
    showMessage("Path too long", S_BROWSE, now);
    return;
  }
  if (view[i].isDir) {
    browseSel = browseTop = 0;
    listFolder(p);
  } else {
    snprintf(pickPath, sizeof(pickPath), "%s", p);
    snprintf(pickName, sizeof(pickName), "%s", view[i].name);
    actionSel = actionTop = 0;
    buildActionUnits();
    gotoScreen(S_ACTION);
  }
}

void handleInput(int steps, Event evt, uint32_t now) {
  switch (screen) {
  case S_STATUS:
    if (steps) {
      statusTop += steps;
      dirty = true; // drawStatus keeps it in range
    }
    if (evt != EVT_NONE) {
      menuSel = menuTop = 0;
      gotoScreen(S_MENU);
    }
    break;
  case S_MENU:
    moveSel(steps, menuItems(), &menuSel, &menuTop);
    if (evt == EVT_LONG)
      gotoScreen(S_STATUS);
    else if (evt == EVT_SHORT)
      onMenuSelect(now);
    break;
  case S_BROWSE:
    if (!loading)
      moveSel(steps, browseItems(), &browseSel, &browseTop);
    if (evt == EVT_LONG) {
      cancelWait(); // also abandons a listing in progress
      gotoScreen(S_MENU);
    } else if (evt == EVT_SHORT) {
      onBrowseSelect(now);
    }
    break;
  case S_ACTION:
    moveSel(steps, actionCount + 1, &actionSel, &actionTop);
    if (evt == EVT_LONG || (evt == EVT_SHORT && actionSel >= actionCount)) {
      gotoScreen(S_BROWSE);
    } else if (evt == EVT_SHORT) {
      char t[32];
      snprintf(t, sizeof(t), "Mount on D%d", actionUnit[actionSel] + 1);
      request(UI_REQ_MOUNT, actionUnit[actionSel], 0, pickPath, t);
    }
    break;
  case S_UNIT:
    moveSel(steps, UNIT_ITEMS, &unitSel, &unitTop);
    if (evt == EVT_LONG)
      gotoScreen(S_MENU);
    else if (evt == EVT_SHORT)
      onUnitSelect();
    break;
  case S_DRIVES:
    if (steps) {
      drivesValue += steps;
      if (drivesValue < 1)
        drivesValue = 1;
      if (drivesValue > UI_MAX_UNITS)
        drivesValue = UI_MAX_UNITS;
      dirty = true;
    }
    if (evt == EVT_LONG) {
      gotoScreen(S_MENU);
    } else if (evt == EVT_SHORT) {
      if (drivesValue == st.unitCount)
        gotoScreen(S_MENU);
      else
        request(UI_REQ_SET_UNITS, 0, (uint8_t)drivesValue, nullptr, "Drives");
    }
    break;
  case S_MSG:
    if (evt != EVT_NONE)
      gotoScreen(msgNext);
    break;
  case S_WAIT:
    if (evt == EVT_LONG) {
      cancelWait();
      gotoScreen(S_MENU);
    }
    break;
  }
}

// ---- results from core 0 ----------------------------------------------------

void pollRequests(uint32_t now) {
  if (haveWant) {
    uint32_t id = tryPost();
    if (id) {
      haveWant = false;
      waitId = id;
      if (waitKind == UI_REQ_LIST)
        listId = id;
    }
    return;
  }
  if (!waitId)
    return;

  if (waitKind == UI_REQ_LIST) {
    bool ok = false;
    char text[UI_MSG_LEN];
    {
      Lock l;
      if (g_list.id != waitId || g_list.state == LIST_RUNNING)
        return;
      ok = g_list.state == LIST_DONE;
      if (ok) {
        viewCount = g_list.count;
        memcpy(view, g_list.entry, sizeof(UiDirEntry) * viewCount);
        viewTruncated = g_list.truncated;
      }
      snprintf(text, sizeof(text), "%s", g_list.msg);
    }
    waitId = 0;
    loading = false;
    if (ok) {
      sortView();
      dirty = true;
    } else {
      // Nothing trustworthy to show: drop the old entries and start over.
      viewCount = 0;
      viewTruncated = false;
      snprintf(dirPath, sizeof(dirPath), "/");
      browseSel = browseTop = 0;
      showMessage(text, S_MENU, now);
    }
  } else {
    char text[UI_MSG_LEN];
    bool ok;
    {
      Lock l;
      if (g_resultId != waitId)
        return;
      ok = g_resultOk;
      snprintf(text, sizeof(text), "%s", g_resultMsg);
    }
    waitId = 0;
    Screen next = S_MENU;
    if (ok)
      next = S_STATUS;
    else if (waitKind == UI_REQ_MOUNT)
      next = S_ACTION;
    else if (waitKind == UI_REQ_SET_MODE || waitKind == UI_REQ_UMOUNT)
      next = S_UNIT;
    else if (waitKind == UI_REQ_SET_UNITS)
      next = S_DRIVES;
    showMessage(text, next, now);
  }
}

// ---- drawing --------------------------------------------------------------

void text(int row, const char *s) {
  disp.setCursor(0, row * 8);
  disp.print(s);
}

void drawTitle(const char *s) {
  disp.fillRect(0, 0, 128, 8, WHITE_PIXEL);
  disp.setTextColor(BLACK_PIXEL);
  text(0, s);
  disp.setTextColor(WHITE_PIXEL);
}

void drawList(const char *title, int count, int sel, int top,
              void (*label)(int, char *, size_t)) {
  drawTitle(title);
  for (int r = 0; r < ROWS; r++) {
    int i = top + r;
    if (i >= count)
      break;
    char buf[24];
    label(i, buf, sizeof(buf));
    if (i == sel) {
      disp.fillRect(0, (r + 1) * 8, 128, 8, WHITE_PIXEL);
      disp.setTextColor(BLACK_PIXEL);
    }
    disp.setCursor(0, (r + 1) * 8);
    disp.print(i == sel ? ">" : " ");
    disp.print(buf);
    disp.setTextColor(WHITE_PIXEL);
  }
}

// A one-character scroll hint at the right edge of a status row.
void drawHint(int row, char c) {
  disp.setCursor(120, row * 8);
  disp.print(c);
}

void drawStatus(uint32_t now) {
  char b[48];
  int units = st.unitCount;
  int maxTop = units > STATUS_UNIT_ROWS ? units - STATUS_UNIT_ROWS : 0;
  if (statusTop > maxTop)
    statusTop = maxTop;
  if (statusTop < 0)
    statusTop = 0;
  for (int r = 0; r < STATUS_UNIT_ROWS && statusTop + r < units; r++) {
    int i = statusTop + r;
    const UiUnitInfo &u = st.unit[i];
    // Column 21 is left free for the scroll hint.
    int w = 20 - (i >= 9 ? 4 : 3);
    if (u.mode == UI_MODE_BYPASS)
      snprintf(b, sizeof(b), "D%d bypass", i + 1);
    else if (u.source == UI_SRC_NONE)
      snprintf(b, sizeof(b), "D%d --", i + 1);
    else if (u.source == UI_SRC_FLOPPY)
      snprintf(b, sizeof(b), "D%d FLOPPY%s", i + 1,
               st.floppyReady ? "" : " (no disk)");
    else if (u.source == UI_SRC_ATX)
      snprintf(b, sizeof(b), "D%d %-*.*s ATX", i + 1, w - 4, w - 4, u.name);
    else
      snprintf(b, sizeof(b), "D%d %.*s", i + 1, w, u.name);
    text(r, b);
  }
  if (statusTop > 0)
    drawHint(0, '^');
  if (statusTop < maxTop)
    drawHint(STATUS_UNIT_ROWS - 1, 'v');
  if (st.frames) {
    snprintf(b, sizeof(b), "%cD%u %c %u %c",
             now - tActivity < ACTIVITY_MS ? '>' : ' ', st.lastUnit,
             st.lastCommand, st.lastAux, st.lastResult);
  } else {
    snprintf(b, sizeof(b), " SIO idle");
  }
  if (st.busy)
    snprintf(b, sizeof(b), "Busy: %.15s", st.busyText);
  text(4, b);
  if (st.floppyMounted) {
    static const char *const dens[] = {"SD", "ED", "DD", "??"};
    if (st.floppyReady) {
      snprintf(b, sizeof(b), "FLP %s %s %uT",
               dens[st.density > 3 ? 3 : st.density],
               st.sides == 2 ? "DS" : "SS", st.tracks);
      text(5, b);
    } else {
      text(5, "FLP no disk");
    }
    char tr[8];
    if (st.cylinder == 0xFF)
      snprintf(tr, sizeof(tr), "T--");
    else
      snprintf(tr, sizeof(tr), "T%02u", st.cylinder);
    snprintf(b, sizeof(b), "%s H%u%s%s", tr, st.head,
             st.floppyMotor ? " MOTOR" : "", st.floppyActive ? " BUSY" : "");
    text(6, b);
  }
  snprintf(b, sizeof(b), "Hold=menu %11s", vbusText);
  text(7, b);
}

void drawWrapped(int row, const char *s) {
  disp.setTextWrap(true);
  disp.setCursor(0, row * 8);
  disp.print(s);
  disp.setTextWrap(false);
}

void render(uint32_t now) {
  drv.clearDisplay();
  disp.setTextSize(1);
  disp.setTextColor(WHITE_PIXEL);
  disp.setTextWrap(false);
  switch (screen) {
  case S_STATUS:
    drawStatus(now);
    break;
  case S_MENU:
    clampScroll(menuItems(), &menuSel, &menuTop);
    drawList("Menu", menuItems(), menuSel, menuTop, menuLabel);
    break;
  case S_BROWSE: {
    int n = browseItems();
    if (loading) {
      drawTitle(dirPath);
      text(3, "Reading...");
    } else {
      clampScroll(n, &browseSel, &browseTop);
      // keep the end of a long path visible
      // ("+" at the end of the title: more entries than are shown)
      size_t len = strlen(dirPath), room = viewTruncated ? 19 : 21;
      char title[24];
      snprintf(title, sizeof(title), "%.21s%s",
               len > room ? dirPath + len - room : dirPath,
               viewTruncated ? " +" : "");
      drawList(title, n, browseSel, browseTop, browseLabel);
      if (n == 0)
        text(3, " (no images)");
    }
    break;
  }
  case S_ACTION:
    clampScroll(actionCount + 1, &actionSel, &actionTop);
    drawList(pickName, actionCount + 1, actionSel, actionTop, actionLabel);
    break;
  case S_UNIT: {
    char title[16];
    snprintf(title, sizeof(title), "D%d", curUnit + 1);
    clampScroll(UNIT_ITEMS, &unitSel, &unitTop);
    drawList(title, UNIT_ITEMS, unitSel, unitTop, unitLabel);
    break;
  }
  case S_DRIVES: {
    char b[24];
    drawTitle("Drives");
    snprintf(b, sizeof(b), "Drives: %d (D1-D%d)", drivesValue, drivesValue);
    text(3, b);
    text(5, "Turn = change");
    text(6, "Press = apply");
    text(7, "Hold = cancel");
    break;
  }
  case S_WAIT:
    drawTitle(waitText);
    if (st.busy) {
      char b[28];
      snprintf(b, sizeof(b), "Busy: %.15s", st.busyText);
      text(3, b);
    } else {
      text(3, "Working...");
    }
    text(7, "Hold = cancel");
    break;
  case S_MSG:
    drawTitle("Message");
    drawWrapped(2, msgText);
    text(7, "Press to continue");
    break;
  }
  drv.display();
}

void readVbus() {
#if defined(PD_SDA_PIN)
  pdShortStatus(vbusText, sizeof(vbusText));
#endif
}

} // namespace

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void uiSetup1() {
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  g_encState = (digitalRead(ENCODER_A_PIN) ? 2 : 0) |
               (digitalRead(ENCODER_B_PIN) ? 1 : 0);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderIsr, CHANGE);

#if OLED_TYPE == OLED_SH1106_I2C
  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
#else
  // Only SCK and TX: begin() would otherwise also claim the default RX pin
  // (GPIO12, the floppy STEP line); the hardware CS is not used either.
  SPI1.setRX(NOPIN);
  SPI1.setSCK(OLED_SPI_SCK_PIN);
  SPI1.setTX(OLED_SPI_MOSI_PIN);
#endif
  haveDisplay = dispBegin();
  disp.setRotation(0);
  lastInput = millis();
}

void uiLoop1() {
  static uint32_t tCopy = 0, tDraw = 0, tVbus = 0;
  uint32_t now = millis();

  Event evt = pollSwitch(now);
  int steps = takeSteps();
  if (steps || evt != EVT_NONE) {
    lastInput = now;
    handleInput(steps, evt, now);
  }

  if (now - tCopy >= STATUS_COPY_MS) {
    tCopy = now;
    {
      Lock l;
      st = g_status;
    }
    if (st.frames != prevFrames) {
      prevFrames = st.frames;
      tActivity = now;
    }
  }
  if (now - tVbus >= 500) {
    tVbus = now;
    readVbus();
  }

  pollRequests(now);
  if (screen == S_MSG && now - msgSince >= MESSAGE_MS)
    gotoScreen(msgNext);
  if ((screen == S_MENU || screen == S_BROWSE || screen == S_ACTION ||
       screen == S_UNIT || screen == S_DRIVES) &&
      now - lastInput >= IDLE_BACK_MS)
    gotoScreen(S_STATUS);

  if (haveDisplay &&
      (dirty || (screen == S_STATUS && now - tDraw >= STATUS_REDRAW_MS))) {
    dirty = false;
    tDraw = now;
    render(now);
  }
  delay(1);
}

#endif // OLED_SDA_PIN
