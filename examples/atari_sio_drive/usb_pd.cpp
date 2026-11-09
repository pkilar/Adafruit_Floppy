// SPDX-FileCopyrightText: 2026 Adafruit Industries
// SPDX-License-Identifier: MIT
//
// The FUSB302's INT_N pin is not wired, so the sink is polled every
// PD_POLL_MS from a repeating timer. The timer's alarm pool is created on
// core 1: core 0 runs flux captures with interrupts off for a whole track,
// longer than a charger waits for an answer (about 25 ms, after which it
// cuts VBUS). The UI on core 1 never blocks interrupts, and nothing else uses
// I2C1.
#include "usb_pd.h"

#if defined(PD_SDA_PIN)
#include <Adafruit_FUSB302Sink.h>
#include <Adafruit_FUSB302Wire.h>
#include <Wire.h>
#include <pico/time.h>
#include <stdio.h>

namespace {
Adafruit_FUSB302Wire pdBus(Wire1, PD_I2C_ADDR);
Adafruit_FUSB302Sink pdSink(pdBus);
bool pdPresent = false;
repeating_timer_t pdTimer;

bool pdTick(repeating_timer_t *) {
  pdSink.poll(millis());
  return true;
}
} // namespace

void pdSetup1() {
  Wire1.setSDA(PD_SDA_PIN);
  Wire1.setSCL(PD_SCL_PIN);
  Wire1.setClock(400000);
  Wire1.setTimeout(5); // a stuck bus must not hold the timer interrupt long
  Wire1.begin();
  pdSink.setTarget(PD_REQUEST_MV, PD_REQUEST_MA);
  pdPresent = pdSink.begin(millis());
  if (!pdPresent)
    return; // the board then runs on whatever the charger gives by default
  alarm_pool_t *pool = alarm_pool_create_with_unused_hardware_alarm(2);
  // A negative period keeps the start-to-start interval fixed.
  if (!pool || !alarm_pool_add_repeating_timer_ms(pool, -PD_POLL_MS, pdTick,
                                                  nullptr, &pdTimer))
    pdPresent = false;
}

void pdShortStatus(char *buf, size_t len) {
  if (!pdPresent) {
    snprintf(buf, len, "no PD chip");
    return;
  }
  switch (pdSink.state()) {
  case Adafruit_FUSB302Sink::PD_DETACHED:
    snprintf(buf, len, "USB power");
    break;
  case Adafruit_FUSB302Sink::PD_READY:
    if (pdSink.contractMillivolts())
      snprintf(buf, len, "PD %uV", pdSink.contractMillivolts() / 1000);
    else
      snprintf(buf, len, "PD 5V");
    break;
  case Adafruit_FUSB302Sink::PD_NO_PD:
    snprintf(buf, len, "5V no PD");
    break;
  default:
    snprintf(buf, len, "PD ...");
    break;
  }
}

void pdPrint(Print &out) {
  if (!pdPresent) {
    out.println("USB-PD: the FUSB302 did not answer on I2C1.");
    return;
  }
  char b[96];
  snprintf(b, sizeof(b),
           "USB-PD: %s, CC%u, contract %u mV %u mA (asking for %u mV)",
           Adafruit_FUSB302Sink::stateName(pdSink.state()), pdSink.ccLine(),
           pdSink.contractMillivolts(), pdSink.contractMilliamps(),
           (unsigned)PD_REQUEST_MV);
  out.println(b);
  for (uint8_t i = 0; i < pdSink.sourcePdoCount(); i++) {
    uint32_t p = pdSink.sourcePdo(i);
    if ((p >> 30) == 0)
      snprintf(b, sizeof(b), "  PDO %u: fixed %lu mV, %lu mA", i + 1,
               ((p >> 10) & 0x3FF) * 50UL, (p & 0x3FF) * 10UL);
    else
      snprintf(b, sizeof(b), "  PDO %u: type %lu (not fixed), 0x%08lx", i + 1,
               p >> 30, (unsigned long)p);
    out.println(b);
  }
  snprintf(b, sizeof(b), "  charger PD %u.0, soft resets %u, hard resets %u",
           pdSink.sourceRevision(), pdSink.softResets(), pdSink.hardResets());
  out.println(b);
}
#endif
