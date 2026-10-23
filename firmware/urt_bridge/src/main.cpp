// Dexhand URT bridge: ESP32-S3 as a transparent USB-serial adapter in front of the URT-2.
//
//   PC --USB CDC (native USB port)--> ESP32-S3 --UART1--> URT-2 TTL input --> servo bus
//
// Bytes pass unchanged in both directions, so the PC tool (SafeBus guard, SDK framing and
// timeouts) works as with the URT-2's own USB port. The UART follows the baud rate the PC
// sets on the CDC port. A link watchdog (watchdog.h) broadcasts torque off when the PC
// goes silent while servos may hold torque. Logs go to UART0 (the DevKit's "UART" port).

#include <Arduino.h>
#include <USB.h>

#include <atomic>

#include "scs_sniffer.h"
#include "watchdog.h"

#ifndef BUS_TX_PIN
#define BUS_TX_PIN 17
#endif
#ifndef BUS_RX_PIN
#define BUS_RX_PIN 18
#endif
#ifndef RGB_LED_PIN
#define RGB_LED_PIN 48
#endif
#ifndef WATCHDOG_MS
#define WATCHDOG_MS 500
#endif
// The watchdog waits for this much silence on the bus so it never talks over a servo reply.
#ifndef BUS_IDLE_MS
#define BUS_IDLE_MS 2
#endif
#ifndef TORQUE_OFF_REPEATS
#define TORQUE_OFF_REPEATS 3
#endif

namespace {

constexpr uint32_t kDefaultBaud = 1000000;
// dexhand_servo/registers.py SUPPORTED_BAUDRATES; anything else is ignored.
constexpr uint32_t kSupportedBauds[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};

USBCDC pc_link;
HardwareSerial& bus = Serial1;
HardwareSerial& log_port = Serial;  // UART0 with ARDUINO_USB_CDC_ON_BOOT=0

LinkWatchdog watchdog(WATCHDOG_MS);
std::atomic<uint32_t> pending_baud{0};
std::atomic<bool> link_lost{false};
uint32_t bus_baud = kDefaultBaud;
uint32_t last_bus_ms = 0;
uint32_t dropped_to_pc = 0;
uint8_t buffer[1024];

enum class Led : uint8_t { kUnset, kIdle, kArmed, kTripped };
Led led = Led::kUnset;

void show(Led state) {
  if (state == led) return;  // neopixelWrite is slow; only on change
  led = state;
  switch (state) {
    case Led::kIdle: neopixelWrite(RGB_LED_PIN, 0, 0, 8); break;
    case Led::kArmed: neopixelWrite(RGB_LED_PIN, 0, 12, 0); break;
    case Led::kTripped: neopixelWrite(RGB_LED_PIN, 40, 0, 0); break;
    case Led::kUnset: break;
  }
}

bool supported(uint32_t baud) {
  for (uint32_t b : kSupportedBauds) {
    if (b == baud) return true;
  }
  return false;
}

void on_cdc_event(void*, esp_event_base_t, int32_t id, void* data) {
  auto* event = static_cast<arduino_usb_cdc_event_data_t*>(data);
  if (id == ARDUINO_USB_CDC_LINE_CODING_EVENT) {
    pending_baud = event->line_coding.bit_rate;
  } else if (id == ARDUINO_USB_CDC_DISCONNECTED_EVENT) {
    link_lost = true;  // DTR dropped: the PC closed the port or its process died
  }
}

void on_usb_event(void*, esp_event_base_t, int32_t id, void*) {
  if (id == ARDUINO_USB_SUSPEND_EVENT || id == ARDUINO_USB_STOPPED_EVENT) link_lost = true;
}

void apply_pending_baud() {
  const uint32_t baud = pending_baud.exchange(0);
  if (baud == 0 || baud == bus_baud) return;
  if (!supported(baud)) {
    log_port.printf("baud %lu ignored (unsupported), bus stays at %lu\n", (unsigned long)baud,
                    (unsigned long)bus_baud);
    return;
  }
  bus.flush();
  bus.updateBaudRate(baud);
  bus_baud = baud;
  log_port.printf("bus baud %lu\n", (unsigned long)baud);
}

void forward_to_pc(const uint8_t* data, size_t len) {
  // USBCDC::write spins while the host buffer is full; a host that stopped reading must
  // not stall this loop (and with it the watchdog), so only what fits is sent.
  const int space = pc_link.availableForWrite();
  const size_t n = space > 0 ? min(len, static_cast<size_t>(space)) : 0;
  if (n > 0) pc_link.write(data, n);
  dropped_to_pc += len - n;
}

void fire_torque_off() {
  static uint8_t out[LinkWatchdog::kFireMaxLen];
  const size_t n = watchdog.fire(out, sizeof(out));
  bus.write(out, n);  // completion of a partial PC packet + first broadcast
  bus.flush();
  uint8_t again[scs::kTorqueOffPacketLen];
  scs::torque_off_broadcast(again);
  for (int i = 1; i < TORQUE_OFF_REPEATS; ++i) {
    delay(1);
    bus.write(again, sizeof(again));
    bus.flush();
  }
  last_bus_ms = millis();
  log_port.printf("WATCHDOG: torque-off broadcast sent (%u bytes incl. %u completion)\n", (unsigned)n,
                  (unsigned)(n - scs::kTorqueOffPacketLen));
}

}  // namespace

void setup() {
  log_port.setTxBufferSize(1024);  // logging must never block the forwarding loop
  log_port.begin(115200);

  bus.setRxBufferSize(sizeof(buffer));
  bus.begin(kDefaultBaud, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);
  bus.setRxTimeout(1);  // hand bytes to the ring buffer one symbol after a reply ends

  pc_link.enableReboot(false);  // no 1200-baud / DTR-RTS reset into the bootloader
  pc_link.setRxBufferSize(sizeof(buffer));
  pc_link.onEvent(on_cdc_event);
  pc_link.begin();

  USB.productName("Dexhand URT Bridge");
  USB.manufacturerName("Nimo");
  USB.onEvent(on_usb_event);
  USB.begin();

  show(Led::kIdle);
  log_port.printf("Dexhand URT bridge: UART1 TX=%d RX=%d @ %lu, watchdog %d ms\n", BUS_TX_PIN, BUS_RX_PIN,
                  (unsigned long)kDefaultBaud, WATCHDOG_MS);
}

void loop() {
  const uint32_t now = millis();
  apply_pending_baud();

  int available = pc_link.available();
  if (available > 0) {
    const size_t n = pc_link.read(buffer, min(static_cast<size_t>(available), sizeof(buffer)));
    bus.write(buffer, n);
    watchdog.on_pc_bytes(buffer, n, now);
    last_bus_ms = now;
  }

  available = bus.available();
  if (available > 0) {
    const size_t n = bus.read(buffer, min(static_cast<size_t>(available), sizeof(buffer)));
    forward_to_pc(buffer, n);
    last_bus_ms = now;
  }

  if (link_lost.exchange(false)) {
    watchdog.on_link_lost();
    log_port.printf("link lost (port closed / USB suspended)%s\n", watchdog.armed() ? ", torque tracked" : "");
  }
  if (watchdog.should_fire(now) && now - last_bus_ms >= BUS_IDLE_MS) fire_torque_off();

  show(watchdog.tripped() ? Led::kTripped : watchdog.armed() ? Led::kArmed : Led::kIdle);
}
