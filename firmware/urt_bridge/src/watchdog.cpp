#include "watchdog.h"

void LinkWatchdog::on_pc_bytes(const uint8_t* data, size_t len, uint32_t now_ms) {
  if (len == 0) return;
  sniffer_.feed(data, len);
  last_pc_ms_ = now_ms;
  link_lost_ = false;
  tripped_ = false;
}

bool LinkWatchdog::should_fire(uint32_t now_ms) const {
  if (!sniffer_.any_torque()) return false;
  // Unsigned subtraction stays correct across the millis() wrap.
  return link_lost_ || now_ms - last_pc_ms_ >= timeout_ms_;
}

size_t LinkWatchdog::fire(uint8_t* out, size_t cap) {
  if (cap < kFireMaxLen) return 0;
  size_t n = sniffer_.completion(out, cap);
  scs::torque_off_broadcast(out + n);
  n += scs::kTorqueOffPacketLen;
  // Feeding our own bytes keeps the sniffer in step with the wire and clears every ID.
  sniffer_.feed(out, n);
  link_lost_ = false;
  tripped_ = true;
  return n;
}
