// Link watchdog: torque off on every servo when the PC stops talking while some
// servo may hold torque.
//
// Trips when the sniffer tracks at least one ID with torque and either no byte came
// from the PC for `timeout_ms`, or the link was reported lost (CDC port closed / DTR
// dropped, USB suspended). Torque off is the same safe state the PC's E-stop uses.
// Plain C++, unit-tested on the host.

#pragma once

#include <cstddef>
#include <cstdint>

#include "scs_sniffer.h"

class LinkWatchdog {
 public:
  explicit LinkWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void on_pc_bytes(const uint8_t* data, size_t len, uint32_t now_ms);
  void on_link_lost() { link_lost_ = true; }

  bool should_fire(uint32_t now_ms) const;

  // Bytes for the bus: completion of a partly sent PC packet (see Sniffer::completion),
  // then the torque-off broadcast. Clears every tracked ID and latches `tripped()` until
  // the PC sends again. `cap` must be at least kFireMaxLen.
  size_t fire(uint8_t* out, size_t cap);

  bool armed() const { return sniffer_.any_torque(); }
  bool tripped() const { return tripped_; }
  const scs::Sniffer& sniffer() const { return sniffer_; }

  static constexpr size_t kFireMaxLen = scs::kMaxCompletionLen + scs::kTorqueOffPacketLen;

 private:
  scs::Sniffer sniffer_;
  uint32_t timeout_ms_;
  uint32_t last_pc_ms_ = 0;
  bool link_lost_ = false;
  bool tripped_ = false;
};
