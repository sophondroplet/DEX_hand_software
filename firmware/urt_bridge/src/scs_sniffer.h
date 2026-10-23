// Passive parser of the PC -> servo-bus byte stream (Feetech SCS protocol).
//
// The bridge forwards every byte unchanged; this parser only watches them to know
// which servo IDs may currently hold torque, so the link watchdog can switch them
// off if the PC goes away. It never needs to be exact about which servos exist:
// it is conservative (any write that could enable torque marks the ID) and only a
// valid torque-off write clears it.
//
// Plain C++ without Arduino dependencies, so it is unit-tested on the host
// (`pio test -e native`).

#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>

namespace scs {

constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kMaxServoId = 253;
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstRegWrite = 0x04;
constexpr uint8_t kInstSyncWrite = 0x83;
// Same address on the SCS0009 and the HLS3606 (dexhand_servo/registers.py REG_TORQUE_ENABLE).
constexpr uint8_t kRegTorqueEnable = 40;
// Goal position / goal time-or-current / goal speed; a goal write may re-enable torque.
constexpr uint8_t kGoalBlockFirst = 42;
constexpr uint8_t kGoalBlockLast = 47;

constexpr size_t kTorqueOffPacketLen = 8;
// Longest completion: a body of 255 bytes of which none has arrived yet.
constexpr size_t kMaxCompletionLen = 255;

// ~(sum of `len` bytes) & 0xFF, i.e. the SCS checksum when `bytes` starts at the ID byte.
uint8_t checksum(const uint8_t* bytes, size_t len);

// The broadcast WRITE "register 40 = 0" (torque off on every servo at the current baud rate).
void torque_off_broadcast(uint8_t out[kTorqueOffPacketLen]);

class Sniffer {
 public:
  void feed(uint8_t byte);
  void feed(const uint8_t* data, size_t len);

  // True between packets; false while a packet is partly received.
  bool at_boundary() const { return state_ == State::kIdle; }
  bool any_torque() const { return torque_.any(); }
  bool torque(uint8_t id) const { return torque_.test(id); }
  uint32_t rejected() const { return rejected_; }

  // Bytes that finish the partly sent packet with a checksum that is guaranteed to be
  // wrong, so every servo discards it and is back at a packet boundary before the
  // watchdog's own packet. Writes nothing (returns 0) at a boundary; `cap` must be at
  // least kMaxCompletionLen.
  size_t completion(uint8_t* out, size_t cap) const;

 private:
  enum class State : uint8_t { kIdle, kHeader1, kId, kLength, kBody };

  void finish();
  void apply_write(uint8_t id, uint8_t address, const uint8_t* data, size_t len);
  void apply_to(uint8_t id, bool on);

  State state_ = State::kIdle;
  uint8_t id_ = 0;
  uint8_t length_ = 0;  // LEN field: instruction + parameters + checksum
  size_t received_ = 0;
  uint8_t body_[256] = {};
  std::bitset<kMaxServoId + 1> torque_;
  uint32_t rejected_ = 0;
};

}  // namespace scs
