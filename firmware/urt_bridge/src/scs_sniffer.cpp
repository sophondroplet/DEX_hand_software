#include "scs_sniffer.h"

namespace scs {

uint8_t checksum(const uint8_t* bytes, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; ++i) sum += bytes[i];
  return static_cast<uint8_t>(~sum);
}

void torque_off_broadcast(uint8_t out[kTorqueOffPacketLen]) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = kBroadcastId;
  out[3] = 4;  // instruction, address, value, checksum
  out[4] = kInstWrite;
  out[5] = kRegTorqueEnable;
  out[6] = 0;
  out[7] = checksum(out + 2, 5);
}

void Sniffer::feed(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) feed(data[i]);
}

void Sniffer::feed(uint8_t byte) {
  switch (state_) {
    case State::kIdle:
      if (byte == 0xFF) state_ = State::kHeader1;
      break;
    case State::kHeader1:
      state_ = byte == 0xFF ? State::kId : State::kIdle;
      break;
    case State::kId:
      if (byte != 0xFF) {  // extra 0xFF before the ID is tolerated, as the servos do
        id_ = byte;
        state_ = State::kLength;
      }
      break;
    case State::kLength:
      length_ = byte;
      received_ = 0;
      if (length_ < 2) {
        ++rejected_;
        state_ = State::kIdle;
      } else {
        state_ = State::kBody;
      }
      break;
    case State::kBody:
      body_[received_++] = byte;
      if (received_ == length_) {
        finish();
        state_ = State::kIdle;
      }
      break;
  }
}

void Sniffer::finish() {
  uint32_t sum = static_cast<uint32_t>(id_) + length_;
  for (size_t i = 0; i + 1 < length_; ++i) sum += body_[i];
  if (static_cast<uint8_t>(~sum) != body_[length_ - 1]) {
    ++rejected_;  // the servos drop it too
    return;
  }
  const uint8_t instruction = body_[0];
  const uint8_t* params = body_ + 1;
  const size_t param_count = length_ - 2u;

  if (instruction == kInstWrite || instruction == kInstRegWrite) {
    // REG_WRITE only takes effect on a later ACTION; tracking it at once is the conservative side.
    if (param_count < 2) return;
    if (id_ == kBroadcastId) {
      for (unsigned id = 0; id <= kMaxServoId; ++id) {
        apply_write(static_cast<uint8_t>(id), params[0], params + 1, param_count - 1);
      }
    } else if (id_ <= kMaxServoId) {
      apply_write(id_, params[0], params + 1, param_count - 1);
    }
  } else if (instruction == kInstSyncWrite) {
    if (param_count < 2) return;
    const uint8_t address = params[0];
    const size_t data_len = params[1];
    const size_t stride = data_len + 1;
    for (size_t at = 2; at + stride <= param_count; at += stride) {
      apply_write(params[at], address, params + at + 1, data_len);
    }
  }
}

void Sniffer::apply_write(uint8_t id, uint8_t address, const uint8_t* data, size_t len) {
  if (len == 0 || id > kMaxServoId) return;
  const size_t last = address + len - 1;
  if (address <= kGoalBlockLast && last >= kGoalBlockFirst) {
    apply_to(id, true);
  } else if (address <= kRegTorqueEnable && last >= kRegTorqueEnable) {
    apply_to(id, data[kRegTorqueEnable - address] != 0);
  }
}

void Sniffer::apply_to(uint8_t id, bool on) { torque_.set(id, on); }

size_t Sniffer::completion(uint8_t* out, size_t cap) const {
  if (cap < kMaxCompletionLen) return 0;
  // A PING with a wrong checksum: harmless even if a servo took it for valid.
  auto bogus_ping = [](uint8_t id, uint8_t* p) {
    const uint8_t frame[3] = {id, 2, kInstPing};
    p[0] = 2;
    p[1] = kInstPing;
    p[2] = static_cast<uint8_t>(checksum(frame, 3) ^ 0xFF);
    return size_t{3};
  };
  switch (state_) {
    case State::kIdle:
      return 0;
    case State::kHeader1:
      out[0] = 0x00;  // "FF 00" is no header
      return 1;
    case State::kId:
      out[0] = 0x00;
      return 1 + bogus_ping(0x00, out + 1);
    case State::kLength:
      return bogus_ping(id_, out);
    case State::kBody: {
      const size_t remaining = length_ - received_;  // >= 1: the checksum is still missing
      uint32_t sum = static_cast<uint32_t>(id_) + length_;
      for (size_t i = 0; i < received_; ++i) sum += body_[i];
      for (size_t i = 0; i + 1 < remaining; ++i) out[i] = 0x00;
      out[remaining - 1] = static_cast<uint8_t>(~sum ^ 0xFF);
      return remaining;
    }
  }
  return 0;
}

}  // namespace scs
