// Packet builders shared by the host tests.

#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

#include "scs_sniffer.h"

inline std::vector<uint8_t> packet(uint8_t id, uint8_t instruction, std::initializer_list<uint8_t> params) {
  std::vector<uint8_t> p = {0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), instruction};
  p.insert(p.end(), params);
  p.push_back(scs::checksum(p.data() + 2, p.size() - 2));
  return p;
}

inline std::vector<uint8_t> write(uint8_t id, uint8_t address, std::initializer_list<uint8_t> data) {
  std::vector<uint8_t> params = {address};
  params.insert(params.end(), data);
  std::vector<uint8_t> p = {0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), scs::kInstWrite};
  p.insert(p.end(), params.begin(), params.end());
  p.push_back(scs::checksum(p.data() + 2, p.size() - 2));
  return p;
}

// SYNC_WRITE of `data_len` bytes at `address` to each (id, bytes...) entry.
inline std::vector<uint8_t> sync_write(uint8_t address, uint8_t data_len,
                                       std::initializer_list<std::vector<uint8_t>> entries) {
  std::vector<uint8_t> params = {address, data_len};
  for (const auto& e : entries) params.insert(params.end(), e.begin(), e.end());
  std::vector<uint8_t> p = {0xFF, 0xFF, scs::kBroadcastId, static_cast<uint8_t>(params.size() + 2),
                            scs::kInstSyncWrite};
  p.insert(p.end(), params.begin(), params.end());
  p.push_back(scs::checksum(p.data() + 2, p.size() - 2));
  return p;
}
