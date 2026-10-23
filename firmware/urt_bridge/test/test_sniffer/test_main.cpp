#include <unity.h>

#include <vector>

#include "../packets.h"
#include "scs_sniffer.h"

using scs::Sniffer;

namespace {

constexpr uint8_t kInstRead = 0x02;

void feed(Sniffer& s, const std::vector<uint8_t>& bytes) { s.feed(bytes.data(), bytes.size()); }

void test_reads_and_pings_track_nothing() {
  Sniffer s;
  feed(s, packet(1, scs::kInstPing, {}));
  feed(s, packet(1, kInstRead, {40, 27}));
  TEST_ASSERT_FALSE(s.any_torque());
  TEST_ASSERT_TRUE(s.at_boundary());
  TEST_ASSERT_EQUAL_UINT32(0, s.rejected());
}

void test_torque_enable_write_tracks_the_id() {
  Sniffer s;
  feed(s, write(3, 40, {1}));
  TEST_ASSERT_TRUE(s.torque(3));
  TEST_ASSERT_FALSE(s.torque(4));
}

void test_torque_off_write_clears_the_id() {
  Sniffer s;
  feed(s, write(3, 40, {1}));
  feed(s, write(3, 40, {0}));
  TEST_ASSERT_FALSE(s.any_torque());
}

void test_goal_block_write_tracks_the_id() {
  Sniffer s;
  feed(s, write(5, 42, {0x01, 0xF4, 0, 0, 0x00, 0x96}));
  TEST_ASSERT_TRUE(s.torque(5));
}

void test_goal_write_wins_over_torque_off_in_the_same_packet() {
  Sniffer s;
  feed(s, write(5, 40, {0, 0, 0x01, 0xF4}));  // 40 = 0, then goal position
  TEST_ASSERT_TRUE(s.torque(5));
}

void test_eeprom_write_tracks_nothing() {
  Sniffer s;
  feed(s, write(1, 5, {7}));    // ID register
  feed(s, write(1, 9, {0x5E, 0x01, 0xDA, 0x02}));  // HLS angle limits
  TEST_ASSERT_FALSE(s.any_torque());
}

void test_sync_write_goals_track_every_entry() {
  Sniffer s;
  feed(s, sync_write(42, 6, {{1, 1, 2, 0, 0, 0, 50}, {2, 1, 2, 0, 0, 0, 50}}));
  TEST_ASSERT_TRUE(s.torque(1));
  TEST_ASSERT_TRUE(s.torque(2));
  TEST_ASSERT_FALSE(s.torque(3));
}

void test_sync_write_torque_mix() {
  Sniffer s;
  feed(s, sync_write(42, 6, {{1, 1, 2, 0, 0, 0, 50}, {2, 1, 2, 0, 0, 0, 50}}));
  feed(s, sync_write(40, 1, {{1, 0}, {2, 1}}));
  TEST_ASSERT_FALSE(s.torque(1));
  TEST_ASSERT_TRUE(s.torque(2));
}

void test_broadcast_torque_off_clears_everything() {
  Sniffer s;
  feed(s, write(1, 40, {1}));
  feed(s, write(253, 40, {1}));
  uint8_t off[scs::kTorqueOffPacketLen];
  scs::torque_off_broadcast(off);
  s.feed(off, sizeof(off));
  TEST_ASSERT_FALSE(s.any_torque());
  TEST_ASSERT_EQUAL_UINT32(0, s.rejected());
}

void test_bad_checksum_is_ignored() {
  Sniffer s;
  auto p = write(3, 40, {1});
  p.back() ^= 0x01;
  feed(s, p);
  TEST_ASSERT_FALSE(s.any_torque());
  TEST_ASSERT_EQUAL_UINT32(1, s.rejected());
}

void test_byte_by_byte_and_extra_header_bytes() {
  Sniffer s;
  std::vector<uint8_t> stream = {0x00, 0x42, 0xFF};  // noise, then an extra 0xFF before the header
  auto p = write(9, 40, {1});
  stream.insert(stream.end(), p.begin(), p.end());
  for (uint8_t b : stream) s.feed(b);
  TEST_ASSERT_TRUE(s.torque(9));
}

void test_completion_is_empty_at_a_boundary() {
  Sniffer s;
  uint8_t out[scs::kMaxCompletionLen];
  TEST_ASSERT_EQUAL(0, s.completion(out, sizeof(out)));
}

// For every cut point of a goal write: the completion ends the packet with a wrong checksum.
void test_completion_never_produces_a_valid_packet() {
  const auto p = write(4, 42, {0x01, 0xF4, 0, 0, 0x00, 0x96});
  for (size_t cut = 1; cut < p.size(); ++cut) {
    Sniffer s;
    s.feed(p.data(), cut);
    TEST_ASSERT_FALSE(s.at_boundary());
    uint8_t out[scs::kMaxCompletionLen];
    const size_t n = s.completion(out, sizeof(out));
    TEST_ASSERT_GREATER_THAN(0, n);
    const uint32_t before = s.rejected();
    s.feed(out, n);
    TEST_ASSERT_TRUE_MESSAGE(s.at_boundary(), "completion must end the packet");
    TEST_ASSERT_FALSE_MESSAGE(s.torque(4), "completion must not be accepted as a write");
    if (cut > 1) TEST_ASSERT_EQUAL_UINT32(before + 1, s.rejected());
    // The next real packet parses normally.
    const auto next = write(6, 40, {1});
    s.feed(next.data(), next.size());
    TEST_ASSERT_TRUE(s.torque(6));
  }
}

void test_length_below_two_is_rejected() {
  Sniffer s;
  const uint8_t bad[] = {0xFF, 0xFF, 0x01, 0x01};
  s.feed(bad, sizeof(bad));
  TEST_ASSERT_TRUE(s.at_boundary());
  TEST_ASSERT_EQUAL_UINT32(1, s.rejected());
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reads_and_pings_track_nothing);
  RUN_TEST(test_torque_enable_write_tracks_the_id);
  RUN_TEST(test_torque_off_write_clears_the_id);
  RUN_TEST(test_goal_block_write_tracks_the_id);
  RUN_TEST(test_goal_write_wins_over_torque_off_in_the_same_packet);
  RUN_TEST(test_eeprom_write_tracks_nothing);
  RUN_TEST(test_sync_write_goals_track_every_entry);
  RUN_TEST(test_sync_write_torque_mix);
  RUN_TEST(test_broadcast_torque_off_clears_everything);
  RUN_TEST(test_bad_checksum_is_ignored);
  RUN_TEST(test_byte_by_byte_and_extra_header_bytes);
  RUN_TEST(test_completion_is_empty_at_a_boundary);
  RUN_TEST(test_completion_never_produces_a_valid_packet);
  RUN_TEST(test_length_below_two_is_rejected);
  return UNITY_END();
}
