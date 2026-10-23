#include <unity.h>

#include <vector>

#include "../packets.h"
#include "watchdog.h"

namespace {

constexpr uint32_t kTimeout = 500;

void send(LinkWatchdog& w, const std::vector<uint8_t>& bytes, uint32_t now) {
  w.on_pc_bytes(bytes.data(), bytes.size(), now);
}

void test_unarmed_never_fires() {
  LinkWatchdog w(kTimeout);
  send(w, packet(1, scs::kInstPing, {}), 0);
  TEST_ASSERT_FALSE(w.should_fire(100000));
  w.on_link_lost();
  TEST_ASSERT_FALSE(w.should_fire(100000));
}

void test_fires_after_timeout_only() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 1000);
  TEST_ASSERT_TRUE(w.armed());
  TEST_ASSERT_FALSE(w.should_fire(1000 + kTimeout - 1));
  TEST_ASSERT_TRUE(w.should_fire(1000 + kTimeout));
}

void test_any_pc_traffic_restarts_the_timer() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 1000);
  send(w, packet(2, 0x02, {40, 27}), 1400);  // a READ keeps the link alive
  TEST_ASSERT_FALSE(w.should_fire(1800));
  TEST_ASSERT_TRUE(w.should_fire(1900));
}

void test_link_lost_fires_at_once_when_armed() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 42, {0x01, 0xF4, 0, 0, 0, 50}), 1000);
  w.on_link_lost();
  TEST_ASSERT_TRUE(w.should_fire(1001));
}

void test_torque_off_from_the_pc_disarms() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 1000);
  send(w, sync_write(40, 1, {{2, 0}}), 1010);
  w.on_link_lost();
  TEST_ASSERT_FALSE(w.should_fire(5000));
}

void test_fire_emits_broadcast_and_latches_until_pc_returns() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 1000);
  uint8_t out[LinkWatchdog::kFireMaxLen];
  const size_t n = w.fire(out, sizeof(out));
  uint8_t expected[scs::kTorqueOffPacketLen];
  scs::torque_off_broadcast(expected);
  TEST_ASSERT_EQUAL(scs::kTorqueOffPacketLen, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, out, n);
  TEST_ASSERT_FALSE(w.armed());
  TEST_ASSERT_TRUE(w.tripped());
  TEST_ASSERT_FALSE(w.should_fire(100000));
  send(w, packet(2, scs::kInstPing, {}), 2000);
  TEST_ASSERT_FALSE(w.tripped());
}

void test_fire_mid_packet_completes_it_first() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 1000);
  const auto goal = write(2, 42, {0x01, 0xF4, 0, 0, 0, 50});
  w.on_pc_bytes(goal.data(), 6, 1010);  // PC died mid-packet
  uint8_t out[LinkWatchdog::kFireMaxLen];
  const size_t n = w.fire(out, sizeof(out));
  const size_t completion = goal.size() - 6;
  TEST_ASSERT_EQUAL(completion + scs::kTorqueOffPacketLen, n);
  TEST_ASSERT_TRUE(w.sniffer().at_boundary());
  TEST_ASSERT_FALSE(w.armed());
  // Last completion byte is the deliberately wrong checksum.
  TEST_ASSERT_NOT_EQUAL(goal.back(), out[completion - 1]);
}

void test_millis_wrap() {
  LinkWatchdog w(kTimeout);
  send(w, write(2, 40, {1}), 0xFFFFFF00u);
  TEST_ASSERT_FALSE(w.should_fire(0xFFFFFF00u + 100));
  TEST_ASSERT_TRUE(w.should_fire(0x00000100u));  // 0x200 = 512 ms later
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_unarmed_never_fires);
  RUN_TEST(test_fires_after_timeout_only);
  RUN_TEST(test_any_pc_traffic_restarts_the_timer);
  RUN_TEST(test_link_lost_fires_at_once_when_armed);
  RUN_TEST(test_torque_off_from_the_pc_disarms);
  RUN_TEST(test_fire_emits_broadcast_and_latches_until_pc_returns);
  RUN_TEST(test_fire_mid_packet_completes_it_first);
  RUN_TEST(test_millis_wrap);
  return UNITY_END();
}
