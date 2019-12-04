/*********************************************************************
 * C++ unit test for dispatch.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include "../src/dispatch.h"

// Test verifying packet sync byte
TEST(PACKET, sync)
{
  // Test the validatePacket() function
  Packet packet;
  memset(&packet, 0x00, sizeof(packet));
  packet.sync = 0xE7 + 1;
  while (packet.sync != 0xE7) {
    ASSERT_FALSE(validatePacket(&packet));
    packet.sync++;
  }
  ASSERT_TRUE(validatePacket(&packet));

  // Call the unused dispatchAssertSizes() function to get full test coverage
  dispatchAssertSizes();
}

// Test verifying packet checksum
TEST(PACKET, chksum3)
{
  Packet packet;

  // Packet of all zero
  memset(&packet, 0x00, sizeof(packet));
  packet.sync = 0xE7;
  packet.chksum3 = 0x00;
  ASSERT_TRUE(validatePacket(&packet));
  packet.chksum3++;
  while (packet.chksum3 != 0x00) {
    ASSERT_FALSE(validatePacket(&packet));
    packet.chksum3++;
  }

  // Packet of all 13
  memset(&packet, 13, sizeof(packet));
  packet.sync = 0xE7;
  packet.chksum3 = (uint8_t)(13*70);
  ASSERT_TRUE(validatePacket(&packet));
  packet.chksum3++;
  while (packet.chksum3 != (uint8_t)(13*70)) {
    ASSERT_FALSE(validatePacket(&packet));
    packet.chksum3++;
  }

  // Packet of all 191
  memset(&packet, 191, sizeof(packet));
  packet.sync = 0xE7;
  packet.chksum3 = (uint8_t)(191*70);
  ASSERT_TRUE(validatePacket(&packet));
  packet.chksum3++;
  while (packet.chksum3 != (uint8_t)(191*70)) {
    ASSERT_FALSE(validatePacket(&packet));
    packet.chksum3++;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

