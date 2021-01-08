// Copyright 2009 Ken Shirriff
// Copyright 2017 David Conran

/// @file
/// @brief RC-5 & RC-6 support
/// RC-5 & RC-6 support added from https://github.com/z3t0/Arduino-IRremote
/// RC-5X support added by David Conran
/// @see https://en.wikipedia.org/wiki/RC-5
/// @see http://www.sbprojects.com/knowledge/ir/rc5.php
/// @see https://en.wikipedia.org/wiki/Manchester_code
/// @see https://en.wikipedia.org/wiki/RC-6
/// @see https://www.sbprojects.net/knowledge/ir/rc6.php
/// @see http://www.pcbheaven.com/userpages/The_Philips_RC6_Protocol/
/// @see http://www.righto.com/2010/12/64-bit-rc6-codes-arduino-and-xbox.html

// Supports:
//   Brand: Philips,  Model: Standard RC-5 (RC5)
//   Brand: Philips,  Model: RC-5X (RC5X)

#include <algorithm>
#include "IRrecv.h"
#include "IRsend.h"
#include "IRtimer.h"
#include "IRutils.h"

// Constants
// RC-5/RC-5X
const uint16_t kRc5T1 = 889;
const uint32_t kRc5MinCommandLength = 113778;
const uint32_t kRc5MinGap = kRc5MinCommandLength - kRC5RawBits * (2 * kRc5T1);
const uint16_t kRc5ToggleMask = 0x800;  // The 12th bit.
const uint16_t kRc5SamplesMin = 11;

// Common (getRClevel())
const int16_t kMark = 0;
const int16_t kSpace = 1;

#if SEND_RC5
/// Send a Philips RC-5/RC-5X packet.
/// Status: RC-5 (stable), RC-5X (alpha)
/// @param[in] data The message to be sent.
/// @param[in] nbits The number of bits of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
/// @note Caller needs to take care of flipping the toggle bit.
///   That bit differentiates between key press & key release.
///   For RC-5 it is the MSB of the data.
///   For RC-5X it is the 2nd MSB of the data.
/// @todo Testing of the RC-5X components.
void IRsend::sendRC5(const uint64_t data, uint16_t nbits,
                     const uint16_t repeat) {
  if (nbits > sizeof(data) * 8) return;  // We can't send something that big.
  bool skipSpace = true;
  bool field_bit = true;
  // Set 36kHz IR carrier frequency & a 1/4 (25%) duty cycle.
  enableIROut(36, 25);

  if (nbits >= kRC5XBits) {  // Is this a RC-5X message?
    // field bit is the inverted MSB of RC-5X data.
    field_bit = ((data >> (nbits - 1)) ^ 1) & 1;
    nbits--;
  }

  IRtimer usecTimer = IRtimer();
  for (uint16_t i = 0; i <= repeat; i++) {
    usecTimer.reset();

    // Header
    // First start bit (0x1). space, then mark.
    if (skipSpace)
      skipSpace = false;  // First time through, we assume the leading space().
    else
      space(kRc5T1);
    mark(kRc5T1);
    // Field/Second start bit.
    if (field_bit) {  // Send a 1. Normal for RC-5.
      space(kRc5T1);
      mark(kRc5T1);
    } else {  // Send a 0. Special case for RC-5X. Means 7th command bit is 1.
      mark(kRc5T1);
      space(kRc5T1);
    }

    // Data
    for (uint64_t mask = 1ULL << (nbits - 1); mask; mask >>= 1)
      if (data & mask) {  // 1
        space(kRc5T1);    // 1 is space, then mark.
        mark(kRc5T1);
      } else {         // 0
        mark(kRc5T1);  // 0 is mark, then space.
        space(kRc5T1);
      }
    // Footer
    space(std::max(kRc5MinGap, kRc5MinCommandLength - usecTimer.elapsed()));
  }
}

/// Encode a Philips RC-5 data message.
/// Status: Beta / Should be working.
/// @param[in] address The 5-bit address value for the message.
/// @param[in] command The 6-bit command value for the message.
/// @param[in] key_released Indicate if the remote key has been released.
/// @return A message suitable for use in sendRC5().
uint16_t IRsend::encodeRC5(const uint8_t address, const uint8_t command,
                           const bool key_released) {
  return (key_released << (kRC5Bits - 1)) | ((address & 0x1f) << 6) |
         (command & 0x3F);
}

/// Encode a Philips RC-5X data message.
/// Status: Beta / Should be working.
/// @param[in] address The 5-bit address value for the message.
/// @param[in] command The 7-bit command value for the message.
/// @param[in] key_released Indicate if the remote key has been released.
/// @return A message suitable for use in sendRC5().
uint16_t IRsend::encodeRC5X(const uint8_t address, const uint8_t command,
                            const bool key_released) {
  // The 2nd start/field bit (MSB of the return value) is the value of the 7th
  // command bit.
  bool s2 = (command >> 6) & 1;
  return ((uint16_t)s2 << (kRC5XBits - 1)) |
         encodeRC5(address, command, key_released);
}

/// Flip the toggle bit of a Philips RC-5/RC-5X data message.
/// Used to indicate a change of remote button's state.
/// Status: STABLE.
/// @param[in] data The existing RC-5/RC-5X message.
/// @return A data message suitable for use in sendRC5() with the toggle bit
///   flipped.
uint64_t IRsend::toggleRC5(const uint64_t data) {
  return data ^ kRc5ToggleMask;
}
#endif  // SEND_RC5
