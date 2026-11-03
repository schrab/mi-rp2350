// Copyright 2021 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// User data manager.
//
// plaits-rp2350 port: the STM32 user-data flash page (0x08007000,
// stmlib/system/flash_programming.h) does not exist on RP2350, so every slot
// reads as "nothing transferred" — the documented LoadUserData(NULL) path.
// With PLAITS_HAS_USER_DATA_BANK=1 the three dx7-bank SixOpEngine slots then
// fall back to the factory banks (fm_patches_table[] = syx_bank_0/1/2 baked
// into resources.cc) in Voice::Render. Save() is gone with the flash: its only
// callers (ui.cc, user_data_receiver.cc) are not part of this port.

#ifndef PLAITS_USER_DATA_H_
#define PLAITS_USER_DATA_H_

#include "stmlib/stmlib.h"

namespace plaits {

class UserData {
 public:
  enum {
    // The stock firmware's legacy region address. Kept for interface
    // compatibility; never dereferenced on this target.
    ADDRESS = 0x08007000,
    SIZE = 0x1000
  };

  UserData() { }
  ~UserData() { }

  // No readable flash region behind any slot: a TIMBRE transfer never took
  // place, so callers fall back to their factory bank data (if any).
  inline const uint8_t* ptr(int slot) const {
    (void) slot;
    return NULL;
  }

  // Bytes of packed patch data behind a bank buffer.
  //
  // The 4096 bytes are fully occupied by 32 packed voices, so a bank that
  // holds fewer has to declare it somewhere; the count lives in the tail of
  // voice 32's name field, which Plaits never displays. Without it every
  // transferred bank reads as a full 32 — an 8-patch pick list had to repeat
  // four times to fill the bank, and HARMONICS swept 8 zones of 4 instead of 8
  // steps, visibly contradicting the short BAKED banks shipped since recipe
  // schema v13.
  //
  //   SIZE-4  0xa5   magic
  //   SIZE-3  count  1..32
  //   SIZE-2  'U'    tag (written by Save)
  //   SIZE-1  bank   tag (written by Save)
  //
  // The magic is not optional. Without it, a payload from an encoder that
  // predates the count carries an arbitrary name character at SIZE-3; printable
  // ASCII is 32..127, so a stray 1..31 would be read as a short bank. Requiring
  // 0xa5 makes "absent" unambiguous, so an OLD file on NEW firmware still reads
  // as 32 — and Save rewrites only SIZE-2/SIZE-1, so both count bytes survive
  // into flash untouched.
  static inline size_t bank_length(const uint8_t* data) {
    const uint8_t kCountMagic = 0xa5;
    const size_t kPackedPatchSize = 128;  // fm::Patch::SYX_SIZE
    const int kMaxPatchesPerBank = SIZE / kPackedPatchSize;
    if (data && data[SIZE - 4] == kCountMagic) {
      const int count = data[SIZE - 3];
      if (count >= 1 && count <= kMaxPatchesPerBank) {
        return count * kPackedPatchSize;
      }
    }
    return SIZE;
  }
};

}  // namespace plaits

#endif  // PLAITS_USER_DATA_H_
