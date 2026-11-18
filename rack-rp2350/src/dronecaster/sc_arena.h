// sc_arena.h — bump allocator over the engine's SRAM carve. Reset per voice,
// with a Mark/ResetTo savepoint so the kit tables (built once) survive swaps.
#ifndef RACK_SRC_DRONECASTER_SC_ARENA_H_
#define RACK_SRC_DRONECASTER_SC_ARENA_H_
#include <cstddef>
#include <cstdint>

namespace sc {

class Arena {
 public:
  void Init(uint8_t* base, size_t bytes) { base_ = base; cap_ = bytes; off_ = 0; }
  void* Alloc(size_t bytes, size_t align = 8) {
    off_ = (off_ + align - 1) & ~(align - 1);
    if (off_ + bytes > cap_) return nullptr;   // caller asserts capacity
    void* p = base_ + off_;
    off_ += bytes;
    return p;
  }
  size_t Mark() const { return off_; }
  void ResetTo(size_t mark) { off_ = mark; }
  void Reset() { off_ = 0; }
  size_t used() const { return off_; }
  size_t capacity() const { return cap_; }

 private:
  uint8_t* base_ = nullptr;
  size_t cap_ = 0, off_ = 0;
};

}  // namespace sc
#endif
