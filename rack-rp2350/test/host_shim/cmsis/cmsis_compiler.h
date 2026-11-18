// Host-test shim: the GCC-generic subset of the CMSIS compiler layer the
// vendored DSP sources use on x86 (no ARM intrinsics — plain C builds).
#ifndef HOST_SHIM_CMSIS_COMPILER_H_
#define HOST_SHIM_CMSIS_COMPILER_H_
#define __ASM            __asm
#define __INLINE         inline
#define __STATIC_INLINE  static inline
#define __FORCE_INLINE   static inline __attribute__((always_inline))
#define __STATIC_FORCEINLINE static inline __attribute__((always_inline))
#define __NO_RETURN      __attribute__((__noreturn__))
#define __USED           __attribute__((used))
#define __WEAK           __attribute__((weak))
#define __PACKED         __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT  struct __attribute__((packed, aligned(1)))
#define __UNALIGNED_UINT32_READ(x) \
  (((union { uint8_t *b; uint32_t u; } *)(x))->u)
#define __UNALIGNED_UINT32_WRITE(x, v)                       \
  do { ((union { uint8_t *b; uint32_t u; } *)(x))->u = (v); } while (0)
#define __UNALIGNED_UINT16_READ(x) \
  (((union { uint8_t *b; uint16_t u; } *)(x))->u)
#define __UNALIGNED_UINT16_WRITE(x, v)                       \
  do { ((union { uint8_t *b; uint16_t u; } *)(x))->u = (v); } while (0)
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __CLZ(x) ((uint8_t)(__builtin_clz(x)))
#endif
