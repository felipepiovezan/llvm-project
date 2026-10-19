//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_UTILITY_WASMVIRTUALREGISTERS_H
#define LLDB_SOURCE_UTILITY_WASMVIRTUALREGISTERS_H

#include "lldb/lldb-private.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/LEB128.h"

namespace lldb_private {

// LLDB doesn't have an address space to represents WebAssembly locals,
// globals and operand stacks. We encode these elements into virtual
// registers:
//
//   | tag: 2 bits | index: 30 bits |
//
// Where tag is:
//    0: Not a Wasm location
//    1: Local
//    2: Global
//    3: Operand stack value
enum WasmVirtualRegisterKinds {
  eWasmTagNotAWasmLocation = 0,
  eWasmTagLocal = 1,
  eWasmTagGlobal = 2,
  eWasmTagOperandStack = 3,
};

static const uint32_t kWasmVirtualRegisterTagMask = 0x03;
static const uint32_t kWasmVirtualRegisterIndexMask = 0x3fffffff;
static const uint32_t kWasmVirtualRegisterTagShift = 30;

inline uint32_t GetWasmVirtualRegisterTag(size_t reg) {
  return (reg >> kWasmVirtualRegisterTagShift) & kWasmVirtualRegisterTagMask;
}

inline uint32_t GetWasmVirtualRegisterIndex(size_t reg) {
  return reg & kWasmVirtualRegisterIndexMask;
}

inline uint32_t GetWasmRegister(uint8_t tag, uint32_t index) {
  return ((tag & kWasmVirtualRegisterTagMask) << kWasmVirtualRegisterTagShift) |
         (index & kWasmVirtualRegisterIndexMask);
}

/// Returns true if \p inst, the bytes of one WebAssembly instruction, is a
/// local.set or local.tee of local \p index.
inline bool IsWasmLocalWrite(llvm::ArrayRef<uint8_t> inst, uint32_t index) {
  constexpr uint8_t local_set = 0x21;
  constexpr uint8_t local_tee = 0x22;
  if (inst.empty() || (inst[0] != local_set && inst[0] != local_tee))
    return false;
  const char *error = nullptr;
  uint64_t local = llvm::decodeULEB128(inst.data() + 1, nullptr,
                                       inst.data() + inst.size(), &error);
  return !error && local == index;
}

} // namespace lldb_private

#endif
