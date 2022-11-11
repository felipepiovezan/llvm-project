//===- MC/MCCASDebugV1.h --------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_MC_CAS_MCCASDEBUGV1_H
#define LLVM_MC_CAS_MCCASDEBUGV1_H

#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/Support/Error.h"

namespace llvm {
namespace mccasformats {
namespace v1 {

/// Returns true if the values associated with a combination of Form and Attr
/// are not expected to deduplicate.
bool doesntDedup(dwarf::Form Form, dwarf::Attribute Attr);

/// Reads data from `CUData[CUOffset]`, interpreting it as a value encoded as
/// `Form`, and returns the number of bytes taken by the encoded value.
Expected<uint64_t> getFormSize(dwarf::Form Form, dwarf::FormParams FP,
                               StringRef CUData, uint64_t CUOffset,
                               bool IsLittleEndian, uint8_t AddressSize);

/// A special value to indicate that a DIE has been placed in a separate CAS
/// block.
/// This should not conflict with any DWARF Tag values.
inline uint16_t getTagForDIEInAnotherBlock() { return 0xf0f0; }
} // namespace v1
} // namespace mccasformats
} // namespace llvm

#endif // LLVM_MC_CAS_MCCASDEBUGV1_H
