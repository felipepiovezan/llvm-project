//===- llvm-cas-object-format.cpp - Tool for the CAS object format --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/CAS/MCCASObjectV1.h"
#include "llvm/Support/Error.h"

namespace llvm {
class raw_ostream;

namespace mccasformats {
namespace v1 {
struct MCCASPrinter {
  /// Creates a printer object capable of printing MCCAS objects inside `CAS`.
  /// Output is sent to `OS`.
  MCCASPrinter(cas::CASDB &CAS, raw_ostream &OS);

  /// If `CASObj` is an MCObject, prints its contents and all nodes referenced
  /// by it recursively. If CASObj or any of its children are not MCObjects, an
  /// error is returned.
  Error printMCObject(cas::ObjectRef CASObj);

  /// Prints the contents of `MCObject` and all nodes referenced by it
  /// recursively. If any of its children are not MCObjects, an error is
  /// returned.
  Error printMCObject(MCObjectProxy MCObj);

private:
  llvm::mccasformats::v1::MCSchema MCSchema;
  int Indent;
  raw_ostream &OS;

  Error printSimpleNested(MCObjectProxy AssemblerRef);
};
} // namespace v1
} // namespace mccasformats
} // namespace llvm
