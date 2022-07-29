//===- llvm-cas-object-format.cpp - Tool for the CAS object format --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCCASPrinter.h"
#include "llvm/Support/FormatVariadic.h"

using namespace llvm;
using namespace llvm::cas;
using namespace llvm::mccasformats::v1;

namespace {
struct IndentGuard {
  constexpr static int IndentWidth = 2;
  IndentGuard(int &Indent) : Indent{Indent} { Indent += IndentWidth; }
  ~IndentGuard() { Indent -= IndentWidth; }
  int &Indent;
};
} // namespace

MCCASPrinter::MCCASPrinter(CASDB &CAS, raw_ostream &OS)
    : MCSchema(CAS), Indent{0}, OS(OS) {}

Error MCCASPrinter::printMCObject(ObjectRef CASObj) {
  // The object identifying the schema is not considered an MCObject, as such we
  // don't attempt to cast or print it.
  if (CASObj == MCSchema.getRootNodeTypeID())
    return Error::success();

  Expected<MCObjectProxy> MCObj = MCSchema.get(CASObj);
  if (!MCObj)
    return MCObj.takeError();
  return printMCObject(*MCObj);
}

Error MCCASPrinter::printMCObject(MCObjectProxy MCObj) {
  OS.indent(Indent);
  OS << formatv("{0, -15} {1} \n", MCObj.getKindString(),
                MCSchema.CAS.getID(MCObj));

  // TODO: Expand this list or allow it to be customized.
  if (MCAssemblerRef::Cast(MCObj) || GroupRef::Cast(MCObj) ||
      SectionRef::Cast(MCObj))
    return printSimpleNested(MCObj);
  return Error::success();
}

Error MCCASPrinter::printSimpleNested(MCObjectProxy AssemblerRef) {
  IndentGuard Guard(Indent);
  return AssemblerRef.forEachReference(
      [this](ObjectRef CASObj) { return printMCObject(CASObj); });
}
