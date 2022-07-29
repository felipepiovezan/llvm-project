//===- llvm-cas-object-format.cpp - Tool for the CAS object format --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCCASPrinter.h"
#include "llvm/MC/CAS/MCCASObjectV1.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/raw_ostream.h"
#include <memory>

using namespace llvm;
using namespace llvm::cas;
using namespace llvm::mccasformats::v1;

cl::opt<std::string> CASPath("cas", cl::Required,
                             cl::desc("Path to CAS on disk."));
cl::list<std::string> InputFiles(cl::Positional,
                                 cl::desc("CAS ID object to print"));

int main(int argc, char *argv[]) {
  ExitOnError ExitOnErr;
  ExitOnErr.setBanner(std::string(argv[0]) + ": ");

  cl::ParseCommandLineOptions(argc, argv);

  std::unique_ptr<CASDB> CAS = ExitOnErr(createOnDiskCAS(CASPath));
  MCCASPrinter Printer(*CAS, llvm::outs());

  for (StringRef IF : InputFiles) {
    auto ID = ExitOnErr(CAS->parseID(IF));
    auto Ref = CAS->getReference(ID);
    if (!Ref) {
      llvm::errs() << "invalid ref\n";
      return 1;
    }
    ExitOnErr(Printer.printMCObject(*Ref));
  }
  return 0;
}
