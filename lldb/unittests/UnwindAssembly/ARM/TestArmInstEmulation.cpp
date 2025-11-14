//===-- TestArmInstEmulation.cpp ----------------------------------------===//

//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "gtest/gtest.h"

#include "Plugins/UnwindAssembly/InstEmulation/UnwindAssemblyInstEmulation.h"

#include "lldb/Core/AddressRange.h"
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Utility/ArchSpec.h"

#include "Plugins/Disassembler/LLVMC/DisassemblerLLVMC.h"
#include "Plugins/Instruction/ARM/EmulateInstructionARM.h"
#include "llvm/Support/TargetSelect.h"

using namespace lldb;
using namespace lldb_private;

class TestArmInstEmulation : public testing::Test {
public:
  static void SetUpTestCase();
  static void TearDownTestCase();
};

void TestArmInstEmulation::SetUpTestCase() {
  llvm::InitializeAllTargets();
  llvm::InitializeAllAsmPrinters();
  llvm::InitializeAllTargetMCs();
  llvm::InitializeAllDisassemblers();
  DisassemblerLLVMC::Initialize();
  EmulateInstructionARM::Initialize();
}

void TestArmInstEmulation::TearDownTestCase() {
  DisassemblerLLVMC::Terminate();
  EmulateInstructionARM::Terminate();
}

TEST_F(TestArmInstEmulation, TestBranchSigned) {
  ArchSpec arch("arm-apple-ios15");
  std::unique_ptr<UnwindAssemblyInstEmulation> engine(
      static_cast<UnwindAssemblyInstEmulation *>(
          UnwindAssemblyInstEmulation::CreateInstance(arch)));
  ASSERT_NE(nullptr, engine);

  AddressRange sample_range;
  UnwindPlan unwind_plan(eRegisterKindLLDB);
  UnwindPlan::Row::AbstractRegisterLocation regloc;

  uint8_t data[] = {
      0x14, 0xd0, 0x4d, 0xe2, // <+0>:   sub    sp, sp, #20
      0x00, 0x20, 0xa0, 0xe3, // <+4>:   mov    r2, #0
      0x10, 0x20, 0x8d, 0xe5, // <+8>:   str    r2, [sp, #0x10]
      0x0c, 0x00, 0x8d, 0xe5, // <+12>:  str    r0, [sp, #0xc]
      0x00, 0x00, 0x00, 0xea, // <+16>:  b      <+24>
      0xff, 0xff, 0xff, 0xea, // <+20>:  b      <+24>
      0x10, 0x00, 0x9d, 0xe5, // <+24>:  ldr    r0, [sp, #0x10]
      0x14, 0xd0, 0x8d, 0xe2, // <+28>:  add    sp, sp, #20
      0x1e, 0xff, 0x2f, 0xe1, // <+32>:  bx     lr
  };

  sample_range = AddressRange(0x1000, sizeof(data));
  EXPECT_TRUE(engine->GetNonCallSiteUnwindPlanFromAssembly(
      sample_range, data, sizeof(data), unwind_plan));

  // Confirm CFA before epilogue instructions is in terms of $fp
  const UnwindPlan::Row *row = unwind_plan.GetRowForFunctionOffset(16);
  row = unwind_plan.GetRowForFunctionOffset(20);
}
