//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Plugins/ObjectFile/wasm/ObjectFileWasm.h"
#include "TestingSupport/SubsystemRAII.h"
#include "TestingSupport/TestUtilities.h"
#include "lldb/Core/Module.h"
#include "lldb/Host/FileSystem.h"
#include "lldb/Host/HostInfo.h"
#include "lldb/Utility/DataBufferHeap.h"
#include "llvm/BinaryFormat/Wasm.h"
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"

using namespace lldb_private;
using namespace lldb_private::wasm;
using namespace lldb;

class ObjectFileWasmTest : public testing::Test {
  SubsystemRAII<FileSystem, HostInfo, ObjectFileWasm> subsystems;
};

static llvm::Expected<addr_t>
GetFunctionFileAddress(ObjectFileWasm &object_file, uint64_t table_index) {
  llvm::Expected<Address> address =
      object_file.GetFunctionAtTableIndex(table_index);
  if (!address)
    return address.takeError();
  return address->GetFileAddress();
}

TEST_F(ObjectFileWasmTest, FunctionAtTableIndex) {
  // Function 0 is imported. The code section holds two empty functions:
  // function 1 at offset 2 and function 2 at offset 5, each after its size.
  auto file = TestFile::fromYaml(R"(
--- !WASM
FileHeader:
  Version:         0x1
Sections:
  - Type:            TYPE
    Signatures:
      - Index:           0
        ParamTypes:      []
        ReturnTypes:     []
  - Type:            IMPORT
    Imports:
      - Module:          env
        Field:           imported
        Kind:            FUNCTION
        SigIndex:        0
  - Type:            FUNCTION
    FunctionTypes:   [ 0, 0 ]
  - Type:            TABLE
    Tables:
      - Index:           0
        ElemType:        FUNCREF
        Limits:
          Minimum:         0x8
      - Index:           1
        ElemType:        FUNCREF
        Limits:
          Minimum:         0x8
  - Type:            ELEM
    Segments:
      - Offset:
          Opcode:          I32_CONST
          Value:           3
        Functions:       [ 2, 2, 0 ]
      # Table 1 is not the table of function pointers.
      - Flags:           2
        TableNumber:     1
        ElemKind:        FUNCREF
        Offset:
          Opcode:          I32_CONST
          Value:           3
        Functions:       [ 1 ]
      # Replaces the function at table index 4.
      - Offset:
          Opcode:          I32_CONST
          Value:           4
        Functions:       [ 1 ]
  - Type:            CODE
    Functions:
      - Index:           1
        Locals:          []
        Body:            0B
      - Index:           2
        Locals:          []
        Body:            0B
...
)");
  ASSERT_THAT_EXPECTED(file, llvm::Succeeded());
  auto module_sp = std::make_shared<Module>(file->moduleSpec());
  auto *object_file =
      llvm::dyn_cast_or_null<ObjectFileWasm>(module_sp->GetObjectFile());
  ASSERT_NE(object_file, nullptr);

  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 3),
                       llvm::HasValue(5));
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 4),
                       llvm::HasValue(2));
  EXPECT_THAT_EXPECTED(
      GetFunctionFileAddress(*object_file, 5),
      llvm::FailedWithMessage("table index 0x5 holds imported function 0"));
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 2),
                       llvm::FailedWithMessage(
                           "the module puts no function at table index 0x2"));
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 6),
                       llvm::FailedWithMessage(
                           "the module puts no function at table index 0x6"));
}

TEST_F(ObjectFileWasmTest, FunctionAtTableIndexWithRelocatableOffset) {
  // Position-independent code puts functions into the table at an offset that
  // an imported global gives.
  auto file = TestFile::fromYaml(R"(
--- !WASM
FileHeader:
  Version:         0x1
Sections:
  - Type:            TYPE
    Signatures:
      - Index:           0
        ParamTypes:      []
        ReturnTypes:     []
  - Type:            IMPORT
    Imports:
      - Module:          env
        Field:           __table_base
        Kind:            GLOBAL
        GlobalType:      I32
        GlobalMutable:   false
  - Type:            FUNCTION
    FunctionTypes:   [ 0 ]
  - Type:            TABLE
    Tables:
      - Index:           0
        ElemType:        FUNCREF
        Limits:
          Minimum:         0x10
  - Type:            ELEM
    Segments:
      - Offset:
          Opcode:          GLOBAL_GET
          Index:           0
        Functions:       [ 0 ]
      - Offset:
          Opcode:          I32_CONST
          Value:           10
        Functions:       [ 0 ]
  - Type:            CODE
    Functions:
      - Index:           0
        Locals:          []
        Body:            0B
...
)");
  ASSERT_THAT_EXPECTED(file, llvm::Succeeded());
  auto module_sp = std::make_shared<Module>(file->moduleSpec());
  auto *object_file =
      llvm::dyn_cast_or_null<ObjectFileWasm>(module_sp->GetObjectFile());
  ASSERT_NE(object_file, nullptr);

  // The later segment wins wherever it puts a function, whatever the earlier
  // one does.
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 10),
                       llvm::HasValue(2));
  EXPECT_THAT_EXPECTED(
      GetFunctionFileAddress(*object_file, 0),
      llvm::FailedWithMessage("the module puts functions into its table at an "
                              "offset that depends on module state"));
}

static void AppendSection(std::vector<uint8_t> &bytes, uint8_t id,
                          llvm::ArrayRef<uint8_t> payload) {
  assert(payload.size() < 0x80 && "the size has to fit in one ULEB byte");
  bytes.push_back(id);
  bytes.push_back(payload.size());
  bytes.insert(bytes.end(), payload.begin(), payload.end());
}

/// A module whose element section has the given payload. yaml2obj writes
/// only segments with an offset and function indices, so the bytes are built
/// here. Function 0 is imported. The code section holds two empty functions:
/// function 1 at offset 2 and function 2 at offset 5, each after its size.
static ModuleSP MakeModuleWithElements(llvm::ArrayRef<uint8_t> elements) {
  std::vector<uint8_t> bytes = {0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00};
  AppendSection(bytes, llvm::wasm::WASM_SEC_TYPE, {0x01, 0x60, 0x00, 0x00});
  AppendSection(bytes, llvm::wasm::WASM_SEC_IMPORT,
                {0x01, 0x03, 'e', 'n', 'v', 0x01, 'f', 0x00, 0x00});
  AppendSection(bytes, llvm::wasm::WASM_SEC_FUNCTION, {0x02, 0x00, 0x00});
  AppendSection(bytes, llvm::wasm::WASM_SEC_ELEM, elements);
  AppendSection(bytes, llvm::wasm::WASM_SEC_CODE,
                {0x02, 0x02, 0x00, 0x0b, 0x02, 0x00, 0x0b});
  auto buffer = std::make_shared<DataBufferHeap>(bytes.data(), bytes.size());
  return std::make_shared<Module>(
      ModuleSpec(FileSpec(), UUID(), std::make_shared<DataExtractor>(buffer)));
}

TEST_F(ObjectFileWasmTest, FunctionAtTableIndexForEachSegmentForm) {
  // Seven segments, one for each flags value but 0.
  ModuleSP module_sp = MakeModuleWithElements(
      {0x07,
       // Flags 1: passive, with an element kind.
       0x01, 0x00, 0x01, 0x01,
       // Flags 3: declarative, with an element kind.
       0x03, 0x00, 0x01, 0x02,
       // Flags 4: active in table 0 at 5, with expressions: ref.func 2 and
       // ref.null func.
       0x04, 0x41, 0x05, 0x0b, 0x02, 0xd2, 0x02, 0x0b, 0xd0, 0x70, 0x0b,
       // Flags 6: active in table 1, with a reference type and expressions.
       0x06, 0x01, 0x41, 0x00, 0x0b, 0x70, 0x01, 0xd2, 0x01, 0x0b,
       // Flags 5: passive, with a reference type and expressions.
       0x05, 0x70, 0x01, 0xd2, 0x01, 0x0b,
       // Flags 7: declarative, with a reference type and expressions.
       0x07, 0x70, 0x01, 0xd2, 0x01, 0x0b,
       // Flags 2: active in table 0 at 7, with an element kind: functions 1
       // and 9, which does not exist.
       0x02, 0x00, 0x41, 0x07, 0x0b, 0x00, 0x02, 0x01, 0x09});
  auto *object_file =
      llvm::dyn_cast_or_null<ObjectFileWasm>(module_sp->GetObjectFile());
  ASSERT_NE(object_file, nullptr);

  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 5),
                       llvm::HasValue(5));
  EXPECT_THAT_EXPECTED(
      GetFunctionFileAddress(*object_file, 6),
      llvm::FailedWithMessage("table index 0x6 holds no function"));
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 7),
                       llvm::HasValue(2));
  EXPECT_THAT_EXPECTED(
      GetFunctionFileAddress(*object_file, 8),
      llvm::FailedWithMessage(
          "table index 0x8 holds function 9, which has no code"));
  // The segments that put nothing into table 0 cover index 0.
  EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 0),
                       llvm::FailedWithMessage(
                           "the module puts no function at table index 0x0"));
}

TEST_F(ObjectFileWasmTest, FunctionAtTableIndexWithComputedOffset) {
  // i32.const 1; i32.const 2; i32.add starts with a constant, but its value is
  // 3, which the parser does not compute.
  ModuleSP module_sp = MakeModuleWithElements(
      {0x01, 0x00, 0x41, 0x01, 0x41, 0x02, 0x6a, 0x0b, 0x01, 0x01});
  auto *object_file =
      llvm::dyn_cast_or_null<ObjectFileWasm>(module_sp->GetObjectFile());
  ASSERT_NE(object_file, nullptr);

  EXPECT_THAT_EXPECTED(
      GetFunctionFileAddress(*object_file, 1),
      llvm::FailedWithMessage("the module puts functions into its table at an "
                              "offset that depends on module state"));
}

TEST_F(ObjectFileWasmTest, FunctionAtTableIndexWithMalformedElements) {
  // A section that does not parse puts nothing into the table, rather than
  // what it would put there if it were complete.
  const std::vector<std::vector<uint8_t>> sections = {
      // The second function index ends in the middle of its ULEB.
      {0x01, 0x00, 0x41, 0x03, 0x0b, 0x02, 0x01, 0x81},
      // The offset expression has no end.
      {0x01, 0x00, 0x41, 0x03},
      // Unknown segment flags.
      {0x01, 0x08, 0x41, 0x03, 0x0b, 0x01, 0x01},
      // i32.const is no element expression.
      {0x01, 0x04, 0x41, 0x03, 0x0b, 0x01, 0x41, 0x01, 0x0b},
  };
  for (const std::vector<uint8_t> &elements : sections) {
    ModuleSP module_sp = MakeModuleWithElements(elements);
    auto *object_file =
        llvm::dyn_cast_or_null<ObjectFileWasm>(module_sp->GetObjectFile());
    ASSERT_NE(object_file, nullptr);
    EXPECT_THAT_EXPECTED(GetFunctionFileAddress(*object_file, 3),
                         llvm::FailedWithMessage(
                             "the module puts no function at table index 0x3"));
  }
}
