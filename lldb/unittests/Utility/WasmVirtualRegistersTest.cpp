//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Utility/WasmVirtualRegisters.h"
#include "gtest/gtest.h"

using namespace lldb_private;

TEST(WasmVirtualRegistersTest, IsWasmLocalWrite) {
  // local.set 0 and local.tee 0.
  EXPECT_TRUE(IsWasmLocalWrite({0x21, 0x00}, 0));
  EXPECT_TRUE(IsWasmLocalWrite({0x22, 0x00}, 0));
  // A padded LEB128 index.
  EXPECT_TRUE(IsWasmLocalWrite({0x21, 0x80, 0x80, 0x00}, 0));
  // local.set 128.
  EXPECT_TRUE(IsWasmLocalWrite({0x21, 0x80, 0x01}, 128));
  EXPECT_FALSE(IsWasmLocalWrite({0x21, 0x80, 0x01}, 0));
  // local.set 4.
  EXPECT_FALSE(IsWasmLocalWrite({0x21, 0x04}, 0));
  // local.get 0 and global.set 0.
  EXPECT_FALSE(IsWasmLocalWrite({0x20, 0x00}, 0));
  EXPECT_FALSE(IsWasmLocalWrite({0x24, 0x00}, 0));
  // Truncated instructions.
  EXPECT_FALSE(IsWasmLocalWrite({}, 0));
  EXPECT_FALSE(IsWasmLocalWrite({0x21}, 0));
  EXPECT_FALSE(IsWasmLocalWrite({0x21, 0x80}, 0));
}
