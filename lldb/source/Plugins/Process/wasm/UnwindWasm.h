//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_PROCESS_WASM_UNWINDWASM_H
#define LLDB_SOURCE_PLUGINS_PROCESS_WASM_UNWINDWASM_H

#include "lldb/Target/RegisterContext.h"
#include "lldb/Target/Unwind.h"
#include <optional>
#include <vector>

namespace lldb_private {
namespace wasm {

class ThreadWasm;

/// UnwindWasm manages stack unwinding for a WebAssembly process.
class UnwindWasm : public lldb_private::Unwind {
public:
  UnwindWasm(lldb_private::Thread &thread) : Unwind(thread) {}
  ~UnwindWasm() override = default;

  /// Returns the PC of frame \p engine_frame_idx of the engine's call stack, or
  /// LLDB_INVALID_ADDRESS.
  lldb::addr_t GetEngineFramePC(uint32_t engine_frame_idx);

protected:
  void DoClear() override {
    m_engine_pcs.reset();
    m_frames.clear();
    m_next_frame.reset();
    m_unwind_started = false;
  }

  uint32_t DoGetFrameCount() override;

  bool DoGetFrameInfoAtIndex(uint32_t frame_idx, lldb::addr_t &cfa,
                             lldb::addr_t &pc,
                             bool &behaves_like_zeroth_frame) override;

  lldb::RegisterContextSP
  DoCreateRegisterContextForFrame(lldb_private::StackFrame *frame) override;

private:
  struct Frame {
    lldb::addr_t pc;
    lldb::addr_t cfa;
    bool behaves_like_zeroth_frame;
    lldb::RegisterContextSP reg_ctx;
    /// The index of the frame in the engine's call stack, if it is there.
    std::optional<uint32_t> engine_frame_idx;
  };

  /// The thread whose call stack the engine has: the thread or, for a thread
  /// that an OS plugin makes, the thread backing it.
  ThreadWasm &GetWasmThread();

  /// The engine's call stack, which is fetched once per stop.
  const std::vector<lldb::addr_t> &GetEngineCallStack();

  Frame MakeEngineFrame(uint32_t engine_frame_idx);

  /// Adds \p frame, unless nothing knows its CFA, and returns its caller.
  std::optional<Frame> AddFrame(Frame frame);

  /// Unwinds until frame \p frame_idx exists or the stack ends.
  void UnwindTo(uint32_t frame_idx);

  std::optional<std::vector<lldb::addr_t>> m_engine_pcs;
  std::vector<Frame> m_frames;
  /// The caller of the last frame.
  std::optional<Frame> m_next_frame;
  bool m_unwind_started = false;

  UnwindWasm(const UnwindWasm &);
  const UnwindWasm &operator=(const UnwindWasm &) = delete;
};

} // namespace wasm
} // namespace lldb_private

#endif
