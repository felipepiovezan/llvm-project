//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "UnwindWasm.h"
#include "Plugins/Process/gdb-remote/ThreadGDBRemote.h"
#include "ProcessWasm.h"
#include "RegisterContextWasm.h"
#include "ThreadWasm.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"

using namespace lldb;
using namespace lldb_private;
using namespace process_gdb_remote;
using namespace wasm;

/// Engine frames have no address, so their CFAs are in a space without memory.
static constexpr lldb::addr_t kWasmSyntheticCFABase =
    uint64_t(WasmAddressType::Invalid) << kWasmAddressTypeShift;

lldb::RegisterContextSP
UnwindWasm::DoCreateRegisterContextForFrame(lldb_private::StackFrame *frame) {
  const uint32_t concrete_frame_idx = frame->GetConcreteFrameIndex();
  UnwindTo(concrete_frame_idx);
  if (m_frames.size() <= concrete_frame_idx)
    return lldb::RegisterContextSP();

  ThreadSP thread = frame->GetThread();
  ThreadGDBRemote *gdb_thread = static_cast<ThreadGDBRemote *>(thread.get());
  ProcessWasm *wasm_process =
      static_cast<ProcessWasm *>(thread->GetProcess().get());

  return std::make_shared<RegisterContextWasm>(*gdb_thread, concrete_frame_idx,
                                               wasm_process->GetRegisterInfo());
}

const std::vector<lldb::addr_t> &UnwindWasm::GetEngineCallStack() {
  if (m_engine_pcs)
    return *m_engine_pcs;

  ThreadWasm &wasm_thread = static_cast<ThreadWasm &>(GetThread());
  llvm::Expected<std::vector<lldb::addr_t>> call_stack_pcs =
      wasm_thread.GetWasmCallStack();
  if (!call_stack_pcs) {
    LLDB_LOG_ERROR(GetLog(LLDBLog::Unwind), call_stack_pcs.takeError(),
                   "Failed to get Wasm callstack: {0}");
    return m_engine_pcs.emplace();
  }

  return m_engine_pcs.emplace(std::move(*call_stack_pcs));
}

void UnwindWasm::UnwindTo(uint32_t frame_idx) {
  const std::vector<lldb::addr_t> &engine_pcs = GetEngineCallStack();
  while (m_frames.size() <= frame_idx && m_frames.size() < engine_pcs.size()) {
    const uint32_t engine_frame_idx = m_frames.size();
    // The distance to the outermost frame does not change as frames are pushed
    // and popped above it.
    m_frames.push_back(
        {engine_pcs[engine_frame_idx],
         kWasmSyntheticCFABase + engine_pcs.size() - 1 - engine_frame_idx});
  }
}

uint32_t UnwindWasm::DoGetFrameCount() {
  // A backtrace goes no deeper than the target asks for, which is what bounds
  // the walk of a stack that recurses without end.
  const uint32_t max_depth = GetThread().GetMaxBacktraceDepth();
  if (max_depth != 0)
    UnwindTo(max_depth - 1);
  return std::min<uint64_t>(m_frames.size(), max_depth);
}

bool UnwindWasm::DoGetFrameInfoAtIndex(uint32_t frame_idx, lldb::addr_t &cfa,
                                       lldb::addr_t &pc,
                                       bool &behaves_like_zeroth_frame) {
  if (frame_idx >= GetThread().GetMaxBacktraceDepth())
    return false;

  UnwindTo(frame_idx);
  if (frame_idx >= m_frames.size())
    return false;

  behaves_like_zeroth_frame = (frame_idx == 0);
  cfa = m_frames[frame_idx].cfa;
  pc = m_frames[frame_idx].pc;
  return true;
}

lldb::addr_t UnwindWasm::GetEngineFramePC(uint32_t engine_frame_idx) {
  std::lock_guard<std::recursive_mutex> guard(m_unwind_mutex);
  const std::vector<lldb::addr_t> &engine_pcs = GetEngineCallStack();
  if (engine_frame_idx >= engine_pcs.size())
    return LLDB_INVALID_ADDRESS;
  return engine_pcs[engine_frame_idx];
}
