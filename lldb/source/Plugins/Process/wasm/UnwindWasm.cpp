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
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Target/LanguageRuntime.h"
#include "lldb/Target/Process.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"
#include "lldb/Utility/RegisterValue.h"

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
  return m_frames[concrete_frame_idx].reg_ctx;
}

ThreadWasm &UnwindWasm::GetWasmThread() {
  ThreadSP backing_thread = GetThread().GetBackingThread();
  return static_cast<ThreadWasm &>(backing_thread ? *backing_thread
                                                  : GetThread());
}

const std::vector<lldb::addr_t> &UnwindWasm::GetEngineCallStack() {
  if (m_engine_pcs)
    return *m_engine_pcs;

  ThreadWasm &wasm_thread = GetWasmThread();
  llvm::Expected<std::vector<lldb::addr_t>> call_stack_pcs =
      wasm_thread.GetWasmCallStack();
  if (!call_stack_pcs) {
    LLDB_LOG_ERROR(GetLog(LLDBLog::Unwind), call_stack_pcs.takeError(),
                   "Failed to get Wasm callstack: {0}");
    return m_engine_pcs.emplace();
  }

  return m_engine_pcs.emplace(std::move(*call_stack_pcs));
}

UnwindWasm::Frame UnwindWasm::MakeEngineFrame(uint32_t engine_frame_idx) {
  ThreadWasm &wasm_thread = GetWasmThread();
  ProcessWasm *wasm_process =
      static_cast<ProcessWasm *>(wasm_thread.GetProcess().get());
  const std::vector<lldb::addr_t> &engine_pcs = GetEngineCallStack();
  Frame frame;
  frame.pc = engine_pcs[engine_frame_idx];
  // The distance to the outermost frame does not change as frames are pushed
  // and popped above it.
  frame.cfa = kWasmSyntheticCFABase + engine_pcs.size() - 1 - engine_frame_idx;
  frame.behaves_like_zeroth_frame = engine_frame_idx == 0;
  frame.reg_ctx = std::make_shared<RegisterContextWasm>(
      wasm_thread, engine_frame_idx, wasm_process->GetRegisterInfo());
  frame.engine_frame_idx = engine_frame_idx;
  return frame;
}

/// Reads the CFA rules of the UnwindPlans of language runtimes.
static std::optional<lldb::addr_t>
ReadCFA(const UnwindPlan::Row::FAValue &cfa_value, lldb::RegisterKind kind,
        RegisterContext &regctx) {
  switch (cfa_value.GetValueType()) {
  case UnwindPlan::Row::FAValue::isConstant:
    return cfa_value.GetConstant();
  case UnwindPlan::Row::FAValue::isRegisterDereferenced: {
    const RegisterInfo *reg_info = regctx.GetRegisterInfoAtIndex(
        regctx.ConvertRegisterKindToRegisterNumber(
            kind, cfa_value.GetRegisterNumber()));
    RegisterValue value;
    if (!reg_info || !regctx.ReadRegister(reg_info, value))
      return std::nullopt;
    llvm::Expected<lldb::addr_t> cfa =
        regctx.GetThread().GetProcess()->ReadPointerFromMemory(
            value.GetAsUInt64());
    if (!cfa) {
      LLDB_LOG_ERROR(GetLog(LLDBLog::Unwind), cfa.takeError(),
                     "Failed to read the CFA: {0}");
      return std::nullopt;
    }
    return *cfa;
  }
  default:
    return std::nullopt;
  }
}

std::optional<UnwindWasm::Frame> UnwindWasm::AddFrame(Frame frame) {
  if (UnwindPlanSP plan = LanguageRuntime::GetRuntimeUnwindPlan(
          GetThread(), frame.reg_ctx.get(), frame.behaves_like_zeroth_frame)) {
    // The runtime makes its UnwindPlan for the PC of the frame.
    const UnwindPlan::Row *row = plan->GetLastRow();
    if (std::optional<lldb::addr_t> cfa =
            row ? ReadCFA(row->GetCFAValue(), plan->GetRegisterKind(),
                          *frame.reg_ctx)
                : std::nullopt) {
      frame.cfa = *cfa;
      m_frames.push_back(frame);

      ThreadWasm &wasm_thread = GetWasmThread();
      ProcessWasm *wasm_process =
          static_cast<ProcessWasm *>(wasm_thread.GetProcess().get());
      Frame caller;
      caller.reg_ctx = std::make_shared<RegisterContextWasmUnwindRow>(
          wasm_thread, m_frames.size(), wasm_process->GetRegisterInfo(), *row,
          plan->GetRegisterKind(), *cfa);
      caller.pc = caller.reg_ctx->GetPC();
      caller.cfa = LLDB_INVALID_ADDRESS;
      caller.behaves_like_zeroth_frame = false;
      if (caller.pc == LLDB_INVALID_ADDRESS)
        return std::nullopt;
      return caller;
    }
  }

  // Without a runtime, only the engine knows a caller, as UnwindLLDB has no
  // UnwindPlan for WebAssembly.
  if (!frame.engine_frame_idx)
    return std::nullopt;
  m_frames.push_back(frame);
  const uint32_t caller_engine_frame_idx = *frame.engine_frame_idx + 1;
  if (caller_engine_frame_idx >= GetEngineCallStack().size())
    return std::nullopt;
  return MakeEngineFrame(caller_engine_frame_idx);
}

void UnwindWasm::UnwindTo(uint32_t frame_idx) {
  if (!m_unwind_started) {
    m_unwind_started = true;
    if (!GetEngineCallStack().empty())
      m_next_frame = MakeEngineFrame(0);
  }
  while (m_frames.size() <= frame_idx && m_next_frame)
    m_next_frame = AddFrame(std::move(*m_next_frame));
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

  behaves_like_zeroth_frame = m_frames[frame_idx].behaves_like_zeroth_frame;
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
