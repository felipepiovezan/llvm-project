//===-- OperatingSystemSwift.cpp -----------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "lldb/Host/Config.h"

#if LLDB_ENABLE_SWIFT

#include "OperatingSystemSwift.h"

#include "Plugins/LanguageRuntime/Swift/ReflectionContextInterface.h"
#include "Plugins/LanguageRuntime/Swift/SwiftLanguageRuntime.h"
#include "Plugins/Process/Utility/ThreadMemory.h"
#include "lldb/Core/Debugger.h"
#include "lldb/Core/Module.h"
#include "lldb/Core/PluginManager.h"
#include "lldb/Target/Process.h"
#include "lldb/Target/Thread.h"
#include "lldb/Target/ThreadList.h"
#include "lldb/Target/ThreadPlanStack.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/StructuredData.h"

#include "swift/Threading/ThreadLocalStorage.h"

#include <memory>

using namespace lldb;
using namespace lldb_private;

LLDB_PLUGIN_DEFINE(OperatingSystemSwift)

std::optional<uint64_t>
OperatingSystemSwift::FindTaskId(std::optional<size_t> task_id_offset,
                                 Thread &thread) {
  if (!task_id_offset)
    return {};
  StructuredData::ObjectSP info_root_sp = thread.GetExtendedInfo();
  if (!info_root_sp)
    return {};

  StructuredData::ObjectSP node =
      info_root_sp->GetObjectForDotSeparatedPath("tsd_address");
  if (!node)
    return {};
  StructuredData::UnsignedInteger *raw_tsd_addr = node->GetAsUnsignedInteger();
  if (!raw_tsd_addr)
    return {};
  addr_t tsd_addr = raw_tsd_addr->GetUnsignedIntegerValue();
  addr_t task_addr_location = tsd_addr + m_task_ptr_offset;
  Status error;
  addr_t task_addr =
      m_process->ReadPointerFromMemory(task_addr_location, error);
  if (error.Fail())
    return {};

  auto task_id = m_process->ReadUnsignedIntegerFromMemory(
      task_addr + *task_id_offset, /*integer width*/ 4, LLDB_INVALID_ADDRESS,
      error);
  if (error.Fail())
    return {};
  return task_id;
}

void OperatingSystemSwift::Initialize() {
  PluginManager::RegisterPlugin(GetPluginNameStatic(),
                                GetPluginDescriptionStatic(), CreateInstance,
                                nullptr);
}

void OperatingSystemSwift::Terminate() {
  PluginManager::UnregisterPlugin(CreateInstance);
}

OperatingSystem *OperatingSystemSwift::CreateInstance(Process *process,
                                                      bool force) {
  if (!process)
    return nullptr;
  return new OperatingSystemSwift(*process);
}

llvm::StringRef OperatingSystemSwift::GetPluginDescriptionStatic() {
  return "Operating system plug-in converts Swift Tasks into Threads from the "
         "point of view of LLDB.";
}

OperatingSystemSwift::OperatingSystemSwift(lldb_private::Process &process)
    : OperatingSystem(&process) {
  m_task_ptr_offset = swift::tls_get_key(swift::tls_key::concurrency_task) *
                      process.GetAddressByteSize();
}

OperatingSystemSwift::~OperatingSystemSwift() = default;

void OperatingSystemSwift::PrunePlansForNonBackedThreads(
    ThreadPlanStackMap &plan_stack_map, ThreadList &threads) {
  for (auto tid : plan_stack_map.GetKnownTIDs()) {
    // If there is a thread running with this TID, keep its ThreadPlanStack
    // alive.
    ThreadSP thread = threads.FindThreadByID(tid, false /*can_update=*/);
    if (thread)
      continue;

    // The thread is not currently active. If it is a Task Thread created by
    // this plugin, keep it alive.
    if (m_task_tids.contains(tid))
      continue;
    // Otherwise, it is a core thread no longer active. Prune it.
    plan_stack_map.RemoveTID(tid);
    LLDB_LOGF(GetLog(LLDBLog::OS),
              "%s: deleting "
              "plan for core thread with tid = %" PRIx64,
              LLVM_PRETTY_FUNCTION, tid);
  }
}

void OperatingSystemSwift::WillUpdateThreadList() {
  // If m_task_id_offset was computed previously, just return.
  if (m_task_id_offset)
    return;
  Log *log = GetLog(LLDBLog::OS);

  if (!m_process->GetLanguageRuntime(lldb::LanguageType::eLanguageTypeObjC)) {
    LLDB_LOGF(log, "%s: no ObjC language runtime", LLVM_PRETTY_FUNCTION);
    return;
  }

  auto *runtime = SwiftLanguageRuntime::Get(m_process);
  ThreadSafeReflectionContext reflection_ctx =
      runtime ? runtime->GetReflectionContext() : ThreadSafeReflectionContext();
  if (!reflection_ctx) {
    LLDB_LOGF(log, "%s no swift reflection context available.",
              LLVM_PRETTY_FUNCTION);
    return;
  }

  m_task_id_offset = reflection_ctx->GetTaskIdOffset();
}

bool OperatingSystemSwift::UpdateThreadList(
    ThreadList &old_thread_list, ThreadList &core_thread_list,
    ThreadList &new_thread_list, ThreadPlanStackMap &plan_stack_map) {
  Log *log = GetLog(LLDBLog::OS);

  LLDB_LOGF(log, "%s fetching thread data from swift for pid %" PRIu64,
            LLVM_PRETTY_FUNCTION, m_process->GetID());

  uint32_t non_mapped_insert_idx = 0;
  for (const ThreadSP &real_thread : core_thread_list.Threads()) {
    std::optional<uint64_t> task_id =
        FindTaskId(m_task_id_offset, *real_thread);
    if (!task_id.has_value()) {
      new_thread_list.InsertThread(real_thread, non_mapped_insert_idx);
      non_mapped_insert_idx++;
      LLDB_LOGF(log, "%s thread %" PRIx64 " is not executing a Task",
                LLVM_PRETTY_FUNCTION, real_thread->GetID());
      continue;
    }

    // Sets higher bits to avoid conflicts with core thread IDs.
    uint64_t masked_task_id = 0xdeadbeef00000000 | *task_id;
    ThreadSP old_thread = old_thread_list.FindThreadByID(masked_task_id);
    // This indicates a conflict of TIDs between the threads we create and core
    // threads. If this happens (unlikely), we must create a new thread instead.
    if (old_thread && !IsOperatingSystemPluginThread(old_thread))
      old_thread = nullptr;

    StringRef name = old_thread ? old_thread->GetName() : "Swift Task Thread";
    ThreadSP swift_thread =
        old_thread ? old_thread
                   : std::make_shared<ThreadMemory>(*m_process, masked_task_id,
                                                    name, "some  queue", 0);
    swift_thread->SetBackingThread(real_thread);
    new_thread_list.AddThread(swift_thread);
    m_task_tids.insert(masked_task_id);
    LLDB_LOGF(log, "%s mapping thread IDs: %" PRIx64 " -> %" PRIx64,
              LLVM_PRETTY_FUNCTION, real_thread->GetID(),
              swift_thread->GetID());
  }
  PrunePlansForNonBackedThreads(plan_stack_map, new_thread_list);
  return true;
}

void OperatingSystemSwift::ThreadWasSelected(Thread *thread) {}

RegisterContextSP
OperatingSystemSwift::CreateRegisterContextForThread(Thread *thread,
                                                     addr_t reg_data_addr) {
  if (!thread || !IsOperatingSystemPluginThread(thread->shared_from_this()))
    return nullptr;
  return thread->GetRegisterContext();
}

StopInfoSP
OperatingSystemSwift::CreateThreadStopReason(lldb_private::Thread *thread) {
  assert(thread->GetBackingThread());
  return thread->GetStopInfo();
}
#endif // #if LLDB_ENABLE_SWIFT
