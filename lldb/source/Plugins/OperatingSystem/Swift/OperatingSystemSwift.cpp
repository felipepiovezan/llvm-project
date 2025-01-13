//===-- OperatingSystemSwift.cpp -----------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#if LLDB_ENABLE_SWIFT

#include "OperatingSystemSwift.h"

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

std::optional<uint64_t> OperatingSystemSwift::FindTaskId(Thread &thread) {
  // Compute the thread local storage address for this thread.
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

  // The Task address is at offset m_task_ptr_offset_in_tls from the thread
  // local storage base pointer.
  addr_t task_addr_location = tsd_addr + m_task_ptr_offset_in_tls;
  Status error;
  addr_t task_addr =
      m_process->ReadPointerFromMemory(task_addr_location, error);
  if (error.Fail())
    return {};

  // The Task ID is at offset m_task_id_offset from the Task pointer.
  constexpr uint32_t num_bytes_task_id = 4;
  auto task_id = m_process->ReadUnsignedIntegerFromMemory(
      task_addr + m_task_id_offset, num_bytes_task_id, LLDB_INVALID_ADDRESS,
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
  return "Operating system plug-in converting Swift Tasks into Threads.";
}

OperatingSystemSwift::OperatingSystemSwift(lldb_private::Process &process)
    : OperatingSystem(&process) {
  size_t ptr_size = process.GetAddressByteSize();
  /// These are ABI guarantees, see swift/RemoteInspection/RuntimeInternals.h
  m_task_ptr_offset_in_tls =
      swift::tls_get_key(swift::tls_key::concurrency_task) * ptr_size;
  m_task_id_offset = 4 * ptr_size + 4;
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
    // this plugin, keep its alive.
    if (m_task_tids.contains(tid))
      continue;
    // Otherwise, it is a core thread no longer active. Prune it.
    plan_stack_map.RemoveTID(tid);
    LLDB_LOGF(GetLog(LLDBLog::OS),
              "OperatingSystemSwift: deleting "
              "plan for core thread with tid = %" PRIx64,
              tid);
  }
}

bool OperatingSystemSwift::UpdateThreadList(
    ThreadList &old_thread_list, ThreadList &core_thread_list,
    ThreadList &new_thread_list, ThreadPlanStackMap &plan_stack_map) {
  Log *log = GetLog(LLDBLog::OS);

  LLDB_LOGF(
      log,
      "OperatingSystemSwift: fetching thread data from swift for pid %" PRIu64,
      m_process->GetID());

  for (const ThreadSP &real_thread : core_thread_list.Threads()) {
    std::optional<uint64_t> task_id = FindTaskId(*real_thread);

    // If this is not a thread running a Task, add it to the list as is.
    if (!task_id.has_value()) {
      new_thread_list.AddThread(real_thread);
      LLDB_LOGF(log,
                "OperatingSystemSwift: thread %" PRIx64
                " is not executing a Task",
                real_thread->GetID());
      continue;
    }

    // Mask higher bits to avoid conflicts with core thread IDs.
    uint64_t masked_task_id = 0xdeadbeef00000000 | *task_id;

    ThreadSP swift_thread = [&]() -> ThreadSP {
      if (ThreadSP old_thread = old_thread_list.FindThreadByID(masked_task_id);
          IsOperatingSystemPluginThread(old_thread))
        return old_thread;

      std::string name =
          llvm::formatv("Swift Task Thread for Task {0}", *task_id);
      llvm::StringRef queue_name = "";
      return std::make_shared<ThreadMemory>(*m_process, masked_task_id, name,
                                            queue_name, 0);
    }();

    swift_thread->SetBackingThread(real_thread);
    new_thread_list.AddThread(swift_thread);
    m_task_tids.insert(masked_task_id);
    LLDB_LOGF(log,
              "OperatingSystemSwift: mapping thread IDs: %" PRIx64
              " -> %" PRIx64,
              real_thread->GetID(), swift_thread->GetID());
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
  return thread->GetStopInfo();
}
#endif // #if LLDB_ENABLE_SWIFT
