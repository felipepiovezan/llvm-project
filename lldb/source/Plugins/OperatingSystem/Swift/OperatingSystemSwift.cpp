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
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/StructuredData.h"

#include <memory>

using namespace lldb;
using namespace lldb_private;

LLDB_PLUGIN_DEFINE(OperatingSystemSwift)

static std::optional<uint64_t>
FindTaskId(Process &process, ThreadSafeReflectionContext &reflection_ctx,
           Thread &thread) {
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
  addr_t task_addr_location = tsd_addr + (103 << 3);
  Status error;
  addr_t task_addr = process.ReadPointerFromMemory(task_addr_location, error);
  if (error.Fail())
    return {};

  llvm::Expected<ReflectionContextInterface::AsyncTaskInfo> task_info =
      reflection_ctx->asyncTaskInfo(task_addr);
  if (!task_info) {
    llvm::consumeError(task_info.takeError());
    return {};
  }
  return task_info->task_id;
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
    : OperatingSystem(&process) {}

OperatingSystemSwift::~OperatingSystemSwift() = default;

bool OperatingSystemSwift::UpdateThreadList(ThreadList &old_thread_list,
                                            ThreadList &core_thread_list,
                                            ThreadList &new_thread_list) {
  if (!LanguageRuntime::FindPlugin(m_process,
                                   lldb::LanguageType::eLanguageTypeObjC)) {
    new_thread_list = core_thread_list;
    return false;
  }
  auto *runtime = SwiftLanguageRuntime::Get(m_process);
  if (!runtime || runtime->IsStub()) {
    new_thread_list = core_thread_list;
    return false;
  }

  Log *log = GetLog(LLDBLog::OS);
  LLDB_LOGF(log,
            "OperatingSystemSwift::UpdateThreadList() fetching thread "
            "data from swift for pid %" PRIu64,
            m_process->GetID());

  ThreadSafeReflectionContext reflection_ctx = runtime->GetReflectionContext();

  uint32_t non_mapped_insert_idx = 0;
  for (const ThreadSP &real_thread : core_thread_list.Threads()) {
    std::optional<uint64_t> maybe_task_id =
        FindTaskId(*m_process, reflection_ctx, *real_thread);
    if (!maybe_task_id.has_value()) {
      new_thread_list.InsertThread(real_thread, non_mapped_insert_idx);
      non_mapped_insert_idx++;
      continue;
    }

    uint64_t task_id = 0xdeadbeef00000000 | *maybe_task_id;
    ThreadSP old_thread = old_thread_list.FindThreadByID(task_id);
    if (old_thread && !IsOperatingSystemPluginThread(old_thread))
      old_thread = nullptr;

    ThreadSP swift_thread =
        old_thread ? old_thread
                   : std::make_shared<ThreadMemory>(
                         *m_process, task_id, "mythreadname", "some  queue", 0);
    swift_thread->SetBackingThread(real_thread);
    new_thread_list.AddThread(swift_thread);
  }
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
