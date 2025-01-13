//===-- OperatingSystemSwift.h ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef liblldb_OperatingSystemSwift_h_
#define liblldb_OperatingSystemSwift_h_

#if LLDB_ENABLE_SWIFT

#include "lldb/Target/OperatingSystem.h"
#include "llvm/ADT/DenseSet.h"

namespace lldb_private {
class OperatingSystemSwift : public OperatingSystem {
public:
  OperatingSystemSwift(Process &process);
  ~OperatingSystemSwift() override;

  static OperatingSystem *CreateInstance(Process *process, bool force);
  static void Initialize();
  static void Terminate();
  static llvm::StringRef GetPluginNameStatic() { return "swift"; }
  static llvm::StringRef GetPluginDescriptionStatic();

  /// PluginInterface Methods

  llvm::StringRef GetPluginName() override { return GetPluginNameStatic(); }

  /// OperatingSystem Methods

  bool UpdateThreadList(ThreadList &old_thread_list,
                        ThreadList &real_thread_list,
                        ThreadList &new_thread_list,
                        ThreadPlanStackMap &plan_stack_map) override;

  void ThreadWasSelected(Thread *thread) override;

  lldb::RegisterContextSP
  CreateRegisterContextForThread(Thread *thread,
                                 lldb::addr_t reg_data_addr) override;

  lldb::StopInfoSP CreateThreadStopReason(Thread *thread) override;

  std::optional<bool> DoesPluginReportAllThreads() override { return false; }

private:
  /// Prune ThreadPlanStacks from `plan_stack_map` if they don't correspond to a
  /// thread in `threads`.
  void PrunePlansForNonBackedThreads(ThreadPlanStackMap &plan_stack_map,
                                     ThreadList &threads);

  /// Find the Task ID of the task being executed by `thread`, if any.
  std::optional<uint64_t> FindTaskId(Thread &thread);

  /// Set of TIDs created by this plugin.
  llvm::DenseSet<lldb::tid_t> m_task_tids;

  /// The offset of the Task pointer inside thread local storage.
  size_t m_task_ptr_offset_in_tls;

  /// The offset of the Task ID inside a Task data structure.
  size_t m_task_id_offset;
};
} // namespace lldb_private

#endif // LLDB_ENABLE_SWIFT

#endif // liblldb_OperatingSystemSwift_h_
