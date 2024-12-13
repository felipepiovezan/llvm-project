//===-- OperatingSystemSwift.h ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef liblldb_OperatingSystemSwift_h_
#define liblldb_OperatingSystemSwift_h_

#include "lldb/Host/Config.h"

#if LLDB_ENABLE_SWIFT

#include "lldb/Target/DynamicRegisterInfo.h"
#include "lldb/Target/OperatingSystem.h"
#include "lldb/Utility/StructuredData.h"
#include "llvm/ADT/DenseSet.h"

class OperatingSystemSwift : public lldb_private::OperatingSystem {
public:
  OperatingSystemSwift(lldb_private::Process &process);
  ~OperatingSystemSwift() override;

  // Static Functions
  static lldb_private::OperatingSystem *
  CreateInstance(lldb_private::Process *process, bool force);

  static void Initialize();

  static void Terminate();

  static llvm::StringRef GetPluginNameStatic() { return "swift"; }

  static llvm::StringRef GetPluginDescriptionStatic();

  // lldb_private::PluginInterface Methods
  llvm::StringRef GetPluginName() override { return GetPluginNameStatic(); }

  // lldb_private::OperatingSystem Methods
  bool
  UpdateThreadList(lldb_private::ThreadList &old_thread_list,
                   lldb_private::ThreadList &real_thread_list,
                   lldb_private::ThreadList &new_thread_list,
                   lldb_private::ThreadPlanStackMap &plan_stack_map) override;

  void ThreadWasSelected(lldb_private::Thread *thread) override;

  lldb::RegisterContextSP
  CreateRegisterContextForThread(lldb_private::Thread *thread,
                                 lldb::addr_t reg_data_addr) override;

  lldb::StopInfoSP
  CreateThreadStopReason(lldb_private::Thread *thread) override;

private:
  /// Prune ThreadPlanStacks from `plan_stack_map` if they don't correspond to a
  /// thread in `threads`.
  void PrunePlansForNonBackedThreads(
      lldb_private::ThreadPlanStackMap &plan_stack_map,
      lldb_private::ThreadList &threads);

  /// Set of TIDs created by this plugin.
  llvm::DenseSet<lldb::tid_t> m_task_tids;
};

#endif // LLDB_ENABLE_SWIFT

#endif // liblldb_OperatingSystemSwift_h_
