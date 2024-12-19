//===----------------- LockGuarded.h ----------------------------*- C++ -*-===//
//
// This source file is part of the Swift.org open source project
//
// Copyright (c) 2014 - 2024 Apple Inc. and the Swift project authors
// Licensed under Apache License v2.0 with Runtime Library Exception
//
// See https://swift.org/LICENSE.txt for license information
// See https://swift.org/CONTRIBUTORS.txt for the list of Swift project authors
//
//===----------------------------------------------------------------------===//

#ifndef liblldb_SwiftLockGuarded_h_
#define liblldb_SwiftLockGuarded_h_

#include <array>
#include <mutex>

namespace lldb_private {
/// A generic wrapper around a resource which holds a lock to ensure
/// exclusive access.
template <typename Resource, int num_locks = 1> struct LockGuarded {
  LockGuarded(Resource *resource, std::recursive_mutex &mutex)
      : m_resource(resource) {
    m_locks[0] = {mutex, std::adopt_lock};
  }

  LockGuarded(Resource *resource, std::recursive_mutex &mutex,
              std::recursive_mutex &mutex2)
      : m_resource(resource) {
    static_assert(num_locks == 2);
    m_locks[0] = {mutex, std::adopt_lock};
    m_locks[1] = {mutex2, std::adopt_lock};
  }

  LockGuarded() = default;

  Resource *operator->() const { return m_resource; }

  Resource *operator*() const { return m_resource; }

  operator bool() const { return m_resource != nullptr; }

private:
  Resource *m_resource;
  std::array<std::unique_lock<std::recursive_mutex>, num_locks> m_locks;
};

} // namespace lldb_private
#endif
