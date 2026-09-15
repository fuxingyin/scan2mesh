/*
 * Copyright 2026 The Eternelle Authors
 *
 * Non-Commercial Source-Available License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to use,
 * copy, modify, and distribute the Software for non-commercial purposes, subject
 * to the following conditions:
 *
 * 1. Non-commercial use only.
 *    The Software may be used for personal, academic, educational, research,
 *    evaluation, and other non-commercial purposes.
 *
 * 2. Commercial use.
 *    Commercial use of the Software, including use in a commercial product,
 *    service, system, or business operation, requires prior written permission
 *    from the copyright holder.
 *
 * 3. Redistribution.
 *    Redistributions of the Software, with or without modification, must retain
 *    this copyright notice and this license.
 *
 * 4. No trademark rights.
 *    This license does not grant permission to use the names "Eternelle",
 *    "Eternelle3D", "Scan2Mesh", or related trademarks to imply endorsement.
 *
 * 5. No warranty.
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *
 * 6. Commercial license.
 *    The copyright holder may grant separate commercial licenses under
 *    individually negotiated terms.
 */

#ifndef THREAD_MUTEX_OBJECT_H_
#define THREAD_MUTEX_OBJECT_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

template <typename T>
class ThreadSafeValue {
 public:
  ThreadSafeValue() = default;

  explicit ThreadSafeValue(const T& initial_value) : value_(initial_value) {}

  ThreadSafeValue(const ThreadSafeValue&) = delete;
  ThreadSafeValue& operator=(const ThreadSafeValue&) = delete;

  void Set(const T& value) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    value_ = value;
  }

  T Get() const {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return value_;
  }

  void SetAndNotifyAll(const T& value) {
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      value_ = value;
    }

    condition_.notify_all();
  }

  void NotifyAll() { condition_.notify_all(); }

  T Wait() {
    boost::unique_lock<boost::mutex> lock(mutex_);
    condition_.wait(lock);

    return value_;
  }

  T GetAfterDelay(int wait_microseconds = 33000) const {
    boost::this_thread::sleep(
        boost::posix_time::microseconds(wait_microseconds));

    return Get();
  }

  void Increment() {
    boost::lock_guard<boost::mutex> lock(mutex_);
    ++value_;
  }

  template <typename U>
  void Add(const U& value) {
    boost::lock_guard<boost::mutex> lock(mutex_);
    value_ += value;
  }

 private:
  T value_{};

  mutable boost::mutex mutex_;
  boost::condition_variable condition_;
};

#endif  // THREAD_MUTEX_OBJECT_H_