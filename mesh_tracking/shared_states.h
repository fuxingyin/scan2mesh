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

#ifndef SHARED_STATE_H_
#define SHARED_STATE_H_

#include "utils/thread_safe_value.h"

class SharedState {
 public:
  static SharedState& get() {
    static SharedState instance;
    return instance;
  }

  void reset() {
    pause_tracking.Set(false);
    frame_id.Set(0);
  }

  ThreadSafeValue<bool> pause_tracking;
  ThreadSafeValue<int> frame_id;

 private:
  SharedState() { reset(); }

  ~SharedState() = default;

  SharedState(const SharedState&) = delete;
  SharedState& operator=(const SharedState&) = delete;
  SharedState(SharedState&&) = delete;
  SharedState& operator=(SharedState&&) = delete;
};

#endif  // SHARED_STATE_H_