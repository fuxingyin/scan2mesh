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

#ifndef DEVICE_MEMORY_IMPL_HPP_
#define DEVICE_MEMORY_IMPL_HPP_

/////////////////////  Inline implementations of DeviceMemory ///////////////////
template<class T> inline       T* DeviceMemory::ptr()       { return (      T*)data_; }
template<class T> inline const T* DeviceMemory::ptr() const { return (const T*)data_; }
                        
template <class U> inline DeviceMemory::operator PtrSz<U>() const
{
    PtrSz<U> result;
    result.data = (U*)ptr<U>();
    result.size = sizeBytes_/sizeof(U);
    return result; 
}

/////////////////////  Inline implementations of DeviceMemory2D ///////////////////               
template<class T>        T* DeviceMemory2D::ptr(int y_arg)       { return (      T*)((      char*)data_ + y_arg * step_); }
template<class T>  const T* DeviceMemory2D::ptr(int y_arg) const { return (const T*)((const char*)data_ + y_arg * step_); }
  
template <class U> DeviceMemory2D::operator PtrStep<U>() const
{
    PtrStep<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    return result;
}

template <class U> DeviceMemory2D::operator PtrStepSz<U>() const
{
    PtrStepSz<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    result.cols = colsBytes_/sizeof(U);
    result.rows = rows_;
    return result;
}

#endif /* DEVICE_MEMORY_IMPL_HPP_ */

