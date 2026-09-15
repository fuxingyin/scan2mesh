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

#ifndef DEVICE_MEMORY_HPP_
#define DEVICE_MEMORY_HPP_

#include "kernel_containers.hpp"

/** \brief @b DeviceMemory class
  *
  * \note This is a BLOB container class with reference counting for GPU memory.
  *
  * \author Anatoly Baksheev
  */

class DeviceMemory
{
    public:
        /** \brief Empty constructor. */
        DeviceMemory();

        /** \brief Destructor. */
        ~DeviceMemory();

        /** \brief Allocates internal buffer in GPU memory
          * \param sizeBytes_arg: amount of memory to allocate
          * */
        DeviceMemory(size_t sizeBytes_arg);

        /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
          * \param ptr_arg: pointer to buffer
          * \param sizeBytes_arg: buffer size
          * */
        DeviceMemory(void *ptr_arg, size_t sizeBytes_arg);

        /** \brief Copy constructor. Just increments reference counter. */
        DeviceMemory(const DeviceMemory& other_arg);

        /** \brief Assigment operator. Just increments reference counter. */
        DeviceMemory& operator=(const DeviceMemory& other_arg);

         /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
           * \param sizeBytes_arg: buffer size
           * */
        void create(size_t sizeBytes_arg);

        /** \brief Decrements reference counter and releases internal buffer if needed. */
        void release();

        /** \brief Performs data copying. If destination size differs it will be reallocated.
          * \param other_arg: destination container
          * */
        void copyTo(DeviceMemory& other) const;

        /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
          * \param host_ptr_arg: pointer to buffer to upload
          * \param sizeBytes_arg: buffer size
          * */
        void upload(const void *host_ptr_arg, size_t sizeBytes_arg);

        /** \brief Downloads data from internal buffer to CPU memory
          * \param host_ptr_arg: pointer to buffer to download
          * */
        void download(void *host_ptr_arg) const;

        /** \brief Performs swap of data pointed with another device memory.
          * \param other: device memory to swap with
          * */
        void swap(DeviceMemory& other_arg);

        /** \brief Returns pointer for internal buffer in GPU memory. */
        template<class T> T* ptr();

        /** \brief Returns constant pointer for internal buffer in GPU memory. */
        template<class T> const T* ptr() const;

        /** \brief Conversion to PtrSz for passing to kernel functions. */
        template <class U> operator PtrSz<U>() const;

        /** \brief Returns true if unallocated otherwise false. */
        bool empty() const;

        size_t sizeBytes() const;

    private:
        /** \brief Device pointer. */
        void *data_;

        /** \brief Allocated size in bytes. */
        size_t sizeBytes_;

        /** \brief Pointer to reference counter in CPU memory. */
        int* refcount_;
};

/** \brief @b DeviceMemory2D class
  *
  * \note This is a BLOB container class with reference counting for pitched GPU memory.
  *
  * \author Anatoly Baksheev
  */

class DeviceMemory2D
{
    public:
        /** \brief Empty constructor. */
        DeviceMemory2D();

        /** \brief Destructor. */
        ~DeviceMemory2D();

        /** \brief Allocates internal buffer in GPU memory
          * \param rows_arg: number of rows to allocate
          * \param colsBytes_arg: width of the buffer in bytes
          * */
        DeviceMemory2D(int rows_arg, int colsBytes_arg);


        /** \brief Initializes with user allocated buffer. Reference counting is disabled in this case.
          * \param rows_arg: number of rows
          * \param colsBytes_arg: width of the buffer in bytes
          * \param data_arg: pointer to buffer
          * \param stepBytes_arg: stride between two consecutive rows in bytes
          * */
        DeviceMemory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg);

        /** \brief Copy constructor. Just increments reference counter. */
        DeviceMemory2D(const DeviceMemory2D& other_arg);

        /** \brief Assigment operator. Just increments reference counter. */
        DeviceMemory2D& operator=(const DeviceMemory2D& other_arg);

        /** \brief Allocates internal buffer in GPU memory. If internal buffer was created before the function recreates it with new size. If new and old sizes are equal it does nothing.
           * \param ptr_arg: number of rows to allocate
           * \param sizeBytes_arg: width of the buffer in bytes
           * */
        void create(int rows_arg, int colsBytes_arg);

        /** \brief Decrements reference counter and releases internal buffer if needed. */
        void release();

        /** \brief Performs data copying. If destination size differs it will be reallocated.
          * \param other_arg: destination container
          * */
        void copyTo(DeviceMemory2D& other) const;

        /** \brief Uploads data to internal buffer in GPU memory. It calls create() inside to ensure that intenal buffer size is enough.
          * \param host_ptr_arg: pointer to host buffer to upload
          * \param host_step_arg: stride between two consecutive rows in bytes for host buffer
          * \param rows_arg: number of rows to upload
          * \param sizeBytes_arg: width of host buffer in bytes
          * */
        void upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg);

        /** \brief Downloads data from internal buffer to CPU memory. User is resposible for correct host buffer size.
          * \param host_ptr_arg: pointer to host buffer to download
          * \param host_step_arg: stride between two consecutive rows in bytes for host buffer
          * */
        void download(void *host_ptr_arg, size_t host_step_arg) const;

        /** \brief Performs swap of data pointed with another device memory.
          * \param other: device memory to swap with
          * */
        void swap(DeviceMemory2D& other_arg);

        /** \brief Returns pointer to given row in internal buffer.
          * \param y_arg: row index
          * */
        template<class T> T* ptr(int y_arg = 0);

        /** \brief Returns constant pointer to given row in internal buffer.
          * \param y_arg: row index
          * */
        template<class T> const T* ptr(int y_arg = 0) const;

         /** \brief Conversion to PtrStep for passing to kernel functions. */
        template <class U> operator PtrStep<U>() const;

        /** \brief Conversion to PtrStepSz for passing to kernel functions. */
        template <class U> operator PtrStepSz<U>() const;

        /** \brief Returns true if unallocated otherwise false. */
        bool empty() const;

        /** \brief Returns number of bytes in each row. */
        int colsBytes() const;

        /** \brief Returns number of rows. */
        int rows() const;

        /** \brief Returns stride between two consecutive rows in bytes for internal buffer. Step is stored always and everywhere in bytes!!! */
        size_t step() const;
    private:
        /** \brief Device pointer. */
        void *data_;

        /** \brief Stride between two consecutive rows in bytes for internal buffer. Step is stored always and everywhere in bytes!!! */
        size_t step_;

        /** \brief Width of the buffer in bytes. */
        int colsBytes_;

        /** \brief Number of rows. */
        int rows_;

        /** \brief Pointer to reference counter in CPU memory. */
        int* refcount_;
};

#include "device_memory_impl.hpp"

#endif /* DEVICE_MEMORY_HPP_ */
