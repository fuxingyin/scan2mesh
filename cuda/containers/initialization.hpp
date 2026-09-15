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

#ifndef INITIALISATION_HPP_
#define INITIALISATION_HPP_

#include <string>

/** \brief Returns number of Cuda device. */
int getCudaEnabledDeviceCount();

/** \brief Sets active device to work with. */
void setDevice(int device);

/** \brief Return devuce name for gived device. */
std::string getDeviceName(int device);

/** \brief Prints infromatoin about given cuda deivce or about all deivces
 *  \param deivce: if < 0 prints info for all devices, otherwise the function interpets is as device id.
 */
void printCudaDeviceInfo(int device = -1);

/** \brief Prints infromatoin about given cuda deivce or about all deivces
 *  \param deivce: if < 0 prints info for all devices, otherwise the function interpets is as device id.
 */
void printShortCudaDeviceInfo(int device = -1);

/** \brief Returns true if pre-Fermi generaton GPU.
  * \param device: device id to check, if < 0 checks current device.
  */
bool checkIfPreFermiGPU(int device = -1);

/** \brief Error handler. All GPU functions call this to report an error. For internal use only */
void error(const char *error_string, const char *file, const int line, const char *func = "");

#endif /* INITIALISATION_HPP_ */
