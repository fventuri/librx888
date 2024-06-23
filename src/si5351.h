/*
 * Copyright 2024 Franco Venturi
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef _SI5351_H_
#define _SI5351_H_

#include <libusb.h>

void start_adc(struct libusb_device_handle *dev_handle, double samplerate, double reference, double correction);

#endif /* _SI5351_H_ */ 
