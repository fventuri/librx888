/*
 * Copyright 2024 Franco Venturi
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <libusb.h>
#include <stdio.h>
#include <stdint.h>
#ifndef _WIN32
#include <time.h>
#endif

#include "si5351.h"

/* internal functions */
static void rational_approximation(double value, uint32_t max_denominator, uint32_t *a, uint32_t *b, uint32_t *c);
static int control_send(struct libusb_device_handle *dev_handle, uint8_t cmd, uint16_t value, uint16_t index, unsigned char *data, uint16_t length);
static int control_send_byte(struct libusb_device_handle *dev_handle, uint8_t cmd, uint16_t value, uint16_t index, uint8_t data);

/* Si5351 constants */
static const uint8_t SI5351_ADDR = 0x60 << 1;
static const double SI5351_MAX_VCO_FREQ = 900e6;
static const uint32_t SI5351_MAX_DENOMINATOR = 1048575;

enum SI5351Registers {
    SI5351_REGISTER_PLL_SOURCE   = 15,
    SI5351_REGISTER_CLK_BASE     = 16,
    SI5351_REGISTER_MSNA_BASE    = 26,
    SI5351_REGISTER_MSNB_BASE    = 34,
    SI5351_REGISTER_MS0_BASE     = 42,
    SI5351_REGISTER_MS1_BASE     = 50,
    SI5351_REGISTER_PLL_RESET    = 177,
    SI5351_REGISTER_CRYSTAL_LOAD = 183
};

enum SI5351CrystalLoadValues {
    SI5351_VALUE_CLK_PDN          = 0x80,
    SI5351_VALUE_CRYSTAL_LOAD_6PF = 0x01 << 6 | 0x12,
    SI5351_VALUE_PLLA_RESET       = 0x20,
    SI5351_VALUE_PLLB_RESET       = 0x80,
    SI5351_VALUE_MS_INT           = 0x40,
    SI5351_VALUE_CLK_SRC_MS       = 0x0c,
    SI5351_VALUE_CLK_DRV_8MA      = 0x03,
    SI5351_VALUE_MS_SRC_PLLA      = 0x00,
    SI5351_VALUE_MS_SRC_PLLB      = 0x20
};

/* rx888 contants */
const uint8_t I2CWFX3 = 0xAE;


// SiLabs Application Note AN619 - Manually Generating an Si5351 Register Map (https://www.silabs.com/documents/public/application-notes/AN619.pdf)
void start_adc(struct libusb_device_handle *dev_handle, double samplerate, double reference, double correction) {
    if (samplerate == 0) {
        /* power off clock 0 */
        control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_CLK_BASE+0, SI5351_VALUE_CLK_PDN);
        return;
    }

    /* if the requested sample rate is below 1MHz, use an R divider */
    double r_samplerate = samplerate;
    uint8_t rdiv = 0;
    while (r_samplerate < 1e6 && rdiv <= 7) {
        r_samplerate *= 2.0;
        rdiv += 1;
    }
    if (r_samplerate < 1e6) {
        fprintf(stderr, "ERROR - requested sample rate is too low: %lf\n", samplerate);
        return;
    }

    /* choose an even integer for the output MS */
    uint32_t output_ms = ((uint32_t)(SI5351_MAX_VCO_FREQ / r_samplerate));
    output_ms -= output_ms % 2;
    if (output_ms < 4 || output_ms > 900) {
        fprintf(stderr, "ERROR - invalid output MS: %d  (samplerate=%lf)\n", output_ms, samplerate);
        return;
    }
    double vco_frequency = r_samplerate * output_ms;

    /* feedback MS */
    double reference_corrected = reference * (1.0 + 1e-6 * correction);
    double feedback_ms = vco_frequency / reference_corrected;
    /* find a good rational approximation for feedback_ms */
    uint32_t a;
    uint32_t b;
    uint32_t c;
    rational_approximation(feedback_ms, SI5351_MAX_DENOMINATOR, &a, &b, &c);

    fprintf(stderr, "actual PLL frequency: %lf * (1.0 + %lf * 1e-6) * (%d + %d / %d)\n", reference, correction, a, b, c);

    double actual_ratio = a + (double)b / (double)c;
    double actual_pll_frequency = reference_corrected * actual_ratio;
    fprintf(stderr, "actual PLL frequency: %lf\n", actual_pll_frequency);

    double actual_samplerate = actual_pll_frequency / output_ms / (1 << rdiv);
    fprintf(stderr, "actual sample rate: %lf / %d = %lf\n", actual_pll_frequency, output_ms * (1 << rdiv), actual_samplerate);
    fprintf(stderr, "sample rate difference: %lf\n", actual_samplerate - samplerate);

    /* configure clock input and PLL */
    uint32_t const b_over_c = 128 * b / c;
    uint32_t const msn_p1 = 128 * a + b_over_c - 512;
    uint32_t const msn_p2 = 128 * b    - c * b_over_c;
    uint32_t const msn_p3 = c;

    uint8_t data_clkin[] = {
        (msn_p3 & 0x0000ff00) >>  8,
        (msn_p3 & 0x000000ff) >>  0,
        (msn_p1 & 0x00030000) >> 16,
        (msn_p1 & 0x0000ff00) >>  8,
        (msn_p1 & 0x000000ff) >>  0,
        (msn_p3 & 0x000f0000) >> 12 | (msn_p2 & 0x000f0000) >> 16,
        (msn_p2 & 0x0000ff00) >>  8,
        (msn_p2 & 0x000000ff) >>  0
    };

    control_send(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_MSNA_BASE, data_clkin, sizeof(data_clkin));

    /* configure clock output */
    /* since the output divider is an even integer a = output_ms, b = 0, c = 1 */
    uint32_t const ms_p1 = 128 * output_ms - 512;
    uint32_t const ms_p2 = 0;
    uint32_t const ms_p3 = 1;

    uint8_t data_clkout[] = {
        (ms_p3 & 0x0000ff00) >>  8,
        (ms_p3 & 0x000000ff) >>  0,
        rdiv << 4 | (output_ms == 4 ? 0xc : 0x0) | (ms_p1 & 0x00030000) >> 16,
        (ms_p1 & 0x0000ff00) >>  8,
        (ms_p1 & 0x000000ff) >>  0,
        (ms_p3 & 0x000f0000) >> 12 | (ms_p2 & 0x000f0000) >> 16,
        (ms_p2 & 0x0000ff00) >>  8,
        (ms_p2 & 0x000000ff) >>  0
    };

    control_send(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_MS0_BASE, data_clkout, sizeof(data_clkout));

    /* start clock */
    control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_PLL_RESET, SI5351_VALUE_PLLA_RESET);
    /* power on clock 0 */
    uint8_t const clock_control = SI5351_VALUE_MS_INT | SI5351_VALUE_CLK_SRC_MS | SI5351_VALUE_CLK_DRV_8MA | SI5351_VALUE_MS_SRC_PLLA;
    control_send_byte(dev_handle, I2CWFX3, SI5351_ADDR, SI5351_REGISTER_CLK_BASE+0, clock_control);

/* sleep 10ms */
#ifdef _WIN32
    Sleep(10);
#else
    nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
#endif
    return;
}

/* best rational approximation:
 *
 *     value ~= a + b/c     (where c <= max_denominator)
 *
 * References:
 * - https://en.wikipedia.org/wiki/Continued_fraction#Best_rational_approximations
 */
static void rational_approximation(double value, uint32_t max_denominator, uint32_t *a, uint32_t *b, uint32_t *c) {
    const double epsilon = 1e-5;

    double af;
    double f0 = modf(value, &af);
    *a = (uint32_t) af;
    *b = 0;
    *c = 1;
    double f = f0;
    double delta = f0;
    /* we need to take into account that the fractional part has a_0 = 0 */
    uint32_t h[] = {1, 0};
    uint32_t k[] = {0, 1};
    for(int i = 0; i < 100; ++i){
        if(f <= epsilon){
            break;
        }
        double anf;
        f = modf(1.0 / f,&anf);
        uint32_t an = (uint32_t) anf;
        for(uint32_t m = (an + 1) / 2; m <= an; ++m){
            uint32_t hm = m * h[1] + h[0];
            uint32_t km = m * k[1] + k[0];
            if(km > max_denominator){
                break;
            }
            double d = fabs((double) hm / (double) km - f0);
            if(d < delta){
                delta = d;
                *b = hm;
                *c = km;
            }
        }
        uint32_t hn = an * h[1] + h[0];
        uint32_t kn = an * k[1] + k[0];
        h[0] = h[1]; h[1] = hn;
        k[0] = k[1]; k[1] = kn;
    }
    return;
}

static int control_send(struct libusb_device_handle *dev_handle,
                        uint8_t cmd, uint16_t value, uint16_t index,
                        unsigned char *data, uint16_t length) {

    int ret;

    /* Send the control message. */
    ret = libusb_control_transfer(
        dev_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, cmd, value,
        index, data, length, 0);

    if (ret < 0) {
        fprintf(stderr, "Could not send control: 0x%X with value: 0x%X, index: 0x%X, length: %d. Error : %s.\n",
                cmd, value, index, length, libusb_error_name(ret));
        return -1;
    }

    return 0;
}

static int control_send_byte(struct libusb_device_handle *dev_handle,
                             uint8_t cmd, uint16_t value, uint16_t index,
                             uint8_t data) {

    int ret;

    /* Send the control message. */
    uint8_t ldata = data;
    ret = libusb_control_transfer(
        dev_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, cmd, value,
        index, &ldata, sizeof(ldata), 0);

    if (ret < 0) {
        fprintf(stderr, "Could not send byte control: 0x%X with value: 0x%X, index: 0x%X, data: 0x%X. Error : %s.\n",
                cmd, value, index, data, libusb_error_name(ret));
        return -1;
    }

    return 0;
}
