/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * ioctl ABI of the stock Ingenic "soc_pwm" kernel driver (x2600_510 SDK,
 * module_driver/soc/x2600_510/pwm/pwm.c) as shipped in the AD5X factory
 * firmware. The driver binds the PWM2 hardware block (physical 0x13610000)
 * as a misc character device named "jz_pwm" -> /dev/jz_pwm.
 *
 * Command numbers and struct layouts below were decoded from the vendor
 * module's ioctl dispatcher (pwm_ioctl) and cross-checked against the stock
 * libhardware2.so client (pwm_request/pwm_config/pwm_set_level/...); the two
 * agree on every number and field order. See package README.md for the full
 * derivation and the PWM2 register map.
 *
 * _IOC encoding: dir<<30 | size<<16 | type<<8 | nr, type 'P' (0x50).
 */
#ifndef FX_PWM_ABI_H
#define FX_PWM_ABI_H

#include <stdint.h>

/*
 * REQUEST - arg is a char[12] buffer holding the GPIO name string
 * ("pc12"). The kernel resolves it via str_to_gpio() (utils.ko; id encoding
 * is (port<<5)|pin, so pc12 = 0x4c = 76) against its pwm_gpio_array, and
 * the ioctl RETURNS the channel index for that gpio (pc12 -> 13, "pwm13"
 * in the vendor table). Clients must use that returned index in every
 * later struct - do not guess channel numbers locally. Idempotent:
 * requesting an already-requested channel is a no-op.
 */
#define PWM_IOC_REQUEST 0x8001500b /* _IOR('P', 0x0b, char[12]) */

/*
 * RELEASE - arg is the channel index BY VALUE (despite the _IOR encoding,
 * no pointer is dereferenced). Disables the channel output and clears its
 * requested state.
 */
#define PWM_IOC_RELEASE 0x80045016 /* _IOR('P', 0x16, int) */

/* CONFIG - arg is struct pwm_config_args (24 bytes). */
#define PWM_IOC_CONFIG 0x80185001 /* _IOR('P', 0x01, 24) */

/* SET_WC - arg is struct pwm_ch_value; wc = (high << 16) | low. */
#define PWM_IOC_SET_WC 0xc004502d /* _IOWR('P', 0x2d, 8) */

/* SET_PRESCALE - arg is struct pwm_ch_value; value = clock divider N. */
#define PWM_IOC_SET_PRESCALE 0xc004502e /* _IOWR('P', 0x2e, 8) */

/* SET_LEVEL - arg is struct pwm_ch_value; value = level (0..max_level). */
#define PWM_IOC_SET_LEVEL 0xc004502c /* _IOWR('P', 0x2c, 8) */

/*
 * GET_LEVEL - arg is a uint32_t that carries the channel in and the current
 * level (0..max_level, scaled by the driver) out.
 */
#define PWM_IOC_GET_LEVEL 0xc004502b /* _IOWR('P', 0x2b, 4) */

/* NOT_REALLY_ENABLE / _DISABLE - arg is struct pwm_ch_value; value unused. */
#define PWM_IOC_NOT_REALLY_ENABLE 0x80045058  /* _IOR('P', 0x58, 4) */
#define PWM_IOC_NOT_REALLY_DISABLE 0x80045059 /* _IOR('P', 0x59, 4) */

/*
 * ENABLE/DISABLE_CHANNELS - arg is a uint32_t channel mask written straight
 * to the hardware enable/disable bitmaps (register +0x00 / +0x04). The
 * stock library uses these for cmd_pwm's enable_channels/disable_channels.
 */
#define PWM_IOC_ENABLE_CHANNELS 0x80045062  /* _IOR('P', 0x62, 4) */
#define PWM_IOC_DISABLE_CHANNELS 0x80045063 /* _IOR('P', 0x63, 4) */

/*
 * struct pwm_config_args field order matches what pwm2_config() reads.
 * The channel index is the driver's pwmdata index, NOT the gpio id, and
 * comes from the REQUEST ioctl's return value (vendor table: pb12..pb19
 * are pwm0..pwm7, pc7..pc14 are pwm8..pwm15, so pc12 -> 13).
 *
 * Beware: two of pwm2_config's rejection paths (max_level >= 65535 and
 * freq_hz above the parent clock rate) return without unlocking the
 * channel spinlock, so after such a failure every later ioctl on that
 * channel blocks until the module is reloaded. Keep max_level < 65535 and
 * freq_hz at or below any plausible parent rate.
 */
struct pwm_config_args {
	uint32_t _reserved;    /* @0: not read by the driver */
	uint32_t active_level; /* @4: output polarity, driver normalizes to 0/1 */
	uint32_t levels_exact; /* @8: see below */
	uint32_t freq_hz;      /* @12: target output frequency, nonzero */
	uint32_t max_level;    /* @16: level scale, 1..65534 */
	uint32_t channel;      /* @20: channel index from REQUEST (pc12 -> 13) */
};

/*
 * levels_exact selects which quantity config() makes exact:
 *
 *   0 ("accuracy_priority=freq"):  channel clock stays at the parent rate,
 *      period counts = clk / freq_hz, so the output frequency is exact and
 *      each level step is (clk / freq_hz) / max_level counts of duty.
 *
 *   1 ("accuracy_priority=levels"): period counts are pinned to max_level,
 *      so the level scale is exact and the output frequency becomes
 *      clk / max_level (the channel clock is only reduced if the period
 *      would not fit 16 bits).
 *
 * set_prescale() overrides the computed channel clock divider afterwards:
 * divider register = N-1, channel clock = parent / N. The stock buzzer flow
 * configures freq=50000000 (the parent rate), max_level=300, then applies
 * set_prescale 6 and drives tones with set_wc.
 */
struct pwm_ch_value {
	uint32_t channel;
	uint32_t value;
};

/*
 * set_wc value encoding: the 32-bit duty register of the channel is written
 * verbatim as (high << 16) | low, where the low half is the count at the
 * ACTIVE level and the high half the count at the inactive level. Output
 * frequency = channel_clock / (high + low). The driver latches the new
 * period and muxes the pin to its PWM function on every set_wc.
 */
#define PWM_WC(high, low) ((((high) & 0xffff) << 16) | ((low) & 0xffff))

#endif /* FX_PWM_ABI_H */
