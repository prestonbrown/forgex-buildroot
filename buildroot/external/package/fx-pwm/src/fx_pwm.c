// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * fx-pwm - userspace driver for the Ingenic X2600 PWM2 block behind the
 * stock soc_pwm.ko kernel module, for the FlashForge AD5X piezo buzzer.
 *
 * The stock firmware beeps by shelling out to /usr/bin/cmd_pwm (a thin CLI
 * over libhardware2.so), both of which live in the OEM rootfs that the
 * Forge-X chroot replaces - which is why the buzzer is silent under
 * Forge-X even though the kernel driver is still loaded. This tool talks
 * to the same /dev/jz_pwm ioctl ABI directly, so the chroot needs nothing
 * from the OEM userland.
 *
 * Channel numbers are NOT guessed locally: every verb first issues the
 * REQUEST ioctl with the gpio NAME and uses the channel index the kernel
 * returns (the same thing the stock library does). A firmware update that
 * reshuffles the driver's pwm_gpio_array therefore cannot make this tool
 * poke the wrong channel.
 *
 * The command surface mirrors cmd_pwm so existing call sites (M300 beep
 * hooks, macros) work unchanged:
 *
 *   fx-pwm config <gpio> freq=<hz> max_level=<n> [active_level=<0|1>] [accuracy_priority=freq|levels]
 *   fx-pwm set_level <gpio> <level>
 *   fx-pwm get_level <gpio>
 *   fx-pwm set_wc <gpio> <high> <low>
 *   fx-pwm set_prescale <gpio> <prescale>
 *   fx-pwm disable <gpio>
 *   fx-pwm enable_channels <gpio> [gpio ...]
 *   fx-pwm disable_channels <gpio> [gpio ...]
 *   fx-pwm not_really_enable <gpio>
 *   fx-pwm not_really_disable <gpio>
 *   fx-pwm tone <gpio> <notes> [--base=<hz>] [--prescale=<n>]
 *   fx-pwm probe <gpio>
 *   fx-pwm selftest [gpio] [--tone-hz=<f>] [--prescale=<n>] [--ms=<n>]
 *
 * Safety rails: only pc12 (the buzzer) is accepted unless --force-gpio
 * names another gpio explicitly, and every process arms an alarm() watchdog
 * (default 5 s, --timeout=<s>) so no verb can hang forever. The watchdog
 * exists because the stock driver leaks the channel spinlock on two of
 * pwm2_config's error paths (max_level >= 65535, freq above the parent
 * clock rate): after such a failure every later ioctl on that channel
 * blocks until the module is reloaded. A verb that dies to the watchdog
 * has usually just taken that path - check dmesg for the "PWM:" line.
 */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include "pwm_abi.h"

#define PWM_DEVICE      "/dev/jz_pwm"
#define PWM_DEVICE_ALT  "/dev/misc/jz_pwm"

static int g_watchdog_seconds = 5;

/*
 * gpio allowlist. The kernel driver's own table (pwm_gpio_array, decoded
 * from the vendor module) maps, with gpio ids encoded (port<<5)|pin:
 *
 *   pb12..pb19 -> pwm0..pwm7   (ids 0x2c..0x33)
 *   pc7..pc14  -> pwm8..pwm15  (ids 0x47..0x4e)
 *   pe2        -> pwm10 alt    (id 0x82)
 *
 * so pc12, the AD5X buzzer, is pwm13/channel 13. Only the buzzer is
 * exposed by default: the wiring of the other pins is unknown and some
 * may drive fans or LEDs. The channel number is still taken from the
 * kernel at runtime, never from this table.
 */
static const char *const allowed_gpios[] = { "pc12" };

static void usage(FILE *out)
{
	fprintf(out,
"Usage: fx-pwm <verb> <gpio> [args] [--timeout=<s>] [--force-gpio]\n"
"\n"
"  config <gpio> freq=<hz> max_level=<n> [active_level=<0|1>] [accuracy_priority=freq|levels]\n"
"  set_level <gpio> <level>\n"
"  get_level <gpio>\n"
"  set_wc <gpio> <high> <low>       (raw duty halves; the tone control)\n"
"  set_prescale <gpio> <prescale>   (channel clock = parent / prescale)\n"
"  disable <gpio>\n"
"  enable_channels <gpio> [gpio ...]\n"
"  disable_channels <gpio> [gpio ...]\n"
"  not_really_enable <gpio>\n"
"  not_really_disable <gpio>\n"
"  tone <gpio> <notes> [--base=<hz>] [--prescale=<n>]\n"
"                                  (notes: \"freq:ms ...\"; bare number = rest ms)\n"
"  probe <gpio>                    (request, report the kernel's channel, no output change)\n"
"  selftest [gpio] [--tone-hz=<f>] [--prescale=<n>] [--ms=<n>]\n"
"  --selftest                       (same as: selftest pc12)\n"
"\n"
"Examples (stock parity, as firmwareExe drives the buzzer):\n"
"  fx-pwm config pc12 freq=1000000 max_level=300 active_level=1 accuracy_priority=freq\n"
"  fx-pwm set_prescale pc12 6\n"
"  fx-pwm set_wc pc12 1600 1600\n"
"  fx-pwm set_level pc12 100\n"
"  fx-pwm disable pc12\n"
"\n"
"Only pc12 (the buzzer) is accepted unless --force-gpio is given.\n"
"Every process arms a %d s alarm() watchdog (--timeout=<s> to change); see\n"
"the header comment for why a verb can otherwise hang forever.\n",
		g_watchdog_seconds);
}

static void die(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	exit(1);
}

static void step(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fflush(stdout);
}

static void to_lowercase(char *dst, size_t cap, const char *src)
{
	size_t i;

	for (i = 0; src[i] && i < cap - 1; i++)
		dst[i] = (char)((src[i] >= 'A' && src[i] <= 'Z') ? src[i] + 32 : src[i]);
	dst[i] = '\0';
}

static int open_pwm_device(void)
{
	int fd = open(PWM_DEVICE, O_RDWR);

	if (fd < 0) {
		fd = open(PWM_DEVICE_ALT, O_RDWR);
		if (fd < 0) {
			die("error: cannot open %s (%s)\n"
			    "The stock soc_pwm.ko module creates this node. If it is\n"
			    "missing inside the Forge-X chroot, expose the host node or\n"
			    "create it from /sys/class/misc/jz_pwm/dev (misc major 10).\n",
			    PWM_DEVICE, strerror(errno));
		}
	}
	return fd;
}

/*
 * REQUEST takes the gpio name string; the kernel maps it to its channel
 * index and RETURNS that index, which every later ioctl must use.
 *
 * A failed REQUEST (observed: -EBUSY when the gpio is already claimed)
 * empirically wedges the channel: every later ioctl on it blocks in the
 * kernel unkillably (D-state; three rig reboots were spent on this).
 * Mark the gpio so later invocations refuse before entering the kernel,
 * turning an unkillable hang into a clean error until the next reboot.
 */
static void wedge_marker_path(char *buf, size_t cap, const char *gpio)
{
	char lower[16];

	to_lowercase(lower, sizeof(lower), gpio);
	snprintf(buf, cap, "/run/fx-pwm.%s.wedged", lower);
}

static void check_wedge_marker(const char *gpio)
{
	char path[64];
	FILE *f;

	wedge_marker_path(path, sizeof(path), gpio);
	f = fopen(path, "r");
	if (f) {
		fclose(f);
		die("error: %s was marked wedged by an earlier failed request this boot;\n"
		    "further ioctls on it can block unkillably. Reboot to clear, or remove\n"
		    "%s if you know better.\n", gpio, path);
	}
}

static void write_wedge_marker(const char *gpio)
{
	char path[64];
	FILE *f;

	wedge_marker_path(path, sizeof(path), gpio);
	f = fopen(path, "w");
	if (f) {
		fprintf(f, "request failed; channel mutex wedged\n");
		fclose(f);
	}
}

static int pwm_request_channel(int fd, const char *gpio)
{
	char name[12];
	long ch;

	to_lowercase(name, sizeof(name), gpio);
	step("request %s ... ", name);
	ch = ioctl(fd, PWM_IOC_REQUEST, name);
	if (ch < 0) {
		int saved = errno;

		write_wedge_marker(gpio);
		errno = saved;
		if (errno == EBUSY)
			die("failed: %s (gpio already claimed - check /sys/kernel/debug/gpio\n"
			    "for the claimant; dmesg has the driver's reason)\n", strerror(errno));
		die("failed: %s (see dmesg for the driver's 'PWM:' reason)\n", strerror(errno));
	}
	step("kernel channel %ld\n", ch);
	return (int)ch;
}

static void xioctl(int fd, unsigned long req, void *arg, const char *what)
{
	step("%s ... ", what);
	fflush(stdout);
	if (ioctl(fd, req, arg) < 0)
		die("failed: %s (see dmesg for the driver's 'PWM:' reason)\n", strerror(errno));
	step("ok\n");
}

static void pwm_release_channel(int fd, int ch, const char *gpio)
{
	step("release %s (ch%d) ... ", gpio, ch);
	if (ioctl(fd, PWM_IOC_RELEASE, (unsigned long)ch) < 0)
		die("failed: %s\n", strerror(errno));
	step("ok\n");
}

static int parse_long(const char *s, const char *what)
{
	char *end;
	long v = strtol(s, &end, 0);

	if (*s == '\0' || *end != '\0')
		die("error: bad value for %s: %s\n", what, s);
	return (int)v;
}

static int cmd_config(int fd, const char *gpio, int argc, char **argv)
{
	struct pwm_config_args cfg;
	const char *accuracy = "freq";
	int i, ch;

	memset(&cfg, 0, sizeof(cfg));
	cfg.active_level = 1;
	cfg.levels_exact = 0;

	for (i = 0; i < argc; i++) {
		if (strncmp(argv[i], "freq=", 5) == 0)
			cfg.freq_hz = (uint32_t)parse_long(argv[i] + 5, "freq");
		else if (strncmp(argv[i], "max_level=", 10) == 0)
			cfg.max_level = (uint32_t)parse_long(argv[i] + 10, "max_level");
		else if (strncmp(argv[i], "active_level=", 13) == 0)
			cfg.active_level = (uint32_t)parse_long(argv[i] + 13, "active_level");
		else if (strncmp(argv[i], "accuracy_priority=", 18) == 0)
			accuracy = argv[i] + 18;
		else if (strncmp(argv[i], "--", 2) != 0)
			die("error: not support this arg: %s\n", argv[i]);
	}

	if (cfg.freq_hz == 0)
		die("error: config requires freq=<hz>\n");
	if (cfg.max_level == 0 || cfg.max_level >= 0xffff)
		die("error: config requires max_level=<1..65534>\n");
	if (strcmp(accuracy, "freq") == 0)
		cfg.levels_exact = 0;
	else if (strcmp(accuracy, "levels") == 0)
		cfg.levels_exact = 1;
	else
		die("error: accuracy_priority must be freq or levels\n");

	ch = pwm_request_channel(fd, gpio);
	cfg.channel = (uint32_t)ch;
	xioctl(fd, PWM_IOC_CONFIG, &cfg, "pwm_config");
	printf("configured %s (ch%d): freq=%u max_level=%u active_level=%u accuracy_priority=%s\n",
	       gpio, ch, cfg.freq_hz, cfg.max_level, cfg.active_level, accuracy);
	return 0;
}

static int cmd_set_level(int fd, const char *gpio, const char *level_s)
{
	struct pwm_ch_value v;

	v.channel = (uint32_t)pwm_request_channel(fd, gpio);
	v.value = (uint32_t)parse_long(level_s, "level");

	xioctl(fd, PWM_IOC_SET_LEVEL, &v, "pwm_set_level");
	printf("%s level %u\n", gpio, v.value);
	return 0;
}

static int cmd_get_level(int fd, const char *gpio)
{
	struct pwm_level_args io;

	io.channel = (uint32_t)pwm_request_channel(fd, gpio);
	io.level = 0;

	xioctl(fd, PWM_IOC_GET_LEVEL, &io, "pwm_get_level");
	printf("%s (ch%u) level %u\n", gpio, io.channel, io.level);
	return 0;
}

static int cmd_set_wc(int fd, const char *gpio, const char *high_s, const char *low_s)
{
	struct pwm_ch_value v;
	int high = parse_long(high_s, "high");
	int low = parse_long(low_s, "low");

	if (high < 0 || high > 0xffff || low < 0 || low > 0xffff)
		die("error: set_wc halves must be 0..65535\n");

	v.channel = (uint32_t)pwm_request_channel(fd, gpio);
	v.value = PWM_WC(high, low);

	xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
	printf("%s duty: active %d counts, inactive %d counts\n", gpio, low, high);
	return 0;
}

static int cmd_set_prescale(int fd, const char *gpio, const char *div_s)
{
	struct pwm_ch_value v;

	v.channel = (uint32_t)pwm_request_channel(fd, gpio);
	v.value = (uint32_t)parse_long(div_s, "prescale");
	if (v.value == 0 || v.value >= 0x10000)
		die("error: prescale must be 1..65535 (channel clock = parent / prescale)\n");

	xioctl(fd, PWM_IOC_SET_PRESCALE, &v, "pwm_set_prescale");
	printf("%s channel clock = parent / %u\n", gpio, v.value);
	return 0;
}

static int cmd_disable(int fd, const char *gpio)
{
	int ch = pwm_request_channel(fd, gpio);

	/* RELEASE passes the channel BY VALUE - see pwm_abi.h. */
	step("pwm_release ... ");
	if (ioctl(fd, PWM_IOC_RELEASE, (unsigned long)ch) < 0)
		die("failed: %s (see dmesg)\n", strerror(errno));
	step("ok\n");
	printf("%s disabled\n", gpio);
	return 0;
}

static int cmd_channels(int fd, int argc, char **argv, unsigned long req, const char *what)
{
	uint32_t mask = 0;
	int i;

	for (i = 0; i < argc; i++) {
		if (strncmp(argv[i], "--", 2) == 0)
			continue;
		mask |= 1u << pwm_request_channel(fd, argv[i]);
	}

	xioctl(fd, req, &mask, what);
	printf("%s mask 0x%x\n", what, mask);
	return 0;
}

static int cmd_not_really(int fd, const char *gpio, unsigned long req, const char *what)
{
	struct pwm_ch_value v;

	v.channel = (uint32_t)pwm_request_channel(fd, gpio);
	v.value = 0;
	xioctl(fd, req, &v, what);
	printf("%s %s\n", gpio, what);
	return 0;
}

/* Diagnostics only: report the kernel's channel for a gpio, change nothing. */
static int cmd_probe(int fd, const char *gpio)
{
	uint32_t level;
	int ch = pwm_request_channel(fd, gpio);

	level = (uint32_t)ch;
	if (ioctl(fd, PWM_IOC_GET_LEVEL, &level) == 0)
		printf("%s -> kernel channel %d, current level %u\n", gpio, ch, level);
	else
		printf("%s -> kernel channel %d (get_level: %s)\n", gpio, ch, strerror(errno));
	pwm_release_channel(fd, ch, gpio);
	return 0;
}

/*
 * Play a TONE NOTES sequence ("freq:ms freq:ms n" - a bare number is a rest
 * of n ms; freq <= 0 disables the output for the note's duration). The
 * grammar is byte-compatible with Forge-X's tone_player plugin so its
 * NOTES string can be passed through verbatim, which lets the plugin swap
 * its sysfs-PWMAudio backend for this tool without touching note parsing.
 *
 * One process plays the whole sequence in-process (no fork per note);
 * duty is fixed 50%, the loudest drive for a piezo. The channel clock is
 * base/prescale with base defaulting to the stock 50 MHz assumption; once
 * the real parent rate is measured on a machine, pass --base=.
 */
static int cmd_tone(int fd, const char *gpio, const char *notes, long base, long prescale)
{
	struct pwm_config_args cfg;
	struct pwm_ch_value v;
	const char *p = notes;
	long clock = base / prescale;
	int ch;

	if (clock <= 0)
		die("error: base clock %ld / prescale %ld is not positive\n", base, prescale);

	/*
	 * config's freq is only bookkeeping for this flow (set_prescale and
	 * set_wc override the clock and the duty halves), so ask for a
	 * conservative 1 MHz: any parent rate accepts it, and the two
	 * pwm2_config error paths that would fail (and leak the channel
	 * spinlock) are freq-above-parent and max_level-out-of-range.
	 */
	memset(&cfg, 0, sizeof(cfg));
	cfg.active_level = 1;
	cfg.levels_exact = 0;
	cfg.freq_hz = 1000000;
	cfg.max_level = 300;

	ch = pwm_request_channel(fd, gpio);
	cfg.channel = (uint32_t)ch;
	xioctl(fd, PWM_IOC_CONFIG, &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, PWM_IOC_SET_PRESCALE, &v, "pwm_set_prescale");

	while (*p) {
		char *end;
		double freq = strtod(p, &end);
		double ms = 0;

		if (end == p)
			die("error: cannot parse notes at: %s\n", p);
		p = end;
		if (*p == ':') {
			p++;
			ms = strtod(p, &end);
			if (end == p)
				die("error: cannot parse note duration at: %s\n", p);
			p = end;
		} else {
			/* bare number: a rest of this many ms */
			ms = freq;
			freq = 0;
		}
		while (*p == ' ' || *p == '\t')
			p++;

		if (freq > 0) {
			double total = (double)clock / freq;
			long half = (long)(total / 2.0);

			if (half < 1 || half > 0xffff)
				die("error: frequency %.2f Hz does not fit the 16-bit halves "
				    "(channel clock %ld Hz)\n", freq, clock);

			v.value = PWM_WC(half, half);
			xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
		} else {
			v.value = PWM_WC(0, 0); /* stock silence idiom */
			xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
		}

		if (ms > 0)
			usleep((useconds_t)(ms * 1000));
	}

	v.value = PWM_WC(0, 0);
	xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
	pwm_release_channel(fd, ch, gpio);
	return 0;
}

/*
 * Bring-up aid for a human listening: stock-parity config, then two tones
 * whose period counts bracket the plausible parent-clock rates (the vendor
 * driver defaults its rate global to 500 MHz before clk_get_rate; the
 * stock buzzer stack assumes 50 MHz), then a clean disable. If the first
 * tone is silent but the second is audible (or vice versa), the parent
 * clock differs from the assumption and tone math should be redone
 * against the measured rate.
 */
static int cmd_selftest(int fd, int argc, char **argv)
{
	const char *gpio = "pc12";
	long prescale = 6;
	long ms = 400;
	long tone_hz = 0;
	static const struct { long total; const char *note; } tones[] = {
		{ 3200,  "parent 50 MHz:  ~2.6 kHz" },
		{ 32000, "parent 500 MHz: ~2.6 kHz" },
	};
	struct pwm_config_args cfg;
	struct pwm_ch_value v;
	size_t t;
	int ch;

	for (int i = 0; i < argc; i++) {
		if (strncmp(argv[i], "--tone-hz=", 10) == 0)
			tone_hz = parse_long(argv[i] + 10, "tone-hz");
		else if (strncmp(argv[i], "--prescale=", 11) == 0)
			prescale = parse_long(argv[i] + 11, "prescale");
		else if (strncmp(argv[i], "--ms=", 6) == 0)
			ms = parse_long(argv[i] + 6, "ms");
		else if (strncmp(argv[i], "--", 2) != 0)
			gpio = argv[i];
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.active_level = 1;
	cfg.levels_exact = 0;
	cfg.freq_hz = 1000000; /* conservative: see cmd_tone */
	cfg.max_level = 300;

	printf("selftest on %s: config (max_level 300) ...\n", gpio);
	ch = pwm_request_channel(fd, gpio);
	cfg.channel = (uint32_t)ch;
	xioctl(fd, PWM_IOC_CONFIG, &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, PWM_IOC_SET_PRESCALE, &v, "pwm_set_prescale");
	printf("prescale %ld -> channel clock = parent / %ld\n", prescale, prescale);

	for (t = 0; t < sizeof(tones) / sizeof(tones[0]); t++) {
		long total = tones[t].total;
		const char *note = tones[t].note;

		if (tone_hz > 0) {
			if (t > 0)
				continue; /* explicit tone replaces the bracket */
			total = 50000000 / prescale / tone_hz;
			note = "assuming a 50 MHz parent clock";
		}
		if (total < 2 || total > 131070)
			die("error: tone period %ld counts out of range\n", total);

		long half = total / 2;
		v.value = PWM_WC(total - half, half);
		printf("tone %u: %ld counts total (%s), %ld ms...\n",
		       (unsigned)(t + 1), total, note, ms);
		fflush(stdout);
		xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
		usleep((useconds_t)ms * 1000);

		v.value = PWM_WC(0, 0); /* stock silence idiom */
		xioctl(fd, PWM_IOC_SET_WC, &v, "pwm_set_wc");
		usleep(150000);
	}

	pwm_release_channel(fd, ch, gpio);
	printf("released. If no tone was audible, check dmesg for 'PWM:' lines and\n"
	       "verify /dev/jz_pwm reaches this chroot (see package README).\n");
	return 0;
}

int main(int argc, char **argv)
{
	int fd, i, force_gpio = 0;

	if (argc < 2) {
		usage(stderr);
		return 1;
	}

	/* Global options may appear anywhere; strip them before dispatch. */
	for (i = 1; i < argc; i++) {
		if (strncmp(argv[i], "--timeout=", 10) == 0) {
			g_watchdog_seconds = parse_long(argv[i] + 10, "timeout");
			if (g_watchdog_seconds < 1 || g_watchdog_seconds > 60)
				die("error: --timeout must be 1..60 seconds\n");
		} else if (strcmp(argv[i], "--force-gpio") == 0) {
			force_gpio = 1;
		}
	}

	const char *verb = argv[1];

	if (strcmp(verb, "-h") == 0 || strcmp(verb, "--help") == 0) {
		usage(stdout);
		return 0;
	}

	alarm(g_watchdog_seconds);

	check_wedge_marker(argc >= 3 ? argv[2] : "pc12");

	if (strcmp(verb, "--selftest") == 0) {
		fd = open_pwm_device();
		return cmd_selftest(fd, argc - 2, argv + 2);
	}

	if (argc < 3)
		die("error: verb %s needs a gpio argument\n", verb);
	const char *gpio = argv[2];

	if (!force_gpio) {
		char lower[16];
		size_t k;
		int ok = 0;

		to_lowercase(lower, sizeof(lower), gpio);
		for (k = 0; k < sizeof(allowed_gpios) / sizeof(allowed_gpios[0]); k++)
			if (strcmp(lower, allowed_gpios[k]) == 0)
				ok = 1;
		if (!ok)
			die("error: %s is not in the allowlist (pc12, the buzzer); the wiring of\n"
			    "other pwm gpios is unknown - use --force-gpio deliberately\n", gpio);
	}

	fd = open_pwm_device();

	if (strcmp(verb, "config") == 0) {
		if (argc < 4)
			die("error: config needs freq= and max_level=\n");
		return cmd_config(fd, gpio, argc - 3, argv + 3);
	}
	if (strcmp(verb, "set_level") == 0) {
		if (argc < 4)
			die("error: set_level needs a level\n");
		return cmd_set_level(fd, gpio, argv[3]);
	}
	if (strcmp(verb, "get_level") == 0)
		return cmd_get_level(fd, gpio);
	if (strcmp(verb, "set_wc") == 0) {
		if (argc < 5)
			die("error: set_wc needs <high> <low>\n");
		return cmd_set_wc(fd, gpio, argv[3], argv[4]);
	}
	if (strcmp(verb, "set_prescale") == 0) {
		if (argc < 4)
			die("error: set_prescale needs a value\n");
		return cmd_set_prescale(fd, gpio, argv[3]);
	}
	if (strcmp(verb, "disable") == 0)
		return cmd_disable(fd, gpio);
	if (strcmp(verb, "enable_channels") == 0)
		return cmd_channels(fd, argc - 2, argv + 2, PWM_IOC_ENABLE_CHANNELS, "pwm_enable_channels");
	if (strcmp(verb, "disable_channels") == 0)
		return cmd_channels(fd, argc - 2, argv + 2, PWM_IOC_DISABLE_CHANNELS, "pwm_disable_channels");
	if (strcmp(verb, "not_really_enable") == 0)
		return cmd_not_really(fd, gpio, PWM_IOC_NOT_REALLY_ENABLE, "pwm_not_really_enable");
	if (strcmp(verb, "not_really_disable") == 0)
		return cmd_not_really(fd, gpio, PWM_IOC_NOT_REALLY_DISABLE, "pwm_not_really_disable");
	if (strcmp(verb, "tone") == 0) {
		long base = 50000000, prescale = 6;

		if (argc < 4)
			die("error: tone needs a notes string, e.g. \"1000:100 50 1479:200\"\n");
		for (i = 4; i < argc; i++) {
			if (strncmp(argv[i], "--base=", 7) == 0)
				base = parse_long(argv[i] + 7, "base");
			else if (strncmp(argv[i], "--prescale=", 11) == 0)
				prescale = parse_long(argv[i] + 11, "prescale");
			else if (strncmp(argv[i], "--", 2) != 0)
				die("error: not support this arg: %s\n", argv[i]);
		}
		return cmd_tone(fd, gpio, argv[3], base, prescale);
	}
	if (strcmp(verb, "probe") == 0)
		return cmd_probe(fd, gpio);
	if (strcmp(verb, "selftest") == 0)
		return cmd_selftest(fd, argc - 2, argv + 2);

	usage(stderr);
	return 1;
}
