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
 * Ioctl numbers, overridable via FX_PWM_IOC_<NAME> for bring-up on OEM
 * builds whose soc_pwm encodings differ from the Factory 1.1.7 reference
 * (see pwm_abi.h). Overrides must come from decoding that machine's own
 * libhardware2.so, never from guessing.
 */
#define IOC_OVERRIDE(name, value) \
	static unsigned long ioc_##name(unsigned long fallback) { \
		const char *s = getenv("FX_PWM_IOC_" #name); \
		return (s && *s) ? strtoul(s, NULL, 0) : (fallback); \
	}
IOC_OVERRIDE(REQUEST, PWM_IOC_REQUEST)
IOC_OVERRIDE(RELEASE, PWM_IOC_RELEASE)
IOC_OVERRIDE(CONFIG, PWM_IOC_CONFIG)
IOC_OVERRIDE(SET_WC, PWM_IOC_SET_WC)
IOC_OVERRIDE(SET_PRESCALE, PWM_IOC_SET_PRESCALE)
IOC_OVERRIDE(SET_LEVEL, PWM_IOC_SET_LEVEL)
IOC_OVERRIDE(GET_LEVEL, PWM_IOC_GET_LEVEL)
IOC_OVERRIDE(ENABLE_CHANNELS, PWM_IOC_ENABLE_CHANNELS)
IOC_OVERRIDE(DISABLE_CHANNELS, PWM_IOC_DISABLE_CHANNELS)
IOC_OVERRIDE(NOT_REALLY_ENABLE, PWM_IOC_NOT_REALLY_ENABLE)
IOC_OVERRIDE(NOT_REALLY_DISABLE, PWM_IOC_NOT_REALLY_DISABLE)
IOC_OVERRIDE(DMA_INIT, PWM_IOC_DMA_INIT)
IOC_OVERRIDE(DMA_OP, PWM_IOC_DMA_OP)
IOC_OVERRIDE(DMA_DISABLE_LOOP, PWM_IOC_DMA_DISABLE_LOOP)

/*
 * A process killed between REQUEST and RELEASE leaves the gpio claim
 * orphaned, and the vendor driver's next REQUEST on an orphaned claim can
 * block forever in soc_gpio's lock path (D-state, alarm-proof; only a
 * reboot clears it - measured twice on the rig). Catch the fatal signals
 * and try to release before dying. If the thread is already stuck in the
 * kernel the handler never runs and the reboot is still the only out,
 * but for every kill that lands between ioctls this keeps the machine
 * beeping instead of bricking its buzzer until the next boot.
 */
static int g_fd = -1;
static int g_channel = -1;
static const char *g_gpio_name;

static void release_and_exit(int sig)
{
	if (g_fd >= 0 && g_channel >= 0)
		ioctl(g_fd, ioc_RELEASE(PWM_IOC_RELEASE), (unsigned long)g_channel);
	_exit(128 + sig);
}

static void arm_signal_release(int fd, int ch, const char *gpio)
{
	struct sigaction sa;

	g_fd = fd;
	g_channel = ch;
	g_gpio_name = gpio;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = release_and_exit;
	sigaction(SIGALRM, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
}

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
"  tone <gpio> <notes> [--base=<hz>] [--prescale=<n>] [--backend=dma|setwc|auto]\n"
"                                  (notes: \"freq:ms ...\"; bare number = rest ms;\n"
"                                   default backend is the stock DMA tone path\n"
"                                   with set_wc as fallback - set_wc alone can\n"
"                                   only beep once per boot on this driver)\n"
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
	ch = ioctl(fd, ioc_REQUEST(PWM_IOC_REQUEST), name);
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
	if (ioctl(fd, ioc_RELEASE(PWM_IOC_RELEASE), (unsigned long)ch) < 0)
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
	xioctl(fd, ioc_CONFIG(PWM_IOC_CONFIG), &cfg, "pwm_config");
	printf("configured %s (ch%d): freq=%u max_level=%u active_level=%u accuracy_priority=%s\n",
	       gpio, ch, cfg.freq_hz, cfg.max_level, cfg.active_level, accuracy);
	return 0;
}

static int cmd_set_level(int fd, const char *gpio, const char *level_s)
{
	struct pwm_ch_value v;

	v.channel = (uint32_t)pwm_request_channel(fd, gpio);
	v.value = (uint32_t)parse_long(level_s, "level");

	xioctl(fd, ioc_SET_LEVEL(PWM_IOC_SET_LEVEL), &v, "pwm_set_level");
	printf("%s level %u\n", gpio, v.value);
	return 0;
}

static int cmd_get_level(int fd, const char *gpio)
{
	struct pwm_level_args io;

	io.channel = (uint32_t)pwm_request_channel(fd, gpio);
	io.level = 0;

	xioctl(fd, ioc_GET_LEVEL(PWM_IOC_GET_LEVEL), &io, "pwm_get_level");
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

	xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
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

	xioctl(fd, ioc_SET_PRESCALE(PWM_IOC_SET_PRESCALE), &v, "pwm_set_prescale");
	printf("%s channel clock = parent / %u\n", gpio, v.value);
	return 0;
}

static int cmd_disable(int fd, const char *gpio)
{
	int ch = pwm_request_channel(fd, gpio);

	/* RELEASE passes the channel BY VALUE - see pwm_abi.h. */
	step("pwm_release ... ");
	if (ioctl(fd, ioc_RELEASE(PWM_IOC_RELEASE), (unsigned long)ch) < 0)
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
	if (ioctl(fd, ioc_GET_LEVEL(PWM_IOC_GET_LEVEL), &level) == 0)
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
/*
 * Loop length for DMA tones. The driver requires a multiple of 4 words;
 * 4 identical period-words is the shortest legal loop and repeats fast
 * enough that the output is a continuous tone.
 */
#define DMA_LOOP_WORDS 4

/*
 * Loop length for DMA tones. The driver requires a multiple of 4 words;
 * 4 identical period-words is the shortest legal loop and repeats fast
 * enough that the output is a continuous tone.
 */
#define DMA_LOOP_WORDS 4

/*
 * Push one period word `count` times and start the loop. Same 16-bit
 * halves as set_wc; the DMA engine replays them continuously until
 * disabled, with none of set_wc's one-shot update-engine wedging.
 */
static void dma_note(int fd, int ch, long half)
{
	struct pwm_dma_op_args op;
	uint32_t words[DMA_LOOP_WORDS];
	int i;

	for (i = 0; i < DMA_LOOP_WORDS; i++)
		words[i] = PWM_WC(half, half);

	memset(&op, 0, sizeof(op));
	op.channel = (uint32_t)ch;
	op.count = DMA_LOOP_WORDS;
	op.data = (uint32_t)(uintptr_t)words;
	op.op = 0; /* copy the buffer in */
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_update");
	op.op = 1; /* loop it */
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_enable_loop");
}

/*
 * Swap the loop's waveform without disabling the channel. Measured on the
 * rig: with a disable/enable pair around every note, only the first and
 * last notes of a sequence are audible (the enable-after-disable middle
 * notes come back driver-ok but silent). Rests in legato mode are a
 * minimum-length period - far above hearing - rather than a stop.
 */
static void dma_renote(int fd, int ch, long half)
{
	struct pwm_dma_op_args op;
	uint32_t words[DMA_LOOP_WORDS];
	int i;

	for (i = 0; i < DMA_LOOP_WORDS; i++)
		words[i] = PWM_WC(half, half);

	memset(&op, 0, sizeof(op));
	op.channel = (uint32_t)ch;
	op.count = DMA_LOOP_WORDS;
	op.data = (uint32_t)(uintptr_t)words;
	op.op = 0; /* swap the buffer under the running loop */
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_update");
}

static void dma_silence(int fd, int ch)
{
	if (ioctl(fd, ioc_DMA_DISABLE_LOOP(PWM_IOC_DMA_DISABLE_LOOP),
		  (unsigned long)ch) < 0)
		die("failed: pwm_dma_disable_loop (%s)\n", strerror(errno));
}

static int cmd_tone(int fd, const char *gpio, const char *notes,
		    long base, long prescale, const char *backend)
{
	struct pwm_config_args cfg;
	struct pwm_ch_value v;
	struct pwm_dma_init_args di;
	const char *p = notes;
	long clock = base / prescale;
	int ch, use_dma;

	if (clock <= 0)
		die("error: base clock %ld / prescale %ld is not positive\n", base, prescale);

	/*
	 * The default watchdog is tuned for one-shot verbs; a tune can be
	 * minutes long. Re-arm to the tune's total duration plus slack so
	 * the alarm only fires on a real wedge, never on a long melody.
	 */
	{
		double total_ms = 0;
		char *end;
		const char *q = notes;

		while (*q) {
			double v = strtod(q, &end);

			if (end == q)
				break;
			q = end;
			if (*q == ':') {
				q++;
				strtod(q, &end);
				q = end;
			}
			total_ms += v;
			while (*q == ' ' || *q == '\t')
				q++;
		}
		alarm(0);
		alarm((unsigned)(total_ms / 1000.0) + 3);
	}

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

	/*
	 * A channel left "working" by the stock boot beep (or any earlier
	 * set_wc success) refuses both CONFIG and DMA_INIT, and only RELEASE
	 * clears the flag - measured on the rig. Release and re-request
	 * unconditionally so tone works whatever state the machine is in.
	 */
	pwm_release_channel(fd, ch, gpio);
	ch = pwm_request_channel(fd, gpio);
	arm_signal_release(fd, ch, gpio);

	cfg.channel = (uint32_t)ch;
	xioctl(fd, ioc_CONFIG(PWM_IOC_CONFIG), &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, ioc_SET_PRESCALE(PWM_IOC_SET_PRESCALE), &v, "pwm_set_prescale");

	/*
	 * DMA is how stock beeps; prefer it and fall back to the one-shot
	 * set_wc path when the ioctl is not recognized (older module) or the
	 * channel refuses it. --backend= pins the choice for bring-up.
	 */
	use_dma = strcmp(backend, "setwc") != 0;
	if (use_dma) {
		memset(&di, 0, sizeof(di));
		di.channel = (uint32_t)ch;
		di.active_level = 1;
		di.loop = 1;
		if (ioctl(fd, ioc_DMA_INIT(PWM_IOC_DMA_INIT), &di) < 0) {
			if (strcmp(backend, "dma") == 0)
				die("failed: pwm_dma_init (%s, see dmesg)\n",
				    strerror(errno));
			step("dma init refused (%s), falling back to set_wc\n",
			     strerror(errno));
			use_dma = 0;
		}
	}

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

			if (use_dma) {
				if (ms > 0) {
					dma_note(fd, ch, half);
					usleep((useconds_t)(ms * 1000));
					dma_silence(fd, ch);
				}
				continue;
			}
			v.value = PWM_WC(half, half);
			xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
		} else if (!use_dma) {
			v.value = PWM_WC(0, 0); /* stock silence idiom */
			xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
		}

		if (ms > 0)
			usleep((useconds_t)(ms * 1000));
	}

	if (!use_dma) {
		v.value = PWM_WC(0, 0);
		xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
	}
	/* DMA mode needs no trailing stop: every note's disable already ran,
	 * and a further disable_loop is refused with "dma is not loop". */
	g_channel = -1; /* the release below is the clean one */
	pwm_release_channel(fd, ch, gpio);
	return 0;
}

/*
 * Class-D bring-up probe: one large buffer of period words at a FIXED
 * carrier, each word's duty encoding one sample of a triangle wave. If
 * the engine walks the whole buffer each loop, the piezo reproduces the
 * encoded tone through its own resonance; if it only replays the first
 * word, the carrier alone (or nothing) sounds. Tuner-readable either way:
 * reading the encoded tone means duty-modulated sample streaming works,
 * which is the foundation for real audio synthesis on this transducer.
 */
#define SINE_BUF_WORDS 4096
#define CHORD_MAX_NOTES 4

/*
 * Triangle sample of `freq` at carrier-sample index i, scaled
 * -1000..1000. Phase is tracked in 1/1000-cycle units so integer
 * arithmetic stays exact for fractional frequencies.
 */
static long tri_sample(double freq, long i, long c_total, long c_half)
{
	long pos = (long)((unsigned long long)(freq * 1000.0) * i) % c_total;

	if (pos < c_half)
		return -1000 + 2000 * pos / c_half;
	return 1000 - 2000 * (pos - c_half) / c_half;
}

/*
 * Fill a duty-encoded buffer with the sum of up to CHORD_MAX_NOTES
 * triangle generators - a chord in a single one-shot loop. Tuner-
 * verified on the rig: the piezo demodulates duty-encoded audio (a
 * single 440 Hz triangle reads as its 440 fundamental plus a
 * resonance-boosted 11th harmonic near 4.8 kHz), so simultaneous tones
 * survive the transducer.
 */
static void fill_chord_words(uint32_t *words, long nwords,
			     const double *notes, int n_notes,
			     long period, long c_total, long c_half)
{
	long half = period / 2;
	long amp = period * 2 / 5;
	long w[CHORD_MAX_NOTES], wsum = 0;
	long i;
	int n;

	/*
	 * The disc's response rises with frequency (its mechanical
	 * resonance sits near 5 kHz), so an unweighted chord comes out
	 * treble-heavy. Weight notes by inverse SQUARE ROOT of frequency -
	 * ear-tuned on the rig: 1/f overcorrected, unweighted was too
	 * bright, f^-0.5 sits between.
	 */
	for (n = 0; n < n_notes; n++) {
		long ratio = (long)(notes[0] * 1000000.0 / notes[n]);
		long r = ratio, t;

		while (r > 0 && (t = (r + ratio / r) / 2) < r)
			r = t; /* integer Newton sqrt */
		w[n] = r;
		wsum += w[n];
	}

	for (i = 0; i < nwords; i++) {
		long long acc = 0;
		long t, hi;

		for (n = 0; n < n_notes; n++)
			acc += (long long)tri_sample(notes[n], i, c_total, c_half) * w[n];
		t = (long)(acc * 1000 / wsum);
		hi = half + t * amp / 1000;
		if (hi < 1)
			hi = 1;
		if (hi > period - 1)
			hi = period - 1;
		words[i] = PWM_WC(period - hi, hi);
	}
}

static int cmd_sine(int fd, const char *gpio, const double *notes,
		    int n_notes, double ms, double carrier, long base,
		    long prescale, double swap)
{
	struct pwm_config_args cfg;
	struct pwm_ch_value v;
	struct pwm_dma_init_args di;
	struct pwm_dma_op_args op;
	static uint32_t words[SINE_BUF_WORDS];
	double swap_notes[1];
	long clock = base / prescale;
	long period, c_total, c_half;
	int ch;

	if (clock <= 0)
		die("error: base clock %ld / prescale %ld is not positive\n",
		    base, prescale);
	period = (long)((double)clock / carrier);
	if (period < 2 || period > 131070)
		die("error: carrier %.0f Hz does not fit one word "
		    "(channel clock %ld Hz)\n", carrier, clock);
	/* integer phase: track freq*1000 modulo carrier*1000 */
	c_total = (long)(carrier * 1000.0);
	c_half = c_total / 2;

	fill_chord_words(words, SINE_BUF_WORDS, notes, n_notes,
			 period, c_total, c_half);

	alarm((unsigned)(ms / 1000.0) + 3);

	memset(&cfg, 0, sizeof(cfg));
	cfg.active_level = 1;
	cfg.levels_exact = 0;
	cfg.freq_hz = 1000000; /* conservative: see cmd_tone */
	cfg.max_level = 300;

	ch = pwm_request_channel(fd, gpio);
	pwm_release_channel(fd, ch, gpio);
	ch = pwm_request_channel(fd, gpio);
	arm_signal_release(fd, ch, gpio);

	cfg.channel = (uint32_t)ch;
	xioctl(fd, ioc_CONFIG(PWM_IOC_CONFIG), &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, ioc_SET_PRESCALE(PWM_IOC_SET_PRESCALE), &v, "pwm_set_prescale");

	memset(&di, 0, sizeof(di));
	di.channel = (uint32_t)ch;
	di.active_level = 1;
	di.loop = 1;
	xioctl(fd, ioc_DMA_INIT(PWM_IOC_DMA_INIT), &di, "pwm_dma_init");

	memset(&op, 0, sizeof(op));
	op.channel = (uint32_t)ch;
	op.count = SINE_BUF_WORDS;
	op.data = (uint32_t)(uintptr_t)words;
	op.op = 0;
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_update");
	op.op = 1;
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_enable_loop");
	printf("sine %.0f Hz (%d note%s), %.0f Hz carrier (%ld counts/cycle), "
	       "%d words (%.0f ms), %.0f ms\n", notes[0], n_notes,
	       n_notes == 1 ? "" : "s", carrier, period, SINE_BUF_WORDS,
	       SINE_BUF_WORDS * 1000.0 / carrier, ms);
	fflush(stdout);
	if (swap > 0) {
		usleep((useconds_t)(ms * 500.0));
		/* swap the running loop's buffer: one op-0 copy, no
		 * disable/enable around it - the streaming primitive */
		swap_notes[0] = swap;
		fill_chord_words(words, SINE_BUF_WORDS, swap_notes, 1,
				 period, c_total, c_half);
		op.op = 0;
		xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_update");
		printf("swapped to %.0f Hz under the running loop\n", swap);
		fflush(stdout);
		usleep((useconds_t)(ms * 500.0));
	} else {
		usleep((useconds_t)(ms * 1000));
	}
	dma_silence(fd, ch);
	g_channel = -1; /* the release below is the clean one */
	pwm_release_channel(fd, ch, gpio);
	return 0;
}

/*
 * Raw sample player: read a file of little-endian u32 period words (the
 * duty-encoding produced by a synthesizer elsewhere - helixscreen's
 * sound backend), copy+loop it, hold for --ms, then stop and release.
 * Word count must be a multiple of 4 (driver rule). The caller owns the
 * DSP; this verb is deliberately dumb.
 */
static int cmd_words(int fd, const char *gpio, const char *path,
		     long ms, long prescale)
{
	struct pwm_config_args cfg;
	struct pwm_ch_value v;
	struct pwm_dma_init_args di;
	struct pwm_dma_op_args op;
	static uint32_t words[65536];
	FILE *f;
	long nwords = 0;
	int ch;

	f = fopen(path, "rb");
	if (!f)
		die("error: cannot open %s\n", path);
	nwords = (long)fread(words, 4, sizeof(words) / 4, f);
	fclose(f);
	if (nwords < 4 || nwords % 4 != 0)
		die("error: %s: %ld words (need a positive multiple of 4)\n",
		    path, nwords);

	alarm((unsigned)(ms / 1000) + 3);

	memset(&cfg, 0, sizeof(cfg));
	cfg.active_level = 1;
	cfg.levels_exact = 0;
	cfg.freq_hz = 1000000; /* conservative: see cmd_tone */
	cfg.max_level = 300;

	ch = pwm_request_channel(fd, gpio);
	pwm_release_channel(fd, ch, gpio);
	ch = pwm_request_channel(fd, gpio);
	arm_signal_release(fd, ch, gpio);

	cfg.channel = (uint32_t)ch;
	xioctl(fd, ioc_CONFIG(PWM_IOC_CONFIG), &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, ioc_SET_PRESCALE(PWM_IOC_SET_PRESCALE), &v, "pwm_set_prescale");

	memset(&di, 0, sizeof(di));
	di.channel = (uint32_t)ch;
	di.active_level = 1;
	di.loop = 1;
	xioctl(fd, ioc_DMA_INIT(PWM_IOC_DMA_INIT), &di, "pwm_dma_init");

	memset(&op, 0, sizeof(op));
	op.channel = (uint32_t)ch;
	op.count = (uint32_t)nwords;
	op.data = (uint32_t)(uintptr_t)words;
	op.op = 0;
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_update");
	op.op = 1;
	xioctl(fd, ioc_DMA_OP(PWM_IOC_DMA_OP), &op, "pwm_dma_enable_loop");
	step("words: %ld words, %ld ms\n", nwords, ms);
	usleep((useconds_t)(ms * 1000));
	dma_silence(fd, ch);
	g_channel = -1; /* the release below is the clean one */
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
	xioctl(fd, ioc_CONFIG(PWM_IOC_CONFIG), &cfg, "pwm_config");

	v.channel = (uint32_t)ch;
	v.value = (uint32_t)prescale;
	xioctl(fd, ioc_SET_PRESCALE(PWM_IOC_SET_PRESCALE), &v, "pwm_set_prescale");
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
		xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
		usleep((useconds_t)ms * 1000);

		v.value = PWM_WC(0, 0); /* stock silence idiom */
		xioctl(fd, ioc_SET_WC(PWM_IOC_SET_WC), &v, "pwm_set_wc");
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
		return cmd_channels(fd, argc - 2, argv + 2, ioc_ENABLE_CHANNELS(PWM_IOC_ENABLE_CHANNELS), "pwm_enable_channels");
	if (strcmp(verb, "disable_channels") == 0)
		return cmd_channels(fd, argc - 2, argv + 2, ioc_DISABLE_CHANNELS(PWM_IOC_DISABLE_CHANNELS), "pwm_disable_channels");
	if (strcmp(verb, "not_really_enable") == 0)
		return cmd_not_really(fd, gpio, ioc_NOT_REALLY_ENABLE(PWM_IOC_NOT_REALLY_ENABLE), "pwm_not_really_enable");
	if (strcmp(verb, "not_really_disable") == 0)
		return cmd_not_really(fd, gpio, ioc_NOT_REALLY_DISABLE(PWM_IOC_NOT_REALLY_DISABLE), "pwm_not_really_disable");
	if (strcmp(verb, "tone") == 0) {
		long base = 50000000, prescale = 6;
		const char *backend = "auto";

		if (argc < 4)
			die("error: tone needs a notes string, e.g. \"1000:100 50 1479:200\"\n");
		for (i = 4; i < argc; i++) {
			if (strncmp(argv[i], "--base=", 7) == 0)
				base = parse_long(argv[i] + 7, "base");
			else if (strncmp(argv[i], "--prescale=", 11) == 0)
				prescale = parse_long(argv[i] + 11, "prescale");
			else if (strncmp(argv[i], "--backend=", 10) == 0) {
				backend = argv[i] + 10;
				if (strcmp(backend, "dma") != 0 &&
				    strcmp(backend, "setwc") != 0 &&
				    strcmp(backend, "auto") != 0)
					die("error: --backend must be dma, setwc, or auto\n");
			} else if (strncmp(argv[i], "--", 2) != 0)
				die("error: not support this arg: %s\n", argv[i]);
		}
		return cmd_tone(fd, gpio, argv[3], base, prescale, backend);
	}
	if (strcmp(verb, "sine") == 0) {
		double notes[CHORD_MAX_NOTES], ms = 1500.0, carrier = 32000.0;
		double swap = 0.0;
		long base = 50000000, prescale = 6;
		int n_notes = 0;

		for (i = 4; i < argc; i++) {
			if (strncmp(argv[i], "--freq=", 7) == 0) {
				if (n_notes == 0) {
					notes[0] = strtod(argv[i] + 7, NULL);
					n_notes = 1;
				}
			} else if (strncmp(argv[i], "--notes=", 8) == 0) {
				char spec[128], *p, *tok;

				snprintf(spec, sizeof(spec), "%s", argv[i] + 8);
				n_notes = 0;
				for (tok = strtok_r(spec, ",", &p); tok;
				     tok = strtok_r(NULL, ",", &p)) {
					if (n_notes >= CHORD_MAX_NOTES)
						die("error: at most %d notes\n",
						    CHORD_MAX_NOTES);
					notes[n_notes++] = strtod(tok, NULL);
				}
				if (n_notes == 0)
					die("error: --notes= needs frequencies\n");
			} else if (strncmp(argv[i], "--carrier=", 10) == 0)
				carrier = strtod(argv[i] + 10, NULL);
			else if (strncmp(argv[i], "--ms=", 5) == 0)
				ms = strtod(argv[i] + 5, NULL);
			else if (strncmp(argv[i], "--swap=", 7) == 0)
				swap = strtod(argv[i] + 7, NULL);
			else if (strncmp(argv[i], "--base=", 7) == 0)
				base = parse_long(argv[i] + 7, "base");
			else if (strncmp(argv[i], "--prescale=", 11) == 0)
				prescale = parse_long(argv[i] + 11, "prescale");
			else if (strncmp(argv[i], "--", 2) != 0)
				die("error: not support this arg: %s\n", argv[i]);
		}
		if (n_notes == 0) {
			notes[0] = 440.0;
			n_notes = 1;
		}
		return cmd_sine(fd, gpio, notes, n_notes, ms, carrier,
				base, prescale, swap);
	}
	if (strcmp(verb, "words") == 0) {
		long ms = 500, prescale = 6;

		if (argc < 4)
			die("error: words needs a file path of u32 period words\n");
		for (i = 4; i < argc; i++) {
			if (strncmp(argv[i], "--ms=", 5) == 0)
				ms = parse_long(argv[i] + 5, "ms");
			else if (strncmp(argv[i], "--prescale=", 11) == 0)
				prescale = parse_long(argv[i] + 11, "prescale");
			else if (strncmp(argv[i], "--", 2) != 0)
				die("error: not support this arg: %s\n", argv[i]);
		}
		return cmd_words(fd, gpio, argv[3], ms, prescale);
	}
	if (strcmp(verb, "probe") == 0)
		return cmd_probe(fd, gpio);
	if (strcmp(verb, "selftest") == 0)
		return cmd_selftest(fd, argc - 2, argv + 2);

	usage(stderr);
	return 1;
}
