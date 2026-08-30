# fx-pwm

Userspace driver for the FlashForge AD5X piezo buzzer (Ingenic X2600), and a
cmd_pwm-compatible CLI for the stock kernel's PWM ioctl ABI.

## Why this exists

The stock AD5X firmware beeps by forking `/usr/bin/cmd_pwm` (a thin CLI over
`libhardware2.so`) at the buzzer; both live in the OEM rootfs. The Forge-X
rootfs this tree builds replaces that userland, so under Forge-X the buzzer
is silent even though the kernel side is fully intact: the OEM boot script
`driver_default_init_script.sh` insmods `soc_pwm.ko` ("Ingenic SoC PWM
driver", Ingenic's own x2600_510 SDK module) at every boot, and that module
publishes the PWM as a misc char device, `/dev/jz_pwm`.

`fx-pwm` is the missing client: it speaks the same ioctl ABI and accepts the
same command surface as the stock `cmd_pwm`, so beep hooks (M300 handling,
macros) carry over unchanged:

```
fx-pwm config pc12 freq=50000000 max_level=300 active_level=1 accuracy_priority=freq
fx-pwm set_prescale pc12 6
fx-pwm set_wc pc12 1600 1600        # the tone: raw duty halves
fx-pwm set_level pc12 100           # duty as level/max_level of the period
fx-pwm disable pc12
```

Run `fx-pwm --help` for the full verb list (`config`, `set_level`,
`get_level`, `set_wc`, `set_prescale`, `disable`, `enable_channels`,
`disable_channels`, `not_really_enable/disable`, `selftest`). `selftest`
plays two tones bracketing the plausible parent-clock rates and then
releases the channel - it exists for bring-up with a human listening.

## Hardware facts (with provenance)

All of the below was derived from artifacts of the stock AD5X Factory
bundle (kernel 5.10.186, Ingenic x2600_510 SDK): `soc_pwm.ko`,
`libhardware2.so`, `cmd_pwm`, and `firmwareExe`'s embedded buzzer.cpp
strings. Nothing came from any third-party mod.

- The buzzer GPIO is **pc12**, per stock `firmwareExe` itself:
  `cmd_pwm set_level pc12 100` is hardcoded in its buzzer path.
- The X2600 PWM is a dedicated **PWM2 IP block at physical 0x13610000**
  (KSEG1 0xb3610000 in the driver), NOT the Ingenic TCU. Mainline Linux
  has no driver for it: `drivers/pwm/pwm-jz4740.c` only covers TCU-based
  SoCs (jz4740/jz4725b/x1000), and the Ingenic-community kernel tree has
  no X2600 DTS either. There is no mainline register map to cite - which
  is fine, because this tool pokes no registers; the stock GPL kernel
  module owns the block and does every write.
- The driver's channel table (`pwm_gpio_array`) maps **pc12..pc19 to
  channels 0..7** ("pwm0".."pwm7"; gpio id encoding is `(port<<4)|pin`,
  so pc12 = 0x2c = 44). The earlier note "channel 3" was a misread of the
  stock constants; pc12 is channel 0.
- Clocking: the driver takes its parent rate from the DT clocks
  `div_ahb2`/`div_pwm`/`gate_pwm`, defaulting its rate variable to
  **500 MHz** before `clk_get_rate()`. The stock buzzer stack is written
  against a **50 MHz** parent (`freq=50000000` in firmwareExe's config
  call). `set_prescale 6` sets the channel's clock divider register to
  N-1, i.e. channel clock = parent/6. At 50 MHz that is ~8.33 MHz; a
  `set_wc` total of ~3200 counts gives a ~2.6 kHz tone.
- Which quantity `config` makes exact is chosen by
  `accuracy_priority=freq|levels`: `freq` keeps the channel clock at the
  parent rate and sizes the period as clk/freq; `levels` pins the period
  to max_level counts (the level scale is then exact counts and the
  output frequency becomes clk/max_level). See `src/pwm_abi.h`.

## The ioctl ABI (decoded, cross-verified)

Command numbers and struct layouts were decoded from the vendor module's
dispatcher and cross-checked against the stock `libhardware2.so`; the two
agree on every field. Type is 'P' (0x50):

| ioctl | arg | meaning |
|---|---|---|
| `0x8001500b` | `char[12]` gpio name | request by name ("pc12"); kernel resolves and refuses non-PWM gpios |
| `0x80045016` | `u32` channel **by value** | release (disable + unrequest) |
| `0x80185001` | 24-byte config struct | `{_pad, active_level, levels_exact, freq_hz, max_level, channel}` |
| `0xc004502d` | `{ch, hi<<16\|lo}` | set_wc: raw duty halves, latches period and muxes the pin |
| `0xc004502e` | `{ch, N}` | set_prescale: clock divider register = N-1 |
| `0xc004502c` | `{ch, level}` | set_level: duty = level x period / max_level |
| `0xc004502b` | `u32` ch in, level out | get_level |
| `0x80045058` / `0x80045059` | `{ch, 0}` | not-really-enable / -disable flags |
| `0x80045062` / `0x80045063` | `u32` channel mask | enable / disable bitmaps (raw reg +0x00/+0x04) |

The kernel rejects invalid gpios, unconfigured channels, levels above
max_level, and reconfiguration while running (printed to dmesg as
`PWM: ...`), which is the claim-checking this tool relies on instead of
its own register validation.

## PWM2 register map (informational)

Decoded from the vendor module; documented here for debugging only - fx-pwm
never touches these, and neither should anything else while soc_pwm.ko is
loaded, since the driver owns the block:

```
0x13610000 + 0x00    enable bitmap        (one bit per channel)
           + 0x04    disable bitmap
           + 0x10    update/latch         (write 1<<ch to commit period/duty)
           + 0x14    status               (busy bits; wait clear before latch)
           + 0x20    config-in-progress bitmap
           + 0x24    per-channel inactive-level bitmap
           + 0x28    per-channel active-level bitmap
           + 0x40 + 4*ch  clock divider (16-bit; channel clock = parent/(N+1))
           + 0x80 + 4*ch  duty: low half = active counts, high half = inactive
```

## Bring-up checklist

1. Inside the Forge-X chroot, check the node exists: `ls -l /dev/jz_pwm`.
   /dev/gpiochip0-2 being visible implies devtmpfs reaches the chroot and
   the node should be there too. If it is missing, create it from the
   host's `/sys/class/misc/jz_pwm/dev` (misc major 10) or bind the host
   /dev.
2. `dmesg | grep PWM` after each fx-pwm call - every kernel-side rejection
   prints its reason there.
3. `fx-pwm selftest` and listen: two ~400 ms tones. One is tuned for a
   50 MHz parent, the other for 500 MHz; whichever is audible pins down
   the actual rate for the tone math.
4. Do not sweep other channels: pc13..pc19 may be wired to other
   hardware on a given machine. pc12 is the buzzer; touch only it.
