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
`disable_channels`, `not_really_enable/disable`, `tone`, `selftest`).
`selftest` plays two tones bracketing the plausible parent-clock rates and
then releases the channel - it exists for bring-up with a human listening.

## The stock working sequence (from firmwareExe's own disassembly)

firmwareExe's buzzer thread (buzzer.cpp, symbols intact in the factory
binary) does sprintf + system() into the STOCK cmd_pwm on the stock
rootfs - the boot chirp is literally `system("cmd_pwm ...")`. The exact
sequence:

    one-time (buzzerPlay ctor):
        cmd_pwm config <gpio> freq=50000000 max_level=300 active_level=1 accuracy_priority=freq
        cmd_pwm set_level <gpio> 100
        cmd_pwm set_prescale <gpio> 6
    per tune (thread):
        cmd_pwm enable_channels <gpio>
        cmd_pwm set_wc <gpio> <high> <low>
        cmd_pwm set_wc <gpio> 0 0        (silence)
        cmd_pwm disable_channels <gpio>

Note set_level BEFORE set_prescale in the ctor, and enable_channels
wrapping each tune. The rig's parent clock measures 384 MHz.

## OEM-build ioctl drift and the FX_PWM_IOC_* overrides

The rig's running soc_pwm is evidently NOT the Factory 1.1.7 build (its
kallsyms function order differs from the 1.1.7 file's - pwm2_request
sits below pwm_open), and its CONFIG command encoding apparently differs
while REQUEST and GET_LEVEL happen to match (silent -EPERM = the
dispatcher's unrecognized-command default). The matched client for any
machine is its OWN /usr/lib/libhardware2.so: decode its pwm_config call
site for the real number. Every fx-pwm ioctl number can be overridden
per-run without a rebuild, e.g.

    FX_PWM_IOC_CONFIG=0x........ fx-pwm config pc12 freq=50000000 ...

Valid names: REQUEST RELEASE CONFIG SET_WC SET_PRESCALE SET_LEVEL
GET_LEVEL ENABLE_CHANNELS DISABLE_CHANNELS NOT_REALLY_ENABLE
NOT_REALLY_DISABLE. Values come from decoding, never guessing.

## The tone verb and Forge-X's TONE command

Forge-X's Klipper plugin `tone_player.py` (the `TONE` command behind its
`M300`/`BEEP`/`M356` macros) drives the buzzer through
`/sys/class/pwm/pwmchip0/pwm6` - the sysfs PWM API, which the AD5X's stock
kernel does not provide (the AD5M kernel does; the plugin was written
against it). On the AD5X every `TONE` therefore fails to reach hardware.

`fx-pwm tone` accepts the plugin's NOTES grammar verbatim
(`"freq:ms freq:ms 50"` - a bare number is a rest) and plays the whole
sequence in one process, so the plugin port is a backend swap:

```python
subprocess.Popen(["/usr/bin/fx-pwm", "tone", "pc12", notes_str])
```

`M300 S0 P1` (HelixScreen's silence command) arrives as `NOTES="0:1"` -
a rest, which this verb treats as output-off for the note's duration, so
it is a no-op on silence. A `Popen` (not `run`) keeps the gcode queue
free: the tone plays while klippy moves on; the current plugin's
`reactor.pause`-per-note blocking disappears with the port.

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
- The driver's channel table (`pwm_gpio_array`) uses gpio ids encoded
  **(port<<5)|pin** (pc12 = 0x4c = 76) and maps:
  pb12..pb19 -> pwm0..pwm7, pc7..pc14 -> pwm8..pwm15, pe2 -> pwm10 (alt).
  **pc12, the buzzer, is pwm13 = channel 13.** fx-pwm never relies on
  this: every verb sends the gpio NAME in the REQUEST ioctl and uses the
  channel index the kernel returns, so a firmware update that reshuffles
  the table cannot redirect the tool. (Confirmed on the rig: the kernel
  resolves pc12 -> 13.)
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

## Driver bugs to design around (decoded from the vendor module)

- **Silent EPERM:** an ioctl number the dispatcher does not recognize
  falls through to a default that returns -1 (EPERM) with NO printk. So
  "Operation not permitted" with nothing in dmesg means the number did
  not match this build of the driver - not a permissions problem.
- **REQUEST is a gpio lease:** pwm2_request internally calls the
  kernel's gpio_request(); pwm2_release calls gpio_free(). A process
  that REQUESTs and exits without RELEASE leaves the gpio claimed in
  gpiolib forever (the misc device's release fop frees nothing). Later
  REQUESTs still succeed because pwm2_request short-circuits when the
  channel is already requested - but rmmod'ing soc_pwm orphans the
  claim: the fresh instance's REQUEST fails loudly ("request IO <gpio>
  error !") and, on this vendor stack, a subsequent REQUEST can wedge
  uninterruptibly (D-state; soc_gpio lock path). NEVER rmmod soc_pwm
  after unreleased requests; a reboot is the only clean reset.
- fx-pwm arms an alarm() watchdog (default 5 s, `--timeout=<s>`) around
  every invocation, prints each ioctl step as it happens, and accepts
  only pc12 unless `--force-gpio` is passed. Note the watchdog bounds
  userspace waiting, not a thread already blocked in the kernel.
- **A failed REQUEST wedges the channel (empirical):** every observed
  D-state (three rig reboots) followed a REQUEST that failed with
  -EBUSY; the next ioctl on that channel then blocks inside
  mutex_lock forever (kernel stack: pwm2_request+0xac, the instruction
  after the mutex_lock call). Statically the request error path does
  unlock (branch to the LO16 with the HI16 in its delay slot) - the
  wedge mechanism is unexplained, so fx-pwm treats any REQUEST failure
  as wedging: it writes /run/fx-pwm.<gpio>.wedged and later invocations
  refuse before entering the kernel until the next reboot.
- **Who claims pc12 during a Forge-X boot (open):** on a fresh boot with
  no fx-pwm/stock touches, gpio_request(PC12) returns -EBUSY. Leading
  theory: the stock firmwareExe runs briefly at boot before Forge-X
  disarms it - long enough to REQUEST pc12 through soc_pwm (the same
  path stock beeps use), leaving a gpiolib claim that process death
  never frees. That claim also sets the driver's requested flag, which
  is why requests on the boot instance "succeed" (they take the
  skip-path) while a reloaded instance fails. Identification, one
  command: mount debugfs, then `cat /sys/kernel/debug/gpio` - pc12's
  label will read `pwm13` if the claim came through soc_pwm (ours or
  firmwareExe's), anything else names the real claimant. Note
  clk_summary reads empty until debugfs is actually mounted.

`fx-pwm probe <gpio>` is the first diagnostic to run anywhere: it
reports the channel the KERNEL assigns to the gpio and changes no
output.

## The parent clock and the boot-state dependency (open)

The driver's probe (clock names decoded from the module: `div_ahb2`,
`div_pwm`, `gate_pwm`) reads div_ahb2's rate, then checks
`__clk_is_enabled(gate_pwm)`: if the gate is ALREADY enabled it skips
clock setup entirely and leaves its rate variable at the built-in
500 MHz default; otherwise it runs `clk_set_rate(div_pwm, 500000000)`
plus prepare/enable, and the dance's return value overwrites the rate
variable. pwm2_config rejects any freq above that rate variable.

Rig observation (2026-08-30): on a Forge-X boot, config(freq=1 MHz)
was rejected, implying the rate variable ended up near zero - i.e. the
gate was off at probe time and the setup dance produced ~0. On stock
boots the gate is already enabled by earlier boot activity, the dance
is skipped, and the 500 MHz default holds. This is the leading theory
for the whole "works on stock, EPERMs under Forge-X" matrix; the
next-session runbook below settles it.

Bring-up runbook (in this order, stock-side dmesg reads):
1. Reboot; wait for the UI; do NOT run fx-pwm yet.
2. `dmesg | grep -i pwm` - probe-time clk errors print here.
3. `cat /sys/kernel/debug/clk/clk_summary | grep -iE 'pwm|ahb'` -
   div_pwm's rate and enable count, the ground truth for the theory.
4. `rmmod soc_pwm && insmod /module_driver/soc_pwm.ko` - re-probe now
   that boot services may have enabled the gate (safe: nothing has
   requested pc12 yet, so no orphaned claim).
5. clk_summary again; note whether div_pwm changed.
6. `fx-pwm probe pc12 --timeout=3` (expect channel 13).
7. `fx-pwm config pc12 freq=1000000 max_level=300 active_level=1 accuracy_priority=freq`
   then `dmesg | grep -i pwm`. A "frequency low than <N>" line names the
   live rate N; retry with freq <= N if needed.
8. `fx-pwm set_prescale pc12 6; fx-pwm set_wc pc12 1600 1600` - listen.
   Then `fx-pwm selftest` for the parent-rate bracket, and
   `fx-pwm disable pc12` to silence.

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
