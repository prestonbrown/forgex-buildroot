# forgex-br

The Forge-X platform rootfs: a [Buildroot](https://buildroot.org) external
tree that builds the MIPS userland the AD5X firmware image ships.

The AD5X (Ingenic X2600, MIPS32r5 little-endian) runs its klipper host
inside a rootfs we install at `/usr/data/.mod/.forge-x` on the printer.
This tree builds that rootfs from source - pinned Buildroot, pinned
packages, our board hooks - so the image no longer depends on a rootfs
borrowed from another project's release.

## Layout

```
buildroot/
  Dockerfile        pinned ubuntu:22.04 build environment
  build.sh          ./build.sh ad5x - target-driven entry over a pinned
                    Buildroot clone (BUILDROOT_TAG, default 2025.02.4)
  external/
    external.desc   the BR2_EXTERNAL tree declaration
    configs/        ad5x_defconfig - the target; carries the ABI pins
    board/ad5x/     busybox.fragment, post-build.sh
    package/        fx-pwm (AD5X buzzer via the stock soc_pwm.ko ioctl ABI)
                    and Moonraker's python dependencies as buildroot packages
```

## Building

On any Linux build host with Docker, with the tree checked out:

```sh
cd forgex-br/buildroot
docker build -t forgex-br .
docker run --rm -u $(id -u):$(id -g) -v "$PWD:$PWD" -w "$PWD" \
    forgex-br ./build.sh ad5x
```

The result is `buildroot/output/ad5x/images/rootfs.tar.xz`. The build is
slow the first time (download cache under `buildroot/.dl/`, shared across
targets on purpose) and incremental after.

Record the md5 of a new rootfs in the firmware repo's
`tools/release/rootfs.md5` - the image builder verifies the rootfs it is
handed against that allowlist and refuses the known foreign one.

## The ABI pins are load-bearing

`ad5x_defconfig` pins o32 / hard-float FP64 / NaN2008 with comments
explaining exactly what breaks when each is wrong (the loader name
`/lib/ld-linux-mipsn8.so.1` IS the NaN2008 marker). Do not touch those
lines without a stock-binary `readelf` session to compare against.

The upstream reference material for the AD5X port (factory images,
extracted stock configs) lives outside this repo.
