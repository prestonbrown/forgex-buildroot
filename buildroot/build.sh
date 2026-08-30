#!/usr/bin/env bash
#
# Build a Forge-X platform rootfs.
#
#   ./build.sh ad5x            # build the AD5X (mipsel) rootfs
#   ./build.sh ad5x menuconfig # open Buildroot's config UI for that target
#   ./build.sh ad5x savedefconfig
#
# The result is platform/buildroot/output/<target>/images/rootfs.tar.xz, which
# is what the installer unpacks into $MOD on the printer.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)

# Pinned deliberately. Buildroot's Python version is a property of the release,
# and Moonraker requires >= 3.10, so moving this tag moves the interpreter.
BUILDROOT_TAG=${BUILDROOT_TAG:-2025.02.4}
BUILDROOT_URL=https://github.com/buildroot/buildroot.git

TARGET=${1:-}
shift || true
if [ -z "$TARGET" ]; then
    echo "usage: $0 <target> [make-target...]" >&2
    echo "targets: $(cd "$HERE/external/configs" && ls *_defconfig 2>/dev/null | sed 's/_defconfig//' | tr '\n' ' ')" >&2
    exit 2
fi

DEFCONFIG=$HERE/external/configs/${TARGET}_defconfig
[ -f "$DEFCONFIG" ] || { echo "no such target: $TARGET ($DEFCONFIG missing)" >&2; exit 2; }

SRC=$HERE/.buildroot
OUT=$HERE/output/$TARGET
# Shared across targets on purpose: the download cache is architecture
# independent and is by far the slowest thing to rebuild from cold.
DL=${BR2_DL_DIR:-$HERE/.dl}

if [ ! -d "$SRC/.git" ]; then
    echo "// Cloning Buildroot $BUILDROOT_TAG..."
    git clone --quiet "$BUILDROOT_URL" "$SRC"
fi
git -C "$SRC" fetch --tags --quiet
git -C "$SRC" checkout --quiet "$BUILDROOT_TAG"

mkdir -p "$OUT" "$DL"

MAKE_ARGS=(
    -C "$SRC"
    O="$OUT"
    BR2_EXTERNAL="$HERE/external"
    BR2_DL_DIR="$DL"
)

if [ ! -f "$OUT/.config" ]; then
    echo "// Configuring $TARGET..."
    make "${MAKE_ARGS[@]}" "${TARGET}_defconfig"
fi

# savedefconfig must write back to the tracked defconfig, not into output/.
if [ "${1:-}" = "savedefconfig" ]; then
    make "${MAKE_ARGS[@]}" savedefconfig BR2_DEFCONFIG="$DEFCONFIG"
    exit 0
fi

exec make "${MAKE_ARGS[@]}" "$@"
