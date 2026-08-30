#!/usr/bin/env bash
#
# Buildroot post-build hook. $1 is TARGET_DIR.
set -euo pipefail
TARGET=$1

# The Forge-X payload symlinks /root/moonraker-env/moonraker into place at every
# boot (.shell/S00init:287) and launches $VENV/bin/python3 (.root/S65moonraker:6),
# so the image has to supply a venv that already exists and already works.
#
# It is built statically rather than by running `python -m venv`, because that
# would mean executing a MIPS interpreter on an x86 build host. The layout below
# is byte-for-byte what `python3 -m venv` produces on Linux, where symlinked
# interpreters are the default.
#
# include-system-site-packages is true on purpose: every dependency Buildroot
# builds lands in /usr/lib/pythonX.Y/site-packages, and a sealed venv would hide
# all of them. ZMOD duplicates the whole dependency set into its venv instead;
# that costs ~25 MB for nothing on an appliance running exactly one Python app.
# Glob rather than `ls | head`: with no Python in the rootfs -- which is every
# toolchain-only build -- `ls` exits non-zero, and under `set -euo pipefail`
# that takes the whole build down at target-finalize.
PYVER=
for _d in "$TARGET"/usr/lib/python3.*; do
    [ -d "$_d" ] || continue
    PYVER=${_d##*/python}
    break
done

if [ -n "$PYVER" ]; then
    VENV=$TARGET/root/moonraker-env
    mkdir -p "$VENV/bin" "$VENV/lib/python$PYVER/site-packages"
    ln -sf /usr/bin/python3 "$VENV/bin/python3"
    ln -sf python3 "$VENV/bin/python"
    cat > "$VENV/pyvenv.cfg" <<EOF
home = /usr/bin
include-system-site-packages = true
version = $PYVER
executable = /usr/bin/python3
EOF

    # .py/backlight.py has a `#!/usr/bin/env python` shebang and is executed
    # directly from .shell/screen.sh:292. Buildroot never creates the
    # unsuffixed name.
    ln -sf python3 "$TARGET/usr/bin/python"
fi

# .root/start.sh:68-70 runs every /etc/init.d/S* it finds. Buildroot's skeleton
# leaves service scripts there that would start a second syslogd and network
# stack alongside the host's. The directory is an extension point for the user,
# and must ship empty.
rm -f "$TARGET"/etc/init.d/S[0-9]*

# .shell/commands/zversion.sh:12 treats the presence of this file as "a ZMOD
# image is installed" and hard-fails. Make sure nothing ever creates it.
rm -f "$TARGET"/ZMOD

# .shell/common.sh:33 reads $MOD/version.txt as the "flashed core version" and
# .shell/commands/zversion.sh:17-26 compares it against the payload's own
# version. When it is missing the boot path stalls 30s
# (.shell/S00init:127-138), so this must never be silently skipped -- walk up
# looking for the payload's version.txt, and fall back to a marked development
# stamp rather than leaving the file absent.
version=${FORGEX_VERSION:-}
if [ -z "$version" ] && [ -n "${BR2_EXTERNAL_FORGEX_PATH:-}" ]; then
    _d=$BR2_EXTERNAL_FORGEX_PATH
    while [ "$_d" != "/" ]; do
        if [ -f "$_d/version.txt" ]; then
            version=$(cat "$_d/version.txt")
            break
        fi
        _d=$(dirname "$_d")
    done
fi
printf '%s\n' "${version:-0.0.0-dev}" > "$TARGET/version.txt"
