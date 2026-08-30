# Build environment for the Forge-X platform rootfs.
#
# Pinned rather than :latest so that a rootfs built today and one built next
# year come from the same host toolchain. Buildroot builds its own target
# toolchain, but its host tools are compiled with whatever gcc is here.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        bc bison build-essential ca-certificates cpio file flex g++ gawk gcc \
        git gzip libncurses-dev libssl-dev locales make patch perl pkg-config \
        python3 python3-dev rsync sed tar unzip wget xz-utils bzip2 \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# Buildroot refuses to run some steps as root and produces root-owned output
# that is painful to clean up from the host. Build as an ordinary user whose
# uid the caller can override to match their own.
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID builder && useradd -u $UID -g $GID -m -s /bin/bash builder

USER builder
WORKDIR /work
