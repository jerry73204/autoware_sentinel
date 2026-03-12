#!/usr/bin/env bash
# Download the Jetson Linux L4T 36.4.4 SPE FreeRTOS BSP and
# the ARM GNU Toolchain 13.2.Rel1 (arm-none-eabi).
#
# Skips downloads when the local file's cksum already matches the expected value.
# After download, extracts the SPE BSP from the nested tarball inside
# public_sources.tbz2 and unpacks the toolchain.
#
# Usage: ./scripts/spe/download-bsp.sh [DOWNLOAD_DIR]
#   DOWNLOAD_DIR defaults to scripts/spe/downloads/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DOWNLOAD_DIR="${1:-$SCRIPT_DIR/downloads}"

# ── URLs and checksums (cksum output: checksum size filename) ────────────────
PUBLIC_SOURCES_URL="https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.4/sources/public_sources.tbz2"
PUBLIC_SOURCES_CKSUM="3391786610"

TOOLCHAIN_URL="https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz"
TOOLCHAIN_CKSUM="1842230340"

# ── Helpers ──────────────────────────────────────────────────────────────────
info()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
warn()  { printf '\033[1;33m==> %s\033[0m\n' "$*"; }
die()   { printf '\033[1;31m==> %s\033[0m\n' "$*" >&2; exit 1; }

# check_cksum FILE EXPECTED
# Returns 0 if EXPECTED is non-empty and matches, 1 otherwise.
check_cksum() {
    local file="$1" expected="$2"
    [ -n "$expected" ] || return 1
    [ -f "$file" ] || return 1
    local actual
    actual="$(cksum "$file" | awk '{print $1}')"
    [ "$actual" = "$expected" ]
}

# download_if_needed URL DEST EXPECTED_CKSUM
download_if_needed() {
    local url="$1" dest="$2" expected="${3:-}"
    if check_cksum "$dest" "$expected"; then
        info "Checksum OK, skipping: $(basename "$dest")"
        return 0
    fi
    if [ -f "$dest" ] && [ -z "$expected" ]; then
        info "Already downloaded (no checksum to verify): $(basename "$dest")"
        return 0
    fi
    info "Downloading $(basename "$dest") ..."
    curl -L --fail --progress-bar -o "$dest.part" "$url"
    mv "$dest.part" "$dest"
    info "cksum: $(cksum "$dest")"
}

# ── Main ─────────────────────────────────────────────────────────────────────
mkdir -p "$DOWNLOAD_DIR"

# 1. public_sources.tbz2
PUBLIC_SOURCES="$DOWNLOAD_DIR/public_sources.tbz2"
download_if_needed "$PUBLIC_SOURCES_URL" "$PUBLIC_SOURCES" "$PUBLIC_SOURCES_CKSUM"

# 2. Extract SPE BSP from public_sources.tbz2
SPE_DIR="$DOWNLOAD_DIR/spe-freertos-bsp"
if [ -d "$SPE_DIR" ]; then
    info "SPE BSP already extracted at: $SPE_DIR"
else
    info "Searching for SPE tarball inside public_sources.tbz2 ..."
    SPE_TARBALL=$(tar -tjf "$PUBLIC_SOURCES" | grep -iE 'spe|freertos|aux.cpu' | head -1 || true)

    if [ -z "$SPE_TARBALL" ]; then
        die "No SPE tarball found inside public_sources.tbz2. Contents:\n$(tar -tjf "$PUBLIC_SOURCES" | head -30)"
    fi

    info "Found: $SPE_TARBALL"
    tar -xjf "$PUBLIC_SOURCES" -C "$DOWNLOAD_DIR" "$SPE_TARBALL"

    SPE_INNER="$DOWNLOAD_DIR/$SPE_TARBALL"
    mkdir -p "$SPE_DIR"
    info "Extracting $SPE_INNER → $SPE_DIR ..."
    tar -xf "$SPE_INNER" -C "$SPE_DIR" --strip-components=1 2>/dev/null \
        || tar -xf "$SPE_INNER" -C "$SPE_DIR"
    info "SPE BSP extracted."
fi

# 3. ARM toolchain
TOOLCHAIN_ARCHIVE="$DOWNLOAD_DIR/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz"
TOOLCHAIN_DIR="$DOWNLOAD_DIR/arm-gnu-toolchain-13.2.rel1"
download_if_needed "$TOOLCHAIN_URL" "$TOOLCHAIN_ARCHIVE" "$TOOLCHAIN_CKSUM"

if [ -d "$TOOLCHAIN_DIR" ]; then
    info "Toolchain already extracted at: $TOOLCHAIN_DIR"
else
    info "Extracting ARM toolchain ..."
    mkdir -p "$TOOLCHAIN_DIR"
    tar -xJf "$TOOLCHAIN_ARCHIVE" -C "$TOOLCHAIN_DIR" --strip-components=1
    info "Toolchain extracted."
fi

# 4. Summary
echo ""
info "Done."
echo ""
echo "  SPE BSP:    $SPE_DIR"
echo "  Toolchain:  $TOOLCHAIN_DIR"
echo ""
echo "  export SPE_FREERTOS_BSP=$SPE_DIR"
echo "  export CROSS_COMPILE=$TOOLCHAIN_DIR/bin/arm-none-eabi-"
