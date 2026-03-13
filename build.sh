#!/bin/bash
set -e

CONTAINER=$(docker ps | grep 'vsc-zmk' | awk '{print $1}')
BUILD_DIR="/workspaces/zmk/app"
CONFIG="/workspaces/zmk-config/config"
BOARD="nice_nano//zmk"
OUTPUT_DIR="/workspaces/zmk-config"

build() {
    local shield=$1
    local extra_modules=$2
    local output_name=$3

    echo "=== Building $output_name ==="
    docker exec -w "$BUILD_DIR" "$CONTAINER" \
        west build -o=-j12 -p -b "$BOARD" -- \
        -DZMK_CONFIG="$CONFIG" \
        -DSHIELD="$shield" \
        ${extra_modules:+"-DZMK_EXTRA_MODULES=$extra_modules"}

    docker exec "$CONTAINER" cp "$BUILD_DIR/build/zephyr/zmk.uf2" "$OUTPUT_DIR/$output_name"
    echo "=== $output_name done ==="
}

build_dongle() {
    local board="${DONGLE_BOARD:-xiao_ble}"
    local target_name="${DONGLE_TARGET:-Charybdis}"
    echo "=== Building dongle.uf2 (board: $board, target: $target_name) ==="
    docker exec -w /workspaces/zmk "$CONTAINER" \
        west build -o=-j12 -p -b "$board" \
        -s /workspaces/zmk-config/dongle \
        -d /workspaces/zmk-config/dongle/build \
        -- -DBOARD_ROOT=/workspaces/zmk/app/module \
        -DDONGLE_TARGET_NAME="$target_name"

    docker exec "$CONTAINER" cp /workspaces/zmk-config/dongle/build/zephyr/zephyr.uf2 "$OUTPUT_DIR/dongle.uf2"
    echo "=== dongle.uf2 done ==="
}

case "${1:-all}" in
    left)
        build "charybdis_left" "/workspaces/zmk-modules/zmk-input-processor-text-mode/" "charybdis_left.uf2"
        ;;
    right)
        build "charybdis_right" "/workspaces/zmk-modules/zmk-pmw3610-driver/" "charybdis_right.uf2"
        ;;
    reset)
        build "settings_reset" "" "settings_reset.uf2"
        ;;
    dongle)
        build_dongle
        ;;
    all)
        build "charybdis_left" "/workspaces/zmk-modules/zmk-input-processor-text-mode/" "charybdis_left.uf2"
        build "charybdis_right" "/workspaces/zmk-modules/zmk-pmw3610-driver/" "charybdis_right.uf2"
        build "settings_reset" "" "settings_reset.uf2"
        build_dongle
        ;;
    *)
        echo "Usage: $0 {left|right|reset|dongle|all}"
        echo "  DONGLE_TARGET=Name  keyboard name to pair with (default: Charybdis)"
        exit 1
        ;;
esac

echo "=== Done ==="
ls -lh "$OUTPUT_DIR"/*.uf2 2>/dev/null
