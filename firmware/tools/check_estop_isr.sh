#!/usr/bin/env bash
# Builds the variant with the emergency ISR compiled in (the GPIO below is only a
# compile target, not an approved wiring) and checks that the handler lives in IRAM.
set -euo pipefail
cd "$(dirname "$0")/.."
if command -v idf.py >/dev/null; then
    idf=(idf.py)
elif [ -n "${IDF_PATH:-}" ] && command -v python >/dev/null; then
    idf=(python "$IDF_PATH/tools/idf.py")  # installs that define idf.py as a shell function
else
    echo 'Ative o ambiente ESP-IDF instalado antes de executar tools/check_estop_isr.sh.' >&2
    exit 1
fi
dir=build/estop-check
mkdir -p "$dir"
sed 's/^CONFIG_MAGNIBOT_ESTOP_GPIO=.*/CONFIG_MAGNIBOT_ESTOP_GPIO=17/' sdkconfig.defaults > "$dir/sdkconfig.defaults"
"${idf[@]}" -B "$dir" -D SDKCONFIG="$dir/sdkconfig" -D SDKCONFIG_DEFAULTS="$dir/sdkconfig.defaults" build
address=$(xtensa-esp32s3-elf-nm "$dir/magnibot_firmware.elf" | awk '$3 == "emergency_isr" {print $1}')
[ -n "$address" ] || { echo 'emergency_isr ausente no ELF.' >&2; exit 1; }
# ESP32-S3 internal instruction RAM window.
if (( 0x$address >= 0x40370000 && 0x$address < 0x403e0000 )); then
    echo "OK: emergency_isr em IRAM (0x$address)"
else
    echo "FALHA: emergency_isr fora da IRAM (0x$address)" >&2
    exit 1
fi
