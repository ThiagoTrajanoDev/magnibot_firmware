#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if command -v idf.py >/dev/null; then
    idf=(idf.py)
elif [ -n "${IDF_PATH:-}" ] && command -v python >/dev/null; then
    idf=(python "$IDF_PATH/tools/idf.py")  # installs that define idf.py as a shell function
else
    echo 'Ative o ambiente ESP-IDF instalado antes de executar tools/build.sh.' >&2
    exit 1
fi
"${idf[@]}" -B build/bench build
