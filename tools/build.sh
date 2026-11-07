#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! command -v idf.py >/dev/null; then
    echo 'Ative o ambiente ESP-IDF instalado antes de executar tools/build.sh.' >&2
    exit 1
fi
idf.py -B build/bench build
