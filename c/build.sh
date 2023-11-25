#!/bin/bash
set -e

if [[ ! -v PICO_BOARD ]]; then
    PICO_BOARD="pico"
fi

cmake -S . -B "build_${PICO_BOARD}" -D PICO_BOARD=${PICO_BOARD}
make -C "build_${PICO_BOARD}" -j $(getconf _NPROCESSORS_ONLN)
