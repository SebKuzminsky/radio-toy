#!/bin/bash
set -e

cmake -S . -B build -D PICO_BOARD=pico
make -C build -j $(getconf _NPROCESSORS_ONLN)
