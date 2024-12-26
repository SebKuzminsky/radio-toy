#!/bin/bash
set -e

clear

cargo build

echo 'bootloader' > /dev/ttyACM1 || true
sleep 2

sudo mount -o umask=0 /dev/sda1 /mnt
cargo run
sudo umount /mnt
