#!/bin/bash

ZMK_DIR="$HOME/projects/zmk/app"
CONFIG_DIR="$HOME/projects/zmk-config/config"
OUTPUT_DIR="$HOME/Documents/skeletyl-firmware"
WIN_OUTPUT='C:\Users\Install\Documents\skeletyl-firmware'

mkdir -p $OUTPUT_DIR

cd $ZMK_DIR

python3 -m west build -p -b nice_nano_v2 -- -DZMK_CONFIG=$CONFIG_DIR -DSHIELD=tbkmini_left
mv build/zephyr/zmk.uf2 $OUTPUT_DIR/skeletyl-left.uf2

python3 -m west build -p -b nice_nano_v2 -- -DZMK_CONFIG=$CONFIG_DIR -DSHIELD=tbkmini_right
mv build/zephyr/zmk.uf2 $OUTPUT_DIR/skeletyl-right.uf2

cd -
cd $OUTPUT_DIR
nautilus . &
cd -
