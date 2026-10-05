#!/bin/sh
# 迷路シミュレータを PC 向けにビルドする(ファームウェアのビルドではない)。
# ファームウェアと同じ logic 層のソースをそのままコンパイルする。
set -e
cd "$(dirname "$0")"
ROOT=../..
mkdir -p build
gcc -std=c11 -Wall -Wextra -O2 \
    -I"$ROOT/Core/Inc" \
    -o build/maze_sim \
    maze_sim.c \
    "$ROOT/Core/Src/logic/command.c" \
    "$ROOT"/Core/Src/logic/maze/*.c
echo "built tools/maze_sim/build/maze_sim"
