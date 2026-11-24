#!/bin/sh
# yuho の迷路シミュレータの CLI を PC 向けにビルドする(ファームウェアのビルドではない)。
# ファームウェアと同じ logic 層のソースをそのままコンパイルする。
#   build/maze_sim        CLI(--verbose で1区画ごとの地図、最短走行の経路を本当の迷路で走らせる確かめ)
# 画面(msim.py gui)と、まとめて試す msim.py batch は mouse maze sim が mouse.json を読んで自分でビルドする。
set -e
cd "$(dirname "$0")"
ROOT=../..
mkdir -p build

L=$ROOT/lib/mouse_logic/Src/logic
LOGIC="$L/command.c $L/maze/*.c $L/control/slalom.c $L/control/velocity_profile.c"
CFLAGS="-std=c11 -Wall -Wextra -O2 -I$ROOT/Core/Inc -I$ROOT/lib/mouse_logic/Inc"

gcc $CFLAGS -o build/maze_sim maze_sim.c sim_core.c ideal_time.c $LOGIC -lm
echo "built tools/maze_sim/build/maze_sim"
