#!/bin/sh
# 迷路シミュレータを PC 向けにビルドする(ファームウェアのビルドではない)。
# ファームウェアと同じ logic 層のソースをそのままコンパイルする。
#   build/maze_sim        CLI
#   build/maze_sim_lib.*  GUI (gui.py) が読み込む共有ライブラリ(gui.py も必要なら自動でビルドする)
set -e
cd "$(dirname "$0")"
ROOT=../..
mkdir -p build

LOGIC="$ROOT/Core/Src/logic/command.c $ROOT/Core/Src/logic/maze/*.c $ROOT/Core/Src/logic/control/slalom.c $ROOT/Core/Src/logic/control/velocity_profile.c"
CFLAGS="-std=c11 -Wall -Wextra -O2 -I$ROOT/Core/Inc"

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) LIB=build/maze_sim_lib.dll; LIBFLAGS="-shared -static-libgcc" ;;
    Darwin*)              LIB=build/libmaze_sim.dylib; LIBFLAGS="-shared -fPIC" ;;
    *)                    LIB=build/libmaze_sim.so;    LIBFLAGS="-shared -fPIC" ;;
esac

gcc $CFLAGS -o build/maze_sim maze_sim.c sim_core.c search_time.c plant_bridge.c $LOGIC -lm
echo "built tools/maze_sim/build/maze_sim"

gcc $CFLAGS $LIBFLAGS -o "$LIB" sim_api.c sim_core.c search_time.c plant_bridge.c $LOGIC -lm
echo "built tools/maze_sim/$LIB"
