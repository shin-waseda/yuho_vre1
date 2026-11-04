"""yuho の迷路探索を画面で見る(python tools/maze_sim/msim.py gui と同じ)。

    python tools/maze_sim/gui.py [迷路ファイル | --random N] [--opt 名前=値 ...]

画面は mouse maze sim(公開版の迷路シミュレータ)。yuho の logic 層は mouse.json と yuho_mouse.c でつなぐ。
"""

import sys

import msim

if __name__ == "__main__":
    sys.exit(msim.main(["gui"] + sys.argv[1:]))
