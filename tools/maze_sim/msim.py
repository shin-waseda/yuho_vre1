"""yuho の機体(mouse.json)で mouse maze sim(公開版の迷路シミュレータ)を動かす。

    python tools/maze_sim/msim.py gui   [迷路ファイル | --random N] [--opt 名前=値 ...]
    python tools/maze_sim/msim.py run   [迷路ファイル | --random N] [--opt 名前=値 ...]
    python tools/maze_sim/msim.py batch [迷路ファイル ... | --random N | --list KEY] [--opt 名前=値 ...]
    (list / fetch / image / image-batch / verify もそのまま渡す)

--mouse を付けなければ tools/maze_sim/mouse.json(yuho)を使う。

mouse maze sim の場所: pip install -e で入れてあればそれ。なければ、環境変数 MAZESIM_PATH か、
tools/maze_sim/mazesim.local の1行目(git に入れない。例 M:/path/to/maze-sim)のフォルダ。
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MOUSE = os.path.join(HERE, "mouse.json")
WITH_MOUSE = ("gui", "run", "batch")


def import_mazesim():
    try:
        import mazesim.cli  # noqa: F401
        return
    except ImportError:
        pass
    path = os.environ.get("MAZESIM_PATH")
    local = os.path.join(HERE, "mazesim.local")
    if not path and os.path.exists(local):
        with open(local, encoding="utf-8") as f:
            path = f.readline().strip()
    if not path:
        sys.exit("mouse maze sim not found: pip install -e <maze-sim>, or write its folder in "
                 "tools/maze_sim/mazesim.local (or set MAZESIM_PATH)")
    sys.path.insert(0, path)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] in WITH_MOUSE and "--mouse" not in argv:
        argv[1:1] = ["--mouse", MOUSE]
    import_mazesim()
    from mazesim.cli import main as mazesim_main
    return mazesim_main(argv)


if __name__ == "__main__":
    sys.exit(main())
