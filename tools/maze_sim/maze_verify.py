"""変換した迷路(my_mazes/)が公式図と合っているかを確かめる。

公式図の下には「西回り 45歩28折 南回り 43歩32折」のように最短経路の歩数が印刷されている。
読み取った迷路から最短歩数を計算し、印刷された歩数の小さい方と一致すれば、
経路に関わる壁は正しく読めていると考えてよい。

使い方:
    python tools/maze_sim/maze_verify.py                     全部の迷路の最短歩数を表にする
    python tools/maze_sim/maze_verify.py MM2025CM MM2025MM   指定した迷路だけ
    python tools/maze_sim/maze_verify.py --sheet             さらに、図の下の文字(印刷された歩数)を切り出し、
                                                             計算した歩数と並べた画像を build/verify_sheet_NN.png に作る
                                                             (画像を開いて、黄色の帯の数字と図の数字を見比べる)
    python tools/maze_sim/maze_verify.py --same A B          2つの迷路の壁が同じか比べる
                                                             (図の「過去の出題」に書かれた迷路どうしなど)

歩 = スタート区画からゴールのどれかの区画に入るまでに進む区画数(ゴールは迷路ファイルの G)。

注意(2020〜2025 年の公式図 92 枚で確かめた結果):
- 印刷された歩数は「西回り・南回り」など代表的な経路のもので、全体の最短とは限らない。
  計算の方が短い図が 8 枚あったが、どれも読み取りの誤りではなかった(北や東から回る経路が載っていない)。
- 図の作成時に文言をコピーしたままの図もある(中部 2022 の CM と MM は同じ文言)。
- 合わないときは、確認用の画像(build/<名前>_check.png)と、「過去の出題」が同じ迷路との比較(--same)で確かめる。
"""

import argparse
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import maze_catalog as mc      # noqa: E402
import maze_from_image as mfi  # noqa: E402

MAZE_DIR = mc.MY_MAZE_DIR
IMAGE_DIR = os.path.join(HERE, "maze_file")
SHEET_PER_IMAGE = 9

DXY = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 北 東 南 西


def wall_reader(north, east, n):
    """has(x, y, d): 区画(x,y)の向き d(0北 1東 2南 3西)に壁があるか(外周を含む)"""
    def has(x, y, d):
        if d == 0:
            return y == n - 1 or north[y][x]
        if d == 1:
            return x == n - 1 or east[y][x]
        if d == 2:
            return y == 0 or north[y - 1][x]
        return x == 0 or east[y][x - 1]
    return has


def shortest_steps(north, east, goals):
    """スタート(0,0)からゴールのどれかに入るまでの最短歩数(行けなければ None)"""
    n = len(north)
    has = wall_reader(north, east, n)
    dist = {(0, 0): 0}
    q = deque([(0, 0)])
    while q:
        x, y = q.popleft()
        if (x, y) in goals:
            return dist[(x, y)]
        for d, (dx, dy) in enumerate(DXY):
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < n and not has(x, y, d) and (nx, ny) not in dist:
                dist[(nx, ny)] = dist[(x, y)] + 1
                q.append((nx, ny))
    return None


def maze_path(name):
    return name if name.endswith(".txt") else os.path.join(MAZE_DIR, name + ".txt")


def steps_of(name):
    """(大きさ, ゴールの数, 最短歩数) か None"""
    r = mc.read_maze(maze_path(name))
    if r is None:
        return None
    north, east, goals = r
    return len(north), len(goals), (shortest_steps(north, east, goals) if goals else None)


def find_image(name):
    for ext in (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tif", ".tiff", ".webp"):
        p = os.path.join(IMAGE_DIR, name + ext)
        if os.path.exists(p):
            return p
    return None


def make_sheets(rows):
    """図の下の文字(印刷された歩数)を切り出し、計算した歩数の帯と並べた画像を作る"""
    from PIL import Image, ImageDraw, ImageFont

    try:
        font = ImageFont.truetype("C:/Windows/Fonts/meiryo.ttc", 20)
    except OSError:
        font = ImageFont.load_default()
    width = 640
    tiles = []
    for name, info in rows:
        image = find_image(name)
        if image is None:
            continue
        try:
            img = mfi.load_image(image).convert("RGB")
            mask, _ = mfi.wall_mask(image, False)
            _, _, _, y1 = mfi.find_frame(mask)
        except Exception as e:  # 1枚の失敗で全体を止めない
            print(f"skip {name}: {e}", file=sys.stderr)
            continue
        h = img.height
        crop = img.crop((0, int(y1 + 0.015 * h), img.width, min(h, int(y1 + 0.15 * h))))
        crop = crop.resize((width, max(1, int(crop.height * width / crop.width))))
        n, _, steps = info
        bar = Image.new("RGB", (width, 26), (255, 255, 160))
        ImageDraw.Draw(bar).text((4, 0), f"{name}   {n}x{n}   計算した最短 = {steps if steps is not None else '-'}",
                                 fill=(0, 0, 0), font=font)
        tiles.append((bar, crop))

    os.makedirs(mfi.CHECK_DIR, exist_ok=True)
    paths = []
    for k in range(0, len(tiles), SHEET_PER_IMAGE):
        group = tiles[k:k + SHEET_PER_IMAGE]
        sheet = Image.new("RGB", (width, sum(b.height + c.height + 4 for b, c in group)), (255, 255, 255))
        y = 0
        for bar, crop in group:
            sheet.paste(bar, (0, y))
            y += bar.height
            sheet.paste(crop, (0, y))
            y += crop.height + 4
        path = os.path.join(mfi.CHECK_DIR, f"verify_sheet_{k // SHEET_PER_IMAGE:02d}.png")
        sheet.save(path)
        paths.append(path)
    return paths


def compare(a, b):
    ra = mc.read_maze(maze_path(a))
    rb = mc.read_maze(maze_path(b))
    if ra is None or rb is None:
        print("cannot read", a if ra is None else b, file=sys.stderr)
        return 2
    if len(ra[0]) != len(rb[0]):
        print(f"different size: {len(ra[0])} vs {len(rb[0])}")
        return 1
    n = len(ra[0])
    diff = [(x, y, "NE"[k]) for k in range(2) for y in range(n) for x in range(n) if ra[k][y][x] != rb[k][y][x]]
    if diff:
        print(f"{len(diff)} walls differ (cell x, y, side N/E): {diff[:20]}")
    else:
        print("walls are identical" + ("" if ra[2] == rb[2] else f" (goals differ: {sorted(ra[2])} vs {sorted(rb[2])})"))
    return 1 if diff else 0


def main():
    p = argparse.ArgumentParser(description="Check converted mazes against the official diagrams")
    p.add_argument("names", nargs="*", help="maze names in my_mazes (default: all)")
    p.add_argument("--sheet", action="store_true", help="make images to compare with the printed step counts")
    p.add_argument("--same", nargs=2, metavar=("A", "B"), help="compare the walls of two mazes")
    args = p.parse_args()

    if args.same:
        return compare(*args.same)

    names = args.names or sorted(f[:-4] for f in os.listdir(MAZE_DIR) if f.endswith(".txt"))
    rows = []
    for name in names:
        info = steps_of(name)
        if info is None:
            print(f"{name:32} cannot read")
            continue
        n, goals, steps = info
        shown = steps if steps is not None else ("no goal" if goals == 0 else "unreachable")
        print(f"{name:32} {n:2}x{n:<2}  shortest = {shown}")
        rows.append((name, info))

    if args.sheet:
        for path in make_sheets(rows):
            print("sheet:", mfi.show_path(path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
