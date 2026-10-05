"""公式の迷路図(画像)を読み取って、classic 形式の迷路ファイルにする。

使い方:
    python tools/maze_sim/maze_from_image.py MM2025_touhoku_CM.png
        → tools/maze_sim/my_mazes/MM2025_touhoku_CM.txt を作る
        → 確認用の画像 tools/maze_sim/build/MM2025_touhoku_CM_check.png も作る
    (たくさんあるときは maze_batch.py でフォルダごと変換する)

オプション:
    --name NAME        保存する名前(拡張子なし。既定は画像のファイル名)
    --out DIR          保存先のフォルダ(既定 tools/maze_sim/my_mazes)
    --size N           迷路の1辺の区画数(既定 auto: 16 か 32 を図から判定)
    --rotate DEG       読み取った迷路を時計回りに回す(90/180/270/auto)。スタートが左下に来るように
    --crop X0,Y0,X1,Y1 迷路の外枠の位置を画素で指定する(自動で見つからないとき)
    --invert           白い壁・黒い背景の図のとき
    --threshold T      壁とみなす途切れのなさ(0〜1。既定は図ごとに自動で決める)
    --force            同じ名前のファイルがあっても上書きする

読み取り方(akiaki96/maze_sim_py_c_v2 の parse_maze_image.py と同じ考え方を、OpenCV なしで):
    1. 灰色にして、大津の方法で「壁(濃い色)」と「床」に分ける
    2. いちばん長く途切れない縦線を外枠の左右、その間を端から端まで続く横線を上下とする
       (タイトルや下線を外枠と間違えないため)
    3. 格子線(柱・点線)の並び方から 16×16 か 32×32 かを判定する
    4. 外枠を N 等分した位置の近くで、黒い画素が最も多い列(行)を格子線とする
    5. 区画の境目ごとに、両端の柱を除いた真ん中の部分が、長さ方向にどれだけ途切れずに
       黒いかを調べる。壁はほぼ 1.0、点線の補助線は途切れるので低い。その分布のすき間を閾値にする
    6. 区画の中に文字(G)がある区画をゴールとする(スタートの ↑ は除く)

読み取りが自動で正しいとは限らないので、確認用の画像と、自信のない壁の一覧を必ず見ること。
"""

import argparse
import os
import sys
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "my_mazes")
CHECK_DIR = os.path.join(HERE, "build")

UNSURE_MARGIN = 0.05  # 閾値からこれ以内の判定は「自信なし」として一覧に出す
MIN_GAP = 0.08        # 壁と点線の判定値のすき間がこれより狭ければ、図ごと要確認
MARK_DENSITY = 0.015  # 区画の中の濃い画素がこの割合を超えたら、文字(G など)があるとみなす


def show_path(path):
    """表示用のパス(今のフォルダからの相対パス。別のドライブならそのまま)"""
    try:
        return os.path.relpath(path)
    except ValueError:
        return path


# ------------------------------------------------------------
# 画像 → 壁の画素
# ------------------------------------------------------------

def otsu_threshold(gray):
    """大津の方法で 0〜255 の閾値を求める"""
    hist = np.bincount(gray.ravel(), minlength=256).astype(np.float64)
    total = gray.size
    levels = np.arange(256)
    w0 = np.cumsum(hist)
    w1 = total - w0
    m0 = np.cumsum(hist * levels)
    mean_all = m0[-1]
    with np.errstate(divide="ignore", invalid="ignore"):
        between = (mean_all * w0 / total - m0) ** 2 / (w0 * w1 / total)
    between[~np.isfinite(between)] = 0
    return int(np.argmax(between))


def load_image(path):
    """画像を開く。PDF なら1ページ目を画像にする(PyMuPDF が必要)"""
    if path.lower().endswith(".pdf"):
        try:
            import fitz  # PyMuPDF
        except ImportError:
            raise RuntimeError("reading PDF needs PyMuPDF: pip install pymupdf")
        with fitz.open(path) as doc:
            pix = doc[0].get_pixmap(dpi=200)
            return Image.frombytes("RGB", (pix.width, pix.height), pix.samples)
    img = Image.open(path)
    # 透明度付きの画像は、透明な所が黒として読まれないよう白い背景に重ねる
    if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
        rgba = img.convert("RGBA")
        white = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
        img = Image.alpha_composite(white, rgba).convert("RGB")
    return img


def wall_mask(path, invert):
    """壁の画素を True にした2次元配列(y は上が 0)"""
    img = load_image(path).convert("L")
    gray = np.asarray(img, dtype=np.uint8)
    t = otsu_threshold(gray)
    mask = gray <= t          # 濃い色が壁
    if invert:
        mask = ~mask
    return mask, img


def longest_runs(mask):
    """各列(mask の2次元目)で、壁の画素が途切れずに続く最長の長さ"""
    m = mask.astype(np.int8)
    padded = np.vstack([np.zeros((1, m.shape[1]), np.int8), m, np.zeros((1, m.shape[1]), np.int8)])
    runs = np.zeros(m.shape[1], dtype=np.int64)
    d = np.diff(padded, axis=0)
    for c in range(m.shape[1]):
        starts = np.flatnonzero(d[:, c] == 1)
        if starts.size:
            ends = np.flatnonzero(d[:, c] == -1)
            runs[c] = int((ends - starts).max())
    return runs


def find_frame(mask):
    """外枠の (x0, y0, x1, y1)。
    外枠の縦線は図の中でいちばん長く途切れない縦線なので、まずそれで左右を決め、
    次に左右の間で(ほぼ)端から端まで続く横線で上下を決める。
    タイトルの文字や「ロボット名＿＿」の下線を外枠と間違えないため。"""
    col_runs = longest_runs(mask)
    if col_runs.max() < 20:
        raise RuntimeError("maze frame not found (try --crop)")
    cols = np.flatnonzero(col_runs >= col_runs.max() * 0.8)
    x0, x1 = int(cols[0]), int(cols[-1])
    if x1 - x0 < 20:
        raise RuntimeError("maze frame not found (try --crop)")

    row_runs = longest_runs(mask[:, x0:x1 + 1].T)
    rows = np.flatnonzero(row_runs >= (x1 - x0) * 0.8)
    if len(rows) < 2:
        raise RuntimeError("maze frame not found (try --crop)")
    y0, y1 = int(rows[0]), int(rows[-1])
    return x0, y0, x1, y1


def detect_size(mask, frame, candidates=(16, 32)):
    """迷路の1辺の区画数を、格子線(柱・点線)の並び方から推定する。
    32×32 なら 32 等分したすべての位置に線があるが、16×16 なら奇数番目の位置は区画の真ん中で線がない。"""
    x0, y0, x1, y1 = frame
    profile = mask[y0:y1 + 1, :].sum(axis=0).astype(np.float64)
    small, big = sorted(candidates)
    ratio = big // small
    seg = (x1 - x0) / big
    w = max(1, int(seg * 0.1))

    def peak(i):
        c = int(round(x0 + seg * i))
        return profile[max(0, c - w):c + w + 1].max()

    on = np.mean([peak(i) for i in range(0, big + 1, ratio)])            # どちらの大きさでも線がある位置
    mid = np.mean([peak(i) for i in range(1, big) if i % ratio != 0])   # 大きい方だけ線がある位置
    return big if mid > on * 0.35 else small


def line_positions(mask, start, end, n, axis, other_start, other_end):
    """外枠の間を n 等分した位置の近くで、黒い画素が最も多い列(axis=0)/行(axis=1)を探す"""
    if axis == 0:
        profile = mask[other_start:other_end + 1, :].sum(axis=0)
    else:
        profile = mask[:, other_start:other_end + 1].sum(axis=1)
    seg = (end - start) / n
    lines = []
    for i in range(0, n + 1):
        guess = start + seg * i
        # 外枠は太いことがあるので、内側だけを探して線の真ん中に合わせる
        lo = int(max(start, guess - seg * 0.25))
        hi = int(min(end, guess + seg * 0.25))
        window = profile[lo:hi + 1]
        if window.size == 0 or window.max() == 0:
            lines.append(int(round(guess)))
        else:
            # 一番濃い所が何画素か続くときは、その真ん中
            best = np.where(window == window.max())[0]
            lines.append(lo + int(best[len(best) // 2]))
    return lines


def detect_marks(mask, xs, ys, n):
    """区画の中に文字(G や ↑ など)が書いてある区画の集合。壁に近い所は見ない"""
    marks = set()
    for y in range(n):
        top, bottom = ys[n - 1 - y], ys[n - y]
        for x in range(n):
            left, right = xs[x], xs[x + 1]
            mx = int((right - left) * 0.25)
            my = int((bottom - top) * 0.25)
            inner = mask[top + my:bottom - my, left + mx:right - mx]
            if inner.size and inner.mean() > MARK_DENSITY:
                marks.add((x, y))
    return marks


def edge_coverage(mask, a0, a1, b, band, vertical):
    """壁の候補の線分の真ん中部分が、長さ方向にどれだけ途切れずに黒いか(0〜1)。
    vertical: 縦の壁なら True(a は y、b は x)"""
    length = a1 - a0
    lo = int(a0 + length * 0.25)
    hi = int(a1 - length * 0.25)
    if hi <= lo:
        return 0.0
    b0 = max(0, b - band)
    b1 = b + band + 1
    if vertical:
        strip = mask[lo:hi, b0:b1]
        covered = strip.any(axis=1)
    else:
        strip = mask[b0:b1, lo:hi]
        covered = strip.any(axis=0)
    return float(covered.mean()) if covered.size else 0.0


# ------------------------------------------------------------
# 壁の配列(区画座標: x が東、y が北。y=0 が下)
# ------------------------------------------------------------

def auto_threshold(values):
    """途切れのなさの分布から閾値を決める。(閾値, すき間の幅) を返す。
    壁はほぼ 1.0 に集まり、点線の補助線は図によって(縦と横でも)0.2〜0.9 にばらつく。
    そこで上(1.0 側)から見て、最初に現れる MIN_GAP 以上のすき間の真ん中を閾値にする。
    (いちばん広いすき間を選ぶと、縦と横の点線の間を選んでしまうことがある)"""
    v = sorted(s for s in values if 0.2 <= s <= 1.0)
    if len(v) < 2:
        return 0.7, 0.0
    for i in range(len(v) - 2, -1, -1):
        if v[i + 1] - v[i] >= MIN_GAP:
            return (v[i] + v[i + 1]) / 2, v[i + 1] - v[i]
    # 十分なすき間がなければ、いちばん広い所(要確認になる)
    gap, i = max((v[k + 1] - v[k], k) for k in range(len(v) - 1))
    return (v[i] + v[i + 1]) / 2, gap


def extract_walls(mask, frame, n, threshold=None):
    """壁を読み取る。threshold が None なら図ごとに auto_threshold で決める。
    (north, east, scores, xs, ys, 使った閾値, すき間の幅) を返す。"""
    x0, y0, x1, y1 = frame
    xs = line_positions(mask, x0, x1, n, 0, y0, y1)
    ys = line_positions(mask, y0, y1, n, 1, x0, x1)   # 画像の上から
    seg = min((x1 - x0) / n, (y1 - y0) / n)
    band = max(2, int(seg * 0.08))

    # 外周の南・西は迷路ファイルでは常に壁なので配列には持たないが、外枠の検出が正しいかを
    # 確かめるために読み取って scores には入れる。
    scores = []  # (score, 区画x, 区画y, 向き 'north'/'east'/'south'/'west')
    for y in range(n):
        top = ys[n - 1 - y]       # この区画の北の線(画像の上から)
        bottom = ys[n - y]
        for x in range(n):
            left, right = xs[x], xs[x + 1]
            scores.append((edge_coverage(mask, left, right, top, band, vertical=False), x, y, "north"))
            scores.append((edge_coverage(mask, top, bottom, right, band, vertical=True), x, y, "east"))
            if y == 0:
                scores.append((edge_coverage(mask, left, right, bottom, band, vertical=False), x, y, "south"))
            if x == 0:
                scores.append((edge_coverage(mask, top, bottom, left, band, vertical=True), x, y, "west"))

    gap = None
    if threshold is None:
        threshold, gap = auto_threshold([s for s, *_ in scores])

    # north[y][x]: 区画(x,y)の北の壁 / east[y][x]: 東の壁
    north = [[False] * n for _ in range(n)]
    east = [[False] * n for _ in range(n)]
    for s, x, y, d in scores:
        if d == "north":
            north[y][x] = s >= threshold
        elif d == "east":
            east[y][x] = s >= threshold
    return north, east, scores, xs, ys, threshold, gap


def rotate_cw(north, east, n):
    """迷路を時計回りに90°回す(区画(x,y) → (y, n-1-x))"""
    def has(x, y, d):  # d: 0北 1東 2南 3西
        if d == 0:
            return north[y][x]
        if d == 1:
            return east[y][x]
        if d == 2:
            return True if y == 0 else north[y - 1][x]
        return True if x == 0 else east[y][x - 1]

    n2 = [[False] * n for _ in range(n)]
    e2 = [[False] * n for _ in range(n)]
    for y in range(n):
        for x in range(n):
            nx, ny = y, n - 1 - x
            # 時計回りに回すと、もとの西が北、北が東になる
            n2[ny][nx] = has(x, y, 3)
            e2[ny][nx] = has(x, y, 0)
    return n2, e2


def to_text(north, east, n, goals):
    """classic 形式(北が上)。スタート(0,0)に S、ゴールに G を書く"""
    lines = ["o" + "---o" * n]
    for y in range(n - 1, -1, -1):
        cell = "|"
        for x in range(n):
            mark = "S" if (x, y) == (0, 0) else ("G" if (x, y) in goals else " ")
            cell += f" {mark} " + ("|" if east[y][x] else " ")
        lines.append(cell)
        if y > 0:
            row = "o"
            for x in range(n):
                row += ("---" if north[y - 1][x] else "   ") + "o"
            lines.append(row)
    lines.append("o" + "---o" * n)
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------
# 確認
# ------------------------------------------------------------

def outer_missing(scores, n, threshold):
    """外周(画像の外枠)のうち、壁と読めなかった数"""
    count = 0
    for score, x, y, d in scores:
        outer = (d == "north" and y == n - 1) or (d == "east" and x == n - 1) or d in ("south", "west")
        if outer and score < threshold:
            count += 1
    return count


def wall_reader(north, east):
    """has(x, y, d): 区画(x,y)の向き d(0北 1東 2南 3西)に壁があるか。南端・西端は外周"""
    def has(x, y, d):
        if d == 0:
            return north[y][x]
        if d == 1:
            return east[y][x]
        if d == 2:
            return True if y == 0 else north[y - 1][x]
        return True if x == 0 else east[y][x - 1]
    return has


def reachable(north, east, n):
    """スタート(0,0)から行ける区画の集合"""
    has = wall_reader(north, east)
    seen = {(0, 0)}
    q = deque([(0, 0)])
    dxy = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    while q:
        x, y = q.popleft()
        for d, (dx, dy) in enumerate(dxy):
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < n and not has(x, y, d) and (nx, ny) not in seen:
                seen.add((nx, ny))
                q.append((nx, ny))
    return seen


def check_maze(north, east, n, goals, missing_outer):
    """(警告, 参考情報) の2つの文字列リストを返す。警告があれば要確認"""
    warnings, infos = [], []
    has = wall_reader(north, east)

    if missing_outer:
        warnings.append(f"外周の壁が {missing_outer} か所欠けている(読み取りの誤りか、外枠の位置がずれている。"
                        "確認用の画像を見て、必要なら --crop)")
    if not has(0, 0, 1):
        warnings.append("スタート区画(0,0)の東に壁がない(規定では壁。図の向きが違うなら --rotate)")
    if has(0, 0, 0):
        warnings.append("スタート区画(0,0)の北に壁がある(出られない。図の向きが違うなら --rotate)")

    seen = reachable(north, east, n)
    if goals and not any(g in seen for g in goals):
        warnings.append("スタートから中央のゴールへ行けない(読み取りの誤りか、ゴールが中央でない迷路)")
    if len(seen) < n * n:
        infos.append(f"スタートから行けない区画が {n * n - len(seen)} 個(閉じた区画。多すぎるなら向きか読み取りの誤り)")
    return warnings, infos


def edge_segment(xs, ys, n, x, y, d):
    """区画(x,y)の向き d の壁の、画像上の線分の両端"""
    top, bottom = ys[n - 1 - y], ys[n - y]
    left, right = xs[x], xs[x + 1]
    if d == "north":
        return (left, top), (right, top)
    if d == "south":
        return (left, bottom), (right, bottom)
    if d == "east":
        return (right, top), (right, bottom)
    return (left, top), (left, bottom)


def save_check_image(img, frame, xs, ys, scores, n, threshold, path, rotated):
    """元の図に、読み取った壁(緑)と自信のない壁(橙)を重ねた画像"""
    x0, y0, x1, y1 = frame
    pad = 10
    ox, oy = max(0, x0 - pad), max(0, y0 - pad)
    crop = img.convert("RGB").crop((ox, oy, x1 + pad, y1 + pad))
    draw = ImageDraw.Draw(crop)
    width = max(2, int((x1 - x0) / n * 0.08))
    for score, x, y, d in scores:
        a, b = edge_segment(xs, ys, n, x, y, d)
        a = (a[0] - ox, a[1] - oy)
        b = (b[0] - ox, b[1] - oy)
        if abs(score - threshold) < UNSURE_MARGIN:
            draw.line([a, b], fill=(255, 140, 0), width=width + 2)
        elif score >= threshold:
            draw.line([a, b], fill=(0, 200, 0), width=width)
    if rotated:
        draw.text((4, 4), "(before --rotate)", fill=(255, 0, 0))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    crop.save(path)


def start_ok(north, east):
    """スタート区画(0,0)が規定どおり(東に壁があり、北が開いている)か"""
    return east[0][0] and not north[0][0]


def choose_rotation(north, east, n, goals):
    """図の向き(時計回りに回す角度)を決める。(角度, 決まったか, 候補の角度のリスト) を返す。
    公式図はふつうスタートが左下なので、そのままでスタート区画の規定を満たせば回さない。
    満たさなければ他の向きを探し、複数あればスタートから中央のゴールへ行ける向きで絞る。
    注意: 回った図でも、左下の角がたまたま規定を満たすと見分けられない(回さずに OK になる)。"""
    if start_ok(north, east):
        return 0, True, [0]
    candidates = []
    nn, ee = north, east
    for deg in (90, 180, 270):
        nn, ee = rotate_cw(nn, ee, n)
        if start_ok(nn, ee):
            goal_ok = any(g in reachable(nn, ee, n) for g in goals)
            candidates.append((deg, goal_ok))
    with_goal = [deg for deg, ok in candidates if ok]
    if len(with_goal) == 1:
        return with_goal[0], True, with_goal
    if len(candidates) == 1:
        return candidates[0][0], True, [candidates[0][0]]
    return 0, False, (with_goal or [deg for deg, _ in candidates])


def convert(image, name=None, out_dir=DEFAULT_OUT, size="auto", rotate=0, crop=None, invert=False,
            threshold=None, force=False):
    """画像を読み取って迷路ファイルを書き、結果を dict で返す。
    size: 区画数か "auto"(16 か 32 を図から判定)。
    rotate: 0/90/180/270 か "auto"(スタート区画の規定から向きを決める)。
    threshold: 壁とみなす途切れのなさ。None なら図ごとに決める。
    すでにファイルがあって force=False なら、何もせず skipped=True を返す。
    画像が開けない・外枠が見つからないときは例外を投げる。"""
    name = name or os.path.splitext(os.path.basename(image))[0]
    out_path = os.path.join(out_dir, name + ".txt")
    check_path = os.path.join(CHECK_DIR, name + "_check.png")
    result = {"name": name, "out_path": out_path, "check_path": check_path, "skipped": False}
    if os.path.exists(out_path) and not force:
        result["skipped"] = True
        return result

    mask, img = wall_mask(image, invert)
    frame = tuple(int(v) for v in crop.split(",")) if crop else find_frame(mask)
    n = detect_size(mask, frame) if size == "auto" else int(size)
    north, east, scores, xs, ys, threshold, gap = extract_walls(mask, frame, n, threshold)
    missing = outer_missing(scores, n, threshold)
    if missing > 2 * n:
        # 外周の半分以上が読めないなら、迷路の図ではないか、外枠をまったく見つけられていない
        raise RuntimeError(f"maze not found ({missing} of {4 * n} outer walls missing; try --crop)")

    warnings, infos = [], []
    if gap is not None and gap < MIN_GAP:
        warnings.append(f"壁と点線の区別がはっきりしない(すき間 {gap:.2f})。確認用の画像で全体を確かめる")

    # 文字のある区画をゴールとする(回す前の向きで読むので、回すときは一緒に回す)
    marks = detect_marks(mask, xs, ys, n)
    marks.discard((0, 0))  # スタートの ↑
    goals = set(marks)
    if goals:
        gx = [x for x, _ in goals]
        gy = [y for _, y in goals]
        box = {(x, y) for x in range(min(gx), max(gx) + 1) for y in range(min(gy), max(gy) + 1)}
        corners = {(min(gx), min(gy)), (min(gx), max(gy)), (max(gx), min(gy)), (max(gx), max(gy))}
        if goals == corners and len(box) > 4:
            # 3×3 のゴールの四隅にだけ G を書く図がある(例: 2023 全日本ハーフ)
            goals = box
            infos.append(f"四隅の G から {max(gx) - min(gx) + 1}×{max(gy) - min(gy) + 1} のゴールとした")
        elif len(box) == 4 and len(goals) == 3:
            # "GOAL" の4文字のうち細い文字(L など)を読み落とすことがある
            goals = box
            infos.append("2×2 のうち3区画に印があったので、2×2 のゴールとした")
        elif goals != box:
            warnings.append(f"ゴールの印(G)が長方形に並んでいない({sorted(marks)})。確認用の画像で確かめる")

    if rotate == "auto":
        c = n // 2
        center = goals or ({(c - 1, c - 1), (c, c - 1), (c - 1, c), (c, c)} if n % 2 == 0 else {(c, c)})
        rotate, decided, candidates = choose_rotation(north, east, n, center)
        if not decided:
            hint = f"候補: {', '.join(str(c) for c in candidates)}°" if candidates else "規定に合う向きがない"
            warnings.append(f"図の向きを自動で決められなかった({hint})。確認用の画像を見て --rotate で指定する")
        elif rotate:
            infos.append(f"向きを自動で決めて時計回りに {rotate}° 回した")

    # 確認用の画像と自信のない壁の一覧は、回す前の向き(元の図と同じ向き)で出す
    save_check_image(img, frame, xs, ys, scores, n, threshold, check_path, rotate != 0)

    for _ in range(rotate // 90):
        north, east = rotate_cw(north, east, n)
        goals = {(y, n - 1 - x) for (x, y) in goals}

    if not goals:
        infos.append("ゴールの印(G)が見つからない(サーキット競技などゴールのない迷路なら問題ない)")
    elif len(goals) not in (1, 4, 9):
        warnings.append(f"ゴールの印のある区画が {len(goals)} 個(ふつうは 1・4・9 個)。確認用の画像で確かめる")

    text = to_text(north, east, n, goals)
    os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)

    result.update({
        "text": text,
        "frame": frame,
        "size": n,
        "threshold": threshold,
        "rotate": rotate,
        "goals": sorted(goals),
        "walls": sum(map(sum, north)) + sum(map(sum, east)),
        "unsure": sorted((s, x, y, d) for s, x, y, d in scores if abs(s - threshold) < UNSURE_MARGIN),
    })
    more_warnings, more_infos = check_maze(north, east, n, goals, missing)
    result["notes"] = warnings + more_warnings   # 警告(あれば要確認)
    result["infos"] = infos + more_infos         # 参考情報
    return result


def main():
    p = argparse.ArgumentParser(description="Convert an official maze diagram image to a classic maze file")
    p.add_argument("image")
    p.add_argument("--name")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--size", default="auto")
    p.add_argument("--rotate", choices=["0", "90", "180", "270", "auto"], default="0")
    p.add_argument("--crop")
    p.add_argument("--invert", action="store_true")
    p.add_argument("--threshold", type=float, default=None)
    p.add_argument("--force", action="store_true")
    args = p.parse_args()

    rotate = args.rotate if args.rotate == "auto" else int(args.rotate)
    try:
        r = convert(args.image, args.name, args.out, args.size, rotate, args.crop, args.invert,
                    args.threshold, args.force)
    except (OSError, RuntimeError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if r["skipped"]:
        print(f"{show_path(r['out_path'])} already exists (use --force to overwrite)", file=sys.stderr)
        return 2

    print(r["text"], end="")
    print(f"size: {r['size']}x{r['size']}, frame (px): {r['frame']}, walls: {r['walls']}, "
          f"threshold: {r['threshold']:.2f}, rotate: {r['rotate']}, goals: {r['goals']}")
    print(f"saved: {show_path(r['out_path'])}")
    print(f"check image: {show_path(r['check_path'])}  (green = wall, orange = unsure)")
    if r["unsure"]:
        print(f"unsure edges ({len(r['unsure'])}, cell (x,y) in the image before rotating, "
              "score = continuity 0..1):")
        for s, x, y, d in r["unsure"][:40]:
            print(f"  ({x},{y}) {d:<6} {s:.2f} -> {'wall' if s >= r['threshold'] else 'open'}")
    for note in r["notes"]:
        print("CHECK:", note)
    for info in r["infos"]:
        print("info :", info)
    return 0


if __name__ == "__main__":
    sys.exit(main())
