"""迷路ファイルの一覧を作る(GUI の迷路の一覧と CLI の --paths で使う)。

迷路は2か所から読む。
    my_mazes/  自分で追加した迷路(公式の迷路図から maze_from_image.py / maze_batch.py で作る)。主役
    mazes/     fetch_mazes.sh で取ってきた micromouseonline/mazefiles の迷路。「インポート」にまとめる

自分の迷路は、公式図のファイル名の付け方 MM<年>[_<地区>]_<種目> で分類する(tools/maze_sim/maze_file/ の命名)。
    MM2025CM               → 全日本   2025 クラシック
    MM2025MM               → 全日本   2025 マイクロマウス(32×32)
    MM2025_touhoku_CM      → 地区大会 2025 東北 クラシック
    MM2025_students_MM     → 学生大会 2025 マイクロマウス
    MM2023_hokushinetsu_MM_CM → 地区大会 2023 北陸信越 マイクロマウス／クラシック(共通の迷路)
命名に合わないものは「自分で追加: その他」に入る。インポートした迷路は、ファイル名から大会を推定して分ける。

お気に入りと最近開いた迷路は maze_prefs.json に覚える(git には入れない)。

CLI:
    python tools/maze_sim/maze_catalog.py                          一覧
    python tools/maze_sim/maze_catalog.py --paths regional --type CM       自分の地区大会のクラシック
    python tools/maze_sim/maze_catalog.py --paths alljapan --year 2025     自分の全日本の2025年
    python tools/maze_sim/maze_catalog.py --paths imported/alljapan --round final   インポートの全日本の決勝
    tools/maze_sim/build/maze_sim $(python tools/maze_sim/maze_catalog.py --paths regional --type CM)
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MAZE_DIR = os.path.join(HERE, "mazes")         # fetch_mazes.sh で取ってきた迷路(git に入れない)
MY_MAZE_DIR = os.path.join(HERE, "my_mazes")   # 自分で追加した迷路(git に入れる)
PREFS_FILE = os.path.join(HERE, "maze_prefs.json")
RECENT_MAX = 10
SIM_SIZE = 16  # シミュレータが扱える大きさ(params.h の MAZE_SIZE)

# ------------------------------------------------------------
# 自分の迷路(公式図の命名)
# ------------------------------------------------------------

OWN_CATEGORIES = [
    ("alljapan", "全日本"),
    ("student", "学生大会"),
    ("regional", "地区大会"),
    ("mine_other", "自分で追加: その他"),
]

OWN_REGIONS = {
    "chubu": "中部", "chubuf": "中部初級者", "east": "東日本", "hokushinetsu": "北陸信越",
    "kanazawa": "金沢草の根", "kansai": "関西", "kyusyu": "九州", "kyushu": "九州", "touhoku": "東北",
}
OWN_STUDENT = ("student", "students")

OWN_TYPES = {
    "CM": "クラシック",
    "MM": "マイクロマウス",
    "MS": "マイクロマウス セミファイナル",
    "CF": "クラシック フレッシュマン",
    "SC": "支部サーキット",
}
# 同じ年・同じ大会の中での並び順
TYPE_ORDER = ["CM", "CF", "MM", "MS", "SC"]


def parse_own_name(name):
    """MM<年>[_<地区>]_<種目> を読む。(年, 地区のトークン or None, [種目...], その他の文字) か None"""
    tokens = name.split("_")
    m = re.match(r"^MM(\d{4})([A-Z]{2})?$", tokens[0])
    if not m:
        return None
    year = int(m.group(1))
    types = [m.group(2)] if m.group(2) else []
    region = None
    extra = []
    for t in tokens[1:]:
        if re.fullmatch(r"[A-Z]{2}", t):
            types.append(t)
        elif re.fullmatch(r"[a-z]+", t) and region is None:
            region = t
        else:
            extra.append(t)
    if not types:
        return None
    return year, region, types, " ".join(extra)


# ------------------------------------------------------------
# インポートした迷路(mazefiles の命名はばらばらなので推定)
# ------------------------------------------------------------

# (キー, 表示名, ファイル名(小文字)の正規表現)。上から順に調べ、最初に合ったものに入れる。
IMPORTED_CATEGORIES = [
    ("alljapan", "全日本", r"^alljapan-"),
    ("student", "学生大会", r"^student-|student|^japan1987stu"),
    ("alljapan_old", "全日本(別名・重複あり)", r"^japan"),
    ("regional", "地区大会", r"^(chubu|eastjapan|hokkaido|hokuriku|kyushu|northeast|kansai|kantou|higashi|west-|"
                            r"nagoya|niigata|tyubu|other-hokurobo)"),
    ("apec", "APEC", r"^apec"),
    ("uk", "UK", r"^uk|^other-uk"),
    ("minos", "MINOS(UK)", r"^minos"),
    ("usa", "米国", r"^(us\d|us-|usa|allamerica|aamc|camm)"),
    ("taiwan", "台湾", r"^taiwan"),
    ("korea", "韓国", r"^(kor|seoul)"),
    ("portugal", "ポルトガル", r"^portugal"),
    ("brazil", "ブラジル", r"^br20"),
    ("spain", "スペイン(OSHWDEM)", r"^oshwdem"),
    ("asia", "シンガポール・香港", r"^(sg\d|sing|hk\d)"),
    ("world", "ヨーロッパ・世界大会", r"^(euro|other-world|expo)"),
    ("test", "テスト・練習用", r"^(test-|001|empty|diag|zigzag|loop|long|torture|tricky|solver|mtest|bug|ph-|wiggly|"
                              r"cut$|dame$)"),
    ("other", "その他", r""),
]

IMPORTED_REGIONS = {
    "chubu": "中部", "tyubu": "中部", "eastjapan": "東日本", "higashi": "東日本", "hokkaido": "北海道",
    "hokuriku": "北陸", "kyushu": "九州", "northeast": "東北", "kansai": "関西", "kantou": "関東",
    "west": "西日本", "nagoya": "名古屋", "niigata": "新潟", "other-hokurobo": "北海道ロボ",
}

# 2桁の年が付く接頭辞(例: uk89f → 1989)
TWO_DIGIT_YEAR = re.compile(r"^(uk|us|kor|sg|hk|ies|tor|sec|chi|mont|kyot-|yama)(\d{2})(?!\d)")

IMPORTED_KEY = "imported"
IMPORTED_LABEL = "インポート(mazefiles)"
FAVORITES_LABEL = "★ お気に入り"
RECENT_LABEL = "最近開いた迷路"


def classify_imported(name):
    low = name.lower()
    for key, _, pattern in IMPORTED_CATEGORIES:
        if re.search(pattern, low):
            return key
    return "other"


def parse_year(name):
    low = name.lower()
    m = re.search(r"(19[7-9]\d|20[0-4]\d)", low)
    if m:
        return int(m.group(1))
    m = TWO_DIGIT_YEAR.match(low)
    if m:
        yy = int(m.group(2))
        return 1900 + yy if yy >= 70 else 2000 + yy
    return None


def imported_detail(name, category):
    """クラス(エキスパート等)と回(予選・決勝)をファイル名から読み取る"""
    low = name.lower()
    words = []
    if category == "alljapan":
        m = re.match(r"alljapan-(\d+)-\d{4}-?(.*)$", low)
        if m:
            words.append(f"第{int(m.group(1))}回")
            low = m.group(2)
    if category == "regional":
        for key, jp in IMPORTED_REGIONS.items():
            if low.startswith(key):
                words.append(jp)
                break
    m = re.search(r"(?:19|20)\d{2}(.*)$", low)
    suffix = m.group(1).strip("-_") if m else ""
    if "frsh" in low:
        words.append("フレッシュマン")
    elif "novice" in low:
        words.append("ノービス")
    elif "exp" in low or suffix.startswith("e") or "cef" in suffix:
        words.append("エキスパート")
    elif "stu" in low:
        words.append("学生")
    if re.search(r"fin|final", low) or suffix in ("f", "ef", "cef"):
        words.append("決勝")
    elif re.search(r"pre|qual", low) or suffix in ("q", "p", "eq"):
        words.append("予選")
    elif "follower" in low:
        words.append("ライントレース")
    return " ".join(words)


# ------------------------------------------------------------
# 迷路1つ
# ------------------------------------------------------------

def maze_size(path):
    """1行目(北の外周)の幅から1辺の区画数を求める。読めなければ None"""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            first = f.readline().rstrip("\r\n")
    except OSError:
        return None
    return (len(first) - 1) // 4 if len(first) >= 5 else None


class MazeEntry:
    def __init__(self, name, folder):
        self.name = name                       # 拡張子なしのファイル名
        self.path = os.path.join(folder, name + ".txt")
        self.mine = folder == MY_MAZE_DIR
        self.size = maze_size(self.path)
        self.types = []
        self.region = None
        self.extra = ""
        if self.mine:
            parsed = parse_own_name(name)
            if parsed:
                self.year, self.region, self.types, self.extra = parsed
                if self.region is None:
                    self.category = "alljapan"
                elif self.region in OWN_STUDENT:
                    self.category = "student"
                else:
                    self.category = "regional"
            else:
                self.year = parse_year(name)
                self.category = "mine_other"
        else:
            self.category = classify_imported(name)
            self.year = parse_year(name)

    @property
    def id(self):
        """お気に入り・最近開いた迷路で覚える名前(自分の迷路は my: を付けて区別する)"""
        return ("my:" + self.name) if self.mine else self.name

    @property
    def node_key(self):
        """一覧の中でこの迷路が入る場所のキー"""
        return self.category if self.mine else f"{IMPORTED_KEY}/{self.category}"

    @property
    def playable(self):
        return self.size == SIM_SIZE

    @property
    def detail(self):
        if not self.mine:
            return imported_detail(self.name, self.category)
        words = []
        if self.region and self.region not in OWN_STUDENT:
            words.append(OWN_REGIONS.get(self.region, self.region))
        if self.types:
            words.append("／".join(OWN_TYPES.get(t, t) for t in self.types))
        if self.extra:
            words.append(self.extra)
        return " ".join(words)

    @property
    def label(self):
        """一覧に出す説明(例: '2025 東北 クラシック')"""
        parts = [str(self.year)] if self.year else []
        if self.detail:
            parts.append(self.detail)
        if self.size and self.size != SIM_SIZE:
            parts.append(f"[{self.size}×{self.size}]")
        return " ".join(parts) if parts else self.name

    def sort_key(self):
        # 新しい年から。同じ年なら 地区 → 種目 の順
        type_rank = min((TYPE_ORDER.index(t) for t in self.types if t in TYPE_ORDER), default=len(TYPE_ORDER))
        return (-(self.year or 0), self.region or "", type_rank, self.name)


def _names_in(folder):
    if not os.path.isdir(folder):
        return []
    return sorted(f[:-4] for f in os.listdir(folder) if f.lower().endswith(".txt"))


def load_entries():
    """my_mazes/ と mazes/ の全迷路"""
    return ([MazeEntry(n, MY_MAZE_DIR) for n in _names_in(MY_MAZE_DIR)]
            + [MazeEntry(n, MAZE_DIR) for n in _names_in(MAZE_DIR)])


# ------------------------------------------------------------
# 一覧の木
#   ノード: {"key", "label", "items": [MazeEntry]} か {"key", "label", "children": [ノード]}
# ------------------------------------------------------------

def build_tree(entries, prefs):
    by_id = {e.id: e for e in entries}
    tree = []
    fav = [by_id[i] for i in prefs["favorites"] if i in by_id]
    if fav:
        tree.append({"key": "favorites", "label": FAVORITES_LABEL, "items": fav})
    recent = [by_id[i] for i in prefs["recent"] if i in by_id]
    if recent:
        tree.append({"key": "recent", "label": RECENT_LABEL, "items": recent})

    for key, label in OWN_CATEGORIES:
        items = sorted((e for e in entries if e.mine and e.category == key), key=MazeEntry.sort_key)
        if items:
            tree.append({"key": key, "label": label, "items": items})

    children = []
    for key, label, _ in IMPORTED_CATEGORIES:
        items = sorted((e for e in entries if not e.mine and e.category == key), key=MazeEntry.sort_key)
        if items:
            children.append({"key": f"{IMPORTED_KEY}/{key}", "label": label, "items": items})
    if children:
        tree.append({"key": IMPORTED_KEY, "label": IMPORTED_LABEL, "children": children})
    return tree


def node_count(node):
    if "items" in node:
        return len(node["items"])
    return sum(node_count(c) for c in node["children"])


def find_node(tree, key):
    for node in tree:
        if node["key"] == key:
            return node
        if "children" in node:
            found = find_node(node["children"], key)
            if found:
                return found
    return None


# ------------------------------------------------------------
# お気に入り・最近開いた迷路(MazeEntry.id で覚える)
# ------------------------------------------------------------

def load_prefs():
    try:
        with open(PREFS_FILE, encoding="utf-8") as f:
            prefs = json.load(f)
    except (OSError, ValueError):
        prefs = {}
    prefs.setdefault("favorites", [])
    prefs.setdefault("recent", [])
    return prefs


def save_prefs(prefs):
    with open(PREFS_FILE, "w", encoding="utf-8") as f:
        json.dump(prefs, f, ensure_ascii=False, indent=2)


def is_favorite(prefs, entry):
    return entry.id in prefs["favorites"]


def toggle_favorite(prefs, entry):
    if entry.id in prefs["favorites"]:
        prefs["favorites"].remove(entry.id)
    else:
        prefs["favorites"].append(entry.id)
    save_prefs(prefs)


def add_recent(prefs, entry):
    if entry.id in prefs["recent"]:
        prefs["recent"].remove(entry.id)
    prefs["recent"].insert(0, entry.id)
    del prefs["recent"][RECENT_MAX:]
    save_prefs(prefs)


# ------------------------------------------------------------
# プレビュー用に壁とゴールを読む(sim_core.c の SimCore_LoadMazeFile と同じ読み方)
# ------------------------------------------------------------

def read_maze(path, n=None):
    """(north, east, goals) を返す。north[y][x] は区画(x,y)の北の壁、east[y][x] は東の壁(True/False)。
    goals は区画の真ん中に G がある区画の集合。n を省くとファイルから大きさを決める。読めなければ None。"""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            lines = [line.rstrip("\r\n") for line in f]
    except OSError:
        return None
    if n is None:
        n = maze_size(path)
    if not n or len(lines) < 2 * n + 1:
        return None
    width = 4 * n + 1
    lines = [line.ljust(width) for line in lines]
    north = [[False] * n for _ in range(n)]
    east = [[False] * n for _ in range(n)]
    goals = set()
    for y in range(n):
        row_north = 2 * (n - 1 - y)
        row_cell = row_north + 1
        for x in range(n):
            north[y][x] = lines[row_north][4 * x + 2] == "-"
            east[y][x] = lines[row_cell][4 * x + 4] == "|"
            if lines[row_cell][4 * x + 2] == "G":
                goals.add((x, y))
    return north, east, goals


def read_walls(path, n=16):
    """(north, east) だけを返す(読めなければ None)"""
    r = read_maze(path, n)
    return None if r is None else (r[0], r[1])


# ------------------------------------------------------------
# CLI
# ------------------------------------------------------------

def print_tree(nodes, indent=""):
    for node in nodes:
        print(f"{indent}== {node['label']} [{node['key']}] ({node_count(node)})")
        if "children" in node:
            print_tree(node["children"], indent + "   ")
        else:
            for e in node["items"]:
                print(f"{indent}   {e.label:<34} {e.name}")


def main():
    p = argparse.ArgumentParser(description="List / select maze files")
    p.add_argument("--paths", metavar="KEY",
                   help="print paths of a list (alljapan, student, regional, mine_other, favorites, recent, "
                        "imported, imported/<category>)")
    p.add_argument("--year", type=int, help="only this year")
    p.add_argument("--type", help="own mazes only: CM / MM / MS / CF / SC")
    p.add_argument("--round", choices=["final", "qual"], help="imported mazes only: finals / qualifiers")
    p.add_argument("--all-sizes", action="store_true", help=f"include mazes that are not {SIM_SIZE}x{SIM_SIZE}")
    args = p.parse_args()

    entries = load_entries()
    if not entries:
        print("no mazes. convert diagrams with maze_batch.py or run fetch_mazes.sh")
        return 0
    tree = build_tree(entries, load_prefs())

    if args.paths:
        # Windows でも行末を LF にする(bash の $(...) で受けたときにパスの末尾に CR が残らないように)
        sys.stdout.reconfigure(newline=chr(10))
        node = find_node(tree, args.paths)
        if node is None:
            print(f"unknown key: {args.paths}", file=sys.stderr)
            return 2
        items = node["items"] if "items" in node else [e for c in node["children"] for e in c["items"]]
        if args.year:
            items = [e for e in items if e.year == args.year]
        if args.type:
            items = [e for e in items if args.type in e.types]
        if args.round:
            word = "決勝" if args.round == "final" else "予選"
            items = [e for e in items if word in e.detail]
        if not args.all_sizes:
            items = [e for e in items if e.playable]
        for e in items:
            print(os.path.relpath(e.path).replace("\\", "/"))
        return 0

    print_tree(tree)
    return 0


if __name__ == "__main__":
    sys.exit(main())
