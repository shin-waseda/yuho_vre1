"""yuho の迷路探索を pygame で見るGUI。

探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行う (sim_lib.py 経由)。
迷路の分類・お気に入りは maze_catalog.py が行う。このファイルは描画と操作だけを受け持つ。

使い方:
    python tools/maze_sim/gui.py                    前回開いた迷路(なければ乱数の迷路)
    python tools/maze_sim/gui.py --random 7         シード7の乱数迷路
    python tools/maze_sim/gui.py maze.txt           迷路ファイル(classic 形式)
オプション:
    --algo dijkstra|adachi     探索のアルゴリズム
    --goal S,T90,T180,K        行きのコスト(直進,90°,180°,既知区画の上乗せ)
    --back S,T90,T180,K        帰りのコスト
    --speed N                  1秒あたりの区画数(既定 8)

操作(画面右にも出る):
    SPACE / P   再生・一時停止          S / →   1区画だけ進める(一時停止中)
    F           最後まで一気に進める    R       同じ迷路で最初から
    L           迷路の一覧(大会・年・予選/決勝で分類、お気に入り・最近開いた迷路)
    M           今の迷路をお気に入りに登録 / 解除
    N / B       次 / 前の迷路(一覧の同じ分類の中で。乱数ならシード)
    A           アルゴリズムを切り替えて最初から
    C           コスト(歩数)の表示      T       まだ見ていない壁の表示
    ↑ / ↓       速さ                   ESC / Q 終わる
迷路の一覧の中:
    ↑ / ↓ / PageUp / PageDown  選ぶ(選んだ迷路は左にプレビューが出る)
    → / Enter  分類に入る・迷路を開く    ← / BackSpace  分類の一覧へ戻る
    M          お気に入りに登録 / 解除    ESC / L        一覧を閉じる
"""

import argparse
import os
import sys

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # pygame の起動メッセージを出さない
import pygame  # noqa: E402

import maze_catalog as mc  # noqa: E402
import sim_lib as sl  # noqa: E402

CELL = 36       # 1区画の大きさ[px]
MARGIN = 28
PANEL_W = 400   # 右側の情報欄の幅
FPS = 60

COLOR_BG = (24, 24, 28)
COLOR_GRID = (52, 52, 58)
COLOR_KNOWN_CELL = (40, 64, 44)      # 4方向とも壁が分かった区画
COLOR_GOAL = (90, 70, 20)
COLOR_START = (30, 50, 90)
COLOR_WALL_KNOWN = (235, 60, 60)     # 見つけた壁
COLOR_WALL_HIDDEN = (90, 90, 96)     # まだ見ていない本当の壁
COLOR_WALL_PREVIEW = (210, 210, 220) # 一覧のプレビューの壁
COLOR_MOUSE = (0, 200, 255)
COLOR_MOUSE_DIR = (255, 230, 80)
COLOR_TRAIL = (0, 120, 160)
COLOR_ROUTE = (255, 210, 60)
COLOR_TEXT = (225, 225, 230)
COLOR_DIM = (140, 140, 150)
COLOR_BAD = (255, 90, 90)
COLOR_GOOD = (120, 230, 120)
COLOR_SELECT = (60, 70, 110)
COLOR_STAR = (255, 210, 60)

JP_FONTS = ["bizudgothic", "meiryo", "yugothic", "msgothic"]


def parse_cost(text):
    parts = text.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("cost must be S,T90,T180,K")
    return tuple(int(p) for p in parts)


def jp_font(size):
    """日本語が出るフォント(なければ既定のフォント)"""
    for name in JP_FONTS:
        if pygame.font.match_font(name):
            return pygame.font.SysFont(name, size)
    return pygame.font.SysFont(None, size + 4)


class MazeGui:
    def __init__(self, args):
        self.sim = sl.MazeSim()
        self.n = self.sim.size
        self.algo = sl.ALGO_ADACHI if args.algo == "adachi" else sl.ALGO_DIJKSTRA
        self.goal_cost = args.goal
        self.back_cost = args.back
        self.speed = args.speed

        # 迷路の一覧(my_mazes/ と mazes/)とお気に入り
        self.entries = mc.load_entries()
        self.prefs = mc.load_prefs()

        # 開く迷路: 引数 → (どちらもなければ)前回開いた迷路 → 乱数の迷路
        self.maze_file = args.maze_file
        self.seed = args.random if args.random is not None else 1
        if self.maze_file is None and args.random is None:
            by_id = {e.id: e for e in self.entries}
            recent = [by_id[i] for i in self.prefs["recent"] if i in by_id and by_id[i].playable]
            if recent:
                self.maze_file = recent[0].path

        self.show_cost = True
        self.show_hidden = True
        self.paused = True
        self.message = ""

        # 迷路の一覧の状態
        self.browser_open = False
        self.browser_path = []     # 今いる場所(一覧の木のノードのキーを根から並べたもの)。[] なら一番上
        self.browser_index = 0
        self.browser_scroll = 0
        self.browser_msg = ""
        self.preview_cache = {}

        pygame.init()
        maze_px = self.n * CELL
        self.screen = pygame.display.set_mode((MARGIN * 2 + maze_px + PANEL_W, MARGIN * 2 + maze_px))
        pygame.display.set_caption("yuho maze simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 15)
        self.font_small = pygame.font.SysFont("consolas", 11)
        self.font_big = pygame.font.SysFont("consolas", 20, bold=True)
        self.font_jp = jp_font(15)
        self.font_jp_small = jp_font(12)

        self.load_maze()

    # ------------------------------------------------------------
    # 迷路と探索の状態
    # ------------------------------------------------------------

    def current_entry(self):
        """今の迷路が一覧(my_mazes/ か mazes/)のものなら、その MazeEntry"""
        if not self.maze_file:
            return None
        target = os.path.normcase(os.path.abspath(self.maze_file))
        for e in self.entries:
            if os.path.normcase(os.path.abspath(e.path)) == target:
                return e
        return None

    def load_maze(self):
        if self.maze_file:
            if not self.sim.load_file(self.maze_file):
                print(f"cannot load {self.maze_file} (not a {self.n}x{self.n} maze?)", file=sys.stderr)
                sys.exit(2)
            e = self.current_entry()
            if e:
                mc.add_recent(self.prefs, e)
        else:
            self.sim.new_random(self.seed)
        self.restart()

    def restart(self):
        if not self.sim.reset(self.algo, self.goal_cost, self.back_cost):
            print("invalid cost (straight must be >= 1, total must fit in 13 bits)", file=sys.stderr)
            sys.exit(2)
        self.status = sl.SIM_MOVED
        self.last_action = None
        self.pose = self.sim.pose()
        self.prev_pose = self.pose
        self.anim = 1.0           # 0→1 で prev_pose から pose へ動く
        self.trail = [self.pose[:2]]
        self.step_timer = 0.0
        self.route = None         # 探索が終わったら (指令の列, 見つけたコスト, 真の最短)
        self.message = ""

    def finished(self):
        return self.status not in (sl.SIM_MOVED, sl.SIM_AT_GOAL)

    def do_step(self):
        if self.finished():
            return
        self.prev_pose = self.pose
        self.status, action, cells = self.sim.step()
        self.pose = self.sim.pose()
        self.last_action = (action, cells)
        self.anim = 0.0
        if self.pose[:2] != self.trail[-1]:
            self.trail.append(self.pose[:2])

        if self.status == sl.SIM_AT_GOAL:
            self.message = "reached goal"
        elif self.status == sl.SIM_DONE:
            self.route = self.sim.route()
            self.message = "search done"
            self.paused = True
        elif self.finished():
            self.message = sl.STATUS_NAMES[self.status]
            self.paused = True

    def run_to_end(self):
        while not self.finished():
            self.do_step()
        self.anim = 1.0

    # ------------------------------------------------------------
    # 迷路の一覧(分類・お気に入り)
    # ------------------------------------------------------------

    def browser_tree(self):
        return mc.build_tree(self.entries, self.prefs)

    def browser_rows(self):
        """今の階層に並ぶもの(ノードの dict か MazeEntry)"""
        tree = self.browser_tree()
        if not self.browser_path:
            return tree
        node = mc.find_node(tree, self.browser_path[-1])
        if node is None:
            self.browser_path = []
            return tree
        return node["children"] if "children" in node else node["items"]

    @staticmethod
    def first_entry(row):
        """ノードならその中の最初の迷路、迷路ならそのもの"""
        while isinstance(row, dict):
            inner = row.get("items") or row.get("children") or []
            if not inner:
                return None
            row = inner[0]
        return row

    def open_browser(self):
        if not self.entries:
            self.message = "no mazes: run maze_batch.py or fetch_mazes.sh"
            return
        self.browser_open = True
        self.browser_scroll = 0
        self.browser_msg = ""
        # 今の迷路があれば、その分類の中でその迷路を選んだ状態で開く
        e = self.current_entry()
        if e:
            self.browser_path = [e.node_key] if e.mine else [mc.IMPORTED_KEY, e.node_key]
            ids = [r.id for r in self.browser_rows() if not isinstance(r, dict)]
            self.browser_index = ids.index(e.id) if e.id in ids else 0
        else:
            self.browser_path = []
            self.browser_index = 0

    def browser_selected_entry(self):
        """プレビューする迷路(ノードを選んでいるときはその中の最初の迷路)"""
        rows = self.browser_rows()
        if not rows:
            return None
        return self.first_entry(rows[min(self.browser_index, len(rows) - 1)])

    def browser_key(self, key):
        rows = self.browser_rows()
        count = len(rows)
        self.browser_msg = ""
        if key in (pygame.K_ESCAPE, pygame.K_l):
            self.browser_open = False
        elif key == pygame.K_UP:
            self.browser_index = max(0, self.browser_index - 1)
        elif key == pygame.K_DOWN:
            self.browser_index = min(count - 1, self.browser_index + 1)
        elif key == pygame.K_PAGEUP:
            self.browser_index = max(0, self.browser_index - 10)
        elif key == pygame.K_PAGEDOWN:
            self.browser_index = min(count - 1, self.browser_index + 10)
        elif key in (pygame.K_RIGHT, pygame.K_RETURN, pygame.K_KP_ENTER) and count > 0:
            row = rows[self.browser_index]
            if isinstance(row, dict):
                self.browser_path.append(row["key"])
                self.browser_index = 0
                self.browser_scroll = 0
            elif not row.playable:
                self.browser_msg = f"{row.size}×{row.size} はシミュレータ未対応(プレビューのみ)"
            else:
                self.maze_file = row.path
                self.browser_open = False
                self.load_maze()
        elif key in (pygame.K_LEFT, pygame.K_BACKSPACE) and self.browser_path:
            left = self.browser_path.pop()
            keys = [r["key"] for r in self.browser_rows() if isinstance(r, dict)]
            self.browser_index = keys.index(left) if left in keys else 0
            self.browser_scroll = 0
        elif key == pygame.K_m and count > 0 and not isinstance(rows[self.browser_index], dict):
            mc.toggle_favorite(self.prefs, rows[self.browser_index])
            # お気に入りの中で解除したら、その行は消えるので位置を詰める
            self.browser_index = min(self.browser_index, max(0, len(self.browser_rows()) - 1))

    def neighbor_file(self, step):
        """一覧の同じ分類の中で step 個先の(開ける)迷路。一覧にないファイルなら同じフォルダの名前順"""
        e = self.current_entry()
        if e:
            node = mc.find_node(self.browser_tree(), e.node_key)
            items = [x for x in node["items"] if x.playable] if node else []
            ids = [x.id for x in items]
            if e.id in ids:
                return items[(ids.index(e.id) + step) % len(items)].path
        folder = os.path.dirname(os.path.abspath(self.maze_file))
        names = sorted(f for f in os.listdir(folder) if f.lower().endswith(".txt"))
        current = os.path.basename(self.maze_file)
        i = names.index(current) if current in names else 0
        return os.path.join(folder, names[(i + step) % len(names)])

    # ------------------------------------------------------------
    # 座標
    # ------------------------------------------------------------

    def cell_rect(self, x, y):
        px = MARGIN + x * CELL
        py = MARGIN + (self.n - 1 - y) * CELL
        return pygame.Rect(px, py, CELL, CELL)

    def cell_center(self, x, y):
        r = self.cell_rect(x, y)
        return r.centerx, r.centery

    def wall_line(self, x, y, d):
        r = self.cell_rect(x, y)
        if d == 0:
            return (r.left, r.top), (r.right, r.top)
        if d == 1:
            return (r.right, r.top), (r.right, r.bottom)
        if d == 2:
            return (r.left, r.bottom), (r.right, r.bottom)
        return (r.left, r.top), (r.left, r.bottom)

    # ------------------------------------------------------------
    # 描画
    # ------------------------------------------------------------

    def draw_grid(self):
        for i in range(self.n + 1):
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN + i * CELL, MARGIN),
                             (MARGIN + i * CELL, MARGIN + self.n * CELL))
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN, MARGIN + i * CELL),
                             (MARGIN + self.n * CELL, MARGIN + i * CELL))

    def draw_start_goal(self):
        for (x, y) in self.sim.goals:
            pygame.draw.rect(self.screen, COLOR_GOAL, self.cell_rect(x, y).inflate(-2, -2))
        sx, sy = self.sim.start
        pygame.draw.rect(self.screen, COLOR_START, self.cell_rect(sx, sy).inflate(-2, -2))

    def draw_cells(self):
        for y in range(self.n):
            for x in range(self.n):
                if (x, y) in self.sim.goals or (x, y) == self.sim.start:
                    continue
                if self.sim.cell_known(x, y):
                    pygame.draw.rect(self.screen, COLOR_KNOWN_CELL, self.cell_rect(x, y).inflate(-2, -2))
        self.draw_start_goal()
        self.draw_grid()

    def draw_cost(self):
        if not self.show_cost:
            return
        for y in range(self.n):
            for x in range(self.n):
                v = self.sim.cell_value(x, y)
                if v is None:
                    continue
                img = self.font_small.render(str(v), True, COLOR_DIM)
                cx, cy = self.cell_center(x, y)
                self.screen.blit(img, img.get_rect(center=(cx, cy)))

    def draw_walls(self):
        # 北と東だけ見れば内壁を1回ずつ、外周の南と西は別に描く
        for y in range(self.n):
            for x in range(self.n):
                dirs = [0, 1]
                if y == 0:
                    dirs.append(2)
                if x == 0:
                    dirs.append(3)
                for d in dirs:
                    truth = self.sim.wall(0, x, y, d)
                    known = self.sim.wall(1, x, y, d)
                    if known == sl.WALL_EXISTS:
                        a, b = self.wall_line(x, y, d)
                        pygame.draw.line(self.screen, COLOR_WALL_KNOWN, a, b, 4)
                    elif known == sl.WALL_UNKNOWN and truth == sl.WALL_EXISTS and self.show_hidden:
                        a, b = self.wall_line(x, y, d)
                        pygame.draw.line(self.screen, COLOR_WALL_HIDDEN, a, b, 3)

    def draw_trail(self):
        if len(self.trail) < 2:
            return
        pts = [self.cell_center(x, y) for x, y in self.trail]
        pygame.draw.lines(self.screen, COLOR_TRAIL, False, pts, 2)

    def draw_route(self):
        if not self.route:
            return
        actions = self.route[0]
        x, y = self.sim.start
        d = 0
        pts = [self.cell_center(x, y)]
        for t, cells in actions:
            d = (d + sl.ACTION_QUARTER_TURNS[t]) % 4
            for _ in range(cells):
                x += sl.DIR_DX[d]
                y += sl.DIR_DY[d]
                pts.append(self.cell_center(x, y))
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, COLOR_ROUTE, False, pts, 4)

    def draw_mouse(self):
        x0, y0, _ = self.prev_pose
        x1, y1, d = self.pose
        a = min(self.anim, 1.0)
        c0 = self.cell_center(x0, y0)
        c1 = self.cell_center(x1, y1)
        cx = c0[0] + (c1[0] - c0[0]) * a
        cy = c0[1] + (c1[1] - c0[1]) * a
        r = CELL * 0.28
        pygame.draw.circle(self.screen, COLOR_MOUSE, (int(cx), int(cy)), int(r))
        tip = (cx + sl.DIR_DX[d] * r, cy - sl.DIR_DY[d] * r)
        pygame.draw.line(self.screen, COLOR_MOUSE_DIR, (cx, cy), tip, 3)

    def draw_preview(self, entry):
        """一覧で選んでいる迷路を、全部の壁が見えた状態で描く(32×32 なども縮めて描く)"""
        if entry is None:
            self.draw_start_goal()
            self.draw_grid()
            return
        if entry.path not in self.preview_cache:
            self.preview_cache[entry.path] = mc.read_maze(entry.path)
        maze = self.preview_cache[entry.path]
        if maze is None:
            return
        north, east, goals = maze
        n = len(north)
        cs = CELL * self.n / n  # 画面上の区画の大きさ

        def rect(x, y):
            return pygame.Rect(int(MARGIN + x * cs), int(MARGIN + (n - 1 - y) * cs), int(cs) + 1, int(cs) + 1)

        for (x, y) in goals:
            pygame.draw.rect(self.screen, COLOR_GOAL, rect(x, y).inflate(-2, -2))
        pygame.draw.rect(self.screen, COLOR_START, rect(0, 0).inflate(-2, -2))
        size_px = self.n * CELL
        for i in range(n + 1):
            v = int(MARGIN + i * cs)
            pygame.draw.line(self.screen, COLOR_GRID, (v, MARGIN), (v, MARGIN + size_px))
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN, v), (MARGIN + size_px, v))

        width = 3 if n <= 16 else 2
        for y in range(n):
            for x in range(n):
                r = rect(x, y)
                if north[y][x] or y == n - 1:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, r.topleft, (r.left + int(cs), r.top), width)
                if east[y][x] or x == n - 1:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, (r.left + int(cs), r.top),
                                     (r.left + int(cs), r.top + int(cs)), width)
                if y == 0:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, (r.left, r.top + int(cs)),
                                     (r.left + int(cs), r.top + int(cs)), width)
                if x == 0:
                    pygame.draw.line(self.screen, COLOR_WALL_PREVIEW, r.topleft, (r.left, r.top + int(cs)), width)

    def blit(self, text, font, color, pos):
        self.screen.blit(font.render(text, True, color), pos)

    def draw_browser(self):
        x0 = MARGIN * 2 + self.n * CELL
        width = PANEL_W - MARGIN
        y = MARGIN
        tree = self.browser_tree()
        title = "迷路の一覧"
        for key in self.browser_path:
            node = mc.find_node(tree, key)
            if node:
                title += " > " + node["label"]
        self.blit(title, self.font_jp, COLOR_TEXT, (x0, y))
        y += self.font_jp.get_linesize() + 6

        help_lines = ["↑↓ 選ぶ  →/Enter 開く  ← 戻る", "M お気に入り  ESC/L 閉じる"]
        if self.browser_msg:
            help_lines.insert(0, self.browser_msg)
        bottom = self.screen.get_height() - MARGIN - len(help_lines) * self.font_jp_small.get_linesize()

        rows = self.browser_rows()
        has_entries = any(not isinstance(r, dict) for r in rows)
        row_h = self.font_jp.get_linesize() + (self.font_jp_small.get_linesize() if has_entries else 0) + 4
        visible = max(1, (bottom - y) // row_h)
        # 選んでいる行が見えるようにずらす
        if self.browser_index < self.browser_scroll:
            self.browser_scroll = self.browser_index
        elif self.browser_index >= self.browser_scroll + visible:
            self.browser_scroll = self.browser_index - visible + 1

        for i in range(self.browser_scroll, min(len(rows), self.browser_scroll + visible)):
            rect = pygame.Rect(x0 - 4, y, width, row_h - 2)
            if i == self.browser_index:
                pygame.draw.rect(self.screen, COLOR_SELECT, rect)
            row = rows[i]
            if isinstance(row, dict):
                mark = " ▶" if "children" in row else ""
                self.blit(f"{row['label']}  ({mc.node_count(row)}){mark}", self.font_jp, COLOR_TEXT, (x0, y + 2))
            else:
                star = "★ " if mc.is_favorite(self.prefs, row) else "   "
                color = COLOR_TEXT if row.playable else COLOR_DIM
                self.blit(star, self.font_jp, COLOR_STAR, (x0, y + 2))
                self.blit(row.label, self.font_jp, color, (x0 + 22, y + 2))
                self.blit(row.name, self.font_jp_small, COLOR_DIM, (x0 + 22, y + 2 + self.font_jp.get_linesize()))
            y += row_h

        if len(rows) > visible:
            self.blit(f"{self.browser_index + 1} / {len(rows)}", self.font_small, COLOR_DIM, (x0 + width - 60, MARGIN))

        y = bottom
        for line in help_lines:
            color = COLOR_BAD if line == self.browser_msg else COLOR_DIM
            self.blit(line, self.font_jp_small, color, (x0, y))
            y += self.font_jp_small.get_linesize()

    def draw_panel(self):
        x0 = MARGIN * 2 + self.n * CELL
        y = MARGIN
        lines = []

        def add(text, color=COLOR_TEXT, font=None):
            lines.append((text, color, font or self.font))

        add("yuho maze simulator", COLOR_TEXT, self.font_big)
        add("")
        e = self.current_entry()
        if e:
            star = "★ " if mc.is_favorite(self.prefs, e) else ""
            where = "" if e.mine else "インポート: "
            add(f"{star}{where}{e.label}", COLOR_STAR if star else COLOR_TEXT, self.font_jp)
            add(f"maze   : {e.name}")
        elif self.maze_file:
            add(f"maze   : {os.path.basename(self.maze_file)}")
        else:
            add(f"maze   : random seed {self.seed}")
        add(f"algo   : {sl.ALGO_NAMES[self.algo]}")
        if self.algo == sl.ALGO_DIJKSTRA:
            add(f"goal   : {self.cost_text(self.goal_cost, False)}")
            add(f"back   : {self.cost_text(self.back_cost, True)}")
        add(f"speed  : {self.speed} cells/s")
        # 区画の数字は、プランナーが最後に計算した目的地までの値
        target = "goal" if self.sim.phase() == 0 else "start"
        unit = "steps" if self.algo == sl.ALGO_ADACHI else "cost"
        add(f"numbers: {unit} to {target}", COLOR_DIM)
        add("")

        phase = sl.PHASE_NAMES[self.sim.phase()]
        bad = self.status in (sl.SIM_FAILED, sl.SIM_CRASH, sl.SIM_LOST)
        add(f"phase  : {phase}", COLOR_BAD if bad else COLOR_TEXT)
        x, y_, d = self.pose
        add(f"pos    : ({x},{y_}) {'NESW'[d]}")
        if self.last_action:
            t, cells = self.last_action
            name = sl.ACTION_NAMES[t] + (f" x{cells}" if t == sl.ACTION_FORWARD else "")
            add(f"action : {name}")
        to_goal, back = self.sim.moves()
        add(f"moves  : {to_goal} + {back} = {to_goal + back}")
        if self.message:
            add(f">> {self.message}", COLOR_BAD if bad else COLOR_GOOD)

        if self.route:
            actions, found, best = self.route
            add("")
            add("fastest-run route (known walls)")
            if found == best:
                add(f"  cost {found} = optimal", COLOR_GOOD)
            else:
                add(f"  cost {found} (best {best})", COLOR_TEXT)
            add(f"  {len(actions) - 1} commands")

        add("")
        add("PAUSED" if self.paused else "RUNNING", COLOR_GOOD if not self.paused else COLOR_DIM)
        for k in ["SPACE/P  play / pause", "S / ->   step", "F        run to end",
                  "R        restart", "L        maze list", "M        favorite",
                  "N / B    next / prev maze", "A        switch algo",
                  f"C        cost [{'on' if self.show_cost else 'off'}]",
                  f"T        hidden walls [{'on' if self.show_hidden else 'off'}]",
                  "UP/DOWN  speed", "ESC/Q    quit"]:
            add(k, COLOR_DIM)

        for text, color, font in lines:
            if text:
                self.screen.blit(font.render(text, True, color), (x0, y))
            y += font.get_linesize()

    def cost_text(self, cost, back):
        if cost is None:
            return "params.h" + (" (+known)" if back else "")
        return ",".join(str(c) for c in cost)

    def draw(self):
        self.screen.fill(COLOR_BG)
        if self.browser_open:
            self.draw_preview(self.browser_selected_entry())
            self.draw_browser()
        else:
            self.draw_cells()
            self.draw_cost()
            self.draw_trail()
            self.draw_route()
            self.draw_walls()
            self.draw_mouse()
            self.draw_panel()
        pygame.display.flip()

    # ------------------------------------------------------------
    # 操作
    # ------------------------------------------------------------

    def handle_key(self, key):
        if self.browser_open:
            self.browser_key(key)
            return True
        if key in (pygame.K_ESCAPE, pygame.K_q):
            return False
        if key in (pygame.K_SPACE, pygame.K_p):
            if not self.finished():
                self.paused = not self.paused
        elif key in (pygame.K_s, pygame.K_RIGHT):
            self.paused = True
            self.do_step()
        elif key == pygame.K_f:
            self.run_to_end()
        elif key == pygame.K_r:
            self.restart()
        elif key == pygame.K_l:
            self.paused = True
            self.open_browser()
        elif key == pygame.K_m:
            e = self.current_entry()
            if e:
                mc.toggle_favorite(self.prefs, e)
        elif key in (pygame.K_n, pygame.K_b):
            step = 1 if key == pygame.K_n else -1
            if self.maze_file:
                self.maze_file = self.neighbor_file(step)
            else:
                self.seed = max(0, self.seed + step)
            self.load_maze()
        elif key == pygame.K_a:
            self.algo = sl.ALGO_ADACHI if self.algo == sl.ALGO_DIJKSTRA else sl.ALGO_DIJKSTRA
            self.restart()
        elif key == pygame.K_c:
            self.show_cost = not self.show_cost
        elif key == pygame.K_t:
            self.show_hidden = not self.show_hidden
        elif key == pygame.K_UP:
            self.speed = min(200, self.speed * 2)
        elif key == pygame.K_DOWN:
            self.speed = max(1, self.speed // 2)
        return True

    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    running = self.handle_key(event.key)

            interval = 1.0 / self.speed
            self.anim += dt / min(interval, 0.25)
            if not self.paused and not self.finished() and not self.browser_open:
                self.step_timer += dt
                while self.step_timer >= interval and not self.finished():
                    self.step_timer -= interval
                    self.do_step()
                if self.speed > 30:
                    self.anim = 1.0  # 速いときは補間しない

            self.draw()
        pygame.quit()


def main():
    p = argparse.ArgumentParser(description="GUI for the yuho maze logic (same sources as the firmware)")
    p.add_argument("maze_file", nargs="?", help="maze file (classic text format)")
    p.add_argument("--random", type=int, default=None, help="seed of the random maze")
    p.add_argument("--algo", choices=sl.ALGO_NAMES, default="dijkstra")
    p.add_argument("--goal", type=parse_cost, default=None, help="cost to goal: S,T90,T180,K")
    p.add_argument("--back", type=parse_cost, default=None, help="cost back to start: S,T90,T180,K")
    p.add_argument("--speed", type=int, default=8, help="cells per second (default 8)")
    args = p.parse_args()
    MazeGui(args).run()


if __name__ == "__main__":
    main()
