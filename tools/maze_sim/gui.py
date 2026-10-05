"""yuho の迷路探索を pygame で見るGUI。

探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行う (sim_lib.py 経由)。
このファイルは描画と操作だけを受け持つ。

使い方:
    python tools/maze_sim/gui.py                    乱数の迷路(シード1)
    python tools/maze_sim/gui.py --random 7         シード7の乱数迷路
    python tools/maze_sim/gui.py maze.txt           迷路ファイル(classic 形式)
    python tools/maze_sim/gui.py tools/maze_sim/mazes/alljapan-045-2024-exp-fin.txt
                                                    大会の迷路(fetch_mazes.sh で取ってくる)
オプション:
    --algo dijkstra|adachi     探索のアルゴリズム
    --goal S,T90,T180,K        行きのコスト(直進,90°,180°,既知区画の上乗せ)
    --back S,T90,T180,K        帰りのコスト
    --speed N                  1秒あたりの区画数(既定 8)

操作(画面右にも出る):
    SPACE / P   再生・一時停止          S / →   1区画だけ進める(一時停止中)
    F           最後まで一気に進める    R       同じ迷路で最初から
    N / B       次 / 前の迷路(乱数ならシード、ファイルなら同じフォルダの次のファイル)
    A           アルゴリズムを切り替えて最初から
    C           コスト(歩数)の表示      T       まだ見ていない壁の表示
    ↑ / ↓       速さ                   ESC / Q 終わる
"""

import argparse
import os
import sys

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # pygame の起動メッセージを出さない
import pygame  # noqa: E402

import sim_lib as sl  # noqa: E402

CELL = 36       # 1区画の大きさ[px]
MARGIN = 28
PANEL_W = 330   # 右側の情報欄の幅
FPS = 60

COLOR_BG = (24, 24, 28)
COLOR_GRID = (52, 52, 58)
COLOR_KNOWN_CELL = (40, 64, 44)      # 4方向とも壁が分かった区画
COLOR_GOAL = (90, 70, 20)
COLOR_START = (30, 50, 90)
COLOR_WALL_KNOWN = (235, 60, 60)     # 見つけた壁
COLOR_WALL_HIDDEN = (90, 90, 96)     # まだ見ていない本当の壁
COLOR_MOUSE = (0, 200, 255)
COLOR_MOUSE_DIR = (255, 230, 80)
COLOR_TRAIL = (0, 120, 160)
COLOR_ROUTE = (255, 210, 60)
COLOR_TEXT = (225, 225, 230)
COLOR_DIM = (140, 140, 150)
COLOR_BAD = (255, 90, 90)
COLOR_GOOD = (120, 230, 120)


def parse_cost(text):
    parts = text.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("cost must be S,T90,T180,K")
    return tuple(int(p) for p in parts)


class MazeGui:
    def __init__(self, args):
        self.sim = sl.MazeSim()
        self.n = self.sim.size
        self.maze_file = args.maze_file
        self.seed = args.random
        self.algo = sl.ALGO_ADACHI if args.algo == "adachi" else sl.ALGO_DIJKSTRA
        self.goal_cost = args.goal
        self.back_cost = args.back
        self.speed = args.speed

        self.show_cost = True
        self.show_hidden = True
        self.paused = True
        self.message = ""

        pygame.init()
        maze_px = self.n * CELL
        self.screen = pygame.display.set_mode((MARGIN * 2 + maze_px + PANEL_W, MARGIN * 2 + maze_px))
        pygame.display.set_caption("yuho maze simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 15)
        self.font_small = pygame.font.SysFont("consolas", 11)
        self.font_big = pygame.font.SysFont("consolas", 20, bold=True)

        self.load_maze()

    # ------------------------------------------------------------
    # 迷路と探索の状態
    # ------------------------------------------------------------

    def load_maze(self):
        if self.maze_file:
            if not self.sim.load_file(self.maze_file):
                print(f"cannot load {self.maze_file}", file=sys.stderr)
                sys.exit(2)
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

    def draw_cells(self):
        for y in range(self.n):
            for x in range(self.n):
                r = self.cell_rect(x, y)
                if (x, y) in self.sim.goals:
                    color = COLOR_GOAL
                elif (x, y) == self.sim.start:
                    color = COLOR_START
                elif self.sim.cell_known(x, y):
                    color = COLOR_KNOWN_CELL
                else:
                    continue
                pygame.draw.rect(self.screen, color, r.inflate(-2, -2))

        for i in range(self.n + 1):
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN + i * CELL, MARGIN),
                             (MARGIN + i * CELL, MARGIN + self.n * CELL))
            pygame.draw.line(self.screen, COLOR_GRID, (MARGIN, MARGIN + i * CELL),
                             (MARGIN + self.n * CELL, MARGIN + i * CELL))

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

    def draw_panel(self):
        x0 = MARGIN * 2 + self.n * CELL
        y = MARGIN
        lines = []

        def add(text, color=COLOR_TEXT, font=None):
            lines.append((text, color, font or self.font))

        add("yuho maze simulator", COLOR_TEXT, self.font_big)
        add("")
        maze_name = os.path.basename(self.maze_file) if self.maze_file else f"random seed {self.seed}"
        add(f"maze   : {maze_name}")
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
                  "R        restart", "N / B    next / prev maze", "A        switch algo",
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

    def neighbor_file(self, step):
        """今の迷路ファイルと同じフォルダにある .txt を名前順に並べ、step 個先のファイル"""
        folder = os.path.dirname(os.path.abspath(self.maze_file))
        names = sorted(f for f in os.listdir(folder) if f.lower().endswith(".txt"))
        current = os.path.basename(self.maze_file)
        i = names.index(current) if current in names else 0
        return os.path.join(folder, names[(i + step) % len(names)])

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
            if not self.paused and not self.finished():
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
    p.add_argument("--random", type=int, default=1, help="seed of the random maze (default 1)")
    p.add_argument("--algo", choices=sl.ALGO_NAMES, default="dijkstra")
    p.add_argument("--goal", type=parse_cost, default=None, help="cost to goal: S,T90,T180,K")
    p.add_argument("--back", type=parse_cost, default=None, help="cost back to start: S,T90,T180,K")
    p.add_argument("--speed", type=int, default=8, help="cells per second (default 8)")
    args = p.parse_args()
    MazeGui(args).run()


if __name__ == "__main__":
    main()
