#!/usr/bin/env python3
"""yuho のログ(SD の .bin、get_log.py が作った .csv)を見る GUI。

- 左: 列を選ぶ(クリックで付け外し)。同じ種類の列(速度・角度・壁センサーなど)は同じ段に描く。
- 真ん中: 時系列のグラフ。下のツールバーで拡大・移動。イベントの所に縦線を引き、
  見えているイベントが少ないときは名前も書く。グラフをクリックすると、その時刻にカーソルを置く。
- 右上: イベントの一覧(ev_text)。クリックするとその時刻へ飛ぶ(今の拡大の幅のまま真ん中に来る)。
  「絞る」に名前(正規表現)を書くと、その名前のイベントだけを一覧と縦線に出す。← → キーで前後のイベントへ。
- 右下: 迷路の上の軌道(車輪の距離とジャイロの向きで作る)。STEP イベントで読んだ壁を描く。
  カーソルの時刻の位置と向きを出す。軌道をクリックすると、その時刻へ飛ぶ。
  スタート区画の真ん中を、尻当て(SETPOS)が2回終わった所(なければ最初の STEP、それもなければ記録の始め)とする。
  「STEP で合わせ直す」を付けると、STEP のたびに位置と向きを区画の境界(止まっていれば真ん中)に合わせ直す。

使い方:
    pip install matplotlib numpy
    python tools/log_viewer.py                                  # 起動するとログを選ぶ画面が出る
    python tools/log_viewer.py logs/search/search_0006.bin      # 開くログを決めて起動
"""

import math
import re
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

import yuho_common as yc

LOG_DIR = yc.ROOT / "logs"
SECTION = yc.SECTION_MM

# 同じ段に描く列の組(上から順)。ここにない列は1列で1段。
GROUPS = [
    ("速度 [mm/s]", ["target", "vl", "vr", "vl_ref", "vr_ref"]),
    ("加速度 [mm/s²]", ["target_acc"]),
    ("距離 [mm]", ["pos_ref", "dist", "x_mm"]),
    ("位置の補正 [mm/s]", ["pos_corr"]),
    ("角速度 [dps]", ["omega_ref", "gyro_z", "ang_corr"]),
    ("角度 [deg]", ["angle_ref", "angle"]),
    ("壁の補正 [deg]", ["wall_ofs"]),
    ("PWM", ["pwm_l", "pwm_r"]),
    ("電圧 [V]", ["ff_l", "ff_r", "i_l", "i_r"]),
    ("壁センサー", ["ad_l", "ad_fl", "ad_fr", "ad_r"]),
    ("電池 [V]", ["vbat"]),
]
DEFAULT_COLUMNS = ["target", "vl", "vr", "angle_ref", "angle", "ad_l", "ad_fl", "ad_fr", "ad_r"]
HIDDEN_COLUMNS = {"time_s", "ev", "ev_a", "ev_b", "ev_c", "ev_d", "ev_e", "ev_text", "event"}
LABEL_MAX = 40  # 見えているイベントがこれ以下なら、縦線に名前を書く
PLAY_SPEEDS = ["0.05", "0.1", "0.25", "0.5", "1", "2", "4"]
PLAY_FRAME_MS = 40  # 再生で絵を描き直す間隔(描くのが遅ければ、実際の時間に合わせてコマを飛ばす)

DIR_NAMES = ["N", "E", "S", "W"]
MAP_CELLS_EVENT = 45  # LOG_EV_MAP_CELLS(機体の app/log_event.h)
MAP_SIZE = int(yc.read_params().get("MAZE_SIZE", 16))  # 機体の地図の1辺の区画数


def heading_deg(h: int) -> float:
    """迷路の向き(0 北 1 東 2 南 3 西)→ 数学の角度(+x が 0°、+y が 90°)"""
    return 90.0 - 90.0 * h


def dir_vec(h: int):
    a = math.radians(heading_deg(h))
    return round(math.cos(a)), round(math.sin(a))


class LogViewer:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("yuho log viewer")
        yc.setup_japanese_font()
        self.log = None
        self.t = None
        self.events = []        # [(行, 時刻, 文字)] 全部
        self.shown_events = []  # 絞った後
        self.cursor_t = None
        self.path_xy = None     # 迷路の上の軌道 (x, y, 向き[deg])
        self.label_texts = []

        # ---- 上: ファイル ----
        top = ttk.Frame(root, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(top, text="開く", command=self.open_dialog).pack(side=tk.LEFT)
        # 再生: カーソルの時刻を実際の時間 × 速さで進める(グラフ・迷路の印・イベントの一覧がついてくる)
        self.play_btn = ttk.Button(top, text="▶ 再生", width=8, command=self.toggle_play)
        self.play_btn.pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(top, text="速さ ×").pack(side=tk.LEFT)
        self.play_speed = tk.StringVar(value="1")
        ttk.Combobox(top, textvariable=self.play_speed, values=PLAY_SPEEDS, width=5,
                     state="readonly").pack(side=tk.LEFT)
        self.seek_var = tk.DoubleVar(value=0.0)
        self.seek = ttk.Scale(top, from_=0.0, to=1.0, orient=tk.HORIZONTAL, length=300,
                              variable=self.seek_var, command=self.on_seek)
        self.seek.pack(side=tk.LEFT, padx=6)
        self.seek_label = ttk.Label(top, text="", width=18, font=("Consolas", 9))
        self.seek_label.pack(side=tk.LEFT)
        self.playing = False
        self.play_job = None
        self.play_wall = 0.0  # 前のコマの実際の時刻
        self.seeking = False  # スライダーをプログラムで動かしている間は on_seek を無視する
        self.file_label = ttk.Label(top, text="(ログを開いてください)")
        self.file_label.pack(side=tk.LEFT, padx=8)

        self.status = ttk.Label(root, text="", anchor=tk.W, font=("Consolas", 9))
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

        pw = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        pw.pack(fill=tk.BOTH, expand=True)

        # ---- 左: 列 ----
        left = ttk.Frame(pw, padding=4)
        pw.add(left, weight=0)
        ttk.Label(left, text="列 (クリックで付け外し)").pack(anchor=tk.W)
        self.col_list = tk.Listbox(left, selectmode=tk.MULTIPLE, exportselection=False, width=16, height=30)
        self.col_list.pack(fill=tk.Y, expand=True)
        self.col_list.bind("<<ListboxSelect>>", lambda e: self.redraw_plots())
        grow = ttk.Frame(left)
        grow.pack(fill=tk.X)
        self.group_var = tk.StringVar(value=GROUPS[0][0])
        ttk.Combobox(grow, textvariable=self.group_var, values=[g[0] for g in GROUPS],
                     state="readonly", width=14).pack(fill=tk.X)
        ttk.Button(grow, text="この組を足す", command=self.add_group).pack(fill=tk.X)
        ttk.Button(grow, text="全部外す", command=self.clear_columns).pack(fill=tk.X)

        # ---- 真ん中: グラフ ----
        mid = ttk.Frame(pw)
        pw.add(mid, weight=3)
        self.fig = Figure(figsize=(9, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=mid)
        self.toolbar = NavigationToolbar2Tk(self.canvas, mid)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_plot_click)
        self.axes = []

        # ---- 右: イベントと迷路 ----
        right = ttk.PanedWindow(pw, orient=tk.VERTICAL)
        pw.add(right, weight=2)
        evf = ttk.Frame(right, padding=4)
        right.add(evf, weight=1)
        frow = ttk.Frame(evf)
        frow.pack(fill=tk.X)
        ttk.Label(frow, text="絞る").pack(side=tk.LEFT)
        self.ev_filter = tk.StringVar(value="")
        e1 = ttk.Entry(frow, textvariable=self.ev_filter, width=16)
        e1.pack(side=tk.LEFT)
        ttk.Label(frow, text="除く").pack(side=tk.LEFT)
        self.ev_exclude = tk.StringVar(value="STEP_INFO")
        e2 = ttk.Entry(frow, textvariable=self.ev_exclude, width=16)
        e2.pack(side=tk.LEFT)
        for e in (e1, e2):
            e.bind("<Return>", lambda ev: self.apply_event_filter())
        ttk.Button(frow, text="適用", command=self.apply_event_filter).pack(side=tk.LEFT)
        lf = ttk.Frame(evf)
        lf.pack(fill=tk.BOTH, expand=True)
        self.ev_list = tk.Listbox(lf, font=("Consolas", 9), exportselection=False, width=60)
        sb = ttk.Scrollbar(lf, orient=tk.VERTICAL, command=self.ev_list.yview)
        self.ev_list.configure(yscrollcommand=sb.set)
        self.ev_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.ev_list.bind("<<ListboxSelect>>", self.on_event_select)

        mzf = ttk.Frame(right)
        right.add(mzf, weight=2)
        mrow = ttk.Frame(mzf)
        mrow.pack(fill=tk.X)
        self.reanchor = tk.BooleanVar(value=False)
        ttk.Checkbutton(mrow, text="STEP で合わせ直す", variable=self.reanchor,
                        command=self.rebuild_maze).pack(side=tk.LEFT)
        # 壁: 読んだ壁(赤 = ある、緑の破線 = ない。カーソルの時刻まで)、機体の地図(灰 = ある、点線 = 未知)
        self.show_steps = tk.BooleanVar(value=True)
        ttk.Checkbutton(mrow, text="読んだ壁", variable=self.show_steps,
                        command=self.on_layer_toggle).pack(side=tk.LEFT, padx=(8, 0))
        self.show_map = tk.BooleanVar(value=True)
        ttk.Checkbutton(mrow, text="機体の地図", variable=self.show_map,
                        command=self.on_layer_toggle).pack(side=tk.LEFT)
        self.mfig = Figure(figsize=(5, 5))
        self.max = self.mfig.add_subplot(111)
        self.mcanvas = FigureCanvasTkAgg(self.mfig, master=mzf)
        NavigationToolbar2Tk(self.mcanvas, mzf).update()
        self.mcanvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.mcanvas.mpl_connect("button_press_event", self.on_maze_click)
        self.mcanvas.mpl_connect("draw_event", self.on_maze_draw)
        self.maze_bg = None
        self.plot_bg = None
        self.canvas.mpl_connect("draw_event", self.on_plot_draw)

        root.bind("<Left>", lambda e: self.step_event(-1))
        root.bind("<Right>", lambda e: self.step_event(+1))
        root.bind("<space>", self.on_space)

    # ---- 読む ----
    def open_dialog(self):
        start = getattr(self, "last_dir", LOG_DIR)  # 前に開いたフォルダから選ぶ
        p = filedialog.askopenfilename(title="ログ", initialdir=str(start),
                                       filetypes=[("log", "*.bin *.csv"), ("all", "*.*")])
        if p:
            self.last_dir = Path(p).parent
            self.open(Path(p))

    def open(self, path: Path):
        try:
            log = yc.load_log(path)
        except Exception as ex:  # 読めなかったことを知らせる
            messagebox.showerror("開く", f"{path.name}: {ex}")
            return
        if "time_s" not in log or len(log["time_s"]) < 2:
            messagebox.showerror("開く", f"{path.name}: time_s の列がない、または行が少ない")
            return
        self.stop_play()
        self.log = log
        self.t = np.array(log["time_s"])
        self.seeking = True
        self.seek.configure(from_=float(self.t[0]), to=float(self.t[-1]))
        self.seek_var.set(float(self.t[0]))
        self.seeking = False
        self.events = yc.find_events(log)
        dt = float(np.median(np.diff(self.t)))
        self.file_label.config(text=f"{path}  ({len(self.t)} 行, {self.t[-1] - self.t[0]:.2f} s, "
                                    f"{dt * 1000:.0f} ms ごと, イベント {len(self.events)} 個)")
        cols = [c for c in log if c not in HIDDEN_COLUMNS]
        self.col_list.delete(0, tk.END)
        for c in cols:
            self.col_list.insert(tk.END, c)
        for i, c in enumerate(cols):
            if c in DEFAULT_COLUMNS:
                self.col_list.selection_set(i)
        self.cursor_t = None
        self.axes = []  # 前のログの拡大の幅は使わない
        self.apply_event_filter(redraw=False)
        self.redraw_plots()
        self.rebuild_maze()

    # ---- 列 ----
    def selected_columns(self):
        return [self.col_list.get(i) for i in self.col_list.curselection()]

    def add_group(self):
        names = dict(GROUPS).get(self.group_var.get(), [])
        for i in range(self.col_list.size()):
            if self.col_list.get(i) in names:
                self.col_list.selection_set(i)
        self.redraw_plots()

    def clear_columns(self):
        self.col_list.selection_clear(0, tk.END)
        self.redraw_plots()

    # ---- イベント ----
    def apply_event_filter(self, redraw=True):
        inc, exc = self.ev_filter.get().strip(), self.ev_exclude.get().strip()
        try:
            inc_re = re.compile(inc, re.I) if inc else None
            exc_re = re.compile(exc, re.I) if exc else None
        except re.error as ex:
            messagebox.showwarning("絞る", f"正規表現の誤り: {ex}")
            return
        self.shown_events = [e for e in self.events
                             if (inc_re is None or inc_re.search(e[2]))
                             and (exc_re is None or not exc_re.search(e[2].split(" ", 1)[0]))]
        self.ev_list.delete(0, tk.END)
        for _, t, s in self.shown_events:
            self.ev_list.insert(tk.END, f"{t:8.3f}  {s}")
        if redraw:
            self.redraw_plots()

    def on_event_select(self, _e):
        sel = self.ev_list.curselection()
        if sel:
            self.set_cursor(self.shown_events[sel[0]][1], center=True, from_list=True)

    def step_event(self, d):
        if not self.shown_events:
            return
        sel = self.ev_list.curselection()
        if sel:
            i = sel[0] + d
        else:
            t = self.cursor_t if self.cursor_t is not None else self.t[0]
            i = int(np.searchsorted([e[1] for e in self.shown_events], t)) + (0 if d > 0 else -1)
        i = max(0, min(len(self.shown_events) - 1, i))
        self.ev_list.selection_clear(0, tk.END)
        self.ev_list.selection_set(i)
        self.ev_list.see(i)
        self.set_cursor(self.shown_events[i][1], center=True, from_list=True)

    # ---- グラフ ----
    def redraw_plots(self):
        if self.log is None:
            return
        old_xlim = self.axes[0].get_xlim() if self.axes else None
        self.fig.clear()  # 名前の文字も一緒に消える
        self.label_texts = []
        self.axes = []
        cols = self.selected_columns()
        panels = []
        used = set()
        for title, names in GROUPS:
            sel = [c for c in names if c in cols]
            if sel:
                panels.append((title, sel))
                used.update(sel)
        for c in cols:
            if c not in used:
                panels.append((c, [c]))
        if not panels:
            self.canvas.draw_idle()
            return
        ev_t = [e[1] for e in self.shown_events]
        for k, (title, names) in enumerate(panels):
            ax = self.fig.add_subplot(len(panels), 1, k + 1, sharex=self.axes[0] if self.axes else None)
            for c in names:
                ax.plot(self.t, self.log[c], lw=0.9, label=c)
            ax.set_ylabel(title, fontsize=8)
            ax.legend(loc="upper right", fontsize=7, ncol=min(len(names), 5))
            ax.grid(True, lw=0.3)
            ax.tick_params(labelsize=7)
            if ev_t:
                ax.vlines(ev_t, 0, 1, transform=ax.get_xaxis_transform(), colors="tab:purple",
                          lw=0.6, alpha=0.35)
            self.axes.append(ax)
        for ax in self.axes[:-1]:
            ax.tick_params(labelbottom=False)
        self.axes[-1].set_xlabel("time [s]")
        self.cursor_lines = [ax.axvline(np.nan, color="red", lw=1.0, animated=True) for ax in self.axes]
        self.plot_bg = None
        if old_xlim is not None:
            self.axes[0].set_xlim(old_xlim)
        self.axes[0].callbacks.connect("xlim_changed", lambda ax: self.update_labels())
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.06, hspace=0.08)
        self.update_labels()
        self.update_cursor_lines()
        self.canvas.draw_idle()

    def update_labels(self):
        for txt in self.label_texts:
            txt.remove()
        self.label_texts = []
        if not self.axes:
            return
        ax = self.axes[0]
        x0, x1 = ax.get_xlim()
        vis = [e for e in self.shown_events if x0 <= e[1] <= x1]
        if len(vis) <= LABEL_MAX:
            by_t = {}  # 同じ行のイベントは1つの名前にまとめる
            for _, t, s in vis:
                by_t.setdefault(t, []).append(s.split(" ", 1)[0])
            for t, names in by_t.items():
                self.label_texts.append(ax.text(t, 1.0, "/".join(names), transform=ax.get_xaxis_transform(),
                                                rotation=90, fontsize=6, va="bottom", ha="center",
                                                color="tab:purple", clip_on=False))
        self.canvas.draw_idle()

    def update_cursor_lines(self):
        """カーソルの縦線を、取っておいた背景に重ねて描く(グラフ全体は描き直さない)"""
        if not self.axes or self.cursor_t is None:
            return
        for ln in self.cursor_lines:
            ln.set_xdata([self.cursor_t, self.cursor_t])
        if self.plot_bg is None:
            return
        self.canvas.restore_region(self.plot_bg)
        for ax, ln in zip(self.axes, self.cursor_lines):
            ax.draw_artist(ln)
        self.canvas.blit(self.fig.bbox)

    def on_plot_draw(self, _ev):
        if not self.axes:
            self.plot_bg = None
            return
        self.plot_bg = self.canvas.copy_from_bbox(self.fig.bbox)
        self.plot_bg_xlim = self.axes[0].get_xlim()
        self.update_cursor_lines()

    def on_plot_click(self, ev):
        if self.toolbar.mode or ev.inaxes not in self.axes or ev.xdata is None:
            return
        self.set_cursor(ev.xdata, center=False)

    # ---- 再生 ----
    def on_space(self, ev):
        if isinstance(ev.widget, (tk.Entry, ttk.Entry)):
            return  # 絞る・除くに文字を打っているとき
        self.toggle_play()
        return "break"

    def toggle_play(self):
        if self.playing:
            self.stop_play()
        else:
            self.start_play()

    def start_play(self):
        if self.log is None:
            return
        if self.cursor_t is None or self.cursor_t >= self.t[-1]:
            self.cursor_t = float(self.t[0])  # 終わりまで行っていたら最初から
        self.playing = True
        self.play_btn.config(text="⏸ 停止")
        self.play_wall = time.perf_counter()
        self.play_job = self.root.after(PLAY_FRAME_MS, self.play_tick)

    def stop_play(self):
        self.playing = False
        if self.play_job is not None:
            self.root.after_cancel(self.play_job)
            self.play_job = None
        self.play_btn.config(text="▶ 再生")

    def play_tick(self):
        self.play_job = None
        if not self.playing or self.log is None:
            return
        now = time.perf_counter()
        try:
            speed = float(self.play_speed.get())
        except ValueError:
            speed = 1.0
        t = self.cursor_t + (now - self.play_wall) * speed
        self.play_wall = now
        if t >= self.t[-1]:
            self.set_cursor(self.t[-1], follow=True)
            self.stop_play()
            return
        self.set_cursor(t, follow=True)
        self.play_job = self.root.after(PLAY_FRAME_MS, self.play_tick)

    def on_seek(self, _value):
        if self.seeking or self.log is None:
            return
        self.set_cursor(self.seek_var.get(), follow=True)
        if self.playing:
            self.play_wall = time.perf_counter()

    def set_cursor(self, t, center=False, from_list=False, follow=False):
        if self.log is None:
            return
        self.cursor_t = float(t)
        if follow and self.axes:
            # 拡大しているときは、カーソルが右の端近くまで来たら、左の端近くへ来るように送る
            x0, x1 = self.axes[0].get_xlim()
            w = x1 - x0
            if w < (self.t[-1] - self.t[0]) * 0.999 and not (x0 <= self.cursor_t <= x0 + 0.9 * w):
                lo = min(max(self.cursor_t - 0.1 * w, self.t[0]), self.t[-1] - w)
                self.axes[0].set_xlim(lo, lo + w)
        self.seeking = True
        self.seek_var.set(self.cursor_t)
        self.seeking = False
        self.seek_label.config(text=f"{self.cursor_t:7.3f} / {self.t[-1]:.2f} s")
        if center and self.axes:
            x0, x1 = self.axes[0].get_xlim()
            w = x1 - x0
            full = self.t[-1] - self.t[0]
            if w >= full * 0.999:  # まだ拡大していなければ、前後 1 秒を見る
                w = min(2.0, full)
            lo = min(max(self.cursor_t - w / 2, self.t[0]), self.t[-1] - w)  # 記録の外を見せない
            self.axes[0].set_xlim(lo, lo + w)
        if self.axes and self.axes[0].get_xlim() != getattr(self, "plot_bg_xlim", None):
            # 見ている範囲が変わった: 全部描き直す(描き終わりで背景を取り直し、カーソルも描く)
            self.plot_bg = None
            self.canvas.draw_idle()
        self.update_cursor_lines()
        self.update_maze_marker()
        self.update_status()
        if not from_list and self.shown_events:
            # 一覧で、カーソルに一番近い(前の)イベントを選ぶ
            ts = [e[1] for e in self.shown_events]
            i = max(0, int(np.searchsorted(ts, self.cursor_t, side="right")) - 1)
            self.ev_list.selection_clear(0, tk.END)
            self.ev_list.selection_set(i)
            self.ev_list.see(i)

    def update_status(self):
        i = int(np.clip(np.searchsorted(self.t, self.cursor_t), 0, len(self.t) - 1))
        parts = [f"t={self.t[i]:.3f}s"]
        for c in self.selected_columns():
            parts.append(f"{c}={self.log[c][i]:.4g}")
        texts = self.log.get("ev_text")
        if texts and texts[i]:
            parts.append(f"[{texts[i]}]")
        self.status.config(text="  ".join(parts))

    # ---- 迷路 ----
    def find_anchor(self):
        """スタート区画の真ん中・北向きとする行"""
        setpos_end = [e for e in yc.find_events(self.log, "SETPOS")
                      if yc.event_values(e[2]).get("phase", 0.0) >= 0.5]
        if len(setpos_end) >= 2:
            return setpos_end[1][0], "尻当て2回目の終わり"
        steps = yc.find_events(self.log, "STEP")
        if steps:
            return steps[0][0], "最初の STEP"
        return 0, "記録の始め"

    def rebuild_maze(self):
        ax = self.max
        ax.clear()
        self.maze_bg = None
        self.path_xy = None
        if self.log is None or "dist" not in self.log or "angle" not in self.log:
            ax.set_title("dist と angle の列がないので軌道は描けない", fontsize=9)
            self.mcanvas.draw_idle()
            return
        n = len(self.t)
        dist = np.array(self.log["dist"])
        ang = np.array(self.log["angle"])
        ds = np.diff(dist, prepend=dist[0])
        ds[ds < -5.0] = 0.0  # 制御を有効にし直して距離が 0 に戻った所
        i0, anchor_name = self.find_anchor()
        steps = [(i, yc.event_values(s)) for i, _, s in yc.find_events(self.log, "STEP")]

        # 合わせる点: (行, x, y, 向き[deg])
        anchors = [(i0, 0.5 * SECTION, 0.5 * SECTION, 90.0)]
        if self.reanchor.get():
            target = np.array(self.log["target"]) if "target" in self.log else None
            for i, v in steps:
                if i <= i0 or not all(k in v for k in ("x", "y", "heading")):
                    continue
                h = int(v["heading"])
                cx, cy = (v["x"] + 0.5) * SECTION, (v["y"] + 0.5) * SECTION
                moving = target is not None and abs(target[i]) > 10.0
                if moving:  # 境界で壁を読んだ: 区画の真ん中から、来た向きへ半区画戻った所
                    dx, dy = dir_vec(h)
                    cx -= dx * 0.5 * SECTION
                    cy -= dy * 0.5 * SECTION
                anchors.append((i, cx, cy, heading_deg(h)))

        x = np.full(n, np.nan)
        y = np.full(n, np.nan)
        th = np.full(n, np.nan)
        # 合わせる点から先へ進める(最初の点より前は、後ろへ戻して作る)
        bounds = [a[0] for a in anchors] + [n]
        for k, (ia, ax0, ay0, h0) in enumerate(anchors):
            ib = bounds[k + 1]
            off = h0 - ang[ia]
            x[ia], y[ia], th[ia] = ax0, ay0, h0
            for i in range(ia + 1, ib):
                th[i] = ang[i] + off
                a = math.radians(th[i])
                x[i] = x[i - 1] + ds[i] * math.cos(a)
                y[i] = y[i - 1] + ds[i] * math.sin(a)
        off = anchors[0][3] - ang[i0]
        for i in range(i0 - 1, -1, -1):
            th[i] = ang[i] + off
            a = math.radians(th[i + 1])
            x[i] = x[i + 1] - ds[i + 1] * math.cos(a)
            y[i] = y[i + 1] - ds[i + 1] * math.sin(a)
        self.path_xy = (x, y, th)

        # 区画の大きさ: 軌道と STEP が入るだけ
        cells_x = [v.get("x", 0) for _, v in steps] + [0]
        cells_y = [v.get("y", 0) for _, v in steps] + [0]
        fin = ~np.isnan(x)
        nx_ = int(max(max(cells_x), math.floor(np.nanmax(x[fin]) / SECTION))) + 1
        ny_ = int(max(max(cells_y), math.floor(np.nanmax(y[fin]) / SECTION))) + 1
        mx0 = min(0, int(math.floor(np.nanmin(x[fin]) / SECTION)))
        my0 = min(0, int(math.floor(np.nanmin(y[fin]) / SECTION)))
        for gx in range(mx0, nx_ + 1):
            ax.axvline(gx * SECTION, color="0.9", lw=0.6, zorder=0)
        for gy in range(my0, ny_ + 1):
            ax.axhline(gy * SECTION, color="0.9", lw=0.6, zorder=0)
        for gx in range(mx0, nx_ + 1):
            for gy in range(my0, ny_ + 1):
                ax.add_patch(Rectangle((gx * SECTION - 6, gy * SECTION - 6), 12, 12, color="0.3", zorder=1))

        # 壁(機体の地図と、カーソルの時刻までに STEP で読んだ壁)。描く範囲は軌道・STEP・地図で分かった区画
        self.maze_region = (mx0, my0, nx_, ny_)
        self.map_cells = self.parse_map_cells()
        self.step_walls = self.parse_step_walls(steps)
        self.map_known = LineCollection([], colors="0.25", linewidths=3, zorder=2, capstyle="butt")
        self.map_unknown = LineCollection([], colors="0.6", linewidths=0.8, linestyles=":", zorder=2)
        self.step_wall_lc = LineCollection([], colors="tab:red", linewidths=3, zorder=3, capstyle="butt")
        self.step_open_lc = LineCollection([], colors="tab:green", linewidths=1.0, linestyles="--", zorder=3,
                                           alpha=0.7)
        for lc in (self.map_known, self.map_unknown, self.step_wall_lc, self.step_open_lc):
            ax.add_collection(lc)
        self.update_map_layer()
        self.step_count_shown = -1
        self.update_step_layer(force=True)

        ax.plot(x, y, lw=1.0, color="tab:blue", zorder=4)
        ax.plot([x[i0]], [y[i0]], "go", ms=5, zorder=4)
        ax.set_aspect("equal")
        ax.set_xlim(mx0 * SECTION - 20, nx_ * SECTION + 20)
        ax.set_ylim(my0 * SECTION - 20, ny_ * SECTION + 20)
        ax.tick_params(labelsize=7)
        title = f"軌道 (車輪+ジャイロ、基準: {anchor_name})"
        if self.map_cells is None:
            title += "\n機体の地図はログにない"
        ax.set_title(title, fontsize=9)
        # カーソルの位置と向き(再生で速く動かすため、背景を取っておいて重ねて描く)
        self.maze_dot, = ax.plot([], [], "o", color="red", ms=7, zorder=6, animated=True)
        self.maze_dir, = ax.plot([], [], "-", color="red", lw=2, zorder=6, animated=True)
        self.maze_bg = None
        self.mcanvas.draw_idle()

    @staticmethod
    def wall_segment(cx, cy, d):
        """区画 (cx, cy) の d の向き(0 北 1 東 2 南 3 西)の壁の線分"""
        x0, y0 = cx * SECTION, cy * SECTION
        return {0: ((x0, y0 + SECTION), (x0 + SECTION, y0 + SECTION)),
                1: ((x0 + SECTION, y0), (x0 + SECTION, y0 + SECTION)),
                2: ((x0, y0), (x0 + SECTION, y0)),
                3: ((x0, y0), (x0, y0 + SECTION))}[d]

    def parse_map_cells(self):
        """LOG_EV_MAP_CELLS から機体の地図 {(x, y): 1バイト} を作る(なければ None)。
        同じ区画が何度も出たら、後のもの(走りの終わりに近いもの)を使う。"""
        need = ("ev", "ev_a", "ev_b", "ev_c", "ev_d", "ev_e")
        if any(k not in self.log for k in need):
            return None
        ev = np.array(self.log["ev"])
        rows = np.where(np.round(ev) == MAP_CELLS_EVENT)[0]
        if len(rows) == 0:
            return None
        cells = {}
        for r in rows:
            index = int(round(self.log["ev_a"][r]))
            for k, col in enumerate(("ev_b", "ev_c", "ev_d", "ev_e")):
                w = int(round(self.log[col][r]))
                for j, byte in enumerate((w & 0xFF, (w >> 8) & 0xFF)):
                    c = index + 2 * k + j
                    if c < MAP_SIZE * MAP_SIZE:
                        cells[(c % MAP_SIZE, c // MAP_SIZE)] = byte
        return cells

    def parse_step_walls(self, steps):
        """STEP で読んだ壁を [(行, 線分, ある?)] にする(前・右・左。bit0 前, bit1 右, bit2 左)。"""
        out = []
        for i, v in steps:
            if not all(k in v for k in ("x", "y", "heading", "walls")):
                continue
            cx, cy, h, w = int(v["x"]), int(v["y"]), int(v["heading"]), int(v["walls"])
            for bit, turn in ((1, 0), (2, 1), (4, -1)):
                out.append((i, self.wall_segment(cx, cy, (h + turn) % 4), bool(w & bit)))
        return out

    def update_map_layer(self):
        known, unknown = [], []
        if self.map_cells is not None and self.show_map.get():
            mx0, my0, nx_, ny_ = self.maze_region
            for (cx, cy), byte in self.map_cells.items():
                if not (mx0 <= cx < nx_ and my0 <= cy < ny_):
                    continue
                # 北0x8 東0x4 南0x2 西0x1。下位4bit 探索用(未知は「ない」)、上位4bit 最短用(未知は「ある」)
                for d, bit in ((0, 0x8), (1, 0x4), (2, 0x2), (3, 0x1)):
                    lo, hi = bool(byte & bit), bool(byte & (bit << 4))
                    if lo and hi:
                        known.append(self.wall_segment(cx, cy, d))
                    elif hi and not lo:
                        unknown.append(self.wall_segment(cx, cy, d))
        self.map_known.set_segments(known)
        self.map_unknown.set_segments(unknown)

    def update_step_layer(self, force=False):
        """カーソルの時刻までに読んだ壁を描く(カーソルがなければ全部)。変わったら True。"""
        if not self.show_steps.get():
            n = 0
        elif self.cursor_t is None:
            n = len(self.step_walls)
        else:
            i_cur = int(np.searchsorted(self.t, self.cursor_t, side="right"))
            n = sum(1 for i, _, _ in self.step_walls if i < i_cur)
        if n == self.step_count_shown and not force:
            return False
        self.step_count_shown = n
        # 同じ壁を何度も読んだら、後に読んだ結果を使う
        latest = {}
        for _, seg, exists in self.step_walls[:n]:
            latest[tuple(sorted(seg))] = (seg, exists)
        self.step_wall_lc.set_segments([seg for seg, e in latest.values() if e])
        self.step_open_lc.set_segments([seg for seg, e in latest.values() if not e])
        return True

    def on_layer_toggle(self):
        if self.path_xy is None:
            return
        self.update_map_layer()
        self.update_step_layer(force=True)
        self.mcanvas.draw_idle()

    def on_maze_draw(self, _ev):
        """迷路を描き直したら、背景を取っておき、カーソルの印を重ねる"""
        if self.path_xy is None:
            return
        self.maze_bg = self.mcanvas.copy_from_bbox(self.mfig.bbox)
        self.blit_maze_marker()

    def update_maze_marker(self):
        if self.path_xy is None:
            return
        if self.update_step_layer():
            self.mcanvas.draw_idle()  # 壁が増えた: 全部描き直す(描き終わりで印も描く)
            return
        self.blit_maze_marker()

    def blit_maze_marker(self):
        x, y, th = self.path_xy
        if self.cursor_t is None:
            self.maze_dot.set_data([], [])
            self.maze_dir.set_data([], [])
        else:
            i = int(np.clip(np.searchsorted(self.t, self.cursor_t), 0, len(self.t) - 1))
            if np.isnan(x[i]):
                self.maze_dot.set_data([], [])
                self.maze_dir.set_data([], [])
            else:
                a = math.radians(th[i])
                self.maze_dot.set_data([x[i]], [y[i]])
                self.maze_dir.set_data([x[i], x[i] + 50 * math.cos(a)], [y[i], y[i] + 50 * math.sin(a)])
        if self.maze_bg is None:
            return
        self.mcanvas.restore_region(self.maze_bg)
        self.max.draw_artist(self.maze_dir)
        self.max.draw_artist(self.maze_dot)
        self.mcanvas.blit(self.mfig.bbox)

    def on_maze_click(self, ev):
        if self.path_xy is None or ev.inaxes is not self.max or ev.xdata is None:
            return
        tb_mode = getattr(self.mcanvas.toolbar, "mode", "") if hasattr(self.mcanvas, "toolbar") else ""
        if tb_mode:
            return
        x, y, _ = self.path_xy
        d = np.hypot(x - ev.xdata, y - ev.ydata)
        if np.all(np.isnan(d)):
            return
        i = int(np.nanargmin(d))
        self.set_cursor(self.t[i], center=True)


def main():
    root = tk.Tk()
    root.geometry("1600x950")
    app = LogViewer(root)

    def on_close():
        app.stop_play()  # 再生の次のコマの予約を消してから閉じる
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    if len(sys.argv) > 1:
        root.after(100, lambda: app.open(Path(sys.argv[1])))
    else:
        root.after(100, app.open_dialog)  # 起動したらすぐログを選ぶ(あとから「開く」で選び直せる)
    root.mainloop()


if __name__ == "__main__":
    main()
