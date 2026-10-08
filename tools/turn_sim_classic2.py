# turn_sim_pygame.py  （変更済み）
import pygame
import sys
import numpy as np
from functools import partial
import math

pygame.init()
FONT_PATH = r"M:\User\Desktop\色々\動画\k-font\keifont.ttf"
FONT = pygame.font.Font(FONT_PATH, 15)
BIGFONT = pygame.font.Font(FONT_PATH, 20)

# --- world coordinate extents (match roughly the original matplotlib axes) ---
WORLD_MIN_X = -100.0
WORLD_MAX_X = 280.0
WORLD_MIN_Y = -10.0
WORLD_MAX_Y = 370.0

# --- screen layout ---
SCREEN_W = 1000
SCREEN_H = 700
CANVAS_SIZE = 540  # square drawing area
CANVAS_LEFT = 200
CANVAS_TOP = 40

# Colors
BG = (245, 245, 245)
PANEL = (230, 230, 230)
GRID_COLOR = (170, 170, 170)
PILLAR_COLOR = (200, 30, 30)
LINE_BLUE = (5, 5, 255)
LINE_GREEN = (40, 255, 40)
LINE_RED = (255, 40, 40)
GREY_LINE = (136, 136, 136)
TEXT_COLOR = (10, 10, 10)
BUTTON_COLOR = (200, 200, 200)
BUTTON_HOVER = (180, 180, 180)
INPUT_BG = (255, 255, 255)

screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("たーんしみゅれーた (pygame)")

# --- utility coordinate transform: world (mm) -> canvas pixels ---
def world_to_screen(x, y):
    """
    Map world coordinates to canvas pixel coordinates.
    world y increases upward; screen y increases downward.
    """
    sx = CANVAS_LEFT + (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X) * CANVAS_SIZE
    sy = CANVAS_TOP + CANVAS_SIZE - (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y) * CANVAS_SIZE
    return int(sx), int(sy)

# --- UI widgets (very small, self-contained) ---
class Button:
    def __init__(self, rect, text, callback):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.callback = callback
    def draw(self, surf):
        mx, my = pygame.mouse.get_pos()
        color = BUTTON_HOVER if self.rect.collidepoint((mx,my)) else BUTTON_COLOR
        pygame.draw.rect(surf, color, self.rect, border_radius=4)
        txt = FONT.render(self.text, True, TEXT_COLOR)
        surf.blit(txt, txt.get_rect(center=self.rect.center))
    def handle_event(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.collidepoint(ev.pos):
                self.callback()

class TextBox:
    def __init__(self, rect, text=""):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.active = False
        self.cursor_visible = True
        self.cursor_timer = 0.0
    def draw(self, surf):
        pygame.draw.rect(surf, INPUT_BG, self.rect)
        pygame.draw.rect(surf, (150,150,150), self.rect, 1)
        txt = FONT.render(self.text, True, TEXT_COLOR)
        surf.blit(txt, (self.rect.x+4, self.rect.y+4))
        # cursor blink
        if self.active:
            self.cursor_timer += clock.get_time()/1000.0
            if self.cursor_timer > 0.5:
                self.cursor_visible = not self.cursor_visible
                self.cursor_timer = 0.0
            if self.cursor_visible:
                cx = self.rect.x+4 + txt.get_width()+1
                pygame.draw.line(surf, TEXT_COLOR, (cx, self.rect.y+4), (cx, self.rect.y+4 + txt.get_height()))
    def handle_event(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            self.active = self.rect.collidepoint(ev.pos)
        if self.active and ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_RETURN:
                self.active = False
            elif ev.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif ev.key == pygame.K_v and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                # paste
                try:
                    import pyperclip
                    self.text += pyperclip.paste()
                except:
                    pass
            else:
                # allow numbers, dot, minus
                char = ev.unicode
                if char:
                    self.text += char

# --- simulation global defaults (from original tkinter code) ---
ini_x = 0.0
ini_y = 90.0
ini_angle = 0.0
fin_angle = 90.0

# Create UI elements
buttons = []
textboxes = {}

# parameter labels and defaults (same names as tkinter fields)
params = [
    ("速度(mm/s)", "300", "Set_Speed"),
    ("最大角速度 °/s", "200", "Set_low_AngVel"),
    ("角加速度", "1000", "Set_Low_AngAcl"),
    ("入口オフセット(mm)", "90", "Set_pri_offset"),
    ("出口オフセット(mm)", "55", "Set_post_offset"),
    ("スリップアングル係数", "0.002", "Set_K_SP"),
    ("機体の横幅(mm)", "86", "Set_Width")
]

# place textboxes on right panel
start_y = 60
for i, (label, default, key) in enumerate(params):
    y = start_y + i*44
    textboxes[key] = TextBox((CANVAS_LEFT + CANVAS_SIZE + 24, y+18, 140, 28), default)

# angle preset callbacks
def set_angle_preset(foge):
    global ini_x, ini_y, ini_angle, fin_angle
    if foge == 90:
        ini_x = 0.0; ini_y = 90.0; ini_angle = 0.0; fin_angle = 90.0
    elif foge == 180:
        ini_x = 0.0; ini_y = 90.0; ini_angle = 0.0; fin_angle = 180.0
    elif foge == 45:
        ini_x = 0.0; ini_y = 90.0; ini_angle = 0.0; fin_angle = 45.0
    elif foge == 135:
        ini_x = 0.0; ini_y = 90.0; ini_angle = 0.0; fin_angle = 135.0
    elif foge == 91:
        ini_x = 0.0; ini_y = 180.0; ini_angle = 45.0; fin_angle = 135.0
    elif foge == 46:
        ini_x = 0.0; ini_y = 180.0; ini_angle = 45.0; fin_angle = 90.0
    elif foge == 136:
        ini_x = 0.0; ini_y = 180.0; ini_angle = 45.0; fin_angle = 180.0

# create preset buttons on left
preset_list = [("90°", 90), ("180°", 180), ("入45°", 45), ("出45°", 46), ("V90°", 91), ("入135°", 135), ("出135°", 136)]
for i, (label, val) in enumerate(preset_list):
    b = Button((24, 40 + i*46, 140, 36), label, partial(set_angle_preset, val))
    buttons.append(b)

# DrawTrace will be called to render the trace onto a surface
def parse_float(s, fallback=0.0):
    try:
        return float(s)
    except:
        return fallback

def draw_canvas(surface):
    """Draw grid, slanted lines and pillars (approximate original)"""
    # background of canvas
    pygame.draw.rect(surface, (255,255,255), (CANVAS_LEFT, CANVAS_TOP, CANVAS_SIZE, CANVAS_SIZE))
    # grid lines: verticals at x = -90,0,90,180,270 ; horizontals at 0,90,180,270,360
    verticals = [-90.0, 0.0, 90.0, 180.0, 270.0]
    horizontals = [0.0, 90.0, 180.0, 270.0, 360.0]
    for x in verticals:
        p1 = world_to_screen(x, 0.0)
        p2 = world_to_screen(x, 360.0)
        pygame.draw.line(surface, GRID_COLOR, p1, p2, 1)
    for y in horizontals:
        p1 = world_to_screen(-90.0, y)
        p2 = world_to_screen(270.0, y)
        pygame.draw.line(surface, GRID_COLOR, p1, p2, 1)
    # slanted lines (right-up)
    segs = [
        ((0.0,360.0),(-90.0,270.0)),
        ((180.0,360.0),(-90.0,90.0)),
        ((270.0,270.0),(0.0,0.0)),
        ((270.0,90.0),(180.0,0.0)),
        # right-down
        ((180.0,360.0),(270.0,270.0)),
        ((0.0,360.0),(270.0,90.0)),
        ((-90.0,270.0),(180.0,0.0)),
        ((-90.0,90.0),(0.0,0.0)),
    ]
    for (ax,ay),(bx,by) in segs:
        pygame.draw.line(surface, GRID_COLOR, world_to_screen(ax,ay), world_to_screen(bx,by), 1)
    # pillars (12x12 mm) drawn as red rects; original used centers near -90..264 etc.
    pillar_centers = [
        (-90, 0), (-90, 180), (-90, 360),
        (84, 0), (84, 180), (84, 360),
        (264, 0), (264, 180), (264, 360)
    ]
    # original rectangles used coords like (-96,-6,-84,6) etc -> pillar size 12x12
    half = 6.0
    for cx, cy in pillar_centers:
        x1 = cx - half
        y1 = cy - half
        x2 = cx + half
        y2 = cy + half
        p_tl = world_to_screen(x1, y2)  # note y flipped
        p_br = world_to_screen(x2, y1)
        rect = pygame.Rect(p_tl, (p_br[0]-p_tl[0], p_br[1]-p_tl[1]))
        pygame.draw.rect(surface, PILLAR_COLOR, rect)

# --- 追加: グローバル変数で軌道を保持 ---
trace_segments = []
# ここに表示用情報行を格納する（on_generate が書き換える）
info_lines = []

def draw_trace(surface):
    """trace_segments に保存された軌道を描画"""
    for (x1, y1), (x2, y2), color in trace_segments:
        p1 = world_to_screen(x1, y1)
        p2 = world_to_screen(x2, y2)
        if color == GREY_LINE:
            pygame.draw.line(surface, color, p1, p2, 2)
        else:
            pygame.draw.line(surface, color, p1, p2, 2)


def on_generate():
    global lines, info_lines, trace_segments

    # 入力値を取得
    speed = parse_float(textboxes["Set_Speed"].text, 300.0)          # [mm/s]
    low_AngVel = parse_float(textboxes["Set_low_AngVel"].text, 200.0)  # [deg/s]
    Low_AngAcl = parse_float(textboxes["Set_Low_AngAcl"].text, 1000.0)   # [deg/s^2]
    pri_offset = parse_float(eval(textboxes["Set_pri_offset"].text), 90.0)
    post_offset = parse_float(eval(textboxes["Set_post_offset"].text), 55.0)
    K_slip_angle = parse_float(textboxes["Set_K_SP"].text, 0.002)
    Width = parse_float(textboxes["Set_Width"].text, 86.0)

    # 時間ステップ [s]
    dt = 0.001  # 1 ms

    # 1ステップごとの並進距離 [mm]
    step_dist = speed * dt

    # 各パラメータ
    ang_acc = Low_AngAcl            # [deg/s^2]
    ang_vel_max = low_AngVel        # [deg/s]
    v_m_s = speed / 1000.0          # [m/s] for slip calc

    # 初期状態
    befor_x, befor_y = 0.0, 0.0
    now_AngVel = 0.0
    now_angle = ini_angle
    lines = []

    pi = np.pi
    fin_x = ini_x + pri_offset * np.sin(np.deg2rad(ini_angle))
    fin_y = ini_y + pri_offset * np.cos(np.deg2rad(ini_angle))
    lines.append(((ini_x, ini_y), (fin_x, fin_y), LINE_RED))
    lines.append(((ini_x + (Width/2)*np.cos(np.deg2rad(ini_angle)),
                       ini_y - (Width/2)*np.sin(np.deg2rad(ini_angle))),
                      (fin_x + (Width/2)*np.cos(np.deg2rad(ini_angle)),
                       fin_y - (Width/2)*np.sin(np.deg2rad(ini_angle))), GREY_LINE))
    lines.append(((ini_x - (Width/2)*np.cos(np.deg2rad(ini_angle)),
                       ini_y + (Width/2)*np.sin(np.deg2rad(ini_angle))),
                      (fin_x - (Width/2)*np.cos(np.deg2rad(ini_angle)),
                       fin_y + (Width/2)*np.sin(np.deg2rad(ini_angle))), GREY_LINE))


    dist = 0
    # --- 加速フェーズ ---
    while now_AngVel < ang_vel_max:
        befor_x, befor_y = fin_x, fin_y

        now_AngVel += ang_acc * dt
        if now_AngVel > ang_vel_max:
            now_AngVel = ang_vel_max
        now_angle += now_AngVel * dt

        # スリップ角度補正
        omega_rad = np.deg2rad(now_AngVel)  # rad/s
        s_now_angle = now_angle - np.rad2deg(K_slip_angle * v_m_s * omega_rad)

        fin_x = befor_x + step_dist * np.sin(np.deg2rad(s_now_angle))
        fin_y = befor_y + step_dist * np.cos(np.deg2rad(s_now_angle))
        lines.append(((befor_x, befor_y), (fin_x, fin_y), LINE_BLUE))
        lines.append(((befor_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))
        lines.append(((befor_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))
        dist += step_dist

    acc_dist = round(dist, 3)
    # print(f"acc:    {round(dist, 3)}")

    # --- 等角速度フェーズ ---
    dist = 0
    target_angle = fin_angle - now_angle
    while now_angle < fin_angle - ang_vel_max**2 / (2 * ang_acc):
        now_angle += now_AngVel * dt

        omega_rad = np.deg2rad(now_AngVel)
        s_now_angle = now_angle - np.rad2deg(K_slip_angle * v_m_s * omega_rad)

        fin_x = befor_x + step_dist * np.sin(np.deg2rad(s_now_angle))
        fin_y = befor_y + step_dist * np.cos(np.deg2rad(s_now_angle))

        lines.append(((befor_x, befor_y), (fin_x, fin_y), LINE_GREEN))

        lines.append(((befor_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))
        lines.append(((befor_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))

        dist += step_dist

        befor_x, befor_y = fin_x, fin_y
    const_dist = round(dist, 3)
    # print(f"const:  {round(dist,3)}")

    # --- 減速フェーズ ---
    while now_AngVel > 0.0:
        now_AngVel -= ang_acc * dt
        if now_AngVel < 0.0:
            now_AngVel = 0.0
        now_angle += now_AngVel * dt

        omega_rad = np.deg2rad(now_AngVel)
        s_now_angle = now_angle - np.rad2deg(K_slip_angle * v_m_s * omega_rad)

        fin_x = befor_x + step_dist * np.sin(np.deg2rad(s_now_angle))
        fin_y = befor_y + step_dist * np.cos(np.deg2rad(s_now_angle))

        lines.append(((befor_x, befor_y), (fin_x, fin_y), LINE_BLUE))

        lines.append(((befor_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x + (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y - (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))
        lines.append(((befor_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           befor_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))),
                          (fin_x - (Width/2)*np.cos(np.deg2rad(s_now_angle)),
                           fin_y + (Width/2)*np.sin(np.deg2rad(s_now_angle))), GREY_LINE))


        befor_x, befor_y = fin_x, fin_y

    # --- 直進フェーズ（後方オフセット） ---
    befor_x = fin_x; befor_y = fin_y
    fin_x = befor_x + post_offset * np.sin(np.deg2rad(fin_angle))
    fin_y = befor_y + post_offset * np.cos(np.deg2rad(fin_angle))
    lines.append(((befor_x, befor_y), (fin_x, fin_y), LINE_RED))
    lines.append(((befor_x + (Width/2)*np.cos(np.deg2rad(fin_angle)),
                    befor_y - (Width/2)*np.sin(np.deg2rad(fin_angle))),
                    (fin_x + (Width/2)*np.cos(np.deg2rad(fin_angle)),
                    fin_y - (Width/2)*np.sin(np.deg2rad(fin_angle))), GREY_LINE))
    lines.append(((befor_x - (Width/2)*np.cos(np.deg2rad(fin_angle)),
                    befor_y + (Width/2)*np.sin(np.deg2rad(fin_angle))),
                    (fin_x - (Width/2)*np.cos(np.deg2rad(fin_angle)),
                    fin_y + (Width/2)*np.sin(np.deg2rad(fin_angle))), GREY_LINE))

    befor_x, befor_y = fin_x, fin_y

    # --- prepare info lines (these replace previous prints) ---
    total_segments = len(lines)
    total_length = 0.0
    seg_lengths = []
    for (ax,ay),(bx,by),col in lines:
        ln = math.hypot(bx-ax, by-ay)
        seg_lengths.append(round(ln, 3))
        total_length += ln
    total_length = round(total_length, 3)

    info_lines = [
        f"acc   : {acc_dist} mm",
        f"const : {const_dist} mm",
        f"total : {total_length} mm",
    ]

    # 保存
    trace_segments = lines


gen_button = Button((CANVAS_LEFT + CANVAS_SIZE - 120, CANVAS_TOP + CANVAS_SIZE + 8, 120, 34), "軌道生成", on_generate)
buttons.append(gen_button)

# Prepare background surfaces
canvas_surface = pygame.Surface((SCREEN_W, SCREEN_H))
canvas_surface_dirty = True  # initially need to draw

# clock
clock = pygame.time.Clock()

# main loop
running = True
while running:
    dt = clock.tick(60)
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            running = False
        if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
            running = False
        # forward events to widgets
        for tb in textboxes.values():
            tb.handle_event(ev)
        for b in buttons:
            b.handle_event(ev)

    # draw UI frame
    screen.fill(BG)
    # left panel
    pygame.draw.rect(screen, PANEL, (12, 12, 176, SCREEN_H-24))
    # right panel
    pygame.draw.rect(screen, PANEL, (CANVAS_LEFT + CANVAS_SIZE + 12, 12, SCREEN_W - (CANVAS_LEFT + CANVAS_SIZE + 24), SCREEN_H-24))

    # draw preset buttons
    for b in buttons:
        b.draw(screen)

    # draw labels and textboxes
    # title
    title = BIGFONT.render("たーんしみゅれーた (pygame)", True, TEXT_COLOR)
    screen.blit(title, (CANVAS_LEFT, 6))

    for i, (label, default, key) in enumerate(params):
        y = 60 + i*44
        lab = FONT.render(label, True, TEXT_COLOR)
        screen.blit(lab, (CANVAS_LEFT + CANVAS_SIZE + 24, y))
        textboxes[key].draw(screen)

    # draw canvas背景
    pygame.draw.rect(screen, (200,200,200), (CANVAS_LEFT-2, CANVAS_TOP-2, CANVAS_SIZE+4, CANVAS_SIZE+4), border_radius=4)
    draw_canvas(screen)

    # 軌道を毎フレーム描画
    draw_trace(screen)

    # draw info_lines on right panel (below text boxes)
    info_x = CANVAS_LEFT + CANVAS_SIZE + 24
    info_y = start_y + len(params)*44 + 8  # place below the last textbox
    line_h = 18
    for i, line in enumerate(info_lines):
        # wrap long lines naively to panel width
        txt = FONT.render(line, True, TEXT_COLOR)
        screen.blit(txt, (info_x, info_y + i*line_h))

    # draw footer instructions
    inst = FONT.render("テキストボックスはクリックして入力。軌道生成で計算・描画します。ESCで終了。", True, TEXT_COLOR)
    screen.blit(inst, (CANVAS_LEFT, CANVAS_TOP + CANVAS_SIZE + 48))

    pygame.display.flip()

pygame.quit()
sys.exit()
