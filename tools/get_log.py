#!/usr/bin/env python3
"""yuho の Logger (Core/Src/app/logger.c) が送るログと、SD_DUMP モード
(Core/Src/app/sd_dump.c) が送る SD カードのファイルを受け取り、保存する。

SD_DUMP の形式 (SD 上のパスのまま logs/<path> に保存。同名があれば _dupN を付ける):
    FILE_START
    <path>
    SIZE:<bytes>
    <ファイルの中身>
    FILE_END            (FILE_ERROR なら SD 側で読めなかったので保存しない)

Logger の形式 (divergence_v3 と同じ):
    BIN_START
    <dir>
    <file>              (空なら "log")
    TIMESTAMP:<0|1>     (1 ならファイル名に日時を付ける)
    SIZE:<bytes>
    <name0>,<name1>,...
    <float32 (little endian) x 列数 x 行数 の生データ>
    BIN_END

使い方:
    pip install pyserial matplotlib
    python tools/get_log.py COM5            # 受信して logs/<dir>/<file>_<日時>.csv に保存し続ける
    python tools/get_log.py COM5 --plot     # 受信するたびにグラフも表示
    python tools/get_log.py --plot-file logs/vel_pid/step_20261003_120000.csv  # 保存済みCSVを表示
    python tools/get_log.py --bin2csv logs/search/search_0008.bin  # 保存済みのバイナリログをCSVにする

SD の .bin (探索など、走りながら追記したバイナリログ。形式は parse_ylog) は、受け取ると
隣に同じ名前の .csv も作る。

ログ以外の行 (printf の出力) はそのまま画面に表示する。
UART を他のターミナルソフトで開いていると受信できないので、閉じてから使うこと。
"""

import argparse
import csv
import datetime
import re
import struct
import sys
from pathlib import Path

# グラフでまとめて表示する列の組 (先頭から順に1段ずつ)
PLOT_GROUPS = [
    ("velocity [mm/s]", ["target", "vl", "vr", "vl_ref", "vr_ref"]),
    ("accel [mm/s^2]", ["target_acc"]),
    ("position [mm]", ["pos_ref", "x_mm"]),
    ("angular [dps]", ["omega_ref", "gyro_z", "ang_corr"]),
    ("angle [deg]", ["angle_ref", "angle"]),
    ("pwm", ["pwm_l", "pwm_r"]),
    ("voltage term [V]", ["ff_l", "ff_r", "i_l", "i_r"]),
    ("vbat [V]", ["vbat"]),
]
TIME_COLUMN = "time_s"


def safe_name(name: str, default: str) -> str:
    """PC 側のパスに使えるよう、英数字・_・-・. 以外を _ に置き換える ('..' は使わせない)。"""
    name = name.strip()
    if not name:
        return default
    name = re.sub(r"[^A-Za-z0-9_.\-]", "_", name)
    if name in (".", ".."):
        return default if name == ".." else name
    return name


def read_line(ser) -> str:
    """1行読む (タイムアウトなら空文字)。CRLF は取り除く。"""
    raw = ser.readline()
    return raw.decode("ascii", errors="replace").rstrip("\r\n")


def read_exact(ser, size: int) -> bytes:
    """size バイトそろうまで読む。途中で長く止まったら例外。"""
    buf = bytearray()
    idle = 0
    while len(buf) < size:
        chunk = ser.read(size - len(buf))
        if chunk:
            buf.extend(chunk)
            idle = 0
        else:
            idle += 1
            if idle >= 5:
                raise TimeoutError(f"binary body stalled at {len(buf)}/{size} bytes")
    return bytes(buf)


def receive_table(ser):
    """BIN_START の直後から BIN_END までを読み、(dir, file, timestamp, names, rows) を返す。"""
    dir_name = read_line(ser)
    file_name = read_line(ser)

    ts_line = read_line(ser)
    if not ts_line.startswith("TIMESTAMP:"):
        raise ValueError(f"expected TIMESTAMP:, got {ts_line!r}")
    timestamp = ts_line.split(":", 1)[1].strip() == "1"

    size_line = read_line(ser)
    if not size_line.startswith("SIZE:"):
        raise ValueError(f"expected SIZE:, got {size_line!r}")
    size = int(size_line.split(":", 1)[1])

    names = [n for n in read_line(ser).split(",") if n]
    if not names:
        raise ValueError("no column names")

    body = read_exact(ser, size)

    end_line = read_line(ser)
    if end_line != "BIN_END":
        print(f"[warn] expected BIN_END, got {end_line!r}", file=sys.stderr)

    row_bytes = 4 * len(names)
    if size % row_bytes != 0:
        raise ValueError(f"size {size} is not a multiple of {row_bytes} ({len(names)} columns)")

    n_rows = size // row_bytes
    values = struct.unpack(f"<{n_rows * len(names)}f", body)
    rows = [values[i * len(names):(i + 1) * len(names)] for i in range(n_rows)]
    return dir_name, file_name, timestamp, names, rows


def receive_file(ser):
    """FILE_START の直後から FILE_END/FILE_ERROR までを読み、(path, data, ok) を返す。"""
    path = read_line(ser)
    size_line = read_line(ser)
    if not size_line.startswith("SIZE:"):
        raise ValueError(f"expected SIZE:, got {size_line!r}")
    size = int(size_line.split(":", 1)[1])
    data = read_exact(ser, size)
    end_line = read_line(ser)
    if end_line not in ("FILE_END", "FILE_ERROR"):
        print(f"[warn] expected FILE_END, got {end_line!r}", file=sys.stderr)
    return path, data, end_line == "FILE_END"


def save_file(out_root: Path, path: str, data: bytes):
    """SD 上のパス (例: straight/trapezoid_0001.csv) を out_root の下に同じ構成で保存する。
    同じ名前(または _dupN)で中身も同じファイルがあれば保存せず None を返す。
    同名で中身が違えば上書きせず _dup1, _dup2 ... を付ける。"""
    parts = [safe_name(p, "_") for p in path.replace("\\", "/").split("/") if p not in ("", ".", "..")]
    if not parts:
        parts = ["unnamed"]
    dest = out_root.joinpath(*parts)
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        stem, suffix = dest.stem, dest.suffix
        candidates = [dest] + sorted(dest.parent.glob(f"{stem}_dup*{suffix}"))
        if any(c.read_bytes() == data for c in candidates):
            return None
        i = 1
        while (dest.parent / f"{stem}_dup{i}{suffix}").exists():
            i += 1
        dest = dest.parent / f"{stem}_dup{i}{suffix}"
    dest.write_bytes(data)
    return dest


YLOG_MAGIC = b"YLOG1"


def parse_ylog(data: bytes):
    """探索などの追記用のバイナリログ (logger.c の LOG_BIN_MAGIC) を (names, rows) にする。
    形式: "YLOG1\\n<列数>\\n<列名,...>\\n" + float32 (little endian) x 列数 x 行数。
    最後の行が途中で切れていれば (書いている途中で電源が切れたなど) 捨てる。"""
    lines = data.split(b"\n", 3)
    if len(lines) < 4 or lines[0] != YLOG_MAGIC:
        raise ValueError("not a YLOG1 file")
    count = int(lines[1])
    # 列名の行は、データの書き始めをセクタの区切りにそろえるため後ろが空白で埋めてある
    names = lines[2].decode("ascii").strip().split(",")
    if len(names) != count:
        raise ValueError(f"header says {count} columns but has {len(names)} names")
    body = lines[3]
    row_bytes = 4 * count
    n_rows = len(body) // row_bytes
    values = struct.unpack_from(f"<{n_rows * count}f", body)
    rows = [values[r * count:(r + 1) * count] for r in range(n_rows)]
    return names, rows


def write_csv(path: Path, names, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(names)
        for row in rows:
            writer.writerow([f"{v:.6g}" for v in row])


def ylog_to_csv(bin_path: Path) -> Path:
    """.bin の隣に同じ名前の .csv を作る (あれば上書き)。"""
    names, rows = parse_ylog(bin_path.read_bytes())
    csv_path = bin_path.with_suffix(".csv")
    write_csv(csv_path, names, rows)
    return csv_path


def save_csv(out_root: Path, dir_name, file_name, timestamp, names, rows) -> Path:
    out_dir = out_root / safe_name(dir_name, ".")
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = safe_name(file_name, "log")
    if timestamp:
        stem += "_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"{stem}.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(names)
        for row in rows:
            writer.writerow([f"{v:.6g}" for v in row])
    return path


def plot_table(names, rows, title=""):
    import matplotlib.pyplot as plt

    columns = {name: [row[i] for row in rows] for i, name in enumerate(names)}
    t = columns.get(TIME_COLUMN, list(range(len(rows))))

    used = {TIME_COLUMN}
    groups = []
    for label, cols in PLOT_GROUPS:
        present = [c for c in cols if c in columns]
        if present:
            groups.append((label, present))
            used.update(present)
    others = [n for n in names if n not in used]
    if others:
        groups.append(("other", others))

    fig, axes = plt.subplots(len(groups), 1, sharex=True, figsize=(10, 2.4 * len(groups)), squeeze=False)
    for ax, (label, cols) in zip(axes[:, 0], groups):
        for c in cols:
            ax.plot(t, columns[c], label=c)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize="small")
    axes[-1, 0].set_xlabel(TIME_COLUMN)
    fig.suptitle(title)
    fig.tight_layout()
    plt.show()


def plot_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        names = next(reader)
        rows = [[float(v) for v in row] for row in reader if row]
    plot_table(names, rows, title=str(path))


def run_receiver(args):
    import serial  # pyserial

    out_root = Path(args.out)
    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        print(f"listening on {args.port} @ {args.baud} (Ctrl+C to quit)")
        while True:
            line = read_line(ser)
            if not line:
                continue
            if line == "FILE_START":
                try:
                    path, data, ok = receive_file(ser)
                except (ValueError, TimeoutError) as e:
                    print(f"[error] broken file: {e}", file=sys.stderr)
                    continue
                if not ok:
                    print(f"[error] {path}: read error on the SD side, not saved", file=sys.stderr)
                    continue
                dest = save_file(out_root, path, data)
                if dest is None:
                    print(f"[skip] {path} (already received, same content)")
                    continue
                print(f"[saved] {dest} ({len(data)} bytes)")
                if dest.suffix.lower() == ".bin":
                    # 追記用のバイナリログは、読めるように隣に CSV も作る
                    try:
                        csv_path = ylog_to_csv(dest)
                        print(f"[converted] {csv_path}")
                        dest = csv_path
                    except (ValueError, struct.error) as e:
                        print(f"[warn] {dest}: could not convert to CSV ({e})", file=sys.stderr)
                if args.plot and dest.suffix.lower() == ".csv":
                    plot_csv(dest)
                continue
            if line != "BIN_START":
                print(line)
                continue
            try:
                dir_name, file_name, timestamp, names, rows = receive_table(ser)
            except (ValueError, TimeoutError) as e:
                print(f"[error] broken log: {e}", file=sys.stderr)
                continue
            path = save_csv(out_root, dir_name, file_name, timestamp, names, rows)
            print(f"[saved] {path} ({len(rows)} rows x {len(names)} cols)")
            if args.plot:
                plot_table(names, rows, title=str(path))
            if args.once:
                return


def main():
    parser = argparse.ArgumentParser(description="Receive yuho logger dumps over UART and save them as CSV.")
    parser.add_argument("port", nargs="?", help="serial port (e.g. COM5, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="logs", help="output root directory (default: logs)")
    parser.add_argument("--plot", action="store_true", help="show a plot after each received log")
    parser.add_argument("--once", action="store_true", help="exit after receiving one log")
    parser.add_argument("--plot-file", type=Path, help="plot an existing CSV and exit")
    parser.add_argument("--bin2csv", type=Path, nargs="+", metavar="BIN",
                        help="convert saved binary logs (.bin, e.g. search) to CSV next to them and exit")
    args = parser.parse_args()

    if args.bin2csv:
        for p in args.bin2csv:
            print(f"[converted] {ylog_to_csv(p)}")
        return
    if args.plot_file:
        plot_csv(args.plot_file)
        return
    if not args.port:
        parser.error("port is required unless --plot-file or --bin2csv is given")
    try:
        run_receiver(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
