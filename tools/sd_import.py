#!/usr/bin/env python3
"""PC に挿した SD カードから yuho のログを logs/ に取り込み、.bin の隣に .csv も作る。

SD の中身 (Core/Src/interface/sdcard.c):
    <dir>/<prefix>_NNNN.bin        まだ UART で送っていないログ (古いプログラムのものは .csv)
    sent/<dir>/<prefix>_NNNN.bin   UART で送った (SD_DUMP) ログ
どちらも logs/<dir>/<file> に置く (sent/ は外す。get_log.py で受け取ったときと同じ場所)。

- 同じ名前で中身も同じファイルが logs/ にあれば取り込まない (何度実行してもよい)。
- 同じ名前で中身が違えば、上書きせず _dup1, _dup2 ... を付ける (get_log.save_file と同じ)。
- .bin は隣に同じ名前の .csv を作る (get_log.ylog_to_csv と同じ。ev_text 付き)。
- SD のファイルは消さない・動かさない (読むだけ)。

使い方:
    python tools/sd_import.py              # 挿してある SD カードを探して取り込む
    python tools/sd_import.py D:           # ドライブを決める
    python tools/sd_import.py --wait       # SD カードが挿されるまで待ってから取り込む
    python tools/sd_import.py --dry-run    # 何を取り込むかを表示するだけ
"""

import argparse
import re
import string
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import get_log  # tools/get_log.py

ROOT = Path(__file__).resolve().parent.parent
SENT_DIR = "sent"
LOG_EXTS = (".bin", ".csv")
LOG_NAME = re.compile(r"^.+_\d{4}\.(bin|csv)$", re.I)  # sdcard.c の "%s/%s_%04lu.%s"
SKIP_DIRS = {"system volume information", "$recycle.bin"}


def removable_drives():
    """Windows の取り外しできるドライブ (SD カード・USB メモリ) の一覧 ("D:\\" など)。"""
    if sys.platform != "win32":
        return []
    import ctypes
    k32 = ctypes.windll.kernel32
    mask = k32.GetLogicalDrives()
    out = []
    for i, letter in enumerate(string.ascii_uppercase):
        if mask & (1 << i):
            root = f"{letter}:\\"
            if k32.GetDriveTypeW(root) == 2:  # DRIVE_REMOVABLE
                out.append(root)
    return out


def find_logs(sd: Path):
    """SD のログを [(SD 上のパス, logs/ の下に置くパス), ...] で返す (<dir>/ と sent/<dir>/ の1段下だけ見る)。"""
    found = []

    def scan(base: Path, prefix: str):
        try:
            dirs = sorted(p for p in base.iterdir() if p.is_dir())
        except OSError:
            return
        for d in dirs:
            if d.name.lower() in SKIP_DIRS or d.name.startswith(".") or (not prefix and d.name == SENT_DIR):
                continue
            try:
                files = sorted(f for f in d.iterdir() if f.is_file())
            except OSError:
                continue
            for f in files:
                if f.suffix.lower() in LOG_EXTS and LOG_NAME.match(f.name):
                    found.append((f, f"{d.name}/{f.name}"))

    scan(sd, "")
    scan(sd / SENT_DIR, SENT_DIR)
    return found


def find_sd(drive=None):
    """取り込む SD を決める。決まらなければ (None, 理由)。"""
    if drive:
        p = Path(drive if not re.fullmatch(r"[A-Za-z]:?", drive) else drive[0] + ":\\")
        if not p.exists():
            return None, f"{drive} が見つからない (SD カードは挿してある?)"
        return p, None
    cands = [Path(d) for d in removable_drives()]
    with_logs = []
    for d in cands:
        try:
            if find_logs(d):
                with_logs.append(d)
        except OSError:
            pass  # カードの入っていないカードリーダーなど
    if len(with_logs) == 1:
        return with_logs[0], None
    if len(with_logs) > 1:
        return None, "ログのあるドライブが複数ある: " + ", ".join(str(d) for d in with_logs) + " (ドライブを指定して)"
    if cands:
        return None, "取り外しできるドライブ (" + ", ".join(str(d) for d in cands) + ") にログが見つからない"
    return None, "SD カードが見つからない"


def import_logs(sd: Path, out_root: Path, dry_run=False):
    logs = find_logs(sd)
    print(f"SD: {sd}  ログ {len(logs)} 個 → {out_root}")
    n_new = n_skip = n_csv = n_err = 0
    for src, rel in logs:
        try:
            data = src.read_bytes()
        except OSError as e:
            print(f"[error] {src}: 読めない ({e})", file=sys.stderr)
            n_err += 1
            continue
        if dry_run:
            dest = out_root / rel  # save_file と同じく、同じ名前と _dupN の中に同じ中身があれば飛ばす
            cands = [dest] + sorted(dest.parent.glob(f"{dest.stem}_dup*{dest.suffix}"))
            same = any(c.exists() and c.read_bytes() == data for c in cands)
            print(f"[{'skip' if same else 'new '}] {rel} ({len(data)} bytes)")
            continue
        dest = get_log.save_file(out_root, rel, data)
        if dest is None:
            n_skip += 1
            dest = out_root / rel  # 前に取り込んだもの (.csv がなければ作る)
            if dest.suffix.lower() != ".bin" or dest.with_suffix(".csv").exists():
                continue
        else:
            n_new += 1
            print(f"[saved] {dest.relative_to(out_root)} ({len(data)} bytes)")
        if dest.suffix.lower() == ".bin":
            try:
                csv_path = get_log.ylog_to_csv(dest)
                n_csv += 1
                print(f"   → {csv_path.name} ({get_log.ylog_summary(dest)})")
            except (ValueError, struct.error) as e:
                print(f"[warn] {dest.name}: CSV にできなかった ({e})", file=sys.stderr)
                n_err += 1
    if not dry_run:
        print(f"新しく取り込んだ {n_new} 個、取り込み済みで飛ばした {n_skip} 個、CSV を作った {n_csv} 個"
              + (f"、失敗 {n_err} 個" if n_err else ""))
    return n_err == 0


def main():
    ap = argparse.ArgumentParser(description="Import yuho logs from an SD card inserted in the PC.")
    ap.add_argument("drive", nargs="?", help="SD カードのドライブ (例: D:)。省くと探す")
    ap.add_argument("--out", default=str(ROOT / "logs"), help="取り込む先 (既定: リポジトリの logs/)")
    ap.add_argument("--wait", action="store_true", help="SD カードが見つかるまで待つ")
    ap.add_argument("--dry-run", action="store_true", help="取り込むものを表示するだけ")
    args = ap.parse_args()

    sd, why = find_sd(args.drive)
    if sd is None and args.wait:
        print("SD カードを待っている... (Ctrl+C でやめる)")
        try:
            while sd is None:
                time.sleep(1.0)
                sd, why = find_sd(args.drive)
        except KeyboardInterrupt:
            return 1
    if sd is None:
        print(f"[error] {why}", file=sys.stderr)
        return 1
    ok = import_logs(sd, Path(args.out), args.dry_run)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
