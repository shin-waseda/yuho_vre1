"""フォルダにある迷路図(画像)を、まとめて classic 形式の迷路ファイルにする。

使い方:
    python tools/maze_sim/maze_batch.py 迷路図のフォルダ
        → tools/maze_sim/my_mazes/<画像の名前>.txt を作る(すでにあるものは飛ばす)
        → 確認用の画像 tools/maze_sim/build/<名前>_check.png
        → 結果の一覧 tools/maze_sim/build/maze_batch_report.csv

オプション:
    --out DIR         保存先(既定 tools/maze_sim/my_mazes)
    --force           変換済みのものも作り直す
    --no-auto-rotate  向きを自動で決めない(全部そのままの向きで読む)
    --threshold T     壁とみなす途切れのなさ(既定は図ごとに自動で決める)
    --invert          白い壁・黒い背景の図のとき(全部に適用)

対応する画像: png, jpg, jpeg, bmp, gif, tif, tiff, webp(PDF は pip install pymupdf を入れれば読む)

結果の状態:
    OK     気になる点なし
    CHECK  確認が必要(自信のない壁がある、外周が欠けている、向きが決まらない など)
    ERROR  読み取れなかった(外枠が見つからない など)。maze_from_image.py で1枚ずつ --crop 等を試す
    SKIP   変換済み(--force で作り直す)

CHECK のものは、確認用の画像を見て、必要なら maze_from_image.py で1枚ずつ作り直す:
    python tools/maze_sim/maze_from_image.py 画像 --name 名前 --rotate 90 --force
"""

import argparse
import csv
import os
import re
import sys

import maze_from_image as mfi

IMAGE_EXT = (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tif", ".tiff", ".webp", ".pdf")
REPORT_PATH = os.path.join(mfi.CHECK_DIR, "maze_batch_report.csv")


def safe_name(stem):
    """ファイル名に使えない文字と空白を _ にする(日本語はそのまま)"""
    name = re.sub(r'[\\/:*?"<>|\s]+', "_", stem).strip("._")
    return name or "maze"


def main():
    p = argparse.ArgumentParser(description="Convert all maze diagram images in a folder to classic maze files")
    p.add_argument("folder")
    p.add_argument("--out", default=mfi.DEFAULT_OUT)
    p.add_argument("--force", action="store_true")
    p.add_argument("--no-auto-rotate", action="store_true")
    p.add_argument("--threshold", type=float, default=None)
    p.add_argument("--invert", action="store_true")
    args = p.parse_args()

    if not os.path.isdir(args.folder):
        print(f"not a folder: {args.folder}", file=sys.stderr)
        return 2
    images = sorted(f for f in os.listdir(args.folder) if f.lower().endswith(IMAGE_EXT))
    if not images:
        print(f"no images in {args.folder}", file=sys.stderr)
        return 2

    rotate = 0 if args.no_auto_rotate else "auto"
    rows = []
    counts = {"OK": 0, "CHECK": 0, "ERROR": 0, "SKIP": 0}
    used_names = set()

    for i, fname in enumerate(images, 1):
        name = safe_name(os.path.splitext(fname)[0])
        # 拡張子だけ違う同じ名前の画像があれば、番号を付けて区別する
        base, k = name, 2
        while name in used_names:
            name = f"{base}_{k}"
            k += 1
        used_names.add(name)

        path = os.path.join(args.folder, fname)
        row = {"image": fname, "name": name, "status": "", "size": "", "goals": "", "rotate": "", "walls": "",
               "unsure": "", "notes": "",
               "info": ""}
        try:
            r = mfi.convert(path, name, args.out, rotate=rotate, invert=args.invert,
                            threshold=args.threshold, force=args.force)
            if r["skipped"]:
                row["status"] = "SKIP"
            else:
                row["rotate"] = r["rotate"]
                row["walls"] = r["walls"]
                row["size"] = r["size"]
                row["goals"] = " ".join(f"({x},{y})" for x, y in r["goals"])
                row["unsure"] = len(r["unsure"])
                row["notes"] = " / ".join(r["notes"])
                row["info"] = " / ".join(r["infos"])
                row["status"] = "CHECK" if (r["notes"] or r["unsure"]) else "OK"
        except Exception as e:  # 1枚の失敗で全体を止めない
            row["status"] = "ERROR"
            row["notes"] = f"{type(e).__name__}: {e}"
        counts[row["status"]] += 1
        rows.append(row)
        detail = row["notes"] if row["status"] in ("CHECK", "ERROR") else ""
        if row["status"] == "CHECK" and row["unsure"]:
            detail = f"unsure {row['unsure']}" + (f" / {detail}" if detail else "")
        if row["rotate"]:
            detail = f"rotated {row['rotate']}" + (f" / {detail}" if detail else "")
        print(f"[{i}/{len(images)}] {row['status']:<5} {fname} -> {name}.txt  {detail}".rstrip())

    os.makedirs(mfi.CHECK_DIR, exist_ok=True)
    # Excel で開いても文字化けしないよう BOM 付きの UTF-8 で書く
    with open(REPORT_PATH, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print()
    print("  ".join(f"{k} {v}" for k, v in counts.items()))
    print(f"report: {mfi.show_path(REPORT_PATH)}")
    print(f"mazes : {mfi.show_path(args.out)}")
    if counts["CHECK"] or counts["ERROR"]:
        print("CHECK / ERROR のものは、確認用の画像 (build/<名前>_check.png) を見て確かめること")
    return 1 if counts["ERROR"] else 0


if __name__ == "__main__":
    sys.exit(main())
