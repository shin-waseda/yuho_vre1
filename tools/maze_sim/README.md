# maze_sim — yuho の迷路シミュレータ

yuho の迷路 logic 層(`Core/Src/logic/maze/*.c`，`Core/Src/logic/command.c`)を PC で動かして確かめるツールです．
**ファームウェアと同じソースと `params.h` をそのままコンパイルする**ので，シミュレータで見ている動きは実機の探索ロジックそのものです．

- **CLI**(`maze_sim`)：1つの迷路の結果を文字で表示する／たくさんの迷路でまとめて試して集計する
- **GUI**(`gui.py`)：探索の様子を pygame で表示する

ファームウェアのビルドとは関係ありません(PC の gcc でコンパイルします)．

## 必要なもの

| もの | 用途 | 確認方法 |
|---|---|---|
| gcc(mingw64，64bit) | CLI と GUI 用 DLL のビルド | `gcc -dumpmachine` → `x86_64-w64-mingw32` |
| Git Bash | `*.sh` の実行 | |
| Python 3(64bit) | GUI | `python --version` |
| pygame | GUI | `python -c "import pygame"` |

pygame がなければ `pip install pygame` で入ります．
Python と gcc のビット数(64bit)が揃っていないと，GUI が DLL を読み込めません．

## はじめに

プロジェクトのルート(`yuho/`)で，Git Bash から実行します．

```sh
sh tools/maze_sim/build.sh          # CLI と GUI 用 DLL をビルド
sh tools/maze_sim/fetch_mazes.sh    # 大会の迷路を取ってくる(任意．ネットにつながる)
```

- ビルド結果は `tools/maze_sim/build/` にできます(git には入りません)．
- 大会の迷路は [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles) の classic(16×16)522 個を `tools/maze_sim/mazes/` に置きます．リポジトリにライセンスの記載がないので，**取ってきたファイルは git に入れません**．別の PC では，もう一度 `fetch_mazes.sh` を実行してください．

## GUI

```sh
python tools/maze_sim/gui.py                    # 乱数の迷路(シード1)
python tools/maze_sim/gui.py --random 7         # シード7の乱数迷路
python tools/maze_sim/gui.py tools/maze_sim/mazes/alljapan-045-2024-exp-fin.txt   # 全日本2024決勝
python tools/maze_sim/gui.py --algo adachi --speed 16
python tools/maze_sim/gui.py --goal 1,0,2,0 --back 1,0,2,3
```

起動時は一時停止しています．**SPACE** で再生します．
logic 層や `params.h` を書き換えた場合，次に起動したときに DLL が**自動でビルドし直されます**(`build.sh` を実行し直す必要はありません)．

### オプション

| オプション | 意味 | 既定値 |
|---|---|---|
| `迷路ファイル` | classic 形式の迷路ファイル | なし(乱数の迷路) |
| `--random N` | 乱数の迷路のシード | 1 |
| `--algo dijkstra\|adachi` | 探索のアルゴリズム | dijkstra |
| `--goal S,T90,T180,K` | 行きのコスト(下の「コスト」を参照) | `params.h` |
| `--back S,T90,T180,K` | 帰りのコスト | `params.h` |
| `--speed N` | 1秒あたりに進む区画数 | 8 |

### キー操作

| キー | 動作 |
|---|---|
| SPACE / P | 再生・一時停止 |
| S / → | 1区画だけ進める |
| F | 最後まで一気に進める |
| R | 同じ迷路で最初から |
| N / B | 次 / 前の迷路(乱数ならシード，ファイルなら同じフォルダの名前順で次のファイル) |
| A | 足立法 ⇔ Dijkstra を切り替えて最初から |
| C | 区画の数字の表示を切り替え |
| T | まだ見ていない壁(灰色)の表示を切り替え |
| ↑ / ↓ | 速さを2倍 / 半分 |
| ESC / Q | 終わる |

### 画面の見方

| 表示 | 意味 |
|---|---|
| 赤い線 | 見つけた壁 |
| 灰色の線 | まだ見ていない本当の壁(T で消せる) |
| 緑の区画 | 4方向の壁がすべて分かった区画 |
| 茶の区画 / 青の区画 | ゴール / スタート |
| 水色の丸と黄色の棒 | 機体と向き |
| 青い線 | 機体が通った跡 |
| 黄色の太線 | 探索が終わった後，分かった壁だけで求めた最短経路 |
| 区画の数字 | プランナーが最後に計算した値．行きはゴールまで，帰りはスタートまで(右の `numbers:` 欄に出る)．Dijkstra はコスト(4つの向きのうち最小)，足立法は歩数 |

右の欄には，フェーズ(`to goal` → `to start` → `done`)，位置，直前の指令，移動した区画数(行き＋帰り)が出ます．
探索が終わると，最短経路のコストと，迷路を全部知っていた場合の最短のコストが出ます．一致すれば `= optimal` です．

## CLI

### 1つの迷路

```sh
tools/maze_sim/build/maze_sim --random 5                 # 乱数の迷路
tools/maze_sim/build/maze_sim tools/maze_sim/mazes/alljapan-045-2024-exp-fin.txt
tools/maze_sim/build/maze_sim --random 5 --verbose       # 1区画ごとに地図を表示
```

出力の順番:

1. `=== maze ===`：本当の迷路(`^` がスタートの機体，`G` がゴール)
2. `reached goal ...`：ゴールに着くまでに動いた区画数
3. `=== explored map ===`：探索で分かった地図．数字はゴールまでのコスト，`.` と `:` は未知の壁，`*` は分かっている壁だけでは行けない区画
4. `=== fastest-run route ===`：最短走行の指令の列(`FORWARD x3`，`RIGHT` など)
5. `route cost: 96 (best possible 86)`：見つけた経路のコストと，真の最短のコスト

壁にぶつかる(`CRASH`)，プランナーの位置がずれる，探索が終わらない，といった場合は止まって終了コード 1 になります．

### まとめて試す

```sh
tools/maze_sim/build/maze_sim --batch 300                          # 乱数の迷路(シード1〜300)
tools/maze_sim/build/maze_sim tools/maze_sim/mazes/alljapan-*.txt  # ファイルが2つ以上ならまとめて
```

```
dijkstra goal 1,7,50,0 back 1,7,50,3 | 61 mazes, 0 failed, optimal  24, moves 113.3 + 80.3 = 193.6, cost ratio 1.090
```

| 項目 | 意味 | よい方向 |
|---|---|---|
| `goal` / `back` | 行き・帰りのコスト | |
| `failed` | 失敗した迷路の数 | 0 |
| `optimal` | 真の最短を見つけた迷路の数 | 多い |
| `moves` | 探索で動いた区画数の平均(行き＋帰り) | 少ない |
| `cost ratio` | 見つけた経路 ÷ 真の最短 の平均 | 1 に近い |

`mazes/` の中の `001.txt` と `001-anomaly-test.txt` は，ゴールに到達できないテスト用の迷路なので，`FAILED` になるのが正しい動作です．

### コストの比較(sweep.sh)

```sh
sh tools/maze_sim/sweep.sh 300     # 乱数300迷路で比較(約1.5分)．mazes/ があれば全日本の迷路でも比較
```

試したい組み合わせがあれば，`sweep.sh` の `for` の値を書き換えてください．

### その他のオプション

| オプション | 意味 |
|---|---|
| `--algo dijkstra\|adachi` | 探索のアルゴリズム |
| `--goal S,T90,T180,K` / `--back S,T90,T180,K` | 行き・帰りのコスト |
| `--known-back K` | 帰りの既知区画の上乗せだけを変える |
| `--sizes` | 構造体の大きさを表示して終わる |

## コスト

Dijkstra は，1区画進むごとに次のコストを足して，合計が最小の経路を選びます．

```
直進 S  ＋  (向きを変えたら) 90°: T90 / 180°: T180  ＋  (入る区画の壁が4方向とも既知なら) K
```

- 既定値は `params.h` の `MAZE_COST_STRAIGHT` / `MAZE_COST_TURN90` / `MAZE_COST_TURN180`(1, 7, 50)と，帰りの `MAZE_COST_KNOWN_CELL_RETURN`(3)です．
- **経路の評価(`route cost` や `optimal`)は，常に `params.h` の最短走行用コスト**で計算します．`--goal` / `--back` で変わるのは探索中の判断だけなので，設定を変えても比べる基準は変わりません．
- `S` は 1 以上が必要です．コストは 13bit(8190 まで)で持つので，大きすぎる値はエラーになります．
- 足立法は歩数で動くので，これらのコストは使いません．
- 乱数の迷路と大会の迷路で，よい設定が逆になることがあります．調整には大会の迷路(全日本 61 個)を主に使うのがおすすめです．

## 迷路ファイルの形式

micromouseonline/mazefiles の classic 形式と同じです．北(上)から 33 行(16×16 の場合)．

```
o---o---o---o ...
|       |
o   o---o   o ...
| S |
o---o---o---o ...
```

- 角は `o` か `+`，横の壁は `---`，縦の壁は `|` です．
- 区画の中の `S` / `G` は読み飛ばします．スタートとゴールは `params.h`(`MAZE_START_X/Y`，`MAZE_GOALS`)で決まります．
- CLI が表示する `=== maze ===` の 33 行をそのまま保存しても読めます．

## ファイル構成

| ファイル | 役割 |
|---|---|
| `maze_sim.c` | CLI |
| `sim_core.c` / `sim_core.h` | CLI と GUI で共有する部分(迷路の生成・読み込み，壁の観測，指令の実行) |
| `sim_api.c` | GUI 用 DLL の窓口．中で logic 層の `SearchPlanner` を動かす |
| `sim_lib.py` | DLL を ctypes で読み込む．ソースが変わっていたら自動で再ビルドする |
| `gui.py` | pygame での描画と操作 |
| `build.sh` | CLI と DLL のビルド |
| `fetch_mazes.sh` | 大会の迷路の取得 |
| `sweep.sh` | コストの比較 |

GUI の構成は [akiaki96/maze_sim_py_c_v2](https://github.com/akiaki96/maze_sim_py_c_v2)(C のソルバー ＋ Python/pygame の描画)を参考にしています．

## うまくいかないとき

| 症状 | 対処 |
|---|---|
| `build.sh` で `cannot open output file build/maze_sim_lib.dll: Permission denied` | GUI が DLL を使用中です．GUI を閉じてから実行してください |
| GUI が DLL を読み込めない(`OSError`) | Python と gcc のビット数を確認してください(両方 64bit が必要) |
| `*.sh` で `$'\r': command not found` | 改行が CRLF になっています．`.gitattributes` で `*.sh` は LF に固定してあるので，`git checkout -- tools/maze_sim/*.sh` で直ります |
| `invalid cost` | `S` が 0，またはコストが 13bit に収まりません |
