# maze_sim — yuho の迷路シミュレータ

yuho の迷路 logic 層(`Core/Src/logic/maze/*.c`，`Core/Src/logic/command.c`)を PC で動かして確かめるツールです．
**ファームウェアと同じソースと `params.h` をそのままコンパイルする**ので，シミュレータで見ている動きは実機の探索ロジックそのものです．

画面・迷路の一覧・まとめて試す・迷路図の変換は，公開版の **mouse maze sim**(別のリポジトリ)を使います．
このフォルダには yuho をそれにつなぐ部分と，C の CLI があります．

| 使うもの | 中身 |
|---|---|
| `msim.py gui` / `gui.py` | 画面(mouse maze sim．ブラウザに開く) |
| `msim.py batch` / `run` | まとめて試す / 1つの迷路の結果(mouse maze sim) |
| `build/maze_sim` | C の CLI(`--verbose` で1区画ごとの地図，最短走行の経路を本当の迷路で走らせる確かめ) |

画面・迷路の一覧・迷路図の変換の使い方は，mouse maze sim の README を見てください．ここには yuho で使うときのことだけを書きます．

## 必要なもの

| もの | 用途 | 確認方法 |
|---|---|---|
| mouse maze sim | 画面，まとめて試す | 下の「はじめに」 |
| gcc(mingw64，64bit) | yuho の logic 層のビルド | `gcc -dumpmachine` → `x86_64-w64-mingw32` |
| Git Bash | `*.sh` の実行 | |
| Python 3(64bit)，ブラウザ | 画面 | |

## はじめに

1. mouse maze sim を置き，yuho から見えるようにする(どちらか)
   - `pip install -e <mouse maze sim のフォルダ>`
   - `tools/maze_sim/mazesim.local` の1行目にそのフォルダを書く(git に入れない)．環境変数 `MAZESIM_PATH` でもよい
2. プロジェクトのルート(`yuho/`)で動かす

```sh
python tools/maze_sim/gui.py                        # 前回開いた迷路(なければ乱数の迷路)
python tools/maze_sim/gui.py --random 7             # シード7の乱数迷路
python tools/maze_sim/gui.py --opt algo=adachi --opt scope=full
python tools/maze_sim/msim.py batch --random 300    # 乱数300迷路でまとめて試す
python tools/maze_sim/msim.py batch --list alljapan --type CM   # 同梱の全日本のクラシック(公式図から)
python tools/maze_sim/msim.py fetch                 # mazefiles の迷路を取ってくる(~/.mazesim/mazefiles)
sh tools/maze_sim/build.sh                          # C の CLI をビルド
```

- `msim.py` は mouse maze sim のコマンドに `--mouse tools/maze_sim/mouse.json` を付けて渡すだけです(`list`，`fetch`，`image`，`image-batch`，`verify` もそのまま使える)．
- logic 層や `params.h` を書き換えた場合，次に起動したときに yuho の部分が**自動でビルドし直されます**(`build/msim_yuho_<ハッシュ>.dll`)．
- 公式図から変換した迷路(前の `my_mazes/`)は mouse maze sim の `mazes/` に同梱しました．新しい迷路図は `msim.py image-batch tools/maze_sim/maze_file` で
  `~/.mazesim/mazes/` に変換されます(同梱に足すときは `--out <mouse maze sim>/mazes`)．
- お気に入り・最近開いた迷路は `~/.mazesim/prefs.json` に移りました(前の `maze_prefs.json` は読みません)．

## yuho のオプション

画面では，右の欄の「探索」で，選択肢のあるもの(`algo`，`scope`)を選び(数字キー 1 / 2 と下の表のキーでも)，
数字のもの(`search`，`run`，コスト)は入力欄に打ち込んで Enter で決めます．コマンドでは `--opt 名前=値`．

| 名前 | 値 | キー | 意味 | 前の GUI のオプション |
|---|---|---|---|---|
| `algo` | `dijkstra` / `adachi` | A | 探索のアルゴリズム | `--algo` |
| `scope` | `round` / `oneway` / `full` | O | 探索の行き先(往復・片道・全面．機体の RUN → SEARCH の SCOPE) | `--scope` |
| `search` | `V,TURN_V,ACCEL` | | 探索の速さ(直進の最高速度，小回りの速さ，加速度)．既定は `params.h` の `SEARCH_V_MM_S`，`SLALOM_V_MM_S`，`SEARCH_ACCEL_MM_S2` | `--search` |
| `run` | `VMAX,ACCEL,SMALL90,LARGE90,LARGE180` | | 最短走行の速さ(直進の最高速度，加速度，旋回3種の速さ)．既定は `params.h` の `RUN_*`．探索はやり直さない | (前は `--band`) |
| `goal_cost` | `S,T90,T180,K` | | 行きのコスト(下の「コスト」) | `--goal` |
| `back_cost` | `S,T90,T180,K` | | 帰りのコスト | `--back` |

```sh
python tools/maze_sim/msim.py batch --random 300 --opt run=3000,5000,600,850,850   # 速さを変えたとき，時間で選ぶ経路がどう変わるか
```

### 時間の数え方(msim の画面とコマンド)

maze_sim はアルゴリズムの確認だけなので，時間は**計算上の速さの運動だけ**で数えます．滑り，ログで合わせた補正，尻当て・待ち・超信地旋回の時間は数えません
(時間で選ぶ経路など，時間に依存するアルゴリズムを確かめるための時間です．実機の時間とは合いません)．

| 動き | 数え方 |
|---|---|
| 最短走行の直進 | 台形加速(加速度 = 減速度．入りと出は前後の旋回の速さ，スタートとゴールは 0) |
| 最短走行の旋回 | 理想の円弧の長さ ÷ 旋回の速さ(小回りは半径 1/2 区画の 90°，大回り 90° は半径 1 区画，大回り 180° は半区画 + 柱を回る半円 + 半区画)．前後のオフセットなし |
| 探索の直進 | 境目から境目まで台形加速．区画の境目はいつも小回りの速さで通る(真ん中からなら止まった状態から半区画) |
| 探索の左右 | 小回りの円弧 ÷ 小回りの速さ．真ん中にいるときは，その場で回って(時間 0)半区画加速 |
| 探索の 180°，ゴール・スタート | 真ん中まで半区画で止まる．その場で回る時間は 0 |

最短走行は logic 層の `RunProfile_Default()` の形に `run` の速さを入れたもの，探索は `yuho_mouse.c` の `SearchTime_*` です．

右の欄の yuho の行: 段階(`to goal` → `to start` → `done`．全面探索は `full search` → `to start` → `done`)，探索の行き先，区画の数字の意味，
探索の時間(`time:`．ゴールに止まった・最短経路が決まった時刻も)，探索の後の経路のコスト(`= optimal` なら真の最短)．

最短走行の経路は2つです(**V** で切り替え．それぞれ「探索で分かった壁だけ」と「迷路を全部知っていたら(all walls)」)．

| 名前 | 経路 |
|---|---|
| `time-optimal` | **走行時間が最短**の経路(時間 Dijkstra．下の「最短走行(走行時間)」) |
| `cost + large` | **コストが最短**の経路を，大回りに置き換えたもの(機体の最短走行と同じ選び方) |

`msim.py batch` の行の後ろには，yuho が出す数値の平均が出ます: `search_s`(探索の時間)，`goal_s`(ゴールに止まった・最短経路が決まった時刻)，
`cost_optimal`(コストが真の最短と同じだった割合)，`cost_ratio`(見つけた経路 ÷ 真の最短)．

## C の CLI(build/maze_sim)

`msim.py run` / `batch` と同じ探索を C だけで行い，yuho だけの内訳を出します．
時間の数え方は画面と同じ(上の「時間の数え方」．`ideal_time.c` を共有)で，同じ迷路・同じ速さなら同じ時間になります．迷路ファイルの形式と，迷路の `G`(ゴール)の扱いは mouse maze sim と同じです(この CLI は 16×16 だけ)．

### 1つの迷路

```sh
tools/maze_sim/build/maze_sim --random 5                 # 乱数の迷路
tools/maze_sim/build/maze_sim ~/.mazesim/mazefiles/alljapan-045-2024-exp-fin.txt
tools/maze_sim/build/maze_sim --random 5 --verbose       # 1区画ごとに地図を表示
tools/maze_sim/build/maze_sim <mouse maze sim>/mazes/MM2023CM.txt --scope full --search 600,500,2000   # 全面探索(機体の 600/500)
```

出力の順番:

0. `search: ... | fast run: ...`：探索の行き先と速さ，最短走行の速さ
1. `=== maze ===`：本当の迷路(`^` がスタートの機体，`G` がゴール)
2. `reached goal ...`：ゴールに着くまでに動いた区画数と時刻(全面探索は `shortest route decided ...`：最短経路が決まった所)．
   続く `search time: ...` は探索の時間(運動だけ)
3. `=== explored map ===`：探索で分かった地図．数字はゴールまでのコスト，`.` と `:` は未知の壁，`*` は分かっている壁だけでは行けない区画
4. `=== fastest-run route ===`：コストが最短の経路(区画ごとの指令の列．`FORWARD x3`，`RIGHT` など)
5. `=== fastest-run commands (route above, large turns) ===`：4 の経路を大回りに置き換えた最短走行の指令と，指令ごとの時間
6. `=== fastest-run commands (time-optimal) ===`：走行時間が最短の経路の指令と，指令ごとの時間
7. 結果:
   - `route cost: 96 (best possible 86)`：見つけた経路のコストと，真の最短のコスト
   - `run time: 12.763 s small turns only, 11.385 s with large turns`：4 の経路を小回りだけで走った時間と，大回りに置き換えた時間(負なら加速度が足りない)
   - `time-optimal run: 11.385 s (best possible 10.554 s)`：走行時間が最短の経路の時間と，迷路を全部知っていた場合の時間

最短走行の指令の直進は半区画単位で `x7 /2` のように出ます．
走行時間が最短の経路は，区画の経路に戻して本当の迷路で走らせ，壁にぶつからずにゴールで止まるかも確かめています．

壁にぶつかる(`CRASH`)，プランナーの位置がずれる，探索が終わらない，といった場合は止まって終了コード 1 になります．

### まとめて試す

```sh
tools/maze_sim/build/maze_sim --batch 300                          # 乱数の迷路(シード1〜300)
tools/maze_sim/build/maze_sim ~/.mazesim/mazefiles/alljapan-*.txt  # ファイルが2つ以上ならまとめて
tools/maze_sim/build/maze_sim $(python tools/maze_sim/msim.py list --paths alljapan --type CM)  # 同梱の全日本のクラシック
```

```
dijkstra goal 1,7,50,0 back 1,7,50,3 | 61 mazes, 0 failed, optimal  24, moves 113.3 + 80.3 = 193.6, cost ratio 1.090, run time 12.296 s (small) 11.203 s (large) 11.085 s (time-opt, best 10.611 s, optimal 19)
```

| 項目 | 意味 | よい方向 |
|---|---|---|
| `goal` / `back` | 行き・帰りのコスト | |
| `failed` | 失敗した迷路の数 | 0 |
| `optimal` | 真の最短を見つけた迷路の数 | 多い |
| `moves` | 探索で動いた区画数の平均(行き＋帰り．全面探索の「行き」は最短経路が決まるまで) | 少ない |
| `search` / `goal` | 探索の時間の平均 / ゴールに止まった(全面探索は最短経路が決まった)時刻の平均 | 短い |
| `cost ratio` | 見つけた経路 ÷ 真の最短 の平均 | 1 に近い |
| `run time` | 最短走行の時間の平均．`small` / `large` はコストが最短の経路を小回りだけ / 大回りに置き換えて走った時間，`time-opt` は走行時間が最短の経路の時間 | 短い |
| `best` / `optimal`(run time の中) | 迷路を全部知っていた場合の時間の平均 / それと同じ時間だった迷路の数 | `time-opt` が `best` に近い / 多い |

mazefiles の `001.txt` と `001-anomaly-test.txt` は，ゴールに到達できないテスト用の迷路なので，`FAILED` になるのが正しい動作です．

### コストの比較(sweep.sh)

```sh
sh tools/maze_sim/sweep.sh 300     # 乱数300迷路で比較(約1.5分)．~/.mazesim/mazefiles があれば全日本の迷路でも比較
```

試したい組み合わせがあれば，`sweep.sh` の `for` の値を書き換えてください．

### その他のオプション

| オプション | 意味 |
|---|---|
| `--algo dijkstra\|adachi` | 探索のアルゴリズム |
| `--goal S,T90,T180,K` / `--back S,T90,T180,K` | 行き・帰りのコスト |
| `--known-back K` | 帰りの既知区画の上乗せだけを変える |
| `--scope round\|oneway\|full` | 探索の行き先(往復・片道・全面．既定 round) |
| `--search V,TURN_V,ACCEL` | 探索の速さ(直進の最高速度，小回りの速さ，加速度．既定は `params.h`) |
| `--run VMAX,ACCEL,SMALL90,LARGE90,LARGE180` | 最短走行の速さ(既定は `params.h` の `RUN_*`) |
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

## 最短走行(走行時間)

最短走行の経路を，コストではなく**走行時間**で選びます．logic 層の `run_path`(指令と時間の見積もり)と `time_dijkstra`(時間で最短の経路)で，ファームウェアと同じソースです．

### 指令

| 指令 | 区画ごとの曲がり方 | 動き |
|---|---|---|
| `STRAIGHT` | 0 | 半区画単位の直進 |
| `SMALL90` | R | 境目 → 同じ区画の横の境目(探索と同じ小回り) |
| `LARGE90` | 0 R 0 | 区画 A の中心 → C の中心 |
| `LARGE180` | 0 R R 0 | 区画 A の中心 → 隣の列の D の中心(柱を回る U ターン) |

大回りは「半区画・曲がる区画・半区画」を 1 つの指令にしたものです．残りの半区画は前後の直進に入ります(例：`STRAIGHT 3, LARGE90_R, STRAIGHT 3`)．
大回りどうしは区画を半分ずつ使い合って直接つながれます(`0 R 0 L 0` → `LARGE90_R, LARGE90_L`)．

### 時間の数え方

上の「時間の数え方(msim の画面とコマンド)」のとおり，計算上の速さの運動だけです(logic 層の `RunProfile_Default()` の形に `run` / `--run` の速さを入れる)．
**速度は直進の間だけで変えます**．そのため，小回りの直後に大回りが来るときは間に半区画の直進が入り，そこで加速します．
スタートはスタート区画の中心で静止，ゴールは最初に入ったゴールの区画の中心で静止します．

### 時間 Dijkstra

ノードは「これから旋回を始める位置(小回りは境目，大回りは区画の中心)× 向き × 旋回の種類」，辺は「旋回 1 つ + その後の直進」です．ゴール側から逆向きに広げるので，迷路のどこからでもゴールまでの最短時間が求まります．
機体の最短走行は経路を**コスト**(Dijkstra)で選ぶので，`time-optimal` は「時間で選んだらどうなるか」の比較用です．

## ファイル構成

| ファイル | 役割 |
|---|---|
| `mouse.json` | mouse maze sim に渡す yuho の設定(ビルドするソース，インクルード) |
| `yuho_mouse.c` | mouse maze sim の窓口(`msim.h`)の yuho の実装．中で logic 層の `SearchPlanner` を動かす．時間は運動だけで数える |
| `msim.py` / `gui.py` | yuho の設定で mouse maze sim を動かす |
| `maze_sim.c` | C の CLI |
| `sim_core.c` / `sim_core.h` | C の CLI と窓口で共有する部分(迷路の読み込み・乱数の迷路，壁の観測，指令の実行) |
| `ideal_time.c` / `ideal_time.h` | 時間の数え方(計算上の速さの運動だけ)．C の CLI と窓口で共有する |
| `mazesim.local` | この PC の mouse maze sim の場所(git に入れない) |
| `maze_file/` | 公式サイトから集めた迷路図の画像(git に入れない) |
| `build.sh` | C の CLI のビルド |
| `sweep.sh` | コストの比較 |

乱数の迷路は C(`sim_core.c`)と mouse maze sim(Python)で同じ作り方なので，同じシードなら同じ迷路です(2026-10-21 に 100 個で結果が一致することを確かめた)．

## うまくいかないとき

| 症状 | 対処 |
|---|---|
| `mouse maze sim not found` | 上の「はじめに」の 1．`mazesim.local` か `pip install -e` |
| 画面がライブラリを読み込めない(`OSError`) | Python と gcc のビット数を確認してください(両方 64bit が必要) |
| `*.sh` で `$'
': command not found` | 改行が CRLF になっています．`.gitattributes` で `*.sh` は LF に固定してあるので，`git checkout -- tools/maze_sim/*.sh` で直ります |
| `goal_cost must be ...` | `S` が 0，またはコストが 13bit に収まりません |
