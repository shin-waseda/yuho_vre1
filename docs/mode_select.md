# モードの選び方(一覧)

ブランチ `feature/run-menu` のコード(2026-10-09 の終わり)をもとに書いた．メニューの表は `Core/Src/app/mode_ui.c`，
値の表は `Core/Inc/params.h`．コードを変えたらここも直す．

## 操作

- 起動すると電池の残りを LED の棒グラフで出した後，モードを選ぶ画面になる．
- **右のタイヤを回して選び，ボタンで決める**．タイヤ 1/4 周で1つ進む．端では輪のようにつながる(1 番から逆に回すと最後の番号)．
- モードの番号 n は，シフトレジスタの **LED n と n+1** が点く．
- 値(速さなど)を選ぶ画面は **1 番(一番遅い値)から始まり**，n 番なら **LED1〜n** が点く棒グラフ．
- 確実に光るのは LED1〜7(U6 のはんだ不良)．8 番以降は見えないことがあるので，1 番から逆に回して選ぶ．
- UART をつないでいれば，番号・名前・選んだ値が出る．
- 一度モードを決めたら，選び直すにはリセット(電源を入れ直す)．

## 一番上の階層

| 番号 | 中身 | 決まっている値(選ばない) | 逆に回すと |
|---|---|---|---|
| 1 | **RUN**(中で選ぶ) | ― | |
| 2 | **TEST**(中で選ぶ) | ― | |
| 3 | **SD**(中で選ぶ) | ― | |
| 4 | STRAIGHT_SWEEP(v) | SPEED 1400〜1500，ACCEL 2000〜2000(4本) | |
| 5 | STRAIGHT_SWEEP(acc) | SPEED 1000〜1000，ACCEL 3000〜10000(12本) | |
| 6 | SLALOM_SWEEP(low) | FROM 300，TO 700(20本) | |
| 7 | SLALOM_SWEEP(high) | FROM 800，TO 900(8本) | |
| 8 | FAST_SWEEP | なし(中身は RUN → TEST の 1 と同じ．値を選ぶ) | 3つ戻す |
| 9 | LONG_LOG | PART 2，STEP 1 | 2つ戻す |
| 10 | SENSOR_SPIN | 選ぶものはない | 1つ戻す |

4〜10 はショートカット．選ぶとすぐそのモードに決まり，決まっている値は選ぶ画面を飛ばす(`params.h` の `SHORTCUT_*`)．
決まっている値が選べる値の表にないときは，その所だけふつうに選ぶ．

## RUN(走る)

| 番号 | モード | 選ぶもの(この順．最後に手かざし) | ログ |
|---|---|---|---|
| 1 | SEARCH | MAP(1 初期化，2 重ねる) → SCOPE(1 往復，2 片道，3 全面) → ALGO(1 Dijkstra，2 足立法) → SPEED → ACCEL → SLALOM → TURN(1 スラローム，2 超信地旋回) | `search/search_NNNN` |
| 2 | FAST | SPEED → ACCEL(最初は SPEED に合う値) → SMALL TURN → TURN(1 SMALL，2 LARGE，3 PIVOT) | `search/fast_NNNN` |
| 3 | TEST | TEST(1 FAST_SWEEP，2 SEARCH_SPIN，3 FAST_BANDS，4 LONG_LOG) → それぞれの選ぶもの | 下を見る |

どの値も，何も回さずに決めると 1 番(FAST の ACCEL だけは SPEED に合う値)．

- **SEARCH**: スタート区画に北向きに置く．終わったら同じ設定で次の手かざしを待つ．
  - MAP: 1 は空の地図から．2 は flash に残した地図(前の探索)から始める(なければ空の地図)．
  - SCOPE: 1 はゴールへ行ってスタートに戻る．2 はゴールの真ん中で止まって終わる．どちらもゴールまで行けば地図を flash に残す．
    3(全面探索)は**まだ作っていない**(選ぶと走らずに止まる)．
  - ACCEL: 直進の加速度・減速度(1 番の 2000 が今までの値)．
  - 壁が全部分かっている区間はまとめて走る(直線の加速．SPEED が大回りの速さ以上なら大回り)．
- **FAST**: 探索で残した地図で，スタートからゴールまで走る(ゴールで終わり．同じ設定で次の手かざしを待つ)．
  ACCEL は最初に SPEED に合う値(下の表)が出る．減速度は ACCEL と 5000 の小さい方．
- **TEST**: ログ取りの走行．
  - 1 FAST_SWEEP: SPEED FROM → SPEED TO → SMALL FROM → SMALL TO → TYPE(1 SMALL，2 LARGE，3 両方)．全部の組み合わせを1回の手かざしで続けて走る．
    1本ごとにゴールからスタートへ自分で戻る．加速度・減速度は SPEED から決まる．ログは `search/fast_NNNN`，`search/back_NNNN`．
  - 2 SEARCH_SPIN: SPEED → SLALOM → 曲がり方(クリックで切り替え: PIVOT / **SMALL**)．Dijkstra の往復の探索で，初めて入った区画ごとに真ん中で止まって
    左に1周・右に1周回る(壁センサーのモデル用．1区画 約18秒)．区画と壁は `SENSOR_SPIN` のイベントに残る．ログは `search/spin_NNNN`．
  - 3 FAST_BANDS: BAND FROM → BAND TO．速度帯(下の表)を遅い順に「最短走行 → スタートへ戻る」をくり返す．本番は FROM と TO に同じ速度帯．
  - 4 LONG_LOG: PART → STEP(TEST の LONG_LOG と同じ)．

ゴールは `params.h` の `MAZE_GOALS`((7,7)，(8,7)，(7,8)，(8,8) の4区画)．試しのゴールを使うときは `SEARCH_USE_TEST_GOAL` を 1 にする．

## TEST(試験)

| 番号 | モード | 選ぶもの | ログ |
|---|---|---|---|
| 1 | SENSOR | なし(センサー・ジャイロ・エンコーダの値を出し続ける) | ― |
| 2 | SENSOR_LOG | なし(ボタンで数秒ぶんの壁センサーの値を記録．止まった状態) | `sensor/wall_NNNN` |
| 3 | VEL_PID | なし(ボタンで速度のステップ応答を1回) | `vel_pid/step_NNNN` |
| 4 | STRAIGHT | SPEED → ACCEL | `straight/wall_run_NNNN` |
| 5 | PIVOT | OMEGA(180 / 360 / 540) | `pivot/turn90_NNNN` |
| 6 | SLALOM | SPEED(s90) → 旋回(クリックで切り替え: s90r / s90l / l90r / l90l / l180r / l180l) | `slalom/<旋回>_NNNN` |
| 7 | LED_TEST | なし | ― |
| 8 | PARTY | なし(宴会芸．床を回されても同じ向きを保つ) | `party/hold_NNNN` |
| 9 | LONG_LOG | PART → STEP | `search/*` |
| 10 | SLALOM_SWEEP | FROM(s90) → TO(s90) | `slalom/s90r_NNNN`，`slalom/s90l_NNNN` |
| 11 | STRAIGHT_SWEEP | SPEED FROM → SPEED TO → ACCEL FROM → ACCEL TO | `straight/sweep_NNNN` |
| 12 | SENSOR_SPIN | なし(真ん中に手で置いて手かざし．左に1周・右に1周) | `sensor/spin_NNNN` |

- **LONG_LOG**: 探索を速さを変えて，次に最短走行を小回り・大回りで，続けて走る．PART 1〜4 は探索(直進の速さ 300 / 400 / 500 / 600)，
  5 は最短走行の小回り，6 は大回り．STEP は PART の中の番号(電池を替えた後に続きから始めるため)．
- **SLALOM_SWEEP**: C の L字の通路の A に北向きに置く．各速さで右と左を2往復(4本)．
- **STRAIGHT_SWEEP**: 直線の通路の端に置く．加速度ごとに速さを上げ，各組み合わせで行きと帰り(2本)．
- **SENSOR_SPIN**: 区画・どの壁があったか・スタートの向きを書き留める(plant_sim の壁センサーのモデル用)．

## SD

| 番号 | モード | 中身 |
|---|---|---|
| 1 | SD_DUMP | まだ送っていないログを，ボタンで UART へ送る(PC は `tools/get_log.py`)．送ったものは SD の `sent/` へ移す |
| 2 | SD_DUMP_ALL | 送ったものも含めて全部を UART へ送る |
| 3 | STREAM_TEST | 走りながら SD へ流すログの試験(モーターは動かさない．クリックで間隔 1 / 5ms) |

SD のログは，PC に SD カードを挿して `tools/sd_import.py` で取り込むこともできる．

## 選べる値の表(params.h)

| 選ぶもの | 値(1 番から．遅い順) | params.h |
|---|---|---|
| 探索の SPEED | 300，400，500，600，800，1000，1200，1500 | `SPEED_SELECT_SEARCH_V_MM_S` |
| 探索の SLALOM | 300，400，500，600，700 | `SPEED_SELECT_SEARCH_TURN_V_MM_S` |
| 最短走行の SPEED | 600，800，1000，1200，1400，1500 | `SPEED_SELECT_FAST_V_MM_S` |
| 最短走行の SMALL TURN | 300，400，500，600，700 | `SPEED_SELECT_FAST_SMALL_V_MM_S` |
| 直進の試験の SPEED | 300，400，500，600，800，1000，1200，1400，1500 | `SPEED_SELECT_STRAIGHT_V_MM_S` |
| 直進の試験の ACCEL | 2000，3000，4000，5000，6000，8000，10000 | `SPEED_SELECT_ACCEL_MM_S2` |
| スラロームの試験の速さ | 300，400，500，600，700，800，900 | `SPEED_SELECT_SLALOM_TEST_V_MM_S` |
| PIVOT の OMEGA | 180，360，540 | `SPEED_SELECT_PIVOT_OMEGA_DPS` |

### 最短走行の加速度・減速度(FAST_SWEEP は SPEED から決まる．FAST は ACCEL の最初の値)

| SPEED [mm/s] | 600 | 800 | 1000 | 1200 | 1400 | 1500 |
|---|---|---|---|---|---|---|
| 加速度 [mm/s²] | 3000 | 3000 | 4000 | 5000 | 6000 | 8000 |
| 減速度 [mm/s²] | 3000 | 3000 | 4000 | 5000 | 5000 | 5000 |

`FAST_ACCEL_FOR_SPEED_MM_S2`，`FAST_DECEL_FOR_SPEED_MM_S2`．

### FAST_BANDS の速度帯

| 帯 | 直進 [mm/s] | 加速度 | 減速度 | 小回り [mm/s] | 走り方 |
|---|---|---|---|---|---|
| 1 | 800 | 3000 | 3000 | 400 | SMALL |
| 2 | 1000 | 4000 | 4000 | 500 | SMALL |
| 3 | 1200 | 5000 | 5000 | 600 | SMALL |
| 4 | 1200 | 5000 | 5000 | 600 | LARGE |
| 5 | 1400 | 6000 | 5000 | 600 | LARGE |
| 6 | 1500 | 8000 | 5000 | 600 | LARGE |

`FAST_BANDS`．大回りの速さはどの帯も `FAST_LARGE*`(600mm/s)．

## 止まったとき

- **シフトレジスタの LED が本数ぶん点滅**: 連続のモード(FAST_SWEEP，FAST_BANDS，STRAIGHT_SWEEP，SLALOM_SWEEP，LONG_LOG)が止めた印．
  電池(走る前の静止時で 7.5V 未満)，経路がない，走りの打ち切りなど．点滅した本数が，何本目で止まったか．
- **マイコン直結の LED が点滅**: FailSafe．LED_5 だけ: 電池の低電圧(7.0V)，LED_1・2: 車輪の速度の誤差，LED_1〜3: 角速度，全部: それ以外．
- **左後ろの LED が点いたまま**: SD へのログの保存に失敗した(走りは続ける)．
