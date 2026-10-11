# モードの一覧(入れ子)

ブランチ `feature/autonomous` のコード(2026-10-11)をもとに書いた．メニューの表は `Core/Src/app/mode_ui.c`，値の表は `Core/Inc/params.h`．
操作の仕方・値の表・止まったときの LED は [mode_select.md](mode_select.md) を見る．

- 右のタイヤを回して選び，ボタンで決める．番号はエンコーダで送る順(1 から)．
- `→` は選ぶ順．`[ ]` の中は選べる値(1 番から)．何も回さずに決めると 1 番．
- 「手かざし」と書いたものは，選び終わった後に手かざしで走り出す．

```
MENU
├─ 1 RUN
│   ├─ 1 SEARCH ─ MAP[1 NEW, 2 CONTINUE] → SCOPE[1 往復, 2 片道, 3 全面] → ALGO[1 Dijkstra, 2 足立法]
│   │             → SPEED → ACCEL → SLALOM → TURN[1 スラローム, 2 超信地旋回] → 手かざし(終わったら次の手かざしを待つ)
│   ├─ 2 FAST ─── SPEED → ACCEL(最初は SPEED に合う値) → SMALL TURN → TURN[1 SMALL, 2 LARGE, 3 PIVOT]
│   │             → 手かざし(終わったら次の手かざしを待つ)
│   ├─ 3 TEST ─── TEST で選ぶ
│   │   ├─ 1 FAST_SWEEP ── SPEED FROM → SPEED TO → SMALL FROM → SMALL TO → TYPE[1 SMALL, 2 LARGE, 3 両方] → 手かざし
│   │   ├─ 2 SEARCH_SPIN ─ SPEED → SLALOM → 曲がり方(クリックで PIVOT / SMALL) → 手かざし
│   │   ├─ 3 FAST_BANDS ── BAND FROM → BAND TO → 手かざし
│   │   └─ 4 LONG_LOG ──── PART → STEP → 手かざし
│   └─ 4 AUTO(自立賞) ─ 選ぶものはない(params.h の AUTONOMOUS_*) → 手かざし1回で最後まで
├─ 2 TEST
│   ├─  1 SENSOR ───────── なし(センサー・ジャイロ・エンコーダの値を出し続ける)
│   ├─  2 SENSOR_LOG ───── なし(ボタンで壁センサーの値を記録)
│   ├─  3 VEL_PID ──────── なし(ボタンで速度のステップ応答)
│   ├─  4 STRAIGHT ─────── SPEED → ACCEL
│   ├─  5 PIVOT ────────── OMEGA[180, 360, 540]
│   ├─  6 SLALOM ───────── SPEED(s90) → 旋回(クリックで s90r / s90l / l90r / l90l / l180r / l180l)
│   ├─  7 LED_TEST ─────── なし
│   ├─  8 PARTY ────────── なし(宴会芸)
│   ├─  9 LONG_LOG ─────── PART → STEP
│   ├─ 10 SLALOM_SWEEP ─── FROM(s90) → TO(s90)
│   ├─ 11 STRAIGHT_SWEEP ─ SPEED FROM → SPEED TO → ACCEL FROM → ACCEL TO
│   └─ 12 SENSOR_SPIN ──── なし(真ん中に置いて手かざし)
├─ 3 SD
│   ├─ 1 SD_DUMP ───── まだ送っていないログを UART へ送る
│   ├─ 2 SD_DUMP_ALL ─ 全部のログを UART へ送る
│   └─ 3 STREAM_TEST ─ SD へ流すログの試験(クリックで間隔 1 / 5ms)
│
│  ── ここから下はショートカット(選ぶとすぐ決まる．値は params.h の SHORTCUT_* で決まっていて選ばない) ──
├─  4 STRAIGHT_SWEEP(v) ─── SPEED 1400〜1500，ACCEL 2000〜2000(4本)
├─  5 STRAIGHT_SWEEP(acc) ─ SPEED 1000〜1000，ACCEL 3000〜10000(12本)
├─  6 SLALOM_SWEEP(low) ─── FROM 300，TO 700(20本)
├─  7 SLALOM_SWEEP(high) ── FROM 800，TO 900(8本)
├─  8 FAST_SWEEP ────────── 値は選ぶ(RUN → TEST → 1 と同じ)
├─  9 LONG_LOG ──────────── PART 2，STEP 1
└─ 10 SENSOR_SPIN ───────── 選ぶものはない
```

## 選ぶ値の表

| 選ぶもの | 値(1 番から) | params.h |
|---|---|---|
| SEARCH・SEARCH_SPIN の SPEED | 300，400，500，600，800，1000，1200，1500 | `SPEED_SELECT_SEARCH_V_MM_S` |
| SEARCH・SEARCH_SPIN の SLALOM | 300，400，500，600，700 | `SPEED_SELECT_SEARCH_TURN_V_MM_S` |
| SEARCH・FAST の ACCEL，STRAIGHT 系の ACCEL | 2000，3000，4000，5000，6000，8000，10000 | `SPEED_SELECT_ACCEL_MM_S2` |
| FAST・FAST_SWEEP の SPEED | 600，800，1000，1200，1400，1500 | `SPEED_SELECT_FAST_V_MM_S` |
| FAST・FAST_SWEEP の SMALL | 300，400，500，600，700 | `SPEED_SELECT_FAST_SMALL_V_MM_S` |
| STRAIGHT 系の SPEED | 300，400，500，600，800，1000，1200，1400，1500 | `SPEED_SELECT_STRAIGHT_V_MM_S` |
| SLALOM 系の速さ(s90) | 300，400，500，600，700，800，900 | `SPEED_SELECT_SLALOM_TEST_V_MM_S` |
| FAST_BANDS の BAND | 1〜6(帯の中身は mode_select.md) | `FAST_BANDS` |

## AUTO(自立賞)の今の設定

```
手かざし1回
 └─ 探索: 往復，Dijkstra，600 / スラローム 500，加速度 2000
     └─ 最短走行 × 4 本: 1200，加速度 2000，減速度 2000，小回り 600，SMALL
         ├─ 1〜3 本目の後: ゴールからスタートへ帰る(500)
         └─ 4 本目の後: ゴールから全面探索(600 / 500)をしてスタートへ帰り，地図を flash に残す
```

`AUTONOMOUS_SEARCH_*`，`AUTONOMOUS_FAST_*`，`AUTONOMOUS_FINAL_FULL_SEARCH`(params.h)．
ログは1本ごとに SD の `search/` へ流して残す(`search` → `fast` → `back` → … → `fast` → `search` の 9 ファイル)．
