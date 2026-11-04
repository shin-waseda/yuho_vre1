# tools/matlab — ログの確認とスラロームの係数合わせ(MATLAB 版)

Python のツール(`tools/turn_sim.py`，`get_log.py --plot-file`)の MATLAB 版と，ログビューア(配布版の `mouse_log_viewer` を yuho の設定で開く窓口)です．
Python のログビューア(`tools/log_viewer.py`)は 2026-11-04 に消した．
UART での受信と保存は今までどおり `tools/get_log.py` を使います(MATLAB は保存された `.csv` / `.bin` を読むだけ)．

計算は Python 版と同じです(`params.h` と `log_event.h` を読み，機体の `slalom.c`・`velocity_profile.c` と同じ台形で計算する)．
同じログで Python 版と結果が一致することを確かめてあります(2026-10-08，R2026b Prerelease)．

## 使い方

MATLAB でこのフォルダをパスに入れます(`cd` してもよい)．
`log_viewer` を使うには，配布版のログビューア(別のリポジトリ `M:/User/Desktop/school/club/log-viewer`)も入れておきます
(`release/` の `.mltbx` をダブルクリックするか，その `toolbox` を addpath する)．

```matlab
addpath('M:/User/STM32CubeIDE/workspace_1.19.0/yuho/tools/matlab')
addpath('M:/User/Desktop/school/club/log-viewer/toolbox')   % .mltbx を入れたなら要らない

turn_sim                                             % スラロームのシミュレータ(係数合わせ)
turn_sim({'logs/slalom/turn_0001.bin', 'logs/slalom/turn_0002.bin'})   % ログを開いた状態で起動
log_viewer                                           % ログを見る GUI(ファイルを選ぶ画面が出る)
log_viewer('logs/search/search_0008.bin')
plot_log                                             % ログを時系列のグラフにする(複数選ぶと重ねる)
plot_log('logs/vel_pid/step_0001.csv')
plot_log({'logs/slalom/turn_0001.bin', 'logs/slalom/turn_0002.bin'}, {'omega_ref', 'gyro_z'})
```

パスはリポジトリの一番上(`yuho/`)から見た形でも書けます．

## ファイル

| ファイル | 役割 |
|---|---|
| `turn_sim.m` | スラロームのシミュレータ．機体と同じ前後のオフセット，スリップ角，出口のずれ，柱との距離，ADJ の提案．ログを重ねて K と C を合わせる |
| `log_viewer.m` | ログを見る GUI．時系列(列の付け外し)，イベントの一覧と絞り込み，カーソル，再生，迷路の上の軌道と壁．本体は配布版の `mouse_log_viewer` で，ここは yuho の設定を渡して呼ぶだけ |
| `yuho_viewer_config.m` | 配布版のログビューアに渡す yuho の設定(列の組，`log_event.h` のイベント，軌道の基準と STEP での合わせ直し，機体の地図と読んだ壁) |
| `plot_log.m` | ログを段に分けたグラフにする(`get_log.py --plot-file` の代わり)．複数のファイルを重ねられる |
| `study/` | プラントモデルとシステム同定の演習(`docs/system_identification_study.md` の6章)．`addpath tools/matlab/study` してから使う |
| `+yc/` | 共通の部品(`yuho_common.py` に当たる)．`yc.load_log`，`yc.read_params`，`yc.simulate_turn`，`yc.TurnLog`，`yc.fit_slip` など |

コマンドウィンドウから部品だけ使うこともできます．

```matlab
L = yc.load_log('logs/search/search_0008.bin');      % 列名 → 列ベクトル．イベントがあれば L.ev_text も
plot(L.time_s, L.vl)
[idx, t, txt] = yc.find_events(L, 'STEP');           % STEP イベントの行・時刻・文字
p = yc.read_params();                                % params.h の値(式も計算する)
lg = yc.TurnLog('logs/slalom/turn_0001.bin', p);     % SLALOM テストの1回分
res = yc.fit_slip([lg, yc.TurnLog('logs/slalom/turn_0002.bin', p)], 8.0);   % K と C を合わせる
disp(res.text)
```

## Python 版との違い

- **K と C の合わせ方**: Python 版と同じ格子(K 0〜0.03 を 151 点，C 0〜0.2 を 101 点)で探した後，`fminsearch`(MATLAB 本体の関数．
  Toolbox は要らない)で格子の目より細かく詰める．`turn_sim` の「格子の後 fminsearch で細かく詰める」を外すと Python 版と同じ結果になる．
- **`log_viewer`**: 2026-10-20 に一般化して別のリポジトリ(log-viewer)へ移した．yuho に固有の取り決めは `yuho_viewer_config.m` にある．
  `log_viewer` を呼ぶと，この設定を `'yuho'` という名前で登録する(`mlv.add_profile`)．一度呼べば，`mouse_log_viewer` を直接開いても
  最後に使った設定(yuho)で開き，画面の上の「設定」で選び直せる．既定の設定で開くと，列の組は出る(よくある名前から作る組が yuho の組と同じになる)が，
  機体の地図・読んだ壁と，尻当て・STEP を使う軌道の基準は出ない．
  `+yc/load_log.m` などは `plot_log` と `turn_sim` が使うので残してある(配布版の `mlv.load_log` と中身は同じ．同じ結果になることを確かめてある)．
  グラフの拡大・移動は，軸の右上に出るツールバーで行う(ホイールで横に拡大する設定も入れたが，
  マウスでの操作はまだ確かめていない)．グラフをクリックしてカーソルを置くのは，ツールバーの拡大・移動を使っていないとき．
- **`plot_log`**: 列の組は `log_viewer` と同じ(組にない列は「その他」の段にまとめる)．複数のファイルは線の種類を変えて重ねる．
- **`turn_sim`**: ログのファイルを引数で渡して起動できる．
