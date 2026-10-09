# yuho 開発記録

マイクロマウス yuho(STM32F405RGT6)のファームウェアと PC ツールの開発記録．
立ち上げ(2026-06)から 2026-10-08 時点までの，実装の目的・経過・結果・設計の考え方・調整のやり方をまとめる．

- コミット済みの内容は `git log` のハッシュを付けた．
- 2026-10-08 時点で **未コミット** の内容(bbfbcf7 より後)は「未コミット」と書いた．
- 「未確認」と書いたものは，実機やログでまだ確かめていない．

---

## 目次

1. [機体とプロジェクトの概要](#1-機体とプロジェクトの概要)
2. [開発の進め方](#2-開発の進め方)
3. [設計の考え方](#3-設計の考え方)
4. [実装の解説(コードで読む)](#4-実装の解説コードで読む)
5. [開発の経過(時系列)](#5-開発の経過時系列)
6. [調整のやり方(手順集)](#6-調整のやり方手順集)
7. [ログの仕組みと使い方](#7-ログの仕組みと使い方)
8. [PC ツール](#8-pc-ツール)
9. [つまずいたことと教訓](#9-つまずいたことと教訓)
10. [今のパラメータ(2026-10-08)](#10-今のパラメータ2026-10-08)
11. [残っている課題](#11-残っている課題)
12. [開発日記(2026-10-08 以降)](#12-開発日記2026-10-08-以降)

---

## 1. 機体とプロジェクトの概要

| 項目 | 内容 |
|---|---|
| マイコン | STM32F405RGT6(Cortex-M4，168MHz) |
| 開発環境 | STM32CubeIDE 1.19.0(`yuho.ioc` から CubeMX でコード生成) |
| モーター | FAULHABER，エンコーダ IEH2-4096(4逓倍)，ギア 13:42 |
| タイヤ | 実測 φ23.35mm(走行に合わせた実効値 23.81mm) |
| トレッド | 設計 57.90mm(ジャイロに合わせた実効値 64.75mm) |
| ジャイロ | ICM-42688-P(SPI2，±2000dps) |
| 壁センサー | IR 4個(L，FL，FR，R)．L/R は斜め前向き．値は点灯 − 消灯の差 |
| 電池 | 2S LiPo(分圧 33k/20k で PC0) |
| 記録 | SD カード(SDIO 4bit + DMA + FatFs)，UART |
| 表示 | 直結 LED 6個 + 74HC595 のシフトレジスタの LED |
| 制御周期 | TIM6 の 1kHz 割り込み |

参考にした先行コード:
- **BlueEyes_Final**(過去機体，F303K8): 迷路の判断・探索の流れ(searchB，start_sequence，set_position)の元．
- **divergence_v3**(akiaki96): ログ機能の元．
- **akiaki96/maze_sim_py_c_v2**: 迷路シミュレータの構成の参考．

---

## 2. 開発の進め方

- **役割分担**: ビルド・書き込み・実機での走行はユーザーが行う．Claude はコードの編集(許可された範囲)，ログの解析，PC ツールの作成を行う．`.ioc` の編集とコード生成はユーザーが CubeIDE で行う．
- **ログで決める**: パラメータの多くは「走らせる → SD のログを取る → ログを見て直す」で決めた．経緯は `params.h` のコメントにも残している．
- **段階を踏む**: 低い速度で確かめてから上げる(例: 直進 300 → 800mm/s，探索 300 → 500mm/s)．
- **機体が使えない間は logic とシミュレータ**を進める(迷路の判断・最短経路は PC で先に確かめた)．

---

## 3. 設計の考え方

### 3.1 層の構成(詳しくは `architecture_discussion.md`)

```
Core/Inc, Core/Src
├── interface/   ハードに直結(HAL を呼んでよいのはここだけ)
├── logic/       計算(入力 → 出力の純粋な関数．グローバル変数・HAL_Delay を使わない)
│   ├── control/          PID，台形，壁の制御，スラロームの形
│   ├── maze/             壁の地図，Dijkstra，足立法，探索プランナー，最短の指令
│   └── state_estimation/ 運動学，オドメトリ
├── app/         手順・状態遷移(logic と interface をつなぐ)
├── global.h     共有の型
└── params.h     数値(ゲイン・寸法・しきい値)．式は logic に置く
```

- 判断の基準は「**副作用があるか**」．副作用のない計算はすべて logic に寄せる．
  そうすると，同じ C のソースを PC でコンパイルしてシミュレーションできる(`tools/maze_sim` がそうしている)．
- `interface` と `logic` は互いを include しない．`app` だけが両方を使う．
- 例外は HAL のコールバック(`HAL_TIM_PeriodElapsedCallback`)だけ．中身は書かず，`app` の関数を1回呼ぶ．

### 3.2 制御の考え方

- **カスケード制御**: 外側に位置(並進)と角度・角速度，内側に車輪速度の PI + FF を置く．
  ```
  v_cmd = v_ref + POSITION_KP × (s_ref − s)
  ω_cmd = ω_ref + ANGLE_KP × (θ_ref − θ_gyro) + ANGULAR_KP × (ω_ref − ω_gyro)
  車輪の目標 = v_cmd ∓ ω_cmd × TREAD/2
  電圧 = FF(目標だけから) + PI(車輪の目標 − 実速度)
  duty = 電圧 / 電池電圧
  ```
- **出力は電圧 [V]**: 電池で割って duty にするので，電池が減ってもゲインの効きが変わらない．
- **2自由度制御**: FF は目標(プロファイル)だけから計算し，実測を通さない．外側の補正(位置・角度)も FF には入れない．
- **常に効かせる**: 止まっている間も向きと位置を保つ．
- **閾値・不感帯で隠さない**: 振動(リミットサイクル)が出たら，閾値で制御を切るのではなくゲインで対処する(ユーザーの方針)．
- **目標は積分で作り，リセットしない**: θ_ref，s_ref は制御を有効にした時点を 0 にし，プロファイルを積分して作る．旋回ごとのずれが積み重ならない．
- **調整の目安は I 項**: FF が合っていれば，定常の I 項はほぼ 0 になる．I 項の符号と大きさから FF を直す．

### 3.3 迷路と走行の考え方

- **BlueEyes と同じ流れ**: 最初の半区画，境界で壁を読む，境界で経路を計算し直す，尻当て(set_position)．
- **地図のビット配置**: 1区画1バイトで BlueEyes と同じ．
  - 下位 4bit は探索用で，未知の壁は「ない」とみなす．
  - 上位 4bit は最短走行用で，未知の壁は「ある」とみなす．
  - 各 4bit は 北 0x8，東 0x4，南 0x2，西 0x1．
- **指令(アクションコマンド)を介す**: logic は「次にどう動くか」だけを返し，どう動くか(超信地かスラロームか)は実行側が決める．これでシミュレータと実機が同じ判断部分を使える．
- **最短経路は走行時間で選ぶ**(コストではなく)．
- **シミュレータでは軌道を作り込まない**: シミュレータの目的は経路の選び方の評価．軌道は実機側で作る(ユーザーの方針)．

### 3.4 ログの考え方

- **何でも記録する**: 制御の内部値・センサー・電池に加えて，イベント(モード，ボタン，指令，壁の判定，補正)も残す．
- **走りを止めない**: 長い走行は走りながら SD へ流す(DMA で待たない)．
- **マイコンは生のバイナリを書くだけ**: CSV への変換やグラフは PC がやる．

### 3.5 UI の考え方

- モードは右タイヤを回して選び，ボタンで決める．階層は RUN / TEST / SD．番号は 1 から(n 番 = LED n, n+1)．
- 走り出しは手かざし(FL のセンサー)．機体に触れずに始められるので，置いた位置がずれない．
- エラーは直結 LED の場所で知らせる．電圧低下は右後ろ，SD の失敗は左後ろ．

### 3.6 設計の判断と理由(思想)

3.1〜3.5 の「どう作ったか」の裏にある「なぜそうしたか」をまとめる．
それぞれ **考え**(何を選んだか)，**理由**(なぜか．もとになった出来事があればそれも)，**代わりに**(あきらめたこと・手間)の順に書く．

#### (1) 計算とハードを分ける

- **考え**: 副作用のない計算は，すべて logic 層の純粋な関数にする．HAL を呼ぶのは interface 層だけ．
- **理由**:
  - 過去機体(BlueEyes_Final)の反省．グローバル変数への依存が強く，割り込みの中で壁の制御則まで計算し，busy-wait でグローバルな状態を待つ作りだった．そのため，一部だけを取り出して試すことができなかった．
  - 将来は Simulink/MATLAB でシミュレーションや自動の調整をしたい．計算が純粋な関数なら，そのまま外へ持ち出せる．
  - 実際に，迷路の判断(`search_planner`，`dijkstra`)は同じ C のソースを PC でコンパイルして，大会の迷路 92 枚で確かめられた(`tools/maze_sim`)．
- **代わりに**: 値を構造体で受け渡す手間が増え，app 層に「つなぐだけ」のコードが増えた．

#### (2) 値と式を分ける

- **考え**: ゲイン・寸法・しきい値などの**数値**は `params.h` に，**式**は logic に置く．例えば PID の式は `pid.c`，Kp/Ki の値は `params.h`．
- **理由**:
  - 調整は数値を変えるだけで済み，式を壊す心配がない．
  - 値を変えた理由(どのログで何を見たか)を `params.h` のコメントに積み重ねていける．
  - PC のツール(turn_sim，maze_sim)が `params.h` を読んで機体と同じ値で計算できる．
- **代わりに**: `params.h` が長くなった(400 行)．

#### (3) 物理の単位で考える

- **考え**: 制御の中は V(電圧)，mm，deg，s で考える．出力も PWM ではなく電圧にして，最後に電池電圧で割る．
- **理由**:
  - 電池の減り方によらず，同じゲインが同じように効く．
  - FF の各項に意味が付く(「摩擦で 0.32V」「1mm/s あたり 0.00075V」)．ログを見て「どの項が足りないか」が分かる．
  - ログの値がそのまま読める．
- **例外**: 壁センサーは AD の値のまま使い，mm に直さない．
  - 理由: 直すには距離ごとに測る手間がかかる(ユーザーの判断)．しきい値や，基準からのずれ(壁の制御)には mm は要らない．
  - 代わりに: 横の位置を mm で知りたいとき(スリップの見積もり)は，傾きをログから見積もる必要がある．

#### (4) FF が主役で，FB は残りを直す

- **考え**: 電圧のほとんどを FF(目標だけから計算)で出し，PI は見込みとのずれだけを直す(2自由度制御)．
- **理由**:
  - FB だけだと，ずれが出てから直すので必ず遅れる．FF なら最初から目標どおりの電圧を出せる．
  - FF が合っていれば定常の I 項は 0 になるので，**I 項が FF の正しさを測る目盛り**になる．調整の方法がはっきりする(6.1)．
  - FF を目標だけから作ると，止まっているときに補正の揺れで摩擦の符号が反転する，という問題が起きない(4.6)．
- **進め方**: まず FF をある程度まで合わせ，大会までの時間を考えて途中で切り上げ，残りは FB(位置・角度の制御)で受け持つ方針にした．
- **代わりに**: 駆動系の摩擦が変わる(ねじを締め直す，ぶつける)たびに，FF を合わせ直す必要がある．

#### (5) 隠さずに直す

- **考え**: 振動やずれを，閾値・不感帯・「止まったら制御を切る」で隠さない．振動が出たらゲインで直す．
- **理由**:
  - ユーザーの方針．「リミットサイクルは PID のゲイン調整でどうにかする場所だから」．
  - 閾値で切ると，問題が見えなくなる．しかも切り替えの境目に別の不連続ができる．
- **実際の例**:
  - 止まっている間も角度・位置の制御を効かせ続ける．PARTY モードで床ごと回されても戻れるのはこのため．
  - 超信地旋回のずれは，閾値ではなく `ANGLE_KP` を 15 → 25 に上げて減らした．
- **代わりに**: ゲインの調整が要る．構造の問題は，ゲインに頼らず別に直す(FF の符号の問題は 2自由度の形で直した)．

#### (6) 目標は絶対値で持ち，積み重ねない

- **考え**: 目標の向き・目標の距離は，制御を有効にした時点からの積分で持ち，動きごとにリセットしない．探索でも，次の境界の位置を「目標の距離の絶対値」で決める．
- **理由**:
  - 1回ごとの小さなずれ(旋回の終わりの 0.5° など)が積み重ならず，次の動きの間に直される．
  - 経路の計算に 20〜25ms かかっても，その間に進んだ分で位置がずれない．
- **同じ考えで直したもの**: 壁の制御は初め「向きを回す速さ」(向きを積み重ねる形)だった．大きく寄ったときに向きが 8.7° 回ったまま残り，壁にぶつかった．「迷路の軸 + オフセット」の形にして，真ん中に戻れば向きも戻るようにした(4.10)．
- **代わりに**: 尻当てのように「ここを新しい基準にする」場面では，基準を明示的に取り直す処理が要る．

#### (7) 機体と同じ計算を PC でも

- **考え**: 機体が実際に使う計算(1ms 刻みの台形，スラロームの形)を，式で近似せず，同じ手順で PC でも計算する．
- **理由**:
  - 「シミュレーションでは合っていたのに実機と違う」を減らす．離散化の誤差まで一致させれば，違いは物理(スリップなど)だけになる．
  - スラロームの前後のオフセットも，機体自身が同じ台形を 1ms ずつ進めて計算している．速さやパラメータを変えても，計算し直すだけで合う．
- **ただし目的で使い分ける**: 迷路のシミュレータ(maze_sim)では軌道を作り込まない．目的は経路の選び方の評価で，軌道は関係ないため(ユーザーの方針)．スラロームの形まで合わせるのは，それ自体を調べる turn_sim だけ．

#### (8) 判断と動きを指令で分ける

- **考え**: 探索の判断(logic)は「次は右へ」のような指令(`Action`)だけを返す．どう曲がるか(超信地旋回かスラロームか)は，実行する側(app)が決める．
- **理由**:
  - 同じ判断を，シミュレータと実機の両方で使える．
  - 曲がり方を増やしても，判断のコードを変えずに済む．実際，スラロームを足したときにプランナーは変えていない．
- **代わりに**: 判断の側は「曲がるのに時間がかかる」ことを知らない．走行時間で経路を選ぶ最短走行では，時間の見積もり(`run_path`)を別に持っている．

#### (9) 実績のある流れに合わせる

- **考え**: 探索の手順(最初の半区画，境界で壁を読む，境界で経路を計算し直す，尻当て)と地図のビット配置は，BlueEyes と同じにする．
- **理由**:
  - 動くと分かっている手順を使えば，迷う所が減る．
  - 比べやすい．おかしな動きをしたとき，BlueEyes と同じかどうかで切り分けられる．
- **代わりに**: yuho の流儀(層の分け方)に合わせて，書き方は作り直した．

#### (10) 測ってから決める(ログが中心)

- **考え**: パラメータは推測で決めず，走らせたログで決める．何でも記録する．記録のために走りを変えない．
- **理由**:
  - 測らずに決めた値は，どこが間違っているか後から分からない．ログがあれば，I 項・ずれ・イベントから原因を調べられる．
  - **イベントも記録する**: 「いつ何が起きたか」(壁を読んだ，補正した，曲がり始めた)がないと，値の変化の理由が分からない．
  - **記録のために止まらない**: 初めは「止まって SD に追記する」方式だった．しかし最短走行は止まらないので記録しきれない．また，記録のために止まると，測りたい動きそのものが変わってしまう．そこで，走りながら DMA で流す方式にした．
  - **マイコンは生のデータを書くだけ**: 文字にする・グラフにする・意味を付けるのは PC がやる．マイコンの時間を制御に使える．イベントの番号表も `log_event.h` の1か所だけにして，PC はそれを読む．
- **記録の範囲**: 常に記録するのではなく，手かざしで走り出してから止めるまでにした．ファイルが走り1回ずつに分かれて，整理しやすい(ユーザーの提案)．
- **代わりに**: 調べるための PC のツール(get_log，log_viewer，turn_sim)が要った．

#### (11) 小さく，同じ条件で確かめる

- **考え**: 動きごとに試験モード(STRAIGHT，PIVOT，SLALOM，SENSOR_LOG，VEL_PID)を作り，低い速さから確かめて上げていく．試験の前には尻当てで置き方をそろえる．
- **理由**:
  - 1つずつ確かめれば，問題がどこにあるか分かる．
  - 置き方がばらつくと，結果のばらつきが機体のせいか置き方のせいか分からない．スラロームの試験で柱に当たったのは，置き方のずれだった．
- **代わりに**: 試験モードが増え，モードを階層(RUN / TEST / SD)に分けることになった．

#### (12) 失敗したら安全な側へ

- **考え**:
  - フェイルセーフは一度発動したら戻らない(ラッチ)．割り込みが毎 tick モーターを止め直す．
  - 待ちにはすべてタイムアウトを付ける．
  - ログの保存に失敗しても走りは続け，LED で知らせる．
- **理由**:
  - 異常のときに勝手に動き出さない．メインが誤って制御を有効にしても止まる．
  - ログは走りの役に立つためのもので，ログのせいで走りを止めない．
- **代わりに**: 発動したらリセットが要る．

#### (13) 割り込みとメインの約束を守る

- **考え**:
  - 1つの変数を書くのは，片方(メインか割り込み)だけにする．
  - 指令はパラメータを書いてから最後にフラグを立てる．
  - 割り込みの中では待たない(書けなければ捨てて数える)．
- **理由**:
  - 割り込みはいつ入るか分からないので，約束がないと「書きかけを読む」不具合が起きる．この種の不具合は再現しにくい．
  - 割り込みの中で待つと，制御の周期(1ms)が崩れる．
- **代わりに**: 「0 にしてほしい」をフラグで頼む，のような回りくどい書き方が要る(4.9)．

#### (14) 生成されたコードを触らない

- **考え**: CubeMX が作ったファイルは，`USER CODE BEGIN/END` の中以外は書き換えない．生成コードの不具合は，`__weak` の関数を同じ名前で定義し直して直す(SD の待ち)．`.ioc` の変更とコードの生成はユーザーが CubeIDE で行う．
- **理由**: 生成し直すと，書き換えた所は消える．直したことが消えると気づきにくい．
- **代わりに**: 直し方が回りくどくなることがある．FatFs の設定(`_USE_EXPAND`)を変えずに済むよう，SD の領域の確保を `f_lseek` で行った(4.16)．

#### (15) 機体だけで使えるように

- **考え**:
  - 走り出しは手かざし．
  - 状態は LED の場所で知らせる(電圧低下は右後ろ，SD の失敗は左後ろ)．
  - ログは UART の線がなくても SD に残る．
  - 走る前の設定(曲がり方，速さ)は機体の上で選ぶ．
- **理由**:
  - 大会や迷路では PC をつなげない．
  - ボタンを押すと機体が動いて，置いた位置がずれる．手かざしならずれない．
- **代わりに**: 選べるものが増えるほど，操作の手順が長くなる．

#### (16) やらないと決めたこと

| やらないこと | 理由 |
|---|---|
| 壁センサーの値を mm に直す | 距離ごとに測る手間がかかり，今の使い方には要らない |
| 迷路のシミュレータで軌道を作り込む | シミュレータの目的は経路の評価で，軌道は実機側の仕事 |
| マイコンで CSV を作る | 遅く，ファイルも大きくなる．PC でやれば済む |
| ログを常に取る | 走り1回ずつに分かれていた方が整理しやすい |
| 帰りの探索の A 案 | 今は優先度が低い(保留) |

---

## 4. 実装の解説(コードで読む)

この章では，実際のコードを引用しながら「何をしているか」と「なぜそう作ったか」を説明する．
引用は要点だけを抜き出したもので，`...` は省略を表す．ファイルの場所は見出しに書いた．
数値は 2026-10-08 時点の `params.h` の値．

### 4.1 1ms の割り込みの流れ(`interface/timer.c`)

制御のすべては TIM6 の 1kHz の割り込みの中で動く．HAL が名前を決めているコールバックは interface に置き，中身は書かずに app の関数を呼ぶだけにしている(3.1 の唯一の例外)．

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM6) return;

  Sensor_ReadAll();   // 1. 壁センサーと電池電圧を読む
  App_ControlTick();  // 2. 制御を1周期進める(目標 → 補正 → PWM)
  Logger_Sample();    // 3. このtickで確定した値を記録する
}
```

順番に意味がある．センサーを読んでから制御するので，制御は「今の tick の」センサー値を使う．制御の後に記録するので，ログの1行は「その tick の入力・目標・出力」がそろった状態になる．

メイン(`while` のループ，探索の手順など)は割り込みの合間に動き，目標を渡したり，終わるのを待ったりするだけ．計算の重い経路探索(`SearchPlanner_Step`，20〜25ms)もメイン側で動くので，制御は止まらない．

### 4.2 壁センサーの読み方(`interface/sensor.c`)

IR の LED を消した値と点けた値の**差**を使う．外の光(蛍光灯・日光)の影響を打ち消すため．

```c
// --- 1. 全LED消灯でOFF値を取得 ---
...
r_off  = adc_next();  fr_off = adc_next();  fl_off = adc_next();  l_off = adc_next();
vabt   = adc_next();

// --- 2. R だけ点けて読む ---
HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
tim6_wait_us(IR_SETTLE_US);          // 50us 待って光を安定させる
r_on = adc_next();
HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
... (FR, FL, L も同じ)

// --- 6. 差分(アンダーフローしないように) ---
ad_r = (r_on > r_off) ? (r_on - r_off) : 0;
```

- 4つの LED を**1つずつ**点ける．同時に点けると，隣のセンサーの光を拾ってしまう．
- 待ち時間 50us は TIM6 のカウンタ(1MHz)で数える．割り込みの中なので `HAL_Delay` は使えない．
- ADC は「次の rank を1つだけ変換する」設定(Discontinuous)にしてあり，`adc_next()` を呼ぶたびに R → FR → FL → L → 電池 の順に進む．

この値(`ad_l` など)は mm に直さず，そのまま使う(5.14 の方針)．

### 4.3 エンコーダとオドメトリ(`interface/encoder.c`，`params.h`)

エンコーダのカウンタ(TIM4/TIM8)は 16bit で回り続ける．絶対値ではなく，**前回からの差**を使う．

```c
int16_t Encoder_GetDeltaL(void) {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(now - s_last_l); // 16bitのラップアラウンドを利用した差分計算
    s_last_l = now;
    return delta;
}
```

`uint16_t` どうしの引き算を `int16_t` にすると，65535 → 0 と回り込んでも正しい差になる(1ms で ±32767 パルスを超えなければよい)．

パルスから距離への換算:

```c
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_SPUR_T / GEAR_PINION_T)
//                            = 4096 × 4 × 42 / 13 ≈ 52935 [パルス/タイヤ1回転]
#define MM_PER_PULSE ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_PULSES_PER_REV)
//                    ≈ 0.00141 mm/パルス
```

エンコーダはモーターの軸に付いているので，ギア比を掛ける．`WHEEL_DIAMETER_MM` はノギスの値ではなく，走らせた距離に合わせた実効値(6.3)．

### 4.4 運動学: 車輪の速さ ↔ 機体の速さ(`logic/state_estimation/kinematics.c`)

```c
WheelVelocity Kinematics_RobotToWheel(RobotVelocity robot) {
    float turn_term = (robot.angular_rad_s * TREAD_WIDTH_MM) / 2.0f;
    WheelVelocity w;
    w.left_mm_s  = robot.linear_mm_s - turn_term;
    w.right_mm_s = robot.linear_mm_s + turn_term;
    return w;
}
```

左右の車輪の速さ v_L, v_R と，機体の並進 v・回転 ω の関係は

- v = (v_L + v_R)/2
- ω = (v_R − v_L)/T(T はトレッド，反時計回りが +)

`TREAD_WIDTH_MM` も設計値(57.90)ではなく，ジャイロで測った回り方に合わせた実効値(64.75)．タイヤの接地点のずれや横滑りを含む．

### 4.5 PID のコア(`logic/control/pid.c`)

```c
float PID_Update(PID_t *pid, float error, float dt, float output_min, float output_max) {
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output <= output_max && output >= output_min) {   // 出力が飽和していないときだけ
        pid->integral += error * dt;                        // 積分を進める
        output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    }

    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;
    return output;
}
```

式は u = Kp·e + Ki·∫e dt + Kd·de/dt．

- **積分の止め方(アンチワインドアップ)**: 出力が上限・下限に張り付いている間は積分を進めない．張り付いている間も積分を続けると，I 項が溜まりすぎ，目標に届いた後に大きく行き過ぎる．
- **微分は誤差の微分**．今は `VELOCITY_KD = 0` で，微分は使っていない(P と I で足りている)．
- この関数は**状態(`integral`，`prev_error`)を構造体に持つ純粋な計算**で，HAL もグローバル変数も使わない．だから logic 層にあり，PC でもテストできる．

車輪速度の PI は，左右に1つずつこの `PID_t` を持つ(`logic/control/velocity_pid.c`)．

### 4.6 車輪速度のループと FF(`app/control_loop.c` の `App_ControlTick`)

制御の出力はモーターに**かける電圧 [V]** で，最後に電池電圧で割って PWM にする．

```c
// 2自由度制御: FFは目標(速度・加速度)だけから、PIDは偏差から。単位はどちらも電圧[V]。
float ff_l = VELOCITY_FF_FRIC_L * SignOf(target_wheel_ff.left_mm_s)    // 摩擦(速さによらず一定)
           + VELOCITY_FF_GAIN_L * target_wheel_ff.left_mm_s             // 速さに比例(逆起電力など)
           + VELOCITY_FF_ACC_L  * target_wheel_acc.left_mm_s;           // 加速に要る分
...
out = VelocityPID_Update(&s_vpid, target_wheel, s_actual, CONTROL_DT_S);   // 補正(PI)
s_pwm.left = VoltageToPWM(ff_l + out.left_mm_s, vbat);
```

```c
static int16_t VoltageToPWM(float volt, float vbat) {
    float pwm = volt / vbat * (float)PWM_MAX;    // duty = 電圧 / 電池電圧
    ...
}
```

**なぜ電圧で考えるか**: DC モーターの式はおおよそ「電圧 = 摩擦の分 + 速さに比例する分 + 加速に比例する分」になる．PWM の duty で考えると，電池が 8.2V から 7.4V に下がるだけで同じ duty でも出る電圧が 10% 変わり，ゲインの効き方がずれる．電圧で考えて最後に電池電圧で割れば，電池の減り方によらず同じ効き方になる．

**なぜ FF を目標だけから作るか(2自由度制御)**: FF は「この目標ならこれくらいの電圧が要る」という見込みで，ほとんどの電圧を FF が出し，PI は見込みとのずれだけを直す．FF に実測や外側の補正を混ぜると，止まっているとき(目標 0，補正がほぼ 0)に補正の小さな揺れで `SignOf` の符号が入れ替わり，摩擦の電圧(±0.3V)がバタバタと反転してしまう．

```c
// 摩擦FF用の符号。目標0のときは0(止まっている所に摩擦ぶんの電圧をかけない)。
static float SignOf(float v) {
    if (v > 0.0f) return 1.0f;
    if (v < 0.0f) return -1.0f;
    return 0.0f;
}
```

**超信地旋回だけの摩擦 FF**: その場で回るときはタイヤが横にこすれるので，直進用の式では 0.75〜0.84V 足りなかった．回る向きで要る電圧も違ったので，4つに分けている．

```c
if (s_target_mm_s == 0.0f && s_target_acc == 0.0f && s_target_omega_dps != 0.0f) {
    if (s_target_omega_dps > 0.0f) {   // 反時計回り: 右の車輪が前へ、左が後ろへ
        ff_l -= PIVOT_FF_FRIC_CCW_L;
        ff_r += PIVOT_FF_FRIC_CCW_R;
    } else { ... }
}
```

**FF が合っているかの見分け方**: ログの `i_l`，`i_r`(I 項の電圧)を見る．FF がぴったりなら，定常の I 項は 0 付近になる．調整のやり方は 6.1．

### 4.7 外側のループ: 角度と位置(`app/control_loop.c`)

車輪速度のループの外側に，機体の向き(ジャイロ)と進んだ距離(エンコーダ)のループがある．出力は「車輪速度の目標への上乗せ」．

```c
// ---- 位置のループ(外側、並進方向) ----
// v_cmd = v_ref + POSITION_KP×(s_ref − s)
pos_corr = POSITION_KP * (s_pos_ref_mm - s_dist_mm);           // 上限 ±200mm/s

// ---- 角度・角速度のループ(外側) ----
// ω_cmd = ω_ref + ANGLE_KP×(θ_ref − θ) + ANGULAR_KP×(ω_ref − ω)
ang_corr = ANGLE_KP * angle_err_deg * DEG_TO_RAD + ANGULAR_KP * (omega_ref - omega_meas);

RobotVelocity target_robot = {
    .linear_mm_s = s_target_mm_s + pos_corr,
    .angular_rad_s = omega_ref + ang_corr,
};
WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);
```

**目標の向き・距離の作り方**: 目標の向き θ_ref と目標の距離 s_ref は，制御を有効にした時点を 0 とし，プロファイルの目標を毎 tick 積分して作る．

```c
if (control_active) {
    s_angle_axis_deg += s_target_omega_dps * CONTROL_DT_S;   // 目標の向き(迷路の軸)
    s_pos_ref_mm     += s_target_mm_s * CONTROL_DT_S;        // 目標の距離
    s_dist_mm += 0.5f * (s_actual.left_mm_s + s_actual.right_mm_s) * CONTROL_DT_S; // 進んだ距離
    ...
} else {
    s_angle_axis_deg = s_gyro_angle_deg;   // 無効の間は今の向きに合わせておく
    s_pos_ref_mm = 0.0f;  s_dist_mm = 0.0f;
}
```

止まっても目標をリセットしないので，90° 回るたびに 0.5° ずつずれる，のようなずれが積み重ならない(旋回の終わりのずれは，次の動きの間に直される)．

**角度のずれが縮む速さ**: 内側(車輪速度)が十分速く，ω = ω_cmd になるとする．角度の誤差を e = θ_ref − θ とすると de/dt = ω_ref − ω なので，

```
ω_ref − ω = −K_θ·e − K_ω·(ω_ref − ω)
→ (1 + K_ω)(ω_ref − ω) = −K_θ·e
→ de/dt = −K_θ / (1 + K_ω) · e
```

誤差は時定数 (1 + K_ω)/K_θ = (1 + 0.5)/25 = 0.06 秒で縮む．同じように，位置の誤差は時定数 1/K_p = 1/10 = 0.1 秒で縮む．

**止まっている間も効かせる**: 止まったら制御を切る(閾値で I 項を捨てるなど)作り方もあるが，yuho では切らない．振動が出たらゲインで直す(3.2)．PARTY モードで床ごと回しても同じ向き・位置に戻るのは，これが常に効いているから．

**尻当ての間は止める**: 壁に押し当てている間は，機体が壁にそろって回るのを打ち消さないよう，向きと位置の補正を止める(`s_wall_push`)．

### 4.8 目標の作り方: 台形プロファイル(`logic/control/velocity_profile.c`)

直進・超信地旋回・スラロームの角速度は，すべて同じ台形のコードで作る．超信地旋回とスラロームでは「距離 = 角度 [deg]，速さ = 角速度 [dps]」として使う．

```c
void VelocityProfile_Step(VelocityProfile *p, float dt) {
    float remaining = p->distance_mm - p->pos_mm;

    // 減速開始の判定: 今の速度からv_endまで減速するのに要る距離に、1tick先の位置が入ったら減速に入る
    if (!p->decelerating) {
        float decel_dist = (p->v * p->v - p->v_end * p->v_end) / (2.0f * p->accel);
        if (remaining - p->v * dt <= decel_dist) p->decelerating = true;
    }

    float a_cmd;
    if (p->decelerating) {
        // 残りの距離でちょうどv_endになる減速度を毎tick計算し直す
        a_cmd = -(p->v * p->v - p->v_end * p->v_end) / (2.0f * remaining);
        if (a_cmd < -PROFILE_DECEL_MARGIN * p->accel) a_cmd = -PROFILE_DECEL_MARGIN * p->accel;
    } else if (p->v < p->v_max) {
        a_cmd = p->accel;      // 加速
    } else {
        a_cmd = 0.0f;          // 等速
    }
    float v_next = p->v + a_cmd * dt;
    ...
    p->pos_mm += 0.5f * (p->v + v_next) * dt;   // 台形の面積で進める
```

- **減速度を毎 tick 計算し直す理由**: 減速度を固定にすると，1ms 刻みの誤差で，終点に速度が残ったまま着いたり，手前で止まったりする．「残りの距離 d で v から v_end にする」減速度 a = (v² − v_end²)/(2d) を毎回計算すれば，ぴったり終点で v_end になる．上限(指定の 1.5 倍)は，計算が暴れたときの保険．
- **`v_start` で直進をつなぐ**: 走りながら次の直進を始めると，今の速さから始まる(`UpdateProfile` で `v_start = s_target_mm_s`)．探索で区画ごとに止まらずに走れるのはこのため．

### 4.9 メインと割り込みの受け渡し(`app/control_loop.c`)

メインと割り込みは同じ変数を触るので，壊れないように順番を決めている．

**動きの指令**: パラメータを書いてから，最後にフラグを立てる．割り込みは次の tick でフラグを見て取り込む．

```c
void App_StartStraight(float distance_mm, float v_max, float v_end, float accel) {
    s_pending.type = MOTION_STRAIGHT;
    s_pending.distance = fabsf(distance_mm);
    ...
    s_motion_done = false;
    s_start_pending = true; // 最後に立てる
}
```

フラグを先に立てると，パラメータを書いている途中で割り込みが取り込んでしまう可能性がある．

**割り込み側の値を読む**: 壁切れの位置などは，割り込みが「最後に通し番号を進める」．メインは番号が読む前と後で同じかを確かめ，違えば読み直す．

```c
uint32_t App_GetWallEdge(uint8_t *side, float *pos_ref_mm) {
    uint32_t seq;
    do { // ISR が途中で書き換えたら読み直す
        seq = s_edge_seq;
        if (side != NULL) *side = s_edge_side;
        if (pos_ref_mm != NULL) *pos_ref_mm = s_edge_pos_ref;
    } while (seq != s_edge_seq);
    return seq;
}
```

**割り込み側の変数を 0 にしたいとき**: ジャイロの積算角はメインが直接 0 にせず，「0 にしてほしい」というフラグを立て，割り込みが 0 にしてからフラグを下ろす．割り込みが足し込んでいる途中で 0 にされる，ということが起きない．

### 4.10 壁の制御(`logic/control/wall_control.c`)

```c
float err_l = (float)v.l - (float)WALL_REF_L; // 正: 左の壁に近い
float err_r = (float)v.r - (float)WALL_REF_R; // 正: 右の壁に近い
float err;
if (use_l && use_r)  err = err_l - err_r;      // 両方: 左右の差
else if (use_l)      err = 2.0f * err_l;       // 片方だけ: その側の差の2倍
else if (use_r)      err = -2.0f * err_r;
else                 return 0.0f;              // 壁なし: オフセット 0(軸の向きへ戻る)

float speed = (speed_mm_s > WALL_KP_MIN_V_MM_S) ? speed_mm_s : WALL_KP_MIN_V_MM_S;
float kp = WALL_KP_DEG * (WALL_KP_REF_V_MM_S / speed);   // 速いほど弱く
float offset = -kp * err;                                // 左に寄っていれば右へ向ける
```

制御の側では，目標の向きを「迷路の軸 + オフセット」にする．

```c
s_angle_ref_deg = s_angle_axis_deg + s_wall_offset_deg;
```

**なぜオフセットの形か**: 最初は「誤差に比例して向きを回す速さ」(向きを積分する形)にしていた．すると大きく寄ったときに向きが 8.7° 回ったまま残り，曲がった後に斜めのまま壁にぶつかった(5.15 の記録の経緯)．オフセットの形なら，真ん中に戻れば(誤差 0)オフセットも 0 になり，向きは迷路の軸に戻る．

**なぜゲインを 1/v にするか**: 横に戻る速さは「速さ × sin(オフセット)」なので，同じオフセットでも速いほど速く戻る．ゲインを 1/v にすると，真ん中に戻るまでの時間が速さによらず同じになる．

**壁の切れ目では使わない**: 2mm 進むごとに値を比べ，15 を超えて変わった側は 40mm の間使わない．切れ目や柱の横では値が急に変わり，「壁が遠い」と区別がつかないため．

```c
if (dist_mm - s->last_dist_mm >= WALL_EDGE_STEP_MM) {
    int32_t diff = (int32_t)value - (int32_t)s->last_value;
    if (diff > WALL_EDGE_DIFF || diff < -WALL_EDGE_DIFF) {
        s->hold_until_mm = dist_mm + WALL_EDGE_HOLD_MM;
    }
    ...
}
return value > threshold && dist_mm >= s->hold_until_mm;
```

オフセットを変える速さには上限(90dps)を付けている．壁が切れたときなどに，目標の向きが跳ばないように．

### 4.11 壁切れ補正(`app/control_loop.c`，`app/search_run.c`)

横の壁が切れた瞬間は，機体が区画の境界を決まった距離(`WALL_EDGE_POS_MM` = 78mm，仮)だけ過ぎた所にいるはず．そこで距離の数え方のずれを直す．

割り込みで「壁あり → 壁なし」に変わった瞬間を見つける:

```c
if (value > th) {                       // 壁あり
    if (!t->above) { t->above = true; t->above_from = s_pos_ref_mm; }
    return;
}
if (t->above) {                         // 壁あり → 壁なし に変わった
    t->above = false;
    if (s_pos_ref_mm - t->above_from >= WALL_EDGE_MIN_WALL_MM) {   // 30mm 以上続いた壁だけ
        s_edge_side = side;
        s_edge_pos_ref = s_pos_ref_mm;
        s_edge_seq++; // 最後に進める(メインはこれが変わったら読む)
    }
}
```

メイン側で，一番近い境界からのずれを求めて，次の境界の位置をずらす:

```c
// 一番近い境界(boundary0 + 1区画 × k)を選ぶ
float k = floorf((pos - WALL_EDGE_POS_MM - ec->boundary0) / SECTION_MM + 0.5f);
float expected = ec->boundary0 + k * SECTION_MM + WALL_EDGE_POS_MM;
float c = pos - expected;
if (c > WALL_EDGE_WINDOW_MM || c < -WALL_EDGE_WINDOW_MM) return false; // 予想から外れすぎ(使わない)
if (c > WALL_EDGE_MAX_CORR_MM) c = WALL_EDGE_MAX_CORR_MM;               // 1回に直すのは ±20mm まで
...
*corr = c;   // 呼び出し側が、待つ位置(次の境界)を c だけずらす
```

- 柱の横で一瞬だけ跳ねた値に引っかからないよう，その前に 30mm 以上壁が続いたときだけ数える．
- 予想から ±30mm より外れた壁切れは，読み違いか別の壁とみなして使わない．
- 結果はイベント `EDGE_CORR`(expected，edge_pos)に残るので，`WALL_EDGE_POS_MM` はログを見て合わせる．

### 4.12 スラロームの形(`logic/control/slalom.c`)

スラロームは，並進の速さ v を保ったまま，角速度だけを台形で動かして曲がる．曲がる前後の直進(オフセット)の長さを，**機体と同じ台形を 1ms ずつ進めて**求める．

```c
SlalomShape Slalom_ComputeShape(const SlalomParams *p) {
    VelocityProfile prof;
    VelocityProfile_Start(&prof, p->angle_deg, 0.0f, p->omega_dps, 0.0f, p->alpha_dps2);
    float theta = 0.0f;
    while (!prof.done && ticks < 100000u) {
        VelocityProfile_Step(&prof, CONTROL_DT_S);   // ISR と同じ角速度の台形
        theta += prof.v * CONTROL_DT_S;              // その tick の向き
        float th = theta * DEG_TO_RAD;
        s.forward_mm += p->v_mm_s * cosf(th) * CONTROL_DT_S;   // 曲がり始めの向きへ進んだ距離
        s.side_mm    += p->v_mm_s * sinf(th) * CONTROL_DT_S;   // 曲がる側へ進んだ距離
        ...
    }
}

void Slalom_Turn90Offsets(const SlalomShape *s, float span_mm, float *pre_mm, float *post_mm) {
    *pre_mm = span_mm - s->forward_mm;    // 前のオフセット
    *post_mm = span_mm - s->side_mm;      // 後ろのオフセット
}
```

- 小回り 90° は span = 半区画(境界の真ん中 → 隣の境界の真ん中)．500mm/s・450dps・8000dps² で pre ≈ 12.3mm，post ≈ 11.4mm．
- 大回り 90° は span = 1区画(区画の中心 → 斜め隣の中心)．
- 大回り 180° は「横にちょうど 1区画移る」角速度を二分法で求める．
- **なぜ数式ではなく数値で積分するか**: 機体は 1ms ごとに台形を進めて曲がるので，同じ計算をすれば離散化の誤差まで一致する．PC の turn_sim も同じ計算をしていて，スリップ 0 なら出口のずれは 0.00mm になる．

**速さを変えても形を保つ**:

```c
void Slalom_ScaleToSpeed(SlalomParams *p, float v_mm_s) {
    float k = v_mm_s / p->v_mm_s;
    p->omega_dps *= k;        // 曲率 κ = ω / v を保つ
    p->alpha_dps2 *= k * k;   // 角速度の台形を「進んだ距離に対して」同じ形にする
    p->v_mm_s = v_mm_s;
}
```

### 4.13 探索の流れ(`app/search_run.c`，`logic/maze/search_planner.h`)

判断(どこへ行くか)は logic 層の `SearchPlanner`，動き(どう走るか)は app 層が受け持つ．その間は，壁の観測 `WallObservation` と指令 `Action` だけでやり取りする．

```c
while (1) {
    Action act = SearchPlanner_Step(&s_planner, obs);   // 今の区画の壁を渡して、次の指令をもらう
    Record(pos, heading, obs, sv, act, plan_ms);        // STEP イベントに残す
    switch ((ActionType)act.type) {
        case ACTION_FORWARD:
            ok = GoToNextBoundary(&dp, act.cells, &obs, &sv);    // 次の境界まで走り、壁を読む
            break;
        case ACTION_TURN_RIGHT:
        case ACTION_TURN_LEFT:
            if (s_search_slalom && !dp.at_center) ok = SlalomTurn(&dp, ...);   // 止まらずに曲がる
            else ...                                                            // 真ん中で超信地旋回
        ...
    }
}
```

- `SearchPlanner_Step` は PC のシミュレータ(`tools/maze_sim`)でも同じものが動く．シミュレータでは「壁を読む」を迷路のファイルから作り，「走る」を位置を進めるだけにしている．
- 壁は区画の境界で読む．境界に着いた時点で，センサーは入ろうとする区画の壁を見ている．
- 距離は「目標の距離の絶対値」(`dp->ref_mm`)で決める．経路の計算に時間がかかっても，その間に進んだ分が積み重ならない．

### 4.14 フェイルセーフ(`app/failsafe.c`)

```c
// 条件が成立している間カウントを進め、limit_msに達したらtrueを返す。
static bool Persist(uint16_t *counter_ms, bool cond, uint16_t limit_ms) {
    if (!cond) { *counter_ms = 0; return false; }
    if (*counter_ms < limit_ms) (*counter_ms)++;
    return *counter_ms >= limit_ms;
}
...
if (Persist(&s_low_voltage_ms, s_vbat_filtered < FAILSAFE_LOW_VOLTAGE_V, FAILSAFE_LOW_VOLTAGE_MS)) {
    FailSafe_Trip(FAILSAFE_LOW_VOLTAGE, s_vbat_filtered);
}
```

- 「続けて○ms 成り立ったら」発動する．加速した瞬間の電圧の落ち込みなど，一瞬のことでは止まらない．
- 一度発動したら，リセットまで戻らない(ラッチ)．割り込みが毎 tick モーターを止め直すので，メインが誤って有効にしても止まる．
- 原因は直結 LED で知らせる(電圧低下は右後ろ)．

### 4.15 SD カード(1): ふつうの書き方(`interface/sdcard.c`)

試験モードの保存や SD_DUMP では，FatFs(`f_open`，`f_write`，`f_close`)をそのまま使う．

```c
FRESULT res = f_open(&s_file, path, FA_CREATE_NEW | FA_WRITE);   // 新しい番号のファイルを作る
...
FRESULT res = f_write(&s_file, data, len, &written);
...
f_close(&s_file);
```

**ファイルの番号**: `dir/prefix_NNNN.bin` の NNNN を 1 から順に探し，`.csv` でも `.bin` でも，送った後の `sent/` の中でも使われていない番号にする．PC で `.bin` から `.csv` を作ったとき，昔の同じ番号の `.csv` とぶつからないようにするため．

**CubeMX が作ったコードの不具合を直す**: 生成された `sd_diskio.c` は，カードが前の書き込みを内部で処理している間(PROGRAMMING，数 ms)に次の操作を始めると，`FR_INVALID_OBJECT` で失敗する．生成ファイルは触らず(再生成で消えるため)，`bsp_driver_sd.c` の `__weak` 関数を同じ名前で定義し直して，カードが待機(TRANSFER)に戻るまで待つようにした．

```c
static bool WaitCardReady(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t0 >= timeout_ms) return false;
    }
    return true;
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks) {
    if (!WaitCardReady(SD_READY_TIMEOUT_MS)) return MSD_ERROR;   // 待機に戻ってから始める
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK) return MSD_ERROR;
    return MSD_OK;
}
```

**512 バイトにそろえる**: 書くデータの場所(ファイルの中の位置とメモリの番地)がセクタ(512 バイト)の区切りにそろっていないと，`sd_diskio` は 1セクタずつ別のバッファにコピーして書く遅い道を通る．ログのファイルの先頭を 512 バイトちょうどにすると，追記が 1.2 秒 → 0.08 秒になった．

### 4.16 SD カード(2): 走りながら流す(`interface/sdcard.c`)

`f_write` は書き終わるまで戻らず，SD は時々数百 ms 待たせる(最大 0.6 秒ほど)．走っている最中にメインがそこで止まると，境界の処理が遅れる．そこで，**DMA を始めるだけで戻る**書き方を作った．

**考え方**: FatFs を通さずにカードのセクタへ直接書けば待たずに済む．ただし，ファイルがカードのどのセクタにあるかは FatFs しか知らない．そこで，

1. 先にファイルを 8MB まで伸ばして，場所を確保する．
2. FatFs の高速シークの「クラスタの対応表(cltbl)」を作らせる．
3. ファイルの中の位置 → カードのセクタ番号は，その表から自分で計算する．

```c
// 先に伸ばして領域を確保する(書き込みで開いたファイルは、終わりより先へ f_lseek すると伸びる)
FRESULT res = f_lseek(&s_file, reserve);
...
// クラスタの対応表を作る
s_clmt[0] = STREAM_CLMT_ITEMS;
s_file.cltbl = s_clmt;
res = f_lseek(&s_file, CREATE_LINKMAP);
```

```c
// ファイルの中の位置 off の、カードのセクタ番号と、そこから連続しているセクタ数
static bool OffsetToSector(uint32_t off, uint32_t *sector, uint32_t *contig) {
    FATFS *fs = s_file.obj.fs;
    uint32_t clsz = (uint32_t)fs->csize * 512u;
    uint32_t cl = off / clsz;              // 何番目のクラスタか
    uint32_t in_cl = (off % clsz) / 512u;  // クラスタの中の何番目のセクタか
    const DWORD *t = &s_clmt[1];           // (クラスタ数, 始まりのクラスタ) の組の並び
    while (t[0] != 0) {
        if (cl < t[0]) {
            uint32_t clst = t[1] + cl;
            *sector = fs->database + (clst - 2u) * fs->csize + in_cl;
            *contig = (t[0] - cl) * fs->csize - in_cl;
            return true;
        }
        cl -= t[0];
        t += 2;
    }
    return false;
}
```

FAT のデータ領域の最初のクラスタの番号は 2 なので，セクタ番号 = データ領域の始まり + (クラスタ番号 − 2) × クラスタの大きさ + クラスタの中の位置．領域が途中で途切れていても(断片化)，連続している所までで DMA を分ける．

**書き込みの状態**: メインは待ちのループの中で `SDCard_StreamPoll()` を何度も呼ぶ．呼ぶたびに今の状態を見て，次に進めるだけで，すぐ戻る．

```
IDLE --StreamWrite--> WAIT --カードが TRANSFER--> DMA --DMA 終わり--> WAIT --残り 0--> IDLE
                        ^                                              |
                        +------------- 残りあり(次の連続した所) ---------+
```

```c
case STREAM_DMA:
    if (HAL_SD_GetState(&hsd) != HAL_SD_STATE_READY) return 1;          // まだ送っている
    ...
    s_stream_state = STREAM_WAIT;     // カードが書き終わる(PROGRAMMING → TRANSFER)のを待つ
    return 1;
case STREAM_WAIT:
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) return 1;  // カードが書いている
    if (s_stream_left == 0) { s_stream_state = STREAM_IDLE; return 0; }
    StreamStartRun();                  // 次の連続した所の DMA を始める
```

最後に `f_truncate` で書いた所まで縮めて閉じる．こうすると，`.ioc` の FatFs の設定(`_USE_EXPAND`)を変えずに済む．

### 4.17 ロガー: ダブルバッファと YLOG2(`app/logger.c`)

ロガーは「列の名前と，値の入っている変数の場所」を登録しておき，割り込みが tick ごとに値を写す．

```c
Logger_AddField("target", &d->target_mm_s);   // 名前と、値の入っている変数の場所
Logger_AddField("vl", &d->vl);
...
```

**走りながら流すときは，バッファを2つのブロック(24KB ずつ)に分けて交互に使う**．割り込みが片方に行を足し，いっぱいになったらもう片方へ移る．メインはいっぱいになったブロックを SD へ送る．

```c
// 流す方式の1行(ISR)
static void SampleStream(void) {
    uint8_t a = s_blk_active;
    if (s_blk_full[a]) {
        uint8_t o = (uint8_t)(a ^ 1u);
        if (s_blk_full[o]) {   // 両方いっぱい(SD が追いついていない)
            s_stream_dropped++;  // その行は捨てて数える
            return;
        }
        s_blk_rows[o] = 0;
        s_blk_active = o;      // もう片方へ移る
        a = o;
    }
    float *row = BlockBase(a) + STREAM_BLOCK_HEAD + s_blk_rows[a] * s_field_count;
    for (uint32_t i = 0; i < s_field_count; i++) row[i] = *s_fields[i].value;
    if (++s_blk_rows[a] >= s_blk_max_rows) s_blk_full[a] = true;
}
```

- 34 列・5ms ごとなら，1ブロックは約 0.9 秒ぶん．SD が 1 ブロックを書くのに約 0.9 秒以上かかって初めて行が欠ける．実測は平均 16〜30ms，最大 234ms で，十分に余裕がある．
- 割り込みは送っている最中のブロックには書かない．`s_blk_full` が「割り込みが書いてよいか」の印になっている．
- 両方いっぱいのときは，待たずに捨てて数える(割り込みの中では待てない)．数は `dropped` として表示され，ログの `time_s` の飛びで PC からも分かる．

**ブロックの中身**: 頭に [magic "YBLK"，通し番号，行数，列数] を書く．PC は通し番号が続いている所まで読むので，電源が途中で切れても，そこまでのログは読める．

**ファイルの先頭は 512 バイト**: 列名を書いた後ろを空白で埋めて，ちょうど 512 バイトにする．続くブロックがセクタの区切りから始まり，そのまま DMA で書ける．

```c
// 改行を含めて LOG_BIN_HEADER_BYTES(512)になるよう、最後の改行の前を空白で埋める
while (n < total - 1u) s_header_buf[n++] = ' ';
s_header_buf[n++] = '\n';
```

### 4.18 イベントの記録(`app/logger.c`，`app/log_event.h`)

「境界で壁を読んだ」「スラロームを始めた」のような**出来事**を，ログの列として残す．

```c
void Logger_Event(uint16_t code, float a, float b, float c, float d, float e) {
    if (s_state != LOGGER_RECORDING) return;
    // メインからも ISR からも呼べるよう、入れる間だけ割り込みを止める
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t next = (s_ev_head + 1u) % EVENT_FIFO_SIZE;
    if (next == s_ev_tail) {
        s_ev_dropped++;                      // いっぱい: 捨てて数える
    } else {
        float *ev = s_ev_fifo[s_ev_head];
        ev[0] = (float)code;  ev[1] = a;  ...  ev[5] = e;
        s_ev_head = next;
    }
    __set_PRIMASK(primask);                  // 元の状態に戻す(割り込みの中から呼ばれても壊さない)
}
```

割り込みは記録する行ごとに，順番待ちから1つ取り出して `ev`〜`ev_e` の列に入れる(なければ 0)．1行に入るのは1つなので，同時に起きた出来事は次の行からずれて入る(5ms ごとの記録なら 5ms ずつ)．

- `__disable_irq()` ではなく PRIMASK を保存して戻すのは，割り込みの中(制御の ISR)から呼ばれたときに，割り込みを勝手に有効に戻さないため．
- 番号と中身の名前は `log_event.h` の1か所で決める．PC の `get_log.py` はこのファイルを読んで，`STEP x=1 y=0 ...` のような文字にする．機体と PC で表を二重に持たないので，ずれない．

### 4.19 PC でログを読む(`tools/get_log.py`)

```python
if magic == b"YLOG2":
    lines = data[:YLOG_HEADER_BYTES].split(b"\n")
    count = int(lines[1])
    names = lines[2].decode("ascii").strip().split(",")
    block_bytes = int(lines[3].strip())
    pos = YLOG_HEADER_BYTES
    seq = 0
    while pos + 16 <= len(data):
        blk_magic, blk_seq, n_rows, n_cols = struct.unpack_from("<4I", data, pos)
        if blk_magic != YBLK_MAGIC or blk_seq != seq or n_cols != count:
            break  # 書かれていない所 (または壊れたブロック)
        values = struct.unpack_from(f"<{n_rows * count}f", data, pos + 16)
        rows.extend(values[r * count:(r + 1) * count] for r in range(n_rows))
        pos += block_bytes
        seq += 1
```

ファイルは 8MB まで伸ばしてから書くので，後ろには書かれていない所が残ることがある(最後は縮めるが，途中で電源が切れた場合など)．magic と通し番号が続いている所までを読む．

**マイコンで CSV にしない理由**: 文字にする処理は遅く，ファイルも数倍の大きさになる．マイコンは float をそのまま書き，変換は PC がやる．

---

## 5. 開発の経過(時系列)

### 5.1 立ち上げ(2026-06-27 〜 10-01)

| 日付 | コミット | 内容 |
|---|---|---|
| 06-27 | 70e65f4，aa8f1ec | L チカ，UART の Hello World |
| 07-01 | 0e67f1b〜a73998d | ジャイロの Who-Am-I，値の取得，関数化 |
| 07-02 | cdc9920 | 書き込めるがモーターが回らない状態 |
| 09-21 | 0a34f6a | モーターとエンコーダが動いた |
| 09-26 | b04f14d〜0f3016a | タイマー割り込みで動かない問題を解決し，センサー値を取得．SD 以外の動作確認 |
| 09-29〜10-01 | 5c87b12，e435ae7 | センサーと機体全体の動作確認 |

### 5.2 層の整理(10-02，e6287fa)

- **目的**: CubeMX の平らな構成から，3.1 の層構成へ移す．
- **実装**: `global.h` から HAL への依存を外した．ISR に直書きしていた IR センサーの読み取りを `interface/sensor.c` へ切り出し，ISR は `Sensor_ReadAll()` を呼ぶだけにした．

### 5.3 速度制御の骨組みとモード選択(10-02，7811903)

- 汎用の PID コア(`logic/control/pid`)，運動学とオドメトリ，左右輪の速度 PID，1kHz の `App_ControlTick()` を作った．
- 右エンコーダとボタンでモードを選ぶ仕組みと，TEST モード(センサー・ジャイロ・自己位置の表示)を作った．

### 5.4 速度制御・フェイルセーフ・ロガー(10-03，3a2a287)

- **速度制御**:
  - 出力を電圧にし，FF(摩擦 + 速度 + 加速度，左右別)+ PID にした．
  - 床の上で取ったログからゲインと FF を合わせた．
  - 止まっているときは積分をリセットするようにした．
- **台形プロファイル**(`logic/control/velocity_profile`):
  - 減速の区間では，残りの距離でちょうど v_end になる減速度を毎 tick 計算し直す．
  - 減速度の上限は指定の 1.5 倍．
  - 固定の減速度だと，離散化の誤差で速度が残ったまま終点に着いてしまうため．
- **フェイルセーフ**: 低電圧，速度の偏差，角速度のどれかが続けて大きかったら止めて，そのまま止まった状態を保つ(ラッチ)．
- **ジャイロ**: ±2000dps にし，バースト読み出しを ISR へ移した．起動時にゼロ点を補正する．
- **ロガー**: divergence_v3 のログ機能を C へ移植した．`tools/get_log.py` が CSV で受け取る．
- **エンコーダの換算**: IEH2-4096 ×4 逓倍，13:42，φ23.35 に合わせた．

### 5.5 SD への保存・角速度制御・超信地旋回(10-05，7069d3e)

- **SD**:
  - SDIO 4bit + DMA と FatFs を有効にした．連番のファイルを作り，送ったファイルを管理する(`interface/sdcard.c`)．
  - SD_DUMP モードで，SD の中身を UART 経由で PC に送る．
- **カードが busy の間のエラー**: `FR_INVALID_OBJECT` が出るので，BSP_SD の `__weak` 関数を上書きして避けた．
- **制御**: 車輪速度ループの外側に，ジャイロの角速度 PI を足した．
- **試験**: PIVOT の試験モードを作った．

### 5.6 迷路の logic 層とシミュレータ(10-06，b806cec〜b5a9b7b)

- **目的**: 機体が使えない間に，迷路の判断を移植して PC で確かめる．
- **実装**:
  - BlueEyes から次の4つを移植した: 壁の地図(`wall_map`)，向き付き Dijkstra(`dijkstra`)，足立法の歩数マップ(`step_map`)，探索プランナー(`search_planner`)．
  - **`tools/maze_sim`**: 同じ C のソースを PC でコンパイルする．CLI と pygame の GUI(DLL を ctypes で呼ぶ)がある．
  - **大会の迷路を取り込む**: 公式の迷路図(画像)を classic 形式に変換する(`maze_from_image.py`，`maze_batch.py`)．2020〜2025 年の 92 枚を `my_mazes/` に入れ，印刷された最短歩数と照らし合わせて確かめた(`maze_verify.py`)．
- **結果**: 帰りに既知の区画へ入るときのコストの上乗せ `MAZE_COST_KNOWN_CELL_RETURN`(仮に 3)は，乱数の迷路では逆効果だった．全日本の 61 迷路では，最短を見つけた数を 10 → 21 に戻す効果があった．乱数の迷路と大会の迷路で結論が逆になるので，調整は大会の迷路で評価する．
- **保留**: 帰りの探索の A 案．探索用の見方での最短経路の候補の上で，未知の通路の両側の区画を目的地にする．

### 5.7 LED での知らせ方(10-06，f8ceb68，41bfdc4)

- 直結 LED に位置の名前を付けた(右前，左，左前，右，右後ろ，左後ろ)．
- フェイルセーフは，原因の番号 n を直結 LED の点滅で示す．電圧低下は右後ろで知らせる．
- SD への保存は，成功なら全部を 0.5 秒点け，失敗なら左後ろを 0.1 秒周期で点滅させる．

### 5.8 超信地旋回の追従の改善(10-06，79e996b)

- **経過**: PIVOT のログで，ジャイロの角速度が (vr − vl)/57.90 の 0.912 倍しかなかった．
- **対策 1**: `TREAD_WIDTH_MM` を 57.90 → 63.50(実効値)にした．その後のログ2本で，比は 0.995 / 1.000 になった．
- **対策 2**: 超信地旋回のときだけ足す摩擦 FF(`PIVOT_FF_FRIC`)を作った．タイヤが横にこすれるので，直進用の FF では 0.75〜0.84V 足りなかった．
- **結果**:
  - 角速度の補正の最大は 133 → 約 60 dps に下がった．
  - 車輪の速度と目標の比は 0.79〜0.88 → 0.93〜1.05 になった．
  - 右回りの振動が半分になった．

### 5.9 角度の制御を常時有効に・ジャイロのゼロ点の測り直し・手かざし(10-07，4e93154)

- 目標の向き θ_ref を持ち，止まっている間も角度の P + 角速度の P で向きを保つようにした．
- **ジャイロのゼロ点**: 起動時の補正だけでは数 dps ずれ(ログで −3〜−5.6dps)，向きの制御が機体を曲げていた．そこで，走り出す直前に 0.5 秒かけて測り直す(ISR の中で平均)ようにした．
- 試験の走り出しを手かざしにした．

### 5.10 位置の追従・モードの階層化・宴会芸モード(10-07，7eb8d14)

- **位置の制御**: 目標の距離と，進んだ距離(左右の平均)の差に P をかけて並進の目標に足す．
  - 入れる前は，止まる直前に静止摩擦に負けて −4.5mm のずれが残っていた．
- 旋回用の摩擦 FF を，回る向きでも分けて4つにした．
- モードを TEST / SD の階層にした．
- **PARTY(宴会芸)モード**: 床ごと回されても，ジャイロで同じ方向を向き続ける．

### 5.11 タイヤの直径とトレッドを走行に合わせる(10-07，9ec31a8)

- **経過**: 直進 1080mm を 300mm/s と 800mm/s で走らせると，どちらも約2%(21〜22mm)行き過ぎた．
- **判断**: 行き過ぎが速さによらないので，滑りではなく換算のずれとした．
- **対策**:
  - `WHEEL_DIAMETER_MM` を 23.35 → 23.81 にした．
  - トレッドも同じ割合で 63.50 → 64.75 にし，ジャイロとの関係を保った．
  - 加速度 FF を 10% 上げた．加速中の I 項が加速度に比例して正だったため．
- **結果**: 次のログで，止まった位置は目で見てほぼぴったりになった．

### 5.12 最短経路を走行時間で選ぶ(10-07，df86e29)

- **logic 層に `run_path` を足した**:
  - 最短走行の指令: 直進は半区画単位，ほかに小回り 90，大回り 90/180．
  - 台形加速での走行時間の見積もり．
  - 区画の経路を大回りに置き換える処理．
- **`time_dijkstra` を足した**: 走行時間が最短の経路を探す．ノードは境目 × 向き × 速度の種類で，ゴール側から逆向きに探す．
- **大回りの形**: 区画の中心から中心へ曲がる(90° は 0R0，180° は 0RR0)．
- **シミュレータ**: GUI で最短走行の経路を表示し，再生できるようにした．

### 5.13 車輪速度の P と右の摩擦 FF(10-07，f0f2303，0b99539)

- **`VELOCITY_KP` 0.0094 → 0.015**: ステップを入れると，立ち上がりの間に I 項が溜まって 20% 行き過ぎていた．P を強めて，立ち上がりを P に受け持たせた．行き過ぎは 12% に減った．
- **`VELOCITY_FF_FRIC_R` 0.53 → 0.16**: 右の車軸のねじを締め直したら駆動系が軽くなり，巡航中の I 項が −0.37V になったため．
- **結果**: 直進 300mm/s で，位置のずれ ±0.3mm，向きのずれ 0.4° 以内になった．
- **`ANGLE_KP` 15 → 25**: 超信地旋回で，FF の足りない分を受けるのに 2〜3° のずれが要っていた．25 にすると 1〜2° に減り，揺れは出なかった．

### 5.14 壁センサー・壁の判定・壁の制御(10-07，1d948fc)

- **SENSOR_LOG モード**: ボタンを押すと，3秒ぶんの壁センサーの値を記録する．
- **壁の判定**(`logic/wall_sense`):
  - 区画の境界に止めて測った値の，「壁なしの最大」と「壁ありの最小」の真ん中をしきい値にした: L 180，R 150，FL+FR 220．
  - 前の壁を FL+FR の合計で見るのは，FR が壁ありでも小さく，機体が少し斜めでも合計はあまり変わらないため．
- **壁の制御**: 直進中，左右の値の基準からのずれで目標の向きを動かす．
- **壁の切れ目・柱**: 2mm 進む間に値が 15 を超えて変わった側は，40mm の間使わない．
  - 比べる間隔は初め 5mm だったが，切れ目を見つけるまでに補正が漏れるので 2mm にした．
  - ログでは，切れ目では 5mm で 30〜170 変わり，壁に寄っていくだけなら 5mm で 2 程度だった．
- **ユーザーの指摘**: 壁がなくなったときに，ない壁に吸い寄せられるのでは？ → 切れ目の検出で，その側を使わないようにした．
- **方針**: センサーの値を mm に直すのは時間がかかるのでしない(ユーザーの判断)．値の単位のまま扱い，ゲインはログで合わせる．

### 5.15 探索走行・最短走行(10-07，bbfbcf7)

- **探索**: BlueEyes の searchB と同じ流れにした．
  - start_sequence: 右 90° → 尻当て → 左 90° → 尻当て で，区画の真ん中に合わせる．
  - 最初の半区画を走り，境界で壁を読んで経路を計算し直す．
  - 直進は止まらない．曲がるときは真ん中で超信地旋回する(このときはまだ)．
  - 180° のときは，後ろと横に壁があれば尻当てを2回する．
  - ゴールまで行けたら，地図を flash の最後のセクタに保存する．このため，リンカースクリプトの FLASH を 896K にした．
- **最短走行**: flash の地図の既知の壁だけで Dijkstra を計算し，続く直進をまとめて 800mm/s で走る．
- **壁の制御を作り直した**:
  - 初めは「誤差に比例して向きを回す速さ」の形だった．大きく寄ったときに向きが 8.7° 回ったまま残り，曲がった後に斜めのまま壁にぶつかった．
  - 「迷路の軸の向き + 横のずれに比例したオフセット」の形に変えた．横のずれが 0 に戻ればオフセットも 0 に戻るので，向きのずれが残らない．
- **尻当て**: 時間で下がる方式にした(800ms では短く，1200ms にした)．押し当てている間は向きと位置の補正を止める．
- **ログ**: 時間で区切って止まり，SD にバイナリで追記する方式にした．先頭を 512 バイトにそろえて DMA でまとめて書けるようにすると，追記が 1.2 秒 → 0.08 秒になった．
- **細かい調整**:
  - 最初の半区画は 44mm で試した後，尻当て後の前進を 40mm にした(ログで真ん中から壁まで 39〜40mm)．
  - スタートでジャイロの積算角をリセットする．
  - 加速度 3000 では，真ん中で止まる減速が強すぎて安定しなかったので 2000 にした．
- **迷路の座標**: BlueEyes と合わせた．x が東，y が北．試しの迷路は 5×7 で，ゴールはスタートの右隣(1,0)．

### 5.16 スラローム(未コミット)

- **目的**: 探索で止まらずに曲がる(小回り 90°)．最短走行の大回りも用意する．
- **実装**:
  - `logic/control/slalom`: 並進の速さ v を保ったまま，角速度を台形で動かして曲がる．
    - 前後のオフセットは，機体の ISR と同じ `VelocityProfile` を 1ms 刻みで進めて計算する．
    - 小回り 90: 境界の真ん中 → 隣の境界の真ん中(span = 半区画)．
    - 大回り 90: 区画の中心 → 斜め隣の中心(span = 1区画)．
    - 大回り 180: 横にちょうど1区画移る ω を二分法で求める．pre = 区画 − 一番奥まで進んだ距離，post = pre + 出口の前後の位置．
  - SLALOM の試験モード: s90 / l90 / l180 × 右 / 左 をクリックで選ぶ．
  - 探索で PIVOT / SMALL，最短走行で PIVOT / SMALL / LARGE を，走る前にクリックで選ぶ(maze_sim と同じ3種)．
- **経過**:
  - 最初の試験で，5本のうち2本が柱に当たった．手で置いたときのずれが原因と考え，試験の前に尻当てをするようにした．
  - 探索のスラロームの曲がり始めが遅れている．境界で経路を計算する `Step` に 20〜25ms かかる．小回りの前のオフセット(約 12mm = 500mm/s で 24ms)に近い．A/B/C の対策案は未決定．

### 5.17 走りながら SD へ流すログとイベント(未コミット)

- **目的**: 最短走行のログが RAM(48KB)に収まらない．走行全体を止まらずに記録したい．
- **実装**:
  - **`interface/sdcard`**: 待たない書き込み(`SDCard_Stream*`)．
    1. ファイルを先に 8MB 伸ばす．
    2. 高速シークの cltbl からセクタ番号を計算する．
    3. `HAL_SD_WriteBlocks_DMA` を始めるだけで戻る．
  - **`logger`**: 24KB のブロック2つを交互に使う(YLOG2 形式)．`Logger_StreamBegin` / `Poll` / `End`．
  - **イベント**: `Logger_Event(code, a〜e)` を 32 個の FIFO に入れ，ISR が次に記録する行の `ev`〜`ev_e` 列に1つずつ入れる．番号は `app/log_event.h` で決める．
  - **`run_log`**: 探索，最短走行，PARTY で同じ全部の列(27列 + イベント 6列，5ms ごと)を記録する．
  - 試験モードは今までどおり RAM に記録し，走った後に SD へ保存する．SD のファイルはすべて `.bin` にし，CSV は PC が作る．
  - **STREAM_TEST モード**: モーターを動かさずに，流す方式を確かめる．
  - 記録は手かざしで始めた時から(常に取るのではなく)．
- **結果**: PARTY で 46 秒，116 秒，134 秒走らせ，行の欠けはなかった．書き込みは平均 16〜30ms，最大 234ms で，1ブロックは約 0.9 秒ぶんなので余裕がある．
- **PARTY の拡張**: 向きに加えて位置も保つようにした．押されてから戻るまで 0.12〜0.25 秒だった．

### 5.18 前壁補正・壁切れ補正(未コミット)

- **前壁補正**(探索の小回り):
  - 曲がる区画の奥に壁があるとき，距離で決めた曲がり始めの位置の前後 15mm の間で，FL+FR が `SLALOM_FRONT_REF_SUM`(仮に 400)に届いた瞬間に曲がり始める．
  - 結果はイベント `FRONT_TRIG` に残す．
- **壁切れ補正**(進む方向の位置):
  - 直進中に横の壁が切れた瞬間は，境界を `WALL_EDGE_POS_MM`(仮に 78mm)過ぎた所にいるはずとみなす．ずれていた分だけ次の境界をずらす．
  - 予想から ±30mm を超えてずれた壁切れは使わない．1回に直す量は ±20mm まで．
  - 結果はイベント `EDGE_CORR` に残す．
- **残り**: 400 と 78 は仮の値で，迷路で走ったログで合わせる．

### 5.19 壁の制御のゲインを速さに合わせる(未コミット)

- ゲインを `WALL_KP_DEG × 500 / v` にした(100mm/s より遅いときは 100 として計算)．
- 横に戻る速さは v × オフセット なので，真ん中に戻るまでの時間が速さによらず同じになる．

### 5.20 PC ツール: turn_sim，log_viewer(未コミット，2026-10-08)

- **turn_sim**(`tools/turn_sim.py`): `turn_sim_classic2.py` を作り直したもの．
  - 機体と同じ計算(params.h を読み，同じ台形・同じオフセット)．スリップ K=0 なら出口のずれは 0.00mm で一致した．
  - **スリップアングル**: 1次遅れ dβ/dt = (K·v[m/s]·ω[rad/s] − β)/C．C = 0 なら遅れなし．
  - 結果を表示する: 出口のずれ，柱との距離，ずれを消す `*_ADJ`(params.h に貼れる形)．
  - **実機のログを重ねる**: 車輪とジャイロで作った軌道と，壁センサーで見た横の位置の差から K と C を合わせる．
  - **今のログで合わせた結果**(turn_0001, 0003〜0005):
    - K ≈ 0.008．
    - C は探す範囲の端(0.2s)で，決まっていない．
    - 右・左共通の右へのずれ b = +8.3mm．
    - 横センサーの傾きは，探索のログから約 7.8 AD/mm と見積もった(だいたいの値)．
- **log_viewer**(`tools/log_viewer.py`):
  - 列を選んで時系列のグラフにする．イベントの縦線と名前を描く．
  - イベントの一覧を出し，クリックでその時刻へ飛ぶ．
  - 迷路の上に軌道を描き，カーソルの位置を示す．
  - 再生と停止(速さ ×0.05〜×4，スライダー)．
  - 起動するとログを選ぶ画面が出る．
- **迷路の壁の表示**:
  - 読んだ壁(STEP イベント)をカーソルの時刻までの分だけ描く．
  - 機体の地図も描く．地図は，機体が走りの終わりに `LOG_EV_MAP_CELLS` で 8区画ずつログに入れる．CSV の有効数字 6桁のため，1つの値には 16bit まで詰める．
- **再生の描画**: グラフ全体を描き直すと1コマ約 0.3 秒かかったので，カーソルだけを重ねて描く方式(blit)にした．約 74ms になった．

### 5.21 速さを選ぶ・番号を 1 から(未コミット，2026-10-08)

- モードの番号を 1 から(n 番 = LED n, n+1)にし，0 番の全点灯は無くした．
- **モードを決めた直後に速さを選ぶ**: 操作はモード選択と同じ．最初は params.h の値に一番近いもの．

  | モード | 選べる値 |
  |---|---|
  | 探索，STRAIGHT，SLALOM(s90) | 300/400/500/600 mm/s |
  | 最短走行(直進の最高速度) | 600/800/1000/1200 mm/s |
  | PIVOT | 180/360/540 dps |

- **小回りの速さを変えたとき**: ω ∝ v，α ∝ v² にして，曲がる形を保つ(`Slalom_ScaleToSpeed`)．オフセットと前壁補正の位置はほぼ同じで，横加速度は v² で増える(300: 0.14G，500: 0.40G，600: 0.58G)．

---

## 6. 調整のやり方(手順集)

### 6.1 車輪速度の FF(`VELOCITY_FF_*`)

1. **摩擦と速度の項**: 定常で走らせて，2つの速さで電圧を測る．直線 `電圧 = FRIC + GAIN × 速度` で近似する．左右で分ける．
2. **微調整**: 巡航中の I 項を見る．I 項が正なら FF が足りない(FRIC を上げる)．0 付近になれば合っている．
3. **加速度の項**:
   - 台形の加速中・減速中の電圧から，摩擦と速度の分を引き，加速度で割る．加速と減速で平均すると，摩擦の影響が打ち消される．
   - その後，加速度を変えたログで，加速中の I 項が加速度に比例しているかを見る．比例して正なら足りない．
4. **駆動系が変わったら合わせ直す**: 車軸のねじを締め直しただけで，右の FRIC は 0.53 → 0.16 に変わった．分解したり，ぶつけたりした後は I 項を見る．

### 6.2 超信地旋回の摩擦 FF(`PIVOT_FF_FRIC_*`)

1. PIVOT の試験でログを取る．回る向きとは逆の I 項が出ていれば多すぎ，同じ向きなら足りない．
2. 2つの値で取ったログの I 項から，直線で補間して I 項が 0 になる値にする．
3. 左回り / 右回りで分ける．右回りはいつも左回りの後に回しているので，順番の影響が混ざっている可能性がある．

### 6.3 寸法(タイヤの直径・トレッド)

- **直径**: 決めた距離を走らせ，行き過ぎた量を測る．速さを変えても同じ割合なら換算のずれとみなして，直径を比で直す．
- **トレッド**: PIVOT のログで，ジャイロの角速度 / ((vr − vl)/TREAD) の比を見て合わせる．直径を変えたら，トレッドも同じ割合で変える．

### 6.4 ゲイン

- **車輪速度の P**: VEL_PID のステップで，立ち上がりに I 項が溜まって行き過ぎるなら P を上げる．
- **角度の P**: 旋回中の向きのずれを見る．止まった後に揺れが出ない範囲で上げる．
- **振動が出たら**: 閾値・不感帯ではなくゲインで直す(3.2)．

### 6.5 壁センサー

- **しきい値**: SENSOR_LOG モードで，境界に止めて「壁あり / 壁なし」を測る．真ん中にする．
- **WALL_REF**: 真ん中にいるときの L / R．走行のログのスタート直後の値にした．
- **壁の制御のゲイン**: STRAIGHT のログ(壁センサーの列あり)で，横のずれの戻り方を見る．
- **壁切れの位置 `WALL_EDGE_POS_MM`**: 迷路を走ったログの `EDGE_CORR` イベント(expected，edge_pos)を見て合わせる．
- **前壁の `SLALOM_FRONT_REF_SUM`**: `FRONT_TRIG` イベント(diff，sum)を見て合わせる．

### 6.6 スラロームのオフセットとスリップ

1. SLALOM の試験で，右・左の両方のログを取る．曲がった後の通路は，左右に壁がある所にする．
2. turn_sim でログを開き，「傾きを探索のログから見積もる」で横センサーの傾きを入れる．
3. 「K と C をログに合わせる」を押す(共通の横のずれ b も合わせる)．
4. 出てくる `*_PRE_ADJ_MM` / `*_POST_ADJ_MM` を params.h に貼る．
5. もう一度走らせて確かめる．

### 6.7 迷路の判断のコスト

- `tools/maze_sim` で，全日本の迷路(`my_mazes/`)を使って評価する．乱数の迷路と結論が逆になることがある(5.6)．

---

## 7. ログの仕組みと使い方

### 7.1 形式

| 形式 | 使うところ | 中身 |
|---|---|---|
| YLOG1 | 試験モード(RAM に記録してから保存) | `"YLOG1\n<列数>\n<列名>..."` + float32 × 列数 × 行数 |
| YLOG2 | 探索，最短走行，PARTY，STREAM_TEST(走りながら流す) | 512 バイトの先頭 + 24KB のブロック(`YBLK`，通し番号，行数，列数 + float32) |

- SD 上のファイル名は `<モードのフォルダ>/<名前>_NNNN.bin`．番号は `.csv` / `.bin` をまたいで重ならない．
- 間引き: 走りながら流すものは 5ms ごと．

### 7.2 イベント(`Core/Inc/app/log_event.h`)

- 1行に1つ．`ev` 列が番号で，`ev_a`〜`ev_e` が中身．
- 「`LOG_EV_名前 = 番号, // a:名前 b:名前 ...`」の行を `get_log.py` が読み，CSV に `ev_text` 列(例: `STEP x=1 y=0 ...`)を足す．
- 番号は増やすだけで，使っている番号は変えない(昔のログを読めるように)．

| 番号 | 内容 |
|---|---|
| 1〜6 | 操作と状態: MODE，RUN_TYPE，HAND_START，CLICK，GYRO_RECAL，CTRL_ENABLE |
| 10〜14 | 制御の切り替え: WALL_CTRL，WALL_USE，WALL_EDGE，EDGE_CORR，FRONT_TRIG |
| 20〜24 | 動き: STRAIGHT，PIVOT，SLALOM，MOTION_DONE，SET_VELOCITY |
| 30〜34 | 走りの単位: TURN_KIND，SETPOS，STOP_CENTER，RUN_CMD，ROUTE_CMD |
| 40〜45 | 迷路: STEP，STEP_INFO，PHASE，MAP_SAVED，ROUTE，MAP_CELLS |
| 50〜51 | 異常: FAILSAFE，TIMEOUT |

### 7.3 PC での使い方

```
python tools/get_log.py COM5                     # SD_DUMP / ロガーの送信を受け取る(.bin の隣に .csv も作る)
python tools/get_log.py --bin2csv logs/search/search_0006.bin
python tools/sd_import.py                        # PC に挿した SD カードから logs/ に取り込み、.csv も作る
python tools/log_viewer.py                       # 起動してログを選ぶ
python tools/turn_sim.py                         # スラロームのシミュレーションとログの照合
```

MATLAB 版(`tools/matlab/`．受信は上の `get_log.py` のまま．使い方は `tools/matlab/README.md`):

```matlab
addpath('tools/matlab')    % yuho/ で
turn_sim                   % スラロームのシミュレータ(K と C の合わせは格子の後 fminsearch で詰める)
log_viewer                 % ログを見る GUI
plot_log                   % ログを時系列のグラフにする(複数のファイルを重ねられる)
```

---

## 8. PC ツール

| ツール | 役割 |
|---|---|
| `tools/get_log.py` | UART で受け取る(SD_DUMP，ロガー)．`.bin` → `.csv`(`ev_text` 付き)．同じ名前の古い CSV は上書きせず `_dupN` を付ける |
| `tools/sd_import.py` | PC に挿した SD カードのログ(`<dir>/` と `sent/<dir>/`)を `logs/<dir>/` に取り込み，`.bin` の隣に `.csv` を作る．取り込み済みは飛ばす．SD は読むだけ |
| `tools/log_viewer.py` | ログを見る GUI(時系列，イベントの一覧，迷路の上の軌道，壁，再生) |
| `tools/turn_sim.py` | スラロームのシミュレータ(機体と同じ計算，スリップ，ログとの照合，ADJ の提案) |
| `tools/yuho_common.py` | 上の2つで共通: params.h の読み込み，台形とスラロームの Python 版，ログの読み込み，軌道の作成 |
| `tools/maze_sim/` | 迷路の探索と最短経路のシミュレータ(C の logic 層を共有)，迷路図の取り込み |
| `tools/turn_sim_classic2.py` | 元の pygame のスラロームのシミュレータ(残してある) |
| `tools/matlab/` | MATLAB 版: `turn_sim.m`，`log_viewer.m`，`plot_log.m`(`--plot-file` の代わり)，共通の部品 `+yc/`(`yuho_common.py` に当たる) |

---

## 9. つまずいたことと教訓

| 問題 | 原因 | 対策・教訓 |
|---|---|---|
| タイマー割り込みで動かない(09-26) | (コミットの記録のみ．詳細は未記録) | — |
| SD の `FR_INVALID_OBJECT` | カードが書き込み中(PROGRAMMING)のまま次の操作をした | BSP_SD の `__weak` 関数を上書きして待つ |
| `SD: mount failed (FRESULT=1)`(FR_DISK_ERR) | 原因は未確認 | 書き込み直したら直った |
| SD への追記が 1.2 秒 | 書き込む位置がセクタの区切りにそろっておらず，`sd_diskio` が1セクタずつの遅い道を通った | 先頭を 512 バイトにそろえる → 0.08 秒 |
| `.bin` と `.csv` の番号が重なり，古い CSV が上書きされた | 番号を拡張子ごとに数えていた | 両方を見て番号を決める．PC 側も `_dupN` |
| ロガーの列の上限で vbat が落ちた | 上限 18(時刻を含む) | 列を減らした後，上限を 40 にした |
| 壁の制御で向きが 8.7° 回ったまま残った | 「向きを回す速さ」の形(積分になる) | 「軸 + オフセット」の形にした(5.15) |
| 尻当てが短い | 下がる時間が 800ms | 1200ms にした |
| 減速で不安定 | 加速度 3000 | 2000 にした |
| スラロームの試験で柱に当たる | 手で置いたときのずれ | 試験の前に尻当て |
| スラロームの曲がり始めが遅い | 境界の経路計算に 20〜25ms | 未解決(前壁補正で一部は吸収) |
| E31 のデバッガのエラー | (ユーザーが貼ったもの．書き込み環境の問題) | — |
| PC でファイルを書き換えると C の `\r\n` が本物の改行になる | Git Bash の heredoc + Python のエスケープ | エスケープを含む編集は Edit ツールで行い，後で文字列の壊れを確かめる |
| 電池が 7.06V まで下がっていた | 充電忘れ | フェイルセーフは 7.0V．走る前に充電する |

---

## 10. 今のパラメータ(2026-10-08)

詳しい理由は `Core/Inc/params.h` のコメントにある．

| 分類 | 値 |
|---|---|
| 寸法 | WHEEL_DIAMETER 23.81mm，TREAD 64.75mm |
| 車輪速度 | KP 0.015，KI 0.0376 [V/(mm/s)] |
| 速度 FF | FRIC L 0.32 / R 0.16 V，GAIN L 0.00075 / R 0.0005，ACC 0.000275 |
| 旋回 FF | CCW L 0.57 / R 0.69，CW L 0.64 / R 0.76 V |
| 角度 | ANGULAR_KP 0.5，ANGLE_KP 25，補正の上限 6 rad/s |
| 位置 | POSITION_KP 10，補正の上限 200mm/s |
| スタート | FL > 350 が 50ms |
| 壁の判定 | L 180，R 150，FL+FR 220 |
| 壁の制御 | REF L 312 / R 282，KP 0.03 deg(500mm/s のとき，∝1/v)，最大 5°，90dps |
| 壁切れ補正 | 境界 + 左 78mm / 右 83mm，±30mm，最大 20mm |
| 探索 | 500mm/s(選べる)，2000mm/s²，超信地 180dps / 1800dps² |
| 尻当て | 100mm/s で 1200ms 下がる，前へ 40mm |
| 最短走行 | 800mm/s(選べる)，2000mm/s² |
| 小回り | 500mm/s・450dps・8000dps²(速さに合わせて ω ∝ v，α ∝ v²)，オフセット約 12mm，調整はモデルで計算(K 0.0219，C 0.02，スリップ以外の遅れ 7.5mm(速さによらず一定))．500mm/s で前 −6mm・後ろ +12.4mm |
| 大回り 90 | 600mm/s・230dps・3000dps² |
| 大回り 180 | 500mm/s・3000dps²(ω は計算で約 323dps) |
| 前壁補正 | FL+FR 400 + 4.5 × 前の調整分(500mm/s で 373)，±15mm |
| フェイルセーフ | 7.0V が 500ms，速度の偏差 500mm/s が 100ms，角速度 1500dps が 10ms |

---

## 11. 残っている課題

- **ビルドと実機での確認**: 2026-10-08 に 5×7 の試しの迷路で探索と最短走行が完走した(12章)．
- **迷路で確かめる**:
  - 前壁補正の `SLALOM_FRONT_REF_SUM`(400)と，壁切れの `WALL_EDGE_POS_MM`(78)を，`FRONT_TRIG` / `EDGE_CORR` のイベントで合わせる．
    前壁補正は，下の曲がり始めの遅れを直してからでないと合わせられない(範囲の前半が使えていない)．
  - 機体の地図をログに入れる `LOG_EV_MAP_CELLS` を，PC の log_viewer で描いて確かめる(ログには入っていた)．
  - 大回り(l90 / l180)の試験のログを取る．試しの迷路の最短走行(LARGE)では大回りが1つも選ばれなかった．
- **スラロームの曲がり始めの遅れ**: 境界の経路計算(実測 27〜28ms，500mm/s で約 14mm)．
  打ち切り＋バケツのキューと，コンパイルの最適化(-O2)を入れた(2026-10-08，未確認)．実機の `plan_ms` と `FRONT_TRIG` で確かめる．
- **スラロームの後に機体が遅れる**: 小回り1回で，出口の向きに 6〜12mm 遅れ，外へ 2〜6mm ずれる(2026-10-08 の解析)．
  外へのずれはスリップ(K≈0.015)で説明できるが，前後の遅れはスリップの分(約 3mm)より大きく，残りの原因は未確認．
  1回だけ曲がった後の壁切れで，前後のずれを直接測る．
- **最短走行の途中で止まることがある**(fast_0007，fast_0008．壁に当たった後・尻当ての途中．原因は未確認)．
  HardFault とリセットの原因を残す仕込み(`interface/fault_diag.c`)を入れた．次に起きたら起動時のシリアルとログの `BOOT` を見る．
- **フェイルセーフのイベントがログに入らないことがある**: 順番待ちのイベントがあると，`HAL_Delay(SEARCH_LOG_DECIMATION * 2)` の間に入りきらない．
- **スリップ**:
  - K と C は，今のログでは C が決まらなかった．
  - 右・左の両方，両側に壁のある出口で取り直す．
  - 速さ(300〜600mm/s)を変えたログもあると，K と C を分けやすい．
- **最短走行の小回りでの前壁補正**: 未実装．
- **帰りの探索の A 案**: 保留．
- **センサーの値の mm への換算**: しない方針．横センサーの傾きは，探索のログからの見積もり(約 7.8 AD/mm)を使っている．
- **LED8 以降**: U6 のはんだ不良で，確実に光るのは LED1〜7．モードの番号 7 以降は LED8 を使う．
- **turn_sim_classic2.py**: 残すか消すかは未定．

---

## 12. 開発日記(2026-10-08 以降)

ここから先は，作業をするたびに時系列どおりに書き足していく．
1〜11 章(まとめ)は，内容が変わったときだけ直す(パラメータの表・残課題など)．

書き方: 日付の見出しの下に，作業ごとに「やったこと・なぜ・結果(確かめたこと / 未確認)・次にやること」．

### 2026-10-08

- **開発記録を作った**(この文書)．立ち上げから今日までの経過・設計・調整のやり方をまとめた．
  以後は作業ごとにこの日記へ書き足す．

- **push した**: feature/maze のコミット済みの 9 個(79e996b〜bbfbcf7)を origin へ push した．
  bbfbcf7 より後の変更(5.16〜5.21 と開発記録)はまだコミットしていないので，push には入っていない．

- **未コミットの変更を feature/maze にコミットした**(内容ごとに6つ: ログ，制御，スラローム，走行と UI，PC ツール，開発記録)．
  どのコミットもビルドと実機での確認をしていないので，コミットの本文に「未確認」と書いた．
  ファイル単位で分けたので，途中のコミットだけを取り出すとビルドできない可能性がある(全部そろった状態で使う)．
  `.settings/stm32cubeide.project.prefs` と `Debug/yuho.bin` は，今のソースと合っているか分からないので入れなかった．

- **push に失敗した**: 6つのコミット(752f795〜1e54424)を push しようとしたが，GitHub が
  `remote rejected (Internal Server Error)` で拒否した．最初のコミット1つだけでも同じだった．
  送るファイルは最大 48KB と小さく，GitHub の状態表示は正常だった．原因は未確認(GitHub 側の一時的な不具合か，
  リポジトリの設定かは分からない)．コミットは手元の feature/maze に残っている．時間をおいて push し直す．
  新しいブランチ feature/slalom(feature/maze と同じ位置)を作って push しても，同じ Internal Server Error だった
  (Request ID C16F:1ED705:1E1A95:2A33EC:6AC679D1)．ブランチ名の問題ではなく，この時点の GitHub への push 全体が失敗している．
  ユーザーが GitHub の画面でプルリクを許可しようとしたら error 500 だった．GitHub の API で見ると，
  プルリクは1つもなく，ブランチの保護もルールセットもなかった(リモートの feature/maze は bbfbcf7 のまま)．
  画面の操作でも 500 が出るので，GitHub 側でこのリポジトリへの書き込みが失敗している状態と考えている(未確認)．
  時間をおいて push し直したら成功した(bbfbcf7..1e54424 → origin/feature/maze)．GitHub 側の一時的な不具合だったと考えている．
  試しに作った feature/slalom は手元にだけある(リモートには作られていない)．
- **開発日記を push した**: 日記の追記をコミットして(f193907)，origin/feature/maze へ push した．
  日記は書き足したらコミットして push する．

- **開発記録に「4. 実装の解説(コードで読む)」の章を足した**: ユーザーの依頼(PID の式や SD カードの使い方のコードを入れ，
  読んだ人が分かるように)．1ms の割り込みの流れ，センサーの読み方，エンコーダ，運動学，PID のコア(アンチワインドアップ)，
  車輪速度のループと FF(電圧で考える理由，2自由度制御)，外側のループ(角度の誤差の時定数 (1+K_ω)/K_θ = 0.06 秒の導出)，
  台形プロファイル，メインと割り込みの受け渡し，壁の制御，壁切れ補正，スラロームの形，探索の流れ，フェイルセーフ，
  SD カード(FatFs の使い方，BSP の上書き，走りながら流す仕組み)，ロガー(ダブルバッファ，YLOG2)，イベントの FIFO，
  PC でのログの読み方を，実際のコードを引用して説明した．元の 4〜11 章は 5〜12 章に番号を付け直した．

- **開発記録の 3 章に「3.6 設計の判断と理由(思想)」を足した**: ユーザーの依頼(どういう理由でこういう設計にしたか)．
  16 項目(計算とハードを分ける，値と式を分ける，物理の単位，FF が主役，隠さずに直す，目標は絶対値，機体と同じ計算を PC でも，
  判断と動きを指令で分ける，実績のある流れ，測ってから決める，小さく同じ条件で，安全な側へ，割り込みとの約束，生成コードを触らない，
  機体だけで使える，やらないと決めたこと)を，考え・理由(もとになった出来事)・代わりにあきらめたこと の形で書いた．

- **走行のログを確認した**(SD カードからコピーした `logs/search/` の7本．`get_log.py --bin2csv` で CSV にした)．
  - **search_0006**: 8MB(確保した大きさのまま)で，先頭の後に書かれたブロックが1つもない．閉じられていない(縮められていない)ので，
    最初のブロック(約 0.9 秒分)を書く前にリセットか電源断があったと考えている(原因は未確認)．
  - **search_0007**: 28.7 秒で止まった．**電池の電圧低下でフェイルセーフが働いた**．走り始めの電圧が 7.5V と低く，
    尻当てで壁に押し付けている間(モーターがほぼ止まって電流が大きい)に 6.7〜6.9V まで下がり，7.0V 未満が
    ちょうど 500ms(`FAILSAFE_LOW_VOLTAGE_MS`)続いた所で PWM が 0 になっていた．
    ただし `LOG_EV_FAILSAFE` のイベントはログに入っていなかった．直前に SET_VELOCITY と CTRL_ENABLE が順番待ちしていて，
    `CheckFailSafe` が閉じる前に待つ 10ms(2行)ではフェイルセーフのイベントまで入りきらなかった(次の行に入るはずだった)．
  - **search_0008**(電池 8.2V): **探索が最後まで動いた**．ゴール(試しのゴール (1,0))に 31.4 秒，スタートに戻って 57.2 秒，地図の保存も成功．
    行の抜けなし．角度の誤差は最大 3.5°，壁の制御のオフセットは ±2.8° 以内．
  - **最短走行**: fast_0003(超信地旋回) は指令の間 約 26.9 秒．fast_0004〜0006(小回り，0005 は LARGE の指定)は3本とも
    同じ経路・同じ時間(指令の間 約 8.3 秒，見積もりの est_s は 7.05 秒)．LARGE でも大回りは選ばれず，全部 s90 だった
    (試しの迷路では大回りできる所がなかったと考えている．未確認)．スラローム後の角度の誤差は ±1.2° 以内．
  - **分かったこと1: 曲がり始めが必ず遅れる**．探索の `FRONT_TRIG` の diff は +1.3〜+15.3mm で，マイナス(手前)が1つもない．
    diff が +1.7〜+2.0 のときは FL+FR が 415〜445 で，調べ始めた時にはもう 400 を超えていた．境界での経路計算が 27〜28ms
    (500mm/s で約 14mm)かかり，曲がり始めの位置(境界 + 前のオフセット 約 12.2mm)を過ぎてから調べ始めているため．
    前壁補正の範囲(±15mm)の前半が使えていないので，`SLALOM_FRONT_REF_SUM` はこの遅れを直してから合わせる．
    センサーで決まらず範囲の終わり(+15.3)で曲がったのは 35 回中 7 回(FL+FR が 346〜374)．
  - **分かったこと2: スラロームの後に機体が遅れている**．最短走行で，最初の壁切れ補正は毎回 +7.6mm(探索では +5.2mm)で，
    同じ直線のその後は 0〜−1.6mm にそろう(区画の長さは合っている)．小回り5回の後の壁切れでは +18.8 / +20(上限)/ +18.0mm．
    1回あたり約 4mm，目標の距離より実際の機体が後ろにいる．スリップで外へふくらむことなどが考えられるが，原因は未確認．
  - 次: 電池は 7.5V 以下では走らせない．曲がり始めの遅れ(経路計算を境界より前に済ませるなど)と，
    フェイルセーフのイベントの入れ方をどうするか決める．

- **開発記録の書き方の決まりを足した**: ユーザーの指示．1〜11 章(まとめの章)を変えたときは，変える前と後をこの日記にも書く
  (まとめの章は書き換えると前の内容が消えるため)．さかのぼって，上の「走行のログを確認した」で変えた 11 章の分を書いておく．
  - 「ビルドと実機での確認」 前: 「5.16〜5.21 の変更は 752f795〜1e54424 にコミット・push 済みだが，ビルドも実機での走行もまだ．」
    → 後: 「2026-10-08 に 5×7 の試しの迷路で探索と最短走行が完走した(12章)．」
  - 「迷路で確かめる」の前壁補正の項 前: (補足なし) → 後: 「前壁補正は，曲がり始めの遅れを直してからでないと合わせられない」を足した．
  - 「迷路で確かめる」の地図の項 前: 「`LOG_EV_MAP_CELLS` を，実機で確かめる．」 → 後: 「PC の log_viewer で描いて確かめる(ログには入っていた)．」
  - 「迷路で確かめる」の大回りの項 前: 「大回り(l90 / l180)の試験のログを取る．」 → 後: 同じ文に「試しの迷路の最短走行(LARGE)では大回りが1つも選ばれなかった．」を足した．
  - 「スラロームの曲がり始めの遅れ」 前: 「境界の経路計算(20〜25ms)の対策を決める．」 → 後: 「(実測 27〜28ms，500mm/s で約 14mm)の対策を決める．」
  - 新しく足した項: 「スラロームの後に機体が遅れる」「フェイルセーフのイベントがログに入らないことがある」．

- **境界での経路計算を速くした**(曲がり始めの遅れの対策．ユーザーが「計算そのものを速くする」を選んだ)．
  - **調べたこと**: 探索の1歩ごとに，16×16 の 1024 個のノード(区画 × 向き)全部について，ヒープの優先度付きキューで
    Dijkstra を計算し直していた．Debug のビルドは最適化なし(`-O0`)だった．
  - **① 打ち切り**(`Dijkstra_ComputeFrom`): 探索で要るのは「今いるノードの次の向き」だけなので，そのノードを取り出した
    (コストが決まった)所で止める．取り出す順は変わらないので結果は同じ．
  - **② バケツのキュー**(`bucket_queue.c`，Dial の方法): コストが小さな整数(1区画で増えるのは最大 54)なので，
    コストの値ごとの箱(64 個を輪にして使う)に入れる．取り出しが O(1) になる．コストが大きくて箱に収まらないとき
    (GUI でコストを変えたときなど)は，前のヒープを使う．ヒープとバケツの作業領域は重ねて置いた(RAM の増えは約 1KB)．
  - **同じコストのときの選び方を決めた**: 同じコストで行ける向きが2つあるとき，前は「ヒープから先に出てきた方」で決まっていた．
    キューを替えると選ぶ道が変わるので，「曲がりの小さい向き → Direction の番号の小さい向き」と決めた．
    これで結果がキューの種類によらなくなった．**探索の経路が同じコストの所で変わる**(ユーザーが了承)．
  - **シミュの表示**: 打ち切ると，プランナーの計算結果は今いるノードより遠い所が入っていない．GUI の区画の数字と
    CLI の詳しい表示は，`SimCore_PlannerValues` で全部を計算し直した値を出すようにした．
  - **確かめたこと**(PC，maze_sim と同じ logic 層，609 迷路(my_mazes と mazefiles)，127,240 歩):
    ① は探索の指令の列が前と全迷路で同じ．② は選び方の決まりのため 498 迷路で経路が変わったが，完走は同じ 606
    (3 迷路は前から解けない)，歩数の合計は 126,726 → 125,356(−1.1%)．② のバケツ版とヒープ版(強制)は全迷路で一致．
    GUI の DLL で 40 歩進めた後も全区画の値がそろっていた．
  - **速さ**(PC，1歩あたり，中央値 / 遅い方10%): 前 `-O0` 202 / 240µs，前 `-O2` 80 / 100µs，
    ①② `-O0` 41 / 77µs，①② `-O2` 12 / 24µs．実機の前の値(27〜28ms)に比を当てはめると，①② と `-O2` で約 3ms
    (500mm/s で約 1.5mm)になる見込み(推定．実機では未確認)．
  - **C++ の `std::priority_queue` にする案**も話に出たが，中身は同じ二分ヒープなので速くならない(decrease-key がなく積み直しが増え，
    `-O0` では関数呼び出しが多くかえって遅い)．速さを決めるのはアルゴリズムと最適化．
  - **最適化(-O2)**: ユーザーが CubeIDE で Debug の構成を `-O2` にする．その前に，割り込みとメインで共有する変数に `volatile` が
    付いているか(control_loop.c，logger.c，sensor.c の `ad_*`)と，センサーの発光の待ちがタイマー(TIM6)で測っていて
    最適化で変わらないことを確かめた．
  - 未確認: ビルド，実機．次: 探索して `STEP_INFO` の `plan_ms` と `FRONT_TRIG` の diff を見る(diff にマイナスが出るか)．
  - 11 章の変更: 「スラロームの曲がり始めの遅れ」 前: 「境界の経路計算(実測 27〜28ms，500mm/s で約 14mm)の対策を決める．」
    → 後: 「境界の経路計算(実測 27〜28ms，500mm/s で約 14mm)．打ち切り＋バケツのキューと，コンパイルの最適化(-O2)を入れた
    (2026-10-08，未確認)．実機の `plan_ms` と `FRONT_TRIG` で確かめる．」

- **ログの解析のツールを MATLAB にも作った**(`tools/matlab/`)．ユーザーが「Python の代わりに MATLAB を使えないか」と言い，
  turn_sim の係数合わせ，CSV のグラフ，できれば log_viewer を移すことにした．
  - **分け方**: UART の受信と保存(`get_log.py`)は Python のまま．テキストの中にバイナリが混ざる形式をすでに正しく扱えているので，
    作り直す利点がない．MATLAB は保存された `.csv` / `.bin` を読むだけにした．
  - **作ったもの**: `turn_sim.m`(スラロームのシミュレータと K，C の合わせ)，`log_viewer.m`(ログを見る GUI．Python 版と同じ機能)，
    `plot_log.m`(`get_log.py --plot-file` の代わり．複数のファイルを重ねられる)，共通の部品 `+yc/`(`yuho_common.py` に当たる．
    `.bin`(YLOG1 / YLOG2)の読み込み，`params.h` と `log_event.h` の読み込み，機体と同じ台形とスラロームの計算，`TurnLog` クラス)．
  - **Python 版との違い**: K と C は，同じ格子で探した後に `fminsearch` で格子の目より細かく詰める(この PC の MATLAB には
    Optimization Toolbox がないので，本体の関数を使った)．チェックを外せば Python 版と同じ格子だけの結果になる．
    turn_sim はログのファイルを引数で渡して起動できるようにした．
  - **確かめたこと**(PC，MATLAB R2026b Prerelease．この PC で実際に動く MATLAB はこれだけ．R2024b〜R2026a はフォルダだけ残っている):
    同じログと `params.h` で Python 版と比べた．`params.h` の値(120 個)，`.bin` / `.csv` の読み込み(行数・列の和・イベントの文字)，
    軌道の終わりの位置，s90 / l90 / l180 のオフセット・出口のずれ・柱との距離，横センサーの傾き(8.41 AD/mm，3 個)，
    `logs/slalom` の5本で合わせた K，C，b，RMS(K = 0.0270，C = 0，b = +1.3 mm，RMS 2.27 mm．スリップなしなら 6.70 mm)が
    浮動小数点の桁まで一致した．`fminsearch` で詰めても K = 0.02703，C = 0.0002 で，ほとんど変わらなかった．
    GUI は MATLAB から作ってボタンの処理を呼び，画面を画像に保存して見た(turn_sim の合わせと l180，log_viewer のイベントへの移動・
    再生・STEP で合わせ直す・迷路のクリック)．マウスとキーボードでの実際の操作(ホイールの拡大，← → キー，スペース)はまだ．
  - 次: ユーザーが MATLAB で使ってみる．使い勝手で Python 版と違う所があれば直す．
  - まとめの章の変更: 7.3 節 前: Python のコマンドだけ → 後: MATLAB 版の使い方(`turn_sim`，`log_viewer`，`plot_log`)を足した．
    8 章の表 前: Python のツールと maze_sim だけ → 後: `tools/matlab/` の行を足した．

- **経路計算を速くした後(-O2)のログを確認した**(search_0009，search_0010，fast_0007，fast_0008)．
  - **経路計算の時間**: `plan_ms` は 27〜28ms → **0〜3ms**(68 歩のうち 0ms が 35，1ms が 12，2ms が 8，3ms が 13)．
  - **前壁補正が働くようになった**: `FRONT_TRIG` の diff は前 +1.3〜+15.3mm(マイナスなし) → 今 −7.0〜+15.3mm．
    センサーで決まった回の FL+FR はほぼ 400〜406(前は調べ始めた時に 415〜445 で，もう過ぎていた)．
    ただし diff の中央値は約 +5mm で，範囲の終わり(+15.3)まで届かなかった回が 35 回中 7〜8 回ある．
    距離で決めた曲がり始めの位置では FL+FR が 400 より小さいことが多い．`SLALOM_FRONT_REF_SUM` が大きすぎるのか，
    機体が目標の距離より後ろにいるのか(壁切れ補正も + が多い)は，まだ分けられていない．
  - **探索の時間が 63.4 → 69.1 秒に延びた**: 経路は 32 歩目まで前回と同じで，区画 (3,1) で前回は左へ曲がり，今回は
    同じコストの道の選び方(曲がりの小さい向き)でまっすぐ (2,1) へ進んだ．そこが行き止まりで 180° ターンが1回増えた
    (尻当てを含めて約 5 秒)．シミュの歩数では選び方の決まりで −1.1% だったが，この迷路では悪い方に出た．
    シミュは歩数で比べていて，実機で時間のかかる 180° ターンの重さは見ていない．
  - **-O2 で制御は変わっていない**: 最短走行(同じ経路)の角度の誤差の最大 3.25〜3.43°(前 3.35〜3.46°)，速度の誤差の
    99% 点 75〜80mm/s(前 73〜81)，スラロームの後の角度 ±1.6° 以内，壁切れ補正 [7.6, −1.6, 0, 19.5](前 [7.6, −1.6, 0, 18.8])．
    スラロームの後の遅れ(+19.5〜20mm)は最短走行では経路計算をしないので変わらない(別の原因)．
  - **最短走行の2本のログが閉じられていない**(8MB のまま)．fast_0007 は 10.8 秒(走っている途中，ブロック 12 個ちょうど)，
    fast_0008 は 16.2 秒(ゴールの後の2回目の尻当てで押し付けている途中，ブロック 18 個ちょうど)で切れている．
    書けたのは満杯になったブロックだけで，最後のブロックを書いて閉じる処理(Logger_StreamEnd)が終わっていない．
    探索の2本は閉じられていた．原因は未確認(電源を切った，SD への書き込みが失敗した，止まってしまった，など)．
  - **止まった様子**(ユーザー): fast_0007 は最後のスラロームで壁に当たって止まった(SD の失敗の LED は点いていない)．
    fast_0008 は，ゴールでの尻当てで押し付けている途中で止まり，中心へ戻らなかった．電源はその後に切った．
    コードでは，どの待ちにも `CheckFailSafe`(発動したらログを閉じて LED を点滅)と 5 秒の打ち切り(打ち切ったらログを閉じる)が
    あるので，どちらにも入らずにプログラムが止まった(固まった，またはリセットした)と考えている．原因は未確認．
  - 気づいたこと(コードは変えていない): 速度のフェイルセーフは「誤差が 500mm/s を超えたら」なので，500mm/s で走っていて
    両輪が完全に止まると誤差がちょうど 500 で働かないことがある．
- **最短走行を取り直した**(fast_0009，fast_0010．-O2 のまま)．2本とも完走し，ログも閉じられた(地図のイベント 32 個あり)．
  制御は前と同じ(角度の誤差の最大 3.34〜3.36°，速度の誤差の 99% 点 75〜80mm/s)．スラロームの後の壁切れ補正は +19.5〜20mm で変わらない．
  - 尻当ての間の電池の電圧の最低: fast_0009 7.07〜7.16V，fast_0010 7.01〜7.15V(フェイルセーフの 7.0V のすぐ上)．
    止まった fast_0008 の最後の押し付けは，記録の残った 0.7 秒の間は 7.51V で低くなかった．電圧が原因かは分からない．
  - 止まった原因は再現しなかった．次に起きたときに分かるよう，HardFault のときにモーターを止めて情報を残す処理と，
    起動時にリセットの原因(`RCC_CSR`)を出す処理を提案した(ユーザーの返事待ち)．

- **止まった原因を調べる仕込みを入れた**(ユーザーの依頼．未確認: ビルド・実機)．
  - **HardFault のとき**(`Core/Src/interface/fault_diag.c`，`stm32f4xx_it.c` の USER CODE): まずモーターを止める
    (CPU が止まっても PWM はタイマーが出し続けるため)．CPU が積んだレジスタから止まった場所(PC，LR)を取り出し，
    原因のレジスタ(CFSR，HFSR，MMFAR，BFAR)と一緒に CCMRAM に残す．LED を左右交互に速く(0.1 秒)3 秒点滅させてから，
    自分でリセットする．HardFault_Handler の始めで SP と LR(EXC_RETURN)をアセンブラで渡し，積まれた LR の
    すぐ上を CPU の積んだ所とみなす(最適化で関数の始めの形が変わっても探せるように)．
  - **CCMRAM を使った理由**: 起動の処理(startup)は .data のコピーと .bss のゼロ埋めしかしないので，.ccmram に置いた変数は
    リセットしても残る．リンカスクリプトを変えずに済む．電源を切ると消えるので，固まったままにせず自分でリセットする．
  - **起動したとき**(`main.c` の USER CODE): HAL_Init の後に RCC_CSR のリセットの原因を読んで消し，前回の記録があれば読んで消す．
    UART の初期化の後に「reset cause: ...」を出し，HardFault の後なら PC などを出して LED を約 2 秒同じ形で点滅させる．
  - **ログ**: 走り始め(StartRun)に毎回 `LOG_EV_BOOT`(リセットの原因，HardFault の有無)を入れ，HardFault の後なら
    `LOG_EV_FAULT_PC`，`LOG_EV_FAULT_REG` も入れる(32bit は 16bit ずつに分ける．CSV が有効数字6桁のため)．番号は 7〜9．
  - **分からないこと**: 固まった(無限ループ，割り込みが止まらない)場合は HardFault にならないので，これでは分からない．
    その場合はウォッチドッグ(IWDG)が要る(入れていない)．
  - 11 章の変更: 「最短走行の途中で止まることがある」の項を新しく足した(前はなかった)．

- **前壁補正の閾値を 400 → 420 にした**(`SLALOM_FRONT_REF_SUM`．ユーザーが了承．未確認: ビルド・実機)．
  - **調べ方**: search_0009/0010 の `FRONT_TRIG` 70 回を，「前回位置を直してから何回曲がったか」で分けた
    (位置を直すのは壁切れ補正と尻当て)．
  - **曲がっていない回(14 回)**: diff の中央値 −4mm(400 に届いたのが距離で決めた位置より 4mm 手前)．
    FL+FR は 1mm あたり約 4.5(ログの行から 4〜5)増えるので，正しい位置で見える値は 400 + 4 × 4.5 ≒ 420．
    境界に止めて測った 367(2026-10-07)から 12mm 先として見積もっても 367 + 12 × 4.5 ≒ 420 で合う．
  - **曲がった後の回(56 回)**: diff の中央値 +10mm 前後で，範囲の端(+15.3)に張り付く回も多い．距離で決めた位置で見える
    FL+FR は約 369 で，10〜12mm 手前にいることになる．閾値では合わせず，スラロームの後のずれとして別に調べる
    (範囲を広げて吸収するのは原因を隠すことになるのでしない)．曲がる向きの並び(R，L)で分けても単純な規則はなかったが，
    2本の探索で同じ並びの回はほぼ同じ値で，ばらつきではなく系統的なずれに見える．横のずれ・向きでセンサーの和が
    小さく見えている可能性も残っている．
  - 次: ユーザーがスラロームの多い迷路を組んで走らせる．そのログでスラロームの後のずれの原因を調べる．
  - 10 章の変更: 「前壁補正」 前: 「FL+FR 400(仮)，±15mm」 → 後: 「FL+FR 420(2026-10-08 にログから合わせた)，±15mm」．

- **スラロームの後のずれを調べた**(ユーザーがスラロームの多い迷路を組んで走らせた: search_0011，fast_0011〜0013．
  シミュは MATLAB に移したので，解析も MATLAB(`tools/matlab` の `yc`)で行った．スクリプトは作業用の場所に置いた)．
  - 新しいログの `BOOT` は `reset_flags=14`(電源投入 + 電圧低下 + リセットピン．普通に電源を入れたとき)で，診断の仕込みは動いていた．
    電池は最低 6.97V まで下がっていた(フェイルセーフの 7.0V を下回ったが 0.5 秒続かなかった)．
  - 最短走行(小回り)の壁切れ補正は −20〜+20mm で，**プラスにもマイナスにもなる**．曲がる向きの並びで決まる．
  - **モデル**: 1回の小回りで，出口の向きに前後 a，外へ l ずれる(右と左で別)．向きを変えるたびにずれも回るので，横のずれは
    反対に曲がると前後のずれになる．壁切れ補正の量 = 壁切れの偏り − 前後のずれ，前壁補正の diff = 閾値の偏り − 前後のずれ．
    壁切れ補正は前後を，壁の制御は横を，尻当ては両方を直し，前壁補正で曲がり始めると前後は閾値の偏りに置き換わる．
    上限に張り付いた回は使わない．未知数 7 個(a，l の右左，偏り3つ)の1次式なので最小二乗で解いた．
  - **全部のログ(154 回)**: a = −8.8(右)/−8.1(左)mm，l = +4.9 / +3.1mm(±1.1〜1.3)，壁切れの偏り +4.1mm，
    閾値 400 の偏り +0.9mm，閾値 420 の偏り +3.5mm．残差 4.5mm(RMS)．
  - **迷路で分けると値が違う**: 古い迷路 a ≈ −6mm，l ≈ −2〜+1mm．新しい迷路 a ≈ −12mm，l ≈ +8mm．
    「曲がると遅れる」はどちらでも同じだが，大きさは決めきれない(前後のずれと偏りの項が入れ替わりやすい)．
  - **横のずれをセンサーで直接測った**(探索の境界の `STEP_INFO` の l，r．両側に壁のある区画．傾き 7.8 AD/mm)．
    まっすぐ来た後との差で，右スラロームの後は外へ +2〜+6mm，左スラロームの後は外へ +5〜+6mm．モデルの l ≈ +4 と合う．
  - **スリップの模型と比べた**(MATLAB の `yc.simulate_turn`)．外へ 4mm ずれるのは K ≈ 0.015 で，そのときの前後は約 −3mm．
    前後の −6〜−12mm に K を合わせると外へのずれが 7.5〜14mm と大きすぎる．**外へのずれはスリップで説明できるが，
    前後の遅れにはスリップ以外の分(3〜9mm)がある**．候補(未確認): 曲がっている間に車輪が前後に滑って，車輪で測った距離が
    実際より長くなる，など．
  - **前壁補正の閾値の見直し**: さっき 420 にしたのは，「曲がっていない回の diff が −4mm」からの見積もりだったが，
    モデルでは，その −4mm の大部分は壁切れの偏り(+4mm．壁切れ補正の後は機体が 4mm 前にいる)で説明できた．
    閾値 400 の偏りは +0.9mm でほぼ合っていて，420 では +3.5mm(曲がり始めが遅い)．**420 にしたのは誤りだった可能性が高い**．
  - 次の提案(未実施): 閾値を 400 に戻す．壁切れの位置 `WALL_EDGE_POS_MM` を 78 → 82 にするかは，もう少し確かめてから．
    前後のずれは，1回だけ曲がった後の直線で壁切れを測れば直接分かるので，そういう形の迷路で測る．
  - 11 章の変更: 「スラロームの後に機体が遅れる」 前: 「最短走行で，小回り5回の後の壁切れ補正が +18.8〜+20mm(上限)．
    1回あたり約 4mm(原因は未確認)．」 → 後: 「小回り1回で，出口の向きに 6〜12mm 遅れ，外へ 2〜6mm ずれる(2026-10-08 の解析)．
    外へのずれはスリップ(K≈0.015)で説明できるが，前後の遅れはスリップの分(約 3mm)より大きく，残りの原因は未確認．
    1回だけ曲がった後の壁切れで，前後のずれを直接測る．」

- **前壁補正の閾値を 420 → 400 に戻した**(ユーザーが了承．未確認: ビルド・実機)．理由は上の解析(420 の根拠だった −4mm は
  壁切れ補正の偏りで説明でき，閾値 400 の偏りは +0.9mm でほぼ合っていた)．`params.h` のコメントにも経緯を書いた．
  境界に止めて測った 367 から見積もると 420 になる点はログと合わない(止めて測ったときの条件が違った可能性．未確認)．
  - 次: ユーザーが「まっすぐ(壁切れあり)→ 1回だけ曲がる → 2区画以上まっすぐ(壁切れあり)」の迷路を組んで走らせる．
    曲がった後の最初の壁切れで，1回分の前後の遅れと，壁切れの偏り(`WALL_EDGE_POS_MM`)を直接測る．
  - 10 章の変更: 「前壁補正」 前: 「FL+FR 420(2026-10-08 にログから合わせた)，±15mm」
    → 後: 「FL+FR 400(2026-10-08 に一度 420 にしたが，壁切れの偏りと混ざっていたので戻した)，±15mm」．

- **SD カードからログを取り込むスクリプトを作った**(`tools/sd_import.py`)．ユーザーは機体から UART で吸い出さず，SD カードを
  PC に挿して D ドライブからコピーし，`--bin2csv` で CSV にしていた．この手間をなくすため．
  - **動き**: 取り外しできるドライブ(Windows の `GetDriveTypeW` が REMOVABLE)のうち，ログ(`<dir>/<名前>_NNNN.bin|csv`)が
    あるものを探す(ドライブを引数で決めることもできる．`--wait` で挿されるまで待つ)．`<dir>/` と `sent/<dir>/` を
    `logs/<dir>/` に置き(get_log.py で受け取ったときと同じ場所)，`.bin` の隣に `.csv`(`ev_text` 付き)を作る．
    保存と変換は `get_log.save_file` / `ylog_to_csv` をそのまま使うので，同じ中身は飛ばし，同じ名前で中身が違えば `_dupN` を付ける．
    SD のファイルは消さない・動かさない．
  - **確かめたこと**(PC，SD の形に並べたフォルダで): 取り込みと CSV 作り，2回目はすべて飛ばす，同じ名前で中身が違うと `_dup1`，
    CSV だけ消したときは作り直す，`--dry-run` の表示．本物の SD カードではまだ(このときは D:，E: にログがなかった)．
  - まとめの章の変更: 7.3 節 前: get_log.py，log_viewer.py，turn_sim.py のコマンド → 後: `sd_import.py` の行を足した．
    8 章の表 前: (sd_import.py なし) → 後: `tools/sd_import.py` の行を足した．

- **1回だけ曲がる形の迷路のログを確認した**(search_0012，fast_0014．閾値 400 に戻した版．`BOOT` は reset_flags=14．電池は最低 6.97V)．
  - 1回だけ曲がった後の壁切れ(上限で切る前の値 `edge_pos − expected`): 探索 +13.1mm，最短走行 +25.4，+29.9mm．
    最短走行の方が大きいが，目標の距離と車輪の距離の差はスラロームの前後でも 1mm 未満で，位置の制御の遅れではない．
  - 同じ直線で左右の壁切れが続けて出た3組: 尻当ての後の2組は 右 − 左 = +4.0mm，曲がった後の1組は −9.5mm．
    ここから「横にずれると壁切れの位置が動く(横 1mm で 1.35mm)」と見立てたが，全部のログ(200 回)で合わせると
    その向きでは合わず(一番合うのは逆向きの 0.5mm)，**見立ては取り消した**．
  - **合わせ直した結果**(MATLAB．壁切れの偏りを左右別にし，壁切れは上限で切る前の値を使った): 小回り1回で
    前後 −8〜−12mm，外へ +5〜+7mm(右左ほぼ同じ)，壁切れの偏り 右 +5.3〜+5.5mm・左 −0.5〜−1.8mm，閾値 400 の偏り −1.8〜+0.2mm．
    迷路で分けると前後 −7.5〜−14.5mm，外へ +1〜+10mm で幅がある．
  - 外へ 6mm のスリップ(K≈0.02)では前後の遅れは約 4.5mm なので，**残りの 5〜7mm はスリップ以外**(車輪は回ったのに
    機体が進んでいない．タイヤの前後の滑りなど．未確認)．
  - 提案(未実施): `SLALOM_POST_ADJ_MM` +10，`SLALOM_PRE_ADJ_MM` −6，`SLALOM_FRONT_REF_SUM` 約 373(曲がり始めを 6mm 早めるのに合わせる)，
    壁切れの位置を左右別に(右 83mm，左 78mm．コードの変更が要る)．入れた後のログで，ずれが 0 の周りに来るかを確かめる．

- **スラロームのずれの調整を入れた**(ユーザーが了承．未確認: ビルド・実機)．
  - `SLALOM_PRE_ADJ_MM` 0 → −6(外へ約 6mm ずれる分，早く曲がり始める)，`SLALOM_POST_ADJ_MM` 0 → +10(出口の向きに約 10mm
    遅れる分，長く進む)．最短走行の小回りも同じ値を使う．
  - `SLALOM_FRONT_REF_SUM` 400 → 373: 曲がり始めの位置が 6mm 手前になるので，そこで見える値に合わせた(400 − 6 × 4.5)．
    変えないと前壁補正が元の位置で曲がり始めてしまう．
  - 壁切れの位置を左右で別に: `WALL_EDGE_POS_MM`(78)を `WALL_EDGE_POS_L_MM` 78 と `WALL_EDGE_POS_R_MM` 83 に分け，
    `search_run.c` の `EdgeCorr_Check` で壁切れの側(side)に合わせて選ぶようにした．
  - MATLAB の模型で柱までの距離を確かめた: 調整なし 78〜84mm，調整あり 78〜84mm(機体の中心から)で変わらない．
  - 注意: 曲がり始めが境界から約 6mm 先になったので，前壁補正の範囲(±15mm)の前の方は，境界での計算(約 2ms ≒ 1mm)の後の
    約 5mm 分しか使えない．
  - 次: 満充電で探索と最短走行を走らせ，ずれ(合わせ直したモデルの a，l)と，壁切れ補正・前壁補正のずれが 0 の周りに来るかを見る．
  - 10 章の変更: 「壁切れ補正」 前: 「境界 + 78mm(仮)，±30mm，最大 20mm」 → 後: 「境界 + 左 78mm / 右 83mm，±30mm，最大 20mm」．
    「小回り」 前: 「…，オフセット約 12mm」 → 後: 「…，オフセット約 12mm，ADJ 前 −6mm・後ろ +10mm」．
    「前壁補正」 前: 「FL+FR 400(2026-10-08 に一度 420 にしたが，壁切れの偏りと混ざっていたので戻した)，±15mm」
    → 後: 「FL+FR 373(曲がり始めを 6mm 早めたのに合わせた)，±15mm」．

- **スラロームの調整の後のログを確認した**(search_0013，fast_0015〜0017．満充電 8.0〜8.1V から．`BOOT` は reset_flags=14)．
  - **壁切れ補正**(上限で切る前の値．同じ迷路の前後で比べた): 前(fast_0014 + search_0012)23 回 RMS 14.5mm・平均 +10.6mm・
    上限に張り付き 4 回 → 後(fast_0017 + search_0013)22 回 **RMS 5.0mm・平均 +0.4mm・張り付き 0 回**．
    1回だけ曲がった後の壁切れは，最短走行 +25/+30 → +7/+5mm，探索 +13 → 0mm．尻当ての後の右の壁切れは +4 → 0/−2/−2mm．
  - **前壁補正**: 15 回中 14 回がセンサーで決まり，FL+FR は 373〜379．diff は −5.8〜+14.3mm(平均 +2.0，RMS 5.8)．
    1回(diff −6.2，FL+FR 398)は調べ始めた時にもう閾値を超えていた(曲がり始めが境界から約 6mm 先になり，範囲の手前側が
    約 5mm しか使えないため．予想どおり)．
  - **残っているずれ**(MATLAB で調整後の2本だけ合わせた．37 回): 右 前後 −5.8・外 −3.4mm，左 前後 +3.4・外 +0.3mm，
    壁切れの偏り 左 +0.6・右 −1.0mm，閾値 373 の偏り +1.7mm．回数が少なく，0 と区別できるほどではない．
  - **fast_0016 はフェイルセーフ(速度の偏差)で止まった**: 原因はスタートでのジャイロのゼロ点の測り直しの失敗．
    `GYRO_RECAL offset_lsb` が 132.7(他のログは全部 5.2〜6.0)で，差の 127 LSB は約 7.8dps(16.4 LSB/dps)．
    止まっていても −8〜−25dps が出ていて，走り始めには向きが −39.7° ずれていた．そのまま走って左の壁に寄り，
    6.33 秒に壁に当たって引っかかり，車輪が空転した後に止まって発動した．測っている間(手をかざした後の 0.5 秒)に
    機体が動いたと考えている(未確認)．今回の調整とは関係ない．ログは閉じられ，`FAILSAFE` のイベントも残っていた．
  - 次の提案(未実施): ゼロ点の測り直しで，値が起動時の値から大きく外れたら(または測っている間に揺れていたら)
    やり直す・知らせる仕組み．

- **壁を読む位置を境界の 12mm 手前にした**(前壁補正の範囲の手前側を使えるようにするため．ユーザーの依頼．未確認: ビルド・実機)．
  - 理由: 曲がり始めが境界から約 6mm 先になったので，前壁補正の範囲(±15mm)の手前の端は境界の約 9mm 手前．
    境界で壁を読んで曲がると決めていたので，手前側は約 5mm しか使えていなかった(search_0013 で1回当たった)．
  - `SEARCH_WALL_READ_BEFORE_MM`(12mm．0 なら前と同じ)を `params.h` に足した．`GoToNextBoundary` と `SlalomTurn` で，
    境界(`dp->ref_mm`)のこの距離だけ手前で壁を読む．読んだ後も機体は同じ速さで進み続け，その間に次の動きを決める
    (走りながら次の直進を始めると今の速さからつながる作りになっていることを `UpdateProfile` で確かめた)．
    `WaitTargetDistanceEdge` に「手前で待つのをやめる距離」の引数を足した．
  - 壁の判定への影響: 探索のログ 56 回で，境界で読んだ値と 11mm 手前の値で判定が変わる回はなかった
    (閾値までの余裕の最小: 左 54，右 96，前 13)．
  - 副作用: 曲がるときは壁の制御を切るのが 12mm 早くなる．スラロームの後は，曲がり終わってから約 9mm 進んだ所で壁を読む
    (後ろのオフセットが約 21mm なので)．
  - 次: 探索のログで，`FRONT_TRIG` の diff が手前側(−15mm まで)にも出るか，壁の判定・壁の地図がおかしくならないかを見る．

- **壁を手前で読むようにした後のログを確認した**(search_0014，fast_0018〜0021)．
  - 前壁補正: 15 回すべてがセンサーで決まり，FL+FR は 374〜378．diff は −10〜+11mm で，手前側(−10mm)でも曲がり始めた．
    調べ始めた時にもう閾値を超えていた回はなかった．
  - 壁の判定: search_0013(境界で読んだ)と search_0014(12mm 手前で読んだ)で，56 歩の位置・向き・壁・指令がすべて同じ，
    壁の地図も同じ．閾値までの余裕の最小は 左 79，右 103，前 41．
  - 最短走行 fast_0018/0020/0021 は完走(壁切れのずれ −11〜+9mm)．fast_0019 はジャイロのゼロ点が 75.4 で(fast_0016 と同じ失敗)，
    1.8 秒で止めた(ログは閉じられていない)．原因は分かっているのでそのまま．
- **コミット・マージ・プッシュをした**(ユーザーの依頼)．feature/maze に内容ごとの 6 コミット(経路計算の高速化，-O2，診断の仕込み，
  探索・最短走行の調整，MATLAB 版のツールと sd_import.py，Debug/yuho.bin)を作り，master に `--no-ff` でマージした(7584867)．
  `.settings/stm32cubeide.project.prefs` は入れていない．
  - 失敗: GitHub の既定のブランチは master なのに，確かめずに `main` へマージしようとして，手元の古い main(c91789f)を
    GitHub に新しいブランチとして作ってしまった．ユーザーの了承を得て GitHub から消した．

- **ブランチ作りを Claude の担当にした**(ユーザーの指示)．新しいまとまった作業を始めるときに，ファイルを変える前に
  `feature/内容` か `fix/内容` で作る(元は master の最新，まだ master に入っていない作業の続きならそのブランチ)．
  master へのマージと master へのプッシュは，これまでどおりユーザーに頼まれてから行う．

- **スラロームのずれの調整を，速さによらない1つのモデルにした**(ブランチ feature/slalom-speed-model．feature/maze から作った．
  未確認: ビルド・実機)．
  - 最初は「選べる速さごとに調整値の組を持つ」案だったが，ユーザーの指示で「モデルがちゃんとしていれば同じモデルで使える」形にした．
  - **モデル**: (1) スリップ(1次遅れ dβ/dt = (K v ω − β)/C)を機体の中で選んだ速さで計算し(tools/matlab の yc.simulate_turn と
    同じ計算を `logic/control/slalom.c` の `Slalom_SlipError90` に入れた)，出口のずれを打ち消すように前と後ろのオフセットを直す
    (turn_sim の ADJ の提案と同じく3回くり返す)．(2) スリップで説明できない前後の遅れは，横加速度に比例してタイヤが滑ると仮定し，
    速さの2乗に比例させる(仮定．未確認)．(3) 前壁補正の閾値 = 400 + 4.5 × 前の調整分(`Slalom_FrontRefSum`)．
  - **係数**: 500mm/s で外へ 6mm・前後に 10mm 遅れ(これまでの解析)を再現するよう，MATLAB で K を合わせた．C は決まっていないので
    0.02(K = 0.0219，スリップ以外の遅れ 5.1mm)．C を 0〜0.05 で変えても，700mm/s までの調整値の違いは 1mm 以内だった．
  - **予想される調整値**: 300mm/s 前 −2.1・後ろ +3.7(閾値 391)，400mm/s −3.8・+6.5(383)，500mm/s −6.0・+10.0(373．これまでと同じ)，
    600mm/s −8.7・+14.3(361)．PC の gcc で機体の `slalom.c` を動かして，MATLAB の値と一致することを確かめた．
  - `params.h`: `SLALOM_SLIP_K`，`SLALOM_SLIP_C_S`，`SLALOM_EXTRA_LAG_MM`，`SLALOM_FRONT_REF_SUM_AT_PRE0`，`SLALOM_FRONT_SUM_PER_MM` を足し，
    `SLALOM_PRE_ADJ_MM` / `SLALOM_POST_ADJ_MM` は「モデルの上に手で足す分」として 0 にした．`SLALOM_FRONT_REF_SUM` はなくした．
  - **最短走行の小回りの速さを選べるようにした**(これまでは 500mm/s 固定)．直進の最高速度の次に `SPEED_SELECT_FAST_SMALL_V_MM_S`
    (300〜600mm/s)から選ぶ．探索・最短走行・SLALOM の試験の小回りは，どれも同じモデルでオフセットを計算する．
  - 注意: 600mm/s では前のオフセットが 3.7mm になり，前壁補正の範囲の手前の端が境界の約 11.4mm 手前になる．壁を読むのは 12mm 手前で，
    経路の計算(約 1.5mm)の後に調べ始めるので，範囲の手前の約 1mm は使えない．
  - 次: 300・400・600mm/s で探索(と最短走行の小回り)を走らせ，壁切れ補正と前壁補正のずれが 0 の周りに来るかで，モデル(特に
    「速さの2乗」の仮定と C)を確かめる．
  - 10 章の変更: 「小回り」 前: 「500mm/s・450dps・8000dps²(速さに合わせて ω ∝ v，α ∝ v²)，オフセット約 12mm，ADJ 前 −6mm・後ろ +10mm」
    → 後: 「500mm/s・450dps・8000dps²(速さに合わせて ω ∝ v，α ∝ v²)，オフセット約 12mm，調整はモデルで計算(K 0.0219，C 0.02，
    スリップ以外の遅れ 5.1mm @500mm/s ∝ v²)．500mm/s で前 −6mm・後ろ +10mm」．「前壁補正」 前: 「FL+FR 373(曲がり始めを 6mm 早めたのに合わせた)，±15mm」
    → 後: 「FL+FR 400 + 4.5 × 前の調整分(500mm/s で 373)，±15mm」．

- **走る前に速さを選ぶ画面が，回しても変わらなかったのを直した**(ユーザーの報告．探索で「速度を選べない」)．
  - 原因: 値を選ぶ画面(`ModeUI_SelectValue` → `SelectIndex`)は右タイヤの回転を `Encoder_GetDeltaR()`(前回読んだ時からの差分．
    読むと消える)で読んでいたが，モードを決めた直後から 1kHz の制御の割り込みが同じ関数で差分を持っていくので，画面にはほぼ 0 しか
    届かなかった(`main.c` のコメントの「奪い合い」．モードの選択だけは割り込みの前に済ませてあった)．探索・最短走行・各試験の
    速さの選択は，どれも効いていなかったはず．
  - 直し方: カウンタの値をそのまま返す `Encoder_GetCountR()` を足し，`SelectIndex` はそれを読んで自分で差を取る(割り込みの差分に触らない)．
  - 表示: 値を選ぶときは棒グラフ(n 番で LED1〜n を点ける)にし，1番(一番遅い値)から始める(ユーザーの指定)．値の並びは遅い順．
    モードの選択の表示(LED n, n+1)は変えていない．
- **探索のスラロームの速さを，直進の速さとは別に選べるようにした**(`SPEED_SELECT_SEARCH_TURN_V_MM_S`．直進の速さの次に「SLALOM」で選ぶ)．
  - 曲がるかどうかは境界の 12mm 手前で決まるので，そこからでは減速が間に合わない．そのため探索の直進は，区画ごとに
    境界で小回りの速さになるように走る(区画の中では直進の速さまで加速する)．前のオフセット・前壁補正の範囲・後ろのオフセットは
    小回りの速さのまま走る．
  - 区画の中で速さが変わるので，壁切れ補正で境界を直したら，直した境界で小回りの速さになるようにプロファイルを引き直すようにした
    (`WaitTargetDistanceEdge`．最短走行の `FastStraightTo` と同じ)．
  - 未確認: ビルド・実機．

- **速さを変えた探索・最短走行のログを確認した**(search_0015〜0019，fast_0022〜0023)．速さを選ぶ画面の修正で，300〜600mm/s で走れた．
  - 壁切れ補正のずれの平均: 300mm/s +3.3/+3.4mm，400mm/s +1.9mm，500mm/s 0.0/+0.9mm，600mm/s −0.4mm(遅いほど機体が後ろ)．
  - 速さごとに小回り1回の残りのずれを合わせた(右左同じとした．MATLAB): 前後の残り 300mm/s −7.2mm，400mm/s −9.9mm(回数少)，
    500mm/s −2.4mm，600mm/s +0.6mm．外の残りは ±3mm 以内，壁切れの偏り・前壁補正の閾値の偏りもどの速さでも ±3mm 以内．
  - 必要だった後ろの調整を逆算すると 300mm/s 約 11mm，500mm/s 約 12mm，600mm/s 約 14mm で，速さによらずほぼ同じ．
    スリップの分を引いた「スリップ以外の遅れ」は約 9 / 7.5 / 7mm で，**速さの2乗に比例という仮定は外れていて，ほぼ一定(約 8mm)**．
    時間の遅れなら速さに比例するはずなので，速さによらない一定の距離だけ遅れる原因がありそう(未確認)．
  - 提案(未実施): スリップ以外の遅れを「速さによらず一定」に変える．次の長い走行のログで確かめてから決める．
  - search_0016(400mm/s)は 26 秒で終わっていた(イベントなし．途中で止めたと考えている)．電池は最低 7.06V．
- **長い走行のモード(TEST → LONG_LOG)を作った**(ユーザーの依頼．未確認: ビルド・実機)．
  - 探索を直進 × スラロームの速さの組み合わせ 16 通りで2回ずつ(32 回)，最短走行を小回り(小回りの速さ4通り)と大回り(1通り)で
    2回ずつ(10 回)，置き直さずに続けて走る(`LONG_LOG_REPEAT`)．最短走行の後は，ゴールから自分でスタートへ戻る
    (探索の帰りと同じ計算．`ReturnToStart`．速さ `LONG_LOG_RETURN_V_MM_S`)．1回ごとに別のログ(search / fast / back)．
  - 電池: 走る前に止まった状態の電圧が `LONG_LOG_MIN_VBAT_V`(7.5V)より低ければ止まり，次に走る「段」と「番号」を LED の棒グラフで
    交互に出す(段の間は直結の LED も全部点く)．電池を替えたら，モードを選んだ後に段と番号を選んで続きから始める
    (選ぶ画面の LED が 15 個までなので2回に分けた．段 1〜4 が探索(直進の速さ)，5 が最短の小回り，6 が最短の大回り)．
  - ログに `LOG_EV_LONG_RUN`(46: 何番目・段・番号・速さ2つ)を入れる．`LOG_EV_RUN_TYPE` の kind 2 を帰り道にした．
  - 作り直した所: `RunSearch` の繰り返しを `SearchLoop` に，`StartRun` を `StartRunBegin`(記録と制御の開始)と尻当てに，
    探索の後の処理を `FinishSearchRun` に，最短走行の経路の準備を `PrepareFastRoute` に分けた(探索・最短走行のモードも同じ関数を使う)．
  - 最初の1回だけ手かざしで始める．全部終わったら段 7 番 1 を出して止まる．

- **長い走行のモードのログを確認した**(search_0020〜0032．途中まで)．
  - 自動で続ける走りは動いた: 段1(直進 300mm/s × スラローム 300〜600mm/s × 2回)の8回が置き直さずに完走した(search_0023〜0030)．
    `LONG_RUN` のイベントも入っていた．
  - search_0022(#3)は 400mm/s でふつうに直進している途中で切れていて，ファイルが閉じられていない．次の起動の `BOOT` は
    reset_flags=14(電源投入)・fault=0 で，HardFault ではない(電源を切ったと考えている．未確認)．その後，段1の1番からやり直していた．
  - **段1のずれ(直進 300mm/s，スラロームの速さごと)**: 前後の残り 300 −7.0，400 −4.4，500 −1.6，600 +0.7mm．
    必要だったスリップ以外の遅れは 8.8 / 7.7 / 6.7 / 6.6mm で，前回と同じく**速さによらずほぼ一定(7〜9mm)**．
  - **search_0031(#9，直進 400・スラローム 300)で起きたこと**:
    25.5 秒の左の壁切れで補正 +12.2mm(遅れている)と出たが，次の2回の前壁補正は −13.7mm，−14.9mm(前に出ている．2回目は
    調べ始めた時にもう閾値を超えていた)で逆向き．直前に左のセンサーが 457 と左の壁にかなり寄っていて，**壁切れの読み違いで
    12mm 余計に進ませた**可能性が高い．ゴール(1,0)の「真ん中」で前に出すぎて止まり，超信地旋回で前の壁にぶつかって(FL 1457)
    向きが 17° ずれた．そのまま帰り始め，33〜35 秒は壁に押し付けられて動けない(ジャイロの角度が変わらない)のに，目標の向きだけが
    スラローム2回分回り，スタートに着いたと判断した．**フェイルセーフは働かなかった**(速度の偏差は「500mm/s 超」なので，
    300〜400mm/s で止まっても届かない．向きのずれ(最大約 180°)で止める仕組みはない)．壁の方を向いて終わり，
    次の search_0032 は「スタート区画は前が開いている」の決まりで進んで壁の手前で行き止まりと判断し，すぐ FAILED で止まった．
  - 電池: #9 は走る前(止まった状態)が約 7.6V で閾値 7.5V を超えていたが，走っている間は 6.73V まで下がった．
  - 提案(未実施): (1) スリップ以外の遅れを速さによらず一定(約 7.5mm)にする．(2) フェイルセーフに「向きの誤差が大きい状態が続いたら止める」を足し，
    速度の偏差を速さに対する割合でも見る．(3) 長い走行の電池の閾値を上げる．(4) 壁切れの読み違い(横に寄っているとき)を調べる．

- **スリップ以外の遅れを速さによらず一定(7.5mm)にした**(ユーザーが了承．未確認: ビルド・実機)．`SLALOM_EXTRA_LAG_MM` 5.1 → 7.5，
  `Slalom_SmallTurnOffsets` で速さの2乗をかけるのをやめた．後ろの調整は 300/400/500/600mm/s で +9.3/+10.7/+12.4/+14.5mm
  (前の調整と前壁補正の閾値は変わらない)．PC で機体の `slalom.c` を動かして確かめた．
  - 10 章の変更: 「小回り」の「スリップ以外の遅れ 5.1mm @500mm/s ∝ v²．500mm/s で前 −6mm・後ろ +10mm」
    → 「スリップ以外の遅れ 7.5mm(速さによらず一定)．500mm/s で前 −6mm・後ろ +12.4mm」．
- **壁切れの読み違いを調べた**(ユーザーの依頼)．
  - 壁切れの直前にその側の壁へどれだけ寄っていたか(センサーの値から)と，壁切れの位置の関係を全部のログ(427 回)で見たが，
    関係は弱い(相関 0.1〜0.2)．ふつうの寄り(±5mm)では左右とも見込みの位置(左 78mm，右 83mm)の近くで起きている．
  - search_0031 の +12.2mm は特別な場面だった: その直前のスラロームのイベントの速さが **v=366〜370mm/s** で，選んだ 300mm/s ではなかった
    (角速度・角加速度は 300mm/s 用の 270dps・2880dps²)．直進 400mm/s から減速しきる前に曲がり始め，300mm/s 用の角速度のまま
    370mm/s で曲がって大きく外へふくらみ(曲がった後に左のセンサーが 1247)，左へ約 18mm 寄ったまま次の直線に入って(壁の制御は上限 5° に
    張り付き)左の壁切れを読み違えた．**原因は，探索の直進とスラロームの速さを分けたときの不具合**．
    - 境界へ向かう直進は「境界で」スラロームの速さになるように減速していたが，曲がると決めるのは境界の 12mm 手前で，そこではまだ速かった．
    - 曲がる前の直進は，今の速さが上限より速いとき，区間の終わりの近くでしか減速しない作りだった(`VelocityProfile`)．
      前壁補正で区間の途中から曲がり始めると，減速しないまま曲がった．
  - 段1(直進 300，スラロームの方が速いか同じ)は加速する側なので起きていない: 各ログのスラロームのイベントの速さが選んだ速さと一致していた．
    モデルの確認の結果はそのまま使える．
  - **直した**(未確認: ビルド・実機): (1) 境界へ向かう直進は，壁を読む所(境界の 12mm 手前)までにスラロームの速さにする
    (壁切れ補正の引き直しも同じ)．(2) `VelocityProfile` で，始めの速さが上限より速いときはすぐに上限まで減速する
    (PC で確かめた: 370 → 300mm/s を 11.7mm で．止まった状態から加速して止まる使い方は今までどおり)．

- **K と C はまだ決まっていない**と分かった(ユーザーの質問から調べた)．段1のログで，曲がった後の横のずれをセンサーで直接測り，
  モデルの補正(前の調整)を足し戻すと，実際の外へのずれは スラロームの速さ 300/400/500/600mm/s で 右 4.6/6.9/6.5/7.7mm，
  左 8.6/10.5/10.6/11.1mm．速さの2乗(4倍)ではなく 3mm ほどしか増えず，**速さによらない一定の分(右 約 4mm，左 約 8mm)と，
  スリップの分(500mm/s で 2.3〜2.9mm．K は今の半分の約 0.01)**に分かれそう．以前の turn_sim の合わせ(K≈0.008，共通の横のずれ
  b=+8.3mm)とも近い．C はこの速さの範囲では影響が小さく決められない．センサーの mm への換算(7.8 AD/mm)は概算．
  - 次: SLALOM の試験で横のずれをきれいに測って，MATLAB の `yc.fit_slip` で K，C，一定の横のずれ(右と左)を合わせる．
- **小回りの連続の試験のモード(TEST → SLALOM_SWEEP)を作った**(ユーザーの依頼．未確認: ビルド・実機)．
  - 右で行って左で戻る: A(西と南に壁)から北向きに出て右の試験 → B(東へ2・北へ2区画．東と南に壁)の真ん中で東向きに止まる →
    180° 回る → 左の試験で A に南向きで戻る → 180° 回る，をくり返す．置き直さずに右と左のログが同じ数ずつ取れる．
  - 始めと終わりの速さを選び，`SPEED_SELECT_SLALOM_TEST_V_MM_S` の順に上げながら，各速さで `SLALOM_SWEEP_REPEAT`(2)往復する．
    打ち切り・電池の低下(`LONG_LOG_MIN_VBAT_V`)・全部終わったときは，その時の速さの番号を棒グラフで点滅させて止まる．
  - フェイルセーフで止まったときも，そのときのログを SD に残すようにした(SLALOM の試験の `CheckFailSafe`．限界を後で見るため)．
  - **試験の速さを 300〜900mm/s に広げた**(ユーザーの「ぎりぎりまで」)．試験は曲がり始めまで 270mm + 前のオフセットしかなく，
    2000mm/s² で加速しきれるのは約 900mm/s まで(PC で計算．1000mm/s 以上は届かず，選んだ速さより遅く曲がり始めてしまう)．
    横加速度は 800mm/s で約 1.0G，900mm/s で約 1.3G．今のモデルでは 800mm/s で前のオフセットが −8.9mm，900mm/s で −20.9mm(早く曲がり始める)．
  - 探索・最短走行で選べる速さは，この試験で限界が分かってから広げる．

- **直進の速さを 1.5m/s まで選べるようにした**(ユーザーの依頼．未確認: ビルド・実機)．小回りを速くするのは，900mm/s までの
  小回りの連続の試験のログを取ってから考える(小回りは形が決まっているので，1.5m/s では横加速度が約 3.6G になる)．
  - 最短走行の直進 `SPEED_SELECT_FAST_V_MM_S`: 600〜1200 → 600/800/1000/1200/1400/1500mm/s．
  - 探索の直進 `SPEED_SELECT_SEARCH_V_MM_S`: 300〜600 → 300/400/500/600/800/1000/1200/1500mm/s(境界ではスラロームの速さに落とすので，
    速く走れるのは何区画も続けて進むときだけ)．
  - STRAIGHT の試験は専用の `SPEED_SELECT_STRAIGHT_V_MM_S`(300〜1500mm/s)にした(それまでは探索の表を使っていた)．6区画(1080mm)・
    2000mm/s² では最高でも約 1470mm/s なので，1.5m/s を保つ所まで見るなら `STRAIGHT_TEST_SECTIONS` を 8 くらいにする．
  - 長い走行のモードは探索の表を使っていたので，走る回数が増えないよう専用の `LONG_LOG_SEARCH_V_MM_S`(300〜600mm/s)にした．

- **プラントモデルの解説を書いた**(`docs/plant_model.md`．ユーザーの依頼)．電源(電圧で考える)，車輪の FF の式，超信地旋回の摩擦，
  寸法の実効値，旋回中のスリップとスリップ以外の遅れ，その上のフィードバックを，今の値と決めた経緯(コミットとログ)つきでまとめた．
  - 書いていて分かったこと(計算．実機で測った値ではない): FF の式 電圧 = FRIC + GAIN·v + ACC·a は，不感帯つきの1次遅れ
    T dv/dt + v = K(電圧 − FRIC) の逆になっていて，K = 1/GAIN，T = ACC/GAIN．左 K 1333mm/s/V・T 0.37s，右 K 2000mm/s/V・T 0.55s．
    divergence_v3 の PRBS の同定(K 1519，T 0.445s)と形も桁もほぼ同じ．
  - 車輪速度のループの帯域は KP/ACC ≈ 55 rad/s(時定数 約 18ms)で，ACC が左右同じなので K，T が違っても左右で同じになる．
    外側の角度(0.06s)・位置(0.1s)より十分速い．

- **明日(速くするのが目標．スラロームも含む)のための準備をした**(ユーザーの依頼．未確認: ビルド・実機)．
  - 加速度を選べるようにした: `SPEED_SELECT_ACCEL_MM_S2`(2000/3000/4000/5000/6000/8000/10000mm/s²)．STRAIGHT の試験と最短走行の直進
    (`s_fast_accel`．それまでは `FAST_ACCEL_MM_S2` 固定)で，速さの次に「ACCEL」で選ぶ．探索は 2000mm/s² のまま．
  - **直進の連続の試験(TEST → STRAIGHT_SWEEP)を作った**: 速さと加速度の範囲(FROM / TO)を選び，加速度ごとに速さを上げながら各組み合わせで
    `STRAIGHT_SWEEP_REPEAT`(2)回走る．両側と両端に壁のある通路(`STRAIGHT_TEST_SECTIONS` + 1 区画)の端に置いて手かざしで始めると，
    尻当てでそろえて走り，反対の端で 180° 回って尻当てして戻る，をくり返す(置き直さない)．ログは `sweep_NNNN`．限界を見るため，
    PWM・FF・I 項・車輪の目標・目標の加速度・壁センサーを記録する．
  - STRAIGHT の試験と直進の連続の試験も，フェイルセーフで止まったときのログを SD に残すようにした．

### 2026-10-09 の予定(速くする．スラロームも含む)

ログを取れる時間が限られているので，迷路の組み替えが少ない順にまとめた．自動で続くモード(STRAIGHT_SWEEP，SLALOM_SWEEP，LONG_LOG)は
止まると LED の棒グラフで「何回目・何番の速さ」を出すので，見て覚えておく(ログからも分かる)．

**前日(10-08 の夜)にやること**
- 電池を全部充電する．SD カードの古いログを PC に移して空ける．
- ビルドして書き込み，起動を確かめる(10-08 はコードの変更が多く，まだ一度もビルドしていない)．

**順番**

| # | 内容 | 迷路の形 | 取り方(モードと選ぶ値) | 分かること | 目安 |
|---|---|---|---|---|---|
| 1 | 動作の確認 | A: 試しの迷路 | RUN → SEARCH．SPEED 400，SLALOM 300 で1回．続けて SPEED 800，SLALOM 500 で1回 | 10-08 の変更(減速の修正，壁を手前で読む，モデル)で探索がふつうに走るか．帰り道の既知の区間をまとめて走るか(`KNOWN_RUN` のイベント．大回りは SPEED 600 以上のとき)．おかしければ `SEARCH_KNOWN_FAST_ENABLE` を 0 にしてビルドし直す | 5分 |
| 2 | 直進の限界(速さ) | B: 直線の通路 | TEST → STRAIGHT_SWEEP．SPEED 600〜1500，ACCEL 2000〜2000 | 1.5m/s で制御がついてくるか(PWM の張り付き，速度の誤差，電池の電圧) | 5分 |
| 3 | 直進の限界(加速度) | B: 直線の通路 | TEST → STRAIGHT_SWEEP．SPEED 1000〜1000，ACCEL 3000〜10000 | タイヤが滑り出す加速度，フェイルセーフで止まる所 | 5分 |
| 4 | スラロームの限界 | C: L字の通路 | TEST → SLALOM_SWEEP．FROM 300，TO 700．様子を見て 800〜900 | K，C，一定の横のずれ(右と左)，横の限界 | 10分 |
| ― | 解析(Claude) | ― | ログを渡す | モデルの係数の更新，最短走行に使う値(加速度，小回りの速さ)を決める | 15〜30分 |
| 5 | 最短走行を速くする | A: 試しの迷路 | まず RUN → SEARCH を1回(地図を作る)．次に RUN → FAST_SWEEP で SPEED・ACCEL・SMALL の範囲と走り方を選び，1回の手かざしで全部の組み合わせを走る(毎回スタートへ自分で戻る)．限界の 8 割くらいから範囲を選ぶ | 実際にどこまで速く走れるか | 残りの時間 |
| 6 | (時間があれば)長い走行 | A: 試しの迷路 | TEST → LONG_LOG．PART 2，STEP 1 から | 直進がスラロームより速い組み合わせ(減速の修正の確認)，最短の後の帰り道 | 長い |

この順にした理由: 1 で 10-08 の変更が壊れていないかを最初に短く確かめる．直進の限界(2，3)はスラロームの試験の加速にも最短走行にも効き，
同じ通路で続けて取れる．スラロームのモデルを直してから(4 の後の解析)最短走行をすると，速さを上げたときに失敗しにくい．

**A: 試しの迷路**(5×7．search_0030 で機体が見た壁．`?` は見ていない壁．S がスタート(北向き)，G が試しのゴール (1,0))

```
+ ? +---+---+---+---+
|                   |  y=6
+ ? +   +---+---+   +
|   |   |   ?   |   |  y=5
+ ? +   + ? + ? +   +
|       |   ?   |   |  y=4
+---+   +---+---+   +
|               |   |  y=3
+   +   +---+---+   +
|   |   |           |  y=2
+   +   +   +---+---+
|   |   |   |   ?   ?  y=1
+   +---+   + ? + ? +
| S | G     |   ?   ?  y=0
+---+---+---+---+---+
  x=0   x=1   x=2   x=3   x=4
```

**B: 直線の通路**(STRAIGHT_SWEEP．`STRAIGHT_TEST_SECTIONS` + 1 区画．今は 6 なので 7 区画．両側と両端に壁)

```
+---+---+---+---+---+---+---+
| > |   |   |   |   |   |   |    > : 置く所(端の区画．通路の向き)
+---+---+---+---+---+---+---+
```
- 端の区画に通路の向きに置いて手をかざす．尻当てでそろえて走り，反対の端で 180° 回って尻当てして戻る，をくり返す．
- 6 区画(1080mm)では 2000mm/s² で最高でも約 1470mm/s(加速してすぐ減速)．1.5m/s を保つ所まで見るなら
  `straight_test.c` の `STRAIGHT_TEST_SECTIONS` を 8 にして 9 区画の通路にする(ビルドし直しが要る)．
- 選ぶ値: SPEED FROM / TO，ACCEL FROM / TO(加速度ごとに速さを上げる．各組み合わせで2回 = 行きと帰り)．

**C: L字の通路**(SLALOM_SWEEP．A と B の位置関係が大事)

```
        x=0   x=1   x=2
      +---+---+---+
 y=2  |         B |     B=(2,2): 東と南に壁(B で 180° 回って尻当てする)
      +   +---+---+     (1,2)(2,2): 北と南に壁(右の試験の出口の直線．横のずれを測る)
 y=1  |   |             (0,1): 西と東に壁(左の試験の出口の直線)
      +   +
 y=0  | A |             A=(0,0): 西・東・南に壁．北向きに置く
      +---+
```
- 右の試験: A から北へ1区画半 → (0,2) で右に曲がる → 東へ1区画半で B の真ん中に東向きで止まる．
- 左の試験: B で 180° 回って西向き → 西へ1区画半 → (0,2) で左に曲がる → 南へ1区画半で A の真ん中に南向きで止まる → 180° 回る．
- 選ぶ値: FROM / TO(100mm/s ずつ上げる．各速さで右と左を2往復 = 4本)．300〜700 で様子を見てから 800〜900．
- 800mm/s で横加速度 約 1.0G，900mm/s で 約 1.3G．今のモデルの K は大きすぎる可能性が高く，高い速さでは早めに曲がり始めすぎるかもしれない．

**気をつけること**
- 3 では，加速度を上げるとフェイルセーフで止まることがある．止まったときのログも SD に残るので，それが「限界」のデータになる．
  止まったらモードを選び直して続きを取る．
- 電池は試験ごとに満充電に近いものに替えると条件がそろう(長い走行・連続の試験は 7.5V 未満で止まる)．
- 5 の最短走行は SEARCH で作った地図を使う．SEARCH は探索の SPEED / SLALOM を選んでから走る(速さを選ぶ画面は1番から始まる)．

**モードの選び方**(起動後．右のタイヤを回して選び，ボタンで決める．タイヤ 1/4 周で1つ進む．端では輪のようにつながる)

1. 一番上の階層: 1 RUN，2 TEST，3 SD(LED n と n+1 が点く)．
2. その中のモード(LED n と n+1 が点く):
   - RUN: 1 SEARCH，2 SEARCH_ADACHI，3 FAST_RUN，4 FAST_SWEEP
   - TEST: 1 SENSOR，2 SENSOR_LOG，3 VEL_PID，4 STRAIGHT，5 PIVOT，6 SLALOM，7 LED_TEST，8 PARTY，
     **9 LONG_LOG，10 SLALOM_SWEEP，11 STRAIGHT_SWEEP**
   - SD: 1 SD_DUMP，2 SD_DUMP_ALL，3 STREAM_TEST
   - **注意**: 確実に光るのは LED1〜7(U6 のはんだ不良)なので，TEST の 9〜11 番は LED が光って見えないかもしれない．
     1番から**逆に回す**と，1つ戻して 11 STRAIGHT_SWEEP，2つ戻して 10 SLALOM_SWEEP，3つ戻して 9 LONG_LOG になる(UART がつながっていれば番号と名前も出る)．
3. 走りの設定の値(速さなど): **1番(一番遅い値)から始まり，n 番なら LED1〜n を点ける棒グラフ**．値は遅い順．
   8 番目以降は LED8 より上を使うので見えないかもしれない．そのときも逆に回すと最後の値に行ける(1つ戻すと一番速い値)．

**各モードで選ぶもの**(この順に出る．最後に手かざしで始める)

| モード | 選ぶもの(値の並び．n 番目) |
|---|---|
| SEARCH | SPEED: 300 / 400 / 500 / 600 / 800 / 1000 / 1200 / 1500 → SLALOM: 300 / 400 / 500 / 600 → 曲がり方(クリックで PIVOT / SMALL．LED1 / LED2) |
| FAST_RUN | SPEED: 600 / 800 / 1000 / 1200 / 1400 / 1500 → ACCEL: 2000 / 3000 / 4000 / 5000 / 6000 / 8000 / 10000 → SMALL TURN: 300 / 400 / 500 / 600 → 走り方(クリックで PIVOT / SMALL / LARGE．LED1 / 2 / 3) |
| FAST_SWEEP | SPEED FROM → SPEED TO(FAST_RUN と同じ 6 つ)→ ACCEL FROM → ACCEL TO(7 つ)→ SMALL FROM → SMALL TO(300 / 400 / 500 / 600)→ TYPE(1 SMALL / 2 LARGE / 3 両方)．小回りの速さ → 加速度 → 直進の速さ → 走り方 の順に入れ子で回す |
| STRAIGHT_SWEEP | SPEED FROM → SPEED TO(300 / 400 / 500 / 600 / 800 / 1000 / 1200 / 1400 / 1500)→ ACCEL FROM → ACCEL TO(2000〜10000 の 7 つ) |
| SLALOM_SWEEP | FROM → TO(300 / 400 / 500 / 600 / 700 / 800 / 900) |
| LONG_LOG | PART(1〜6)→ STEP(段の中の番号) |
| STRAIGHT(1回ずつ) | SPEED(STRAIGHT_SWEEP と同じ)→ ACCEL |
| SLALOM(1回ずつ) | SPEED(s90)(SLALOM_SWEEP と同じ)→ 旋回(クリックで s90r / s90l / l90r / l90l / l180r / l180l) |

明日の順番で使う選び方の例:
- 1: RUN(1)→ SEARCH(1)→ SPEED 2番(400)→ SLALOM 1番(300)→ SMALL(クリック不要．既定)→ 手かざし
- 2: TEST(2)→ STRAIGHT_SWEEP(11．1つ戻す)→ SPEED FROM 4番(600)→ SPEED TO 9番(1500．1つ戻す)→ ACCEL FROM 1番 → ACCEL TO 1番(2000)→ 手かざし
- 3: TEST(2)→ STRAIGHT_SWEEP(11)→ SPEED FROM 6番(1000)→ SPEED TO 6番(1000)→ ACCEL FROM 2番(3000)→ ACCEL TO 7番(10000)→ 手かざし
- 4: TEST(2)→ SLALOM_SWEEP(10．2つ戻す)→ FROM 1番(300)→ TO 5番(700)→ A に北向きで置いて手かざし
- 5(例: 直進 1000〜1200，加速度 3000〜4000，小回り 500，小回りだけ): RUN(1)→ FAST_SWEEP(4)→ SPEED FROM 3番(1000)→ SPEED TO 4番(1200)
  → ACCEL FROM 2番(3000)→ ACCEL TO 3番(4000)→ SMALL FROM 3番(500)→ SMALL TO 3番(500)→ TYPE 1番(SMALL)→ スタートに北向きで置いて手かざし
  (この例は 2 × 2 = 4 本．本数は UART に出る．範囲を広げると本数がかけ算で増えるので，電池の持ちに注意)

- **最短走行の連続のモード(RUN → FAST_SWEEP)を作った**(ユーザーの依頼「いくつかのパラメータを試す際は，一度の手かざしでその行動パターンの
  ログを取り切れるようにしたい」．未確認: ビルド・実機)．連続の試験の3つ(LONG_LOG，SLALOM_SWEEP，STRAIGHT_SWEEP)はすでにその形だったので，
  そうなっていなかった最短走行に作った．
  - 直進の速さ・加速度・小回りの速さの範囲(FROM / TO)と走り方(1 SMALL / 2 LARGE / 3 両方)を選び，1回の手かざしで全部の組み合わせを
    `FAST_SWEEP_REPEAT`(1)回ずつ走る．1本ごとにゴールからスタートへ自分で戻る．置く所は RUN の4番(TEST の後ろだと LED が見えないため)．
  - 長い走行の「最短走行を走って戻る」部分を `FastRunAndReturn` に分けて，両方で使う．
  - どの値で走ったかが分かるよう，走り始めに `LOG_EV_FAST_PARAMS`(47: 直進の速さ・加速度・小回りの速さ・走り方)，
    `LOG_EV_SEARCH_PARAMS`(48: 探索の直進の速さ・スラロームの速さ)を入れるようにした(ふつうの探索・最短走行でも入る)．
  - 10-09 の予定の 5 番をこのモードを使う形に書き直し，モードの一覧と選び方の表・例に足した．

- **プラントモデルとシステム同定を学ぶ文書と演習を作った**(`docs/system_identification_study.md`，`tools/matlab/study/`．ユーザーの依頼．
  「同定はやりたいが知識が足りないので，今回のを元に勉強したい」)．DC モーターの式から FF の式を導き(FRIC = 摩擦トルク×R/Kt，
  GAIN = (n/r)(Ke + Rb/Kt)，ACC = (R/Kt)(J_m n/r + m r/2n))，最小二乗法・ARX・検証・IMC・同定可能性を，yuho のログの演習5つで説明した．
  - 演習のログ: 右の車軸を締め直した後の探索 search_0015〜0029(同定)，0030〜0031(確かめ)．前進の直線だけを使う．
  - **分かったこと(演習．提案だけで params.h は変えていない)**:
    (1) ログ全部で最小二乗をすると GAIN が負になった．尻当て(壁に押し付けて止まる)とスラローム(左右が機体の回転でつながる)は
    車輪のモデルに合わない．前進の直線だけなら FRIC 0.33/0.11V，GAIN 0.00047/0.00039，ACC 0.00023/0.00023(左/右)．
    (2) 左の電圧は左右の加速度にほぼ同じ係数(0.000131，0.000142)で効く(機体の重さを左右で分け合う)．並進(左右の平均)の
    モデルにすると R² 0.58 → 0.75，別のログでのシミュレーションの一致度 35〜45% → 67%(今の FF のモデルは 29%)．
    (3) 今の FF は 600mm/s の巡航で速さを低く計算する．GAIN を 200/300mm/s の2点で決めたので，速い所で大きすぎる可能性
    (速い巡航の I 項で確かめる)．(4) 今の PI(KP 0.015，KI 0.0376)は，モデルから見ると λ ≈ 18〜20ms の IMC とほぼ同じ
    (位相余裕 87°)．(5) スリップの C は誤差の地形が C の向きに平らで，今のログでは決まらない(K 0.0185〜0.030，C 0〜0.145s が最良 + 0.1mm 以内)．
  - 次にやるとよい実験(提案．ファームの変更が要る): 開ループの電圧ステップ，速さを変えた巡航の I 項，回転のモデル，SLALOM_SWEEP で C，PRBS．

- **PC 上のシミュレータ(plant_sim，仮の名前)の設計を始めた**(ユーザーの依頼「迷路とパラメータを決めて走らせ，実機と同じ状態を再現して，
  フル迷路がなくても探索や速度域を試せるようにしたい」)．ブランチ `feature/plant-sim`(元は `feature/slalom-speed-model`)．
  - 決めたこと: 壁センサまで再現する，`maze_sim` とは別の新しいツールにする，作り方は C のコア(実機のコードを PC でコンパイル)+ MATLAB．
  - 設計の記録は `docs/plant_sim_design.md` に分けた．「第1部 最新の設定(書き換えてよい)」と「第2部 記録(書き換えない．書き足すだけ)」の2部構成．

### 2026-10-09

- **plant_sim の寸法と答えをもらった**(`docs/plant_sim_design.md` 記録 3)．原点は4輪の接地点の真ん中．壁センサ4つの位置と向き，
  外形，迷路の寸法を `base_plate.DXF` と照らして 0.02mm 以内で合うことを確かめた．案4(車輪ごとのモデル)，案6(区画の真ん中で回るログ)，
  案10(CLI とファイル)が決まった．トレッドの設計値(54.91 か 57.90 か)など4つは確認待ち．

- **plant_sim の確認待ちの答えをもらい，設計の案を全部決めた**(`docs/plant_sim_design.md` 記録 4)．トレッドの設計値は 54.910mm
  (実機が設計どおりに組み立てられていないため．`params.h` のコメントと `plant_model.md` 2.4 の「設計 57.90mm」は直していない)．
  発光部は受光部と上下に重ねて付いている．案3 は「GUI で選び，中で app の関数を呼ぶ」．

- **plant_sim のセンサの高さ**(`docs/plant_sim_design.md` 記録 5)．床から基板の上面まで 3.7mm(設計)．受光部は床から 14.55mm，
  発光部は 7.85mm．これで設計の確認待ちはなくなった．

- **探索で，既知の区間をまとめて走るようにした**(直線の加速・大回り．ユーザーの依頼．未確認: ビルド・実機)．
  - 探索で「まっすぐ進む」と決まったとき，その先をプランナーの経路(Dijkstra の next_dir．今いるノードより先の経路はコストが小さいので，
    途中で計算を止めても決まっている)でたどり，壁が全部分かっている区画が続く間をまとめる(`TryKnownRun`)．
    最短走行と同じ指令の列(`RunPath_FromRoute`)にして `RunList_Drive` で走る．直線は探索の直進の速さまで加速し，「0 R 0」は大回りにする
    (大回りは探索の直進の速さが `FAST_LARGE*` の速さ以上のときだけ)．小回りの速さは探索のスラロームの速さ．
  - つなぎ方: 探索は境界，最短走行の部品は区画の真ん中が基準なので，1つ手前の区画の真ん中から始めたことにして指令の列を作る．
    終わりは，まだ分かっていない区画・目的地の入口の境界．最後の直進を半区画短くし，壁を読む所(12mm 手前)までにスラロームの速さにして
    (`RunList_Drive` に最後の速さと手前で待つのをやめる距離の引数を足した．最短走行は 0, 0)，そこで壁を読んで探索に戻る．
    大回りが最後の区画の真ん中で終わると境界に戻れないので，最後の2区画はまっすぐ進む形になるよう手前で切る．合わせて3区画以上のときだけまとめる．
  - 探索・帰り道の始め(`ComputeSlalomOffsets`)で，最短走行の部品の変数(直進の速さ・加速度・小回りの速さ・旋回の表)に探索の値を入れる．
    最短走行は走る前に自分の値を入れ直すので，影響しない．
  - `SEARCH_KNOWN_FAST_ENABLE`(0 で前と同じ)，`SEARCH_KNOWN_LARGE`，`SEARCH_KNOWN_MIN_MOVES`，`SEARCH_KNOWN_ACCEL_MM_S2` を足した．
    ログに `LOG_EV_KNOWN_RUN`(49: 区画の数・指令の数・大回りを使ったか・終わりの区画)が入る．
  - 10-09 の予定の 1 番に，この確認を足した．

- **plant_sim の最初の実装**(`tools/plant_sim`，`docs/plant_sim_design.md` 記録 6)．実機の `app/`(`test_mode.c` 以外)と `logic/` を
  そのまま PC の gcc でコンパイルし，`main.h` と interface 層だけを差し替えた．仮想の時計で 1ms ごとに `App_ControlTick` を回す．
  迷路なしの直進 540mm が走った(本当の機体の値をずらすと FB が働くことも確認)．壁センサ，迷路，ログのファイル，探索のシナリオはまだ．

- **plant_sim を worktree に分けた**(`docs/plant_sim_design.md` 記録 7．設計の記録は worktree の中)．シミュは
  `M:/User/Desktop/school/club/sim`(`feature/plant-sim`)，このフォルダは `feature/slalom-speed-model` に戻した(コミット前の変更はそのまま)．
  シミュは `fw_root.local` でこのフォルダのファームウェア(コミット前の変更も含む)をコンパイルする．シミュの日記もここに書く．

- **plant_sim: SD の代わりに PC のファイルへログを書く**(worktree の `docs/plant_sim_design.md` 記録 8)．`--sd-dir` のフォルダを SD とみなし，
  実機と同じ形式の `.bin` を書く(`logger.c`，`run_log.c` はそのまま)．`yuho_common.load_log` で読めた(34 列，409 行)．
  - **`logger.c` の流す方式に競合の候補を見つけた(未確認・直していない)**: 割り込みがブロックを 180 行で埋めた後，次に記録する(5ms 後)までに
    SD の書き込みが終わると，同じブロックに 181 行目を書き足し，ブロック(とバッファ)の外まで書く．シミュでは書き込みを 3ms 以下にすると起きた．
    実機の書き込みは平均 16〜30ms なのでふだんは起きないはずだが，5ms より速く終わることが一度でもあれば起きうる．
    ユーザーの決定: 実機で起きるまでは対策しない(記録 9)．

- **maze_sim の GUI に最新の探索を入れ，速さ・探索法・最短走行の走り方を選べるようにした**(ユーザーの依頼．ブランチ `feature/maze-sim-gui`．
  未確認: ファームウェアのビルド・実機)．
  - 既知の区間をまとめる計算を logic 層へ移し，ファームウェアとシミュで同じものを使うようにした:
    `SearchPlanner_KnownRun`(先読み)，`RunPath_FromKnownRun`(指令の列)，`RunProfile_ForSpeeds`(速さから旋回の表を作る．
    前は `search_run.c` の `ComputeFastTurns` の中にあった)．`search_run.c` の `TryKnownRun` はこれらを呼ぶだけになった．
    シミュのビルドに `logic/control/slalom.c` と `velocity_profile.c` を足した．
  - `sim_api.c` の `sim_step` を機体の `app/search_run` と同じ流れにした．ゴールで止まって 180° 回り，まっすぐ進むときは既知の区間を
    まとめて進む．探索の時間の目安(台形の加減速，スラロームの形，超信地旋回，尻当て，ゴールで止まる時間)も足していく．
    実機のログとはまだ合わせていない．
  - GUI(`gui.py`)の変更:
    - TAB で行，`,` / `.` で値を選ぶ．行は探索法，探索の直進，スラロームの速さ，曲がり方(スラローム/超信地旋回)，既知の区間の on/off，
      最短の直進・加速度・小回りの速さ．値の表は `params.h` の `SPEED_SELECT_*` を DLL から読む．
    - V の走り方は機体の LARGE / SMALL / PIVOT に，比べる相手の time-optimal / all walls known を加えた．PIVOT は時間だけ出す．
    - 既知の区間をまとめて走った所は，軌跡を緑で描く．
  - **時間の見積もりのずれを1つ直した**: `RunList_EstimateTime` は列の両端を速度0とするので，そのままだと既知の区間ありのほうが遅く出た
    (609 迷路の合計 73539s 対 72899s)．機体は走りながら入って抜けるので，シミュ側で両端の直進だけ見積もり直した(72745s 対 72899s)．
    既定の速さ(500/500)では，直線の最高速度が探索と同じなので，縮むのは 0.2% だけ．
  - 確認(609 迷路，Dijkstra と足立法，4通りの速さ): CRASH・LOST は0．既知の区間あり・なしで探索の手数は同じ(123030)．
    FAILED の3つは前からある，ゴールへ行けない迷路．
  - 残り: コマンドラインの `maze_sim.c` は前のままの流れ(既知の区間なし)．

- **10-09 の朝のログを確認した**(`logs/straight/sweep_0001`〜`0011`，`logs/slalom/s90r_0001`，`logs/search/search_0033`)．
  - 直進の連続(1080mm)は，600×3，800×2，1000×2，1200×3(加速度 2000)と 1000(加速度 3000)の 11 本だった．
    速さの行き過ぎは 2〜3%(1000 で 1029，1200 で 1215)で，誤差の最大は 21〜37mm/s．止まる位置の誤差は 1mm 以下，向きのずれは 0.5° 以下．
  - `sweep_0001` だけ，左の壁に寄った所から始まり(ad_l 513．ほかは 260 前後)，0.6s で左の壁に当たった．
    向きが −9° になったまま，PWM が左 −800・右 +1800 で押し付け続け，止まっても失敗の判定にならなかった(前からある，低速で壁に押し付けられたときの問題)．
  - `sweep_0009` と `sweep_0010` は中身(.bin)がまったく同じ．取り込みで同じファイルを2回写した可能性がある(未確認)．
  - スラローム `s90r_0001` は 300mm/s の右 1 回だけ．終わりの角度の誤差は 0.2°．
  - 探索 `search_0033`(Dijkstra，直進 500・スラローム 300)はゴール → スタートまで 42s で終わった．
    `EDGE_CORR` の補正の RMS は約 6mm(12 回)．`KNOWN_RUN` のイベントはなかった．
    既知の区間を使うビルドでなかったのか，3区画以上続く既知の区間がなかったのかは，このログからは分からない．

- **ログ取りのモードへのショートカットを足した**(ユーザーの依頼．未確認: ビルド・実機)．モードの一番上の階層(1 RUN，2 TEST，3 SD)に，
  4 LONG_LOG，5 SLALOM_SWEEP，6 STRAIGHT_SWEEP，7 FAST_SWEEP を足した．中のモードが1つだけの階層は選んだ時点で決まるので，
  1回の選択で始められる(TEST の 9〜11 番は LED が見えにくかった)．TEST / RUN の中にも残してある．`mode_ui.c` の表だけを変えた．
- **直進の連続の `sweep_0010` が `0009` と同じになった理由を調べた(推定．未確認)**．SD では別のファイルだった．
  - 0002〜0009 は1回の起動の続き(走り始めの角度が 180° ずつ増えている)．0011 と 0001 は別の起動．
  - 保存するのは，走った後(`StraightSweep_Run`)と，FailSafe が働いたとき(`DelayWatching` と `RunOnce` の中)だけ．`SweepHalt` は保存しない．
    SD の層にやり直しはない．中身(大きさも)が 0009 と同じなので，次の走行の記録を始める前に，前のバッファをもう一度保存したと考えられる．
    この経路は，180° 回って尻当てをしている間(`TurnAroundAndAlign` の `DelayWatching`)に FailSafe が働いたときしかない．
  - ユーザーが見たのは，シフトレジスタの LED が走った本数ぶん点滅する止まり方(`SweepHalt`)だった．
    FailSafe ならマイコン直結の LED が点滅するので，0010 を残した止まり方とは別の時(0011 の起動など)のことかもしれない．
    `SweepHalt` の原因は，電池(走る前の静止時で `LONG_LOG_MIN_VBAT_V` = 7.5V 未満)，回転・尻当ての失敗，SD の保存の失敗のどれか．
    ログの静止時の電池の電圧は 7.54V 前後で，閾値に近い．

- **10-09 の午後のログ取りの手順を組み直した**(ショートカットを使う形．迷路の形は「2026-10-09 の予定」の A・B・C のまま)．
  - 選び方: 一番上の階層で 4 LONG_LOG，5 SLALOM_SWEEP，6 STRAIGHT_SWEEP，7 FAST_SWEEP を選ぶと，すぐ値を選ぶ画面になる．
  - 朝に取れた分を除いた．直進は 600〜1200(2000mm/s²)が取れたので，残りは 1400・1500 と加速度を上げる分．
    探索の確認(予定の 1 番)は，最新でビルドしていなかった可能性があるので取り直す．
  - 連続の試験は，走る前の静止時の電池が 7.5V 未満だと止まる(朝は 7.54V で止まりかけていた)．試験ごとに満充電の電池にする．
  - 順番:
    1. A で探索の確認: 1 RUN → 1 SEARCH → SPEED 2番(400)→ SLALOM 1番(300)．続けて SPEED 5番(800)→ SLALOM 3番(500)．
    2. B で直進の速さ: 6 STRAIGHT_SWEEP → SPEED FROM 8番(1400)→ TO 9番(1500)→ ACCEL FROM 1番 → TO 1番(2000)．4本．
    3. B で直進の加速度: 6 STRAIGHT_SWEEP → SPEED FROM 6番 → TO 6番(1000)→ ACCEL FROM 2番(3000)→ TO 7番(10000)．12本．
    4. C でスラローム: 5 SLALOM_SWEEP → FROM 1番(300)→ TO 5番(700)．20本．様子を見て FROM 6番(800)→ TO 7番(900)．
    5. 解析(Claude)．
    6. A で最短走行: 1 RUN → 1 SEARCH で地図を作り，7 FAST_SWEEP で範囲を選ぶ．
    7. (時間があれば)A で 4 LONG_LOG → PART 2，STEP 1．

- **ショートカットで値を選ぶのも省けるようにした**(ユーザーの依頼．未確認: ビルド・実機)．一番上の階層のショートカットに値を持たせ，
  そのモードの `ModeUI_SelectValue` を選ぶ順に埋めて飛ばす(表にない値ならその所だけふつうに選ぶ)．値は `params.h` の `SHORTCUT_*`．
  - 4 STRAIGHT_SWEEP(v): 1400〜1500，2000mm/s²(4本)．5 STRAIGHT_SWEEP(acc): 1000，3000〜10000mm/s²(12本)．
  - 6 SLALOM_SWEEP(low): 300〜700(20本)．7 SLALOM_SWEEP(high): 800〜900(8本)．
  - 8 FAST_SWEEP: 範囲は解析の後で決めるので，値は選ぶ．9 LONG_LOG: PART 2，STEP 1．
  - 8・9 は LED が見えないことがあるので，1 から逆に回す(1つ戻すと 9，2つ戻すと 8)．手かざしで始めるのは今までどおり．
  - 午後の手順の 2〜4・7 は，それぞれ 4〜7・9 を選んで手かざしするだけになる．

- **plant_sim: 迷路，壁センサの仮のモデル，壁との接触**(worktree の `docs/plant_sim_design.md` 記録 10)．`--maze` で maze_sim の迷路を読み，
  受光部から光線を飛ばして壁までの距離と角度から AD 値を出す(仮: `logs/sensor` の止まった状態のログと `WALL_REF` に合わせた)．
  区画の真ん中で L 312 / R 280〜283，境界で前に壁があるとき FL 239 / FR 128 になった．機体の外形(DXF)と壁の重なりを調べ，
  後ろの面なら尻当て(壁にそろう)，それ以外は衝突として止める．案6 のログ(区画の真ん中で回る)が取れたらセンサのモデルを合わせ直す．

- **区画の真ん中で回りながら壁センサーを記録する試験 SENSOR_SPIN を作った**(plant_sim のセッション yuho-3a からの依頼を，ユーザーが
  「専用の試験モード」で承認．plant_sim の設計記録 2 の案6・記録 3・記録 10．未確認: ビルド・実機)．
  - TEST の 12 番と，一番上の階層の 10 番(1 から 6 つ戻す)．値を選ぶ所はない．`app/sensor_spin.c` を足した．
  - 手かざし → ジャイロのゼロ点を測り直し，向きを 0° にする → 0.5s 止まる → 左に 360° → 0.5s → 右に 360° → 0.5s．
    最高 45°/s，角加速度 360°/s²(`params.h` の `SENSOR_SPIN_*`)．回っている間も位置を保つ制御で真ん中に留まる．
  - 流す方式で SD の `sensor/spin_NNNN` へ 2ms ごとに書く(列: angle，angle_ref，gyro_z，ad_l/fl/fr/r，dist，vl，vr，pwm_l/r，vbat)．
    終わると次の手かざしを待つので，壁の組み合わせを変えて続けて取れる(組み合わせ1つにつき手かざし1回)．
  - 置く所は手で真ん中に合わせる(尻当ては，壁の組み合わせによっては後ろや横に壁がないので入れていない)．
    どの区画で，どの壁があったか，スタートの向きは別に書き留める．
  - 取る組み合わせの例: (a) 3方が壁(袋小路)，(b) 両横だけ，(c) 前だけ，(d) 壁なし(柱だけ)．

- **探索で初めて入った区画ごとに真ん中で回るモード SEARCH_SPIN を作った**(ユーザーの依頼．ブランチ `feature/search-sensor-spin`
  を `feature/maze-sim-gui` から切った(コミット前の変更も持ち越した)．未確認: ビルド・実機)．
  - ユーザーが選んだこと: 初めて入った区画すべてで回る．左に1周・右に1周．ふつうの探索(スラローム)に止まる所を足す．
  - RUN の 5 番．ふつうの探索(Dijkstra)と同じく SPEED・SLALOM・曲がり方を選んで手かざしで始める．
  - 探索の繰り返しで，壁を地図に書き込んだ直後(区画の4方向の壁が分かった時点)に，初めての区画なら真ん中で止まって
    左に 360°・右に 360° 回る(`SENSOR_SPIN_*` と同じ回り方．前・間・後に `SEARCH_SPIN_HOLD_MS` 止まる)．
    回った後は真ん中から次の動きを始める(ゴールで回った後と同じ流れ)．スタートとゴールの区画でも回る．
  - 区画・回る前の向き・その区画の壁(北東南西のビット)・段階を `LOG_EV_SENSOR_SPIN`(52)で残すので，壁の組み合わせを手で書き留めなくてよい．
    ログはふつうの探索と同じ列で，SD の `search/spin_NNNN`(5ms ごと)．
  - 既知の区間をまとめて走るのは使わない(区画を飛ばさないため)．
  - 1区画に約 18 秒かかる．A の試しの迷路(5×7)全部なら 10 分ほど，16×16 だと 1 時間近くかかりうるので，電池に注意．

### 2026-10-09 の午後のログ取り(まとめ直し)

朝に取れた分(直進 600〜1200mm/s・2000mm/s²，探索1回)を除き，ショートカット(値も決まっていて選ばない)と，
壁センサーのモデル用の2つのモード(SENSOR_SPIN，SEARCH_SPIN)を入れた形でまとめ直した．前の「2026-10-09 の予定」と
「午後のログ取りの手順」はこれで置き換える(迷路の A・B・C は同じ)．

**始める前に**
- ブランチ `feature/search-sensor-spin` の今の状態でビルドして書き込む(ショートカット，SENSOR_SPIN，SEARCH_SPIN，探索の既知の区間が入る)．
- 電池は試験ごとに満充電に近いものにする．連続の試験は，走る前の静止時の電池が 7.5V 未満だと止まる(朝は 7.54V で止まりかけていた)．
- SD カードの朝のログを PC に移して空ける．

**モードの選び方**(起動後．右のタイヤを回して選び，ボタンで決める．タイヤ 1/4 周で1つ進む．端では輪のようにつながる．
n 番は LED n と n+1 が点く．確実に光るのは LED1〜7 なので，8〜10 番は 1 から逆に回す)

| 一番上の番号 | 中身 | 決まっている値(選ばない) | 逆に回すと |
|---|---|---|---|
| 1 RUN | 1 SEARCH，2 SEARCH_ADACHI，3 FAST_RUN，4 FAST_SWEEP，5 SEARCH_SPIN | ―(中で選ぶ) | |
| 2 TEST | 1〜11 は前と同じ，12 SENSOR_SPIN | ― | |
| 3 SD | 1 SD_DUMP，2 SD_DUMP_ALL，3 STREAM_TEST | ― | |
| 4 | STRAIGHT_SWEEP(v) | 1400〜1500mm/s，2000mm/s²(4本) | |
| 5 | STRAIGHT_SWEEP(acc) | 1000mm/s，3000〜10000mm/s²(12本) | |
| 6 | SLALOM_SWEEP(low) | 300〜700mm/s(20本) | |
| 7 | SLALOM_SWEEP(high) | 800〜900mm/s(8本) | |
| 8 | FAST_SWEEP | なし(範囲を選ぶ) | 3つ戻す |
| 9 | LONG_LOG | PART 2，STEP 1 | 2つ戻す |
| 10 | SENSOR_SPIN | 選ぶものはない | 1つ戻す |

決まっている値は `params.h` の `SHORTCUT_*` で変えられる．UART をつないでいれば `(shortcut)` と値が出る．

**順番**

| # | 迷路 | モード(選び方) | 本数・時間の目安 | 分かること |
|---|---|---|---|---|
| 1 | A | 1 RUN → 1 SEARCH → SPEED 2番(400)→ SLALOM 1番(300)→ 手かざし．続けて SPEED 5番(800)→ SLALOM 3番(500) | 2回，5分 | 最新のコードで探索がふつうに走るか．帰り道に `KNOWN_RUN` が出るか(大回りは SPEED 600 以上のとき) |
| 2 | B | 4 → 手かざし | 4本，5分 | 1.4〜1.5m/s に制御がついてくるか |
| 3 | B | 5 → 手かざし | 12本，10分 | タイヤが滑り出す加速度，FailSafe で止まる所 |
| 4 | B の端の区画 | 10 SENSOR_SPIN → 真ん中に手で置いて手かざし | 1回，20秒 | (a) 3方が壁(袋小路)の壁センサー |
| 5 | C | 6 → A に北向きで置いて手かざし．様子を見て 7 | 20本(+8本)，15分 | K，C，横のずれ，横の限界 |
| 6 | C ほか | 10 SENSOR_SPIN → 真ん中に手で置いて手かざし(組み合わせ1つに1回) | 3回，5分 | (b) 両横だけ(C の (0,1))，(c) 前だけ，(d) 壁なし(柱だけ)．(c)(d) は壁を外して作る |
| ― | ― | ログを渡す(Claude が解析) | 15〜30分 | モデルの係数，最短走行に使う値 |
| 7 | A | 1 RUN → 5 SEARCH_SPIN → SPEED・SLALOM を選ぶ → スタートに北向きで置いて手かざし | 1回，10分ほど | 迷路の中の色々な壁の形で壁センサー(壁は地図から自動で記録)．解析の間に走らせられる．ゴールまで行けば地図が flash に残る |
| 8 | A | 8 FAST_SWEEP(3つ戻す)→ 解析で決めた範囲を選ぶ → スタートに北向きで置いて手かざし | 組み合わせの数 | 実際にどこまで速く走れるか．地図は 7(か 1)の探索のもの |
| 9 | A | (時間があれば)9 LONG_LOG(2つ戻す)→ 手かざし | 長い | 直進とスラロームの組み合わせ，最短の後の帰り道 |

この順にした理由: 1 で最新のコードが壊れていないかを最初に短く確かめる．2〜4 は同じ B の通路で続けて取れる(4 は通路の端の区画を使う)．
5・6 は同じ C の通路．7 の SEARCH_SPIN は 10 分ほど走りっぱなしなので，解析の間に走らせる．7 で作った地図をそのまま 8 で使える．

**SENSOR_SPIN / SEARCH_SPIN でやること**
- SENSOR_SPIN(4・6): 置いた区画で，左に 360° → 右に 360°(45°/s，1回 約 18 秒)．ログは SD の `sensor/spin_NNNN`(2ms ごと)．
  **組み合わせごとに，どの区画で，どの壁があったか(東西南北)と，スタートの向きを書き留める**(yuho-3a に渡す)．
- SEARCH_SPIN(7): 初めて入った区画ごとに真ん中で止まって同じように回る．ログは SD の `search/spin_NNNN`(5ms ごと)．
  壁は機体の地図から `LOG_EV_SENSOR_SPIN` に入るので書き留めなくてよい．**どの迷路で走ったか**(A なら「A」)だけ伝える．
  1区画に約 18 秒かかるので電池に注意．

**A: 試しの迷路**(5×7．search_0030 で機体が見た壁．`?` は見ていない壁．S がスタート(北向き)，G が試しのゴール (1,0)．
組み替えたときは写真かスケッチを残す)

```
+ ? +---+---+---+---+
|                   |  y=6
+ ? +   +---+---+   +
|   |   |   ?   |   |  y=5
+ ? +   + ? + ? +   +
|       |   ?   |   |  y=4
+---+   +---+---+   +
|               |   |  y=3
+   +   +---+---+   +
|   |   |           |  y=2
+   +   +   +---+---+
|   |   |   |   ?   ?  y=1
+   +---+   + ? + ? +
| S | G     |   ?   ?  y=0
+---+---+---+---+---+
  x=0   x=1   x=2   x=3   x=4
```

**B: 直線の通路**(STRAIGHT_SWEEP．7 区画．両側と両端に壁)

```
+---+---+---+---+---+---+---+
| > |   |   |   |   |   |   |    > : 端の区画に，通路の向きに置く
+---+---+---+---+---+---+---+
```
- 尻当てでそろえて 6 区画(1080mm)走り，反対の端で 180° 回って尻当てして戻る，をくり返す．
- 1080mm では 2000mm/s² で最高でも約 1470mm/s(加速してすぐ減速)．1.5m/s を保つ所まで見るなら
  `straight_test.c` の `STRAIGHT_TEST_SECTIONS` を 8 にして 9 区画の通路にする(ビルドし直しが要る)．
- 端の区画は3方が壁なので，SENSOR_SPIN の (a) にそのまま使える．

**C: L字の通路**(SLALOM_SWEEP)

```
        x=0   x=1   x=2
      +---+---+---+
 y=2  |         B |     B=(2,2): 東と南に壁(B で 180° 回って尻当てする)
      +   +---+---+     (1,2)(2,2): 北と南に壁(右の試験の出口の直線)
 y=1  |   |             (0,1): 西と東に壁(左の試験の出口の直線．SENSOR_SPIN の (b) に使える)
      +   +
 y=0  | A |             A=(0,0): 西・東・南に壁．北向きに置く
      +---+
```
- 右の試験: A から北へ → (0,2) で右に曲がる → B の真ん中に東向きで止まる．
- 左の試験: B で 180° 回って西向き → (0,2) で左に曲がる → A の真ん中に南向きで止まる → 180° 回る．
- 1つの速さで右と左を2往復(4本)．

**気をつけること**
- 3 では FailSafe で止まることがある．止まったら，何本目まで走ったか(LED)を覚えておいて，モードを選び直して続きを取る．
- 連続の試験がシフトレジスタの LED を走った本数ぶん点滅させて止まったら `SweepHalt`(電池・回転と尻当ての失敗・SD の保存の失敗)．
  マイコン直結の LED が点滅したら FailSafe(LED_5 だけ: 電池，LED_1・2: 速度の誤差，LED_1〜3: 角速度)．
- 8 の FAST_SWEEP は，範囲を広げると本数がかけ算で増えるので電池の持ちに注意．

### 2026-10-09(続き)

- **maze_sim に足した「走りのパラメータ」を外して，前の版に戻した**(ユーザーの依頼)．
  - なぜ: maze_sim はアルゴリズム(経路の選び方)の確認だけをする，という役割分担にしたいため．
  - 戻したもの(上の「maze_sim の GUI に最新の探索を入れ…」の GUI とシミュ側の部分): `gui.py`，`sim_api.c`，`README.md` はコミット済みの版(21ee3f9 の時点)のまま．
    `sim_lib.py` からは `sim_set_params` などパラメータの関数と `PLAN_SMALL` を外した．
    なくなったもの: TAB と `, / .` での値の選択，探索の時間の目安，既知の区間をまとめる流れと緑の軌跡，V の LARGE / SMALL / PIVOT．
  - 残したもの: logic 層へ移した `SearchPlanner_KnownRun`・`RunPath_FromKnownRun`・`RunProfile_ForSpeeds`(ファームウェアが使う)．
    `run_path.c` が `slalom.c` を呼ぶようになったので，シミュのビルド(`build.sh`，`sim_lib.py`)に `logic/control/slalom.c` と
    `velocity_profile.c` を足した所はそのまま．
  - 確かめたこと: `python sim_lib.py` で DLL を作り直し，乱数迷路で最後まで探索できた(`route cost: 86 best: 86`)．GUI は起動していない．

- **10-09 の午後のログを確認した**(`logs/search/search_0034`，`search_0035`，`fast_0024`．取れたのは順番の 1 だけ)．
  - `search_0034`(探索 400/400)は 18.6s で FailSafe の低電圧(cause=1，6.78V)で止まった．走る前は 7.57V．
    尻当てで後ろの壁に押し付けている間(目標 −100mm/s で車輪が止まる)，PWM が −1700〜−2800(最大 4199)まで上がり，
    電池が 6.8V 前後まで下がる．これが 0.5s 以上続いて低電圧と判定された．弱った電池では尻当てで止まりやすい．
    (尻当ての押し付けの強さを抑える案は報告だけ．直していない)
  - `search_0035`(電池を替えて 8.16V，探索 400/400)はゴール → スタートまで 60s で終わり，地図を flash に残した．
    帰り道で既知の区間を2回まとめて走った(`KNOWN_RUN`: 9区画で指令7つ，10区画で指令9つ．大回りは SPEED 400 なので使わない)．
    まとめて走った後も探索に戻れていて，既知の区間の動きは最初の実機の確認が取れた．
    `EDGE_CORR` は 14 回で RMS 9.3mm，最大 20mm(上限)．朝(500/300)の約 6mm より大きい．
    `FRONT_TRIG` のずれは +5〜+12mm が多い(前壁補正が距離で決めた位置より後で働く)．角度の誤差は最大 4.2°．
  - `fast_0024`(FAST_SWEEP．直進 1200，加速度 2000，小回り 300，SMALL の1本)はゴールまで走った．
    走った時間は 9.13s(見積もり 8.06s)．速さは最大 1089mm/s，角度の誤差は最大 3.8°，`EDGE_CORR` は最大 ±15mm．
    ログはゴールで回って尻当てした所で終わっている(スタートへ戻る所は入っていない)．

- **SEARCH_SPIN のログを確認した**(`logs/search/spin_0001`，`spin_0002`．探索 600/500)．
  - `spin_0001`: 17 区画で回った(スタート (0,0) から (2,2) まで)．1区画は約 18 秒(左 360° が 8.1 秒，右 360° が 8.1 秒，止まる時間)．
    回っている間の前後のずれは 1mm ほど，角度の誤差は最大 2°．壁の形は 3方・両横・L字・前だけ・壁なし((3,5))などが取れた．
  - **ログが 300 秒で切れた**: 流す方式のファイルは先に 8MB 確保する(`LOGGER_STREAM_RESERVE_BYTES`)．探索のログは 35 列 × 5ms なので
    約 28KB/s で，300 秒で使い切る．使い切ると `Logger_StreamFailed` になり，左後ろの LED を点けて走りは続ける(ログだけ止まる)．
    (2,2) の右回りの途中で切れていて，ゴール・帰り・地図(`MAP_CELLS`)は入っていない．
    ふつうの探索は 1 分ほどなので足りていたが，SEARCH_SPIN は 1区画 18 秒なので足りない．
  - `spin_0002`: 2 回目の走り．最初のブロック(約 360 行)の後は中身が壊れている(閉じる前に電源を切ったと思われる．未確認)．
  - 記録された壁は日記の A の迷路の図と合わない(例: (1,3) に東と南の壁)．迷路を組み替えた後だと思われる(未確認)．

- **流す方式のログのファイルを先に確保する大きさを 8MB から 32MB にした**(ユーザーが選んだ．`logger.h` の `LOGGER_STREAM_RESERVE_BYTES`．
  未確認: ビルド・実機)．探索のログ(35列・5ms)で約 20 分ぶん．全部の流す方式のログ(探索・最短走行・SEARCH_SPIN)に効く．
  - 気をつけること(未確認): 走る前の確保(`f_lseek` で伸ばす)に前より時間がかかるかもしれない．SD の空きが細切れだと，
    クラスタの対応表(`STREAM_CLMT_ITEMS` = 64，途切れ 31 か所まで)に入りきらず開けないことがある(そのときは UART に
    `stream link map failed` が出て，ログなしで走る)．SD を空けておくと起きにくい．
  - SEARCH_SPIN の迷路は新しく組んだもの(日記の A ではない)．ユーザーが log_viewer で見て，機体が認識した壁は正しかった．
    `spin_0002` は間違えて走り出させて電源を抜いたもの(使わない)．

- **plant_sim: 壁センサのモデルを実機の SEARCH_SPIN のログ(`logs/search/spin_0001.bin`，16 区画)に合わせた**(worktree の `docs/plant_sim_design.md` 記録 13)．
  仮のモデル(d^−2.9)は近い所で全く合わなかった(実機は 50〜65mm で頭打ち，当たる角度で倍ほど変わる)．
  AD = b + k exp(−d/λ) (cos φ)^m に替え，区画ごとの置いた位置と向きのずれと一緒に合わせた(λ 68〜98mm，m 1.6〜2.0)．
  前のセンサはよく合う(80〜250mm で誤差 14〜26)．横のセンサは真っ直ぐ走る所で誤差 48〜84 とまだ大きい．
  求まった置いた位置のずれが同じ向きにそろっていて(−10〜−15mm 西，+6〜+11mm 北)，モデルの足りない所を吸い込んでいる可能性がある．

- **plant_sim: 壁センサの合わせを MATLAB に移した**(worktree の `tools/plant_sim/analysis/fit_sensor_spin.m`，記録 14)．Python と同じ結果．Python のスクリプトは消した．
- **plant_sim: 探索のシナリオ**(記録 15)．ファームウェアの UI はそのまま動かし，シミュの中の「操作する人」が右の車輪を回す・ボタン・手かざしをする
  (`plant_sim run --ops "sel 0; sel 4; sel 3; sel 2; hand"`)．`main.c` と同じ順に起動してモードを選ぶ．
  実機の SEARCH_SPIN(spin_0001)の 17 区画の壁を入れた迷路で走らせると，**回った区画の順番が実機と 17 区画まで同じ**だった．
  シミュの 609 秒の探索が PC で約 8 秒．壁センサの値のずれは L 107，FL 77，FR 28，R 109(二乗平均)．
  回る前の横のセンサが実機と左右逆にずれている(実機は右寄り，シミュは左寄り)．まだ調べていない．

- **plant_sim: GUI(MATLAB)**(worktree の `tools/plant_sim/gui/plant_sim_gui.m`，記録 16)．迷路を開く・クリックで壁を置く・保存する，
  メニューとモード(ファームウェアの `mode_ui.c` から読む)と探索の速さ(`params.h` から読む)を選んで走らせる，
  機体の形・センサの光線・通った跡を再生する．`matlab -batch` で画像に書き出して，探索が最後まで走って表示されるのを確かめた．
  人の手での操作(クリック，▶)は未確認．

- **10-09 の夕方のログを確認した**(`straight/sweep_0012`〜`0016`，`slalom/s90r_0002`〜`0004`・`s90l_0001`〜`0002`，`sensor/spin_0001`．
  ユーザーによると連続の試験が途中で終わった)．
  - 直進(4: 1400〜1500)は 1400 が3本，1500 が1本．1500 は 1080mm では 1464mm/s まで(実測 1462)．速度の誤差は最大 24〜40mm/s，
    止まる位置の誤差 1mm 以下，向きのずれ 0.6° 以下．PWM は最大 1350(上限 4199 の 32%)で余裕がある．
  - 加速度(5: 1000mm/s，3000〜10000)は 3000 の1本だけ．問題なく走った(PWM 最大 1148)．
  - スラローム(6: 300〜700)は 300mm/s の5本(右3，左2)だけ．角度の誤差は終わりで 0.5° 以下．
  - **途中で終わった原因は電池と思われる(未確認)**．どの走りも走る前の電池が 7.54〜7.59V で，連続の試験が止まる
    `LONG_LOG_MIN_VBAT_V`(7.5V)のすぐ上．走るたびに少しずつ下がって 7.5V を切ると `SweepHalt("low battery")` で止まる．
    午前の満充電に近い電池(search_0035)は 8.16V だった．
  - SENSOR_SPIN(`sensor/spin_0001`)は最後まで取れた．17.8 秒，2ms ごと，0°→360°→0°，前後のずれ ±0.5mm，角度の誤差 最大 2.1°．
    始めの値は左右に壁があり前は開いていた(ad_l 295，ad_r 332，前は 1〜2)．

- **尻当てで電池の電圧が下がるのを抑えた**(ユーザーの依頼．ブランチ `fix/setpos-voltage-drop` を `feature/search-sensor-spin` から切った
  (コミット前の変更も持ち越した)．未確認: ビルド・実機)．
  - 原因: 尻当てで後ろの壁に押し当てている間(目標 −100mm/s)は車輪が止まるので，速度の偏差が残り続けて PI の積分が溜まり，
    PWM が −1700〜−2800(最大 4199)まで上がる．電池が 6.8V まで下がり，弱い電池では低電圧の FailSafe になった(search_0034)．
  - 対策: 押し当てている間(`s_wall_push`)だけ，車輪速度の PID の出力の上限を ±`SEARCH_SETPOS_PUSH_PID_MAX_V`(1.5V)にした
    (`control_loop.c`．FF は別)．`PID_Update` は上限に張り付くと積分を止めるので，溜まり続けない．
    下がる間は PWM 200 前後(約 0.4V)で足りているので，壁までは今までどおり下がるはず．
  - 確かめること: 尻当ての後に向きが壁にそろっているか(弱すぎるとそろわない．そのときは上限を上げる)．押し当てている間の PWM と電池の電圧．
- **400mm/s の探索で補正が大きかった件を調べた(コードは変えていない)**．
  - 壁の切れ目での補正(search_0035)は，壁のない区画 (3,5)(SEARCH_SPIN で柱だけと分かった区画)のまわりの 5 回が
    +15.4，−3.8，+18.2，−10.4，+20.0mm と大きく，ほかの 9 回は RMS 3.5mm(最大 5.6mm)だった．速さではなく，
    柱だけの区画のまわりの壁の切れ目が予想の位置で起きないのが原因と思われる(未確認)．
  - 前壁補正のずれ(400/400 で +5〜+12mm，朝の 500/300 で −8〜+4mm)は，走った迷路が違う(朝は A，午後は新しい迷路)ので，
    速さのせいか迷路(前の壁の位置)のせいかは分けられない．同じ迷路で 300 と 400 を比べると分かる．

- **10-09 の夜のログを確認した**(満充電の電池．SD の番号が 0001 からやり直しになったので，名前が重なったものは `_dup1` 付き)．
  - **加速度の連続**(`straight/sweep_0001_dup1`〜`0012_dup1`．1000mm/s，3000〜10000mm/s² を2本ずつ．全部取れた)．
    - 加速する間は 10000 でも速度の誤差 77mm/s 以下，PWM 最大 3065(73%)．
    - 減速する間の誤差が大きくなる: 3000 で ±22，5000 で −56/+38，6000 で ±60，8000 で −96/+117，10000 で −124/+191mm/s．
      左右の車輪の速さの差も 8000 以上で最大 116〜140mm/s．減速でタイヤが滑っている可能性がある(未確認)．向きのずれは 10000 で最大 2.5°．
  - **スラロームの連続**(`slalom/s90r_*`・`s90l_*`．300〜900mm/s を左右2本ずつ，28 本．全部取れた)．
    - 曲がり終わりの角度の誤差: 500 までは 0.3° 以下．700 右 −2.9°，800 −1.2〜−1.4°，900 左 −5.3°
      (角速度の最大が 792°/s で目標 810°/s に届いていない)．
    - 曲がった後の外への横のずれ(横の壁センサー − 車輪とジャイロの軌道．傾き 7.4AD/mm で換算)．
      右: 4，3，2，4，10，19，25mm．左: 8，16，14，16，22，22，26mm(300〜900mm/s)．左がいつも 10mm ほど外へ多い．700 から大きく増える．
    - K と C を `turn_sim.py` と同じ計算で合わせた: 全部で K = 0.042，C = 0.42s，b = +4.1mm(右へ)，残りのずれ RMS 6.0mm．
      速さごとに合わせると K = 0.03〜0.07，C = 0.25〜0.7s とばらつき，よく決まらない．1次遅れのモデルだけでは合わない可能性がある．まだ params.h には入れない．
  - `slalom/turn_0001`〜`0004`(500mm/s の小回り，21:17 に取り込み)は前の形式のスラロームの試験のログ(いつのものかは未確認)．

- **最短走行の FAST_SWEEP の範囲を決めた**(ユーザーが今のビルドで取る)．8 FAST_SWEEP で SPEED 1200〜1400，ACCEL 5000〜6000，
  SMALL 500〜600，TYPE 3(両方)．16 本．
- **最短走行の直進の減速度を加速度と分けた**(ユーザーの依頼．ブランチ `feature/fast-decel` を `fix/setpos-voltage-drop` から切った
  (コミット前の変更も持ち越した)．未確認: ビルド・実機)．
  - 理由: 加速度の連続のログで，加速は 10000 でも大丈夫だったが，減速は 8000 以上で誤差が大きかった(滑っている可能性)．
  - `VelocityProfile_StartAD`(減速度を別に渡す)を足した．`VelocityProfile_Start` は減速度 = 加速度でこれを呼ぶ(今までと同じ)．
  - `App_StartStraightAD` を足した．`App_StartStraight` は減速度 = 加速度．`LOG_EV_STRAIGHT` の e に減速度を入れる．
  - `RunProfile` に `decel` を足した．`RunProfile_Default` は加速度と同じ，`RunProfile_ForSpeeds` は加速度と `FAST_DECEL_MAX_MM_S2`(5000)の
    小さい方．`RunProfile_StraightTime` は加速と減速を別の値で計算する(経路の時間の計算・加速度が足りるかの判定に効く)．
  - 最短走行の直進(`FastStraightTo`，探索の既知の区間も同じ)は，加速度は選んだ値，減速度は `FAST_DECEL_MAX_MM_S2` まで．
    `LOG_EV_FAST_PARAMS` の e に減速度を入れる．
  - PC で確かめた: 1080mm・1000mm/s で，加速度 8000・減速度 5000 のプロファイルの時間 1.243 秒と見積もり(`RunProfile_StraightTime`)が一致．
    減速度 = 加速度(3000)のときは前と同じ 1.413 秒．
  - これで FAST_SWEEP の ACCEL に 8000・10000 を選んでも，減速は 5000 になる．

- **10-09 の夜の探索と最短走行のログを確認した**(`feature/fast-decel` のビルド．別の小さい迷路．`search/search_0001_dup1`，
  `fast_0001_dup1`，`fast_0002_dup1`，`back_0001`)．
  - **尻当ての対策は効いた**: 押し当てている間の PWM は最大 1057(前は 2800 前後)，電池は 7.52V までしか下がらない(前は 6.8V)．
    尻当ての後の走り出しの向きのずれも小さかった(0.1〜0.4°)．
  - 探索(1000/600)は 28.8 秒で終わった．既知の区間を2回まとめて走り，**初めて大回りを使った**(`KNOWN_RUN large=1`，RUN_CMD に LARGE90)．
    `FRONT_TRIG` のずれは +3〜+15mm(600mm/s)．400mm/s(+5〜+12)，300mm/s(−8〜+4)と合わせると，速いほど前壁補正が後で働く傾向がある
    (センサーの遅れの可能性．未確認)．
  - `fast_0001`(FAST_SWEEP の1本目: 1200，加速度 4000，小回り 600，SMALL)は 2.82 秒(見積もり 2.66 秒)．
    **2つ目の小回りで向きの誤差が 16°**: 直前の直進で壁の切れ目の補正が +12.4mm と大きく，曲がり始めが遅れて前の壁に近づき
    (FL 135→303)，曲がっている途中で角速度が目標に届かず(−306°/s，目標 −540)，その後行き過ぎた(−720°/s)．前の壁か柱に当たった可能性．
    1本目の直進の補正も −15.2mm と大きかった．最短走行の小回りには前壁補正がないので，切れ目の補正の誤りがそのまま曲がる位置に出る．
  - `back_0001`(1本目の後のスタートへの帰り道．kind=2)は 1.8 秒の後が壊れている(閉じる前に電源が切れた)．
  - `fast_0002`(FAST_RUN: 1500，加速度 10000，減速度 5000，LARGE)は 2.79 秒(見積もり 2.66 秒)．
    減速度が 5000 になっていることはログで確かめた．**3つ目の大回りで壁か柱に当たった**: 7.11 秒で速さが 297mm/s に落ち，
    右の PWM が上限(4199)に張り付き，向きの誤差が −13〜−21°．その後は右の壁に寄ったまま(AD_R 1400)ゴールで止まった．
    1つ目の大回りの前の切れ目の補正も +19.2mm と大きかった．加速度 10000 の加速では，速度が最大 180mm/s 遅れた．

- **速度帯の最短走行のモード FAST_BANDS を作った**(ユーザーの依頼「色んな速度帯で走る最短モード」．行きと帰りを自動でする，
  ログ取りと本番の両方に使う．速度帯は直進・加減速・小回り・走り方．ブランチ `feature/fast-bands` を `feature/fast-decel` から切った
  (コミット前の変更も持ち越した)．未確認: ビルド・実機)．
  - RUN の 6 番．BAND FROM と BAND TO を選び，1回の手かざしで，遅い速度帯から順に「最短走行 → ゴールからスタートへ自分で戻る」を
    `FAST_BAND_REPEAT`(1)回ずつ走る．本番は FROM と TO に同じ速度帯を選ぶ．止まり方・LED・電池の閾値は FAST_SWEEP と同じ．
  - 速度帯の表は `params.h` の `FAST_BANDS`(遅い順)．今の値(今日のログから):
    1: 800・3000・3000・小回り 400・SMALL，2: 1000・4000・4000・500・SMALL，3: 1200・5000・5000・600・SMALL，
    4: 1200・5000・5000・600・LARGE，5: 1400・6000・5000・600・LARGE，6: 1500・8000・5000・600・LARGE．大回りの速さは FAST_LARGE* のまま．
  - 減速度も速度帯ごとに持つ(`s_fast_decel`．0 なら加速度と `FAST_DECEL_MAX_MM_S2` の小さい方)．経路の時間の計算(RunProfile の decel)も合わせる．
    帰り道の既知の区間では自動(0)に戻す．
  - 走り始めに `LOG_EV_FAST_BAND`(53: 速度帯，何本目，全部で何本)を入れる．

- **尻当てで押す力が足りなかった**(ユーザー)ので，押し当てている間の PID の出力の上限 `SEARCH_SETPOS_PUSH_PID_MAX_V` を
  1.5V から 3.0V に上げた(未確認: ビルド・実機)．PWM は 1.5V で約 1050，対策の前は約 2800．3.0V で約 1800 の見込み(電池 7.8V で計算)．
  確かめること: 壁にそろうか，押し当てている間の電池の電圧(前は弱い電池で 6.8V まで下がった)．

- **FAST_BANDS の最初のログを確認した**(`search/search_0002_dup1`，`fast_0003_dup1`〜`0006_dup1`，`back_0002`〜`0004`．帯 1〜4)．
  - 押し当てている間の PWM は最大 1070 前後で，上限 1.5V のビルドだった(3.0V に上げる前)．
  - 探索(600/500)は 45.6 秒で終わり，既知の区間を3回まとめて走った(3回とも大回りを使った)．
  - FAST_BANDS は仕様どおりに動いた: 帯ごとに行き → 帰り(帰り道は既知の区間 12 区画をまとめて走る)をくり返し，`FAST_BAND` のイベントも入った．
  - 帯ごとの結果(走った時間 / 見積もり，角度の誤差の最大，壁の切れ目の補正):
    - 帯1(800・3000・小回り 400・SMALL): 4.78 / 4.39 秒，3.7°，補正 −19.0〜+19.8mm．
    - 帯2(1000・4000・500・SMALL): 3.88 / 3.59 秒，**28.5°**．6.88 秒の右の小回りで角速度が −69°/s(目標 −450)に落ち，右の PWM が −2417 → 壁か柱に当たった．
      その前の小回りで左のセンサーが 993〜1866(左の壁にかなり近い)．
    - 帯3(1200・5000・600・SMALL): 3.29 / 3.06 秒，**19.2°**．
    - 帯4(1200・5000・600・LARGE): **6.5 秒で FailSafe(低電圧，6.85V)**．5.90 秒の左の大回り 90° の途中で何かに当たり
      (左の車輪の速さが 189 に落ちた)，その後 0.6 秒ほど両方の PWM が 2500〜4199 になって電池が 6.6〜6.9V に下がった．
      続く大回り 180° で向きの誤差が +75° まで広がった．ぶつかって動けなくなった電流で低電圧になったと思われる．
  - まとめ: 帯2 以上(小回り 500 以上・直進 1000 以上)で曲がる所の接触が続いている．壁の切れ目の補正が ±15〜20mm と大きく外れること，
    小回りの後の横のずれ(左で外へ 14〜16mm)が重なっている可能性がある(未確認)．

### 次にログを取るときの予定(2026-10-09 の終わりに書いた)

**今わかっていること(10-09 のログから．推定を含む)**
- スラロームの小回りは同じ形(半径 約 64mm)で速さだけ変えるので，横の加速度は速さの2乗で増える
  (300: 0.14G，500: 0.40G，600: 0.58G，700: 0.78G，800: 1.0G，900: 1.3G)．
- 曲がった後の外へのずれ(右/左): 300: 4/8，400: 3/16，500: 2/14，600: 4/16，700: 10/22，800: 19/22，900: 25/26mm．
  600 まではほぼ一定で，700 から急に増える(タイヤの横の踏ん張りが効かなくなり始めた可能性)．左はいつも 10mm ほど多い．
  今のスリップのモデル(dβ/dt = (K·v·ω − β)/C)では，速さごとに合わせると K = 0.03〜0.07，C = 0.25〜0.7 秒とばらつき，1組に決まらない．
  「急に増える分」と「左右の違い」を表せていない．
- 最短走行の曲がり始めは壁の切れ目の補正で決まるが，新しい迷路では補正が ±15〜20mm 外れることがある．
  センサーや切れ目の見つけ方の遅れなら，速いほどずれが大きくなる(10ms で 1200mm/s なら 12mm)．探索の前壁補正も速いほど遅れて働く
  (300: −2mm 前後，400: +6，600: +10 前後)．どちらも未確認．
- FAST_BANDS は帯1(800・小回り 400)では当たらず，帯2(1000・500)以上で曲がる所で当たった．
  曲がり始めのずれと，曲がった後のずれが重なったと考えている．

**始める前に**
- ブランチ `feature/fast-bands` の今の状態でビルドして書き込む(尻当ての上限 3.0V が入る．10-09 の最後のログは 1.5V のビルドだった)．
- 電池は満充電のもの．SD を空ける．
- 走る迷路は 10-09 の夜と同じもの(組み替えたら写真かスケッチを残す)．

**順番**

| # | 選び方 | 時間の目安 | 分かること |
|---|---|---|---|
| 1 | 1 RUN → 1 SEARCH → SPEED 1番(300)→ SLALOM 1番(300) | 1分 | 地図を作る．切れ目の補正と前壁補正の 300mm/s での値 |
| 2 | 1 RUN → 1 SEARCH → SPEED 4番(600)→ SLALOM 4番(600) | 1分 | 同じ迷路の 600mm/s での値．1 と比べて，ずれが速さで変わるか(遅れか)を見る |
| 3 | 1 RUN → 6 FAST_BANDS → FROM 1 → TO 1 を2回 | 2分 | 帯1 を2回．切れ目の補正が外れる場所が毎回同じか(同じなら迷路が原因，違えば機体の遅れなど) |
| 4 | 1 RUN → 6 FAST_BANDS → FROM 2 → TO 2 | 1分 | 帯2 でもう一度当たるか，当たる場所 |
| 5 | 6 SLALOM_SWEEP(low．300〜700)→ C の迷路の A に北向き | 10分 | 小回りのずれを「左右の違い」と「速さで急に増える分」に分けられるか |
| 6 | (時間があれば)10 SENSOR_SPIN → (c) 前だけ，(d) 壁なし，(a) 3方 | 1分ずつ | 壁センサーのモデル(yuho-3a に渡す)．区画・壁・向きを書き留める |

- 1・2 は同じ迷路で速さだけ変える．3 は同じ帯を2回走る(どちらも比べるため)．
- 尻当ての後に壁にそろっているか(上限 3.0V)も，1〜4 のログで見る．
- 当たったら，どの帯の何本目か(行きか帰りか)を覚えておく．

- **FAST_RUN と FAST_SWEEP の加速度・減速度を，選んだ最高速度から決めるようにした**(ユーザーの依頼．選ぶ手間を減らすため．
  FAST_BANDS は帯ごとに書いた値のまま，LONG_LOG も今のまま．未確認: ビルド・実機)．
  - `params.h` に `FAST_ACCEL_FOR_SPEED_MM_S2` と `FAST_DECEL_FOR_SPEED_MM_S2` を足した(`SPEED_SELECT_FAST_V_MM_S` と同じ並び)．
    600: 3000/3000，800: 3000/3000，1000: 4000/4000，1200: 5000/5000，1400: 6000/5000，1500: 8000/5000．
  - FAST_RUN は SPEED → SMALL TURN → 走り方 になった(ACCEL を選ばない)．FAST_SWEEP は SPEED FROM/TO → SMALL FROM/TO → TYPE になった
    (ACCEL FROM/TO を選ばない．本数は 速さ × 小回り × 走り方)．
  - `search_run.c` の `SetFastAccelForSpeed`．表にない速さは，それ以下で一番近い速さの値．使わなくなった `kFastAccels` は消した．

- **モードの選び方の一覧を `docs/mode_select.md` に書いた**(ユーザーの依頼)．操作，一番上の階層(ショートカット 4〜10)，RUN・TEST・SD の
  モードごとの選ぶもの・ログの場所，選べる値の表，最短走行の加減速度の表，FAST_BANDS の速度帯，止まったときの LED．
  コードを変えたら合わせて直す．

- **RUN の中を SEARCH / FAST / TEST に組み替え，ゴールを (7,7)〜(8,8) にした**(ユーザーの依頼．ブランチ `feature/run-menu` を
  `feature/fast-bands` から切った(コミット前の変更も持ち越した)．未確認: ビルド・実機)．
  - RUN: 1 SEARCH，2 FAST，3 TEST．TEST はログ取りの走行(1 FAST_SWEEP，2 SEARCH_SPIN，3 FAST_BANDS，4 LONG_LOG)を値と同じ画面で選ぶ
    (`mode_ui.c` の `RunTest_Run`．`MODE_RUN_TEST` を足した)．一番上の TEST(試験)とショートカットはそのまま．SEARCH_ADACHI はメニューから外した
    (アルゴリズムは SEARCH の中で選ぶ)．
  - SEARCH(`SearchMenu_Run`): MAP(1 初期化，2 flash の地図に重ねる)→ SCOPE(1 往復，2 片道，3 全面)→ ALGO(1 Dijkstra，2 足立法)→
    SPEED → ACCEL → SLALOM → TURN(1 スラローム，2 超信地旋回)．片道はゴールの真ん中で止まって終わる(地図は flash に残す)．
    全面探索はまだ作っていない(`RunSearch` に枠: 選ぶと走らずに止まり，地図は書かない)．探索の加速度 `s_search_accel` を足した
    (今までは `SEARCH_ACCEL_MM_S2` 固定)．ログに `SEARCH_PARAMS` の加速度・曲がり方と，`LOG_EV_SEARCH_MODE`(54: 地図・行き先・アルゴリズム)を入れる．
  - FAST(`FastRun_Run`): SPEED → ACCEL(最初に SPEED に合う値を出す．`ModeUI_SelectValueFrom` を足した)→ SMALL TURN →
    TURN(1 SMALL，2 LARGE，3 PIVOT)．減速度は加速度と 5000 の小さい方．走り方をクリックで切り替えるのはやめた．
  - ゴール: `SEARCH_USE_TEST_GOAL` を 0 にして `MAZE_GOALS`((7,7)，(8,7)，(7,8)，(8,8))を使う．
  - `docs/mode_select.md` の RUN の所を直した．

- **plant_sim: 新しいメニュー(RUN を SEARCH / FAST / TEST に組み替え)に合わせた**(worktree の `docs/plant_sim_design.md` 記録 17)．
  GUI は値の並びを決め打ちせず，シミュの「下見」(`plant_sim run --probe`)でファームウェアが次に何を聞くか(名前，値の一覧，始めの番号)を聞いて出す．
  `mode_ui.c` だけを関数の名前を変えてコンパイルし，シミュが値の聞き方を横取りする(ファームウェアのソースは変えない)．
  ファームウェアでメニューや値を変えても GUI を直さなくてよい．新しいファームウェアでの探索がシミュで (0,3) の西の壁に衝突した(未調査)．

- **plant_sim: フラッシュをファイルで真似た**(記録 18)．`--flash` のファイルに探索の地図を残し，別の実行で最短走行が走れる(GUI は `build/gui_run/flash.bin`)．
  迷路 MM2020CM で探索(72 秒)→ 最短走行(ゴール (8,7) まで)を確かめた．地図がないときは GUI にファームウェアの理由(no map in flash)を出す．

- **左に寄る原因を調べた(2026-10-10．推定)**．スラロームのモデルは月曜に直すことにして，今は 600mm/s 以下で今のモデルを使う(ユーザー)．
  - 尻当ての後の向き(ジャイロで見た，迷路の軸からのずれ): search_0033(押す力の上限なし．PWM 最大 2541)は −0.5〜+1.2°，
    search_0035(上限なし．2871)は −3.3〜+0.1°(1回 +8.7°)．上限 1.5V の search_0001_dup1(1055)は +0.5〜+5.5°，
    search_0002_dup1(1069)は −0.8〜+11.4°．押す力が弱いと壁にそろわず，左回りに曲がった向きを「まっすぐ」の基準にしてしまう．
  - search_0002_dup1 の両側に壁がある直進では，左のセンサーの中央値 432(真ん中なら 312)，右 240(282)．壁の制御の向きの補正は
    中央値 −3.5°，上限の −5° に張り付いていた(右へ戻そうとしても戻しきれない)．上限なしの search_0033・0035 は左 301〜303，右 281〜286．
  - 昨夜の「尻当てのトルクが足りない」と「左に寄る」は同じ原因と考えている．上限は 3.0V に上げてある(未確認)．
    次のログで，尻当ての後の向きのずれが ±1〜2° に戻るかを見る．
  - スラロームの連続で左に曲がると 10mm ほど多く外へずれる件は別の話(尻当ての影響がない試験)．横の壁センサーの真ん中の値
    (`WALL_REF_L`/`R`)のずれ(約 4mm)の可能性．月曜にモデルを直すときに調べる．

- **スラロームのずれ(左に曲がると 10mm ほど多く外へずれる)の原因を調べた(2026-10-10．推定)**．スラロームの連続の 28 本を，
  曲がる前の直線・曲がった後の直線・車輪とジャイロの軌道に分けて見た(壁の制御は試験の中で切ってある．傾き 7.4AD/mm．9.0 でもほぼ同じ)．
  - 曲がった後の直線で，横の壁センサーで見た横の位置(左が +)を，右に曲がった回と左に曲がった回で平均すると，曲がり方によらない
    一定のずれが出る: 300: −1.5，400〜700: −5.7〜−6.0，800: −3.4，900: −4.2mm．つまり**どちら向きに曲がっても，センサーは機体が
    右へ 5〜6mm ずれていると読む**．曲がる前の直線でも同じ向き(−3〜−10mm)．
  - 「外へのずれ」は右に曲がると左が外，左に曲がると右が外なので，この一定のずれが，右では小さく・左では大きく見える(差は 2 倍の約 11mm)．
    **左右の違いはスラロームのせいではなく，この一定のずれで説明できる**．
  - 一定のずれを除いた，曲がったことによる外へのずれ: 300: +3.1，400: +4.3，500: +1.3，600: −1.1，700: +0.9，800: −8.3，900: −18.2mm．
    **700mm/s までは今のモデルでほぼ合っている**(±4mm)．800 以上は内側へ寄る(モデルが滑りを大きく見積もりすぎて早く曲がり始める．
    900 の左は角速度が目標に届かず曲がり足りない)．
  - 一定のずれ(約 5mm)が，機体が本当に右にいるのか，横の壁センサーの真ん中の値(`WALL_REF_L` 312 / `WALL_REF_R` 282)が
    この迷路の真ん中とずれているのかは，ログからは分けられない．後者なら，壁の制御は REF に合わせるので，探索では機体が本当の真ん中より
    左に寄る．
  - 確かめ方: 機体を定規で通路の真ん中に置いて，TEST の SENSOR(UART)で L と R を読み，REF と比べる(スタート区画と C の通路の両方)．
    約 5mm なら AD で 40 ほどの差になるはず．

- **一定の約 5mm が，センサーの REF のずれか，機体の本当の位置かを，ログで確かめた(2026-10-10．推定)**．
  - 方法: 両側に壁がある所で真ん中で回るとき，同じセンサーで北向き(西の壁)と南向き(東の壁)を比べる(REF にも左右のセンサーの違いにもよらない)．
    横のセンサーは少し前を向いていて，区画の真ん中では前の区画の横の壁を見ている(SEARCH_SPIN のログで分かった)．迷路の壁は格子の線の上にあるので，
    両方の向きで壁が見えていれば比べられる．
  - SEARCH_SPIN(`search/spin_0001`．壁の制御で REF に合わせて走った後に真ん中で止まる．17 区画): 本当の横の位置は平均 0.0mm
    (区画ごとに −6.5〜+4.2mm)．同じ区画で REF から計算した位置は平均 −2.1mm(左が +)．**REF のずれは約 2mm(傾き 9.0 なら 1.7mm)で小さい**．
    区画ごとのばらつき(±5mm)は，迷路の組み立ての誤差(壁の位置)も含む．
  - スラロームの連続(C の通路)は，回った前と後が別のログで，終わりは尻当てをしていない(向きがそろっていない)ので，同じ方法ではきれいに出なかった
    (左のセンサーと右のセンサーで答えが食い違う)．尻当ての後の始めの値は毎回そろっていて，REF で見ると約 6〜7mm 右．
  - 今の結論: REF のずれは約 2mm で小さい．スラロームの試験の約 5〜6mm は，残りの 3〜4mm が試験の中の本当の横の位置(壁の制御を切っていて，
    手で置いた位置や回った後の位置が直らない)の可能性が高い(未確認)．どちらにしても，曲がったことによるずれは 700mm/s まで ±4mm で小さい．
    探索で左に寄った主な原因は尻当ての向き(前の記録)で，REF の分は約 2mm．

- **スラロームの滑りを，今のモデルの見積もりと比べた(2026-10-10．スラロームの連続のログ．推定)**．
  - 測った滑り(横の壁センサー − 車輪とジャイロの軌道．右と左で平均して一定のずれを消したもの)と，モデルの見積もり
    (前後のオフセットを決めるのに使った分 = 車輪とジャイロの軌道が内側へ寄る量)．
    300: 6.1 / 3.1，400: 9.3 / 5.0，500: 8.4 / 7.0，600: 10.1 / 11.2，700: 16.2 / 15.3，800: 20.6 / 28.9，900: 25.7 / 46.6mm．
  - 測った滑りは速さにほぼ比例して増える(100mm/s あたり約 3mm)．モデルは K·v·ω で ω も速さに比例するので，速さの 2 乗で増える．
    500〜700 は合っているが，300〜400 はモデルが 3〜4mm 小さく，800 以上は大きすぎる(早く曲がり始めて内側へ寄る原因)．
    **今のログで，モデルの形(速さへの効き方)が合っていないことまでは分かる**．
  - 900mm/s では，外の車輪の FF が約 5V(電池 7.5V の 2/3)で，角速度が目標に届かない(左 792°/s / 目標 810)．電圧が足りなくなり始めている可能性．
  - 700mm/s: 曲がったことによるずれは ±4mm(1 速さにつき右 2 本・左 2 本)．ただし右の 1 本で曲がり終わりの角度の誤差が −2.9°．

- **スラロームの小回りの速さに 700mm/s を足した**(ユーザーの依頼．`SPEED_SELECT_SEARCH_TURN_V_MM_S` と `SPEED_SELECT_FAST_SMALL_V_MM_S`．
  未確認: ビルド・実機)．LONG_LOG の探索の段も 700 の分だけ本数が増える．`params.h` の加減速度の表の説明コメントを今の動きに直した．
- **今のログで大回りを調整できるかを調べた(2026-10-10)**．大回りの調整値 `FAST_LARGE90/180_PRE/POST_ADJ_MM` は 0 のまま．
  - 大回りが入っているログ: fast_0015・0020・0022(10-08，A の迷路)，fast_0002_dup1・0006_dup1，search_0001_dup1・0002_dup1(既知の区間)．
    合わせて L90R 14，L90L 3，L180R 3，L180L 2 回．左の大回りと 180° が少ない．
  - 曲がり終わりの角度の誤差は，当たった回を除いて ±0.5° 前後(1回 −1.3°，−1.7°)．
  - 曲がった後 100mm の横の位置(REF で見た，左が +)は，同じ迷路・同じ経路の fast_0015 と 0020 でよく一致した(±1mm)が，
    曲がりごとに +6〜+7，−9〜−10，−8，+6mm と向きも大きさも違う．曲がる前の位置(壁の切れ目の補正など)のずれが入っていて，
    大回りそのもののずれと分けられない．
  - 大回りの後は直線が短く，前後の位置を見る壁の切れ目の補正がほとんど入らない．
  - 結論: 今のログでは大回りの調整値は決められない．曲がる前の位置が分かっている状態(尻当ての後)から大回りだけを走る試験
    (TEST の SLALOM で l90r / l90l / l180r / l180l)を左右・90°/180° それぞれ数本ずつ取るのが一番確実．
  - ついでに見えたこと: A の迷路の同じ直進で，左右の壁の切れ目の補正が同じ所で逆向きに出た(−4.7mm と +5.6mm，−6.6mm と +7.5mm)．
    左右の切れ目の位置(`WALL_EDGE_POS_L/R_MM`)の差が実際と約 10mm 違うか，向きがずれている可能性(未確認)．
