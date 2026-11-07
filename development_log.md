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
- **斜め走行**: 機体の最短走行(FAST の 4 DIAG)と試験(RUN の TEST の 6 DIAG)まで作った(2026-11-07)．実機ではまだ走らせていない．
  斜めの旋回の形(`FAST_D*`)は仮の値．全面探索の斜めのコストの経路の計算時間(plan_ms)を確かめる．理想の速さ(`RUN_V_D*`)も見直しが要る．
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

### 2026-10-10 の試走会の予定(大会は 10-11(日)．迷路を使えるのは 10-10 だけ)

ゴールは (7,7)〜(8,8) の4区画(ユーザー確認)．自動で帰ってくる走りは許される見込み(参加者しだい)．持ち時間は参加者しだい．
今日は「日曜に確実に走れる設定を決める」ことを一番にする．一機能だけの試験はしない(探索と最短走行のログで見る)．

**出かける前に(家で)**
- `feature/run-menu` をビルドして書き込み，メニューを一通りたどる(RUN → SEARCH / FAST / TEST の選ぶ画面，値が選べるか)．
- 電池を全部満充電にする．SD を空にする．CubeIDE の入った PC と書き込み器を持っていく．走るたびのメモ用の紙．

**順番(持ち時間が短くても上から)**

| 優先 | 選び方 | 目的 |
|---|---|---|
| 1 | RUN → 1 SEARCH: MAP 1 → SCOPE 1(往復)→ ALGO 1 → SPEED 600(4番)→ ACCEL 2000(1番)→ SLALOM 500(3番)→ TURN 1 | 地図を作る．日曜の探索の安全な設定の確認．尻当て(3.0V)の確認 |
| 2 | RUN → 3 TEST → 3 FAST_BANDS: FROM 1 → TO 3 | 小回り 400〜600 の最短走行3本(自動で帰る)．日曜に使える最速の帯を決める |
| 3 | FAST_BANDS: FROM 4 → TO 5(2 で帯3 まで走れたら) | 大回り(1200・1400)．大回りは調整していないので，当たったら日曜は SMALL |
| 4 | RUN → 1 SEARCH: MAP 2(重ねる)→ SCOPE 1 → ALGO 1 → SPEED 800(5番)→ ACCEL 3000(2番)→ SLALOM 700(5番)→ TURN 1 | 速い探索(スラローム 700) |
| 5 | RUN → 2 FAST: SPEED 1200(4番)→ ACCEL そのまま → SMALL 700(5番)→ TURN 1 | 最短走行の小回り 700 |

- 1 と 2 が取れれば日曜の設定(探索 600/500，2 で当たらなかった一番速い帯)を決められる．
- 自動で帰ってくる走りが許されなければ，2 の代わりに RUN → 2 FAST で1本ずつ(SPEED 800 / SMALL 400 から)．
- 毎回，走る前の電池が 8V 以上．メモ: 何回目・モード・設定・結果(ゴールまで行けたか，当たった場所)・電池．

### 2026-10-10 全面探索(`feature/full-search`，`feature/run-menu` から)

RUN → 1 SEARCH の SCOPE 3(全面)を作った(これまでは選ぶと走らずに止まる枠だけだった)．
「最短になりうる所は全部見る，なりえない所は見ない」を，次の考え方で決めた．

- 未知の壁は「ない」とみなした地図(探索の地図)で，スタートからゴールまでの最短経路を求める．コストは最短走行
  (`PrepareFastRoute`)と同じ Dijkstra の既定のコスト(直進 1，90° 7，180° 50)．未知の壁を「ない」とみなしているので，
  このコストは本当の最短のコストの下限．
- その経路が横切る壁のうち，まだ分かっていない壁の両側の区画(見ていない区画)を行き先にして，一番近いものへ向かう．
- 経路が横切る壁が全部分かったら，その経路は実際に通れて，コストが下限と同じなので本当の最短．そこでスタートへ戻る
  (ゴールでは止まらない)．同じコストの別の経路は見に行かない(最短走行は速くならない)．
- 壁を見ても探索の地図は「ない」→「ある」にしか変わらないので，経路の計算し直しは，経路が横切る壁が見つかったときだけ．

作り(logic 層 `search_planner`): 段階 `SEARCH_PHASE_FULL`(番号 4．ログの LOG_EV_PHASE にも出る)を足し，
`SearchPlanner_StartFull()` で始める．FULL → TO_START → DONE と進む．既知の区間をまとめて走る(`SearchPlanner_KnownRun`)も，
行き先の区画の手前で止まるようにした．app 層は `RunSearch` で全面のときに `SearchPlanner_StartFull` を呼ぶだけ．
地図を flash に残す条件は今までどおり(段階が TO_START か DONE = 最短経路が決まった後)．

PC での確認(scratchpad の試験用プログラムで logic 層だけをコンパイル．ファームのビルドではない):

| 迷路 | 数 | 全面探索の後の最短コスト = 本当の最短 | 往復探索が本当の最短を見つけられなかった | 動いた区画の数(往復 → 全面) |
|---|---|---|---|---|
| tools/maze_sim/mazes | 520 | 520 | 272 | 平均 205 → 293 |
| tools/maze_sim/my_mazes(16×16) | 86 | 86 | 60 | 平均 192 → 312 |

未確認: 実機での1区画ごとの計算の時間(経路を計算し直す境界では Dijkstra が2回になる．ログの plan_ms で見る)．
maze_sim(CLI・GUI)にはまだ全面探索を選ぶ所がない．

### 2026-10-10 全面探索を plant_sim で走らせた

- GUI で「全面」を選んでも `FULL SEARCH: not implemented yet` で止まった → plant_sim.exe が全面探索を入れる前のビルドだった
  (GUI の「ビルド」を押さないと、ファームウェアの変更は入らない)．ビルドし直すと全面探索で走る．
- 迷路は build/gui_run/maze.txt，SEARCH: MAP 1，SCOPE 3，ALGO 1，SPEED 800，ACCEL 2000，SLALOM 500，TURN 1．
  全面探索は 342 区画で DONE まで行き，地図を flash に残した(227 秒)．ただし前の壁に2回当たった
  (行き止まり (10,6) で真ん中に止まるときと，最後にスタート区画で止まるとき)．同じ設定の往復探索は当たらずに 131 秒で終わる．
- 当たった2回は，どちらもスラロームが続いた後に真ん中で止まる所で，止まる位置が真ん中より約 +22mm 先(壁に当たって止まった値)．
  止まった所ごとの真ん中からのずれは，往復探索でも −13〜+19mm ばらつく(スラロームの後)．全面探索は止まる回数と、その前の
  スラロームの数が多いので，ずれが当たる大きさになる所が出た．全面探索の判断ではなく，スラロームが続いた後の前後の位置のずれ
  (真ん中で止まるのは距離だけで決めている)が原因と見ている(実機で同じずれがあるかは未確認)．

- **10-10 の試走会(本番の迷路)のログを確認した**(`logs/search/search_0003_dup1`，`fast_0008_dup1`〜`0010_dup1`，`back_0006`〜`0008`)．
  `fast_0007_dup1`・`back_0005` は 10-09 の夜のもの(押し当ての PWM 約 1000 = 上限 1.5V のビルド)を取り込み直したもの．
  `search_0004_dup1`・`0005_dup1` は先頭(512 バイト)だけで中身がない．
  - **尻当ては直った**: 押し当てている間の PWM は 1874〜1897(上限 3.0V)．尻当ての後の向きのずれは −0.5〜+3.7°(前の 1.5V では 5〜11°)．
    両側に壁がある直進の左右のセンサーの中央値も，探索で 316 / 296(REF 312 / 282)と真ん中に近い．
  - 探索(600/500，往復，Dijkstra)は 29.2 秒で終わった(ゴール 15.7 秒)．帰り道で既知の区間 8 区画を大回り付きでまとめて走った．
  - FAST_BANDS 帯1〜3(SMALL)は3本とも最後まで走って帰ってきた．経路のコスト 36，指令 8 個．
    帯1(800): 4.35 秒(見積もり 4.10)，帯2(1000): 3.54 秒(3.34)，帯3(1200・小回り 600): **3.00 秒**(2.84)．角度の誤差の最大 3.3 / 4.9 / 5.7°．
  - 壁の切れ目の補正は，同じ所で毎回ほぼ同じ値(場所で決まる)．最初の直進の補正は速いほど大きい(800: +4.7，1000: +7.5，1200: +13.6mm)．
    切れ目を見つけるのが速さに比例して遅れる(400mm/s で約 9mm → 約 22ms 相当)可能性がある(未確認)．

- **10-10 の試走会の2回目のログを確認した**(`search_0006_dup1`〜`0008_dup1`，`fast_0011_dup1`・`0012_dup1`，`back_0009`・`0010`．
  経路のコストが 92 で，1回目(36)と違う迷路)．最後の走行で右のタイヤのねじがおかしくなっていた(ユーザー)．
  - `search_0006_dup1` は中身なし(先頭だけ)．
  - 探索(600/500，往復): 35.5 秒で最後まで走った(ゴール 17.0 秒)．尻当ての後の向きのずれ −1.9〜+0.4°．
  - 帯3(1200・小回り 600・SMALL): 3.79 秒(見積もり 3.43)，角度の誤差の最大 4.9°．帰り道も最後まで(14.2 秒でスタート)．
    **1回目の迷路と合わせて，帯3 は2回続けて走れた**．
  - 帯4(1200・LARGE): 走った時間 3.78 秒で帯3 と同じ(この経路では大回りに置き換えられる所がなく，指令の列が同じ)．
    ゴールで 180° 回るとき，両方の PWM が 2300〜4199 になっても回る速さが目標の半分(80〜90°/s，目標 180)にしかならず，
    向きの誤差が −29° まで広がった．その後の帰り道(`back_0010`)は，尻当ての後の向きが −36° / −33° ずれていて，
    7.6 秒の左の小回りで向きの誤差 −25°，7.9 秒で壁に当たって止まり(車輪の速さ 0，右の PWM 4199)，FailSafe(速度の誤差)．
  - 探索(800/600，`search_0008_dup1`): 8.3 秒の右の小回りの途中で右の壁に寄り(R 742〜823)，車輪が止まって FailSafe(速度の誤差)．
  - 帯4 のゴールで回れなかった所から後は，右のタイヤのねじの不具合が始まっていた可能性がある(未確認)．

- **全面探索のビルドエラーを直した**(`search_planner.c` で `NULL` を使っているのに `<stddef.h>` を読んでいなかった．PC の gcc ではほかのヘッダ経由で入っていて通っていた)．`#include <stddef.h>` を足した(未確認: ビルド)．

- **10-10 の別の迷路の試験(全面探索・大回り・帯4)のログを確認した**(`search_0009_dup1`〜`0012_dup1`，`fast_0013_dup1`)．
  5 本のうち 4 本が FailSafe で止まった．
  - search_0009(600/300，加速度 4000，往復): 30.3 秒で最後まで．尻当ての後の向きのずれが −1.1 → +6.9°，1回 +29.3° に広がった．
  - search_0010(**全面探索**，600/500): 19.0 秒で FailSafe(速度の誤差)．区画の真ん中で止まる直進の途中で前の壁に当たり(FL 954)，
    両方の PWM が 4199 のまま動けなくなった．全面探索の判断(区画ごとの計算 最大 6ms)には問題は見えないが，最後まで走っていないので確かめられていない．
  - search_0011・0012(往復): ゴールで 180° 回るときに FailSafe(低電圧)．ゴールの真ん中で止まるはずが前の壁に押し付けたまま(FL 770)，
    PWM 4199 で動けず電池が 6.3〜6.5V に下がった．
  - fast_0013(帯3): 5.9 秒で FailSafe(速度の誤差)．1200mm/s の直進の途中で前の左のセンサーが 1264〜1607 になり(前の壁か柱に近い)，当たった．
  - **共通していること: 前の壁に突っ込む(止まる位置・曲がる位置が前へずれる)．** 超信地旋回の PWM が，最初の2本(search_0003・0007)の
    550〜670 から，その後は 800〜1185 に上がり，車輪の速さも目標の 0.93 → 0.81〜0.89 倍に下がった．同じ回り方に前より力が要っている．
    右のタイヤのねじの不具合(ユーザー)が直りきっていないか，ほかにも機械の不具合がある可能性が高い(未確認)．

- **探索の既知の区間で大回りを使わないようにした**(ユーザーの依頼．`params.h` の `SEARCH_KNOWN_LARGE` を 0．未確認: ビルド・実機)．大回りは調整していないため．既知の区間をまとめて走る(直線の加速・小回り)はそのまま．

- **尻当てで右のタイヤが壁まで届かない**(ユーザー)ので，押し当てている間の PID の出力の上限を 3.0V から 4.0V に上げた(未確認: ビルド・実機)．
  - ログ(`search_0013_dup1`，`fast_0014_dup1`．上限 3.0V): 押し当てている間に左右とも約 40〜48mm 下がった所で車輪の回転が止まる
    (PWM 約 1700〜1950)．目で見て右が届いていないなら，右側を押し切る力が足りない．上限なしのときは PWM 2500〜2900 だった．
  - 新しいログの失敗: search_0013(600/300)は 44.4 秒，右の小回りの後に右の壁に当たって停止(R 1132〜1217，FailSafe 速度の誤差)．
    fast_0014(帯3)は fast_0013 と同じ所(5.6 秒，前の左のセンサー約 800)で当たって停止．
  - 右側の動きが重い(機械の不具合)可能性は残る(超信地旋回の PWM が朝の 1.3〜1.8 倍)．

- **尻当てで壁にそろわない原因を見つけて直した**(ユーザーの見立て「最初以外は垂直を担保できていない」．未確認: ビルド・実機)．
  - ログ(`search_0014_dup1`〜`0017_dup1`，`fast_0015_dup1`〜`0017_dup1`，`back_0011`・`0012`)で押し当てごとに見ると，尻当ての後の向きが
    少しずつずれ(+2 → +5〜+8°)，ときどき大きくずれた(search_0017 の9回目 +19.2°，back_0011 の1回目 +12.9°)．
  - 原因: 尻当ての直前の超信地旋回で溜まった車輪速度の PI の積分が，押し当てに持ち越されていた．search_0017 の9回目では，回ったときに
    前へ回っていた右の車輪の積分が +3.05V 残っていて，「下がれ(−100mm/s)」なのに右の PWM は +829(前向き)から始まり，積分が減るまでの
    約1秒，右はほとんど下がらなかった．その間に左だけ下がって機体が +11.5° 回り，押す時間(1.2 秒)の大半を使ってしまって右が壁に届かなかった．
    back_0011 の1回目も同じ(右の積分 +2.1V)．押し当ての上限(1.5 → 3.0 → 4.0V)を入れてから，この持ち越しを力で打ち消せなくなって目立った．
  - 直し方: `control_loop.c` で，押し当てを始めた瞬間(`s_wall_push` が立った最初の1回)に `VelocityPID_Reset` を呼ぶ．上限 4.0V はそのまま．
  - 確かめること: 往復探索を1回走らせて，尻当ての後に機体がまっすぐか(ログで尻当ての後の向きが ±1〜2° に収まるか)．

- **ゴールでは，壁が一つでもあれば尻当てをするようにした**(ユーザーの依頼．`search_run.c` の `TurnBackAtGoal`．未確認: ビルド・実機)．探索のゴール・スタート，最短走行のゴール(SMALL/LARGE と PIVOT)で使う．袋小路の折り返し(`TurnBack`)は今までどおり．
  - 前と右 / 前と左: 今までどおり 90° → 尻当てを2回．右だけ: 左90° → 尻当て → 左90°．左だけ: 右90° → 尻当て → 右90°．前だけ: 180° → 尻当て．壁がなければその場で 180°．

- **「ゴールの後に戻るとき半区画余分に進む」原因を調べ，前の壁のしきい値を 220 から 150 にした**(ユーザーが 1 を選んだ．
  `params.h` の `WALL_TH_FRONT_SUM`．未確認: ビルド・実機)．
  - back_0013(帯3 の後の帰り道): (8,6) に入る手前で前のセンサーの合計が 196 で，220 未満なので「前に壁なし」と判断．実際は壁がある
    (search_0018 で (8,5) の北の壁として見えていた)．帰り道は探索の地図を使うが，その場の読みで分かっていた壁を「なし」に上書きし，
    既知の区間をまとめて走る計画が (8,6) で曲がらず次の境界(半区画先)までまっすぐ進む形になって，前の壁に当たった(FL 990)．
  - 境界の手前で読んだ前の壁の値(10-08〜10-10 の探索・帰り道，1863 回): 「なし」と判断したもの 0〜99 が 1013 回，100〜139 が 3 回，
    160〜219 が 12 回．「あり」は 220 以上で 835 回．140〜159 は 0 回．160〜219 は本当は壁があるのに弱く見えたもの
    ((8,6) の南の壁は 160，163，196，196，211，218，238 とばらついた)．しきい値を 140〜159 の真ん中の 150 にした．
  - 選ばなかった案: 最短走行の後の帰り道で，分かっている壁をその場の読みで上書きしない(今後の候補)．

- **10-10 の夕方のログ(修正の確認と大回り)を確認した**(`search_0019_dup1`，`fast_0020_dup1`〜`0024_dup1`，`back_0014`〜`0018`)．
  - **尻当ては直った**: 尻当ての後の向きのずれが ±1〜2°(search_0019 は −1.0〜+1.1°)．積分を捨てる変更が効いた．
  - 探索(600/500，往復)は 38.2 秒で最後まで．前の壁の読み違いによる当たりはなかった．
  - **帯3(1200・小回り 600・SMALL)は3回とも 4.60 秒で走れて，帰り道も最後まで**(見積もり 4.17 秒)．角度の誤差の最大 4.2〜7.2°．
    back_0017・0018 は帰り道の最後(段階 DONE の後)に低電圧の FailSafe．電池が 7.1〜7.5V まで下がっていた．
  - 大回り: 同じ経路(最初の直進 3 区画の後に右の大回り 90°)で，直進の速さだけ違う3本を比べた．
    fast_0019(600): 補正 +0.1mm，大回りの後の外へのずれ 4.8mm．fast_0022(帯4，1200): 補正 +5.2mm，ずれ 11.0mm．
    fast_0023(帯5，1400): 補正 +20mm(上限)，ずれ 26.2mm，向きの誤差の最大 20.4°．
    **補正が増えた分とずれが増えた分がほぼ同じ**(+5.1 → +6.2，+19.9 → +21.4)．大回りに入る速さ(600 前後)と曲がり終わりの角度は合っている．
    大回りの形ではなく，その前の直進の壁の切れ目の補正が大きすぎて曲がり始めが遅れたのが原因と考えている．
  - 全部のログ(676 回)で補正と速さを比べると，1050mm/s までは速さによる差ははっきりしない(平均 +0.4〜+2.4mm)．
    1050〜1250 は平均 +4.2(中央値 +6.4)，1400 は +19〜+20．速い直進のブレーキ中に切れ目を読むと大きく出る可能性(機体が前に傾く等．未確認)．
  - この迷路では大回りで速くならなかった(帯3: 4.60 秒，帯4: 4.645 秒，帯5: 4.62 秒)．

- **「真ん中を走れていない，探索の帰りが櫛で揺れる」(ユーザー)を調べた(2026-10-10．推定)**．
  - 両側に壁がある直進の横の位置(センサーで見た)は，平均で ±3mm 以内(search_0019: 行き −1.2，帰り +0.2mm．back_0014・0015: −2.5〜−2.7mm)．
    壁の制御の補正も小さく(標準偏差 0.4〜0.6°)，上限に張り付いていない．平均すると真ん中は走れている．
  - 揺れの元はスラロームの曲がり終わり: 角速度の目標が 0 になった時点で，機体はまだ曲がる向きに 19〜48°/s で回っていて，
    向きが平均 1.5〜3.4°，最大 5.4° 行き過ぎ，0.3 秒ほどかけて戻る．曲がるたびに機体が少し蛇行する．
    search_0018(スラローム 600)は search_0019(500)より大きい(帰り: 行き過ぎの平均 3.4° と 1.5°)．
    櫛のように横の壁が途切れる所では，壁の制御で戻せないので目立つと思われる．
  - 直すなら角度の制御(曲がり終わりの減速の追従)の調整．大会前は試す時間がないので，探索のスラロームは 500 のまま(行き過ぎが小さい)．

- **スラロームの曲がり終わりの行き過ぎを減らすため，回転の加速度の FF を 1.5 倍にした**(ユーザーの依頼．`params.h` の
  `ANGULAR_FF_ACC_SCALE`，`control_loop.c`．未確認: ビルド・実機)．
  - search_0018 のスラローム(600mm/s，角速度 540°/s，角加速度 11520°/s²)を細かく見ると，回る速さを上げる間は実際が目標より
    40〜65°/s 遅れて向きが約 3° 遅れ，下げる間は 50〜120°/s 速いまま残って，最後に約 2.3° 行き過ぎていた．回る速さを変えるときに
    いつも遅れてついていっている．
  - 加速度の FF は車輪の加速度に同じ係数(`VELOCITY_FF_ACC_L/R`)を掛けていて，直進と回転を区別していなかった．回転は機体の慣性の分だけ
    多く要ると考え，回転の加速度の分だけ `ANGULAR_FF_ACC_SCALE` 倍にした(1.0 で前と同じ)．超信地旋回とスラロームの両方に効く．
  - 確かめ方: 曲がり終わりに残る回る速さ(前は 19〜48°/s)と，曲がった後の向きの行き過ぎ(前は平均 1.5〜3.4°)が減るか．
    悪くなったら(向きが逆に先へ進む，揺れる)1.0 に戻すか 1.2 くらいにする．

- **回転の加速度の FF 1.5 倍の確認のログを見た(`search_0020_dup1`〜`0025_dup1`，`fast_0025`〜`0028`)**．
  - **書き込んだビルドに変更が入っていなかったと思われる**: 回り始めに足される FF の増え分が，変える前(search_0018)の 1.56 / 1.64V と
    今回(search_0024)の 1.58 / 1.65V でほぼ同じ(1.5 倍なら 2.3〜2.5V になるはず)．
  - 遅れの正体: 回り始めに車輪は目標どおりに回っている(左右の車輪の差から計算した回る速さ 394°/s，目標 392°/s)のに，ジャイロは 333°/s．
    タイヤの横の滑り・たわみで機体の向きが遅れている．回っている途中(一定)では車輪から計算した値とジャイロの比は約 0.97．
    車輪の速度ループは車輪を目標に合わせるだけなので，FF だけでは機体の遅れは残る可能性がある．ジャイロを見る回る速さのループ
    (`ANGULAR_KP` 0.5)を強くするのが次の候補．
  - スラローム 600 での比較(変える前 → 今回): 曲がり終わりに残る回る速さ 42 → 30〜34°/s，向きの行き過ぎの平均 3.1 → 2.8〜3.4°(変わらず)．
  - search_0025(探索 800/700): 最後まで走ったが，残る回る速さ 88°/s，行き過ぎ平均 5.5°．search_0022 は 5 秒で電池の電圧 3.9V
    (電池が外れかけていた可能性)．
  - 次: 今のコードを確実にビルドして書き込み，往復探索を1回 → FF が効いているか確かめる．遅れが残れば `ANGULAR_KP` を 0.5 → 1.0．

- **回転の加速度の FF 1.5 倍が入ったビルドのログを確認した**(`search_0026_dup1`・`0027_dup1`，`fast_0029`)．**効いた**．
  - 回り始めに足される FF の増え分が 1.88 → 2.77V になった(1.5 倍が入った)．
  - スラローム 600 の探索で比べると(search_0018 → search_0026): 回る速さを上げる間のジャイロの遅れ 46 → 11°/s，下げる間 −74 → −8°/s，
    曲がり終わりに残る回る速さ 42 → −12°/s，曲がった後の向きの行き過ぎ 平均 3.1 → 2.0°(最大 5.4 → 3.8°)．
    車輪は目標より先に回るようになった(−85°/s)が，タイヤの滑り・たわみの分を先回りで補えている．
  - search_0027(探索 1000/600)は 86.6 秒で最後まで．1か所(77.5 秒，既知の区間の右の小回り)で左の壁に当たって向きが 34° ずれたが，その後戻った．
  - fast_0029(1500，加速度 2000，小回り 700，SMALL)は最後まで(7.64 秒，見積もり 7.26 秒)．ただ小回り 700 では
    回り始めの遅れ 54°/s，残る回る速さ 75°/s，行き過ぎ平均 3.9° と大きい(角加速度が大きく，滑りが増える)．

- **全面探索が実機で初めて最後まで動いた**(`search_0028_dup1`．探索 800/700)．17.7 秒で最短経路が決まって帰りに切り替わり，
  既知の区間 23 区画をまとめて走って 25.0 秒でスタート，地図を flash に残した(31.2 秒)．区画ごとの計算は最大 6ms(平均 2.2ms)．
  尻当ての後の向きのずれ 0.1〜2.2°．
- 小回り 700 の最短走行: fast_0031(1400・加速度 3000・小回り 700・LARGE)は最後まで(7.81 秒，見積もり 7.21 秒)，角度の誤差の最大 5.1°，
  L の最大 1317．小回り 700 を使った最短走行は 4 本中 4 本とも最後まで走れた(fast_0025，0028，0029，0031)．壁への近さは 1317〜2198 とばらつく．
  fast_0030(1500・PIVOT)は 9.9 秒，超信地旋回を始めた所でログが終わっている(FailSafe の記録なし．理由は未確認)．
- 日曜の案: 1 本目は帯3(SMALL 600)で確実に，2 本目以降に小回り 700 で攻める．同じ地図での帯3 と小回り 700 の時間の比較はまだ．

- fast_0030(1500・PIVOT)のログが途中で終わっていたのは，ユーザーが自分で止めたため．
- **全面探索をくり返したログを確認した**(`search_0029_dup1`〜`0032_dup1`)．
  - search_0029(片道，800/600): 15.7 秒でゴールに着いて止まり，地図を残した．**片道探索も実機で動いた**．
  - search_0030(全面，800/600)，0031(全面，600/700)，0032(全面，1000/700): **3 回とも最後まで走って地図を残した**．
    どれも 66 区画動いて，最短経路が決まったのが 29.1〜31.0 秒，スタートに戻ったのが 30.4〜32.5 秒，全体で 36.6〜38.7 秒．
    区画ごとの計算は最大 6ms．尻当ての後の向きのずれ −3.1〜+0.1°．
  - **3 回とも同じ所で柱か壁に当たっていた**: (1,4) で左の小回り → すぐ (1,3) で右の小回り，の連続する逆向きの小回りの2つ目の途中で，
    回る速さが急に落ちて(−530 → −105°/s など)向きが 11〜21° ずれ，左のセンサーが 900〜1259 まで上がった．その後は戻って走り続けた．
    1つ目の小回りの終わりで向きが目標より 3.5〜4.8° 遅れていて，それを持ったまま逆向きに曲がり始めている．スラローム 600 でも起きた(11°)．
  - スラロームの曲がった後の行き過ぎの平均: 600 で 2.8°，700 で 4.5°．

### 2026-10-10 ログをもとに maze_sim を改善した(`feature/maze-sim-log-model`，`feature/full-search` から)

ユーザーが選んだ3つ: 最短走行の見積もりを実測に合わせる，探索の時間を見積もる，全面探索を選べるようにする．

**1. 最短走行の見積もり(logic 層 `run_path.c`．ファームの見積もりの表示と LOG_EV_ROUTE も変わる．走りは変わらない)**
- 最短走行のログ(`fast_*` のうち最後まで走った 25 本)で，指令(RUN_CMD)ごとに実測と見積もりを比べた．ファームの前の見積もりは
  実測より **5〜12% 短かった**(例: 4.17 → 4.60 秒，7.21 → 7.81 秒)．
- 原因1(大きい方): 旋回の曲線の部分を「理想の円弧の長さ − オフセット」で数えていた．実機は角速度を台形で上げ下げするので，
  曲がる時間は 角度/ω + ω/α(小回り 700mm/s で 0.183 秒．前の見積もりは 0.162 秒)．旋回の区間の実測は，この式で数 ms 以内に合った
  (小回り 300〜700，大回り 90°・180°)．→ `RunProfile_ForSpeeds` で turn_len を「前オフセット + 曲がる道のり(SlalomShape.length_mm) + 後オフセット」にした．
- 原因2: 前後のオフセットの中でも加速するとして数えていた．実機(RunList_Drive)はオフセットを旋回の速さのまま走り，間の直進だけで加減速する
  (旋回の後の直進1本あたり約 10ms の差)．→ `RunProfile_LinkedStraightTime` の時間をそう数えるようにした．加速度が足りるかの判定
  (戻り値．大回りを小回りに戻す判断に使う)は前と同じにしたので，ファームが選ぶ指令の列は変わらない．
- 直した後: 25 本とも **0〜−1.5%**(平均約 −0.7%)．残りは主に，ログのイベントが 5ms の行に1つずつ並ぶための遅れ(旋回1つあたり約 2ms)．
- maze_sim は今まで `RunProfile_Default()`(params.h の RUN_* の仮の値)で見積もっていた → 機体と同じ速度帯(FAST_BANDS)の速さにした(`--band`，既定は帯3)．

**2. 探索の時間(`tools/maze_sim/search_time.c`．maze_sim だけ)**
- 機体の SearchLoop の動き(直進，スラローム，真ん中で止まって超信地旋回，袋小路の 180° と尻当て，ゴールの TurnBackAtGoal と待ち，
  既知の区間をまとめて走る，スタートの StartSequence)を params.h の値でたどって時間を足す．
- 探索のログを区画ごとに再生して比べた(STEP の指令と壁，KNOWN_RUN の指令の列を渡し，STEP ごとの時刻を比べる):
  今の設定のログ(search_0018_dup1〜0032_dup1，往復・全面・片道 15 本)で **−0.3〜+1.7%**．
  600〜700 のスラロームは前壁補正で曲がり始めが平均 10〜14mm 早まり，見積もりの方が1回あたり約 15ms 長い(補正の量はセンサーで決まるので入れない)．
  古いログ(0001〜0017)は尻当ての時間などが今と違うので +5〜+35%．search_0033〜0035(10-09)はスラロームかどうかの欄が 0 の古い形式．
- maze_sim は1区画ずつ進めるが，既知の区画では壁が変わらないので，まとめて走るときと通る道は同じ．時間だけ，機体と同じ先読み
  (SearchPlanner_KnownRun)でまとめて走ったとして数える．ゴールでは機体と同じく向きを逆にしてから帰る(前は向きを変えずに続けていた)．

**3. 全面探索を選べるようにした**: CLI `--scope round|oneway|full`，GUI は O キー(と `--scope`)．GUI の区画の数字は全面探索では行き先までの値．

**maze_sim で見えたこと(探索 600/500，最短走行は帯3)**

| 迷路 | 探索 | 探索の時間(ゴール / 最短経路が決まる) | 真の最短を見つけた | 最短走行(小回り，コストの経路) |
|---|---|---|---|---|
| my_mazes(86) | 往復 | 103.0 秒(59.9 秒) | 26 | 13.78 秒 |
| my_mazes(86) | 全面 | 144.6 秒(122.5 秒) | 86 | 13.33 秒 |
| mazes(520) | 往復 | 118.7 秒(69.2 秒) | 248 | 13.21 秒 |
| mazes(520) | 全面 | 150.2 秒(129.0 秒) | 520 | 12.85 秒 |

- 全面探索は大会の迷路で探索が約 40 秒長くなり，最短走行が平均 0.45 秒速くなる．
- コスト(1, 7, 50)が最短の経路が，帯3 の速さでは時間が最短でないことがある．例: MM2023CM で全面探索の経路(コスト 172)は 11.46 秒，
  往復探索の経路(コスト 200)は 10.82 秒．機体の最短走行はコストで経路を選ぶので，全面探索で「コストの最短」を確かめても速くなるとは限らない．
- `MM2025_east_SC` はスタートの先で行けなくなる特別な迷路で，前から FAILED(今回の変更とは関係ない)．
- PC での確認: CLI・GUI(画面なし)・全部の帯で「時間で選んだ経路の計算の時間と見積もり」が食い違わない，plant_sim のビルドが通る．ファームのビルドは未確認．

### 2026-10-10 maze_sim から機体と同じ形式のログを出す(plant_sim を呼ぶ)

ユーザーの依頼「機体と同じ形式のログを出したい．シミュレーション上の車輪の速さやジャイロも」．maze_sim に物理を作ると plant_sim と
同じものが2つになるので，plant_sim を呼ぶ形にした(ユーザーが選んだ A・B)．

- **A(plant_sim の不具合)**: `--sd-dir` にドライブ名から始まる絶対パス(`C:/...`，`M:\...`)を渡すと「SD カードなし」になり，
  ログが書かれていなかった(`sim_sd.c` の `EnsurePcDir` が `C:` そのものを mkdir して失敗を返していた)．直した(sim の worktree，
  `feature/plant-sim`．設計記録 19)．plant_sim の MATLAB GUI はフル パスを渡すので，GUI からの SD のログは今まで残っていなかったと思われる(GUI では未確認)．
- **B(maze_sim の `--plant-log DIR`)**: 同じ迷路・同じ探索の設定(`--search`，`--scope`，`--pivot`，`--algo`)で plant_sim を走らせ，
  `DIR/search/search_NNNN.bin` と同じ名前の `.csv`(`get_log.py --bin2csv`)，`truth.csv`，`flash.bin` を残す．走らせる前に plant_sim を
  ビルドし直す．plant_sim の場所は `tools/maze_sim/plant_sim.local`(git に入れない)などから探す．
- 確かめたこと(試走会の迷路，往復 600/500/2000): 約 21 秒で終わり，ログは 122.0 秒(手を離してからの待ち約 1 秒を含む)．
  maze_sim の探索の時間の見積もりは 122.5 秒．CSV の列の見出しは機体のログと同じ．選べない速さ(650 など)や迷路ファイルなしではエラーで止まる．
- **GUI からも plant_sim を呼べるようにした**(maze_sim の gui.py)．X キーで今の迷路(乱数の迷路も書き出して渡す)と今の探索の設定で
  plant_sim を走らせ，`tools/maze_sim/build/plant_logs/<迷路>_<行き先>_<日時>/` にログを残す．別のスレッドで走らせるので GUI は止まらない
  (探索の時間の見積もりは Dijkstra の作業領域を使うので GUI のスレッドで先に計算し，別のスレッドでは plant_sim を呼ぶだけ)．
  Z キーでできたログを log_viewer で開く．画面なしで試した: 乱数の迷路 7 で，plant_sim が走っている間に GUI の探索が 44 区画進み，
  ログは 35 列・33.4 秒(maze_sim の見積もり 33.4 秒)．Z キー(log_viewer の窓)は試していない．

### 2026-10-10 実機の全面探索と plant_sim を同じ迷路・設定で比べた(plant_sim の設計記録 20)

- search_0030〜0032_dup1(全面，800/600・600/700・1000/700)の地図から迷路ファイルを作り(3 本で読み違いなし)，maze_sim の
  `--plant-log` で同じ設定の plant_sim を走らせ，両方のログにある値で区画ごと・曲がりごとに比べた(比べるスクリプトは scratchpad)．
- **合っていた**: 区画ごとの判断(66 区画のうち最初の 63 が同じ)，時刻(差は最大 0.09〜0.21 秒)，全体の時間．
- **合っていなかった**: 曲がった後の向きのばらつき(実機 ±11〜21°，シミュ ±4°)，壁への近づき(横のセンサーの最大 実機 1000〜1350，シミュ約 600)，
  前壁補正のずれ(実機 −8〜−13mm，シミュ −1mm)，FR の値(シミュが約 40 小さい)，曲がり終わりに残る回る速さ(向きが逆)．
  実機が 3 回とも当たった (2,4)→(1,4)→(1,3) の連続する小回りは，シミュでは当たらない．
- 64 区画目 (0,2) 南向きで，実機は 3 回とも右(西の外周)の壁を「なし」と読んでいた(シミュは「あり」)．
- 結論: plant_sim は判断と時間は再現するが，物理のずれ(特に曲がった後の向きのばらつきと当たり)はまだ再現しない．シミュの上でずれを直す前に，
  回転の応答・壁センサ(FR，近い前の壁)・ばらつきのモデルを実機のログに合わせる必要がある．

### 2026-10-10 plant_sim の回転の応答・ばらつき・壁センサを実機に合わせた(sim の worktree `feature/plant-model-fit`．設計記録 21)

- **回転の応答**: 車輪の運動を並進と回転に分け，回転を2慣性系(車輪側と機体を，タイヤのたわみのばね・減衰でつなぐ)にした．ジャイロは機体の回転を測る．
  実機の全面探索(search_0030・0031)と同じ迷路・設定で plant_sim を閉ループのまま走らせ，曲がりの平均の応答と揺れが合うように合わせた
  (探索のログは 5ms の間引きで速度のループの動きを拾えないので，開ループの同定はできなかった)．
  回転の重さ ÷ 並進の重さ = 1.59(ファームの回転の加速度の FF 1.5 倍とほぼ同じ)，その 9 割は機体側，タイヤのたわみの固有角振動数 71 rad/s(約 11Hz)・
  減衰比 0.036，定常の滑り ω_b/ω_w = 0.98 − 0.18·v[m/s]．
- **ばらつき**: 機体の回り方に 11% の揺らぎ(時定数 55ms)を入れた．曲がりごとの向きのずれのばらつき・揺れの大きさが実機と同じくらいになった．
- **壁センサ**: **FR の見え方が 10-10 の 16:11〜16:46 の間に変わっていた**(前の壁に向いて止まったときの FR が約 300 → 700〜800，FR/FL 0.3 → 0.6〜0.8)．
  何かを触った可能性がある(未確認)．前の壁のしきい値(`WALL_TH_FRONT_SUM` 150)・前壁補正の閾値は，FR が弱かったころの値．
  実機で前壁補正が 8〜13mm 早くかかっているのはこのためと思われる．plant_sim は 16:46 以降の探索のログ 12 本で合わせ直した(FR の誤差 165 → 44)．
- **確かめ**: 前壁補正のずれ・止まったときの前のセンサー・FR・横のセンサーの最大・曲がり終わりの向きのばらつきは，実機の範囲に入った．
  連続する小回り (2,4)→(1,4)→(1,3) で実機が 3 回とも当たった所(向きのずれ ±20°)は，まだ再現しない(シミュ 6〜8°)．

### 2026-10-11 plant_sim に横のずれのばらつきと横・前の面の当たりを入れた(sim の worktree `feature/plant-model-fit`．設計記録 22)

- 実機が 3 回とも当たった (1,3) 南向きの右の小回りは，**前壁補正がそこだけ遅れていた**(実機 −6〜+16mm．ほかは −10〜−15mm)．遅く曲がり始めて左前の角が当たった．
  そこでは FL だけが小さく(機体が横にずれて FL の光線が前の壁の端の外へ抜けたと思われる)，その前の区画では実機が約 20mm 右に寄っていた．
- 曲がった後の横の位置は，実機とシミュで場所ごとに −23〜+21mm 違い，曲がる向きでは決まらない(理由は地図から分からない)．→ ばらつきとして入れた
  (スリップ角に 40% の揺らぎ)．横・前の面で当たったときは，当たった点が壁に食い込む速さを機体の回転への力積で止める(摩擦 0.3，跳ねない．どちらも仮の値)．
- 結果: 壁への近づき方(小回りの区画の横のセンサーの最大の 90% 点)と当たる回数は実機と同じくらいになり，当たったときに向きが 20° ずれる走りも出た．
  当たる場所は実機と同じにならない．
- 確認: step 63 (0,2) で外周の壁を「なし」と読んだのは，R 138〜149 が `WALL_TH_R` 150 のすぐ下だったため(壁がないのではない)．

### 2026-10-11 maze_sim の sim to real を確かめた(yuho-09 のセッションからの依頼)

- 各ログの地図から迷路を作り(見ていない壁は「あり」)，同じ設定で maze_sim を走らせて比べた(作業ツリーの feature/maze-sim-log-model)．
- 探索 9 本(全面 5，片道 2，往復 2): 動いた区画の数と経路は全部実機と同じ．全体の時間は 0〜+1.3%，ゴール(全面は最短経路が決まった時)は 0〜+0.8 秒．
  search_0026_dup1 は片道，0027_dup1 は全面だった(SEARCH_MODE)．
- 最短走行: 帯3 SMALL 実機 4.595〜4.600 秒に対し maze_sim 4.553 秒(−1.0%．ファームの前の見積もりは 4.17 秒)．帯4 LARGE 4.645 に対し 4.632 秒．
  経路のコスト 103 はファームと同じ．

- **自立賞のモード AUTO を作った**(ユーザーの依頼「自立賞を目指す」．ブランチ `feature/autonomous` を今の作業ツリー(`feature/maze-sim-log-model`．
  別のセッションの maze_sim の作業とファームの見積もりの変更がコミット前)から切った．未確認: ビルド・実機)．
  - RUN の 4 番．手かざし1回で，機体に触らずに「探索 → 最短走行とスタートへの帰り道 を `AUTONOMOUS_FAST_COUNT` 回」走る(`AutonomousRun_Run`)．
  - ユーザーが選んだこと: 探索の後は同じ速さの最短走行を数本．探索も最短走行も設定は決め打ち(params.h の `AUTONOMOUS_*`．走る前に選ぶものなし)．
  - 今の設定(10-10 の試走会で走れたもの): 探索 往復・Dijkstra・600/500・加速度 2000．最短 1200・加速度 5000・減速度 5000・小回り 600・SMALL を 3 本．
    最短走行の前の電池が 7.4V(`AUTONOMOUS_MIN_VBAT_V`)未満なら止まる．
  - 探索は往復か全面だけ(どちらもスタートに戻って北を向いて終わる)．片道は `_Static_assert` で使えないようにした．
  - 止まったとき(探索で止まった，電池，最短走行で止まった)はシフトレジスタの LED に何本目かを出す(FAST_SWEEP と同じ．探索なら 0)．
  - `docs/mode_select.md` に AUTO を足した．

- **AUTO の設定を変えた**(10-11，大会の当日．ユーザーの依頼．未確認: ビルド・実機)．
  - 最短は 1200・加速度 2000・減速度 2000・小回り 600・SMALL を 4 本(探索は往復 1 回のまま)．
  - 減速度はユーザーの依頼で試走会までのログから決めた: 最短走行のログの減速中の速度の誤差(平均の車輪の速さ − 目標)は
    減速度 2000 で −33/+27，−38/+52mm/s(fast_0024，fast_0029)，3000 で −30〜−51/+22〜+64，5000 で −38〜−93/+40〜+128mm/s．
    2000 は試走会で 1500mm/s まで走れていて(fast_0029)，誤差がいちばん小さいので 2000 にした．
  - 電池の電圧では止めない(`AUTONOMOUS_MIN_VBAT_V` を消した．止まった時点で終わりなので．フェイルセーフの低電圧は効く)．
  - 最後の最短走行の後は，帰り道の代わりにゴールから全面探索をしてスタートへ戻り，地図を flash に残す(`AUTONOMOUS_FINAL_FULL_SEARCH`．
    速さは探索と同じ 600/500．ログは search)．`ReturnToStart` に `full` を足して，プランナーを FULL の段階から始める．
    全面探索の行き先(最短経路の候補が横切る未知の壁の両側の区画)はスタートからの経路で決まるので，ゴールから始めてもよい．
    `FastRunAndReturn` に `full_back` を足した(ほかのモードは false で今までと同じ)．

- **コミット前の変更を読んで不具合を探した**(ユーザーの依頼「バグがないかデバックして」．コードを読んだだけ．ビルド・実機はしていない．ファイルは変えていない)．
  - 機体が止まる・ぶつかるような不具合は見つからなかった．AUTO の値の受け渡し(`ComputeSlalomOffsets` が `s_fast_*` を上書きしても，各回の頭で入れ直す)，
    ゴールから始める全面探索，`turn_len` の変更(時間の計算だけに使う)，maze_sim の `SimFastBand` と `plant_bridge.c` の操作の手順は合っていた．
  - 直したほうがよい点: `RunProfile_LinkedStraightTime` は，時間を「オフセットは旋回の速さ，加減速は直進 dist の中だけ」で数えるようにしたのに，
    戻り値(加速度が足りるか)はまだ dist + オフセット で決めている．戻り値は大回りを小回りへ下げる判定に使うので，LARGE の帯で旋回に速すぎる速さで入る
    おそれがある(今の AUTO は SMALL だけなので影響なし)．
  - 小さな点: AUTO で探索が止まると `FastSweepHalt(..., 0)` で LED が何も光らない．最後の全面探索のログの SEARCH_MODE が ROUND のまま．
    止まったときの UART が `FAST SWEEP stopped`．減速度 2000 で 600→0 は 90mm(半区画ちょうど)で，ゴールの手前の余裕がない(推測)．
  - 次: 直すかはユーザーが決める．
- **モードの一覧を入れ子(木の形)で書いた**(ユーザーの依頼．`docs/mode_tree.md`)．一番上の階層 → RUN / TEST / SD の中 → RUN の TEST の中 と，
  それぞれ選ぶ順，選ぶ値の表，AUTO の今の設定の流れ．操作・速度帯・止まったときの LED は `docs/mode_select.md` を見るようにした．
  `docs/mode_select.md` は SCOPE 3(全面探索)を「まだ作っていない」と書いたままで古い(直していない)．
- **`docs/mode_select.md` の古い所を直した**(ユーザーの依頼)．SCOPE 3 を「まだ作っていない(選ぶと走らずに止まる)」から，全面探索の中身
  (最短経路になりうる区画を回り，決まったらスタートへ戻る．決まれば地図を flash に残す)へ．冒頭の「どのコードをもとに書いたか」に
  `feature/autonomous`(10-11)を足し，`docs/mode_tree.md` への案内を入れた．止まったときの LED の連続のモードに AUTO を足し，
  「AUTO は電池では止めない」「探索で止まったときはシフトレジスタの LED が光らない」を書いた．
- **AUTO がログを SD に残すかを確かめた**(ユーザーの質問)．残す: 1本ごとに `PrepareStart` → `Logger_StreamBegin` で SD の `search/` にファイルを作って
  走りながら流し，`EndRunLog` で閉じる(探索・最短走行・帰り道・最後の全面探索で共通)．AUTO 1 回で 9 ファイル(search，fast × 4，back × 3，search)．
  コードは変えていない．SD が挿さっていない(マウントできない)ときは保存されず，走りは続ける．`mode_select.md` と `mode_tree.md` に書いた．
- **AUTO のログを確かめた**(ユーザーの依頼．`logs/search/` の 10-11 07:41 に取り込んだ 12 ファイル．コードは変えていない)．
  - 電池の電圧の並びから 2 回分と読んだ(推測)．1 回目: search_0033(40s) → fast_0032(ROUTE count=12) → back_0019 が 2.08s で
    FAILSAFE cause=2(速度の発散．イベントの b は電圧で 6.9V)．2 回目: search_0034(66.7s) → fast_0033〜0036 と back_0020〜0022 を交互
    → search_0035(最後の全面探索 22.1s，MAP_SAVED ok=1)の 9 ファイルがそろい，完走．1 回目と 2 回目は ROUTE が違う(別の迷路か，確かめていない)．
  - どのファイルも 5ms の行の抜けはなかった．最短走行 4 本の走り(尻当てを除く)は 12.15〜12.26s でそろっている．
  - back_0019: 直進の間 ad_l が 400〜790(REF_L 312)で左の壁に寄り，右のスラローム(1.33s)で ad_fl が 1301 まで上がって当たった．
    角度は目標 0° に対して 50° 付近で止まり(ang_corr が上限 -343.77)，右の車輪が回らず 0.7s 押し続けて速度の発散で止まった．電圧は 6.8V まで下がった．
  - fast_0035: 左 → 右 → 直進1区画 → 右 と続く所(RUN_CMD 7〜10)で，1 本目の右(7.21s)で ad_fl 1149，2 本目の右(7.70s)で ad_fl 1117 と当たり，
    角度が最大 45° 遅れた．その後の旋回で戻り，最後まで走った．4 本とも同じ所で ad_l が 1100〜1600 まで上がり，左へ寄るのは毎回(当たり方の差だけ)．
    fast_0035 だけ直前(6.785s)に EDGE_CORR corr=19.86(上限 20 の近く．他の 3 本はこの所で壁切れを見ていない)があり，左の旋回の始めの前のセンサが
    331/342(他は 256〜270)と前の壁に近い．壁切れの補正で旋回が遅れたのが重なったと見ている(推測)．
- **探索の補正が最短走行で使われているかを調べた**(ユーザーの質問．コードは変えていない)．最短走行(`RunList_Drive` / `FastStraightTo`)では，
  壁切れ補正は探索と同じく使う．壁の制御は 2 区画以上の直進の，両端の半区画を除いた所だけで，1 区画の直進(halves=2)では使わない．
  前壁補正(`WaitSlalomStart` の FL+FR による曲がり始め)は使わず，距離だけで曲がり始める．探索の既知の区間(`TryKnownRun`)も `RunList_Drive`
  を通るので同じで，back_0019 で当たったのもこの既知の区間の右の旋回だった．
- **最短走行にも探索と同じ補正を入れた**(ユーザーの依頼．ブランチ `fix/fast-run-corrections` を `feature/autonomous` から作った．
  コミットしていない `feature/autonomous` の変更もそのまま持ってきた．未確認: ビルド・実機)．変えたのは `search_run.c` だけ．
  - 壁の制御: `FastStraightTo` で，探索と同じく境界から境界までの間はいつも使う(中心から境界・境界から中心の半区画では使わない)．
    前は 2 区画以上の直進の，両端の半区画を除いた所だけだった．1 区画の直進(halves=2)にも入る．中心から中心へ走る `FastStraight`(超信地の最短)は前と同じ．
  - 前壁補正: `RunList_Drive` の小回りで，曲がる区画の奥に壁があれば `WaitSlalomStartV`(`WaitSlalomStart` を速さ・閾値を渡せるようにしたもの)で
    曲がり始める．奥の壁は地図(`s_map`，`WALL_VIEW_SEARCH` = 分かっている壁だけ)から引く．そのために指令の列をたどって今の区画・向きを持つ
    (`RunTrack`．最短走行はスタートの区画・北，既知の区間は C0 の1つ手前の区画 P・h0 から)．大回りでは使わない(探索に大回りがないので)．
  - 閾値は小回りの速さ(`s_fast_small_v`)での前の調整分から計算する(`s_fast_front_ref_sum`．`ComputeFastTurns` で UART に出す)．
    AT_PRE0 400 は 300〜600mm/s のログで合わせた値なので，速度帯の小回りがそれより速いときに合うかは分からない．
  - 範囲の手前側も使えるように，次が前壁補正をする小回りなら，前の直進・前の旋回の後ろのオフセットは「曲がり始め − 15mm」で待つのをやめる
    (探索で境界の 12mm 手前で壁を読むのと同じ役目)．
  - ログには最短走行でも `FRONT_TRIG` が出る．maze_sim は `search_run.c` を使わないので変わらない(見積もりの時間は前壁補正の分だけ実機とずれうる)．
- **補正を入れた後のログを確かめた**(10-11 08:04 に取り込んだ search_0001_dup2，search_0002_dup2，fast_0001_dup2．コードは変えていない)．
  - fast_0001 には `FRONT_TRIG` が 9 回出て，1 区画の直進でも `WALL_CTRL on=1` になった(新しいコードで走った)．
  - fast_0001 は RUN_CMD 18(区画 (2,5) で西 → 北へ右の小回り)で正面の壁に当たり，10.22s に FAILSAFE cause=2．原因は地図の誤り:
    (2,5) の北の壁(= (2,6) の南)を search_0002 の 54.17s に「ない」と読み，それまで 4 回「ある」と読んでいたのを上書きした．
    そのときは左の小回りの直後で右の壁に寄っていて(r=670，REF_R 282)，L=160 が WALL_TH_L 180 に届かなかった(search_0034 の同じ所は L=198 で「ある」)．
    経路(cost 200，count 31)はこの壁を抜けるので，前(cost 202，count 33)より短くなっていた．補正のせいではない．
  - `FRONT_TRIG` の 9 回のうち 8 回は範囲の手前の端(diff ≈ −14.6)で，FL+FR はすでに 409〜498(閾値はおよそ 360 台と推定)．
    探索でも 10-10 の試走会の迷路から同じで(at_lo が多い)，前壁補正は「いつも 15mm 早く曲がる」になっている．壁が前の迷路より明るく見えている(推測)．
  - search_0001 は 41.85s に FAILSAFE cause=2．ゴールから出るとき右の壁に寄っていて(r≈728)，(7,6) の左の小回りは前壁補正が範囲の
    終わり(diff=+14.9)で決まり，曲がる間に右・前の壁に当たった．行き止まりの (8,6) で真ん中に止まろうとして壁を押し続けた．
- **前壁補正を昨日(10-10)のログで合わせればよいかを調べた**(ユーザーの質問．コードは変えていない)．探索の壁を読む所(境界の 12mm 手前)で，
  前に壁があるときの FL・FR の中央値をファイルごとに並べた．
  - 家の迷路(10-08/09)・試走会の迷路(search_0003・0007_dup1)・10-10 の別の迷路(0009〜0019_dup1): FL 185〜214，FR 108〜143，和 304〜342．
    前壁補正は範囲の中で決まっていた(diff の平均 −3〜+2)．
  - **search_0020_dup1(10-10，FF 1.5 倍の確認)から後は FR だけが 206〜236 に上がった**(FL は 208〜238 でほぼ同じ)．和 404〜466 で，
    前壁補正は範囲の手前の端に張り付く．今日(10-11)のログも同じ．横の L・R の中央値はどの迷路でも 300〜320 でほぼ同じ．
  - 迷路が変わっても FR だけが段になって変わり，その後も続いているので，迷路ではなく機体(FR のセンサー・LED の向きなど)が
    search_0019 と 0020 の間で変わったと見ている(推測．未確認)．そうなら，試走会のログで合わせても今の機体には合わない．
- **FR の値の移り変わりを，スタートの尻当ての値で調べた**(ユーザーの依頼．コードは変えていない)．スタートの1回目の押し当ての最後の 0.2 秒は
  機体の位置が決まっていて，前のセンサーはスタートの区画の向かいの壁を見ているので，日をまたいで比べられる(中央値)．
  - 10-08〜10-09 午前: FL 500〜545，FR 191〜204．10-09 13:06 から FL だけ 424〜454 に下がった(FR はそのまま)．
  - 10-10 の試走会(fast_0008〜0012_dup1，search_0003・0007・0008_dup1): FL 433〜450，FR 198〜206．
  - **10-10 16:10 の fast_0023_dup1(FL 429，FR 200)と fast_0024_dup1(FL 513，FR 404)の間で，FL と FR が一緒に段になって変わった**．
    横の L・R(1回目・2回目の押し当て)は変わっていない．その後 10-10 の夕方は FR 388〜437．
  - 10-11: FR 389 → 441〜447(AUTO) → 454 → 469 → 484 と上がり続けている．2回目の押し当て(前が開いている向き)の FR も 1〜10 → 17〜38 に上がった．
  - 前のセンサーの向き・取り付けが 10-10 16:10 ごろに変わり，FR は今もずれ続けていると見ている(推測．未確認)．試走会のログは変わる前の値．
- **最短走行の前壁補正を切った**(ユーザーの依頼．`fix/fast-run-corrections`．未確認: ビルド・実機)．params.h に `FAST_FRONT_ENABLE 0` を足し，
  `RunList_Drive`(最短走行と探索の既知の区間)の小回りでは前壁補正を使わず，距離だけで曲がり始める(範囲の手前で待つのをやめる所も使わない)．
  探索の小回り(`SLALOM_FRONT_ENABLE`)と，1 区画の直進の壁の制御はそのまま．
  - 壁切れ補正で耐えるかの根拠: 前壁補正なしの最短走行は，10-10 の試走会の 5 本(fast_0008〜0012_dup1)と今朝の AUTO の 4 本(fast_0033〜0036)が
    最後まで走った．壁切れ補正は 1 本に 1〜9 回，RMS 6〜17mm で，上限 20mm に届く回もある．壁切れ補正は前後の位置だけを直すので，
    直進をはさまずに小回りが続く所(fast_0035 で当たった所)では効かない．
- **AUTO の間は低電圧のフェイルセーフを使わないようにした**(ユーザーの依頼．AUTO の途中で低電圧のフェイルセーフで止まった．
  前回は「電池の電圧では止めない」を，最短走行の前の電圧の確かめを消すことと受け取り，フェイルセーフの低電圧は残していた)．
  `FailSafe_SetLowVoltageEnabled()` を足し，`AutonomousRun_Run` の始めで false にする(電源を切るまで戻らないので戻さない)．
  速度偏差・角速度のフェイルセーフは効く．AUTO を選ぶ前に発動していたら今までどおり止まる．params.h・`docs/mode_select.md` の書き方も直した．
  未確認: ビルド・実機．電池が 7.0V を下回っても走り続けるので，過放電に気をつける．
- **search_0007_dup2(08:35 に取り込んだ最後のログ)が止まった理由を調べた**(ユーザーの質問．コードは変えていない)．
  ゴールに着いた後の帰り(PHASE 1)で，26.56s に (0,1) を南向き(スタートへ向かう)に入るとき，前に壁がないのに FL 81 + FR 83 = 164 が
  WALL_TH_FRONT_SUM 150 を超えて「前に壁あり」と読み，スタートの北の壁を地図に入れた．スタートへの道がなくなって PHASE 3(FAILED)．
  フェイルセーフではない．
  - 前が開いているときの壁を読む所の FL+FR: 試走会(search_0003・0007_dup1)は最大 64(壁ありの最小 269)．今日(search_0002〜0007_dup2)は
    最大 150 で，151・154・164 を壁ありと読んだ回がある(本当の壁ありは 282 から)．前のセンサーが 10-10 16:10 ごろから明るく読むので，
    前の壁の閾値の余裕がなくなった．
  - ユーザーの判断: 前の壁の閾値(`WALL_TH_FRONT_SUM`)・前壁補正の閾値は変えない(試走会の会場で合わせた値を信じる)．
- **大会(2026 東北地区大会)の AUTO のログを確かめた**(ユーザーの依頼．`logs/search/` の 10-11 12:18 に取り込んだ 7 ファイル．コードは変えていない)．
  - 並び: search_0001_dup3(80.4s．50.70s に PHASE 1，74.14s に PHASE 2，MAP_SAVED ok=1) → fast_0001_dup3 → back_0001_dup1 → fast_0002_dup2
    → back_0002_dup1 → fast_0003_dup2 → back_0003_dup1(7.02s で FAILSAFE cause=2)．4 本目の最短走行と最後の全面探索はない．どのファイルも 5ms の行の抜けはない．
  - 最短走行 3 本は同じ経路(ROUTE cost=252 est_s=15.14 count=45)で，RUN_CMD 0 から 44 までが 15.09〜15.10s とそろい，見積もりとほぼ同じ．
    壁切れ補正の RMS は 7.8 → 9.7 → 10.4mm と 1 本ごとに増え，3 本目は 7.64s に上限の 20mm に届いた．左の小回り(RUN_CMD 1，5，7，20，21，26 など)で
    ad_l が 900〜1050 まで上がるのは 3 本とも同じ所(朝の AUTO と同じく左へ寄る癖)．電池は 8.02V → 7.81V(走行中の最低 7.03V)．
  - back_0003: 既知の区間(KNOWN_RUN cells=18)の RUN_CMD 10(左の小回り，前壁補正なし)で，曲がり始め 0.06s 後(6.82s)から ad_l が 133，
    6.87〜6.89s に ad_l 170 → 368，ad_fl 132 → 362 と上がり，左(内側)の車輪から止まった．6.91s から右の PWM が 4199 に張り付き，
    角度は 404° で止まって(目標 431°)，7.015s に速度の発散で止まった．back_0001・0002 の同じ所は ad_l が 4〜10 のまま回り切っている．
    曲がる前の直進(RUN_CMD 9)の壁の制御・角度のずれ(−1〜−1.4°)・ad_r(266〜274，他は 255 前後)はほぼ同じで，壁切れ補正も 4.7mm(他は 9.2，18.2)と小さく，
    左の内側の柱に当たった理由はログからは決められなかった(推測: 曲がる間の横のずれか，車輪のすべり)．
- **今の状態を「2026 東北地区大会」として master にマージしてプッシュした**(ユーザーの依頼)．`fix/fast-run-corrections` のコミットしていない変更
  (最短走行の補正，AUTO の低電圧のフェイルセーフを切る，maze_sim の変更，大会で使った bin など)をコミットしてから，master へ `--no-ff` でマージした．
  `base_plate.DXF` と `.settings/stm32cubeide.project.prefs` は入れていない．
- **TEST にすべての LED を点けたままにするモード LED_ALL_ON を足した**(ユーザーの依頼．ブランチ `feature/led-all-on` を master から作った．未確認: ビルド・実機)．
  TEST の 13 番(末尾．前からある番号は変えない)．シフトレジスタの LED1〜16 と直結の LED_1〜6 を点け，電源を切るまでそのまま(`LEDAllOn_Run`，`led_test.c`)．
  `RobotMode` の値も末尾(`MODE_AUTONOMOUS` の後)に足したので，ログの `MODE mode=` の番号は今までと同じ．`docs/mode_select.md` と `docs/mode_tree.md` にも足した．
- **LED_ALL_ON をコミットして master にマージし，プッシュした**(ユーザーの依頼)．
- **大会の日の 12 時以降のログを `logs/archive/2026tohoku/` にまとめ，午後のログを確かめた**(ユーザーの依頼．コードは変えていない)．
  - `logs/search/` の 10-11 12:18 と 14:49 に取り込んだ 19 本(.bin と .csv の 38 ファイル)をコピーした(元の `logs/search/` にも残してある．`logs/` は git に入れない)．
    12:18 の 7 本(AUTO)は上の「大会の AUTO のログを確かめた」で見たもの．ここでは 14:49 の 12 本を見た．
  - 並び(電池の電圧から推測): FAST_RUN(mode=13)で fast_0004〜0013 の 10 本 → SEARCH(mode=11．全面探索 scope=3)で search_0002，0003．
    どのファイルも 5ms の行の抜けはない．経路はどれも AUTO の search_0001_dup3 で残した地図のもの(cost=252)．
  - 最短走行(RUN_CMD 0 からゴールで止まるまで．見積もりは最後の指令の始まりまでと比べた):

    | ファイル | 型 | 速さ/加速度/減速度/小回り | 結果 | 走り | 見積もり |
    |---|---|---|---|---|---|
    | fast_0004 | LARGE | 1500/8000/5000/400 | 完走 | 17.09s | 16.13s(実 16.38，+1.5%) |
    | fast_0005 | 超信地 | 1500/3000/3000/700 | 13.85s で FAILSAFE cause=2 | − | − |
    | fast_0006，0010，0012，0013 | SMALL | 1500/8000/5000/700 | 4 本とも完走 | 12.46〜12.53s | 11.66s(実 11.77〜11.83，+1.0〜1.5%) |
    | fast_0007，0009，0011 | LARGE/SMALL | 1500/8000/5000/700 | ジャイロが効かず途中で終わる(下) | − | − |
    | fast_0008 | LARGE | 1500/8000/5000/700 | 完走 | 15.52s | 14.63s(実 14.82，+1.3%) |

    午前の AUTO(1200/2000/2000/600 SMALL)は 15.78s なので，SMALL 1500/8000/5000/700 で約 3.3 秒速くなった．この迷路では LARGE より SMALL が速い(見積もりも同じ)．
  - 完走した本の様子: 最高 1540〜1590mm/s．速度の誤差(平均の車輪 − 目標)は −176〜+173mm/s(午前は −93〜+70)．壁切れ補正は 6 本とも上限の 20mm に届き，
    SMALL では RUN_CMD 8 で毎回 +20(fast_0013 だけ 19.6 の別の所)．LARGE は ad_l が 1900 まで上がり左の壁にかなり寄る．fast_0008 は角度の誤差が −12.8° まで開いた．
  - **fast_0007，0009，0011: ジャイロが回転を読んでいない**．GYRO_RECAL が 175.8，735.5，1202.0 LSB(ほかの本は 4.9〜5.4)で，最初の超信地(−90°)の間の gyro_z は
    平均 −7〜+13dps(ほかの本は −180)，車輪の左右差は 560〜640mm/s(超信地の 3 倍ほどの速さで回っている)．角度の補正は上限 ±343.8 に張り付いた．
    0009・0011 は 0.9s，0007 は 7.2s で記録が切れ，.bin が 32MB のまま(ファイルを閉じる前に電源を切ったか，リセットした．推測)．
    間の本(0008，0010，0012)は正常なので，起動ごとに起きたり起きなかったりする．`ICM_Init` は WHO_AM_I を表示するだけで確かめず，設定の書き込みも読み返していないので，
    初期化の失敗(または SPI・コネクタの接触)を見分けられない(推測．原因は未確認)．
  - fast_0005(超信地): RUN_CMD 8/9(2 区画の直進，1000mm/s 付近)の減速の始まり(12.53s)で gyro_z が +185 → +252dps と跳ね，両輪が 5ms で 200mm/s 落ちた．
    その後 PWM が 4199 に張り付いたまま車輪が止まり，ぶつかって止まった．直前の角度は目標より 3.2° 右(加速の間に毎回 2〜3° 右へずれる)で，ad_r が 320〜383 に上がっていたので，
    右前を右の柱に当てたと見ている(推測)．そのまま次の超信地と直進へ進み，13.85s に速度の発散で止まった．
  - search_0002(全面探索 800/700/3000): 75.00s に PHASE 1(最短経路が決まり帰る)．帰りの既知の区間(KNOWN_RUN cells=16)の RUN_CMD 1(右の小回り，700mm/s，
    前壁補正なし)の終わりで ad_fl が 996 → 1150 に上がって前左に当たり，89.62s に FAILSAFE cause=2．曲がる前の直進の壁切れ補正は +14.8mm と大きめ．
    曲がる区画の奥に壁はない(ad_fl 57)．曲がり始めが遅れて外へふくらんだ(推測)．
  - search_0003(全面探索 800/600/2000): PHASE 1 の前，85.50s に FAILSAFE cause=1(低電圧 6.77V)．80.92s に行き止まりの (12,1) で真ん中に止まろうとして，
    目標の 14mm 手前で前の壁に当たって止まり(ad_fl 1155)，その後の尻当て 2 回で後ろへ押し続ける間に PWM が −2000〜−2600 まで上がり，電圧が 6.8〜6.9V に下がった．
    電池は走り始めで 7.6V(この日の 12 本目)．AUTO 以外では低電圧のフェイルセーフが効くので止まった．

### 2026-10-13 CubeIDE をやめて VS Code + CMake でビルドする準備(`feature/vscode-cmake`，`master` から)

- **CubeIDE を卒業するため，CubeMX の Toolchain/IDE を CMake にして作り直し，CubeIDE の設定を CMakeLists.txt に移した**(ユーザーの依頼．参考: Qiita の akiaki96 さんの記事)．
  - VS Code の拡張は，記事のやり方(CMake Tools + clangd，Mac 向け，デバッグなし)ではなく **ST 公式の STM32CubeIDE for Visual Studio Code** にした．
    Windows で使え，ツールを自動で入れ，ST-Link でデバッグできるため．どちらもビルドの土台は CubeMX が作る CMake で同じ．
  - CubeMX で作り直すと，`.ioc` の Compiler/Linker が GCC から Starm-Clang に変わり，`.ioc` 側では GCC に戻せなかった．
    `CMakePresets.json` の toolchainFile を `gcc-arm-none-eabi.cmake` に替えて，CubeIDE と同じ GCC でビルドするようにした．
    次に作り直したとき，`CMakePresets.json` が starm-clang に書き戻されるかは未確認(`git diff` で見る)．
  - CubeIDE の設定から引き継いだもの(すべて CubeMX が作り直さない `CMakeLists.txt` に書いた):
    - Debug も `-O2 -g3`(CubeMX の既定は `-O0`)．`add_subdirectory` より前に書かないと HAL・FatFs が `-O0` のままになる．
    - リンカースクリプトは前の `STM32F405RGTX_FLASH.ld`．CubeMX が作った `STM32F405XX_FLASH.ld` は FLASH 1024K で，地図を保存する最後のセクタ(896K〜)を空けていない．
      名前を置き換えられなかったら CMake が止まるようにした．
    - `-u _printf_float`，`Core/Src/app`・`interface`・`logic` の .c をすべて(47 個)，ビルドの後に `yuho.bin` を作る．
  - CubeMX が作ったスタートアップ `startup_stm32f405xx.s` は `Core/Startup/startup_stm32f405rgtx.s` と中身が同じ．`.gitignore` に `build/` を足した．
  - 結果: ビルドはまだしていない(未確認)．
  - 次: VS Code に ST 公式の拡張を入れてビルド・書き込み・デバッグを試す．その後，CubeIDE のファイル(`.cproject`，`Debug/`)と使わないファイルの扱い，
    `Debug/yuho.bin` をコミットする決まり，CLAUDE.md の「ビルドと書き込み」を決め直す．コンパイラの版が CubeIDE と違い得るので，最初はいつもの試験で走りを確かめる．

### 2026-10-20 log_viewer を一般化して配布できるようにした(`feature/log-viewer-package`，`master` から)

- **MATLAB の log_viewer を，他のマイクロマウスの人が自分のログで使える形にして，別のリポジトリ `M:/User/Desktop/school/club/log-viewer` に移した**(ユーザーの依頼)．
  - ユーザーの決定: 相手は他のマイクロマウスの人，対象は log_viewer だけ(plot_log・turn_sim・get_log.py は yuho に残す)，別リポジトリ＋ `.mltbx` で配る，
    読み込みは差し替えられるように，画面と説明は日本語のまま，yuho も新しい方を使う．途中でハーフ(90 mm)の迷路も足すことになった．
  - 作り: 本体(`mouse_log_viewer`，`+mlv` パッケージ)は yuho の名前を一つも知らず，機体ごとの違いを設定の struct で受け取る．
    設定は JSON ではなく MATLAB の関数ファイルにした．読み込み・軌道の基準・壁の取り出しは手順なので，関数ハンドルで渡す必要があるため．
    - 読み込み: `cfg.readers` を上から試す(内蔵は YLOG と CSV)．自分の形式は先頭に足す．
    - イベント: 文字の列(`ev_text`)があればそれを，なければ番号の列と中身の列から作る．yuho の `log_event.h` は `mlv.event_table_from_c` で読む．
    - 迷路: `cfg.maze.kind` が `'classic'`(180 mm，16 区画，柱 12 mm)か `'half'`(90 mm，32 区画，柱 6 mm)．画面でも切り替えられる．
      柱の大きさ，図の余白，向きの矢印の長さも区画の大きさに合わせて変わる．
    - 壁: 「壁の層」(線分，分かった行，描き方の名前)の並び．既定は `WALL x= y= dir= wall=` のイベント．
  - yuho 側: `tools/matlab/yuho_viewer_config.m` に yuho の取り決め(列の組，`log_event.h`，`MAZE_SIZE`，尻当て・STEP の基準と合わせ直し，
    機体の地図と STEP で読んだ壁)を移し，`log_viewer.m` はそれを渡して `mouse_log_viewer` を呼ぶだけにした．`+yc/load_log.m` などは plot_log・turn_sim のために残した．
  - 確かめたこと(R2026b Prerelease の `-batch`，log-viewer の `tests/run_tests.m`):
    - yuho のログ 5 本(search の .bin/.csv，stream の .bin，slalom の .bin/.csv)を `yc.load_log` と `mlv.load_log` で読み，列とイベントの文字がすべて一致した．
    - search_0003_dup3 の画面を `exportapp` で書き出し，前の log_viewer と並べて見比べた．迷路の種類の選択が増えたことと，チェックボックスの並び
      (機体の地図が先．先の層が下に描かれるため)以外は同じ．途中のイベントへ飛んだときと，STEP で合わせ直したときの画面も見た．
    - 合成した最小の例(クラシックとハーフ)で，軌道の終わりの位置と向き，壁の数が合った．自分の読み込みを足す例も通った．`.mltbx` も作れた．
  - まだ確かめていないこと: マウスとキーでの操作(クリック，ホイール，← →，再生)，`.mltbx` を入れて使えるか，R2026b より前の版で動くか．
  - 次: ユーザーが画面を触って確かめる．log-viewer のコミットと GitHub への公開はユーザーが決める．

### 2026-10-20 log-viewer: 設定を画面で選ぶ，イベントの線の切り替え，再生を軽く(`feature/log-viewer-package` のまま)

- ユーザーが `mouse_log_viewer` を設定なしで直接開いたら，「速度・壁センサが1段にまとまらない」「読んだ壁が出ない」「再生で機体の点が飛び飛び」となった．
  - 前の版(master の log_viewer.m，Python の log_viewer.py)と機能を並べたが，なくなった機能はなかった．設定なしだと既定の設定(列の組も壁の層もない)になるのが原因．
    「壁センサの値から壁判定して表示」は，機体が STEP で残した壁(読んだ壁)のことだった(ユーザー確認)．
  - **設定を画面で選べるようにした**: `mlv.add_profile(名前, 関数名, フォルダ)` で登録(`setpref` の `mouse_log_viewer` に残る)，画面の上の「設定」で選ぶ．
    最後に使った設定で開く．読めない設定なら既定で開いて状態の行で知らせる．yuho の `log_viewer` は呼ぶたびに `'yuho'` を登録してそれで開く．
  - 設定のない人向けには，画面で列をまとめる機能は作らず，作り方の手引き(log-viewer の `docs/config_guide.md`)とひな形(`examples/template/my_mouse_config.m`)を作った(ユーザーの決定)．
  - **グラフのイベントの縦線を出す・出さない**のチェックボックス「グラフに線」を足した．
  - **再生が重いわけを測った**(-batch で 1 コマを描き終えるまでの時間．同じ search_0003_dup3)．
    直す前は 1 コマ約 110 ms(約 9 コマ/秒)．内訳は，時系列のカーソルの `xline` が約 120 ms(普通の `line` にすると約 30 ms)，
    つまみ(uislider)と時刻・状態の文字(uilabel)を変えるだけでそれぞれ約 75 ms(画面の部品を変えると窓全体を並べ直すため)，迷路の点は約 15 ms．
    イベントの線・凡例・データの点の数はほとんど効いていなかった．
    `xline` をやめて普通の線にし，再生中はつまみ・時刻・状態の行・イベントの一覧を 0.2 秒に1回だけ直すようにして，
    約 43 ms(yuho の設定)/ 55 ms(既定の設定)になった．前の版も同じ作りだったので，前から重かったと考えられる(実際の画面での速さは未確認)．
  - ついでに直した不具合: 「開いたときに描く列」がどれもログにないと，開くときにエラーになっていた(前の版も同じ)．最初の 4 列を描くようにした．
  - 確かめ: log-viewer の `tests/run_tests.m` がすべて通った(読み込みの一致，設定の登録・記憶・画面での切り替え，読めない設定，縦線の切り替え，ひな形，手引きの壁の層の例)．
  - まだ: 実際の画面での再生の滑らかさ，マウスでの切り替えはユーザーが確かめる．


### 2026-10-21 log-viewer: 設定がなくても列を1つのグラフにまとめる(`feature/log-viewer-package`)

- ユーザーから「センサ値や目標速度と実際の速度を1つのグラフに描けない」．
  - 既定の設定(列の組が空)で開いていたと考えられる(この PC の Prerelease では最後の設定が「既定」だった．正式版の状態は未確認)．
  - 10-20 は「設定のない人は設定を書く」としたが，今回のユーザーの決定で，**名前から自動でまとめる**ことにした．まとめ方は一般化する前の yuho と同じにし，設定ファイルで変えられる形にする．
  - `cfg.groups` の列を**名前の形(正規表現，全体一致，大文字・小文字を区別しない)**で書けるようにした．ふつうの列の名前はそのまま使える．
    既定の組 `mlv.default_groups` は yuho の `yc.plot_groups` と同じ段の名前と並びで，`v_l`，`sen_fr`，`ir_r`，`pwm_l` などのよくある名前にも合う．
  - どの組にも合わない列は，名前の終わり(`_l`/`_r`，`_fl`/`_fr`，`_x`/`_y`/`_z`，`_ref` など)を除いて同じになれば1段にまとめる(`cfg.auto_group`，既定 true)．
  - 開いたときに描く列 `cfg.default_columns` も名前の形にし，既定を yuho と同じ(速度の目標と左右，角度，壁センサ)にした．
  - 「この組を足す」の一覧は，開いたログに列がある組だけを出す(`mlv.resolve_groups`)．
  - 確かめ: log-viewer の `tests/run_tests.m` がすべて通った．yuho のログでは既定の組が yuho の組と一致し，既定の設定で開いても速度・角度・壁センサーの3段になる．`.mltbx` も作り直した．
  - その後「search で壁を読んでいるのに迷路の壁が増えない」．これも既定の設定で開いていたため(既定では STEP の意味を知らないので壁は 0 本)．画面の「設定」で yuho を選んで直った(ユーザー確認)．
  - yuho の `tools/matlab/README.md` の「既定の設定で開くと…」を今の動きに合わせた(列の組は出る，地図・読んだ壁・軌道の基準は出ない)．
  - まだ: 実際の画面での確認はユーザーが行う．

### 2026-10-21 maze_sim を一般化して公開できるようにした(`feature/maze-sim-package`，`feature/log-viewer-package` から)

- **Python の迷路シミュレータ(maze_sim)を，他のマイクロマウスの人が自分の探索の C で使える形にして，別のリポジトリ `M:/User/Desktop/school/club/maze-sim`(mouse maze sim，`mazesim` パッケージ)に移した**(ユーザーの依頼)．
  - はじめは「MATLAB にして公開」の依頼だったが，ユーザーが「MATLAB にする利点がない」として Python のままにした．
  - ユーザーの決定: 機体は小さな C の窓口でつなぐ，入れる機能は探索の画面と一括評価・迷路図の変換・最短経路と時間の表示(任意)，
    公式図から変換した 92 個(前の `my_mazes/`)を同梱する，yuho も公開版を呼ぶ形にする．ハーフ(32×32)は頼まれていないが，窓口に大きさを持たせ，画面も大きさに合わせて描くようにした．
  - log-viewer の公開のときの決まりに合わせた: 本体は yuho を知らない，日本語のまま，MIT，作者名 sararint でメールは載せない(リポジトリの git config も同じ)．
  - 作り: 本当の迷路・壁の観測・移動・衝突の判定を Python 側に移し，C の窓口(`include/msim.h`)は「見えた壁(前・右・左) → 次の1手」だけにした．
    必ず作るのは `msim_api_version`，`msim_reset`，`msim_step` の3つ．作ると画面に出るもの: 地図(`msim_wall`)，区画の数字，状態の行，オプションの申告と途中での変更，
    結果の数値(まとめて試すと平均)，最短走行の経路(分かった壁だけ / 全部知っていたら)．機体ごとのキーは Python の拡張(`mouse.json` の `extension`)で足す．
    機体は `mouse.json`(ソース，インクルード，ビルドし直す条件)で表し，ソースが変わると読み込むときに自動でビルドし直す(ファイル名にハッシュを入れ，画面が使っている間でもビルドできる)．
  - 同梱の例: 足立法(窓口をすべて作った例．16×16 と 32×32)，ひな形(左手法)，yuho のつなぎ方(参考)．
  - 設定(お気に入り，最近開いた迷路，最後に使った機体)は `~/.mazesim/prefs.json`，自分の迷路は `~/.mazesim/mazes/`，mazefiles は `~/.mazesim/mazefiles/` に移した．
  - yuho 側: `yuho_mouse.c`(窓口の yuho の実装．前の `sim_api.c` の中身)，`mouse.json`，`yuho_ext.py`(X / Z キーの plant_sim)，`msim.py`(`--mouse` を付けて公開版を呼ぶ)，`gui.py`(`msim.py gui` の近道)．
    前の GUI のオプションは `--opt` のオプションになった(algo / scope / band / turn / search / goal_cost / back_cost．A / O / K キーはそのまま)．
    `sim_api.c`，`sim_lib.py`，`maze_catalog.py`，`run_motion.py`，迷路図の変換の3つ，`fetch_mazes.sh`，`my_mazes/` を消した．C の CLI(`maze_sim.c`)と `sweep.sh` は残した(迷路の場所だけ直した)．
    公開版の場所は `pip install -e`，`tools/maze_sim/mazesim.local`(gitignore)，環境変数 `MAZESIM_PATH` のどれか．
  - 確かめたこと:
    - 乱数の迷路の作り方を C から Python へそのまま移し，同じシードで同じ迷路になる．C の CLI と公開版で，乱数 100 迷路(Dijkstra，足立法，全面探索)と同梱の全日本クラシック 6 個の
      区画数・探索の時間・ゴールの時刻・コストの最短の数と比・最短走行の時間(時間 Dijkstra，大回り)・全部知っていた場合の時間がすべて一致した．
    - 公開版の `tests/run_tests.py`(yuho の `mouse.json` も追加で試す): 足立法と yuho が乱数 30 迷路と同梱の迷路(SC を除く)を最後まで探索，ひな形がエラーなく動く，
      画面を dummy で動かして探索・最短走行の再生・経路の切り替え・オプションのキー・迷路の一覧から開く，を PNG で見た．
    - X キー: 乱数の迷路 7 で，迷路の書き出し → 探索の時間の見積もり(34.5 秒) → plant_sim(`--plant-no-build` 相当) → ログの変換まで通った．
      plant_sim は 38 秒目に CRASH(reason 4)で止まった．ビルドし直していない plant_sim なので古い可能性がある(今回の範囲外で，調べていない)．
    - 迷路図の変換: 公式図 MM2024CM を変換し，同梱のものと壁が完全に一致．
  - まだ確かめていないこと: 実際の画面での操作(キー・再生の見た目)，`pip install -e` で入れたとき，Windows 以外，Z キー(log_viewer を開く)．
  - 次: ユーザーが画面を触って確かめる．公開版のコミットと GitHub への公開はユーザーが決める．

### 2026-10-21 maze_sim から plant_sim を外し，時間を計算上の速さの運動だけにした(`feature/maze-sim-package` のまま)

- ユーザーの指摘: plant_sim とログ出しは maze_sim に入れないはず．band(速度帯)も分からない．確かめると，band・探索の時間の見積もり(ログで合わせた実機の動き)・plant_sim(X / Z キー，`--plant-log`)は
  10-11 のコミット `4dfb0f3`(東北地区大会のまとめ)で maze_sim に入っていた．10-09 にユーザーは「maze_sim はアルゴリズムの確認だけ」として走りのパラメータを戻していた．
  - ユーザーの決定: maze_sim はアルゴリズムの確認だけ．plant_sim のログ出し(X / Z)は消す．時間に依存するアルゴリズムの確認には速さが要るので，速さは「計算上のもの」を使い，
    **滑りなどは一切考えない**．速さは選べるようにし(既定は `params.h`)，時間は運動だけで数える(尻当て・待ち・超信地旋回は数えない)．
  - msim(公開版を呼ぶ側)で直したこと:
    - `yuho_ext.py` を消し，`yuho_mouse.c` から `yuho_plant_log`・`yuho_search_estimate`，`mouse.json` から `plant_bridge.c`・`search_time.c`・拡張，`msim.py` / `gui.py` から `--plant-*` を外した．
    - オプション `band` と `turn` をやめ，`run`(最短走行の VMAX,ACCEL,SMALL90,LARGE90,LARGE180．既定 `RUN_*`)を足した．`search` は残した(既定 `SEARCH_V_MM_S` など)．
    - 最短走行は logic 層の `RunProfile_Default()` の形(理想の円弧の長さ，オフセット 0，減速度 = 加速度)に `run` の速さを入れる(前は `RunProfile_ForSpeeds`: 滑りのモデルのオフセットと `*_ADJ` の補正入り)．
    - 探索の時間は `yuho_mouse.c` の `SearchTime_*`(運動だけ．境目は小回りの速さ，直進は台形加速，小回りは円弧 ÷ 速さ，真ん中で止まる・回るのは 0)．既知の区間をまとめて走る分は数えない．
  - 確かめ: 警告なしでビルド．小回り 0.236 s(= π/2 × 90 mm ÷ 600 mm/s)・大回り 90° 0.333 s(= π/2 × 180 ÷ 850)．乱数 100 迷路の探索の区画数・コストは前と同じ．
    `run` を速くすると最短走行の時間が短くなる．公開版の `tests/run_tests.py` が yuho を含めて全部通った．
  - C の CLI(`maze_sim.c`)は前のまま(`--band`，`search_time.c`，`--plant-log`)．README に「この CLI の時間は前の数え方」と書いた．直すかはユーザーに聞く．
  - 公開版の `examples/yuho/` も写し直し(`yuho_ext.py` を消した)，`docs/api.md` から yuho の拡張の例への言及を消した．

### 2026-10-21 C の CLI も運動だけの時間にした，画面で数字のオプションを打ち込めるようにした(`feature/maze-sim-package` のまま)

- ユーザーの依頼: CLI も同じ形にする．並進とスラロームの速さを画面で数字で入れられるとよい．
  - 時間の数え方を `tools/maze_sim/ideal_time.c` / `.h` にまとめ，`yuho_mouse.c` と C の CLI(`maze_sim.c`)で共有した．
  - CLI: `--band`，`--pivot`，`--plant-log` などの plant_sim のオプションを消し，`--run VMAX,ACCEL,SMALL90,LARGE90,LARGE180` を足した．探索の時間の内訳(尻当て・待ちなど)の表示もやめた．
    `search_time.c` / `.h`，`plant_bridge.c` / `.h`，`sim_core` の速度帯(`SimCore_FastProfile`，`SIM_DEFAULT_BAND`)を消した．
  - 公開版の画面: 選択肢のないオプション(数字など)を **E** キーで打ち込める(右の欄の `[E]`．Enter で決める，Esc でやめる，Tab / ↑ / ↓ で別の項目．入力中は説明が出る)．
    機体が途中で変えられるもの(yuho の `run`)は探索をやり直さず，それ以外は最初から．おかしな値は機体の `msim_reset` のエラーとして理由が出る．
  - 確かめ: CLI と msim で，乱数 100 迷路の区画数・探索の時間(35.0 s，ゴール 14.9 s)・コスト・最短走行の時間(6.521 / 6.529 s，best 6.224 s)が一致した．
    公開版の `tests/run_tests.py` に入力の確かめを足した(yuho で `run` を打ち込むと探索をやり直さずに最短走行が 11.207 → 8.747 s，おかしな値は ERROR と理由)．全部通った．
  - `tools/maze_sim/plant_sim.local` はもう使わない(git に入れないこの PC のファイルなので消していない)．

### 2026-10-21 mouse maze sim: 画面をブラウザにした，探索関数を名前で選べるようにした(公開版．yuho は `feature/maze-sim-package` のまま)

- ユーザーの依頼: pygame の画面は操作性がよくないので，ブラウザで動く画面にする．C の探索関数の名前で選べるようにする(「窓口の形の関数を自動で一覧」をユーザーが選んだ)．
  - 画面: `python -m mazesim gui` が手元だけ(127.0.0.1)の小さな Web サーバー(Python の標準機能)を立ててブラウザを開く．計算は Python と機体の C，ブラウザは描画と操作だけ．
    ボタン，オプションの選択欄と入力欄(Enter で決める)，速さのつまみ，区画をクリックで数字と壁，迷路の一覧(分類の木，絞り込み，プレビュー，ダブルクリックで開く，お気に入り)．
    最短走行の再生の形は `motion.js`(前の `motion.py` を JavaScript に移した)．キー操作も残した．pygame の画面(`gui.py`，`motion.py`)と画面の拡張(extension)は消し，pygame は要らなくなった．
  - 探索関数: 機体のソースから `MSIM_EXPORT void 名前(const MsimSense *, MsimMove *)` の形の関数を探して一覧にし，画面の「探索関数」とコマンドの `--step` で選ぶ(既定 `msim_step`)．
    選んだ名前は `msim_reset` の options に `step=名前` で来る．ファームウェアの関数は引数の型が機体ごとに違うので，この形の薄い関数で包む必要はある(ユーザーに説明した)．ひな形に例(`right_hand_step`)を足した．
  - yuho が使うのは書き込み用と同じ `Core/Src/logic/` のソースと `params.h` であることを確かめた(ユーザーの質問)．`app/search_run.c` の流れ(区画ごとの Step，ゴールでの 180°，片道)は `yuho_mouse.c` が真似ている．
  - 確かめ: 公開版の `tests/run_tests.py` を書き直した(サーバーの /api で探索・オプション・探索関数・一覧・お気に入り・前後の迷路を試し，画面なしの Chrome で開いて PNG と JavaScript のエラーを見る．
    わざとエラーを入れたページで，エラーを拾えることも確かめた)．足立法・ひな形・yuho で全部通った．yuho の `msim.py gui` でもサーバーが立つことを確かめた．
  - 公開版はコミット1つ(`mouse maze sim 1.0.0`，sararint)にまとめ直した．GitHub への push はリポジトリができてから(この PC に gh がない)．

### 2026-11-03 走行系を機体モデル(yuho_model.md)の形にした(`feature/model-based-control`，`master` から．ワークツリー `yuho-model`)

- **車輪ごとの速度 PID と FF をやめ，システム同定で求めた機体モデルの逆の FF と，並進・旋回に分けた FB にした**(ユーザーの依頼)．
  - モデルは `yuho-sysid` の `yuho_model.md`(11-03 時点)．`docs/yuho_model.md` に写した．
  - 電圧を並進 V_s = (V_L + V_R)/2 と旋回 V_d = (V_R − V_L)/2 に分け，V_L = V_s − V_d，V_R = V_s + V_d でかける．
  - FF(`logic/control/drive_model.c`．ハードに触らない):
    - V_s,ff = a·v + V0·sign(v) + b·a_ref + c·v·ω²
    - V_d,ff = b_ω·ω̇ + D + F_ff(F_ff は h = A±·asinh(κ/κ0(v)) へ σ/v の時定数で近づける．曲がらないと 0 へ戻る)
    - 超信地旋回(v_ref < 0.05 m/s で ω_ref ≠ 0)はモデルの範囲の外なので，V_d,ff = b_ω·ω̇ + D + 摩擦(向きごと) + a·(T/2)·ω の仮の式．
      摩擦 `PIVOT_VD_FRIC_CCW/CW`(0.87/0.94V)は，前の車輪ごとの摩擦 FF を V_d に直しただけ．
  - FB(yuho_model.md の 7 章のまま): 並進は v の PI(5.8 V/(m/s)，29 V/m，I 項 ±0.5V)．旋回は θ・ω の PD(8.28 V/rad，0.386 V/(rad/s)，±1.5V)．
  - そのまま残したもの: 位置のループ(並進の FB の目標 v_cmd に足す)，壁の制御(目標の向き θ_ref のオフセット)，壁切れ，プロファイル，フェイルセーフ，外への関数．
  - 尻当て: 旋回の FF・FB を止めて左右に同じ電圧をかけ，並進の FB(P + I)を ±4V(`SEARCH_SETPOS_PUSH_FB_MAX_V`，前の `..._PID_MAX_V`)にした．
    前は車輪ごとの PID の上限 4V だった．左右に同じ電圧で壁にそろうかは未確認．
  - 消したもの: `VELOCITY_KP/KI/KD`，`VELOCITY_FF_*`，`PIVOT_FF_FRIC_*`，`ANGULAR_KP`，`ANGLE_KP`，`ANGULAR_CORR_LIMIT_RAD_S`，`ANGULAR_FF_ACC_SCALE`，
    `VELOCITY_STOP_RESET_MM_S`(止まったら積分を捨てる処理．角度・位置の制御が有効なら使っていなかった)，`pid.c/h`，`velocity_pid.c/h`，`position_pid.h`．
  - ログの列: `i_l` → `i_s`(並進の I 項)，`i_r` → `f_ff`(F の FF)，`ang_corr` → `fb_d`(旋回の FB[V])．`ff_l/ff_r` は V_s,ff ∓ V_d,ff．
    `vl_ref/vr_ref` は角速度の補正を含まなくなった(位置の補正だけ)．ログビューアの設定(`tools/get_log.py`，`log_viewer.py`，`plot_groups.m` など)はまだ直していない．
  - 結果: ビルドも実機もまだ(未確認)．
  - 次: ビルドを通し，直進(STRAIGHT)→超信地(PIVOT)→スラローム(SLALOM)→尻当て→探索の順に確かめる．
    モデルは 0.3〜2.0 m/s で求めたので，探索の遅い所と止まる直前の当たり方を見る．超信地は `fb_d` が 0 付近になるよう `PIVOT_VD_FRIC_*` を合わせ直す．

### 2026-11-04 ログの周期と列を目標から見直した(`feature/model-based-control` のまま)

- **はじめは yuho-sysid と同じ 1kHz で取ることを考えたが，目標から考えて 5ms ごとのままにした**(ユーザーの判断)．
  - このブランチの目標は FF + FB で走れるかを確かめること(係数を求めるのではない)．見る動きは並進の時定数 0.57 s，旋回の FB の ω_n = 30 rad/s(周期 約 0.2 s)，モデルにない 30 Hz の揺れなので，5ms ごとで足りる．
  - 1ms が要るのは，ログの電圧をそのままモデルに入れてシミュと重ねるとき(FB の電圧は 1ms ごとに変わる)と，F の立ち上がり(σ/v = 1.5〜10ms)やジャイロの遅れを見るとき．後者は yuho-sysid で済んでいる．
  - 間引きは 5 点の平均ではなく 1 点を拾うだけなので，エンコーダの速度の細かい揺れは減らない．
  - 探索を 1kHz にするなら，34 列(136B/行)では RAM(SDIO の DMA が届く 128KB)に収まるブロックでは SD の待ち 0.6 秒に耐えられず，列を 10 本ほどに絞る必要があった．
- 確かめる軸(FF が合えば `i_s`・`fb_d` が 0 付近，`pwm` が上限に届かない)に対して，列が足りない試験を直した．
  - 直進(`straight_test.c`): `STRAIGHT_TEST_LOG_WALL` を 1 → 0(`trapezoid` の列．FF・`i_s`・`f_ff`・`fb_d` が入る)．
  - スラローム(`slalom_test.c`): `fb_d`・`pwm_l`・`pwm_r`・`vbat` を足した(18 → 22 列．4 秒の記録で 6ms → 8ms ごと)．
  - 探索・最短(`run_log.c`)，超信地，速度のステップは今の列で足りる．並進の FB の P 項は 5.8 ×(`vl_ref`・`vr_ref` の平均 − `vl`・`vr` の平均)で出せる．
  - 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 目標の向きを走りの途中で決め直すのをやめた(`feature/model-based-control` のまま)

- 方針(ユーザー): 尻当て・壁の制御・壁切れのような補正は「甘え」．モデルがきちんとしていれば補正は要らないはずなので，
  ジャイロとエンコーダだけで推定して軌道追従させたい(壁は推定にも使わない)．尻当てもしない方向．軌道追従の進め方は desktop 版の Claude で考える．
  寸法(外形・旋回中心の仮定・車輪・壁センサ・迷路)は `docs/robot_geometry.md` にまとめた．
- その手始めに，**目標の向き(迷路の軸の向き `s_angle_axis_deg`)を今のジャイロの角度に合わせ直す処理をやめた**(ユーザーの依頼)．
  - 前は，尻当てで押し当てている間と制御を止めている間，目標の向きを毎 tick 今の向きに合わせていた．そのため尻当ての後や止めて再開した後は，
    その時の向きが新しい基準になり，それまでの向きのずれを捨てていた(ジャイロの積分角そのものはリセットしていなかった)．
  - 今は，向きの基準を決めるのは `App_ResetGyroAngle()` だけ(ジャイロの積分角と目標の向きを両方 0 にする)．走りの始めに1回呼ぶ．
    止めている間・押し当てている間は目標の向きを進めないだけで，合わせ直さない．
  - 走りの始めに `App_ResetGyroAngle()` を足した: 直進(1本ずつ・往復の始め)，超信地，速度のステップ，パーティー．
    探索・最短(`PrepareStart`)，センサーで回る，スラローム(1本)はもともと呼んでいた．
    スラロームの往復は，毎回呼んでいたのを最初の1本の前だけにした．
  - 尻当て(`SetPosition`)は残っている．押し当てで壁にそろって変わった向きは，有効にし直した後に旋回の FB(±1.5V)が走りの始めの基準へ戻す．
    距離を 0 にし直す処理と，壁の制御・壁切れはそのまま．
  - 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 ブランチと worktree を整理した(`master`)

- `feature/model-based-control` と `feature/maze-sim-package`(`feature/log-viewer-package` を含む)を，未コミットの分をコミットしてから `master` へ `--no-ff` でマージした(ユーザーの依頼．未ビルドのまま)．
  開発日記の衝突は両方を残し，日付の順に並べた．プッシュはしていない．
- worktree `yuho-model` は外した．走行系の続きは yuho 本体で行う．
- 残した worktree: `sim`(`feature/plant-model-fit`．クラッシュしうる迷路シミュレータ)，`yuho-sysid`(`feature/system-identification`．同定のやり方の検討)．
- 気づいたこと(直していない): `tools/matlab/study/` の study1・3・4 は，消した `VELOCITY_FF_*`・`VELOCITY_KP` を `params.h` から読むので，今の master では動かない．
- 続き(ユーザーの依頼):
  - Python のログビューア `tools/log_viewer.py` を消した(MATLAB の配布版 `mouse_log_viewer` に一本化)．
  - plant_sim(クラッシュしうる迷路シミュレータ)を別リポジトリ `M:/User/Desktop/school/club/plant-sim`(ブランチ main，リモートなし)へ分けた．
    worktree `sim` の未コミットの分を `feature/plant-model-fit` にコミット(2ad7aa2)してから，`tools/plant_sim` を一番上に，
    `docs/plant_sim_*.md` と `tools/maze_sim/sim_core.c/h`(そのブランチの版)を写した．ファームウェアは `fw_root.local` か `YUHO_FW` で yuho を指す
    (前は無ければ `../..` を使ったが，止まるようにした)．`build/`(走らせた結果・迷路)と `fw_root.local` も写した．worktree `sim` は外した．
  - 残った worktree は `yuho`(master．モデルで動く機体)と `yuho-sysid`(同定のやり方の検討)．
  - plant_sim は今の master(モデルの FF+FB，`pid.c` などを消した)でビルドできるか，機体のモデルと合うかは確かめていない．

### 2026-11-04 6軸(加速度・ジャイロ X/Y/Z)を読んで探索のログに残す(`feature/six-axis-imu`，`master` から)

- **ICM の加速度とジャイロ X・Y も毎 tick 読み，探索・最短のログ(`run_log`)に残すようにした**(ユーザーの依頼．制御は変えない)．
  - yuho-sysid から写した: `ICM_ReadImu()`(ACCEL_DATA_X1 から 12byte の1回のバースト)と加速度の設定(±8g / ODR 1kHz．`ACCEL_CONFIG0` = 0x26)，
    `logic/state_estimation/imu`(機体の座標 x 前・y 左・z 上の物理量にする)，`params.h` の `ACCEL_SENSITIVITY_LSB_PER_G`・`GRAVITY_M_S2`・`IMU_AXIS_MAP/SIGN`．
  - `App_ControlTick` は `ICM_ReadGyro` の代わりに `ICM_ReadImu` で読み，ジャイロ Z の扱いは今までどおり．
    加速度は重力を含んだまま，ジャイロ X・Y のゼロ点は起動時の `ICM_CalibrateBlocking` の値．
  - ログの列: `acc_x`・`acc_y`・`acc_z`[m/s²]，`gyro_x`・`gyro_y`[dps]．ジャイロ Z は今の `gyro_z`．探索のログは time_s とイベントを入れて 39 列(上限 40)．
  - 残っていること: ACCEL_CONFIG0 の値の意味と，左 = センサーの +x は yuho-sysid でもまだ確かめていない．SPI で読む量が 6 → 12byte になった分の tick の時間は測っていない．
    ログビューアの列の組(`yc.plot_groups`)には入れていない．
  - 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 壁で位置・向きを直す補正をコードごと消した(`feature/remove-corrections`，`feature/six-axis-imu` から)

- 方針(ユーザー): 補正は「甘え」．モデルがきちんとしていれば補正なしで走れるはずなので，ジャイロとエンコーダだけで走らせる．
- **消したもの**(ユーザーの依頼．スイッチで切るのではなくコードごと):
  - 壁の制御: `logic/control/wall_control.c/h`，`App_SetWallControl`，目標の向きのオフセット(`wall_offset_deg`．ログの `wall_ofs` 列も)，`WALL_KP_*` などの定数．
  - 壁切れ補正: `search_run` の `EdgeCorr`・`WaitTargetDistanceEdge`，`App_GetWallEdge`，`WALL_EDGE_POS_*` などの定数．
    壁切れの検出とイベント `LOG_EV_WALL_EDGE` は残した(後でログから推定のずれを測るため．補正には使わない)．
  - スラロームの前壁補正: `WaitSlalomStartV` の待ち方，`Slalom_FrontRefSum`，`SLALOM_FRONT_*`・`FAST_FRONT_ENABLE`．曲がり始めは距離だけで決める．
    最短走行の `RunTrack`(前壁補正のために区画をたどっていたもの)も消し，`RunList_Drive`・`FastStraightTo` を簡単にした．
  - 尻当て: `SetPosition`，`App_SetWallPush`(押し当て中の FB の切り替え)，`SEARCH_SETPOS_*`．
- **代わりの動き**:
  - 走り始め(`StartSequence`，`MazeRun_StartSequence`): 機体は後ろの壁に尾を付けて手で置き，`SEARCH_START_FRONT_MM`(40mm．尻当ての記録の値)進んだ所を区画の真ん中とする．横の位置と向きは置いたまま．
  - 区画の真ん中での 180°(行き止まり・ゴール・スタート，最短走行のゴール): その場で超信地旋回するだけ(`TurnBack()`)．
  - 直進の往復の試験: 最初だけ後ろの壁から真ん中へ進み，端では 180° 回るだけ．スラロームの試験: 1本ずつは後ろの壁から，往復は最初の1本だけ後ろの壁から．
- 残したもの: `WALL_REF_L/R`(`tools/turn_sim.py`・`yc.TurnLog` が読む)，`WALL_EDGE_MIN_WALL_MM`，イベントの番号(WALL_CTRL・WALL_USE・EDGE_CORR・FRONT_TRIG・SETPOS は出さなくなったが，昔のログ用に残す)．
- 気づいたこと(直していない): ログビューアの `yuho_viewer_config.m` は軌道の基準に尻当て(SETPOS)を使う(無ければ最初の STEP)．`yc.plot_groups` に `wall_ofs` が残っている．
  `docs/mode_tree.md` などの尻当ての説明も古いまま．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 超信地旋回はジャイロの角度が合うまで終わりにしない(`feature/remove-corrections` のまま)

- 方針(ユーザー): 一旦，超信地旋回は台形が終わった後もジャイロの角度が目標に合うまで FB で合わせる．
- 合わせ方(ユーザーが選んだ): 台形が終わった後だけ，旋回の FB に角度の差の積分(`PIVOT_SETTLE_KI` = 124 V/(rad·s)，上限 1.0 V)を足す．
  積分がないと，摩擦(≈ 0.9 V)に K_θ·e が勝てない所(e ≈ 6°．計算)で止まる．
- 合った: |θ_ref − θ| ≤ 0.5° かつ |ω| ≤ 5 dps．300 ms で合わなければ諦めて次へ進む．どちらも積分を捨てて `App_IsMotionDone()` を true にし，
  イベント `LOG_EV_PIVOT_SETTLE`(25．a:ok b:err_deg c:ms)の後に `MOTION_DONE` を入れる．新しい指令・`App_SetTargetVelocity`・フェイルセーフで合わせ込みはやめる．
- 変えたファイル: `control_loop.c`(`UpdatePivotSettle`)，`params.h`(`PIVOT_SETTLE_*`)，`log_event.h`，`control_loop.h` のコメント．
- 仮に決めた値: K_I，積分の上限，|ω| ≤ 5 dps(ユーザーが決めたのは 0.5° と 300 ms)．PIVOT のログで合わせ直す．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 探索の 180° の前に，前の壁で位置と向きを合わせる(`feature/front-wall-align`，`feature/remove-corrections` から)

- きっかけ: search_0009 で行き止まりの 180° の超信地旋回の途中に左前の壁へ当たり(回転が −85° で止まった)，合わせ込みは 79° ずれたまま打ち切り，
  次の直進で壁を押して VELOCITY_DIVERGE で止まった．1回目の行き止まり(回れた)と比べて L 399→617，R 353→278(左へずれた)，
  FL+FR 1785→1610(約 7mm 手前)，FL−FR 458→382(約 3° 左を向いた)．ジャイロの上では向きの差は ±1° の中だった．
- 調べたこと: 小回り1回の出口のずれを turn_sim と同じ計算で出すと，今の K・C では横 0.0mm(K が半分〜2倍でも ±4mm)．壁切れの位置は
  走るにつれ ±15mm 動き，search_0008 と同じ値だった(決まったずれ)．ずれが大きく変わった区間に1回目の 180° があるので，
  超信地旋回で位置がずれている見込みが高い(ユーザーの判断)．壁切れ補正を戻すのは，これでずれが消えるか見てから決める．
- 動き(`search_run` の `TurnBack`): 前の壁で合わせる → 横の壁のほうへ 90°(左を先に使う) → その壁で合わせる → 90°．
  前だけなら合わせて 180°，横だけなら 90° → 合わせる → 90°，壁がなければ 180° だけ．最短走行のゴールは壁を持っていないので 180° だけ．
- 合わせ方(`App_StartFrontAlign`): 前後のずれ = (FL+FR − 基準)/25 [mm]，向きのずれ = (FL−FR − 基準)/(−25) [deg] を毎 tick 出し，
  速さ −10×ずれ(上限 100mm/s)と角速度 −10×ずれ(上限 90dps)を目標にする．0.5mm・0.5° の中に 20ms いたら終わり(最長 500ms)．
  合わせた分は目標の距離・目標の向きに入るので，区画の真ん中(`dp->ref_mm`)と迷路の向きの基準が合わせ直される．イベント `LOG_EV_FRONT_ALIGN`(26)．
- 基準の値は仮(和 1785 は search_0009 の1回目の行き止まり，差 458 は 10-09 の SEARCH_SPIN の中央値)．傾きの 25/mm は search_0009，
  −25/deg は SEARCH_SPIN のログ．10-09 の SEARCH_SPIN では和が 981 で今日と大きく違うので，センサーの具合が変わっているかもしれない．
- 結果: ビルドも実機もまだ(未確認)．区画の真ん中に手で置いて，SENSOR のモードで FL・FR を測り直すこと．

### 2026-11-04 目標の軌道を区画の線の上に作り，横のずれを返すようにした(`feature/front-wall-align` のまま)

- 調べたこと(search_0009): 推定の軌道(進んだ距離 + ジャイロ)は区画の真ん中から2回目の行き止まりで横に +38mm ずれていたが，
  目標の軌道(目標の距離 + 目標の向き)からは +7mm だけだった．つまり約 31mm は目標の軌道そのものが外れた分．
  原因は小回りの前後のオフセットの補い(K = 0.0219 と遅れ 7.5mm．1回で前後 +10.7mm・横 −3.8mm 区画の線から外れる)で，
  曲がり終わりの目標の距離から次の境界を決める(つないで作る)ので積もっていた．横の壁センサーのずれの向きも推定と毎回合っていた
  (本物は補いほど滑っていない)．超信地旋回の間は，推定の位置はほとんど動かない(1回目の 180° で 0.2mm)．
- ユーザーの方針: いったん，エンコーダとジャイロに見えているずれを直す．180° は前壁合わせ(前の項)を入れる．
- 軌道生成: `SLALOM_SLIP_K` と `SLALOM_EXTRA_LAG_MM` を 0 にした(コードは残す)．大回りには補いが入っていない．
- 追従: ISR で目標の位置と推定の位置を2次元で積み，目標の向きに対する横のずれ e(左 +)を Δθ = −e / 100mm(上限 ±3°，30dps)にして
  目標の向きに足す．直進している間(目標の角速度 0，目標の速さ > 0，前壁合わせでない)だけ効かせる．超信地旋回の合わせ込みも同じ向きで見る．
  ログに `lat_err` の列を足した(40 列で上限)．`LATERAL_CONTROL_ENABLE 0` で切れる．
- 残ること: 見えないずれ(横滑り，超信地旋回のこすれ，車輪の径)は直らない．スラロームの補いを測り直すのは，センサーのマウントを接着してから．
  ログビューアの列の組(`yc.plot_groups`，`get_log.py`)には `lat_err` を入れていない．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 search_0010 を見て，前壁合わせを切り，超信地旋回の合わせ込みを強くした(`feature/front-wall-align` のまま)

- search_0010(往復の探索．最後まで走れた．フェイルセーフなし):
  - 横のずれの FB: 直進中の `lat_err` は平均 −0.18mm，標準偏差 1.37mm，最大 7.9mm．スラロームの後のずれは約 240mm 進む間に 1/10 になった
    (L = 100mm の計算どおり)．縮んだ後に −0.5〜−0.7mm 残る(積分なし．向きのくせ 0.3° 分)．
  - 前壁合わせ: 合わせた後の目標の向きが迷路の軸から +0.1〜+2.2°，46.30s には +10.9° 狂った(終わりは +5.3°)．
    ジャイロは正しかったのに FL − FR の向きで上書きしていた．マウントが固定されていない・横の壁(R = 578)が FR に映り込むのが見当．
  - 超信地旋回の合わせ込み: 10 回とも 300ms で打ち切り(0.5〜1.3° 残る)．積分が摩擦(約 1V)に届いて回り出した所で時間切れになっていた．
- 変えたこと(ユーザーの依頼): `SEARCH_FRONT_ALIGN_ENABLE` を 0(180° はその場で回るだけ)，`PIVOT_SETTLE_KI` を 124 → 400．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 search_0011・0012 を見て，180° の前に位置だけ合わせるようにした(`feature/front-wall-align` のまま)

- search_0011・0012(v = 500，小回り 400．どちらも往復を最後まで走れた):
  - 超信地旋回の合わせ込み(K_I = 400): 7 回とも合った(141〜266ms，0.2〜0.5°)．0 を越えて逆へ最大 −0.47° 振れたが，振れ続けはしない．
    合わせ終わって積分を捨てた後の 0.3s で，最大 +1.0° まで戻る(摩擦で止まっていた分)．
  - 横のずれ: 直進中の `lat_err` の標準偏差 1.8 / 2.0mm，最大 7.8 / 10.2mm．目標の向きは最後に迷路の軸から −0.01°(前壁合わせを切ったので狂わない)．
  - 180° の前の FL + FR は 1099〜1969 とばらつく(19 回．中央値 1655，±211 ≒ ±8mm)．本当の前後のずれか，マウント・壁のばらつきかは分からない．
- 変えたこと(ユーザーの依頼): 前壁合わせを位置だけにして戻した(`SEARCH_FRONT_ALIGN_ENABLE` 1，`FRONT_ALIGN_ANGLE_ENABLE` 0)．
  180° は前のとおり「前で合わせる → 横の壁へ 90° → 合わせる → 90°」．`FRONT_ALIGN_REF_SUM` を 1785 → 1655(19 回の中央値)．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 180° は 90° + 90° に分け，間で横のずれをエンコーダとジャイロで消すようにした(`feature/front-wall-align` のまま)

- ユーザーの判断: 前壁の基準が仮の値のうちは壁を使わない．位置合わせはエンコーダとジャイロで見えているずれだけにする．
  (search_0011・0012 で 180° の直前に残っていた横のずれは 7 回中 5 回が −0.6〜−0.9mm，2 回が +5.2 / +4.3mm)
- 動き(`TurnBack`): 左へ 90° → `AlongSync` → 左へ 90°．`SEARCH_FRONT_ALIGN_ENABLE` は 0 に戻した．
- `App_SyncAlongError`: ISR で，進んだ距離を「目標の距離 − 2次元の前後のずれ」に置き直す．目標は動かさないので，
  位置のループが機体だけをその分動かす(n mm 進めと指令すると目標も動いてずれは縮まない)．90° 回った後なので，前後のずれ = それまでの横のずれ．
  ずれが 0.3mm より小さくなるか 500ms で次へ．イベント `LOG_EV_ALONG_SYNC`(27．a:前後 b:横)．
- 見えないずれ(超信地旋回でこすれて動いた分など)は直らない．`SEARCH_TURN_BACK_SPLIT 0` でその場の 180° に戻せる．
- 結果: ビルドも実機もまだ(未確認)．
- 追記(ユーザーの依頼): 回る前にも前後のずれを消すようにした．`AlongSync` → 左へ 90° → `AlongSync` → 左へ 90°．
  1回目で前後，2回目でそれまでの横のずれを消す(`ALONG_SYNC` のイベントが 180° ごとに2つ入る)．

### 2026-11-04 search_0013・0014 を見て，位置のループを2次元の前後のずれにし，合わせ込みの行き過ぎを止めた(`feature/front-wall-align` のまま)

- search_0013(90° → 合わせ → 90°)・0014(合わせ → 90° → 合わせ → 90°．直進・小回り 500，加速度 3000): どちらも往復を走れた．
  - 0014 の 12.2s の小回りで，曲がり始めに回れなかった(目標 −400dps でジャイロ −24dps，向きの差 −23.6° まで)．直前の L = 693(左の壁にかなり近い)で，
    目標の角速度 0 なのに左へ 20〜37dps 回っていた → 左の壁か柱にこすったと見ている．そのとき見えていた横のずれは +4mm だけ(見えないずれで寄っていた)．
  - そこで生まれた約 25mm の見えるずれが約 4s 残った: 横は直進中の FB で縮むが，前後は位置のループが道に沿った距離(pos_ref − dist)しか見ず，
    小回りのたびに横と前後が入れ替わって消えなかった(14.26s で2次元の前後 +20.1mm，1次元では −0.0mm)．
  - 超信地旋回の合わせ込み: 0014 で 4 回打ち切り．+1.4° → −1.2° と行き過ぎ，反対側で摩擦に止められていた(0013 は 8 回とも合った)．
  - `ALONG_SYNC`: 見えている横のずれ(最大 10.3mm)を消していた．ただ 0.3mm に入らず，多くが 500ms の打ち切り(0.5s 後で −1.3〜+0.6mm)．
- 変えたこと(ユーザーの依頼):
  - 位置のループは2次元の前後のずれ(目標の位置に対する推定の位置を目標の向きに投影)を見る．`AlongSync` は進んだ距離を置き直すのをやめ，
    ずれを `LOG_EV_ALONG_SYNC` に残して小さくなるのを待つだけにした(`App_SyncAlongError` を消し，`App_GetLateralError` を足した)．
  - 合わせ込みは，角度の差が 0 を越えたら積分を捨てる．
- 残ること: 見えないずれ(壁にこすったもの)は直らない．`ALONG_SYNC_TOL_MM`(0.3mm)は変えていない．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 ジャイロの倍率の試験モード(TEST の GYRO_SCALE)を作った(`feature/gyro-scale-test`，`feature/front-wall-align` から)

- きっかけ: 向きの精度を上げる案(傾きを直す，4kHz の値を全部使う，倍率とゼロ点を詰める)を，ログで見積もった．
  - 傾き: 止まった所の加速度を迷路の4つの向きで平均すると，向きで符号が変わる分(床の傾き，約 1°)と変わらない分(センサーの機体に対する傾き，
    0.4〜0.6°)に分かれた．後者だと向きの倍率の誤差は 1 − cos ≈ 0.004% で効かない．機体は床に垂直な軸で回るので，重力の鉛直に直すとかえってずれる．
  - 倍率: `GYRO_SENSITIVITY_LSB_PER_DPS` は代表値 16.4 のままで，実機で測ったことがない．ここが一番大きいはず(ユーザーの判断でここから)．
- 試験の手順(`app/gyro_scale_test.c`): 区画の真ん中に前の壁へまっすぐ向けて置く → 止まって FL − FR を測る → ±5° 回って 1° あたりの FL − FR を測る
  → 左へ 10 回(3600°)回って測る → 右へ 10 回回って戻って測る．回る前後の FL − FR の差 / 傾きで本当に回った角度を出し，
  左回り・右回りの倍率(本当の角度 / ジャイロの角度)と，それに合う感度を UART に出す．同じ壁を同じ場所で続けて見るので，前壁の基準やマウントが
  合っていなくても差は測れる．回っている間に位置がずれると FL − FR も変わるので，FL + FR の変わり(前後の mm の目安)も出す．
- イベント `LOG_EV_GYRO_SCALE`(28．a:phase b:FL−FR c:FL+FR d:ジャイロの角度)．ログは `gyro_scale/turns_*.bin`．
  モードは列挙と TEST のメニューの最後に足した(前のログの MODE の番号と，メニューの番号は変わらない)．
- 結果: ビルドも実機もまだ(未確認)．
- 追記: 実機で途中で止まり，ログが保存されなかった(ユーザー)．フェイルセーフが働くと `FailSafe_Halt()`(戻らない)へ入る前に SD へ保存していなかった．
  原因(`LOG_EV_FAILSAFE`)を残して SD に保存してから止まるようにした．止まった原因そのものはまだ分からない(20 秒あまり回り続けるので，電池の電圧が下がった見込み)．

### 2026-11-04 センサーのマウントを接着し，壁のしきい値と基準を測り直した(`feature/gyro-scale-test` のまま)

- ユーザーがセンサーのマウントを接着した(それまで固定していなかった．10-09 と 11-04 で前壁の値や FL − FR の傾きが大きく違ったのはこのため)．
- SENSOR_LOG で止めて測った値(sensor/wall_0001〜0005．ばらつきは動いていない所で ±1 以下．置いた所は値から読んだ):
  境界の手前で左右に壁あり L 270 / R 240 / FL 72 / FR 59，壁なし L 2 / R 5 / FL 1 / FR 2，前に壁あり FL 270 / FR 216(L 28 / R 13)，
  区画の真ん中で左右と前に壁あり L 236 / FL 872 / FR 563 / R 268．
- 変えた値(ユーザーの了承): `WALL_TH_L` 180 → 148，`WALL_TH_R` 150 → 126(壁なしの最大と壁ありの最小の真ん中)，
  `WALL_TH_FRONT_SUM` 150 → 175(仮．横の壁の反射 131 に近すぎたため．走りながら読むと本当の壁が止めた値の 4〜6 割に見えたことがあるので，
  止めた値の真ん中 309 にはしなかった．次の探索の STEP_INFO で決め直す)，`WALL_REF_L/R` 312 / 282 → 236 / 268，
  `FRONT_ALIGN_REF_SUM/DIFF` 1655 / 458 → 1435 / 309．
- 残ること: `FRONT_ALIGN_SUM_PER_MM` と `FRONT_ALIGN_DIFF_PER_DEG` は接着前の値のまま．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-04 走りながら読んだ壁の値で，しきい値を決め直した(`feature/gyro-scale-test` のまま)

- search_0021〜0024 の STEP_INFO(地図から走った所を除く 155 回): L は壁なしの最大 38・壁ありの最小 181，ただし search_0024 の
  (4,4)〜(4,2) を南向きに走った所で L = 149〜151 を「壁なし」と判断していた(同じ所を search_0022 では L = 182 で壁ありと読んでいる．
  前のしきい値 180 で走ったログに見える)．機体が右に寄って(R = 470〜494)左の壁が遠かった．R は壁なしの最大 57・壁ありの最小 179．
  FL + FR は前に壁なしの最大 127・壁ありの最小 312(止めた値 487 の 6 割より上)．
- 変えた値(ユーザーの了承): `WALL_TH_L` 148 → 94，`WALL_TH_R` 126 → 118，`WALL_TH_FRONT_SUM` 175 → 220(どれも間の真ん中)．
- 気づいたこと: search_0024(600mm/s)の 9.24〜9.84s の直進は R ≈ 480・L ≈ 150 で右に寄ったままなのに，`lat_err` は 1mm 前後だった
  (エンコーダとジャイロに見えないずれ)．直前に 540dps の小回りを2回している．500mm/s までは小回りの後のずれはほぼ見えていたので，
  600mm/s で滑りが効いてきたのかもしれない(確かめていない)．search_0021・0025 が途中で終わっているのはユーザーが止めたため．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-05 走っている途中で止まっている間に，ジャイロのゼロ点を測り直すようにした(`feature/gyro-scale-test` のまま)

- 調べたこと(接着後の search_0021〜0024．50ms の窓):
  - 止まってからの揺れ: 0〜50ms は gyro_z 5.4dps・acc_y 2.2m/s²，50〜100ms は 1.7dps・0.27，400ms 以降は 0.12dps(90% 点 0.19)・
    acc_x / acc_y 0.02・acc_z 0.04，車輪の速さ 0．100〜400ms は AlongSync や合わせ込みでまだ少し動いている窓が混ざる．
  - エンコーダもジャイロも止まっているのに加速度(主に z)だけ揺れている窓が 212 個中 39 個(ただしその窓でもジャイロの平均は偏っていなかった)．
  - 止まっている所のジャイロの平均は −0.008(中央値)〜−0.019(平均)dps．走る前のゼロ点から 0.01〜0.02dps ずれている(30 秒で 0.3〜0.6°)．
  - 滑り(参考): 小回りの一定の所でジャイロの ω / エンコーダの ω は，右 1.031(400)→1.034〜1.039(500)→1.044(600)，左 1.014〜1.019．
    右だけ速いほど横に滑っている．前後は，止まった所からの加速では滑っていない(±2mm)．500〜600 からの減速では加速度が「エンコーダが
    3.6〜8.3mm 多く数えた」と言うが，機体のピッチ(0〜3°)が acc_x に入る分と，(1,0) の前壁の値と合わないことがあり，決めきれない．
- 作ったもの(ユーザーの依頼): `logic/state_estimation/gyro_bias.c`．50ms の窓で，止まれの指令が続き，エンコーダの変化が左右とも 4 count 以下，
  gyro_z の揺れ < 0.3dps，acc_x・acc_y < 0.1，acc_z < 0.15m/s²，窓の平均が今のゼロ点から 0.5dps 以内なら「静か」．静かな窓が続いたら
  最初の 2 個を捨て，その後の窓の平均へゼロ点を 0.1 ずつ寄せる．`control_loop.c` の ISR から呼ぶ(止まれの指令 = 目標の速さ・角速度が 0 で，
  動き・合わせ込み・前壁合わせ・測り直しが終わっている)．続きが終わったらイベント `LOG_EV_GYRO_BIAS`(29．a:ゼロ点 b:変わり[dps] c:窓の数)．
  `GYRO_BIAS_TRACK_ENABLE 0` で切れる．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-05 ゼロ点の測り直しを入れた後のログ(search_0026・0027，back_0001，fast_0004・0005)を見た(`feature/gyro-scale-test` のまま)

- ゼロ点の測り直しは働いた: search_0027 で 9 回・窓 86 個(4.3 秒ぶん)，1 回の続きの変わりは −0.017〜+0.022dps，途中で ±0.02dps 上下し，
  最後は走る前から −0.0005dps．back_0001・fast_0004 も同じくらい．窓 1 個だけの続き(止まる時間が短い所)もあった．
- 壁のしきい値は余裕がある(45 回): L は壁なしの最大 53・しきい値 94・壁ありの最小 170，R は 44・118・166，FL + FR は 140・220・301．
- search_0026 は LOW_VOLTAGE(6.96V)，fast_0005 は電池 6.57V で VELOCITY_DIVERGE．電池が減っていた．
- 迷路の置き方を変えていた(ユーザー)．search_0027 は 0021〜0024 と道順が違うので，行き止まりの向きは前のログと比べられない．
  気づいたこと: 行き止まりの FL − FR は横の位置でも変わる(横の壁の光が FL・FR に入る)ので，向きの目安にするときは L・R と一緒に見ること．

### 2026-11-05 補正の影響を比べるため，探索で壁の補正を選べるようにした(`feature/correction-compare`，`feature/gyro-scale-test` から)

- ユーザーの依頼: 横壁追従・壁切れ補正・前壁制御を入れた状態で走り，補正の影響を量で比べる．補正の入れ切りは RUN の SEARCH の設定の最後で選ぶ
  (`CORR(1 OFF 2 ON)`．1 つで全部)．補正なしの推定と，補正ごとの量をログに残す(ユーザーが貼った案)．
- 決めたこと(ユーザー): 前壁制御は「小回りの前壁補正」と「180° の前壁合わせ(前後だけ)」の両方．補正ありでも，エンコーダとジャイロで見た
  横のずれの FB は効かせる(横壁追従のずらしと足す)．ログの列は x_mm・vl_ref・vr_ref・f_ff を外し，est_x・est_y・wall_ofs・corr_along を足した(40 列のまま)．
- 戻したもの: `logic/control/wall_control.c/h`(97a83df の版のまま)，ISR の横壁追従(直進の間だけ．ずらしは 90dps まで)と `App_GetWallEdge`，
  探索の壁切れ補正(`EdgeCorr`．境界から境界への直進の間)と小回りの前壁補正(`WaitSlalomStartFront`)．180° は補正ありなら前壁合わせの形
  (`TurnBack`)．地図で分かっている区間をまとめて走る所と最短走行には入れていない．尻当ては戻していない．
- 値の決め直し(今の迷路・接着後・今のしきい値の search_0028〜0030): 壁切れの位置 左 76.3mm(n=9，75〜82)・右 80.5mm(n=12，72〜104)
  → 76 / 80(前は 78 / 83)．小回りの曲がり始めの FL + FR(前に壁あり，n=27)中央値 506(488〜568)，手前 10mm で 5.1/mm → `SLALOM_FRONT_REF_SUM` 506．
  横壁追従のゲイン(`WALL_KP_DEG` 0.03)は接着前のまま．`WALL_REF_L/R` は 236 / 268(区画によって真ん中の値が違う)．
  (前の迷路で壁切れが 40〜155mm とばらついて見えたのは，境界の取り方に曲がった後やまとめて走った区間が混ざっていたため)
- ログ: 列 est_x・est_y(補正を受けない推定の位置．向きは angle)，wall_ofs(横壁追従のずらし)，corr_along(前後の補正の合計．走りごとに 0 から)．
  イベント WALL_CTRL・WALL_USE・EDGE_CORR・FRONT_TRIG(曲がり始めを動かした量)，FRONT_ALIGN の e(合わせて動いた量)，SEARCH_PARAMS の e(補正 1/0)．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-05 補正を入れたログ(search_0032〜0039)を見て，補正を1つずつ選べるようにした(`feature/correction-compare` のまま)

- 補正 ON(0032〜0035，0039)と OFF(0036〜0038)．同じ迷路・直進 600・小回り 500・加速度 3000(0036 だけ小回り 600・加速度 5000)．
  - 補正の量(ON の完走 3 本): 壁切れ補正は左 +6〜+14mm・右 +6〜+12mm(平均)で，1 本に 1〜2 回上限の 20mm に当たる．小回りの前壁補正は
    平均 −7mm(9 回中 8〜9 回がセンサーで決まった)．180° の前壁合わせで動いた量は −24〜+17mm．横壁追従のずらしは平均 −0.2°，最大 3〜5°．
    前後の補正の合計は走りの終わりで −44〜−77mm．`lat_err` は OFF とほぼ同じ(引っ張り合いは小さい)．
  - 壁切れ補正(+)と前壁補正(−)が逆を向く．どちらも補正なしの走りの値で決めたのに，ON では壁切れが毎回約 10mm 遅く出る．
    横壁追従で横の位置が変わると，斜め前を向いた横のセンサーで壁切れが見える所が前後に動くためと見ている(確かめていない)．
  - 行き止まりの FL + FR(合わせる前): (0,0) は ON 1408〜1411 / OFF 1104〜1180 / 手で真ん中 1597，(1,0) は ON 1585〜1647 / OFF 1953〜2033 /
    手で真ん中 1892．補正は区画の真ん中ではなく補正の基準の所へ寄せる．
  - 補正を受けない推定 − 区画の真ん中(ON): 後半の行き止まりで x −9〜−13，y −10〜−18mm(0039 は約 2 倍)．OFF は ±2mm 以内．
  - 0037・0039 は最初の行き止まり (1,1) から FL − FR が 92・51(ほかは 255〜374)．走り始めの置き方がずれていた．
- 途中で記録が切れる走り(0029・0031・0033・0035・0036．直進の最中)は，マイコンのリセットで，原因はハードの不具合(ユーザー)．
- 変えたこと(ユーザーの依頼): 補正を 1 つずつ選べるようにした(`WALL`・`EDGE`・`SLALOM FRONT`・`TURN BACK ALIGN`．どれも 1 OFF 2 ON)．
  `SEARCH_PARAMS` の e は使った補正のビット(1 横壁追従，2 壁切れ，4 小回りの前壁，8 180° の前壁合わせ)．0032〜0039 の e = 1 は全部あり．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-05 search_0040〜0042 を見て，横のずれの FB も選べるようにした(`feature/correction-compare` のまま)

- search_0040(壁切れだけ)・0041 / 0042(横壁追従 + 壁切れ．0042 は 19.6s でリセット): 壁切れ補正の平均は，壁切れだけで +2.0mm(上限 0 回)，
  横壁 + 壁切れで +7.8 / +4.4mm，全部で +12.3 / +11.4mm．同じ所でも，(1,6) 西向き左は全部 +20 / 壁切れだけ +2.6 / 横壁 + 壁切れ +20，
  (2,2) 南向き左は +18 / −1.8 / +18．横壁追従で横の位置が変わると，壁切れが見える所が前後に動く(斜め前を向いた横のセンサー)ことを確かめた．
  壁切れの位置は補正なし(滑りで真ん中から外れた走り)の値で決めていたので，横壁追従と一緒に使うと合わない．
- ユーザーの見立て: 滑りがモデルにないので，補正なしは推定の上では経路が続いているが，実際は横壁追従ありのほうが真ん中を走っている．
  基準は，横壁追従だけで真ん中を走っている走りから取るべき．そのとき横のずれの FB(エンコーダとジャイロ)が横壁追従と引っ張り合わないようにする
  (0041・0042 は直進中の lat_err の標準偏差が 4.2 / 3.8mm と大きかった)．
- 変えたこと(ユーザーの依頼): `App_SetLateralControl` を足し，探索の設定の最後で `LAT FB(1 ON 2 OFF)` を選べるようにした．OFF の間は横のずれを
  返さず，目標の位置の横の成分を推定の位置に合わせ続ける(横のずれをためないので，曲がった後に前後のずれに変わって位置のループに引っ張られない)．
  `SEARCH_PARAMS` の e のビット 16 = 横のずれの FB を切った．
- 結果: ビルドも実機もまだ(未確認)．

### 2026-11-06 前壁の物差しを確かめる試験モード(TEST の FRONT_WALL)を作った(`feature/correction-compare` のまま)

- ユーザーの依頼: 計画「前壁の物差しを確かめるログ(FW1・FW2)」(slip_model の 4 章から)を取るためのモード．
- `app/front_wall_test.c`．最初に 1 FW1(止めて測る) / 2 FW2(走って測る)を選ぶ．
  - FW1: モーターを使わない．クリックごとに 0.5 秒待ってから 0.5 秒(1kHz)，L・FL・FR・R・電池を記録して SD に保存し(`front_wall/fw1_*`)，
    平均(FL，FR，FL+FR，FL−FR，L，R)を UART に出す．すぐ次の点へ進める(送信のボタン待ちはない)．
  - FW2: スタートの後ろの壁に付けて置き，手かざしで，壁を読む所(置いた所から 298mm = 40 + 90 + 180 − 12．探索と同じ約束)を通って止まる．
    横壁追従だけ(横のずれの FB は切る)．a は 600 まで上げて壁を読む所で 400 / 500 / 600(選ぶ)になるよう 3000 で下げる，b は 600 のまま
    壁を読む所の 20mm 先まで通る．どちらも壁を読む所の後に 3000 で止まる．1kHz で 12 列(dist・target・target_acc・vl・vr・FL・FR・L・R と，
    a は gyro_z・angle，b は gyro_y・acc_x)，約 1 秒．壁を読む所の値(目標の距離，エンコーダの道のり，速さ，FL・FR・L・R)を UART に出す．
- ISR の順番: 壁センサー(`Sensor_ReadAll`)→ エンコーダ・ジャイロ・制御(`App_ControlTick`)→ 記録(`Logger_Sample`)．同じ行の値は同じ tick のもの．
- 計画の問いとの関係: 置いた所から壁を読む所まではファームの約束(真ん中まで 40mm)で 298mm．計画の寸法の計算(46.3mm → 304mm)とは 6mm 違う(問い 1 は未解決)．
  センサーの発光あり・なしの生の値は記録できない(差だけ)．
- 結果: ビルドも実機もまだ(未確認)．
- 追記(ユーザーの依頼「もうちょいオートで」): FW1・FW2 を，1 回の手かざしで最後まで自動で走る形に作り直した(手で置き直す形はやめた)．
  - どちらも S の後ろの壁に付けて置く(ユーザーの選択)．距離は置いた所からの目標の距離で数え，計画の d とは d = 186 + (298 − 距離) で対応させる
    (FW1 と FW2 が同じ約束なので，計画の問い 1 の 6mm の差は比べ方に効かない)．
  - FW1: d = 216 → 156 → 216mm を 5mm おき(26 点)．100mm/s で 5mm ずつ動き，300ms 待って 500ms(1kHz)読む．1 点ごとの平均を
    `LOG_EV_FW1_POINT`(55)・`LOG_EV_FW1_POINT2`(56)と UART に出す．記録は全体を間引いて 1 本．
  - FW2: A 400・500・600 と B 600 を各 3 本(12 本)．1 本ごとに 1kHz で記録して保存．止まったら 300mm/s で置いた所の 20mm 手前まで下がり，
    100mm/s で 600ms 後ろの壁に押し当て(位置は保たない)，制御を無効にして置き直す(ユーザーの選択．前の尻当てと同じやり方)．
    押し当ての間も向きの FB は効いたまま(前の尻当てでは切っていた)．
- 結果(ログ fw1_0001・0002，fw2a_400_0001〜0003): FW2 は A 400 の 3 本だけで，A 500 から先のファイルはない(止まった理由は未確認)．
  - FW1(物差し): FL + FR を「置いた所からのエンコーダの道のり s」で表すと，s = −39.5 (ln S)² + 582.1 ln S − 1780.8 で 2 回分 26×2 点の
    残りが標準偏差 0.68mm，最大 1.4mm．1 回目と 2 回目で約 1mm ずれる(置き方の差を含む)．壁を読む所(s = 298)で S ≒ 435，約 5 カウント/mm．
    1 点の中のばらつきは 1 カウントほど(0.2mm)．
  - 止まっている間も横壁追従が効いていて，向きが動いた(FL − FR が行き +27 → +64，帰り +63 → 0．L・R も動く)．同じ s で FL − FR が
    15〜25 違っても FL + FR の残りはほとんど変わらない(相関 −0.17)．向きが少し違っても物差しは使える．
  - 5mm ずつの動きは，止まった所が ±1mm ほど交互にずれる(記録した s を使うので結果には効かない)．
  - FW2(A 400): 3 本とも壁を読む所の FL + FR が 444 で同じ．物差しで読むと，エンコーダより 1.2〜1.8mm 先(減速中は +2〜3mm，
    400 から止まるまでは +0.3mm 前後)．止まった後は物差しとエンコーダが 0.1〜0.4mm で合う．押し当てて置き直した 2・3 本目も止まった所が
    同じ(325.1mm)で，置き直しは 0.3mm ほどでそろう．
  - 走っているときの約 +1.5mm の差の原因(センサーの遅れか，減速の前傾か，車輪の滑りか)は分けていない．
- 追記(FW2 が 1 本目の保存の後に止まる): 保存(直結 LED が 0.5 秒光る)の後に後ろへ下がらず，モーターが切れて，LED が消えたまま
  ボタンも効かなくなった(リセットではない)．戻りの直進が 3 秒で終わらずに打ち切り → `EndControl()` → 最後の `while (1)` に入った形と
  見た様子が合う(UART は見ていないので未確認)．下がらない原因はコードを読んでも分からなかった(後ろへの直進を使うのはこのモードだけ)．
  - ユーザーの選択で，保存を「後ろの壁へ戻ってモーターを止めた後」へ動かした．戻れなかったときも `EndControl()` の後に保存する．
    モーターを回したまま保存するのをやめて，疑わしい所を 1 つ減らす．
- 追記(本当の原因): 保存を後ろへ動かしても A 400 の 1 本目の後で止まり続けた(fw2a_400_0004〜0020 の 17 回)．原因は `Fw2_RunOnce` の
  取りこぼし．A は 1 つ目の直進の終わりがちょうど壁を読む所(298mm)で，目標の距離が届くのと終わりの印が同じ tick に立つ．
  `while (!App_IsMotionDone())` が先に抜けて値を取らず，`rp->got` が false → `ok` が false → 戻りを呼ばずに止まっていた
  (戻りの直進や保存のせいではなかった)．終わった回にも値を取るように直し，止まったときの文を `read point not reached` / `timeout` に分けた．
  保存は戻った後のまま．
  - A 400 の 20 本(1 本ずつ手で置いた)の結果: 壁を読む所の「物差し − エンコーダ」は平均 +0.8mm，標準偏差 1.2mm．止まった後は平均 +0.1mm，
    標準偏差 0.5mm．差(読む所 − 止まった後)は +0.7 ± 1.1mm．走りながら 1 回読むと ±1mm ほど．読む所の速さは 402 ± 6mm/s．
- 結果(FW2 の 12 本，fw2a_400_0021〜0023，fw2a_500・fw2a_600・fw2b_600 の各 0001〜0003): 取りこぼしの直しで 12 本を最後まで自動で走った．
  - 壁を読む所の「物差し − エンコーダ」(前後 5 点の平均): A 400 は +1.9mm，A 500 は +0.5mm，A 600 は −1.1mm，B 600 は −1.7mm．
    同じ走り方の 3 本のばらつきは 0.1〜0.6mm(押し当てて置き直した 2・3 本目も，手で置いた 1 本目とそろう)．
  - s = 270(どれも 570〜600mm/s で走っている所)では −2.3〜−3.4mm．速いほど物差しが後ろを示す．センサーの読みの遅れなら 5ms ほど
    (見積もり．確かめていない)．減速している間は + 側へ寄る(A 400 の減速中は +2mm 前後)．
  - B の gyro_y を積んだ前傾は，壁を読む所で +1.7〜+2.1°(ゼロ点は最初の 100ms の平均．積んだ値なので当てにならない)．
  - まとめ: 物差しは，走り方(速さ・減速)が同じなら 0.3mm ほどでそろう．ただし走り方で −3〜+2mm 片寄る．小回りの手前のように
    走り方が決まっている所なら，その走り方で片寄りを測っておけば引ける．
  - 前の A 400 の 20 本(手で置いた)はばらつきが 1.1mm と大きかった．今回の 3 本(0.3mm)との違いの理由は分からない．
- 追記(ユーザーの依頼「B を足して」): FW2 に B 500(`fw2b_500`)を足した(計 15 本)．最近の探索(search_0037〜0043)の小回りの速さが
  500mm/s なので，小回りの手前と同じ形(一定の速さで通る)の片寄りを測る．B の最高速度を通る速さに合わせた(今までは 600 まで上げていたので，
  そのまま B 500 を足すと壁を読む所の手前で減速していた)．B 600 は今までと同じ．
- 結果(FW2 の 15 本，16:03〜16:04 に取った fw2a_400_0024〜0026，fw2a_500・fw2a_600・fw2b_600 の 0004〜0006，fw2b_500_0001〜0003):
  - B 500(探索の小回りの手前と同じ形): 壁を読む所の「物差し − エンコーダ」は −1.5，+0.4，+1.0mm(平均 0.0mm)．3 本で 1 本あたり
    +1.3mm ずつ増えた．止まった所(エンコーダ 360.4mm でそろう)の FL + FR も 951 → 972 → 996 と増えていて，前壁の近くの傾き(約 17/mm．
    A 600・B 600 の止まった所から見積もり)で 1 本あたり約 1.3mm．押し当てて置き直した所が少しずつ前へずれたと見ると，両方が合う(確かめていない)．
  - 前の 12 本(15:51)と比べると，A 400・500・600 は 1.1〜1.4mm，B 600 は 0.4mm，− 側へずれた．同じ回の 3 本はそろう(A 500 は 3 本とも −0.9)．
    止まった所の FL + FR も A 500 で 705 → 698 と 1% 下がった．回ごとに物差しの読みが変わる(電池の電圧で発光が変わるのかもしれない．
    FW2 では電圧を記録していないので確かめられない)．
  - A 600 は v_max = v_end = 600 なので減速せず，B 600 と同じ形だった(作りの見落とし)．
  - まとめ: 物差しの読みのばらつきは，同じ回・同じ走り方なら 0.3mm．ただし「正解」にしているエンコーダの起点(押し当て)と，回ごとの
    読みの変わり方が，どちらも 1mm ほどある．500mm/s で一定に走っているときの片寄りは 0 ± 1mm ぐらいとしか言えない．
- K・C を測る走りに進む(ユーザーの判断)．探索(RUN → SEARCH)を WALL ON・EDGE OFF・SLALOM FRONT OFF・TURN BACK ALIGN OFF・LAT FB OFF，
  小回り 500mm/s で 3 本走る．ユーザーの依頼でゴールを (7,7)〜(8,8) に戻した(`SEARCH_USE_TEST_GOAL` を 0．MAZE_GOALS の 4 区画)．
- 結果(search_0044，K・C を測る探索の 1 本目．corr=17 = WALL ON・LAT FB OFF，直進 600・小回り 500，ゴール (7,7)〜(8,8)):
  往復を最後まで走った(66.6 秒，小回り 101 回，電圧 7.56〜7.59V)．
  - 小回りの始めの前 60ms で FL + FR が物差しの範囲(330〜610)に入った 82 回について，FW1 の式で「曲がり始めの本当の位置(入った境界から)」を
    出すと −19〜+31mm，標準偏差 12mm(+90° 平均 7.3，−90° 平均 3.0)．1 回の 60ms の中では 1mm ほどでそろう．
  - 推定(est_x・est_y)の横のずれは往復の間に ±35mm まで育つ．ただし 1 つ前の小回りの手前の推定の横のずれと，次の小回りの始めの位置の相関は
    0.2 で，推定のずれでは説明できない．
  - 曲がり始めの FL − FR は +40〜+160(FW の試験では +30 前後)．位置との相関が 0.5 ある．物差しを作った条件から外れている．
  - 12mm のばらつきが本当の前後のずれか，物差しの誤り(壁ごとの反射の違い．前に区画ごとの前壁の値が 1435〜1892 とばらついた件，
    FL − FR の違い)かは，まだ分けられない．
- 結果(search_0045・0047．0046・0048 は走り出す前に止めた空のログ): 3 本(0044・0045・0047)とも同じ道順(小回り 101 回，区画と向きが全部同じ)．
  - 3 本とも前壁が物差しの範囲に入った小回り 75 回で，曲がり始めの本当の位置(物差し)を比べた．同じ小回りの 3 本のばらつき 3.6mm に対して，
    小回りどうしの違いは 10.1mm．小回りごとの値は走りをまたいで再現する．
  - 前の方(0〜22 回目): 平均 +5.5mm，小回りどうし 3.4mm，3 本の間 1.6mm．真ん中(23〜59): 小回りどうし 10.9mm，3 本の間 4.9mm．
    後ろ(60〜100): 12.5mm，3.3mm．走りの後ろほど小回りどうしの違いが大きい．
  - 0047 は 23 回目から，同じ小回りで 0044・0045 と 10mm 以上違う所がある(36 回目 −5.0 / −4.4 / −17.9mm)．同じ壁なので反射の違いでは
    説明できず，本当の位置の違いと見られる．
  - 左(+90°)の平均 +5.6mm，右(−90°)の平均 +2.3mm．
- K・C に向けた解析(search_0044・0045・0047，曲がり始めの本当の位置 245 点): 前後・横のずれを小回りごとにつなげて追うモデルで合わせた．
  小回りごとに一定の滑り(前後 s_a，外向き s_l)を足し，左は (a, l) → (l + s_a, −a − s_l)，右は (−l + s_a, a + s_l)，横は横壁追従が効いた
  距離で exp(−距離/λ) で減る，直進の距離に比例する前後のずれ ε，曲がり始めの基準 r0．
  - 結果: s_a = −2.5 ± 2.3mm，s_l = +1.7 ± 2.0mm(λ = 20mm のとき．λ を変えても大きくは変わらない)．残りのばらつきは 10.1mm で，
    元の 11.6mm からほとんど減らない．このモデルでは小回りどうしの違いを説明できない．
  - 同じ前壁を，探索の 1 回目と既知区間を走る 2 回目で見た小回り(5 組)では，2 回目の値が 1 回目と違い(−6〜+11mm)，その違いが 3 本で
    だいたい再現する(例: (2,4) の壁で 3.2 / − / 1.5 → 14.4 / 11.7 / 23.4)．壁の反射では説明できず，道のりで決まる本当の前後のずれと見られる．
  - 同じ小回りの 3 本の差と FL − FR の差に相関 0.35(0.09mm/カウント)．小回りどうしでは 0.45(0.14mm/カウント)．FL − FR を引いても
    小回りどうしの違いは 10.1 → 9.0mm にしか減らない．
  - まとめ: 探索のログから滑り(K・C)を取り出すのは，ほかの前後のずれ(道のりで決まり，再現する．原因は分からない)が 10mm あって難しい．
    滑り自体は 1 回あたり数 mm 以下と見られる．
- 小回りの滑りを止めて測る試験(TEST の SLIP)を作った(ユーザーの依頼．ブランチ `feature/slip-turn-test`，元は `feature/correction-compare`)．
  探索のログでは滑りが別の前後のずれ(10mm)に埋もれたので，決まった短い走りで，止まってから前壁で読む．
  - 迷路(4×4 の中): T1 = S → (0,1) で右へ小回り → (1,1) の東の端で止まる((2,1) の東の壁の 186mm 手前)．
    T2 = S → (0,1) で右 → (1,1) で左 → (1,2) の北の端で止まる((1,3) の北の壁の 186mm 手前)．止まる所は FW の「壁を読む所」と同じ．
  - 速さ 300・400・500・600 の順に T1・T2 を交互に各 3 本(24 本)，手かざし 1 回．横壁追従は切り，横のずれの FB は入れる．
    止まって 300ms 待ち 500ms 読む(`LOG_EV_SLIP_POINT`(57)・`POINT2`(58)，1 本の始めに `LOG_EV_SLIP_RUN`(59))．
  - 1 本ごとに 300mm/s で同じ道を小回りで戻り，S の真ん中で 180°(90° を 2 回)回って，20mm 下がってから後ろの壁に押し当てる
    (FW2 と同じ．下がる間は横のずれの FB を切る)．戻ってモーターを止めてから保存する．ファイルは slip/slip1_300 など．
  - 電圧が `LONG_LOG_MIN_VBAT_V` より低いか，途中で止まったら，後ろの直結 LED 2 つを点滅させてボタンを待つ(止まったことが分かるように)．
  - 解析は 300 の走りを基準にして，速い走りとの差で滑りを読む(壁の反射や置き方の約束のずれを消す)．
  - 結果: ビルドも実機もまだ(未確認)．
  - 追記: ビルドで TEST の一覧が上限(シフトレジスタの LED で出せる 15 個)を超えた(`_Static_assert`)．SLIP は RUN → TEST の 5 番へ移した．
  - 追記: ビルドで TEST の一覧が上限(シフトレジスタの LED で出せる 15 個)を超えた(`_Static_assert`)．SLIP は RUN → TEST の 5 番へ移した．
- 結果(SLIP の 1 回目．slip1_300 ×6，slip2_300 ×5，slip1_400 ×1): 13 本目(T2 400)の前に電圧が 7.5V を下回って止まった(始め 7.57V)．
  - slip1_300_0001・0002 は止まった所の FL + FR が 95 で，前壁((2,1) の東)がなかった．
  - 推定(est)の止まった所は T1 で (220, −257.6)，T2 で (477.7, −180.2)．どちらも狙いどおりで，向きのずれも 0.3° 以下．
  - 前壁の物差しで読んだ前後のずれ(物差しの s − 298): T1(300)は −6.0，−15.5，−24.3，−6.6mm とばらばら(FL − FR も −2〜+60)．
    T2(300)は +0.3，+0.1，−1.3，−3.0，−0.5mm でそろう．
  - 読み: 押し当てで置き直すと前後(北)はそろうが，横(東西)の位置はそろわない(戻りの小回りと超信地旋回でずれたまま)．
    T1 は 1 回曲がるので，置いた所の横のずれが止まった所の前後のずれになる．T2 は 2 回曲がるので，置いた所の前後(押し当てでそろう)が
    止まった所の前後になる．T1 だけがばらつくのと合う(確かめていない)．
  - 直した(ユーザーの依頼): 戻りの最後を，S の真ん中(南向きで着く)→ 左へ 90°(東向き)→ 西の壁に押し当てる → 向きを 0 にして制御を
    有効にし直し，40mm 前へ(真ん中)→ 左へ 90°(北向き)→ 南の壁に押し当てる，にした．押し当て 2 回で前後と横の両方をそろえる．
- 結果(SLIP の 2 回目，西と南の押し当てあり．300: T1・T2 各 3 本，400: T1・T2 各 2 本．電圧 8.12 → 7.99V): 11 本目でフェイルセーフが働いた(ユーザー談)．
  - T1 300: −13.8，−5.5，−2.0mm．T2 300: −5.7，−5.1，−5.9mm．T1 400: +5.2，+14.8mm．T2 400: −8.8，−13.7mm．
  - 走り始め(押し当てた直後)の L − R が，手で置いた 1 本目の −4 から −55，−60，…，−105，−124 と 1 本ごとに同じ向きへずれた．
    押し当てで向き・横がそろっていない．今の押し当ては向きの FB を効かせたまま押すので，壁に倣う向きを打ち消す(昔の尻当ては
    `App_SetWallPush` で FB を止めていた)と見ている．
  - ユーザーの判断で，自動で戻るのをやめ，1 本ごとに手で S に置き直して手をかざす形にした(戻り・押し当て・超信地旋回のコードは消した)．
    走り終わったらモーターを止めて保存し，次の 1 本の番号・種類・速さを UART に出して手かざしを待つ．
- 結果(SLIP の 3 回目，1 本ずつ手で置いた 24 本．slip1_300_0010〜，slip2_300_0009〜 など．電圧 8.01 → 7.90V):
  - 止まった所の前後のずれ e(物差しの s − 298)の平均(標準偏差): T1 は 300: −8.5(0.5)，400: −9.4(0.8)，500: −16.2(9.3)，600: −9.1(0.9)．
    T2 は 300: −2.1(0.5)，400: −0.7(2.0)，500: −1.5(0.3)，600: −1.7(1.1)．
  - T1 500 の #2(FL − FR +94)と #3(走り始めの L − R +17，FL − FR −42)，T2 400 の #4(L − R +7)は置き方が外れている．
    除くと T1 500 は −10.8，T2 400 は −1.8．
  - 300 との差(600 − 300): T1 −0.6mm，T2 +0.4mm．滑りは 600mm/s でも 1mm 以下．
  - 滑りのモデル(C = 0.02s，探索の小回りの形)を Python で回すと，K = 0.01 で 300 → 600 の前後のずれの差が −2.3mm，横 +3.1mm．
    T1 の差(−0.6 ± 0.6mm)からは K はおよそ 0.005 以下(符号の向きは確かめていない)．C は効きが小さすぎて決められない．
  - まとめ: 600mm/s までは小回りの滑りはほとんどない．`SLALOM_SLIP_K` = 0 のままでよい．探索で見た小回りごとの 10mm の前後のずれは
    滑りではない．
- 探索の 10mm の前後のずれの原因を探した(search_0044・0045・0047，前壁の物差しで読んだ曲がり始めの位置)．
  - 小回りの番号ごとのばらつき: 0〜14 回目 3.2mm，15〜29 回目 7.0，30〜44 回目 10.8，45〜59 回目 13.8，60〜79 回目 12.4，
    80〜100 回目 16.6mm．走るほど育つ．壁で前後を直さない(corr = 17)ので，前後のずれがたまり続ける形．
  - 1 つ前の走り(横壁追従の効いた直進の後か，続けて曲がった後か，超信地旋回の後か)で分けても，向き(東西南北)で分けても説明できない．
    曲がり始めの向きのずれ(ジャイロ − 目標，±4°)や FL − FR とは相関 0.3〜0.44．
  - 補正なし(corr = 0，search_0037・0038)の前の方(25 回目まで)はばらつき 12〜20mm で，横壁追従あり(0044〜0047 は 4〜5mm)より大きい．
  - SLIP の結果(速さで変わる滑りは 1mm 以下)とあわせると，1 回ごとに生まれる小さなずれ(速さによらない分を含む)が，100 回ほど
    積み重なって 10mm 以上になっていると見られる．SLIP は 300 との差を取ったので，速さによらない 1 回ごとのずれは測れていない．
- SLIP に「まっすぐの走り」を足した(ユーザーの依頼)．最初に 1 TURN / 2 STRAIGHT を選ぶ．STRAIGHT は，T1・T2 と同じ止まる所の壁を
  曲がらずに読む．E: (0,1) で東向き(西の外周の壁に付ける)→ 298mm で (1,1) の東の端．N: (1,0) で北向き(南の外周の壁に付ける)→
  478mm で (1,2) の北の端．E を 300・500 で各 3 本，続けて N を同じように(置く所が同じものをまとめた)．ファイルは slip3_300・slip4_300 など．
  T1 − E，T2 − N で，曲がって生まれた前後のずれを速さによらない分も含めて出す．次の 1 本の置く所を UART に出す．
- 結果(SLIP の 4 回目: TURN をもう 1 回(slip1_300_0013〜，slip2_300_0012〜 など．B とする)と STRAIGHT(slip3_*・slip4_*)):
  - E(東へまっすぐ)で前壁が見えたのは 5 本(300 の #2・#3，500 の #1〜#3)で，e = +1.8〜+2.2mm(ばらつき 0.2mm)．
    FW の物差しが別の壁でも 2mm 以内で合う．残りの E 7 本は FL + FR が 37〜48 で，前壁((2,1) の東)がなかった．
  - N(北へまっすぐ)は 8 本とも使えない(FL + FR が 108〜119 か，#1 は 1589 で前壁がすぐ前にあった．#2 は記録に止まった所の値がない)．
    (1,3) の北の壁がないか，(1,2) と (1,3) の間に壁があったと見られる(未確認)．
  - TURN の B は，T1 の止まった所の R が 194〜296(A は 43〜72)で，迷路の壁が A と違う．走り始めの L − R が −10〜+45 と広く，
    T1 B の e は −8.9〜−20.6mm でばらつき，L − R が大きいほど e が負に大きい(置いた横の位置がそのまま出ている)．
  - T1 A(−8.5)と E(+2.0)の差は −10.5mm．ただし T1 は手で真ん中に置いた横の位置，E は外周の壁に付けた位置(約束 40mm．寸法では
    46.3mm)が起点なので，約束の差(約 6mm)が入る．曲がって生まれたずれだけを出すには，置き方の約束が同じ T2 − N が要る．
- 結果(SLIP の 5 回目．壁を「外周 + (2,1) の東」にした後．slip1_300_0016〜，slip2_300_0015〜，slip3_*_0007〜，slip4_300_0006〜・slip4_500_0004〜):
  - T1: 300 −11.0(0.3)，400 −12.9(1.5)，500 −12.1(0.8)，600 −12.8(0.9)mm．600 − 300 = −1.8mm．
  - T2: 300 +1.6(3.9)，400 −1.3(1.7)，500 −0.5(3.2)，600 −1.8(1.4)mm．走り始めの L − R が +20 前後の走り(#15・#17・500 #8)で
    e が +3〜+5 になり，ばらつきが大きい．
  - E・N はまた前壁が読めなかった(FL + FR が E で 47〜49，N で 117〜119)．E の止まった所の L・R(267・169)は T1 の止まった所
    (299・276)と違い，走り始めに右の壁の壁切れ(34mm)がある．(0,1) の南は空いているはずなので，E は (0,1) ではない所(S で東向き？)
    に置かれたと見られる(未確認)．
- 結果(SLIP の STRAIGHT，壁を外周 + (2,1) の東の 1 枚にして置き直した後．slip3_*_0010〜0012，slip4_300_0009〜0011，slip4_500_0007〜0009):
  - E: 300 は −5.1，−5.0，−5.0，500 は −4.5，−4.5，−4.1mm．N: 300 は +1.3，+2.2，+0.9，500 は +1.5，+1.6，+1.4mm．
    どちらもばらつき 0.5mm 以下で，300 と 500 の差も 0.5mm 以下．
  - 前々回の E(+2.0mm)は止まった所の L・R が 277・298 で横に壁があり，今回(34・40)と違う所に置かれていた．使わない．
  - 直前の TURN(20:15)の T1 の止まった所の L・R は 299・276 で，今回の E(34・40)と違う．TURN を取った後に壁を外したと見られる
    (未確認)．横の壁が FL + FR にどれだけ入るか分からないので，T1 − E・T2 − N を出すには，今の迷路で TURN を取り直す必要がある．
  - 参考(迷路が違うまま): T1 300 − E 300 = −11.0 − (−5.0) = −6.0mm，T2 − N はおよそ −2mm．
- 結果(SLIP の TURN を今の迷路(外周 + (2,1) の東)で取り直した．slip1_*_0019〜，slip2_*_0018〜 など．20:37):
  止まった所の L・R が T1 は 34〜38・39〜46，T2 は 32〜39・237〜301 で，E(34・40)・N(33〜35・271〜286)と同じ迷路．
  - T1: 300 −11.9，400 −12.5，500 −12.9，600 −13.4mm(全部で −12.7，ばらつき 1.7)．走り始めの L − R と相関 −0.86
    (−0.138mm/カウント)．L − R = 0 にそろえると約 −13.2mm．
  - T2: 300 −0.4，400 −0.8，500 +0.7，600 +0.3mm(全部で −0.1，ばらつき 1.4)．
  - E −4.8(300・500 の平均)，N +1.5mm．T1 − E ≈ −8.4mm，T2 − N ≈ −1.6mm．
  - 読み(確かめていない): T1 − E = 前後の遅れ s_a + 置き方の約束の差(寸法なら +6.3，ファームの約束どおりなら 0)．
    T2 − N = 1 回目の外向きのずれ s_l + 2 回目の前後の遅れ s_a．約束の差が +6.3 なら s_a ≈ −14.7，s_l ≈ +13．0 なら s_a ≈ −8.4，
    s_l ≈ +6.8．どちらでも，速さによらない「遅れて外へ」のずれが 1 回あたり数 mm〜10mm 以上ある形．
  - ただし，T1 と E では止まった所の横の位置も違う(約束の差と s_l の分)．横の位置で FL + FR が変わるかは確かめていない．
    探索のログの合わせ込み(s_a −2.5，s_l +1.7)とは大きさが合わない．
- 結果(E の横の位置を変えた 11 本．slip3_300_0013〜0018，slip3_500_0013〜0017．500 の #14 は止まった所の値がない):
  - 止まった所の L・R が 24〜47・32〜73 と動いた(置いた横の位置の違い)が，e は −2.8〜−5.5mm(平均 −4.4，ばらつき 0.7)．
    横の位置で物差しはほとんど変わらない(どの走りをどれだけずらしたかは記録がないので，L・R から見た範囲で)．
  - 探索の 3 本で，壁のない所で続けて曲がった小回りを，1 つ前と向きが反対(ジグザグ)か同じかで分けると，+6.9(102 回)と +5.2(70 回)．
    差 1.7mm は外向きのずれ s_l の 2 倍に当たるので，s_l ≈ +0.9 ± 0.9mm．
  - まとめ(確かめていない): T2 − N = s_a + s_l ≈ −1.6mm，探索の合わせ込み(s_a −2.5，s_l +1.7)，ジグザグの差(s_l ≈ +0.9)は，
    「1 回曲がるごとに前後に −2.5〜−4mm 遅れ，外へ +1〜+2mm ずれる(速さによらない)」でそろう．T1 − E(−8.4)は，
    手で S に置いた所が真ん中より西へ 4〜5mm 寄っていた(走り始めの L − R が −5 前後．横の壁の基準は L 236・R 268 で，真ん中なら
    L − R ≈ −32)と見ると合う．置き方の約束はファームの 40mm に近い．
- 入れた(ユーザーの了承): `SLALOM_PRE_ADJ_MM` を −1.5，`SLALOM_POST_ADJ_MM` を +3.0 にした(`slalom.c` の数え方: 前を伸ばすと
  出口の線が外へ，後ろを伸ばすと出口が先へ)．小回り 90°(探索・最短走行・SLIP)の全部に効く．SLIP の TURN を取り直して，
  T2 − N が 0 に近づき，T1 が +3mm ほど動くかを見る．
- 結果(SLIP の TURN，調整あり: pre 12.07 → 10.57，post 11.51 → 14.51(300 のとき)．slip1_*_0022〜，slip2_*_0021〜 など．24 本):
  - T2: 300 +1.2，400 +1.1，500 +1.5，600 +1.6mm(ばらつき 0.1〜0.6)．N(+1.5)との差は −0.2mm ほどで，調整前の −1.6mm から 0 に近づいた．
  - T1: 300 −7.6，400 −8.2，500 −9.9，600 −10.0mm．調整前(−12.7)から +3.8mm 動いた(見込みは +3)．
  - T1 の 600 − 300 は −2.4mm(調整前の回は −0.6，−1.5，−1.8)．速さで増える前後の遅れが 600 で 2mm ほどあるかもしれない．
  - T1 − E は −4.1mm．残りは手で置いた所が西へ寄っている分(走り始めの L − R が −1〜−12)と見ている(未確認)．
- ユーザーの依頼でゴールを試しのゴールに戻した(`SEARCH_USE_TEST_GOAL` を 1．`SEARCH_TEST_GOALS` は (1,0) の 1 区画のまま)．
- 結果(新しい迷路の探索，調整あり，search_0051〜0053．ゴール (1,0)，corr = 17．直進 500・小回り 600(計画の 600・500 と逆)):
  - 3 本とも同じ道順で，小回り 20 回．前壁が 3 本とも読めた 17 回で，曲がり始めの位置は平均 +13.8mm，小回りどうしの違い 5.9mm，
    同じ小回りの 3 本の差 2.0mm．1 回目の小回りから +12mm で，13・14 回目は +24・+25mm．
  - 小回りが 20 回しかなく，番号ごとの育ち方は比べられない．調整なしの同じ迷路の走りと比べる必要がある．
- 調整なしと比べるため，`SLALOM_PRE_ADJ_MM`・`SLALOM_POST_ADJ_MM` を一時的に 0 にした(ユーザーの了承．比べ終わったら −1.5・+3.0 に戻す)．
- 結果(新しい迷路の探索，調整なし: search_0054・0055・0057．0056 は 11 回目の小回りの後，直進の途中でログが切れた(リセットと見られる)):
  - 道順は調整ありと同じ(小回り 20 回)．前壁が読めた 17 回で，調整なし − 調整ありの平均は −1.95mm(ばらつき 4.3)．前半 −3.0，後半 −1.0．
    小回りどうしの違いは調整あり 5.9mm，調整なし 7.8mm．0057 だけ 0054・0055 と 10mm ほど違う小回りがある．
  - 1 回目の小回り(それより前に曲がっていない)は，前のオフセットの違いだけなら +1.5mm になるはずが −3.0mm．回をまたいだ物差しの
    片寄り(FW2 で 1mm ほど)か，置き方の違いが数 mm 入っていると見られる．
  - まとめ: 20 回の小回りと，回をまたいだ比べ方では，調整の効き目ははっきりしない．調整のあり・なしを同じ回の中で交互に走れるようにすると，
    回ごとの片寄りが消える．
- 探索の設定の最後に `SLALOM ADJ(1 ON 2 OFF)` を足した(ユーザーの依頼)．OFF なら `SLALOM_PRE_ADJ_MM`・`SLALOM_POST_ADJ_MM` を足さない
  (`Slalom_SetManualAdjust`．起動時は ON なので，ほかのモードは今までどおり調整あり)．`SEARCH_PARAMS` の e の 32 に残す．
  `params.h` の値は −1.5・+3.0 に戻した．書き込み直さずに，調整のあり・なしを同じ回の中で交互に走って比べるため．
- 結果(SLALOM ADJ を選べるようにした後の探索: ON は search_0058・0060・0061・0062，OFF は search_0059．ROUND，小回り 20 回):
  - 0059 は 22.8 秒，0062 は 21.2 秒で，直進の途中でログが切れた(走行中のリセットと見られる)．
  - 前の回(ON: 0051〜0053，OFF: 0054・0055・0057)と合わせ，ON 7 本・OFF 4 本で小回りごとに比べた(前壁が読めた 17 回):
    OFF − ON の平均は −2.2mm(標準誤差 0.9)．見込み(後ろの +3 と前の −1.5 で，曲がり始めがおよそ −1.5mm)と向き・大きさが合う．
  - 同じ小回りのばらつきは ON 2.3mm，OFF 3.9mm(OFF は 0057 が大きく外れる)．
  - 小回りどうしの違い(6mm ほど)は ON・OFF とも残る．壁ごとの物差しの違いが入るので，これはずれの大きさを表さない．
  - まとめ: 調整は見込みどおりの向きに効いていて，ON の方が走りごとにそろう．調整は ON のまま(−1.5・+3.0)にする．
- 探索の `SLALOM ADJ` の切り替えを消した(ユーザーの依頼．OFF は比べるためだけで，走りとしての良さはない)．`Slalom_SetManualAdjust` も消し，
  調整(−1.5・+3.0)はいつも入る元の形に戻した．`log_event.h` の e のビット 32 の説明は，search_0059 を読むための注として残した．
- SLIP に超信地旋回の走りを足した(ユーザーの依頼)．最初の選択を `SLIP(1 TURN 2 STRAIGHT 3 PIVOT)` にした．PIVOT は T1・P1・T2・P2 を
  300・500 で交互に各 3 本(24 本)．置き方は全部 S．
  - P1: S → (0,1) の真ん中で止まる → 前後を合わせる(探索の AlongSync と同じ)→ 右へ 90° 超信地旋回(探索の 180° の半分と同じ
    角速度・角加速度・待ち)→ 前後を合わせる → (1,1) の東の端で止まって前壁を読む(T1 と同じ所)．
  - P2: P1 と同じく回ってから，(1,1) で左へ小回り → (1,2) の北の端(T2 と同じ所)．
  - P1 − T1 で超信地旋回の前後のずれ，P2 − T2 で横のずれが出る．どれも S に置くので，置き方の約束や手で置いた横の位置は，
    同じ回の中で交互に走れば両方に同じように入る．ファイルは slip5_*・slip6_*．
- 結果(SLIP の PIVOT，24 本．T1: slip1_*_0025〜，T2: slip2_*_0024〜，P1: slip5_*，P2: slip6_*):
  - 止まった所 [A] の L・R が 283〜308・262〜288 で，前回(34・40)と違い横に壁があった(迷路がまた変わった．同じ回の中ではそろう)．
  - T1: 300 −6.6(1.4)，500 −7.2(0.7)．P1: 300 −9.0(0.1)，500 −8.4(0.4)．P1 − T1 ≈ −1.8mm(超信地旋回で前後に遅れる)．
  - T2: 300 +1.6(0.3)，500 +1.2(0.6)．P2: 300 +2.4(0.4)，500 +1.7(0.9)．P2 − T2 ≈ +0.7mm(横のずれ．小回りと同じ数え方で外向き)．
  - 超信地旋回の合わせ込みは全部 ok(残り 0.5° 以下，40〜80ms)．
  - まとめ: 90° の超信地旋回 1 回のずれは前後 2mm・横 1mm 以下で小さい．180°(90° を 2 回)でも数 mm．探索のずれの大きな出どころではない．
- 回る中心のずれの式を立てた(ユーザーの問い)．中心が原点から c = (lx, ly)(機体の座標，x 前・y 左)ずれていて，中心が計画どおり動くと，
  原点の誤差は Δ = (R(θ0) − R(θ_end)) c で，始めと終わりの向きだけで決まる(道・半径・速さによらない)．右 90° で前後 −(lx + ly)，
  外向き lx − ly．左 90° で前後 ly − lx，外向き lx + ly．向きが戻る走り(右 → 左)では 0．β = lκ(l 一定)は c = (l, 0) と同じ動き．
  - 今のデータ: 調整なしの T2 − N は −1.6mm(式では 0)．超信地旋回と調整なしの小回りの差は，同じ 90° でも約 2mm．小回りには回った角度
    だけで決まらない分が 2mm ほどある．超信地旋回の P2 − N ≈ +0.55mm は式(0)と合う．
  - (lx, ly) は決めきれない(P1 と同じ回・同じ迷路の E がない，左 1 回を測っていない)．左右の違いも，1 回ずつは測っていない．
- SLIP に LR の組を足した(ユーザーの依頼)．`SLIP(1 TURN 2 STRAIGHT 3 PIVOT 4 LR)` の 4．速さ 300 で，S に置く T1・P1・T2・P2，
  (2,0) に置く T1L・P1L・T2L・P2L(左右反対)，E，EW((2,1) で西向き，(2,1) の東の壁に付ける)，N を順に 3 回くり返す(33 本)．
  小回りは手で足す調整を引いた形で走る．ファイルは slip7〜slip11．

### 2026-11-06 往復探索でゴールに止まらず，ゴールの4区画を回ってから戻るようにした(`feature/search-goal-tour`，`feature/slip-turn-test` から)
- ユーザーの依頼．往復探索でゴールが (7,7)〜(8,8) の4区画のとき，ゴールの真ん中で止まって 180°(超信地旋回)すると
  ずれが出るので，止まらずにゴールの区画を回ってからスタートへ戻るようにした．
- 作り(logic 層 `search_planner`): 旗 `goal_tour`(既定 false)と段階 `SEARCH_PHASE_GOAL_TOUR`(番号 5．LOG_EV_PHASE に出る)を足した．
  ゴールの区画に入ったとき，`goal_tour` でゴールが2区画以上なら STOP を返さず GOAL_TOUR へ移り，まだ入っていないゴールの区画を
  行き先にして進む(行きと同じコスト)．全部入ったら止まらずに TO_START へ移る．残りの区画へ行けなければ回るのをやめて戻る．
  既知の区間をまとめて走る先読み(`SearchPlanner_KnownRun`)も GOAL_TOUR で使える．
- app 層(`search_run.c`): 往復(ROUND)のときに `goal_tour` を立てる．片道・全面は今までどおり．ゴールが1区画(試しのゴール)なら今までどおり止まる．
  ゴールで止まらないので，ゴールでの LED の点灯と待ちはなくなる．地図を flash に残す条件に GOAL_TOUR も入れた．
- maze_sim は `goal_tour` を立てないので動きは変わらない．
- 確かめていないこと: 実機で走らせていない．回り終わった所からスタートへの経路で，Dijkstra が後ろ向き(180°)を選ぶ形が残るかは迷路による．
- 続き(ユーザーの依頼): ゴールの区画に入ったら一度止まって，行きの地図を flash に残すようにした(帰りで止まっても行きの地図が残るように)．
  - ゴールを回るとき: 最初のゴールの区画の真ん中で止まって残し，そこ(真ん中)から回り始める．次が直進なら真ん中から半区画加速で進む．
    次が曲がりなら，真ん中からなので超信地旋回(90°)になる．ゴールを回らないとき(ゴールが1区画)は，180° 回る前に残す．
  - flash を消している間は CPU が止まり，制御の割り込みも回らない(F405 は flash が1バンク)．モーターが前の出力のまま回らないよう，
    `App_SetOutputPaused`(control_loop)を足し，書く間だけモーターの出力を 0 にする．制御を無効にすると目標の距離・位置が 0 に
    戻るので，無効にはしない．止まっている間は LED を全部点ける．LOG_EV_MAP_SAVED に結果が出る．
  - 確かめていないこと: 消すのにかかる時間(1〜2 秒ほどと見ている)，その後の最初の tick で速さ・ジャイロの値が跳ばないか，
    止めている間に機体が動かないか．
- 続き(ユーザーの依頼): maze_sim(msim.py の画面・まとめて試す，C の CLI)で確かめられるようにした．
  - `yuho_mouse.c`・`maze_sim.c`: 往復(round)のとき `goal_tour` を立てる(機体と同じ)．最初のゴールの区画に入った手は，msim には
    MSIM_GOAL に動きを付けて返す(「reached goal」になり，そのまま進む)．回っている間の区画は帰りに数える．段階の名前に `goal tour`．
  - `sim_core.c`: 区画の数字(SimCore_PlannerValues)を GOAL_TOUR では残りのゴールの区画までのコストにした．
  - `ideal_time.c`: ゴールを回り始める所で，真ん中まで半区画で止まる時間を足し，そこをゴールの時刻にする(flash に書く時間は数えない)．
  - 結果(既定の速さ，dijkstra．前 = ゴールで止まって 180°，後 = ゴールを回る):
    - 乱数 300 迷路: 失敗 0．最適 228 → 226，帰りの区画 61.6 → 70.1，探索 35.0 → 37.7 s，コストの比 1.055 → 1.083．
    - 同梱の全日本 12 迷路: 失敗 0．最適・コストの比は同じ(3，1.152)，探索 59.8 → 60.3 s．
    - 迷路によっては帰りの道が変わって悪くなる(乱数のシード 5: コスト 94 → 231)．回り終わった所と向きから帰り道の探索が始まるため．

### 2026-11-07 maze_sim に既知の区間の加速と，止まる・回る・待つ時間を入れた(`feature/search-goal-tour` のまま)
- ユーザーの依頼．既知の区間の加速，行き止まりの 180°(超信地旋回)，ゴールで止まって動き出す動きは，ハードに依らないので maze_sim に入れる．
  180° とゴールの時間は，運動の時間とは別の値で出す．
- logic 層: `RunList_EstimateTimeEnds`(`run_path`)を足した．列の始めと終わりの速度を渡せる `RunList_EstimateTime`(こちらは 0・0 で呼ぶだけ．
  ファームウェアの動きは変わらない)．
- `ideal_time.c`/`.h`:
  - `SearchTime_KnownRun`: 機体の `TryKnownRun` と同じ条件(境目を走っていて直進を返されたとき．ゴールを回り始める所ではまとめない)で
    `SearchPlanner_KnownRun`・`RunPath_FromKnownRun` を呼び，プランナーの位置を区間の終わりへ進める．時間は理想の円弧の形の列を，
    入りと出を小回りの速さにして数える(先頭の走り済みの半区画は除く)．
  - 止まる・回る・待つ時間 `t_extra_s`: 180°(ずれ合わせ → 90° → ずれ合わせ → 90°)，真ん中からの左右(90°)，ゴールで地図を書く，
    ゴールが1区画の往復のゴール(書く → 180° → 待つ)．超信地旋回は台形と前後の待ち．ずれ合わせは上限(500ms)，flash は 1000ms の見込み(確かめていない)．
- `maze_sim.c`: 既知の区間の残りの区画も本当の迷路で進めて，ぶつからないかとプランナーの位置を確かめる．結果に止まる・回る・待つ時間，
  超信地旋回の数，既知の区画の数を出す．`yuho_mouse.c`: 曲がりを含む既知の区間は msim の何手かに分けて返す．結果に `stop_s` など．
- 結果(乱数 300 迷路，往復): 失敗 0．区画の数・最適の数は前と同じ(既知の区間で経路は変わらない)．既知の区画 13.5，超信地旋回 4.4 回，
  止まる・回る・待つ時間 +7.3 s(ゴールまで +1.8 s)．探索の直進 1000 mm/s では，既知の区間で探索 34.1 → 33.7 s．
  片道・全面も失敗 0．msim.py batch でも CLI と同じ値．
- 続き(ユーザーの依頼): 止まる・回る・待つ時間(flash に書く・超信地旋回・待ち)はやめ，既知の区間の加速だけを残した．`t_extra_s`・
  `pivots`・`stop_s` などの出力を消し，時間は前と同じく運動だけで数える．ゴールを回り始める所で真ん中まで半区画で止まる運動はそのまま．
  結果は乱数 300 迷路で失敗 0，既知の区画 13.5，探索 37.7 s(前と同じ)．msim.py batch も同じ．

### 2026-11-07 全面探索で，最短になりうる経路を全部確かめる形を作り，maze_sim で比べた(`feature/full-search-all-routes`，`feature/search-goal-tour` から)
- ユーザーの依頼．今の全面探索は最短経路の候補を1本だけ持ち，その上の未知の壁を見に行く．最短になりうる経路を全部候補にする形を作って比べる．
- 作り(logic 層 `search_planner`): `full_mode`(既定 `SEARCH_FULL_ONE_ROUTE` で今までどおり)に `SEARCH_FULL_ALL_ROUTES` を足した．
  未知の壁を「ない」とみなした地図で，ゴールまでの Dijkstra と，スタートを目的地にした逆向きの Dijkstra を計算し，
  「スタートから (q,d) まで + 動きのコスト + (n,e) からゴールまで ≤ L(最短のコスト)」になる動きが横切る壁を全部 `full_walls` に印する．
  そのうち未知の壁の両側を行き先にする．印した壁が「ある」と分かったときだけ計算し直す．スタートからのコストは，スタートの区画を出る
  曲がりの分だけ小さく見積もる(候補が増える側なので見落としはない)．作業に MazeSolver をもう1つ(2KB)使う．機体のモードからは選べない．
- maze_sim: CLI `--full one|all`，msim のオプション `full`(キー F)．
- 結果(全面探索，既定の速さ，one → all):
  - 乱数 300: 区画 139.7 → 140.6，探索 45.2 → 45.5 s(最短経路が決まる 35.1 → 35.5 s)，time-opt が best と同じ 293 → 296．
  - 全日本 12: 区画 328.3 → 334.0，探索 109.0 → 111.0 s，time-opt 10.899 → 10.883 s(best 10.854)，best と同じ 10 → 11．
  - 同梱 87(16×16): 区画 311.6 → 319.1，探索 103.9 → 106.5 s，time-opt 11.360 → 11.361 s，best と同じ 77 → 78．
  - 足立法(同梱 87): 探索 108.6 → 110.8 s，time-opt 11.354 → 11.344 s，best と同じ 70 → 79．
  - どれもコストの最短は全部で見つかる(one でも all でも 100%)．all は探索が 0.3〜2.6 s 長く，最短走行の時間はほぼ同じ．
  - MM2025_east_SC は往復でも (1,0) で FAILED(迷路の形の問題で，今回の変更とは別)．

### 2026-11-07 全面探索で，先にゴールへ行ってから全面探索を始めるようにした(`feature/full-search-goal-first`，`feature/full-search-all-routes` から)
- きっかけ: 全面探索は最短経路の候補の壁を確かめた所で終わるので，ゴールに一度も入らないことがある(ユーザーの指摘．2021 九州クラシック，
  2025 中部クラシックなど)．maze_sim で数えると，同梱の大会の迷路 86 のうち 36，乱数 300 のうち 13 でゴールに入っていなかった．
- ゴールに入る時期を作業用の写しで比べた(大会の迷路 86，入らない数 / 最初にゴールに入る時刻 / 探索 / 区画):
  今のまま 36 / 47.7 s / 103.9 s / 311.6，先にゴール 0 / 37.5 s / 103.1 s / 309.3，行き先にゴールを入れる 0 / 57.7 s / 107.0 s / 320.9，
  最後にゴールへ寄る 0 / 70.4 s / 112.7 s / 338.5．先にゴールが一番よい(最初の候補の経路はどうせゴールへ向かうので遠回りにならず，
  最短経路が決まる所が出口に近くなって帰りが短い)．最短経路のコスト・最短走行の時間はどれも同じ．
- 作り(logic 層 `search_planner`): `SearchPlanner_StartFull` は段階を TO_GOAL にして `full_pending` を立てる．ゴールの区画に入ったら
  STOP を返さずに FULL にする(その Step で行き先を計算し，なければすぐ TO_START)．
- app 層(`search_run.c`): 「走ったままゴールに入った」(TO_GOAL から STOP 以外で段階が変わった)ら，真ん中で止まって地図を flash に残してから
  次の指令を行う(往復でゴールを回るときと同じ．ユーザーが前に頼んだ「ゴールで止まって行きの地図を残す」に合わせた．180° は回らない)．
  探索の後に地図を残す条件に FULL も入れた．
- maze_sim: 同じく，走ったままゴールに入った所で半区画で止まる運動を数える．CLI は `reached goal ... starting the full search` を出す．msim はそこで reached goal になる．
- 結果(本物のプランナー): 大会の迷路 86・乱数 300 とも，ゴールに入らないのは 0．大会 86: 探索 103.4 s(ゴールで止まる分 0.3 s 増え)，区画 309.3．
  `--full all` でも 0．往復の結果は変わらない．msim.py batch も CLI と同じ．
- 続き(ユーザーの依頼): 最短になりうる経路を全部確かめる形(`SEARCH_FULL_ALL_ROUTES`，`full_mode`，`full_walls`，作業の MazeSolver 2KB)を
  logic 層と maze_sim(`--full`，オプション `full`)から消した．先にゴールへ行く形で比べても，大会の迷路 86 で探索 103.4 → 105.9 s と長くなり，
  最短走行は 11.362 → 11.352 s とほとんど変わらなかったため．`search_planner.h` に比べた経緯を一言残した．
  消した後も，全面探索(大会 86: 探索 103.4 s，区画 309.3)・往復(乱数 300: 37.7 s)・msim.py batch の結果は前と同じ．
- 2026-11-07: ユーザーの依頼でゴールを (7,7)〜(8,8) に戻した(`SEARCH_USE_TEST_GOAL` を 0)．ブランチは `feature/full-search-goal-first`
  (コミット a8d3694 で，往復探索のゴールを回る形・全面探索を先にゴールへ行く形と，SLIP・FW の試験などがまとめて入った)．
- SLIP の LR の組を，置く所ごとにまとめる順にした(ユーザーの依頼．置き直しが 15 回 → 4 回)．S で T1 P1 T2 P2 を 3 回 →
  (2,0) で T1L P1L T2L P2L を 3 回 → E・EW・N を 3 本ずつ．比べる組(T1 と P1 など)は同じ所で交互に走る．
- SLIP の全部の組で，手かざしを待つ間，シフトレジスタの LED を「置く所の番号 × 3 個」つけるようにした(1 S，2 (2,0)，3 (0,1) 東向き，
  4 (2,1) 西向き，5 (1,0))．置く所が前の 1 本から変わったときは，それと直結の LED 全部を 0.15 秒ごとに点滅させる．UART にも
  `NEW PLACE!` と置く所の番号を出す．
- 結果(SLIP の LR，2026-11-07 10:14 に PC へ．19 本で止まった: S の 12 本と，(2,0) の T1L・P1L・T2L が 2 本ずつ，P2L が 1 本．
  20 本目(T1L の 3 本目)のファイルはない．E・EW・N は走っていない．電圧は 7.97V で足りていた)．小回りは調整を引いた形(pre 12.07，post 11.51)．
  - 右: T1 −5.7(1.5)，P1 −6.7(1.6) → P1 − T1 ≈ −1.0mm．T2 +0.7(1.5)，P2 −0.2(1.6)．
  - 左: T1L −4.1・−10.2，P1L −4.6・−4.6 → P1L − T1L ≈ +2.5mm(T1L の 2 本が 6mm 違い，当てにならない)．T2L −0.5・−0.7，P2L −1.9．
  - 同じ 90° の超信地旋回と調整なしの小回りの差は，右で −1.0mm(ばらつき 1.5 ほど)．回る中心のずれの式(差 0)と合う範囲．
    前に回をまたいで見積もった「約 +2mm」とは合わない(見積もりが回をまたいでいたため)．
  - 向きが戻る走りは，同じ壁([B])の T2 − T2L が +1.3mm，P2 − P2L が +1.7mm．N がないので 0 との比べはできない．
  - 左右の絶対のずれ(lx・ly)は，E・EW・N がないので出せない．
- SLIP に `5 LR REST` を足した(ユーザーの依頼「残りの必要な試験だけを続けるモード」)．(2,0) で T1L・P1L・T2L・P2L・P2L(各 3 本に
  なるよう足りない分)→ E・EW・N を 3 本ずつ，計 14 本．小回りは LR と同じく調整を引いた形．同じ迷路のまま走る．
- 結果(SLIP の LR REST と前回の LR を合わせた．止まった所の横の壁センサーで同じ迷路と確かめた: [A] は T1 と E，[A'] は T1L と EW，
  [B] は T2 と N がそろう)．P1L の 1 本は止まった所の値がない．T1L は 4 本．
  - 平均(mm): T1 −5.7，P1 −6.7，E +0.1 / T1L −7.8，P1L −5.4，EW −1.7 / T2 +0.7，P2 −0.2，T2L −0.4，P2L −2.7，N +0.1．
  - 1 回曲がる走り − まっすぐ: 右 T1 − E −5.8，P1 − E −6.8．左 T1L − EW −6.1，P1L − EW −3.7．
  - 向きが戻る走り − N: T2 +0.7，P2 −0.3，T2L −0.4，P2L −2.7(式では 0)．
  - 回る中心のずれの式に当てはめると(置き方の約束の差 δ と手で置いた横の位置 x0 を，右と左で同じと見て):
    左右で同じ分(lx に当たる)は，超信地 (−6.8 − 3.7)/2 = −5.3，小回り (−5.8 − 6.1)/2 = −6.0 で 1mm 以内で合う．
    ただし lx そのものは「−lx + x0 − δ」としか出ず，δ(40mm と 46.3mm の差)が分からないと決まらない(δ = 6.3 なら lx は 1mm 前後)．
    左右で違う分(ly に当たる)は，超信地 1.5mm，小回り −0.15mm で合わない．P2L − N の −2.7 もこれと同じ向き．
- 訂正(2026-11-07): 上の「δ = 6.3 なら lx は 1mm 前後」は符号が逆だった．道のりを追い直すと，T1 − E = 曲がりの前後のずれ + x0 + δ
  (E は物差しを作ったときと同じ置き方なので約束の差が消え，T1 は置いた横の位置が起点なので δ が残る)．
  - 壁の厚さ 12mm(競技規定，ユーザー談)と，4 輪の真ん中から一番後ろまで 37.66mm(robot_geometry.md)から，お尻を壁に付けたときの
    真ん中までは 46.34mm で，δ = 6.3mm．これだと右の小回りで約 −12mm(− x0)，超信地で約 −13mm，lx は 8〜12mm になり，大きすぎて信じにくい．
  - お尻の一番後ろが本当に 37.66mm か(コネクター・電池・配線などが出ていないか)を，ユーザーに測ってもらう(壁の面から前の車軸まで．
    寸法どおりなら約 51mm)．
- ユーザーが測った(2026-11-07): お尻を壁に付けて置くと，壁の面から 4 輪の真ん中(前後の車軸の真ん中)まで約 41mm．
  基板の後ろの端(37.66mm)より 3.3mm 後ろに何かが出ていて，そこが壁に当たっている．お尻を付けて置いたときの区画の真ん中までは
  84 − 41 = 43mm で，ファームの約束(`SEARCH_START_FRONT_MM` 40mm)と 3mm 違う(機体は思っているより 3mm 後ろにいる)．
  - これで 1 回曲がる走りの前後のずれ(− 置いた横の位置 x0)は，右の小回り −8.8，右の超信地 −9.8，左の小回り −9.1，左の超信地 −6.7mm．
    S の走り始めの L(272〜286)が真ん中の基準(236)より大きく，西寄りに約 6mm 置いていたと見ると(0.14mm/カウント)，右の小回りは約 −3mm．
    x0 の見積もりが確かでないので，絶対のずれはまだ決まらない．
- 最短走行の制御全部乗せのモード(RUN の 5 番目 `FAST_ALL`，`FastAll_Run`)を足した(ユーザーの依頼「最短で，制御全部乗せモード」)．
  ブランチは `feature/fast-run-all-corrections`(`feature/full-search-goal-first` から)．選ぶものと走り方は FAST_RUN と同じで，
  探索で選べる壁の補正を全部入れる(横のずれの FB も入れたまま)．
  - 直進: 境界から境界の間だけ横壁追従と壁切れ補正(中心からの半区画・中心までの半区画では使わない．探索と同じ)．
    最後の境界を過ぎてからの壁切れは使わない．PIVOT の走り方の直進も同じ．
  - 小回り: 前の指令を「曲がり始め − `SLALOM_FRONT_WINDOW_MM`」で終え，そこで前の壁があれば(`WALL_TH_FRONT_SUM`)前壁補正で曲がり始める．
    大回りは前壁補正の基準がないので使わない．`SLALOM_FRONT_REF_SUM` は探索の小回り(search_0028〜0030)で決めた値のまま．
  - ゴールの 180°: 地図で分かっている壁(未知は「ない」)で前壁合わせ．前壁の傾き(`FRONT_ALIGN_*_PER_*`)は接着前の値のまま．
  - ログでは MODE が FAST_ALL(新しい番号)になる．走りのパラメータのイベントには補正の組み合わせは入れていない．
- ユーザーの判断(2026-11-07):
  - `SEARCH_START_FRONT_MM` を 40 → 43 にした(測った 41mm と壁の厚さ 12mm から)．探索・最短走行の走り始め，FW の試験(壁を読む所が
    298 → 301mm)，SLIP の境界(`SLIP_B0_MM` 130 → 133mm)に効く．これより前の FW・SLIP のログと比べるときは止まる所が 3mm 違う．
  - 超信地旋回だけ左右でずれが違う件(1.5mm ほど)は，小さいのでこのままにする．
  - 小回りの調整(前 −1.5・後ろ +3.0)が正しいか(曲がりのずれがたまるか)は，最短走行で確かめる．
- 最短走行の ADJ CHECK を作った(ユーザーの判断「最短走行で確かめる」．ブランチ `feature/fast-turn-check`，元は `feature/full-search-goal-first`)．
  - FAST(制御全部乗せでないとき)の設定の最後に `ADJ CHECK(1 OFF 2 ON)`．ON で走り方が SMALL のとき，1 本目は小回りの調整あり，
    2 本目はなし，… と交互に走る(`Slalom_SetManualAdjust` を戻し，1 本ごとに `ComputeFastTurns` で前後のオフセットを計算し直す)．
    待つ間のシフトレジスタの LED: あり 1〜4 番，なし 13〜16 番．
  - ゴールで 180° 回らずに，ゴールの真ん中から 200mm/s で次の境界の 12mm 手前まで進んで止まり，300ms 待って 0.5 秒前壁を読む
    (`LOG_EV_FAST_CHECK`(60)・`FAST_CHECK2`(61)と UART)．ゴールの先に 1 区画あけて壁があること．
  - 比べ相手として SLIP に `6 REF NG` を足した((2,0) で北向きに置き，まっすぐ (2,2) の北の端まで．N と同じ動き．ファイルは slip12_*)．
  - 最後が北向きなら，南北の位置は S の南の壁に付けて置いた所で決まるので，手で置いた横の位置は効かない．回る中心のずれの式が
    正しければ調整なしで NG と同じ値，1 回ごとにずれがたまるなら調整ありの方が NG に近い．
- 走行中のリセット(探索・最短走行のログが途中で切れる)の原因を調べた(ユーザーの依頼)．ログは SD の 1 ブロック(157 行 = 0.785 s)の
  切れ目で終わり，書けずに捨てた行は 0 で，SD は詰まっていなかった．search_0067・0068 の起動の `reset_flags` は 4(NRST ピンだけ)で，
  ソフトのリセット(SOFT)・ウォッチドッグ・電源の低下(BOR・POR)ではない．`yuho.net` の NRST は U1・SW3・J2/J4 の 6 番だけで，
  コンデンサーがなかった．vbat の最小は 7.24 V で，LiPo に保護回路もない．NRST に乗ったノイズと見て，NRST と GND の間に
  コンデンサーを付けたら，途中でリセットしなくなった(ユーザー談)．
- 速い最短走行で壁切れ補正が + に大きくなる原因を調べた(ユーザーの依頼．fast_0011〜0015(FAST_ALL)と search_0069〜0073 の EDGE_CORR 148 回)．
  - 壁切れを見た瞬間の前後のずれ(目標に対する機体の遅れ)は平均 0 前後で，原因ではない．センサーは制御の 1 回(1ms)ごとに読むので，遅れは 1.5mm 以下．
  - 同じ直線の 2 回目以降の壁切れは平均 +0.4mm(n=30)で，`WALL_EDGE_POS_L/R_MM` は合っている．
  - ずれは曲がった後の最初の壁切れに出る(平均 +8.8mm，n=118)．直前の動きごとに: 走り始め +4.1(n=8．お尻を付けたときの
    3mm の約束の差と合う)，超信地旋回 +6.2(n=5)，小回り 400・500・600mm/s で +7.4・+7.4・+8.5(速さによらない)，
    700mm/s で +16.4(n=11 のうち 6 回が上限の +20)．
  - 加速中かどうかの差は小さい(600 の小回りの後は，探索と最短でほぼ同じ)．右 +9.2，左 +5.2 で右のほうが大きい．
  - つまり，小回りを 1 回すると機体は思っているより約 8mm 後ろにいる(SLIP の T1 − E の −5.8 / −6.1mm と同じ向き・同じくらい)．
    700 では滑りでさらに約 8mm 増える．壁切れ補正は本当のずれを測っていて，速いと上限(20mm)で足りなくなっている．
- 今あるログで「曲がりのずれがたまるか」を確かめた(2026-11-07)．たまる形(1 回ごとに前後 −3mm，外へ +1.5mm)なら，向きが戻る 2 回の
  走り(T2・T2L)は N より約 −1.5mm，たまらない形(回る中心のずれ)なら 0．
  - 同じ回の中: T2L(調整なし．LR REST の 1 本)− N = −0.1mm．たまらない形に合う(1 本だけ)．
  - 回をまたぐ: T2 − N +0.7，T2L − N −0.4，11-06 の T2 − N −1.6．回をまたぐと 1〜2mm 動くので見分けに使えない．
  - 超信地を含む P2L − N(同じ回)は −3.0・−3.1mm(超信地の左右の違い)．
  - 探索の調整あり・なし(search_0051〜0062)は，調整で計算どおり位置が動くことしか分からない．
- 次は 16×16 の迷路で確かめる(ユーザーの判断)．案: ゴール (7,7)〜(8,8) に南から北向きで入る迷路で，壁を 1 枚動かして曲がる回数の
  違う 2 つの道を作り，ADJ CHECK(調整あり・なし交互)で比べる．調整なしで 2 つの道が同じ値ならたまらない形．
- 16×16 の確かめの迷路を決めた(ユーザー: 2025 学生のクラシック，MM2025_students_CM)．maze_sim(探索は往復)で:
  - そのまま: 最短走行(小回りだけ)は小回り 39 回(右 19・左 20)で，ゴール (8,8) に西向きで入る．その先は (7,8)(ゴール)で，
    その西が壁なので ADJ CHECK の止まる所が使える．
  - (12,14) の東の壁を 1 枚抜く: 小回り 33 回(右 18・左 15)で，同じく (8,8) に西向きで入る．
  - 1 回ごとにずれがたまる形(前後 −3mm・外へ +1.5mm)で止まった所の前後のずれを足し合わせると，そのまま +7.5mm，抜いた後 −3.0mm で
    差 −10.5mm．たまらない形(回る中心のずれ)なら，最後の向きが同じなので差は 0．
  - 壁を 1 枚ずつ変える全部の場合を maze_sim で調べ，同じ止まる所で差が一番大きいものを選んだ．
- 11-07 の search_0063〜0080 と fast_0011〜0029(FAST_ALL)を見た．全面探索は補正を全部入れた形(corr = 31)で，0073・0075・0079 は
  小回り 208 回(約 106 秒)で全面を走りきった．補正が入っていて調整なしの走りもないので，曲がりのずれがたまるかは決められない．
  - 実機の最短走行(小回りだけ)は 31 回(右 15・左 16)か 35 回(右 17・左 18)で，どちらも最後は 6.5 区画の直進で西向きにゴールへ入る．
    maze_sim(往復の探索の後，39 回)と道が違う．たまる形の見込みは 31 回の道 +7.5mm，35 回の道 +10.5mm で差は 3mm しかない．
  - 小回りの前壁補正で動かした量の平均は，小回りの速さ 700 で +0.8〜+3.3mm，500・600 で −5.2〜−6.1mm．速いほど + 側
    (FW2 の「速いほど前壁の値が後ろを示す」と同じ向き)．
  - 壁切れ補正の平均は +7〜+15mm で，まだ + 側に片寄る．
- search_0081・0082，fast_0030〜0035 を見た(ADJ CHECK の記録はない．FAST_ALL か，ADJ CHECK を選んでいない FAST)．
  - search_0082(全面探索，corr = 31)は小回り 102 回の後，105 秒でフェイルセーフ(速度の偏差，電圧 6.96V)．fast_0033・0035 は
    走り出して 0.6 秒でフェイルセーフ(速度の偏差，電圧 6.98・7.23V)．電池が下がって加速に追いつかなかったと見られる．
  - 最短走行の小回りは 17・21 回で，前の 31・35 回と道が違う(全面探索の地図が変わった)．
  - 小回り 700 の前壁補正の量の平均は +0.7・+4.6・+0.1mm(前回の 700 と同じくらい)．
- ADJ CHECK の最初の 4 本(fast_0045〜0048．最短走行，600 / 小回り 500，小回り 17 回 RRLLRRRLRRLLLRLRR，最後は西向き．道のり約 12m):
  - 物差しで読んだ止まった所(e): 調整あり −7.7・−41.3mm，なし −34.1・−28.4mm．平均はあり −24.5，なし −31.3 で，差 +6.8mm．
    同じ設定の 2 本で 34mm も違い，ばらつきが大きすぎて決められない．FL + FR が 305〜404 で，物差しの範囲(330〜610)の外の走りもある．
  - 走り始めの L − R が −42〜+32 と広い．ゴールは S から東へ約 1.4m・北へ約 1.4m なので，置いたときの向きが 0.5° 違うだけで，
    止まった所が 10mm 以上動く．ばらつきの大部分は置いたときの向き(と横の位置)と見られる(確かめていない)．
  - 全部が −8〜−41mm(平均 −28)で，止まる所より手前に止まっている．12m の道のりでエンコーダの倍率が 0.2% ほど違うと，この大きさになる．
  - この道(RRLLRRRLRRLLLRLRR)では，1 回ごとにずれがたまる形で足し合わせても，止まった所の前後のずれは 0mm になる(曲がりのずれが
    最後の向きの前後で打ち消し合う)．なので，この道では調整のあり・なしの差がもともと出ない．見分けるには，見込みの大きい道が要る．
- ADJ CHECK のときに，直進の境界から境界の間だけ横壁追従を入れるようにした(ユーザーの依頼．置いたときの向きのずれを消すため．
  横のずれの FB は切る．壁切れ・前壁補正は使わない)．
  - 実機の地図(fast_0048 の MAP_CELLS)を maze_sim に入れると，小回りの道が 21 回で，実機の 17 回と合わなかった(maze_sim は探索し直す
    ので，分かっている壁が少し違う)．maze_sim で選んだ壁の変え方が実機で同じ道になる保証はない．
  - 横壁追従を入れると，横のずれが直進で消え前後のずれだけが残るので，回る中心のずれの形でも前後の遅れがたまる．2 つの形の見分けには
    使えない．ユーザーの判断で，本番に近い形(横壁追従あり)で，調整がどれだけ位置を動かすか・置いたときのばらつきが減るかを見る(案 2)．
    600 / 小回り 500，探索はし直さず，調整あり・なし交互に 10 本．
- 結果(ADJ CHECK，横壁追従あり: fast_0052(調整あり)・0053(なし)・0054(あり)．600 / 小回り 500，小回り 17 回，同じ道)．
  横壁追従は直進 9 か所で入った(WALL_CTRL on=1 が 9 回)．
  - 止まった所の e: あり −39.2・−34.0mm，なし −12.8mm．
  - 調整ありの 2 本は，止まった所で 5mm，途中の小回りの曲がり始め(前壁で読んだ値)でも 2mm ほどでそろう．横壁追従なしの前の 4 本
    (同じ設定の 2 本で 34mm 違った)より，ずっとそろう．置いたときの向きのずれが，横壁追従で消えたと見られる．
  - 調整なしは 1 本だけで，あり との差は止まった所で +24mm．調整の計算(この指令の列で，直進で横のずれが消えるとして)では，あり の方が
    3mm 先に止まるはずで，向きも大きさも合わない．0053 は止まった所の FL − FR が +49 で，ほかの走り(+3〜+14)と違う．なし の走りを増やして
    確かめる必要がある．
- 結果(ADJ CHECK，横壁追従あり，9 本: fast_0052〜0060．調整あり 6 本・なし 3 本．小回り 17 回の同じ道)．
  - 止まった所の e: あり 平均 −25.9mm(ばらつき 12)，なし 平均 −30.1mm(15)．ばらつきが大きい．
  - ただし e は止まった所の FL − FR とほぼ一直線に並ぶ(相関 0.99．FL − FR が 1 増えると e が約 0.55mm 増える)．最後の 6.5 区画の直進で
    ゴールの周りに壁が少なく横壁追従が効かず，止まった所の横の位置か向きが走りごとに違うためと見られる(物差しのせいか本当の位置かは
    分けられない)．FL − FR で直すと，あり − なし は約 +1mm(調整の計算では +3mm)．
  - 途中の小回りの曲がり始め(15 か所)では，あり − なし が −0.2〜+7.0mm(平均 +2.2)．調整の計算(直進で横のずれが消えるとして)は
    −1.5〜+3.0mm(平均 +1.3)で，向き・大きさはおおむね合う．調整は計算どおりに位置を動かしている．
  - 予定どおり，この形では調整が正しいか(どちらが正解に近いか)は決められない．
- ユーザーの判断: 小回りの調整は今のまま(前 −1.5・後ろ +3.0)で使う．ADJ CHECK を消した(FAST の選択，ゴールで前壁を読む動き，
  `Slalom_SetManualAdjust`)．`LOG_EV_FAST_CHECK`・`FAST_CHECK2`(60・61)は fast_0045〜0060 を読むために残した．
  SLIP の `6 REF NG`(ADJ CHECK の比べ相手として作った)は残してある．

### 2026-11-07 斜め走行を logic と maze_sim に入れた(`feature/diagonal-run`，`master` から)
- ユーザーの依頼「今のモデルから斜め走行を実装しよう」．始める前に確かめた決めごと(ユーザーの選択):
  - 経路は今の実機と同じく，区画の経路(`Dijkstra_BuildRoute`)を置き換えて作る(走行時間の Dijkstra に斜めのノードを足すのは，
    ノードが約 3.7 倍で配列だけで約 80KB になり，今の RAM の空き約 50KB に入らないのでやめた)．
  - 斜めの入り・出は区画の境目から始まり・境目で終わる(小回りと同じ側．区画の中心からにする案もあった)．
  - 今回は logic と maze_sim まで．実機の走らせ方は次の段階．
  - 今の変更(180° の左右交互など)は先に `feature/turnback-alternate` にコミットし(85a2f1a)，ローカルの `master`(origin より 8 コミット先)から
    ブランチを作った．
- 斜めの決めごと(`run_path.h` の表):
  - 斜めの経路は区画の境目の真ん中だけを通る(区画の中心も柱も通らない)．斜めの点は「区画に向き d で入る境目を，d から 45° 傾いて通る所」．
    斜めの直進 `RUN_DIAG` の1個は 1区画/√2 ≈ 127mm で，区画を1つ斜めに横切る(区画ごとの曲がり方では R L R L … のジグザグの1つ)．
  - 指令(右の場合)と区画ごとの曲がり方: `D45IN` = 0 R(境目 → 前 1.5・横 0.5 区画の斜めの点)，`D135IN` = R R(境目 → 前 0・横 1 区画)，
    `D45OUT` = R 0，`D135OUT` = R R，`DV90` = (R) 0 (R)(斜めの点 → 斜めの点．角は 0 の区画の出ない側の境目)．
    角までの折れ線は 45°: 180 + 127mm，135°: 180 + 255mm，V90: 127 + 127mm．135° は回れる半径が 75mm まで．
  - 同じ向きが 2 つ続く所(L R R L)の V90 は角が通り抜ける境目になり，柱の上を通るので使わない(計算すると柱まで約 0.15 区画)．
  - 番号は今の `RUN_*`(0〜7)を変えずに後ろへ足した(maze_sim の画面が `MSIM_RUN_*` として同じ番号を使うため)．
- 作ったもの:
  - `run_path`: 指令の種類，折れ線の長さ(`RunType_DiagLegs`)，向きの変化(`RunType_TurnDeg`)，区画の移り方，ジグザグの続きの向き．
    `RunPath_FromRouteEx(…, use_diag, …)`: 区画の経路の後ろから「斜めのまま進んで出で境目に戻れるか」を表にし(`s_dcont`)，前から
    斜め → 大回り → 小回りの順に置き換える．斜めの中では斜めのまま進める限り進む(ジグザグ → V90 → 出)．加速度が足りなければ，
    大回りと同じようにその斜めの区間をやめて作り直す．`RunPath_FromRoute` は `use_diag = false` で今までと同じ．
    `RunList_ToRoute` も斜めを戻せるようにした．時間の見積もりは斜めの直進を 127mm 単位，最高速度 `vmax_diag` で数える．
  - 形: 理想の形(`RunProfile_Default`)は折れ線の短い方に収まる一番大きな円弧で，長い方の残りをオフセットにする(速さ `RUN_V_D45/D135/DV90`)．
    実機の形(`RunProfile_ForSpeeds`)は `FAST_D45/D135/DV90_*` のスラロームで，前後のオフセットは新しい `Slalom_TurnOffsets`
    (どの角度でも，折れ線の終わりにちょうど着くように決める．90° なら `Slalom_Turn90Offsets` と同じ)．
    仮の値: 45° 600mm/s・240dps・3000dps²(オフセット 97 / 43mm)，135° 400・400・5000(24 / 98mm)，V90 500・300・4000(13 / 11mm)．
    横加速度は 0.26〜0.28G(大回りと同じくらい)．
  - maze_sim の CLI(`maze_sim.c`): 見つけた経路を大回り + 斜めに置き換えた時間と，その経路を本当の迷路で走らせてぶつからないかの確かめを足した．
  - `slalom.h` の最後の `#endif` が前の行とつながっていた(master にも入っている誤り．turnback のブランチと同じ直し)．
- 確かめたこと(PC．gcc の `-Wall -Wextra` で警告なし):
  - 曲がり方の並びを手で与えたテスト(scratchpad)で，45° 入り → ジグザグ → 45° 出，135° 入り → 135° 出，135° → V90 → 135°，
    柱を回る形(斜めにしない)などを置き換え，逆変換で元の区画の経路に戻ることを確かめた．
  - 乱数の迷路 200 枚: ぶつかった迷路 0．最短走行の時間 大回りまで 6.650 秒 → 斜めあり 6.514 秒(斜めの区間 平均 0.5 個)．
  - 大会のクラシック 41 枚: ぶつかった迷路 0．11.959 → 11.590 秒(斜めの区間 平均 1.0 個)．
  - 斜めが少ないのは，区画の経路を旋回のコストで選ぶのでジグザグを避けることと，理想の形では速さの違う旋回が直接つながると
    (オフセット 0)速さを変えられずに斜めをやめるため．
- 確かめていないこと: ファームウェアのビルド(`params.h`・`slalom`・`run_path` が変わった．`search_run.c` は触っていない．
  実機の最短走行は今までどおり `RunPath_FromRoute` で斜めを使わない)．
- 次: 実機の走らせ方(`RunList_Drive` で 45°・135°・V90 のスラロームと斜めの直進．制御の目標の位置はもともと 2 次元で積んでいるので，
  向きが 45° でも追従できるはず)と FAST のメニュー．maze_sim の画面に斜めを描くには maze-sim の `MSIM_RUN_*` と描き方を足す．
- 11章(残っている課題)に「斜め走行」を足した(前は斜めの項目はなかった)．

### 2026-11-07 maze-sim(Python の公開版)を斜めに対応させた(`feature/diagonal-run` のまま)
- ユーザーの依頼「maze-sim(python)を更新して」．別リポジトリ `M:/User/Desktop/school/club/maze-sim`(main．コミットはしていない)．
  - `include/msim.h`: `MSIM_RUN_DIAG`(8)〜`MSIM_RUN_DV90_L`(18)を足した(yuho の `RunType` と同じ番号)．使わなくてよい指令なので
    `MSIM_API_VERSION` は 1 のまま．`mouse.py` に番号と名前，`cli.py` に斜めの直進の表示．
  - `static/motion.js`: 向きを 45° 単位で持つようにし，斜めの直進(1個 = S/√2)と斜めの旋回の表示用の固定の形
    (角までの折れ線の短い方に収まる一番大きな円弧 + 長い方の残りの直線．yuho の `RunProfile_Default` と同じ形)を足した．
    `app.js` で色を分けた(斜めの直進は緑，斜めの旋回は桃)．`README.md` と `docs/api.md` に説明を足した．
  - `examples/yuho/` を今の yuho のつなぎ(`yuho_mouse.c`・`ideal_time.c/.h`)で写し直した(10-21 から写していなかった分も入る)．
- yuho 側: `tools/maze_sim/yuho_mouse.c` の最短走行の経路を 3 つにした(3 つめ「cost + large + diag」= `RunPath_FromRouteEx` で大回りと斜め)．
- 確かめたこと: maze-sim のテスト(`tests/run_tests.py`，yuho を足して)は全部通った(yuho は 3 つの経路，同梱の迷路 87 枚，画面に JS のエラーなし)．
  学生クラシック 2021 の斜めの経路(D45IN・DIAG・D45OUT・DV90・D135IN・D135OUT を含む 34 指令)を，画面なしの Chrome で `motion.js` に通し，
  指令ごとの終わりが区画の格子の点(斜めの点は境目の真ん中で 45° の向き)に乗ること，向きが前後でつながること，
  ゴールの中心で終わること，再生の時間が機体の見積もり(10.882 秒)と合うことを確かめ，形を絵でも見た．

### 2026-11-07 斜めのコストを入れ，maze_sim で最短走行の曲がり方(超信地・小回り・大回り・斜め)を選べるようにした(`feature/diagonal-run` のまま)
- ユーザーの依頼「斜めのコストを 2 にして見て．ターンは超信地・小回り・大回り・斜めで選べるようにしたい」．
  確かめた決めごと(ユーザーの選択): 斜めのコストは「ジグザグを続ける曲がり(前の区画と逆の向きへ曲がる)の上乗せを turn90 の 7 の代わりに 2」
  (1区画は直進の 1 と合わせて 3)．曲がり方を選ぶのは maze_sim だけ(実機の FAST のメニューは変えない)．
- 11章の「斜め走行」を直した(前: 「maze_sim の画面(別リポジトリ maze-sim の `MSIM_RUN_*`)…は未着手」．
  後: 画面は済んだので消し，「置き換えを時間で比べて斜めにするかを決める形にしたい」を足した)．
- 作ったもの:
  - `logic/maze/diag_route`: 区画の経路の Dijkstra に「入るときの曲がり(なし・右・左)」をノードに足したもの(ノード 3 倍)．
    ジグザグを続ける曲がりだけ `diag_turn` を足す(0 なら今の Dijkstra と同じコスト)．探索で使う `dijkstra.c` は変えていない．
    作業領域(約 22KB)は呼ぶ側の構造体に持つので，ファームウェアが使わなければ RAM は増えない．`params.h` に `MAZE_COST_DIAG_TURN`(2)．
  - `run_path`: 超信地旋回の指令 `RUN_PIVOT90_R/L`(19・20．中心で止まって 90° 回る．速さ 0，曲がる時間は数えない)と，
    区画の経路を「中心から中心への直進 + 超信地旋回」にする `RunPath_FromRoutePivot`．`RunList_ToRoute` も超信地旋回を戻せる．
  - `yuho_mouse.c`: オプション `turn`(pivot|small|large|diag，K キー，既定 diag)と `diag_cost`(既定 2)．最短走行の 2 つめの経路
    「cost + …」をその曲がり方で作る(探索はやり直さない)．前に足した 3 つめの経路はやめて 2 つに戻した．
  - CLI(`maze_sim.c`): `--diag-cost N`．超信地旋回の時間も出す．斜めの時間は斜めのコストで選び直した経路で数える．
  - maze-sim(別リポジトリ，未コミット): `MSIM_RUN_PIVOT90_R/L` と，`motion.js` の長さ 0 の超信地旋回．
    ERROR の後に途中で変えられるオプションを戻してもやり直さず ERROR のままになる不具合を直した(`app.py` の `set_option`)．
- 結果(CLI，理想の形の速さ．大回りまでは 乱数 6.650 秒・クラシック 11.959 秒):

  | 斜めのコスト | 乱数 200 枚 [s] | 斜めの区間 | クラシック 41 枚 [s] | 斜めの区間 |
  |---|---|---|---|---|
  | 7(斜めを考えない) | 6.514 | 0.5 | 11.590 | 1.0 |
  | 4 | 6.519 | 0.5 | 11.546 | 1.0 |
  | 3 | 6.514 | 0.5 | 11.491 | 1.1 |
  | 2 | 6.534 | 0.6 | 11.506 | 1.2 |
  | 1 | 6.543 | 0.6 | 11.492 | 1.2 |

  超信地旋回(回る時間は 0)は 11.314 秒・19.271 秒．どれも本当の迷路でぶつからなかった．
  - コストを下げても斜めの区間はほとんど増えない．区画の経路ではジグザグが増えても，置き換えで小回りに戻っている．
    理想の形の 45° の速さ(1100mm/s)から，出の直後の小回り(600mm/s)まで減速しきれず(約 142mm 要るのに後ろの残りの直線は 53mm)，
    加速度が足りない所を下げる処理で斜めをやめるため．
  - 試しに params.h の写しで斜めの速さを下げると(45° 600・135° 550・V90 600)，斜めの区間は 2.9〜3.2 個に増えるが，
    合計は 12.3 秒と大回りだけより遅い．置き換えは斜めにできる所を必ず斜めにし，時間で比べていないため．
- 確かめたこと: maze-sim のテスト(yuho を足して)は全部通った．学生クラシック 2021 で 4 つの曲がり方の経路を画面なしの Chrome で
  `motion.js` に通し，終わりがゴールの中心・再生の時間が機体の見積もりと合うことと，形を絵で確かめた(超信地旋回の前の直進で向きが
  食い違うのは決まりどおり)．迷路を全部知っているときの斜めの経路は，大回りと同じになった(斜めが 1 つも残らない)．
- 次(案): 置き換えで，斜めの区間ごとに「斜めにする・しない」を時間で比べて選ぶ．斜めの理想の速さを見直す．

### 2026-11-07 斜めの区間ごとに，やめた場合と時間で比べて選ぶようにした(`feature/diagonal-run` のまま)
- ユーザーの依頼(前の回の「次の案」の 1)．`RunPath_FromRouteEx` で，加速度が足りるように下げた後，斜めの区間(入りの区画)ごとに
  「その区画から斜めに入らない」列も作って見積もりの時間を比べ，加速度が足りるようになるか，速くなるならやめる．やめると別の所から
  斜めが始まることもあるので，変わらなくなるまでくり返す(同じ時間なら斜めのまま)．加速度で下げる部分は `BuildFeasible` に分けた
  (斜めを使わないときの動きは前と同じ)．試す列と下げた状態の写し(約 1.6KB)は run_path.c に置いた(ファームウェアの RAM にも入る)．
- 11章の「斜め走行」を直した(前: 「置き換えは斜めにできる所を必ず斜めにするので，時間で比べて…にしたい」．
  後: それは済んだので消し，「斜めの旋回の理想の速さは見直しが要る」を足した)．
- 結果(CLI．クラシック 41 枚，大回りまでは 11.959 秒):
  - 今の理想の速さ(45° 1100・135° 550・V90 700): 変わらない(コスト 7 で 11.590 秒，2 で 11.506 秒)．残っていた斜めはどれも速くなる所だった．
  - params.h の写しで斜めの旋回を遅くした場合(45° 600・135° 550・V90 600): 前は 12.306 秒(大回りより遅い)，今は 11.881 秒(コスト 7)・
    11.880 秒(コスト 2)．乱数 200 枚でも 6.613 秒(大回りは 6.650 秒)．遅くなる斜めを選ばなくなった．
  - 45° 850 の場合は前と同じ(11.689・11.637 秒)．
- 確かめたこと: 形ごとのテスト(逆変換で元の経路に戻る)は前と同じ結果．maze-sim のテスト(yuho を足して)は全部通った．

### 2026-11-07 斜めの直進の最高速度を直進と同じにした(`feature/diagonal-run` のまま)
- ユーザーの決め「斜めの直進で最大速度は直進のものと同等で考えたい」．`RunProfile.vmax_diag` と params.h の `RUN_VMAX_DIAG_MM_S`・
  `FAST_DIAG_VMAX_MM_S` を消し，斜めの直進も `vmax`(と直進の加速度・減速度)で数える．どれも 2000 で同じだったので今の数字は変わらない
  (クラシック 41 枚，斜めのコスト 2 で 11.506 秒)．前は maze_sim のオプション run で直進の最高速度を変えても斜めは 2000 のままだったが，
  今は一緒に変わる(`--run 3000,…` で 11.456 秒)．maze-sim の `motion.js` は前から斜めの直進にも vmax を使っていたので変えていない．

### 2026-11-07 2023 学生クラシックの長い斜めを通るようにした(`feature/diagonal-run` のまま)
- ユーザーの依頼「2023 学生クラシックでは長い斜めを通るようにしたい」．この迷路には (2,0) → (15,13) の 26 区画の斜め(北東のジグザグ)がある．
- 通らなかった理由は 2 つ:
  1. 斜めのコストの数え方: ジグザグの1区画が「直進 1 + 斜めの曲がり」で，一番小さくしても直進の1区画(1)より高い．実際の道のりは
     斜め 127mm・直進 180mm で逆．遠回りの長い斜めは区画の数で負ける．
  2. 探索: 全面探索で確かめる「最短経路の候補」は斜めを考えないコストで選ぶので，長い斜めの上の壁を見に行かない．
- 直したこと:
  - 斜めのコストの意味を変えた(前の回の「ジグザグの曲がりを 7 の代わりに 2」から): `diag_route` はコストを 10 倍で持ち，ジグザグの続きの
    区画は「直進 + 90°」の代わりに `MAZE_COST_DIAG_CELL_X10`(直進の1区画を 10 とした値．既定 7 = 道のりの比 127/180)．
    `MAZE_COST_DIAG_TURN` は消した．maze_sim のオプション `diag_cost` と CLI の `--diag-cost` も同じ意味(既定 7)．
  - 全面探索で，最短経路の候補を斜めのコストで選べるようにした: `SearchPlanner.full_diag`(作業領域へのポインタ．NULL なら今までどおり)と
    `full_diag_cell_x10`．`DiagRoute_BuildDirs` を足した．ファームウェアは NULL のまま(プランナーが 6 バイトほど増えるだけ)．
    maze_sim の画面は「全面探索 + turn = diag」のとき使う(全面探索中に turn・diag_cost を変えたら探索からやり直す)．CLI は `--full-diag`．
- 結果(2023 学生クラシック):
  - 迷路を全部知っている場合: 斜めのコスト 7 で `D135IN_R D1 DV90_L D24 D45OUT_L …`，8.969 → 8.676 秒．
  - 全面探索 + `--full-diag`: 同じ経路(DIAG x24)で 8.676 秒(前は 9.419 秒)．探索は 92.2 → 86.4 秒．画面でも確かめた．
- ただし全体では悪くなる迷路もある(クラシック 41 枚，全面探索): 斜めありの平均 11.156 → 11.299 秒．速くなるのは 11 枚
  (2022 学生 −1.47 秒など)，遅くなるのは 8 枚(2023 東日本・2024 北信越 +2.77 秒など)．斜めのコストを 6〜9 にしても平均は同じ．
  - 遅くなった迷路(2023 東日本)では，斜めのコストで選んだ経路のジグザグに同じ向きが 2 つ続く所(R R・L L)があり，置き換えで
    斜めを出た後は入り直せず，小回りが 12 回続いていた．経路のコストは「ジグザグは全部安い」と数えるので，置き換え後の時間と合わない．
  - R R の所の V90 は，最初に「柱の上を通る」として除いた．ただそれは回れる一番大きな半径(127mm)での話で，柱との隙間は
    約 90 − 0.41 × 半径 mm(半径 80mm で約 57mm)．半径を小さくすれば通れるかもしれない(確かめていない)．ユーザーに相談する．
- 確かめたこと: maze-sim のテスト(yuho を足して)は全部通った．`examples/yuho/yuho_mouse.c` を写し直した．

### 2026-11-07 柱を回る V90(DV90P)を使うようにした(`feature/diagonal-run` のまま)
- ユーザーの選択(前の回の相談の A): ジグザグの途中の「R R」の所も V90 で斜めを続ける．
- `run_path`: 指令 `RUN_DV90P_R/L`(21・22)．斜めの点で入った区画を z(ジグザグの続きの向き)へ抜け，次の区画もさらに z へ抜ける．
  角は間の通り抜ける境目で，内側に柱が1本入る．折れ線は DV90 と同じ(127 + 127mm)．V90 の向きは z，その後のジグザグは −z から．
  置き換えでは「出」より先に試す(R R の後に斜めの続きがあるときだけ．なければ今までどおり D135OUT)．run_path.h の表と，
  「L R R L は柱に当たるので使わない」としていた説明を直した．
  - 形: 理想の形は柱から離れるよう半径を半区画(90mm)に抑えた(前後のオフセット 37mm，`RUN_V_DV90P_MM_S` 600)．
    実機の形は仮に `FAST_DV90P_*` 450mm/s・330dps・5000dps²(オフセット約 34mm，横加速度 0.26G)．柱との隙間 ≒ 90 − 0.41 × 回る半径 mm．
  - `SetDiagTurn` の a・b が初期化されていない警告も消した．
- maze-sim(未コミット): `MSIM_RUN_DV90P_R/L`，`motion.js` で半径 S/2 の形，説明．テストは全部通った．
- 結果(クラシック 41 枚，全面探索，斜めあり):
  - `--full-diag` なし 11.156 → 10.856 秒，あり 11.299 → 10.798 秒(あり の方が速くなった)．
  - 2023 東日本 13.814 → 10.642 秒(DV90P を 2 つ使い，12 区間の斜めが 2 本)．2023 学生は 8.676 秒のまま(26 区画の斜め)．
  - `--full-diag` で速くなる迷路 15 枚・遅くなる 6 枚(前は 11 枚・8 枚)．遅くなる迷路は，斜めのコストで選んだ経路が
    置き換えた後の時間では速いとは限らないため(2025 東日本 +1.64 秒など)．
- 画面なしの Chrome で 2 つの迷路の経路を壁と一緒に描き，終わりがゴールの中心・時間が見積もりと合うこと，表示の形での柱との
  一番近い距離が 52.7mm(DV90P)・74mm(DV90)であることを確かめた．

### 2026-11-07 走行時間の Dijkstra に斜めを入れた(`feature/diagonal-run` のまま)
- ユーザーの問い「時間ベースで斜めが出てこないのは仕様か．出るようにしてほしい」．仕様だった(最初に，RAM のため時間の Dijkstra には
  斜めを足さず，区画の経路の置き換えで斜めを作ると決めていた)．ただ時間の Dijkstra は maze_sim だけで使い，ファームウェアは使わないので，
  作業領域を呼ぶ側に持たせれば RAM は問題にならない．
- 11章の「斜め走行」を直した(前: 「FAST のメニュー，走行時間の Dijkstra への斜めのノードは未着手」．後: 「FAST のメニューは未着手」)．
- `time_dijkstra` を書き直した:
  - ノード: 直交の位置(境目・中心)× 旋回の種類 5(小回り・大回り 90・180・D45IN・D135IN)と，斜めの点(区画・入る向き・ジグザグの続きの向き)
    × 4(D45OUT・D135OUT・DV90・DV90P)．全部で 13312．辺は今までどおり「旋回 + 直進 h」で，斜めの点で終わる旋回の後は斜めの直進
    (1区画/√2 単位．h = 0 も可)．斜めの直進を1区間戻すときは，手前の区画に入る向きが Dir_Turn(向き, z)，続きの向きが −z．
  - 旋回の入口は，4 つの入る向きを試して「最後に区画を移る向きが出口の向き」になるものを探す(斜めの旋回も同じ書き方で済む)．
  - ヒープ・直進の時間の表などの作業領域を全部 `TimeSolver` に移した(約 170KB．ファームウェアは使わない)．
  - `TimeDijkstra_ComputeEx(…, use_diag)` を足した．`TimeDijkstra_Compute` は斜めなしで，結果は前と同じ
    (往復探索で 乱数 6.640 秒・クラシック 11.876 秒のまま)．
- maze_sim: 画面の1つめの経路は turn = diag のとき「time-optimal + diag」．CLI は時間最短の斜めありも出す(計算した時間と
  指令の列から見積もった時間の食い違い，本当の迷路での衝突も確かめる)．
- 結果(クラシック 41 枚，全面探索): 時間最短 + 斜め 10.512 秒(`--full-diag` で 10.369 秒．迷路を全部知っていれば 10.229 秒)．
  コストで選んで置き換えた経路(10.856・10.798 秒)より速い．乱数 200 枚 6.098 秒(全部知っていれば 6.088 秒)．食い違い・衝突は 0．
  2023 学生クラシック(全面探索 + `--full-diag`): 8.646 秒で 26 区画の斜め(DIAG x24)を通る．全部知っていれば 8.226 秒．
- 確かめたこと: maze-sim のテスト(yuho を足して)は全部通った．2023 学生の時間最短 + 斜めの経路を画面なしの Chrome で壁と描き，
  終わりがゴールの中心・時間が見積もりと合う・柱との一番近い距離 52.7mm を確かめた．

### 2026-11-07 斜めの経路を機体で走れるようにした(`feature/diagonal-run` のまま)
- ユーザーの依頼「この斜めの経路を機体で使えるようにしたい」．確かめた決めごと(ユーザーの選択):
  - 経路は斜めのコストの区画の経路(`diag_route`)を置き換える．全面探索でも同じコストの経路を確かめに行く．
    (時間の Dijkstra は作業領域が約 170KB で機体の RAM に入らない)
  - 斜めの旋回を確かめる試験モードも作る．
- 11章の「斜め走行」を直した(前: 「logic と maze_sim の CLI まで．実機の走らせ方・FAST のメニューは未着手」．
  後: 機体の最短走行と試験まで作った，形は仮，全面探索の計算時間を確かめる)．
- 作ったもの(`app/search_run.c` ほか):
  - `RunList_Drive`: 斜めの直進(1区画/√2 単位．壁の補正は制御全部乗せでも使わない)と，斜めの旋回(角度は `RunType_TurnDeg`．
    スラロームはもともとどの角度でも回せ，目標の位置も2次元で積んでいる)．小回りの前壁補正は小回りだけにした(前は「大回りでない」で判定)．
    `LOG_EV_TURN_KIND` の kind に 4〜9(D45IN・D45OUT・D135IN・D135OUT・DV90・DV90P)を足した．
  - 最短走行: FAST の走り方に 4 DIAG．`PrepareFastRoute` で斜めのコストの経路を選び，`RunPath_FromRouteEx` で置き換える．
    作業領域 `s_diag_solver`(約 22KB)は全面探索と共有して RAM に置いた．`ComputeFastTurns` で斜めの旋回の形も表示する．
  - 全面探索: `SEARCH_FULL_DIAG`(1)なら，確かめる最短経路の候補を斜めのコストで選ぶ(`StartFullSearch`．帰りの全面探索も)．
    計算は `DiagRoute_ComputeFrom`(スタートのノードが決まったら止める．経路は同じ)を足して使う．
  - 試験: RUN の TEST に 6 DIAG(`DiagTest_Run`)．形 1 45 / 2 135 / 3 V90 / 4 V90P と右・左，直進の速さを選び，手かざしで走る．
    区画 (0,0) の南の壁に尾を付けて北向きに置く．指令の列は決まった形を直接組む(置き換えだと小回りに戻されることがある)．
    範囲(右回り，x は東): 45 は x 0〜2・y 0〜7，135 は x 0〜3・y 0〜2，V90 は x 0〜6・y 0〜4，V90P は x 0〜3・y 0〜4．ログは search/diag_NNNN，
    走り始めに `LOG_EV_DIAG_TEST`(63．62 は turnback のブランチの SLIP の STAIR が使うので避けた)．
- 確かめたこと(PC):
  - 試験の4つの形を区画の経路に戻せること・範囲(scratchpad のプログラム)．
  - 機体の組み立て(斜めのコスト → 実機の形 `RunProfile_ForSpeeds` で置き換え)を 2023 学生クラシックで再現: 長い斜め(D45IN_L D23 D45OUT_L)が残り，
    加速度は足り，区画の経路に戻せた．
  - `DiagRoute_ComputeFrom` にしても maze_sim の結果は同じ．PC で計算時間を測ると，斜めのコストの経路は今の Dijkstra の約 12 倍
    (190µs と 15µs)．機体では数 ms と見込む(確かめていない)．全面探索で候補が壁でふさがれたときの境界で計算する．
- 確かめていないこと: ファームウェアのビルド(RAM が約 25KB 増える)．実機での走り．

### 2026-11-07 斜めのコストの Dijkstra にバケツのキューを入れ，機体で計算時間を測るモードを作った(`feature/diagonal-run` のまま)
- ユーザーの問い「速度依存の経路の導出で，時間を毎回計算せずテーブルで持てば軽くならないか」．PC で測った(2023 学生クラシック):
  時間の Dijkstra(maze_sim だけ)はもう直進・旋回の時間を表にして引いていて，表づくりは 215µs / 2765µs．機体の最短走行の準備は
  旋回の形(`RunProfile_ForSpeeds`．1ms 刻みでなぞる)が 1342µs で一番重いが，手かざしの前に1回だけ．走っている途中に効くのは，
  全面探索の境界で計算し直す斜めのコストの Dijkstra(速さを使わないコストの計算)で，時間の表では軽くならない．
  そこで，こちらを軽くして効き目を比べることにした(ユーザー「効き目を比べよう」)．
- `diag_route`: 1区画の上乗せの最大(10 倍で 510)が箱の数 512 に収まればバケツのキュー(Dial の方法．dijkstra.c と同じ考え方)，
  そうでなければ二分ヒープ．`heap_only` で二分ヒープだけにできる(比べる用)．heap / heap_pos を箱の次・前に使い回す(+ 箱の先頭 1KB)．
  PC: 大会の迷路 87 枚 + 何も分かっていない地図で，コストと次の向きの表が二分ヒープと全部同じ．時間は 229 → 79µs(今の Dijkstra は 20µs)．
  maze_sim の結果も前と同じ．maze-sim のテストは全部通った．
- 機体で測るモード: RUN の TEST に 7 PLAN(`PlanBench_Run`．モーターは動かさない)．`PLAN_BENCH_REPEAT`(100)回ずつくり返し，
  1回あたりの時間をシリアルへ出す: 今の Dijkstra(スタートで止める)，斜めのコスト(二分ヒープ・バケツ．スタートで止める)，
  斜めのコスト(全部のノード．最短走行の準備)．地図は 何も分かっていない地図で中央へ(全面探索で一番重い形)と，flash の地図で今のゴールへ．
- `SEARCH_USE_TEST_GOAL` を 1 にした(試しの迷路のゴールが (1,0) とユーザー談)．ゴールが近いと全面探索の計算はすぐ終わるので，
  探索のログの plan_ms では差が見えにくい．比べるのは PLAN で．
- 次: ユーザーがビルドして PLAN を走らせ，シリアルの値で二分ヒープとバケツを比べる．
- 結果(機体の PLAN．ユーザーが貼ったシリアル．100 回ずつの 1 回あたり):

  | 地図 | 今の Dijkstra | 斜め(二分ヒープ) | 斜め(バケツ) | 斜め(全部のノード) |
  |---|---|---|---|---|
  | 何も分かっていない地図 → 中央 | 2810µs | 18500µs | 9870µs | 1080µs |
  | flash の地図 → (1,0) | 240µs | 3080µs | 2000µs | 8300µs |

  - バケツの効き目は 1.9 倍(中央)・1.5 倍(1,0)．PC(2.9 倍)より小さい．機体は PC のおよそ 125〜140 倍遅い(Debug も `-O2`．
    前の回に「数 ms」と見積もったのは外れ．一番重い形で約 10ms)．
  - 全部のノードの計算が何も分かっていない地図で小さいのは，最短走行の見方(未知の壁は「ある」)ではスタートから進めないため．
  - 全面探索の境界で使える時間: 壁を読むのが境界の 12mm 手前，小回りの曲がり始めが約 9mm 先なので，500mm/s で約 40ms．
    バケツなら一番重い形でも 2.8 + 9.9 ≒ 13ms で収まる(二分ヒープは約 21ms)．斜めの計算は候補の経路が壁でふさがれた境界だけ．
    バケツのまま使う．実際の遅れは，大きめの迷路の全面探索の plan_ms で確かめる．

### 2026-11-07 斜めの旋回の試験(DIAG)のログを見た(search/diag_0001〜0004．`feature/diagonal-run` のまま)
- 4 本とも 形 1(45°)・右回り・600mm/s．どれもフェイルセーフ(cause 2 = 速度の偏差)で止まった(ログの値は電池の電圧 6.3〜7.2V)．
- 45° 入りと斜めの直進は予定どおり: 0002〜0004 で，推定の位置(est．北が x，西が +y．置いた区画の中心が x ≈ 43)の斜めの線が
  予定(x + y = 493)から 1.5mm ほど，向き −45.0 ± 0.5°，横のずれ 1.4mm 以内．0002 は 45° 出の後の列(y −358.6，予定 −360)も合った．
- 止まった所: 0001 は 45° 入りの終わりで右のセンサーが振り切れ(994〜1118)，(0,3) と (1,3) の間の壁に当たったと見られる(範囲に壁が
  残っていた)．0002 は最後の直進で前の壁(列 2 の 5・6 行目の間)．0003 は 45° 出の初め(t ≈ 1.90)で左の車輪が止まり(vl 11)，
  目標より約 9° 先に左へ回って止まった．0004 も 45° 出の初めで左が触れて 11° 先に回り(持ち直したが横に約 27mm)，最後は前の壁
  (6・7 行目の間)．45° 出の接触の相手(内側の柱か壁)は，機体の大きさが分からないので決められない．
- 私の誤り: 形 1 は y 0〜7(8 行)を使う作りで，試しの迷路が 5 × 7 なら最後の直進が外周に当たる(0004)．V90(7 列)も収まらない．
  走らせた場所の大きさをユーザーに聞き，範囲に収まる形に作り直す．
