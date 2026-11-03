#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

// 同定のブランチ(feature/system-identification)の params.h。
// これまでの経験則で合わせた値(車輪径 23.81、トレッド 64.75、FF・PID のゲインなど)は使わず、
// 実測・データシートの値と、同定の実験(E0/E1/E2、app/sysid_mode)に要る値だけを置く。

// ============================================================
// 機体寸法・エンコーダ換算
// エンコーダはモーター軸に付いているので、タイヤ1回転あたりの
// カウント数 = モーター1回転のパルス数 × 逓倍 × 減速比。
// ============================================================

// ノギスでの実測。ログの速度・距離はこの値で換算する(換算は比例なので、
// 後で直径を変えるときは 新しい直径 / 23.35 を掛ければ解析で直せる)
#define WHEEL_DIAMETER_MM      23.35f

#define ENCODER_PPR        4096.0f // FAULHABER IEH2-4096: モーター1回転あたり
#define ENCODER_MULTIPLIER 4.0f    // TIM4/TIM8 = TIM_ENCODERMODE_TI12 (A/B両エッジで4逓倍)
#define GEAR_PINION_T      13.0f   // M0.5
#define GEAR_SPUR_T        42.0f   // M0.5

// タイヤ1回転あたりのカウント数 (≈52935)
#define ENCODER_PULSES_PER_REV (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_SPUR_T / GEAR_PINION_T)

#define MM_PER_PULSE ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_PULSES_PER_REV)

// エンコーダの取り付け向きにより、パルス増加方向と前進方向が
// 逆転する場合に-1.0fへ反転させる。
#define ENCODER_L_SIGN 1.0f
#define ENCODER_R_SIGN -1.0f

// ============================================================
// 制御周期
// TIM6: Prescaler=84-1, Period=1000-1 → 84MHz/84/1000 = 1kHz
// ============================================================

#define CONTROL_DT_S 0.001f

// ============================================================
// バッテリー電圧 (2S LiPo)
// VBAT -- R14(33k) -- VOL_CHECK(PC0) -- R15(20k) -- GND (yuho.netより)
// ADCの基準電圧(VREF+)は3.3V想定(未確認)。
// ============================================================

#define ADC_VREF_V            3.3f
#define ADC_FULL_SCALE        4095.0f
#define BATTERY_DIVIDER_RATIO ((33.0f + 20.0f) / 20.0f)

#define BATTERY_FULL_V   8.4f  // 2S満充電。起動時LEDバー表示の上端
#define BATTERY_IIR_ALPHA 0.1f // 1kHz更新でのIIR係数(時定数 約10ms)

// ============================================================
// ジャイロ (ICM-42605。WHO_AM_I = 0x42 を 2026-10-30 に実機で確かめた。前は ICM-42688-P と書いていた)
// ICM_Init()で±2000dpsに設定する。
// ============================================================

#define GYRO_SENSITIVITY_LSB_PER_DPS 16.4f
#define GYRO_Z_SIGN 1.0f // 反時計回り(左旋回)を正にしたい場合に実機で合わせる

// 走り出す直前に、機体が止まっている間にジャイロ Z のゼロ点を測り直す時間[ms]
#define GYRO_RECAL_MS 500

// 加速度(同じ ICM。ICM_Init()で ±8g / ODR 1kHz に設定する)
#define ACCEL_SENSITIVITY_LSB_PER_G 4096.0f
#define GRAVITY_M_S2                9.80665f

// 6軸の値の向き(logic/state_estimation/imu)。機体の座標は x = 前、y = 左、z = 上。
// 機体の i 軸 = センサーの IMU_AXIS_MAP[i] 軸 × IMU_AXIS_SIGN[i](0 = X、1 = Y、2 = Z)。加速度とジャイロの両方に使う。
// 回転になっている(右手系のまま)こと: 2 つの軸を入れ替えたら、どれか 1 つの符号を反転する。
// 2026-10-30: z はこれまでの GYRO_Z_SIGN(反時計回りが正)と同じ。止まって水平なら acc_z は約 +9.85 だった(E1S の 0021・0022)。
// 2026-10-30: E1S の 0021・0022 で、前に加速したときセンサーの −y が dv/dt と合った(相関 0.97〜0.99、傾き 1.03〜1.06)ので、
// 前 = センサーの −y にした。左 = センサーの +x は、右手系にするための決め方で、まだ確かめていない(z 軸まわりに −90° の回転)
#define IMU_AXIS_MAP  { 1u, 0u, 2u }
#define IMU_AXIS_SIGN { -1.0f, 1.0f, GYRO_Z_SIGN }

// ============================================================
// 非接触スタート
// 左前の壁センサー(ad_fl)がしきい値を START_HOLD_MS 続けて超えたら手をかざしたと見なし、
// しきい値を下回ったら(手を離したら)走り出す。
// ============================================================

#define SENSOR_START_THRESHOLD 350 // [ADの差分値]
#define SENSOR_START_HOLD_MS   50   // ノイズで1回だけ大きくなっても始めないための継続時間

// ============================================================
// フェイルセーフ
// 発動すると制御を止めてラッチする(リセットまで復帰しない)。
// ============================================================

#define FAILSAFE_LOW_VOLTAGE_V   7.0f // これを下回り続けたら停止 (セル3.5V)
#define FAILSAFE_LOW_VOLTAGE_MS  500  // 加速時の電圧降下で誤発動しないための継続時間

// E2 の 0.5V は理論で約 1820dps まで回るので、ジャイロの上限(約 ±2000dps)の手前にした。
// ふつうは同定の中止(SYSID_ABORT_OMEGA_DPS)のほうが先に効く
#define FAILSAFE_GYRO_DPS        1950.0f // |角速度Z|
#define FAILSAFE_GYRO_MS         10

// ============================================================
// モード選択
// 右エンコーダがこのパルス数だけ回転するたびにモードを1つ送る。
// ============================================================

#define MODE_SELECT_PULSES_PER_STEP (ENCODER_PULSES_PER_REV * 0.25f) // タイヤ1/4周で1モード

// ============================================================
// 同定の実験(app/sysid_mode、logic/sysid/step_test)
// 開ループ: 毎 tick、duty = 電圧 / 電池の電圧 で PWM にする(PID・FF なし)。
// 区間(ログの seg 列): 0 始まり(0V) → 1 +V → 2 −V(止まるまで) → 3 ブレーキ(0V)。中止したら 9(0V)。
// E1: 並進のステップ。E2: 超信地旋回のステップ(電圧の正 = 反時計回り。右に +V、左に −V)。
// E0: 0V からゆっくり上げて、動き出す電圧を見る。
// ============================================================

#define SYSID_VBAT_MIN_V         5.0f   // 電圧→duty の変換に使う電池の電圧の下限[V](電池がないときの0割り防止)
#define SYSID_PRE_MS             50u    // 区間0(0V で記録する始まりの目印)
#define SYSID_BRAKE_MS           100u   // 区間3・9(0V = ショートブレーキで記録してから終わる)
#define SYSID_REST_MS            500u   // 区間4(階段で、止まった後に次のステップをかけるまでの休み)
// 階段(E2S)は −V で止めず、0V(ブレーキ)にして止まるのを待つ。止まった = |角速度|(並進なら |速度|)が
// 下の値より小さく、かつ左右の車輪がどちらも 1 tick あたり SYSID_STOP_COUNTS カウント以下、の状態が
// SYSID_STOP_HOLD_MS 続いた。SYSID_STOP_TIMEOUT_MS を過ぎても止まらなければ中止。
// 車輪の条件は、機体を浮かせて空転させる試験のため(浮かせると機体は回らず、ジャイロはずっと約 0 になる。10-28 に決めた)
#define SYSID_STOP_OMEGA_DPS     5.0f
#define SYSID_STOP_V_MM_S        5.0f
#define SYSID_STOP_COUNTS        1     // [カウント / tick](1 カウント/ms ≒ 1.4mm/s)
#define SYSID_STOP_HOLD_MS       50u
#define SYSID_STOP_TIMEOUT_MS    2000u
#define SYSID_REVERSE_TIMEOUT_MS 1000u  // 区間2(−V)でこれを過ぎても止まらなければ中止
#define SYSID_ABORT_DIST_MM      1000.0f // 並進(E0・E1)で |進んだ距離| がこれを超えたら中止
#define SYSID_ABORT_OMEGA_DPS    1900.0f // 旋回(E0・E2)で |角速度| がこれを超えたら中止
// どの試験でも、左右どちらかの |車輪の速さ| がこれを超えたら中止(浮かせて空転させると負荷がなく、速く回りすぎるため)。
// 2500mm/s はモーター約 6600rpm、エンコーダの A 相 約 450kHz。モーターの最高回転数とエンコーダの最高周波数は
// データシートで確かめていない(仮の値)。床の上の E2S では、1900dps で中止したとき車輪は約 1100mm/s だった
#define SYSID_ABORT_WHEEL_MM_S   2500.0f

#define SYSID_E0_RAMP_V_PER_S    0.1f   // E0 で電圧を上げる速さ[V/s]
#define SYSID_E0_V_MAX           1.0f   // E0 でここまで上げても動かなければ打ち切る[V]
#define SYSID_E0_MOVE_MM         0.5f   // 車輪がこれだけ動いたら「動き出した」とする[mm](1 カウント 約 0.0014mm)

#define SYSID_START_DELAY_MS     1000u  // 手を離してから走り出すまで(この中でジャイロのゼロ点を測る)

// 走る前に選ぶ値の表(小さい順。ModeUI_SelectValue の表示が整数なので mV・ms で持つ)
#define SYSID_SELECT_MV          { 100.0f, 200.0f, 300.0f, 400.0f, 500.0f }  // 電圧(FROM と TO を選ぶ)
#define SYSID_SELECT_TACC_MS     { 500.0f, 1000.0f, 1500.0f, 2000.0f, 3000.0f, 4000.0f, 5000.0f, 6000.0f } // 区間1の長さ T_acc
#define SYSID_SELECT_TACC_START  1u     // 選び始める番号(0 始まり。1 = 1000ms)
#define SYSID_SELECT_REPEAT      { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f }            // 1つの電圧で往復する回数
#define SYSID_SELECT_REPEAT_START 2u    // 2 = 3回

// E2 の階段(E2S)。1回の手かざしで、電圧ごとに 左回り → 右回り のステップを FROM から TO まで続ける。
// 1ステップ: +V を T_acc → 0V で止まるまで → SYSID_REST_MS 休む(−V はかけない。0V にした後の止まり方も応答として使う)。
// 2026-10-28: E2 の 0.1〜0.5V では静止摩擦に負けてほとんど回らなかった(1.0V まで上げる E0 でも回り続けなかった)ので、
// 0.2〜3.0V を 0.2V 刻みにした。表の項目は 15 個まで(値を選ぶ画面の上限)
#define SYSID_STAIR_MV           { 200.0f, 400.0f, 600.0f, 800.0f, 1000.0f, 1200.0f, 1400.0f, 1600.0f, \
                                   1800.0f, 2000.0f, 2200.0f, 2400.0f, 2600.0f, 2800.0f, 3000.0f }
#define SYSID_STAIR_INC_V        0.2f   // 1段ごとに上げる電圧[V](上の表の間隔と同じにする)
#define SYSID_STAIR_TACC_START   3u     // T_acc の選び始め(SYSID_SELECT_TACC_MS の 3 = 2000ms。回らなければ 2 秒で次へ)
// 2026-10-28: 床で 2 秒では高い電圧で定常に届かなかったので、τ_rot を読むために 4 秒も選べるようにした
// 2026-10-29: 床の τ_rot は最大 約 0.54 s。3τ + 定常を平均する区間を長くとれるよう 5・6 秒も足した

// 前進のステップ(E1S)。手かざし1回で1ステップだけ前進し、手で元の場所に戻して次の電圧へ進む。
// 1周(FROM〜TO)を1つのファイルにする(置き直している間は記録を止める)。
// 1ステップ: +V を T_acc → 0V で止まるまで(E2S と同じ判定) → SYSID_REST_MS 休む(−V はかけない)。
// 前進だけ(後退は実際の走行で壁当てにしか使わないため)。ログの step 列が何番目の電圧か(1 = FROM)。
// 2026-10-29: 動き出しのあたりを細かく見るため 0.1V 刻み。表の項目は 15 個まで(値を選ぶ画面の上限)
#define SYSID_E1S_MV             { 200.0f, 300.0f, 400.0f, 500.0f, 600.0f, 700.0f, 800.0f, 900.0f, \
                                   1000.0f, 1100.0f, 1200.0f, 1300.0f, 1400.0f, 1500.0f }
// 中止(区間1。中止したらすぐ 0V にし、止まるまで待って終わる。周の残りの電圧は飛ばす)。
// 2026-10-29: 直線を約 1.26m(180mm × 7 マス)から約 2.88m(16 マス)にしたので、距離 900 → 2400mm、角度 15 → 45deg にした。
// 2026-10-30: 組み直した足回りの直進の応答を測る実験の仕様で、900mm・15deg に戻した(補正なしなので前より曲がるかもしれない)
#define SYSID_E1S_ABORT_DIST_MM   900.0f  // |進んだ距離|(区間1の始まりから)
#define SYSID_E1S_ABORT_V_MM_S    1100.0f // |(vl + vr) / 2|
#define SYSID_E1S_ABORT_ANGLE_DEG 15.0f   // |ジャイロの積分角|
// 浮かせた E1S(PLACE 2): 1回の手かざしで FROM から TO まで、1ステップ(+V を T_acc → 0V で止まるまで → 休み)ずつ電圧を上げて続ける。
// 中止は車輪の速さ(SYSID_ABORT_WHEEL_MM_S)と止まらない(SYSID_STOP_TIMEOUT_MS)だけ(距離・速度・角度は見ない)
#define SYSID_E1S_INC_V           0.1f    // 浮かせた E1S で1段ごとに上げる電圧[V](SYSID_E1S_MV の間隔と同じにする)
// T_acc の選び始め(SYSID_SELECT_TACC_MS の 1 = 1000ms。前の足回りの床の τ 約 0.57s では定常の約 83% までなので、1次遅れの当てはめで定常を出す)
#define SYSID_E1S_TACC_START      1u

// FF だけで走る検証(E1V)。同定したモデル V = KV·v + V0 + KA·dv/dt だけで電圧を出し、
// 並進はフィードバックなしで台形の速度パターン(v_max まで加速 → v_max で CRUISE_MS → 減速 → 0)を走る(logic/sysid/ff_test)。
// 左右: V_L = V + ΔV − V_d、V_R = V − ΔV + V_d。ΔV = LR_C·v_ref + LR_A·a_ref(FF の補正)、V_d は旋回のフィードバック(下)。
// 手かざし1回で1本(1ファイル sysid/e1vf_<v_max>_a<加速度 m/s²>_c<LR_C の mV/(m/s)>_NNNN.bin)。
// 台形は下の表の 1〜6 本目(ログの step 列が表の何本目か)。走る前に「全部 / 1・3・4・6 / 2・5」を選ぶ。
#define SYSID_E1V_KV             0.561f  // [V/(m/s)]
#define SYSID_E1V_V0             0.210f  // [V]
#define SYSID_E1V_KA             0.320f  // [V/(m/s²)](10-31: 0.286 では加速が目標の約 9 割だったので)
// 10-31: c = 0.246 は加速の間は合ったが、V_s が下がると効きすぎた。旋回のフィードバックで測るので 0 にする
#define SYSID_E1V_LR_C           0.0f    // [V/(m/s)](前の値 0.246)
#define SYSID_E1V_LR_A           0.0f    // [V/(m/s²)]
// 旋回のフィードバック V_d = −K_THETA·θ − K_OMEGA·ω(V_L = V_s − V_d、V_R = V_s + V_d。V_d > 0 で左に曲がる向き)。
// まっすぐ保てていれば、V_d が「左に曲げるもの」を打ち消すのに要った左右の差になる。ゆれるなら K_THETA を下げるか K_OMEGA を上げる
#define SYSID_E1V_K_THETA        7.0f    // [V/rad](約 0.12V/deg)
#define SYSID_E1V_K_OMEGA        0.5f    // [V/(rad/s)]
#define SYSID_E1V_VD_MAX         1.0f    // |V_d| の上限[V](止めた時刻を UART に出す)
// 並進の PI(E1V4): V_s = V_FF + KP·e + KI·∫e dt(e = v_ref − v[m/s]、v_ref > 0 の間)。|KI·∫e dt| ≤ I_MAX。0 にすると FF だけ
#define SYSID_E1V_KP             5.8f    // [V/(m/s)]
#define SYSID_E1V_KI             29.0f   // [V/m]
#define SYSID_E1V_I_MAX          0.5f    // [V]
#define SYSID_E1V_CRUISE_MS      500u    // V_d が落ち着くのを見るため 200 → 500
// 台形の表(v_max と加速度・減速度の組。3m/s² を先に、次に 5m/s²。前の足回りでは 10m/s² で空転したので 5 まで)
#define SYSID_E1V_VMAX_MM_S      { 1000.0f, 1500.0f, 2000.0f, 1000.0f, 1500.0f, 2000.0f }
#define SYSID_E1V_ACCEL_MM_S2    { 3000.0f, 3000.0f, 3000.0f, 5000.0f, 5000.0f, 5000.0f }
// 中止(区間1〜3。すぐ 0V にし、止まるまで待って終わる)
#define SYSID_E1V_ABORT_DIST_MM      2600.0f // 走り始めからの |距離|
#define SYSID_E1V_ABORT_V_MARGIN_MM_S 500.0f // |(vl + vr) / 2| > v_max + これ
#define SYSID_E1V_ABORT_ANGLE_DEG    15.0f

// 減速の同定(E1B)。E1V の FF(KV・V0・KA)で 3m/s² で v0 まで加速 → v0 で CRUISE_MS → 一定の電圧 V_brake を、
// 並進の速度が END_MM_S 未満になるまで → 0V(ショートブレーキ)で止める。旋回の FB(SYSID_E1V_K_THETA・K_OMEGA)は区間1〜3 でかける。
// 手かざし1回で1本(1ファイル sysid/e1b_<v0 mm/s>_m<−V_brake の mV>_NNNN.bin)。下の表の順に進む(−1.2V は空転の心配があるので最後)。
// ブレーキの区間の v(t) に1次遅れを当てはめ、v∞ = (V_brake − V0d)/a_d と τ_d = b_d/a_d から減速の係数を出す。
#define SYSID_E1B_ACCEL_MM_S2    3000.0f
#define SYSID_E1B_CRUISE_MS      300u
#define SYSID_E1B_V0_MM_S        { 1000.0f, 1000.0f, 1000.0f, 1500.0f, 1500.0f, 1500.0f, 1000.0f, 1500.0f }
#define SYSID_E1B_BRAKE_V        { 0.0f,    -0.4f,   -0.8f,   0.0f,    -0.4f,   -0.8f,   -1.2f,   -1.2f }
#define SYSID_E1B_END_MM_S       50.0f   // ブレーキを終える並進の速度
// 中止(区間1〜3): 距離 > SYSID_E1V_ABORT_DIST_MM、|angle| > SYSID_E1V_ABORT_ANGLE_DEG、後ろに戻った(下)。電池はフェイルセーフ
#define SYSID_E1B_ABORT_BACK_MM_S 100.0f // (vl + vr) / 2 < −これ

// 旋回の同定(E1T)。走りながら左右に電圧の差 ΔV をステップで入れ、ω の応答から a_ω・b_ω・V0ω と横すべりの限界を見る。
// 並進は E1V4 と同じ FF + PI(SYSID_E1V_KV・V0・KA・KP・KI・I_MAX)を全区間で。旋回の FB は SYSID_E1V_K_THETA・K_OMEGA。
// 1 加速(組ごとの加速度で v まで、FB) → 2 直進(組ごとの時間、FB。最後の組ごとの時間の V_d の平均を v_d_meas に)
// → 3 ステップ(FB なし。V_d = V_d0 ± ΔV、V_d0 = v_d_meas(その本の区間2 の最後の V_d の平均)。TURN_MS たつか、回転が TURN_DEG を越えるまで)
// → 4 止まる(FB の θ を 0 に入れ直し、DECEL で減速) → 5 0V で止まるまで。手かざしで次の ΔV へ。
// 1本1ファイル sysid/e1t3_<v>mms_<p|m><|ΔV| の 0.01V>_NNNN.bin(例 e1t3_1000mms_p030)。
// 11-02(E1T3): 1260mm の向きに走り、曲がる向きと反対側の壁ぎわから出る。回すのは 60° まで。走る前に組(SET)を選ぶ:
//   1 = B(1.0m/s、κ の形・τ_ω・頭打ちの κ)、2 = A(0.5m/s、±0.5V を3回ずつ。c_κ のばらつき)、3 = C(1.5m/s、横の加速度での頭打ち)
// 11-02(9.11): 場所を 7 × 10 マス(1260 × 1800mm)にした。1800mm の向きに走る(横は約 1140mm)。
//   C も直進 0.2s にし、B・C に ±0.25V(κ ≈ 1 が入るようになった)を足した。中止の距離は 1.65m。
// 11-02: 大きい ΔV は 60° では定常に届かなかった(τ 0.11〜0.24s)ので、270° まで回す組を足した:
//   4 = D(1.0m/s、±0.65・0.8・1.0V)、5 = E(0.5m/s、同じ ΔV。D と比べて頭打ちが κ で決まるか v²κ で決まるかを見る)。
//   定常の半径は 0.1〜0.15m なので場所は足りる。回す角度・中止の角度・区間3 の長さの上限は組ごと
//   (中止の角度は、止まる間の回りすぎを見込んで 360°。E は ω が小さいので区間3 の上限を 2.5s に)。
#define SYSID_E1T_SET_SPEED_MM_S { 1000.0f, 500.0f, 1500.0f, 1000.0f, 500.0f }
#define SYSID_E1T_SET_ACCEL_MM_S2 { 3000.0f, 3000.0f, 5000.0f, 3000.0f, 3000.0f }
#define SYSID_E1T_SET_STRAIGHT_MS { 200u, 200u, 200u, 200u, 200u }
#define SYSID_E1T_SET_MEAS_MS    { 100u, 100u, 80u, 100u, 100u }
#define SYSID_E1T_SET_TURN_MS    { 1500u, 1500u, 1500u, 1500u, 2500u }
#define SYSID_E1T_SET_TURN_DEG   { 60.0f, 60.0f, 60.0f, 270.0f, 270.0f } // 11-02: B・A・C は 60°(ΔV の小さい本は τ 30〜80ms で届いた)
#define SYSID_E1T_SET_ABORT_ANGLE_DEG { 80.0f, 80.0f, 80.0f, 360.0f, 360.0f }
#define SYSID_E1T_DV_B_V         { 0.25f, -0.25f, 0.3f, -0.3f, 0.4f, -0.4f, 0.5f, -0.5f, 0.65f, -0.65f, 0.8f, -0.8f }
#define SYSID_E1T_DV_A_V         { 0.5f, -0.5f, 0.5f, -0.5f, 0.5f, -0.5f }
#define SYSID_E1T_DV_C_V         { 0.25f, -0.25f, 0.35f, -0.35f, 0.5f, -0.5f, 0.65f, -0.65f } // すべって壁に向かうので小さい方から
#define SYSID_E1T_DV_D_V         { 0.65f, -0.65f, 0.8f, -0.8f, 1.0f, -1.0f } // 1.0V は内側の車輪の電圧が負になる
#define SYSID_E1T_DV_E_V         { 0.65f, -0.65f, 0.8f, -0.8f, 1.0f, -1.0f }
#define SYSID_E1T_DECEL_MM_S2    5000.0f  // 区間4 で止まる減速度
// 10-31: V_d0 の式(−0.119 − 0.067·V_FF)は 0.5m/s で 0.04〜0.15V 右へ押しすぎたので、その本で測った v_d_meas を使う
#define SYSID_E1T_I_MAX          1.0f     // 並進の積分の上限[V](E1V の SYSID_E1V_I_MAX 0.5 では、大きく回る本で上限近くまで使った)
// 中止(区間1〜4): 距離、|angle|(走り始めから)、|ω|、|V_L| か |V_R| が電池の電圧を越えた、前の壁センサーが障害物を見た
#define SYSID_E1T_ABORT_DIST_MM  1650.0f  // 11-02(9.11): 1800mm の向きに走るので 1.15m → 1.65m
#define SYSID_E1T_ABORT_OMEGA_DPS 1432.0f // 25rad/s
#define SYSID_E1T_OBSTACLE_TH    350.0f  // ad_fl・ad_fr の大きいほう(手かざしと同じしきい値。何 mm で越えるかは確かめていない)
#define SYSID_E1T_OBSTACLE_MS    5u

// V_d = 0 で走ったときの曲がり具合 κ₀ を測る(E1K0)。走り方は E1T と同じ状態機械(ff_test の turn_test)で、
// 1 加速(並進 PI + 旋回 FB) → 2 一定の速さで CRUISE_MS(始めに θ を 0 に入れ直す。旋回 FB あり。V_d の平均を v_d_meas に)
// → 3 旋回 FB を切り V_d = 0 ちょうど(V_d0 も足さない。並進 PI は残す)を組ごとの時間 → 4 θ を 0 に入れ直して FB を戻し、
// SYSID_E1T_DECEL_MM_S2 で止まる → 5 0V で止まるまで。1800mm の向きに、右の壁ぎわ(約 80mm)から出る(左に曲がる見込み)。
// 0.5 → 1.0 → 1.5m/s を ROUNDS 周(2周で各2本。時間があれば3周目)。ファイル名 e1k0_<v>mms_NNNN(例 e1k0_1000mms)。
// 中止は E1T と同じ(距離 SYSID_E1T_ABORT_DIST_MM、|angle| > ABORT_ANGLE_DEG、|ω|、電圧、障害物)。
#define SYSID_E1K0_SPEED_MM_S    { 500.0f, 1000.0f, 1500.0f }
#define SYSID_E1K0_ACCEL_MM_S2   { 3000.0f, 3000.0f, 5000.0f }
#define SYSID_E1K0_FREE_MS       { 1000u, 800u, 600u } // 区間3(V_d = 0)の長さ
#define SYSID_E1K0_CRUISE_MS     100u
#define SYSID_E1K0_ROUNDS        3u
#define SYSID_E1K0_ABORT_ANGLE_DEG 80.0f
// 11-02(9.15): 走る前に SET を選ぶ(1 = 上の速さの組、2 = V_d を振る組)。V_d を振る組は 1.0m/s・区間3 0.8s で、
// 区間3 の V_d だけを表の一定の値にする(V_d0 は足さない)。0V は基準で1本、ほかは2本ずつ(1周目の後に2周目)。
// 右に曲がる本もあるので横は真ん中から出る。ファイル名 e1k0_1000mms_<p|m><|V_d| の 0.01V>(例 e1k0_1000mms_m005)。
#define SYSID_E1K0_VD_SPEED_MM_S 1000.0f
#define SYSID_E1K0_VD_ACCEL_MM_S2 3000.0f
#define SYSID_E1K0_VD_FREE_MS    800u
#define SYSID_E1K0_VD_V          { 0.0f, -0.05f, -0.1f, -0.2f, -0.3f, -0.4f, -0.05f, -0.1f, -0.2f, -0.3f, -0.4f }

// b_ω を V_d の跳びで測る(E1W)。走り方は E1T と同じ状態機械(ff_test の turn_test、turn_seq_n = 4):
// 1 加速(並進 PI + 旋回 FB) → 2 直進 CRUISE_MS(FB あり。最後の MEAS_MS の V_d の平均を V_d0 に)
// → 3〜6 旋回 FB を切り、V_d = V_d0 + A、V_d0 − A、V_d0 + A、V_d0 を STEP_MS ずつ(並進 PI は残す)
// → 7 θ を 0 に入れ直して FB を戻し、SYSID_E1T_DECEL_MM_S2 で止まる → 8 0V で止まるまで。9 中止。
// 切り替えの瞬間の ω̇ の跳び Δω̇ と V_d の跳び ΔV_d から b_ω = ΔV_d / Δω̇。7 × 10 マスで 1800mm の向きに、横は真ん中から出る。
// 速さごとに A = 0.2V を3本 → 0.4V を3本(A の数は速さごと)。走る前に SET を選ぶ:
//   1 = 1.0 → 0.5 → 1.5m/s、2 = 0.3 → 0.75 → 2.0m/s(11-03 に足した)、3 = 速さを1つ選ぶ。
// 2.0m/s は区間を 75ms、直進を 0.1s(V_d0 は最後の 50ms)にして、合計を約 1.60m に収めた(中止の 1.65m まで約 50mm)。
// ファイル名 e1w_<v>mms_a<A の 0.01V>_NNNN(例 e1w_1000mms_a020)。並進の積分の上限は SYSID_E1T_I_MAX。
#define SYSID_E1W_SPEED_MM_S     { 1000.0f, 500.0f, 1500.0f, 300.0f, 750.0f, 2000.0f } // SET 1・2 ではこの順
#define SYSID_E1W_ACCEL_MM_S2    { 3000.0f, 3000.0f, 5000.0f, 3000.0f, 3000.0f, 5000.0f }
#define SYSID_E1W_CRUISE_MS      { 200u, 200u, 200u, 200u, 200u, 100u }
#define SYSID_E1W_MEAS_MS        { 100u, 100u, 100u, 100u, 100u, 50u }
#define SYSID_E1W_STEP_MS        { 100u, 100u, 100u, 100u, 100u, 75u }
#define SYSID_E1W_N_A            { 2u, 2u, 2u, 2u, 2u, 1u } // SYSID_E1W_A_V の先頭から何個使うか(2.0m/s は 0.2V だけ)
#define SYSID_E1W_SET_SIZE       3u // SET 1 は表の 0〜2 番、SET 2 は 3〜5 番
#define SYSID_E1W_A_V            { 0.2f, 0.4f }
#define SYSID_E1W_REPEAT         3u
// 中止(区間1〜7): 距離、|angle|(走り始めから)、|ω|、障害物(SYSID_E1T_OBSTACLE_*)。E1T と同じく電圧も見る
#define SYSID_E1W_ABORT_DIST_MM  1650.0f
#define SYSID_E1W_ABORT_ANGLE_DEG 45.0f
#define SYSID_E1W_ABORT_OMEGA_DPS 573.0f // 10rad/s

#endif
