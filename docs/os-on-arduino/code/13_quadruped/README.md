# 13_quadruped — 8 自由度四脚を歩かせる

第13章の付属コード。**実機で動いているものと同じ**で、シミュレータに対して
検証した経路をそのまま置いてある。

## 部品表

**[BOM.md](BOM.md) に、通販コードと価格つきの部品表がある。** 3D プリント部品の
STL・印刷設定・出力代行の選択肢もそこにまとめてある。

## 必要なもの

| | |
|---|---|
| マイコン | Arduino UNO R4 WiFi |
| サーボ | Feetech STS3215 (12 V) ×8 |
| バス | 半二重の方向切替をするアダプタ。**D0/D1 に直結しても応答は返らない** |
| 電源 | サーボはバッテリから。マイコンの 5 V からは取らない (8 個 × ストール 2.7 A) |

GPU は要らない。学習済みのポリシーは `include/arduino_quad_policy.h` に入っている。

## 書き込む

```
pio run -d 13_quadruped
pio run -d 13_quadruped -t upload
pio device monitor -b 115200
```

実測: **flash 45.2% / RAM 14.4%** (UNO R4 WiFi、WiFi スタック無し)。
ポリシーの重み 59.3 KB は `static const float` なので flash に置かれ、RAM を食わない。
int8 量子化は要らない (本文 13.7.1)。

## シリアルのコマンド

| | |
|---|---|
| `iktest` | FK/IK の往復テスト。**脚は動かない**ので最初にこれ |
| `stand` | home 姿勢へ 2 秒かけて立ち上がり、保持する |
| `trot` | naive 歩行 (本文 13.5) |
| `rl <vx> <wz>` | 学習済みポリシーで歩く。例 `rl 0.15 0`。歩いている最中に打つと速度だけ変わる |
| `drive <vx> <wz>` | `rl` と同じだが、300 ms 途切れると home 保持に戻る。`host/quad_host.py serial` が送り続ける |
| `stop` | ポリシーを切り離して home 姿勢を保持。IMU 版では速度 0 のポリシーで立ち、押されると踏ん張る |
| `imu` | (IMU 版だけ) 胴体座標の角速度・重力の向きと、関節の目標角を出す |
| `zero` | 全関節を 0 度 (脚をまっすぐ) へ移して保持。吊るして使う |
| `free` | トルクを切る |

毎秒この行が出る。**実機での性能証拠はこれだけ。** 本書の実機で `rl 0.15 0` のとき:

```
loop avg 5770 us  max 5797 us  late 0/51  skipped 0  read_fail 0
```

| | 正常な値 |
|---|---|
| `loop avg` | 本書の実機で `rl` 中 5800 前後、保持中 2400 前後。20000 (50 Hz) に収まればよい |
| `late` | 0 のまま |
| `skipped` | 0 のまま増えない。増えるなら締切を落としている |
| `read_fail` | 0。増えるならバスか配線 |

## 立ち上げの順序

**この順でやること。** 飛ばすと機体が壊れる。

1. `iktest` — 脚を動かさずに式を確かめる
2. サーボに ID 1..8 を振る。**どの ID をどの関節に付けるかはサポートサイトの
   組み立て手順の表の通り**（`include/quad_calib.h` の `QUAD_SERVO_ID` と同じ）。
   振り方は [feetech-cli](https://github.com/iory/feetech-cli) の `feetech set-id`
3. **ゼロ点をサーボに書く。** ジャンパを B にして PC から
   `host/quad_host.py zero`。4 本とも脚をまっすぐ真下に伸ばした姿勢 (CAD の 0 度)
   を、各サーボの EEPROM に 2048 として書く。`sign` は脚の組み方で決まるので、
   本書の通りに組めば `quad_calib.h` は書き換えなくてよい
4. 機体を吊るして `stand`。4 本とも同じ格好になること。1 本だけ鏡像なら
   その脚の ID か組み方が違う
5. 吊るしたまま `rl 0.06 0`。`late` と `skipped` を見る
6. 床に下ろす

## IMU 版

胴体前面のセンサ基板の IMU (QMI8658) の角速度と重力の向きも観測に使うポリシーで
歩かせる版。基板は UNO R4 WiFi の Qwiic コネクタ (Wire1) につなぐ。env の名前に
`_imu` を付けると、`include/arduino_quad_policy.h` の代わりに
`include/arduino_quad_policy_imu.h` のポリシーが入る。

```
pio run -d 13_quadruped -e uno_r4_wifi_rcb4_imu -t upload   # RCB-4
pio run -d 13_quadruped -e uno_r4_wifi_imu -t upload        # Feetech
```

`_imu` の無い env は IMU を使わないポリシーのままで、IMU のコードも入らない。
WiFi 版は `uno_r4_wifi_udp_imu` / `uno_r4_wifi_rcb4_udp_imu`。
実機で確かめたのは RCB-4 の USB シリアル版 (`uno_r4_wifi_rcb4_imu`) と WiFi 版
(`uno_r4_wifi_rcb4_udp_imu`)。WiFi 版でも、PC から home 保持の指令 (キーボードの
スペース) を受けると balance になる。

- 起動直後の約 10 秒でジャイロのゼロ点を取る。**この間は機体を動かさない。**
- IMU が見つからない・校正に失敗したときは `!! IMU 無しで動かす` と出て、
  ポリシーには水平に止まっている値 (角速度 0、重力 `0 0 -1`) を渡す。
  毎秒の行の最後に `imu=level` が付く。平らな床なら歩けるが、傾いても押されても
  ポリシーには伝わらない。歩行中に読めなかった周期も、その周期だけ同じ値を渡して
  `imu_fail` を数える
- `stop` は関節を固定せず、ポリシーを速度 0 で回し続ける (`mode=balance`)。
  押されると IMU を見て踏ん張る。関節を固定して保持するなら `stand`。
  IMU が無いときの `stop` は `stand` と同じ

RCB-4 版の実機で Qwiic を抜いて起動し、床で `rl 0.1 0` を 10 秒送ると、
IMU があるときと同じように歩いた (`imu=level`、1 周期は平均 9.2 ms・最大 11.3 ms)。

### 確かめる順序

立ち上げの順序の 5 の前に、この順で確かめる。

1. 床に置いて `imu`。`body g` が `0 0 -1` に近いこと
2. 頭を下げて `imu`。`body g` の 1 つ目 (x、前) が正になること。
   右側を下げて `imu`。2 つ目 (y、左) が負になること。
   逆なら `src/quad_imu.cpp` の `R_BODY_CHIP` (基板の取り付けの向き) が違う
3. **持ち上げて足を床から離し**、水平に持って `stop`。そのまま `imu` を打ち、
   頭を下げる・右を下げるたびに `imu` を打つ。傾けると脚が動き、`target rad`
   が変わること。変わらなければ IMU の値がポリシーに届いていない。
   空中では足が床に着かないので、動き方そのものに意味はない。届いているかだけを見る

RCB-4 版の実機で 3 をやったときの `target rad`
(並びは FL_hip, FL_knee, RL_hip, RL_knee, RR_hip, RR_knee, FR_hip, FR_knee):

| 持ち方 | `body g` | `target rad` |
|---|---|---|
| 水平 | -0.04 0.00 -1.00 | 1.11 -1.77 1.28 -1.76 1.27 -1.73 1.17 -1.77 |
| 頭を約 45° 下げる | 0.71 -0.01 -0.71 | 1.10 -1.16 0.57 -1.59 0.94 -1.73 0.54 -1.81 |
| 右を約 50° 下げる | 0.04 -0.78 -0.63 | 0.98 -1.41 0.83 -1.52 0.98 -1.77 0.87 -1.63 |

## 近藤の ICS サーボで組む (RCB-4 mini 版)

サーボを近藤の ICS サーボにし、RCB-4 mini を挟んで動かす版。`servo_bus.h` の
関数だけを差し替えるので、`main.cpp`・ポリシー・シリアルのコマンドは同じものが走る。

```
pio run -d 13_quadruped -e uno_r4_wifi_rcb4 -t upload
```

| | |
|---|---|
| 配線 | RCB-4 の COM コネクタ (GND 側から GND - Rx - Tx) に UNO の D9 → Rx、D8 ← Tx、GND を共通に。アダプタは要らない。D0/D1 (Serial1) は使わない |
| 通信 | 反転 UART 1.25 Mbps 8E1。UNO R4 の Serial1 は反転できないので、`src/rcb4_link.cpp` が GPT と DMA で D8/D9 を直接扱う |
| ID | Feetech 版と同じ振り方 (`QUAD_SERVO_ID`)。RCB-4 のサーボ番号は右の SIO ポートなら ID × 2、左なら ID × 2 + 1 で、どちらに挿したかは起動時に両方を調べて見つける |
| 割り当て | RCB-4 は HTH4 のプロジェクトで割り当てた番号のサーボとしか話さない。割り当てが無ければ起動時に RAM へ書いて足す (目標位置を脱力にしてから)。ROM のプロジェクトは書き換えない |
| 校正 | **`include/quad_calib_rcb4.h` を埋めるまでは読むだけで、目標角は送らない。** 吊るして脚をまっすぐにし、`raw` で出たパルスをゼロ点に書く。向きも `raw` で確かめてから `QUAD_RCB4_CALIBRATED` を 1 にする |

毎秒の行の最後に `write_fail` が付く。RCB-4 は書き込みにも ACK を返すので、
返らなかった回数を数えている。RCB-4 には Feetech の速度上限レジスタに当たる
ものが無いので、起動時の「速度設定」は書かない。

8 個をつないだ実機 (未校正なので読むだけ) で、1 周期は平均 5.4〜5.7 ms・
最大 11.5 ms (`late 0`、`read_fail 0`)。RCB-4 の返事は 1 往復に約 0.6 ms
かかるので、関節ごとに 2 バイトずつ読んでいる。RCB-4 は自分のサーボ周期の
最中だと答えないことがあり、1 関節 2 回の試行では `read_fail` が毎秒 1〜4 出た
ので 4 回にしてある。

校正したあと吊るして `rl 0.06 0` を送ると足踏みし、1 周期は平均 8.8 ms・
最大 10.7 ms (`late 0`、`read_fail 0`、`write_fail 0`)。`quad_host.py serial` の
キーボード操作でも動いた。床に下ろし、USB を抜いてバッテリーだけで歩くことも
確かめた (このときの loop の値は取っていない)。

電源は近藤科学の ROBO パワーセル E タイプ (Ni-MH 6 セル、公称 7.2 V、800 mAh、
02335) 1 本で足りる。KXR アドバンスセット B Ver.2 に入っているもので、充電は BX-32MH。
RCB-4mini の SIO ポートの Vcc は電源電圧そのままなので、空いている SIO ポートの Vcc と
GND を UNO の VIN と GND へつなぐ (信号線はつながない)。UNO の VIN は 6〜24 V。
床で歩かせても、サーボの電流で UNO がリセットされることはなかった。電池が減って 6 V を
切ると VIN の範囲を外れるので、その前に充電すること。

## WiFi で動かす

USB ケーブルの代わりに WiFi (UDP) で速度を送る版。`src/main.cpp` の代わりに
`src/main_wifi.cpp` が走る。ポリシーと 50 Hz のループはどちらもマイコンの上で、
PC は速度を送るだけ。Feetech 版と RCB-4 版のどちらでも使える。

```
cp include/arduino_secrets.example.h include/arduino_secrets.h   # 接続先を埋める
pio run -d 13_quadruped -e uno_r4_wifi_udp -t upload        # Feetech
pio run -d 13_quadruped -e uno_r4_wifi_rcb4_udp -t upload   # RCB-4
cd host && uv run python quad_host.py wifi
```

`arduino_secrets.h` は `.gitignore` に入っている。

UNO R4 WiFi の RA4M1 と ESP32-S3 の間は 115200 baud 固定で、WiFi のパケット 1 つを
読むのに ESP32-S3 への問い合わせが 2 回 (1 回約 5 ms) かかる。届いていなくても
見に行くだけで約 4.5 ms かかる。WiFi を見に行くのは 2 周期に 1 回で、1 回に読む
パケットは 1 個だけにしている (溜まった分をまとめて読むとループが最大 160 ms
止まった)。PC へ返事を送る周期 (500 ms ごと) がいちばん長く、約 50 ms かかる。

通信が途切れても、最後に受けた速度で 5 秒は歩き続け、5 秒途切れたら止まる。
WiFi はパケットが数百 ms〜数秒まとめて遅れることがあり、見張りを 300 ms にしていた
ときは歩行中に止まっては歩き出すのを繰り返してカクついた。その代わり、PC が
落ちても 5 秒は歩き続ける (0.1 m/s なら約 50 cm)。

RCB-4 + IMU 版 (`uno_r4_wifi_rcb4_udp_imu`) で床に置き、PC から 10 Hz で
vx=0.1 を 4 秒送って止めると、1 周期は最大 50 ms、20 ms を超えたのは指令が
届いている間の 1 割ほどで、なめらかに歩いて balance に入った。PC が送るのを
やめた後の 1 周期は最大 16 ms。

## ホスト側 (`host/`)

マイコンに焼かずに、ノート PC から USB でサーボを直接叩く経路。立ち上げと校正は
こちらのほうが早い。全部の値が画面に出るし、直すたびに焼き直さなくてよい。

```
cd host
uv sync                                  # numpy と feetech-cli
uv run python quad_host.py scan          # バスに何個いるか
uv run python quad_host.py zero          # 脚をまっすぐにした姿勢をゼロ点にする
uv run python quad_host.py stand         # home 保持
uv run python quad_host.py teleop        # キーボードで walk (PC がポリシーを回す)
uv run python quad_host.py serial        # キーボードで walk (Arduino がポリシーを回す)
```

`teleop` はジャンパ B で、PC がポリシーを回してサーボを直接動かす。`serial` は
ジャンパ A で、Arduino に書き込んだこのファームへ USB シリアルで `drive` を送る。

`servo_bus.py` がサーボへの経路を差し替えられるようにしてあるので、同じ制御コードの
まま USB 直結 / Arduino ブリッジ / 無線を選べる (本文 13.10)。

## ファイル

| | |
|---|---|
| `src/leg_ik.cpp` | 平面 2 リンクの FK/IK (本文 13.3) |
| `src/leg_ik_test.cpp` | 往復テスト (本文 13.3.7) |
| `src/servo_bus.cpp` | 半二重シリアルバス。sync write と速度レジスタ (本文 13.2.2, 13.9.1) |
| `src/gait.cpp` | naive trot (本文 13.5) |
| `src/main.cpp` | 50 Hz ループ。締切を落としたら捨てて合わせ直す (本文 13.4.4) |
| `include/quad_control.h` | 観測の組み立てと行動の復号。**チェックポイントから生成** |
| `include/arduino_quad_policy.h` | 重みと観測レイアウト。**生成物、手で編集しない** |
| `include/arduino_quad_policy_imu.h` | IMU 版 (`_imu` の env) の重みと観測レイアウト。`export_quad_policy.py` が書いた `arduino_quad_policy.h` をこの名前にしたもの。ホスト側の `.json` / `.npz` は無い |
| `include/quad_calib.h` | ID / sign / zero。本書の通りに組み、`quad_host.py zero` でゼロ点をサーボに書けば変えなくてよい |
| `src/main_wifi.cpp` | WiFi 版の 50 Hz ループ。速度を UDP で受け取る (`-e uno_r4_wifi_udp` / `uno_r4_wifi_rcb4_udp`) |
| `include/arduino_secrets.example.h` | WiFi の接続先の雛形。`arduino_secrets.h` に写して埋める |
| `src/servo_bus_rcb4.cpp` | `servo_bus.h` の RCB-4 mini 版 (`-e uno_r4_wifi_rcb4`) |
| `src/rcb4_link.cpp` | RCB-4 の COM ポート (反転 UART 1.25 Mbps) を GPT + DMA で叩く |
| `include/quad_calib_rcb4.h` | RCB-4 版のサーボ番号 / sign / zero。埋めるまでサーボに何も送らない |
| `host/export_quad_policy.py` | 学習結果から上の 2 つを書き出す |
| `arduino_quad_policy.json` / `.npz` | ホスト側が読む同じポリシー。C ヘッダと同じ重み |
| `rl/` | 方策の学習コード（mjlab に被せる overlay と、GPU なしで回す `rl/cpu/`） |
| `reference/robot_and_sim2real.md` | 機体仕様と sim2real の設計メモ（実測値の出どころ） |
