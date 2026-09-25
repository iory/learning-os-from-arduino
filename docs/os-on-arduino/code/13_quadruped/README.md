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
| `stop` | ポリシーを切り離して home 姿勢を保持 |
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
| `include/quad_calib.h` | ID / sign / zero。本書の通りに組み、`quad_host.py zero` でゼロ点をサーボに書けば変えなくてよい |
| `host/export_quad_policy.py` | 学習結果から上の 2 つを書き出す |
| `arduino_quad_policy.json` / `.npz` | ホスト側が読む同じポリシー。C ヘッダと同じ重み |
| `rl/` | 方策の学習コード（mjlab に被せる overlay と、GPU なしで回す `rl/cpu/`） |
| `reference/robot_and_sim2real.md` | 機体仕様と sim2real の設計メモ（実測値の出どころ） |
