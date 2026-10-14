# krs_quad_robot（KRS 版四脚の機体モデル）

`docs/ja/source/hardware/krs.md` の第 2 版の胴体で組んだ KRS 版四脚の URDF です。
SolidWorks のアセンブリ（create-assembly-view リポジトリの `cad/quad_robot_krs.SLDASM`）から
sw2robot で書き出しました。脚と足先のサブアセンブリを展開してあるので、
**股 4・膝 4 の 8 関節**が revolute になっています。Arducam（カメラ）は入っていません。

これは CAD から書き出したままのモデルです。学習に使う前に、Feetech 版の
`arduino_os_quad_robot/scripts/normalize_urdf.py` と `build_mjcf.py` と同じように、
関節名・軸の向きの正規化と、下の質量の置き換えをしてください。

## 座標系

`base_link` は胴体（body_krs）の部品座標系そのままです。

- **前は −x**、上は +z。右は +y（前を向いて右）
- z = 0 が胴体の底面。股の軸は x = ±66 mm
- q = 0 は CAD の姿勢で、**脚がまっすぐ真下を向いた姿勢**。実機ではサーボの真ん中
  （パルス 7500）に当たります（ホーンをこの向きで付けるため）

## 関節とサーボ ID

| URDF の関節（先頭） | 位置 | サーボ ID |
|---|---|---|
| `body_servo_krs_1__KRS3300_3__…` | 右前の股 | 1 |
| `leg_right_krs_2__…` | 右前の膝 | 2 |
| `body_servo_krs_1__KRS3300_4__…` | 左前の股 | 3 |
| `leg_left_krs_1__…` | 左前の膝 | 4 |
| `body_servo_krs_1__KRS3300_1__…` | 右後の股 | 5 |
| `leg_right_krs_1__…` | 右後の膝 | 6 |
| `body_servo_krs_1__KRS3300_2__…` | 左後の股 | 7 |
| `leg_left_krs_2__…` | 左後の膝 | 8 |

関節の軸は、右の脚が +y、左の脚が −y を向いています（書き出したまま。正規化で揃える）。

## 関節の範囲とアクチュエータ（KRS-3302 ICS）

URDF の `<limit>` は次の値に書き換えてあります。出典は近藤科学の商品ページ
<https://kondo-robot.com/product/03146> です。

| 項目 | 値 | URDF |
|---|---|---|
| 最大トルク | 6.7 kgf·cm（7.4 V） / 5.5 kgf·cm（6.0 V） | `effort="0.657"`（7.4 V の値、N·m） |
| 最高速度 | 0.16 s/60°（7.4 V） / 0.20 s/60°（6.0 V） | `velocity="6.545"`（rad/s） |
| 動作角度 | 270°（真ん中から ±135°） | `lower="-2.356194" upper="2.356194"` |
| 質量 | 26.4 g | — |
| ギア比 | 262.92:1 | — |
| 使用電圧 | 6〜7.4 V | — |

電源は Ni-MH 7.2 V（ROBO パワーセル E）か AC アダプター 5.9 V なので、実際のトルクは
5.5〜6.7 kgf·cm の間です。Feetech 版の STS3215（12 V、2.94 N·m）の 1/4 以下しかありません。

## 質量

URDF の質量は SolidWorks の材料の密度から出した値（合計 0.704 kg）で、実物とは違います。
電池・基板も入っていません。次の値に置き換えてください。

| 部品 | 質量 | 出典 |
|---|---|---|
| KRS-3302 ICS ×8 | 26.4 g ×8 | <https://kondo-robot.com/product/03146> |
| 胴体（body_krs） | 125.3 g 以下 | Bambu Studio のスライス（H2D、PLA Matte、サポート込み） |
| 蓋（lid_krs） | 41.3 g | Bambu Studio のスライス（X1C、PLA Matte、サポートなし） |
| 脚リンク（leg_link1_krs）×4 | 18.3 g ×4 | Bambu Studio のスライス（4 個で 73.22 g） |
| 脚ブラケット（bracket_outline_krs）×4 | 10.9 g ×4 | Bambu Studio のスライス（4 個で 43.4 g） |
| ROBO パワーセル E 6N-800（48 × 32.5 × 22 mm） | 78 g | <https://kondo-robot.com/product/02335> |
| RCB-4mini | 7.6 g | <https://kondo-robot.com/product/rcb4_mini>（35 × 30 × 12 mm） |
| Arduino UNO R4 WiFi | 25 g | **仮定**（公式の質量の記載なし。UNO R3 の値） |

URDF に無い部品の置き場所（`base_link` 座標、mm、中心の位置）:

| 部品 | 中心 | 根拠 |
|---|---|---|
| バッテリー | およそ (4, 21, 14) | Arduino の下、床（z = 3）に平置き。**目安** |
| Arduino UNO R4 WiFi | (3.8, 21.4, 33) | 基板 x −30.5〜38.0、y −5.2〜48.1、下面 z = 30 |
| RCB-4mini | (23, −22.5, 32) | 基板 x 5.5〜40.5、y −37.5〜−7.5、下面 z = 26 |

Arduino とバッテリーは +y（右）に寄っています。
