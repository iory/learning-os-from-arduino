# 仮想 UNO R4 WiFi（実機なしで動かす）

実機と**同じファームウェア**を PC の中で動かし、12×8 LED マトリクス・
内蔵 LED (D13)・シリアルコンソールをブラウザに出します。

```
pio run -e sim   →   firmware.elf   →   QEMU (RA4M1)   →   ブラウザ
  （いつもの        （実機に書くものと     （実機と同じ        （基板写真の上で
    ビルド）          同じ ELF）           Cortex-M4）         LED が光る）
```

サンプルコードは 1 行も変えません。実機向けビルドとの差は
`-D NO_USB`（Arduino コア公式のフラグ）だけです。

## 必要なもの

- **QEMU（`arduino-uno-r4` マシン入り）** —
  <https://github.com/iory/qemu-arduino-uno-r4/releases> から OS に合ったものを
  展開する（macOS arm64・x86_64 / Windows x64・ARM64 / Linux x86_64・arm64）。本家の
  QEMU には RA4M1 が無いので、Homebrew や apt の `qemu-system-arm` では動かない
- **Python 3.10 以降** — 標準ライブラリだけを使います（pip 不要）
- **PlatformIO** — 本編と同じもの

Renode（<https://github.com/renode/renode/releases>）でも動きます
（後述の「Renode で動かす」）。

## 使い方

```bash
cd ../04_scheduler
pio run -e sim                       # シミュレータ用にビルド
python3 ../sim/board.py --chapter .  # 仮想ボードを起動
```

`http://127.0.0.1:8080` を開くと基板が出ます。シェルのある章
（第5章以降）は、画面下の入力欄にコマンドを打てばそのまま動きます。
終了は Ctrl-C。

QEMU は `--qemu` → 環境変数 `QEMU` → `~/qemu-unor4`（Windows の zip のように
`~/qemu-unor4/<名前>/bin/` でも可）→ PATH の順に探し、`arduino-uno-r4` マシンが
あるものを使います（`emulator_process.find_qemu()`）。見つからないときは、探した
場所とダメだった理由を出して止まります。

- `-icount shift=4,sleep=on`（1 命令 16ns ≈ 48 MHz 相当）で走らせます。付けないと
  CPU がホストの速さで回り、タスクの印字が実機よりずっと頻繁に混ざります
  （`--icount ""` で外せる）
- `Serial` は SCI9 に出ます。実機でも `-D NO_USB` の `Serial`（`_UART1_`）は
  P109/P110 = SCI9 です

## Renode で動かす

`--emulator renode` で Renode を使います。

```bash
python3 ../sim/board.py --chapter . --emulator renode
python3 ../sim/board.py --chapter . --emulator renode \
    --renode /Applications/Renode.app/Contents/MacOS/renode   # 環境変数 RENODE でも同じ
```

macOS は `brew trust renode/tap` してから `brew install renode/tap/renode` も可
（最近の Homebrew は公式以外の tap を既定で信用せず、`brew trust` が無いと
`Refusing to load formula ... from untrusted tap` で止まります）。

QEMU との違いは **第6章のフォールトが起きない** ことです。Renode の Armv7-M は
`CCR.DIV_0_TRP` を実装しておらず、ゼロ除算で UsageFault になりません。未マップ領域
(`0xFFFFFFFF`) への書き込みも、警告を出すだけで BusFault を上げません。どちらも
タスクはそのまま終了するので、「壊れたタスクだけが死に、他は動き続ける」ことは
観察できますが、`FAULT DETECTED` の表示と原因の特定は見られません。

## 動く章

| 章 | 状態 |
| --- | --- |
| 00_intro 〜 11_tiny_python、adv1〜adv3 | 動く（`pio run -e sim` を用意済み） |
| 12_hardware | ✗ センサ・アクチュエータが必要 |
| 13_quadruped | ✗ サーボと四脚ロボットが必要 |
| adv4_fs | ✗ Renesas FSP のフラッシュドライバ (`r_flash_lp.h`) が要る |
| 10_freertos_real | ✗ 環境が 6 つあるため未対応（必要なら個別に `-D NO_USB` を足す） |

第1章は実機と同じく起動時に `S` の入力を待ちます。ブラウザの入力欄に
`S` と打ってください。

## 自動検証に使う

章ごとの期待値（本文の「期待される出力」から起こしたもの）は
`scripts/verify_chapters.py` にあります。実機の代わりにエミュレータで
同じ検証ができます。

```bash
cd ..
uv run python scripts/verify_chapters.py --sim                  # 全章（QEMU）
uv run python scripts/verify_chapters.py --sim --only 05_shell
uv run python scripts/verify_chapters.py --emulator renode      # Renode で
```

ボードも書き込みも要らないので、CI で回せます
（`.github/workflows/simulator.yml`。QEMU が主、Renode は壊れていないかの確認）。
遅いランナーでは `--capture-scale 2` で捕捉時間を伸ばしてください。

エミュレータでは再現しない期待値には `Check(unsupported_on=("renode",))` の
ように確かめられないエミュレータを書いてあり、結果に `SKIP` と理由が残ります。
黙って飛ばすと「通った」が嘘になるためです。第6章のフォールトは Renode では
SKIP、QEMU では確かめます。

ログは `scripts/results/sim-qemu/`（QEMU）と `scripts/results/sim/`（Renode）に
分けて残ります。

## 動きを GIF に録る

```bash
uv run --no-project --with pillow python record.py \
    --chapter ../04_scheduler --out demo.gif --seconds 8

# シェルのある章はコマンドを送ってから録れる
uv run --no-project --with pillow python record.py \
    --chapter ../05_shell --send ps --send "kill 3" --seconds 10
```

（エミュレータの選び方は board.py と同じ。`--emulator renode` も使えます）

ブラウザを使わず、エミュレータから読んだ LED の状態を基板写真に合成します
（CI に Chrome を入れずに済みます）。`--width` `--fps` `--colors` で
大きさと滑らかさを調整できます。

## 実機との違い

エミュレータなので、ここだけは実機と違います。**本文の説明と食い違う
場面があるので、気づいたときはこの節を思い出してください。**

- **`Serial` が USB ではなく UART** — エミュレータには USB が無いので、
  `-D NO_USB` で `Serial` をハードウェア UART（SCI9）に切り替えています。
  実機で `Serial` が USB CDC である話（第5章）は、実機で確かめてください。
- **ESP32 / Wi-Fi が無い** — UNO R4 WiFi のもう 1 つのチップは載っていません。
- **時間は実時間に近いが、割り込みのジッタは本物ではない** — 実時間性の
  議論（第4章・第9章）は、最後は実機で確かめてください。
- **本書で使わない周辺回路は無い** — QEMU が持っているのは AGT・SCI・GPIO・
  ICU（割り込みコントローラ）だけです。ADC・I²C・SPI・GPT などはありません。
- **（Renode のみ）第6章のフォールトは起きない** — 上の「Renode で動かす」を参照。

## 仕組み

`board.py` はエミュレータを起動して、2 本の口で会話しています。

| 見えるもの | 取得元 |
| --- | --- |
| 12×8 LED マトリクス | `Arduino_LED_Matrix` の `framebuffer`（12 バイト）をゲストのメモリから読む |
| L (D13) | ポートレジスタ `PCNTR1` (0x40040020) の PODR bit2 |
| シリアル | QEMU の TCP シリアル／Renode のソケット端末（双方向） |

メモリとレジスタは、QEMU では QMP の `xp`、Renode では Monitor の
`sysbus ReadDoubleWord` で読んでいます。

マトリクスは、実機ではチャーリープレクスで 96 個を高速に走査していますが、
その元データである 12 バイトのフレームバッファをそのまま読んでいます。
ELF からシンボルを引くので、章ごとの設定は要りません。

ブラウザへは Server-Sent Events で流しています。外部ライブラリはゼロです。

エミュレータ（Renode / QEMU）は `emulator_process.py` で自分専用のプロセスグループに入れて起動し、
止めるときはグループごと止めます。Homebrew 版の `renode` はシェルスクリプトが
`dotnet` を子として起動するので、シェルだけを止めると本体が親を失って全速で
走り続けるためです。Ctrl-C・端末を閉じる・`kill` のどれでも止まります。
`kill -9` だけは後始末ができないので、そのときは `pkill -f qemu-system-arm`
（Renode なら `pkill -f Renode`）で止めてください。

## 基板の画像を差し替える

`board_off.jpg` は、サポートサイトの基板写真から「LED マトリクス消灯」の
状態を作ったものです。別の写真に差し替えるときは:

```bash
uv run --no-project --with pillow --with numpy --with scipy \
    python make_board_image.py --photo /path/to/photo.jpg \
    --out board_off.jpg --cells matrix_cells.json
```

LED の位置は写真から自動検出します（96 個見つかれば成功）。ただし
`make_board_image.py` の `AFFINE` と `LED_D13` は元の写真に合わせた値なので、
アングルが変わる場合は当てはめをやり直す必要があります。

## ファイル

| ファイル | 役割 |
| --- | --- |
| `board.py` | エミュレータ（Renode / QEMU）の起動、メモリ／レジスタの読み出し、HTTP + SSE サーバ |
| `board.html` | 基板写真の上に LED を重ねる画面 |
| `unor4_board.repl` | Renode のプラットフォーム定義（同梱の RA4M1 をそのまま使う） |
| `board_off.jpg` | 消灯状態の基板写真 |
| `matrix_cells.json` | LED 96 個 + D13 の位置（パーセント） |
| `make_board_image.py` | 写真から上の 2 つを作り直すスクリプト |
| `record.py` | 動きを GIF に録るスクリプト（Pillow が要る） |
| `emulator_process.py` | エミュレータを孤児にしない起動・停止（上の 3 つと検証スクリプトが使う） |
