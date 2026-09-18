# サンプルコードの検証スクリプト

## verify_chapters.py — 3 OS 共通の検証

全章をビルドし、実機に書き込み、**シリアル出力が本文の記載どおりか**を正規表現で
突き合わせる。Linux / macOS / Windows のいずれでも同じコマンドで動く。
章ディレクトリが並んでいるコードのルートで実行する。

```bash
uv sync                                             # 初回のみ
uv run python scripts/verify_chapters.py --list     # 章と期待値の一覧
uv run python scripts/verify_chapters.py --build-only    # ボード無し（CI と同じ）
uv run python scripts/verify_chapters.py            # 実機で全章
uv run python scripts/verify_chapters.py --only 01_boot 13_quadruped
uv run python scripts/verify_chapters.py --port COM5      # ポートを明示する
uv run python scripts/verify_chapters.py -i              # 対話モード（実機を見ながら）
```

### 対話モード（`-i` / `--interactive`）

1 章ずつ焼いて、**実機の LED を見ながら Enter で次へ進む**。LED やサーボの動きは
自動検査では見られないので、目視での確認はこちらを使う。

| 入力 | 動き |
|---|---|
| Enter だけ | 次の章へ |
| 文字を打つ | そのままボードに送る（`ps`、`kill 1` など） |
| `!` | その章の「試せること」を上から順に自動で送る |
| `r` / `b` / `q` | 焼き直し / 前の章へ / 終了 |

章ごとに「目で見るポイント」（`Chapter.watch`）と「試せること」（`Chapter.try_cmds`）を
表示する。たとえば第5章なら `kill 1` で L LED が止まり `exec 1` で復活すること、
第7章なら `kill 3` で CPU 負荷グラフが下がることを、画面の指示どおりに確かめられる。

`try_cmds` / `send` には `sleep:5` と書ける。第6章の「`d` を打って 3 秒後にクラッシュ」
のように、待ってから次のコマンドを送りたいときに使う。

Windows (PowerShell) でも同じ。

```powershell
cd docs\os-on-arduino\code
uv sync
uv run python scripts\verify_chapters.py
```

- ポートは VID (0x2341) で自動検出する。Linux は `/dev/ttyACM*`、macOS は
  `/dev/cu.usbmodem*`、Windows は `COMn` になる。取り違えるときは `--port` で指定する。
- 捕捉した生ログは `scripts/results/<章>.log` に残る。`--json` で結果を機械可読に出せる。
- 1 章でも失敗すると終了コードが 1 になるので、そのまま CI に載る。

### 期待値の書き方

`CHAPTERS` の各 `Chapter` に `Check(正規表現, 理由)` を並べる。環境で変わる値
（アドレス・時刻・ADC 値）は正規表現で幅を持たせ、**本文が主張している不変量**を
書くのが要点。たとえば第1章なら「BSS がゼロクリアされている」「DATA が Flash から
コピーされている」を、値そのもので確かめている。

`handshake="S"` は `setup()` でしか出力しない章向けで、出力が始まるまで `S` を
送り続ける。本書では `01_boot` / `01_boot_vector_dump` / `adv3_heap` / `adv4_fs`
がこの形（本文 1.10 のコラム参照）。

`send=[...]` は接続後に送るシェルコマンド。`custom` は正規表現では書けない検査で、
いまは 3 つある。

| custom | 何を確かめるか | 本文の該当箇所 |
|---|---|---|
| `cpu_sum` | `ps` の CPU% の合計が 100% を超えない | 4.7「合計がほぼ 100% になっているのが目印」 |
| `preemption` | 重い計算が回っていても 1 秒周期タスクの間隔が崩れない | 第4章の主張そのもの |
| `philosophers` | 5 人の哲学者が全員食事を進める（誰も飢えない） | 応用編1 のデッドロック回避 |

### env が複数ある章

`10_freertos_real` は掲載例ごとに env を分けてあるので、`Chapter(..., env="integrated")`
のように 1 つ選ぶ。`-e` を付けずに `-t upload` すると 6 個すべてを続けて書き込もうとして
失敗する（本文 10.3 も `pio run -e preemptive -t upload` と書いている）。

## 旧スクリプト（Linux 専用）

`flash_and_capture.py` と `run_all_chapters.py` は Ubuntu 機専用で、
`uhubctl` のハブ位置がハードコードされており、**出力の検証はしない**。
新規の検証には `verify_chapters.py` を使うこと。

`probe_serial.py` はポートの当たりを付けるだけの補助。

## 実測（2026-08-30, macOS / Arduino UNO R4 WiFi 実機）

全 20 プロジェクトが PASS。`--build-only` も 20/20。
