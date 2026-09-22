# 08_interpreter_v2 — 第8章に 07_led_matrix_v2 の表示と top を入れる

第8章 `08_interpreter` の改良版。**書籍には載っていない**（本文と掲載コードは
`08_interpreter` のまま）。インタプリタ・カーネル・フォルト処理は同じで、変えたのは
LED マトリクスの表示（`os_display.cpp` / `os_display.h`、`07_led_matrix_v2` と同じ）、
シェルに足した `top`、棒を動かすために足した 2 つのタスク（Light / Heavy）だけ。

## 何が変わるか

マトリクスを「いまの値の横棒」にした。どの棒も **12 個で 100%**、0.5 秒ごとに測り直す。

| 行 | 中身 |
|---|---|
| 0–1 | CPU 全体の負荷（idle 以外）。太い棒 |
| 2 | 空き（区切り） |
| 3–7 | LED / Shell / Display / Light / Heavy の CPU 使用率 |

第8章のタスク（LED・Shell・Display）はどれもほとんど眠っていて、棒が伸びない。
そこで `07_led_matrix_v2` と同じ Light（自分の CPU 時間を 20 ms 使っては 60 ms 眠る）と
Heavy（眠らずに計算し続ける）を足した。書籍の 3 つの後ろに作るので、LED (1) /
Shell (2) / Display (3) の ID は第8章と同じ。

`08_interpreter` の表示は第7章と同じもので、負荷の履歴が時間の順に並んでいない・
下半分がほとんど光らない、という読みにくさも同じだった（`07_led_matrix_v2/README.md`）。

## 書き込む

```
pio run -d 08_interpreter_v2 -t upload
pio device monitor -d 08_interpreter_v2 -b 115200
```

## 試す

```
> top
CPU [########    ] 60%

ID  NAME      STATE     CPU%  MATRIX
0   idle      READY     40%
1   LED       BLOCKED    0%
2   Shell     RUNNING    4%  #
3   Display   BLOCKED    0%
4   Light     READY     16%  ##
5   Heavy     READY     40%  #####

> kill 5
```

| 操作 | CPU 全体 | 目に見えること |
|---|---|---|
| 起動直後 | 約 60% | Light と Heavy の行に棒 |
| `kill 5` | 約 24% | Heavy の棒が消え、全体の棒が縮む |
| `kill 4` | 約 4% | 残るのは `top` を描いている Shell だけ |
| `exec 4` / `exec 5` | 元に戻る | |

値は UNO R4 WiFi の実機で `top` を読んだもの。第8章のもとのタスクがほとんど CPU を
使わないのは、LED タスクの反転もインタプリタの `FORWARD` / `TURN` / `DELAY` も
`os_sleep()` で眠って待つから（眠っている間 CPU はほかのタスクに回る）。

- **スクリプトの実行中（`run`）は `top` の表示が止まる。** 描き直しはシェルが入力を
  待つループの中で行うので、`interp_run()` が終わるまで更新されない。マトリクスは
  表示タスクが描くので、その間も更新される
- 第8章の `kill 1` → `run LED 13 1`（LED タスクを止めてから L を点ける）はそのまま動く
