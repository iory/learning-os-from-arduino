# host — ノート PC からサーボを叩く

マイコンに焼く前の立ち上げと校正はこちらが早い。全部の値が画面に出て、直すたびに
焼き直さなくてよく、Ctrl-C でトルクが抜ける。

```
uv run python quad_host.py scan        # バスに何個いるか、ボーレートは合っているか
uv run python quad_host.py calibrate   # sign と zero を測る -> calib.json
uv run python quad_host.py stand       # home 姿勢を保持
uv run python quad_host.py teleop      # w/s/a/d で歩かせる
uv run python quad_host.py stepid --joint FL_hip_joint --log step.csv
                                       # サーボモデルの同定用ログ (本文 13.9.3)
```

必要なもの: `pyserial`, `numpy`。USB のシリアルアダプタ 1 個。

| | |
|---|---|
| `quad_host.py` | 上のサブコマンド。50 Hz の制御ループもここ |
| `quad_policy.py` | ポリシーの numpy 実装。C 版と同じ観測ベクトルを作る (本文 13.7.4) |
| `servo_bus.py` | サーボへの経路。USB 直結 / Arduino ブリッジ / 無線を差し替える |
| `quad_link.py` | 無線のとき、速度指令を UDP で運ぶ (本文 13.10) |
| `calib.json` | `calibrate` が書く。**機体ごとに違う** |
| `export_quad_policy.py` | 学習結果から C ヘッダを書き出す |
| `quad_recorder.py` | 走行の録画と、指令を同じ時間軸で残す道具。本文では扱わない |
