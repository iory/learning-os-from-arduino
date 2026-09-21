#!/usr/bin/env bash
# サポートサイトの章ページに載せる GIF を録り、check_gif.py で「見せたいことが
# 本当に見えているか」を確かめる。
#
#   sim/chapter_gifs.sh OUT_DIR              # 全部
#   sim/chapter_gifs.sh OUT_DIR ch05 ch07    # 章を絞る
#
# それぞれの --expect が、その GIF で読者に見せたいこと（verify_chapters.py の
# watch= と同じもの）。NG が出た GIF は載せない。
#
# 載せていない章:
#   03  L とマトリクスを空ループで点滅させるので、実機でも 10〜20ms ごとに切り替わる。
#       GIF のコマでは点滅に見えない
#   09  ボタンとサーボで動く章なので、シミュレータではマトリクスの表示が変わらない
set -u
cd "$(dirname "$0")/.."
OUT=$(mkdir -p "${1:?出力先のディレクトリ}" && cd "$1" && pwd)
shift
SELECT="${*:-ch04 ch05 ch06 ch07 ch08 ch11}"
FAILED=""

rec() { uv run --no-project --with pillow python sim/record.py --width 480 --colors 96 "$@"; }
chk() { uv run --no-project --with pillow python sim/check_gif.py "$@"; }
want() { for w in $SELECT; do [ "$w" = "$1" ] && return 0; done; return 1; }
run() {   # run 名前 "record.py の引数" "check_gif.py の引数"
    local name=$1 rec_args=${2//$'\n'/ } chk_args=${3//$'\n'/ }   # 改行は区切りにしない
    echo "=== $name"
    if ! eval rec --out "$OUT/$name.gif" --meta "$OUT/$name.json" "$rec_args" \
        || ! eval chk "$OUT/$name.json" "$chk_args"; then
        FAILED="$FAILED $name"
    fi
}

# 第4章: Heavy が CPU を回していても L と左半分の点滅が乱れない（L は 100ms ごと）。
# 100ms の状態をコマに割るので、GIF の 10ms 単位で割り切れる 50fps で録る
want ch04 && run ch04 \
    '--chapter 04_scheduler --seconds 10 --fps 50' \
    '--expect "all:d13=blink=200" --expect "all:left=blink" --expect "all:right=blink"'

# 第5章: kill 1 で L だけが止まり、exec 1 で戻る
want ch05 && run ch05 \
    '--chapter 05_shell --seconds 16 --fps 10 --serial --send-at "5:kill 1" --send-at "11:exec 1"' \
    '--expect "before:kill 1:d13=blink" --expect "after:kill 1:d13=steady"
     --expect "after:exec 1:d13=blink" --expect "all:left=blink"
     --expect "after:kill 1:serial~suspended" --expect "after:exec 1:serial~resumed"'

# 第6章: ゼロ除算でタスクが落ちても Heartbeat は動き続ける。フォールトの報告が
# 流れて消えないよう、シリアル画面を 14 行にする
want ch06 && run ch06 \
    '--chapter 06_memory_protection --seconds 16 --fps 10 --serial --serial-lines 14 --send-at "3:d"' \
    '--expect "before:d:d13=blink" --expect "after:d:d13=blink"
     --expect "after:d:serial~FAULT DETECTED" --expect "after:d:serial~Divide by zero"
     --expect "after:d:serial~[Heartbeat]"'

# 第7章: kill 3 で CPU 負荷のグラフ（上半分）が下がり、exec 3 で戻る。
# 下半分右端のタスク状態表示は、スケジューラが切り替えるたびに変わる実機どおりの
# ちらつきなので、ちらつきの判定から外す
want ch07 && run ch07 \
    '--chapter 07_led_matrix --seconds 19 --fps 20 --serial --send-at "5:kill 3" --send-at "13:exec 3"' \
    '--expect "after:kill 3:top=drop" --expect "after:exec 3:top=rise"
     --expect "after:kill 3:serial~suspended" --allow-flicker bottom'

# 第8章: LED タスクが L を 500ms ごとに反転し続けるので、kill 1 で止めてから LED 命令を送る
want ch08 && run ch08 \
    '--chapter 08_interpreter --seconds 17 --fps 10 --serial
     --send-at "3:kill 1" --send-at "8:run LED 13 1" --send-at "13:run LED 13 0"' \
    '--expect "before:kill 1:d13=blink" --expect "after:kill 1:d13=steady"
     --expect "after:run LED 13 1:d13=on" --expect "after:run LED 13 0:d13=off"
     --expect "after:run LED 13 1:serial~= ON" --expect "after:run LED 13 0:serial~= OFF"'

# 第11章: led.on() / matrix.fill(1) / led.off()
want ch11 && run ch11 \
    '--chapter 11_tiny_python --seconds 17 --fps 10 --serial
     --send-at "3:led.on()" --send-at "7:matrix.fill(1)" --send-at "11:led.off()"' \
    '--expect "before:led.on():d13=off" --expect "after:led.on():d13=on"
     --expect "after:matrix.fill(1):matrix=lit>=96" --expect "after:led.off():d13=off"'

if [ -n "$FAILED" ]; then
    echo "NG:$FAILED"
    exit 1
fi
echo "すべて OK: $OUT"
