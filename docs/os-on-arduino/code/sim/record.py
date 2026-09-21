#!/usr/bin/env python3
"""仮想ボードの動きを GIF に録る。

ブラウザは使わず、エミュレータ（Renode か QEMU）から読んだ LED の状態を
基板写真に合成する。
CI でも同じものが撮れるので、章ごとの「こう光る」を自動で残せる。

    uv run --no-project --with pillow python record.py \
        --chapter ../04_scheduler --out demo.gif --seconds 8

シェルのある章は、コマンドを送ってから録ることもできる。

    ... --chapter ../05_shell --send ps --send "kill 3" --seconds 10

録っている途中で送ることもできる（送ったコマンドは GIF の下に表示する）。

    ... --chapter ../07_led_matrix --seconds 16 --send-at "6:kill 3"
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
import board as board_mod                                   # noqa: E402
import emulator_process                                     # noqa: E402

HERE = Path(__file__).resolve().parent

# 基板だけを切り出す範囲（写真 1600x1200 のうち）。周りの机は要らない。
CROP = (170, 130, 1430, 1080)


def led_sprites(radius: int, halo: tuple[int, int, int],
                body: tuple[int, int, int], core: tuple[int, int, int]
                ) -> tuple[Image.Image, Image.Image]:
    """点灯した LED 1 個ぶんの絵を 2 枚作る.

    消灯した LED は写真では白っぽいので、加算だけで光らせると白飛びする。
    本体は「置き換え」（琥珀色）、まわりの滲みは「加算」に分ける。
    """
    size = radius * 2
    body_img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    halo_img = Image.new("RGB", (size, size), (0, 0, 0))
    bd, hd = ImageDraw.Draw(body_img), ImageDraw.Draw(halo_img)

    body_r = max(2, int(radius * 0.34))          # LED パッケージとほぼ同じ大きさ
    for i in range(body_r, 0, -1):
        t = i / body_r
        rgb = tuple(int(core[k] * (1 - t) + body[k] * t) for k in range(3))
        bd.ellipse([radius - i, radius - i, radius + i, radius + i],
                   fill=rgb + (255,))
    for i in range(radius, body_r, -1):
        t = (i - body_r) / (radius - body_r)
        falloff = (1 - t) ** 2.4
        hd.ellipse([radius - i, radius - i, radius + i, radius + i],
                   fill=tuple(int(c * falloff) for c in halo))
    return body_img, halo_img


def parse_crop(spec: str) -> tuple[int, int, int, int]:
    """"430,170,1030,620" → 写真（1600x1200）の上の切り出し範囲。"""
    try:
        x0, y0, x1, y1 = (int(v) for v in spec.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"x0,y0,x1,y1 の形で指定してください: {spec!r}") from None
    if x1 <= x0 or y1 <= y0:
        raise argparse.ArgumentTypeError(f"範囲が空です: {spec!r}")
    return x0, y0, x1, y1


def parse_send_at(spec: str) -> tuple[float, str]:
    """"6:kill 3" → (6.0, "kill 3")。録り始めてからの秒数とコマンド。"""
    sec, sep, cmd = spec.partition(":")
    if not sep or not cmd:
        raise argparse.ArgumentTypeError(f"秒数:コマンド の形で指定してください: {spec!r}")
    try:
        return float(sec), cmd
    except ValueError:
        raise argparse.ArgumentTypeError(f"秒数が数値ではありません: {spec!r}") from None


def draw_caption(frame: Image.Image, text: str) -> None:
    """送ったコマンドを、シリアルに打ったように左下へ出す。"""
    draw = ImageDraw.Draw(frame)
    font = ImageFont.load_default(size=max(12, frame.width // 28))
    pad = frame.width // 60
    label = f"> {text}"
    x0, y0, x1, y1 = draw.textbbox((0, 0), label, font=font)
    box = (pad, frame.height - pad - (y1 - y0) - 2 * pad,
           pad + (x1 - x0) + 2 * pad, frame.height - pad)
    draw.rectangle(box, fill=(20, 20, 20))
    draw.text((box[0] + pad - x0, box[1] + pad - y0), label, font=font,
              fill=(235, 235, 235))


def add_glow(canvas: Image.Image, sprites: tuple[Image.Image, Image.Image],
             cx: int, cy: int) -> None:
    body, halo = sprites
    r = halo.width // 2
    box = (cx - r, cy - r, cx + r, cy + r)
    if box[0] < 0 or box[1] < 0 or box[2] > canvas.width or box[3] > canvas.height:
        return
    region = canvas.crop(box)
    region = ImageChops.add(region, halo)        # 基板への滲み
    region.paste(body, (0, 0), body)             # LED 本体
    canvas.paste(region, box)


def main() -> int:
    ap = argparse.ArgumentParser(description="仮想ボードの動きを GIF に録る")
    ap.add_argument("--chapter", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("board.gif"))
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--width", type=int, default=560, help="GIF の横幅")
    ap.add_argument("--colors", type=int, default=64, help="GIF の色数")
    ap.add_argument("--crop", type=parse_crop, default=CROP, metavar="x0,y0,x1,y1",
                    help="写真（1600x1200）のどこを切り出すか。既定は基板全体。"
                         "D13 だけを大きく見せるなら 430,170,1030,620 など")
    ap.add_argument("--warmup", type=float, default=6.0,
                    help="録り始めるまでの待ち時間（起動を待つ）")
    ap.add_argument("--send", action="append", default=[],
                    help="録る前に送るコマンド（複数指定できる）")
    ap.add_argument("--send-at", type=parse_send_at, action="append", default=[],
                    metavar="秒:コマンド",
                    help="録っている途中で送るコマンド（例: '6:kill 3'）。GIF の下に表示する")
    board_mod.add_emulator_args(ap)
    args = ap.parse_args()
    emulator_process.install_signal_handlers()

    elf = args.chapter / ".pio/build/sim/firmware.elf"
    if not elf.exists():
        print(f"ファームウェアが見つかりません: {elf}\n"
              f"  cd {args.chapter} && pio run -e sim", file=sys.stderr)
        return 1

    cells = json.loads((HERE / "matrix_cells.json").read_text(encoding="utf-8"))
    photo = Image.open(HERE / "board_off.jpg").convert("RGB")
    w, h = photo.size
    pitch_px = cells["pitch"][0] / 100 * w
    px_glow = led_sprites(int(pitch_px * 1.7), halo=(120, 20, 6),
                          body=(255, 96, 16), core=(255, 232, 170))
    d13_glow = led_sprites(int(pitch_px * 1.5), halo=(120, 56, 4),
                           body=(255, 150, 20), core=(255, 240, 190))
    pos = [(int(c["x"] / 100 * w), int(c["y"] / 100 * h)) for c in cells["matrix"]]
    d13 = (int(cells["d13"]["x"] / 100 * w), int(cells["d13"]["y"] / 100 * h))

    def render(matrix: list[int], led: int) -> Image.Image:
        canvas = photo.copy()
        for lit, (x, y) in zip(matrix, pos):
            if lit:
                add_glow(canvas, px_glow, x, y)
        if led:
            add_glow(canvas, d13_glow, *d13)
        frame = canvas.crop(args.crop)
        return frame.resize(
            (args.width, round(frame.height * args.width / frame.width)), Image.LANCZOS)

    # 色の表は、LED を全部点けた絵から 1 回だけ作り、全コマで使い回す。コマごとに
    # 減色すると背景の同じ場所でもディザの模様が変わり、GIF が毎コマ全面を持つ。
    palette = render([1] * len(pos), 1).quantize(colors=args.colors)

    def to_gif_frame(frame: Image.Image) -> Image.Image:
        return frame.quantize(palette=palette, dither=Image.Dither.NONE)

    bd = board_mod.Board(elf, board_mod.make_backend(
        args, elf, board_mod.free_port(), board_mod.free_port()))
    if bd.fb_addr is None:
        print("注意: フレームバッファが見つからないので、マトリクスは消灯のままです。",
              file=sys.stderr)
    bd.start()
    try:
        time.sleep(args.warmup)
        for line in args.send:
            bd.send_serial(line + "\n")
            time.sleep(1.0)

        frames: list[Image.Image] = []
        interval = 1.0 / args.fps
        pending = sorted(args.send_at)
        caption = ""
        start = time.time()
        end = start + args.seconds
        while time.time() < end:
            tick = time.time()
            while pending and tick - start >= pending[0][0]:
                caption = pending.pop(0)[1]
                bd.send_serial(caption + "\n")
            state = bd.state
            frame = render(state["matrix"], state["led"])
            if caption:
                draw_caption(frame, caption)
            frames.append(to_gif_frame(frame))
            time.sleep(max(0.0, interval - (time.time() - tick)))
    finally:
        bd.stop()

    if not frames:
        print("フレームが 1 枚も録れませんでした", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / args.fps), loop=0, optimize=True)
    size_mb = args.out.stat().st_size / 1024 / 1024
    print(f"{args.out} : {len(frames)} フレーム, {size_mb:.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
