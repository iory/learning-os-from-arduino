#!/usr/bin/env python3
"""仮想ボードの動きを GIF に録る。

ブラウザは使わず、エミュレータ（Renode か QEMU）から読んだ LED の状態を
基板写真に合成する。
CI でも同じものが撮れるので、章ごとの「こう光る」を自動で残せる。

    uv run --no-project --with pillow python record.py \
        --chapter ../04_scheduler --out demo.gif --seconds 8

シェルのある章は、コマンドを送ってから録ることもできる。

    ... --chapter ../05_shell --send ps --send "kill 3" --seconds 10
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw

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
    ap.add_argument("--warmup", type=float, default=6.0,
                    help="録り始めるまでの待ち時間（起動を待つ）")
    ap.add_argument("--send", action="append", default=[],
                    help="録る前に送るコマンド（複数指定できる）")
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
        end = time.time() + args.seconds
        while time.time() < end:
            tick = time.time()
            canvas = photo.copy()
            state = bd.state
            for lit, (x, y) in zip(state["matrix"], pos):
                if lit:
                    add_glow(canvas, px_glow, x, y)
            if state["led"]:
                add_glow(canvas, d13_glow, *d13)
            frame = canvas.crop(CROP)
            frame = frame.resize(
                (args.width, round(frame.height * args.width / frame.width)),
                Image.LANCZOS)
            frames.append(frame.convert("P", palette=Image.ADAPTIVE,
                                        colors=args.colors))
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
