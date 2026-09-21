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

--serial で基板の下にシリアルの画面を並べる（打ったコマンドと LED の変化を
同じ画面で見せる）。右に並べると GIF が横に広がり、ページで縮められて文字が
読めなくなるので、既定は下。--meta は、GIF の上の LED の位置・コマンドを送ったコマ・
各コマの LED の状態を JSON に書く。check_gif.py がこれを読んで、GIF が本当に
その変化を見せているかを確かめる。
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

# シリアル画面の等幅フォントを探す場所（見つからなければ --font で渡す）
MONO_FONTS = [
    "/System/Library/Fonts/Menlo.ttc",                              # macOS
    "/System/Library/Fonts/Monaco.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",           # Debian / Ubuntu
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",                       # Arch
    "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",    # Fedora
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "C:/Windows/Fonts/consola.ttf",                                  # Windows
    "C:/Windows/Fonts/cour.ttf",
]
SERIAL_BG = (22, 27, 34)
SERIAL_FG = (220, 226, 232)
SERIAL_CMD = (120, 200, 255)      # こちらから送った行（"> kill 1" など）


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


def find_mono_font(explicit: Path | None) -> Path:
    """シリアル画面に使う等幅フォント。見つからなければ理由を出して止める。"""
    cands = [explicit] if explicit else [Path(c) for c in MONO_FONTS]
    for c in cands:
        if c and c.is_file():
            return c
    raise SystemExit(
        "シリアル画面に使う等幅フォントが見つかりません。探した場所:\n  "
        + "\n  ".join(str(c) for c in cands)
        + "\n--font で .ttf / .ttc を渡してください"
          "（Ubuntu なら sudo apt install fonts-dejavu-core）。")


def draw_serial(panel_size: tuple[int, int], text: str, sent: list[str],
                font: ImageFont.FreeTypeFont) -> tuple[Image.Image, list[str]]:
    """シリアルの末尾を、端末のように下詰めで描く。送った行は色を変える。

    戻り値は (画像, 画面に見えている行)。
    """
    w, h = panel_size
    img = Image.new("RGB", (w, h), SERIAL_BG)
    draw = ImageDraw.Draw(img)
    pad = max(6, w // 40)
    char_w = draw.textlength("M", font=font)
    ascent, descent = font.getmetrics()
    line_h = ascent + descent + 2
    cols = max(1, int((w - 2 * pad) // char_w))
    rows = max(1, (h - 2 * pad) // line_h)
    lines: list[str] = []
    for raw in text.replace("\r", "").split("\n"):
        while len(raw) > cols:                     # 折り返す
            lines.append(raw[:cols])
            raw = raw[cols:]
        lines.append(raw)
    sent_set = {f"> {c}" for c in sent} | set(sent)
    visible = lines[-rows:]
    for i, line in enumerate(visible):
        color = SERIAL_CMD if line.strip() in sent_set else SERIAL_FG
        draw.text((pad, pad + i * line_h), line, font=font, fill=color)
    return img, visible


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
    ap.add_argument("--serial", action="store_true",
                    help="シリアルの画面を並べる（既定は基板の下）")
    ap.add_argument("--serial-layout", choices=["below", "right"], default="below")
    ap.add_argument("--serial-width", type=int, default=380,
                    help="右に並べるときのシリアル画面の横幅（px）")
    ap.add_argument("--serial-lines", type=int, default=9,
                    help="下に並べるときのシリアル画面の行数")
    ap.add_argument("--serial-font-size", type=int, default=13)
    ap.add_argument("--font", type=Path, help="シリアル画面の等幅フォント（.ttf / .ttc）")
    ap.add_argument("--meta", type=Path,
                    help="LED の位置・送ったコマンド・各コマの状態を書く JSON（check_gif.py 用）")
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

    font = (ImageFont.truetype(str(find_mono_font(args.font)), args.serial_font_size)
            if args.serial else None)
    board_size = render([0] * len(pos), 0).size
    scale = board_size[0] / (args.crop[2] - args.crop[0])

    def to_frame_xy(x: int, y: int) -> list[float] | None:
        """写真の上の座標 → GIF の上の座標（切り出した範囲の外なら None）。"""
        if not (args.crop[0] <= x < args.crop[2] and args.crop[1] <= y < args.crop[3]):
            return None
        return [round((x - args.crop[0]) * scale, 1), round((y - args.crop[1]) * scale, 1)]

    if font is None:
        panel_box = None
    elif args.serial_layout == "right":
        panel_box = (board_size[0], 0, board_size[0] + args.serial_width, board_size[1])
    else:
        ascent, descent = font.getmetrics()
        pad = max(6, board_size[0] // 40)
        panel_h = args.serial_lines * (ascent + descent + 2) + 2 * pad
        panel_box = (0, board_size[1], board_size[0], board_size[1] + panel_h)

    def compose(board: Image.Image, serial_text: str,
                sent: list[str]) -> tuple[Image.Image, list[str]]:
        if panel_box is None:
            return board, []
        out = Image.new("RGB", (max(board.width, panel_box[2]),
                                max(board.height, panel_box[3])), SERIAL_BG)
        out.paste(board, (0, 0))
        panel, visible = draw_serial(
            (panel_box[2] - panel_box[0], panel_box[3] - panel_box[1]), serial_text, sent, font)
        out.paste(panel, panel_box[:2])
        return out, visible

    # 色の表は、LED を全部点けた絵（とシリアル画面の見本）から 1 回だけ作り、全コマで
    # 使い回す。コマごとに減色すると背景の同じ場所でもディザの模様が変わり、GIF が
    # 毎コマ全面を持つ。
    sample_text = "> ps\nID  NAME   STATE    CPU%\n0   idle   READY    50%\n" * 4
    palette = compose(render([1] * len(pos), 1), sample_text, ["ps"])[0].quantize(
        colors=args.colors)

    def to_gif_frame(frame: Image.Image) -> Image.Image:
        return frame.quantize(palette=palette, dither=Image.Dither.NONE)

    # 1 コマの間に 4 回は読む（QMP の 1 往復は 1ms 未満）
    bd = board_mod.Board(elf, board_mod.make_backend(
        args, elf, board_mod.free_port(), board_mod.free_port()),
        poll_interval=min(0.05, 1.0 / (4 * args.fps)))
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
        truth: list[dict] = []
        events: list[dict] = []
        sent: list[str] = list(args.send)
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
                sent.append(caption)
                events.append({"t": round(tick - start, 3), "frame": len(frames),
                               "cmd": caption})
            # bd.state はポーリングのスレッドがその場で書き換えるので、1 回だけ
            # 取り出した値を、描画と truth の両方に使う
            matrix, led = list(bd.state["matrix"]), bd.state["led"]
            board = render(matrix, led)
            if caption:
                draw_caption(board, caption)
            frame, visible = compose(board, "".join(bd.serial_log), sent)
            frames.append(to_gif_frame(frame))
            entry = {"t": round(tick - start, 3), "led": led,
                     "matrix": "".join(str(v) for v in matrix)}
            if panel_box:
                entry["serial"] = visible
            truth.append(entry)
            time.sleep(max(0.0, interval - (time.time() - tick)))
    finally:
        bd.stop()

    if not frames:
        print("フレームが 1 枚も録れませんでした", file=sys.stderr)
        return 1
    # 各コマの表示時間は、実際に撮った時刻の差から決める（描画が間に合わず fps を
    # 下回っても、GIF が実時間より速く再生されないように）。GIF は 10ms 単位なので、
    # 誤差がたまらないよう累積時刻を丸めてから差を取る。
    stamps = [f["t"] for f in truth] + [truth[-1]["t"] + 1.0 / args.fps]
    ends = [round((t - stamps[0]) * 100) * 10 for t in stamps[1:]]
    durations = [max(10, b - a) for a, b in zip([0] + ends[:-1], ends)]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, optimize=True)
    size_mb = args.out.stat().st_size / 1024 / 1024
    print(f"{args.out} : {len(frames)} フレーム, {size_mb:.1f} MB")
    if args.meta:
        glow_r = {"matrix": px_glow[1].width / 2 * scale, "d13": d13_glow[1].width / 2 * scale}
        meta = {
            "gif": str(args.out), "chapter": args.chapter.resolve().name,
            "fps": args.fps, "seconds": args.seconds, "frames": len(frames),
            "size": list(frames[0].size), "board_size": list(board_size),
            "serial_panel": list(panel_box) if panel_box else None,
            "serial_font_px": args.serial_font_size if font else None,
            "caption_font_px": max(12, board_size[0] // 28),
            "leds": {"d13": to_frame_xy(*d13),
                     "matrix": [to_frame_xy(x, y) for x, y in pos]},
            "glow_radius_px": {k: round(v, 1) for k, v in glow_r.items()},
            "events": events, "truth": truth, "durations": durations,
        }
        args.meta.parent.mkdir(parents=True, exist_ok=True)
        args.meta.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
