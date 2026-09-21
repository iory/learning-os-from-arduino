#!/usr/bin/env python3
"""基板写真から「LED マトリクス消灯」の背景画像と LED 座標を作る.

写真ではマトリクスが点灯しているので、そのままだと常時点灯に見えてしまう。
消灯している LED の見た目を 96 個ぶん貼り直し、赤い光を取り除いて
「電源は入っているが表示は消えている基板」を作る。

    uv run --no-project --with pillow --with numpy --with scipy \
        python make_board_image.py \
        --photo ../../docs/ja/source/_static/uno_r4_wifi_os.jpg \
        --out board_off.jpg --cells matrix_cells.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

# 写真 (1600x1200) の LED 96 個を自動検出してアフィン当てはめした結果。
# (col, row, 1) @ AFFINE -> (x, y) [px]。残差は平均 1.5px。
AFFINE = np.array([[32.7420, 0.8100],
                   [1.3370, 30.7960],
                   [811.931, 581.490]])
COLS, ROWS = 12, 8
PATCH = (34, 32)           # LED 1 個ぶんの切り出しサイズ [px]
LED_D13 = (694.0, 366.0)   # 基板上の "L" LED (D13 = P102) の位置 [px]


def detect_leds(img: Image.Image) -> np.ndarray:
    """写真から LED を検出する（当てはめの健全性チェック用）."""
    a = np.asarray(img.convert("RGB")).astype(np.int16)
    x0, y0, x1, y1 = 780, 550, 1210, 840
    sub = a[y0:y1, x0:x1]
    mask = (sub.mean(axis=2) > 120) & ((sub[:, :, 0] - sub[:, :, 2]) > -10)
    mask = ndimage.binary_opening(mask, np.ones((2, 2)))
    lab, n = ndimage.label(mask)
    sizes = ndimage.sum(mask, lab, range(1, n + 1))
    cents = np.array(ndimage.center_of_mass(mask, lab, range(1, n + 1)))
    keep = (sizes > 40) & (sizes < 900)
    return np.stack([cents[keep][:, 1] + x0, cents[keep][:, 0] + y0], axis=1)


def led_xy(col: int, row: int) -> tuple[float, float]:
    x, y = np.array([col, row, 1.0]) @ AFFINE
    return float(x), float(y)


# 消灯した基板の青 (R, G) を B との比で表したもの。写真から実測。
PCB_R_RATIO, PCB_G_RATIO = 0.35, 0.62


def deglow(region: np.ndarray, strength: float = 60.0) -> None:
    """赤い発光を基板の青で置き換える（その場で書き換える）.

    赤みの強さに応じて基板色へ寄せる。単純に R を引くだけだと
    R==B の紫色が残り、継ぎ目が線になって見えてしまう。
    """
    excess = np.clip(region[:, :, 0] - region[:, :, 2], 0, None)
    w = np.clip(excess / strength, 0, 1)
    blue = region[:, :, 2]
    region[:, :, 0] = region[:, :, 0] * (1 - w) + blue * PCB_R_RATIO * w
    region[:, :, 1] = region[:, :, 1] * (1 - w) + blue * PCB_G_RATIO * w


def unlit_patch(img: Image.Image) -> np.ndarray:
    """消灯している LED を多数集めて中央値を取り、貼り付け用の 1 個を作る.

    平均ではなく中央値なのは、隣の点灯 LED の赤かぶりを拾わないため。
    """
    stack = []
    for row in (0, ROWS - 1):
        for col in range(COLS):
            x, y = led_xy(col, row)
            left, top = int(round(x - PATCH[0] / 2)), int(round(y - PATCH[1] / 2))
            stack.append(np.asarray(
                img.crop((left, top, left + PATCH[0], top + PATCH[1])),
                dtype=np.float64))
    patch = np.median(np.stack(stack), axis=0)
    deglow(patch, strength=140.0)
    return np.clip(patch, 0, 255)


def feather(shape: tuple[int, int], margin: int = 3) -> np.ndarray:
    """貼り付けの継ぎ目が出ないよう、周囲を緩やかに透過させる窓."""
    ramp = (1 - np.cos(np.linspace(0, np.pi, margin * 2))) / 2
    win = []
    for size in shape:
        w = np.ones(size)
        w[:margin] = ramp[:margin]
        w[-margin:] = ramp[margin:]
        win.append(w)
    return np.outer(win[0], win[1])


def build_unlit_board(img: Image.Image) -> np.ndarray:
    canvas = np.asarray(img, dtype=np.float64).copy()
    patch = unlit_patch(img)
    alpha = feather((PATCH[1], PATCH[0]))[:, :, None]

    for row in range(ROWS):
        for col in range(COLS):
            x, y = led_xy(col, row)
            left = int(round(x - PATCH[0] / 2))
            top = int(round(y - PATCH[1] / 2))
            cell = canvas[top:top + PATCH[1], left:left + PATCH[0]]
            cell *= 1 - alpha
            cell += patch * alpha

    # 貼り終えたあとに領域全体の赤を抜く。継ぎ目に残った光もここで消える。
    x0 = int(led_xy(0, 0)[0] - PATCH[0])
    y0 = int(led_xy(0, 0)[1] - PATCH[1])
    x1 = int(led_xy(COLS - 1, ROWS - 1)[0] + PATCH[0])
    y1 = int(led_xy(COLS - 1, ROWS - 1)[1] + PATCH[1])
    deglow(canvas[y0:y1, x0:x1])
    return np.clip(canvas, 0, 255)


def main() -> int:
    ap = argparse.ArgumentParser(description="基板写真から消灯状態の背景を作る")
    ap.add_argument("--photo", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cells", type=Path,
                    help="LED 位置をパーセントで書き出す JSON")
    args = ap.parse_args()

    img = Image.open(args.photo).convert("RGB")
    found = detect_leds(img)
    print(f"写真から検出した LED: {len(found)} 個（期待値 {COLS * ROWS}）")

    Image.fromarray(build_unlit_board(img).astype(np.uint8)).save(
        args.out, quality=92)
    print(f"消灯状態の背景を書き出した: {args.out}")

    if args.cells:
        w, h = img.size
        cells = [{"x": round(led_xy(c, r)[0] / w * 100, 4),
                  "y": round(led_xy(c, r)[1] / h * 100, 4)}
                 for r in range(ROWS) for c in range(COLS)]
        args.cells.write_text(json.dumps({
            "size": [w, h],
            "pitch": [round(AFFINE[0][0] / w * 100, 4),
                      round(AFFINE[1][1] / h * 100, 4)],
            "matrix": cells,
            "d13": {"x": round(LED_D13[0] / w * 100, 4),
                    "y": round(LED_D13[1] / h * 100, 4)},
        }))
        print(f"LED 座標を書き出した: {args.cells}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
