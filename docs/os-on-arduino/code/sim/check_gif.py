#!/usr/bin/env python3
"""record.py で録った GIF が、見せたいことを本当に見せているかを確かめる。

エミュレータの中で LED がどう動いたか（record.py --meta の truth）ではなく、
**GIF の画素**から LED の点灯を読み取って判定する。読者に見えるのは画素だけなので。

    uv run --no-project --with pillow python check_gif.py demo.json \\
        --expect "before:kill 1:d13=blink" --expect "after:kill 1:d13=steady"

いつも確かめること:

- 画素から読んだ点灯が truth と合っているか（描画や減色で LED が消えていないか）
- 点灯と消灯の色の差（見分けがつくか）。消えた LED は写真では白っぽく、点いた LED は
  オレンジなので、明るさ（輝度）はほとんど同じで色だけが違う。RGB の距離で比べる
- 点滅が GIF のコマで潰れていないか（--min-state-ms より短い点灯・消灯が多いとチラつきに見える）
- GIF の再生の速さが実時間と合っているか（各コマの表示時間の合計 = 録った時間）
- コマンドの前後が十分な長さ映っているか（前の様子が見えないと変化が分からない）
- 文字（左下のコマンド表示・シリアル画面）がページに載せた大きさで読めるか
- 長さとファイルサイズ

--expect の書き方は ``いつ:どこ=どう``。

- いつ: ``all`` / ``before:コマンド`` / ``after:コマンド``（次のコマンドまで。切り替わり直後の
  --settle 秒は除く）
- どこ: ``d13`` / ``matrix`` / ``top``（上 4 行）/ ``bottom`` / ``left``（左 6 列）/ ``right``
- どう: ``blink``（点滅している）、``blink=200``（1 周期がおよそ 200ms）、``steady``（変化しない）、
  ``on`` / ``off``（d13 のみ）、``lit>=N`` / ``lit<=N``（点灯数の平均）、``drop`` / ``rise``（直前の
  区間より点灯数の平均が 3 割以上減る / 増える。after: とだけ組み合わせる）

シリアル画面の文字は ``いつ:serial~文字列`` で、その文字列を含む行が画面に 2 秒以上
（--min-visible）見えているかを確かめる（すぐ流れて消えると読めない）。
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image

ROWS, COLS = 8, 12
REGIONS = {
    "matrix": [p for p in range(ROWS * COLS)],
    "top": [p for p in range(ROWS * COLS) if p // COLS < 4],
    "bottom": [p for p in range(ROWS * COLS) if p // COLS >= 4],
    "left": [p for p in range(ROWS * COLS) if p % COLS < 6],
    "right": [p for p in range(ROWS * COLS) if p % COLS >= 6],
}


@dataclass
class Result:
    name: str
    ok: bool
    detail: str


RGB = tuple[float, float, float]


def dist(a: RGB, b: RGB) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def mean_rgb(vals: list[RGB]) -> RGB:
    return tuple(sum(v[k] for v in vals) / len(vals) for k in range(3))


def gif_timeline(gif: Path) -> tuple[list[int], list[Image.Image]]:
    """GIF の各コマの (表示し始める時刻 ms, 画像)。同じコマは 1 枚にまとめられている。"""
    im = Image.open(gif)
    starts: list[int] = []
    images: list[Image.Image] = []
    t = 0
    for i in range(im.n_frames):
        im.seek(i)
        starts.append(t)
        images.append(im.convert("RGB"))
        t += im.info.get("duration", 0)
    starts.append(t)                       # 最後の要素は全体の長さ
    return starts, images


def frames_at(starts: list[int], images: list[Image.Image],
              times: list[int]) -> list[Image.Image]:
    """録ったコマの時刻それぞれに、GIF で表示されているコマ。"""
    out, k = [], 0
    for t in times:
        while k + 1 < len(images) and starts[k + 1] <= t:
            k += 1
        out.append(images[k])
    return out


def sample(frame: Image.Image, xy: list[float], r: int) -> RGB:
    """LED の中心まわり (2r+1)^2 画素の色の平均。"""
    x, y = int(round(xy[0])), int(round(xy[1]))
    return mean_rgb([frame.getpixel((i, j))
                     for i in range(max(0, x - r), min(frame.width, x + r + 1))
                     for j in range(max(0, y - r), min(frame.height, y + r + 1))])


def classify(colors: list[RGB], truth: list[int]) -> tuple[list[int], float]:
    """色 → 点灯 / 消灯。truth の点灯・消灯それぞれの平均色の、近いほうに分ける。

    戻り値: (判定, 点灯と消灯の平均色の距離)。truth に片方しか無いときは距離 0。
    """
    on = [v for v, t in zip(colors, truth) if t]
    off = [v for v, t in zip(colors, truth) if not t]
    if not on or not off:
        return list(truth), 0.0
    c_on, c_off = mean_rgb(on), mean_rgb(off)
    return [1 if dist(v, c_on) < dist(v, c_off) else 0 for v in colors], dist(c_on, c_off)


def runs(states: list[int], dur: list[int]) -> list[int]:
    """同じ状態が続いた時間（ms）の列。最初と最後は途中で切れているので除く。"""
    out, acc = [], dur[0]
    for a, b, d in zip(states, states[1:], dur[1:]):
        if a == b:
            acc += d
        else:
            out.append(acc)
            acc = d
    return out[1:]          # 先頭の区間は録り始めで切れている


def skip_ms(times: list[int], start: int, ms: float) -> int:
    """コマ start から ms ミリ秒たったコマ。"""
    i = start
    while i < len(times) and times[i] < times[start] + ms:
        i += 1
    return i


def segment(meta: dict, when: str, settle: float,
            times: list[int]) -> tuple[int, int, int | None]:
    """いつ → コマの範囲 [a, b)。after: は直前区間の開始も返す（drop / rise 用）。"""
    n = meta["frames"]
    events = meta["events"]
    if when == "all":
        return 0, n, None
    kind, _, cmd = when.partition(":")
    idx = next((i for i, e in enumerate(events) if e["cmd"] == cmd), None)
    if idx is None:
        raise SystemExit(f"--expect のコマンド {cmd!r} は録画中に送られていません"
                         f"（送ったもの: {[e['cmd'] for e in events]}）")
    at = events[idx]["frame"]
    prev = events[idx - 1]["frame"] if idx > 0 else 0
    nxt = events[idx + 1]["frame"] if idx + 1 < len(events) else n
    if kind == "before":
        return (skip_ms(times, prev, settle * 1000) if idx > 0 else prev), at, None
    if kind == "after":
        return skip_ms(times, at, settle * 1000), nxt, prev
    raise SystemExit(f"いつ は all / before:コマンド / after:コマンド のどれか: {when!r}")


def main() -> int:
    ap = argparse.ArgumentParser(description="録った GIF が見せたいことを見せているか確かめる")
    ap.add_argument("meta", type=Path, help="record.py --meta で書いた JSON")
    ap.add_argument("--expect", action="append", default=[], metavar="いつ:どこ=どう")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="コマンドを送った直後、判定に使わない秒数")
    # サポートサイト（pydata-sphinx-theme）で width:100% の図が画面 1280px のときに
    # 表示される幅。1440px では 728px、スマホ（390px）では 342px
    ap.add_argument("--display-width", type=int, default=608,
                    help="ページに載せたときの横幅（px）。文字の大きさの判定に使う")
    ap.add_argument("--min-font", type=float, default=11.0,
                    help="ページ上での文字の高さの下限（px）")
    ap.add_argument("--min-contrast", type=float, default=60.0,
                    help="点灯と消灯の平均色の距離の下限（RGB、0-441）")
    ap.add_argument("--min-before", type=float, default=2.0,
                    help="コマンドの前に映っているべき秒数")
    ap.add_argument("--min-after", type=float, default=3.0,
                    help="コマンドの後に映っているべき秒数")
    ap.add_argument("--min-state-ms", type=float, default=80.0,
                    help="これより短い点灯・消灯はチラつきに見える")
    ap.add_argument("--allow-flicker", action="append", default=[], choices=sorted(REGIONS),
                    help="この領域のマトリクスはちらつきの判定から外す（実機でもそう動くとき）。"
                         "結果には INFO として残る")
    ap.add_argument("--min-visible", type=float, default=2.0,
                    help="serial~ の文字列が画面に見えているべき秒数")
    ap.add_argument("--max-seconds", type=float, default=20.0)
    ap.add_argument("--max-kb", type=float, default=500.0)
    args = ap.parse_args()

    meta = json.loads(args.meta.read_text(encoding="utf-8"))
    gif = Path(meta["gif"])
    if not gif.is_absolute():
        gif = args.meta.parent / gif
    fps, truth, dur = meta["fps"], meta["truth"], meta["durations"]
    starts, images = gif_timeline(gif)
    times = [sum(dur[:i]) for i in range(len(dur))]
    frames = frames_at(starts, images, times)
    n = len(frames)
    results: list[Result] = []

    # --- 再生の速さ: GIF の長さ = 録った時間 か ---
    shot = (truth[-1]["t"] - truth[0]["t"]) * 1000 + 1000 / fps
    speed = shot / starts[-1] if starts[-1] else 0.0
    results.append(Result("再生の速さ（実時間との比）", abs(speed - 1) <= 0.05,
                          f"{speed:.2f} 倍（GIF {starts[-1] / 1000:.1f}s / 録った {shot / 1000:.1f}s、"
                          f"実際に撮れたのは {n / (shot / 1000):.0f}fps）"))
    meta["frames"] = n

    # --- 画素から LED を読む ---
    r13 = max(1, int(meta["glow_radius_px"]["d13"] * 0.25))
    rmx = max(1, int(meta["glow_radius_px"]["matrix"] * 0.2))
    seen: dict[str, list[int]] = {}
    d13_xy = meta["leds"]["d13"]
    if d13_xy:
        t = [f["led"] for f in truth]
        states, contrast = classify([sample(f, d13_xy, r13) for f in frames], t)
        seen["d13"] = states
        agree = sum(a == b for a, b in zip(states, t)) / n
        results.append(Result("D13: 画素と truth の一致", agree >= 0.98, f"{agree:.1%}"))
        if len(set(t)) > 1:
            results.append(Result("D13: 点灯と消灯の色の差",
                                  contrast >= args.min_contrast, f"RGB 距離 {contrast:.0f}"))
            results.append(Result("D13: LED の光の大きさ", meta["glow_radius_px"]["d13"] >= 4,
                                  f"半径 {meta['glow_radius_px']['d13']:.1f} px"))
    cells: list[list[int]] = [[0] * n for _ in range(ROWS * COLS)]
    worst_agree, contrasts = 1.0, []
    for p, xy in enumerate(meta["leds"]["matrix"]):
        if xy is None:
            continue
        t = [int(f["matrix"][p]) for f in truth]
        states, contrast = classify([sample(f, xy, rmx) for f in frames], t)
        cells[p] = states
        if len(set(t)) > 1:
            contrasts.append(contrast)
            worst_agree = min(worst_agree,
                              sum(a == b for a, b in zip(states, t)) / n)
    if contrasts:
        results.append(Result("マトリクス: 画素と truth の一致（最も悪い LED）",
                              worst_agree >= 0.98, f"{worst_agree:.1%}"))
        results.append(Result("マトリクス: 点灯と消灯の色の差（最も小さい LED）",
                              min(contrasts) >= args.min_contrast, f"RGB 距離 {min(contrasts):.0f}"))
    for name, idx in REGIONS.items():
        seen[name] = [sum(cells[p][i] for p in idx) for i in range(n)]

    # --- 点滅がコマで潰れていないか ---
    allowed = {p for r in args.allow_flicker for p in REGIONS[r]}
    signals = {"d13": (seen.get("d13", []), False)}
    signals.update({f"LED{p}": (cells[p], p in allowed) for p in range(ROWS * COLS)})
    for name, (states, ok_to_flicker) in signals.items():
        rl = runs(states, dur)
        if len(rl) < 3:
            continue
        short = sum(1 for v in rl if v < args.min_state_ms) / len(rl)
        if name == "d13" or short > 0.1:
            detail = f"{short:.0%}（{len(rl)} 区間、中央値 {statistics.median(rl):.0f}ms）"
            if ok_to_flicker and short > 0.1:
                print(f"INFO {name}: ちらつき {detail}（--allow-flicker で許可）")
                continue
            results.append(Result(f"{name}: {args.min_state_ms:.0f}ms 未満で終わる点灯・消灯の割合",
                                  short <= 0.1, detail))

    # --- コマンドの前後 ---
    ev = meta["events"]
    bounds = [0] + [e["frame"] for e in ev] + [n]
    t_at = times + [times[-1] + dur[-1]]
    for i, e in enumerate(ev):
        before = (t_at[bounds[i + 1]] - t_at[bounds[i]]) / 1000
        after = (t_at[bounds[i + 2]] - t_at[bounds[i + 1]]) / 1000
        results.append(Result(f"'{e['cmd']}' の前後の長さ",
                              before >= args.min_before and after >= args.min_after,
                              f"前 {before:.1f}s / 後 {after:.1f}s"))

    # --- 文字 ---
    shrink = min(1.0, args.display_width / meta["size"][0])
    if ev:
        px = meta["caption_font_px"] * shrink
        results.append(Result("コマンド表示の文字（ページ上）", px >= args.min_font, f"{px:.1f}px"))
    if meta.get("serial_font_px"):
        px = meta["serial_font_px"] * shrink
        results.append(Result("シリアル画面の文字（ページ上）", px >= args.min_font,
                              f"{px:.1f}px（GIF 幅 {meta['size'][0]} → {args.display_width}px）"))

    # --- 長さとサイズ ---
    secs = starts[-1] / 1000
    kb = gif.stat().st_size / 1024
    results.append(Result("長さ", secs <= args.max_seconds, f"{secs:.1f}s"))
    results.append(Result("ファイルサイズ", kb <= args.max_kb, f"{kb:.0f} KB"))

    # --- 章ごとの期待 ---
    for spec in args.expect:
        if ":serial~" in spec or spec.startswith("serial~"):
            when, _, needle = spec.partition("serial~")
            when = when.rstrip(":") or "all"
            if "serial" not in truth[0]:
                raise SystemExit("serial~ は record.py --serial で録った GIF にだけ使える")
            a, b, _ = segment(meta, when, 0.0, times)
            secs = sum(d for f, d in zip(truth[a:b], dur[a:b])
                       if any(needle in line for line in f["serial"])) / 1000
            results.append(Result(spec, secs >= args.min_visible,
                                  f"画面に見えている時間 {secs:.1f}s"))
            continue
        when, _, rest = spec.rpartition(":")
        target, _, prop = rest.partition("=")
        if target not in seen:
            raise SystemExit(f"どこ は {sorted(seen)} のどれか: {spec!r}")
        a, b, prev = segment(meta, when, args.settle, times)
        seg = seen[target][a:b]
        if not seg:
            results.append(Result(spec, False, "区間が空"))
            continue
        toggles = sum(1 for x, y in zip(seg, seg[1:]) if x != y)
        mean_lit = statistics.mean(seg)
        if prop == "blink" or prop.startswith("blink="):
            ok, detail = toggles >= 2, f"切り替わり {toggles} 回"
            if ok and prop.startswith("blink="):
                want = float(prop.split("=", 1)[1])
                rl = runs(seg, dur[a:b])
                period = 2 * statistics.mean(rl) if rl else float("inf")
                ok = abs(period - want) <= 0.3 * want
                detail += f"、1 周期 {period:.0f}ms（期待 {want:.0f}ms ±30%）"
        elif prop == "steady":
            ok, detail = toggles == 0, f"切り替わり {toggles} 回"
        elif prop in ("on", "off"):
            want = 1 if prop == "on" else 0
            ok = all(v == want for v in seg)
            detail = f"点灯しているコマ {sum(seg)}/{len(seg)}"
        elif prop.startswith("lit>=") or prop.startswith("lit<="):
            want = float(prop[5:])
            ok = mean_lit >= want if prop[3] == ">" else mean_lit <= want
            detail = f"点灯数の平均 {mean_lit:.1f}"
        elif prop in ("drop", "rise"):
            if prev is None:
                raise SystemExit(f"{prop} は after: と組み合わせる: {spec!r}")
            at = next(e["frame"] for e in meta["events"] if e["cmd"] == when.partition(":")[2])
            ref = statistics.mean(seen[target][prev:at] or [0])
            ok = (mean_lit <= 0.7 * ref if prop == "drop"
                  else mean_lit >= 1.3 * ref and mean_lit >= ref + 1)
            detail = f"点灯数の平均 {ref:.1f} → {mean_lit:.1f}"
        else:
            raise SystemExit(f"どう が分かりません: {spec!r}")
        results.append(Result(spec, ok, detail))

    width = max(len(r.name) for r in results)
    for r in results:
        print(f"{'OK ' if r.ok else 'NG '} {r.name.ljust(width)}  {r.detail}")
    bad = [r for r in results if not r.ok]
    print(f"{gif.name}: {len(results) - len(bad)}/{len(results)} OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
