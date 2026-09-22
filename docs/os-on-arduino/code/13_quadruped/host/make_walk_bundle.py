"""ブラウザ版シミュレータが読む policy.json を書き出し物から作る。

`export_quad_policy.py` が出す 3 点のうち、ブラウザ版
(`docs/assembly/quadruped/walk/`) が読むのは重みとメタデータを 1 つに
まとめた bundle です。`.npz` を読む手立てがブラウザに無いので、配列を
JSON の配列として展開します。

    python host/make_walk_bundle.py <書き出し先のディレクトリ> <出力する policy.json>

公開する bundle に学習機の絶対パスを残さないよう、`checkpoint` は
学習ログの中の相対パスに書き直します。
"""

import argparse
import json
import os

import numpy as np

ARRAY_KEYS = ("obs_mean", "obs_std", "default_joint_pos", "action_scale")


def log_relative(checkpoint: str, training: dict) -> str:
    """Rewrite an absolute checkpoint path as a path inside the log root.

    Parameters
    ----------
    checkpoint : str
        The path the exporter recorded, absolute or already relative.
    training : dict
        The exported metadata's ``training`` block (``experiment_name`` and
        ``run_dir``).

    Returns
    -------
    str
        ``logs/rsl_rl/<experiment>/<run>/<file>``.
    """
    return "logs/rsl_rl/{}/{}/{}".format(
        training["experiment_name"], training["run_dir"],
        os.path.basename(checkpoint))


def build(export_dir: str) -> dict:
    """Read an export directory and return the browser bundle.

    Parameters
    ----------
    export_dir : str
        Directory holding ``arduino_quad_policy.json`` and ``.npz``.

    Returns
    -------
    dict
        ``{"meta": ..., "n_layers": ..., "w0": ..., "b0": ..., ...}``.
    """
    meta = json.load(open(os.path.join(export_dir, "arduino_quad_policy.json")))
    meta["checkpoint"] = log_relative(meta["checkpoint"], meta["training"])
    weights = np.load(os.path.join(export_dir, "arduino_quad_policy.npz"))
    n_layers = int(weights["n_layers"])
    bundle = {"meta": meta, "n_layers": n_layers}
    for key in ARRAY_KEYS:
        bundle[key] = weights[key].astype(float).tolist()
    for i in range(n_layers):
        bundle["w%d" % i] = weights["w%d" % i].astype(float).tolist()
        bundle["b%d" % i] = weights["b%d" % i].astype(float).tolist()
    return bundle


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export_dir", help="export_quad_policy.py の出力先")
    parser.add_argument("out", help="書き出す policy.json")
    args = parser.parse_args()
    bundle = build(args.export_dir)
    with open(args.out, "w") as handle:
        json.dump(bundle, handle)
    print("wrote {} (checkpoint={}, phase_command_threshold={})".format(
        args.out, bundle["meta"]["checkpoint"],
        bundle["meta"].get("phase_command_threshold")))


if __name__ == "__main__":
    main()
