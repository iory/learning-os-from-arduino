"""TWO_STAGE_RESULTS.md 4.1 節の報酬曲線 (200 iteration ごとの平均) をグラフにする。"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

curves = {
    ("joint", 1): "0 7 29 23 32 52 60 64 67 69 70 65 73 75 75 76 76 78 79 81 82 83 83",
    ("joint", 2): "-0 3 4 1 1 1 0 1 1 7 27 36 45 48 52 54 56 57 58 60 61 61 61",
    ("joint", 3): "-0 3 6 10 47 68 77 81 83 85 87 80 90 90 91 92 92 92 92 92 92 91 91",
    ("baseline", 1): "-2 2 5 1 1 1 1 1 1 1 1 1 1 20 45 61 68 73 76 77 80 81 81",
    ("baseline", 2): "-2 1 21 14 26 41 49 52 55 56 58 54 62 63 64 65 64 65 64 64 64 62 62",
    ("baseline", 3): "-0 4 5 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 6 25 46 62 70",
}
color = {"joint": "tab:red", "baseline": "tab:blue"}
marker = {1: "o", 2: "s", 3: "^"}
label = {"joint": "joint (A+B)", "baseline": "baseline (A only)"}

fig, ax = plt.subplots(figsize=(9, 5))
for (mode, seed), s in curves.items():
    y = [float(v) for v in s.split()]
    x = [200 * i + 100 for i in range(len(y))]  # 各区間の中央
    ax.plot(x, y, color=color[mode], marker=marker[seed], markersize=5,
            linewidth=1.2, alpha=0.9, label=f"{label[mode]} seed {seed}")
ax.set_xlabel("iteration")
ax.set_ylabel("mean reward (per 200 iterations)")
ax.set_title("Training curves: joint vs baseline (3 seeds each)")
ax.grid(alpha=0.3)
ax.legend(ncol=2, fontsize=9)
fig.tight_layout()
fig.savefig("two_stage_reward_curves.png", dpi=150)
