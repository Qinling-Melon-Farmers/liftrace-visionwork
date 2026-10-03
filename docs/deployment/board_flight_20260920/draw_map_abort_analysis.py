from pathlib import Path
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "map_abort_mechanism.png"
fig = plt.figure(figsize=(15, 5.4))
gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1, 1.1])

ax = fig.add_subplot(gs[0, 0])
ax.plot([0, .6], [0, 0], color="#2878b5", lw=3, label="executed first segment")
ax.plot([.6, 3], [0, 0], color="#999999", lw=2, ls="--", label="requested second segment")
ax.scatter([0, .6, 3], [0, 0, 0], c=["black", "#2ca02c", "#d62728"], s=[70, 90, 110], zorder=4)
ax.add_patch(Circle((3, 0), .15, fill=False, color="#d62728", lw=2, label="0.15 m goal adjustment"))
ax.text(.6, .12, "first point\nreached", ha="center", fontsize=9)
ax.text(3, .12, "goal occupied\nin_map=1, inflated=1", ha="center", fontsize=9)
ax.set(xlim=(-.2, 3.35), ylim=(-.55, .55), xlabel="Mission X (m)", ylabel="Mission Y (m)", title="What actually stopped progress")
ax.grid(alpha=.25)
ax.legend(loc="lower left", fontsize=8)

ax = fig.add_subplot(gs[0, 1])
for row, (name, res, color) in enumerate([("simulation", .05, "#2878b5"), ("board trial", .10, "#d62728")]):
    cells = math.ceil(.15 / res)
    half = cells * res
    y0 = 1 - row * 1.35
    ax.add_patch(Rectangle((-half, y0 - half), 2 * half, 2 * half, fill=False, ec=color, lw=2))
    points = []
    for i in range(-cells, cells + 1):
        for j in range(-cells, cells + 1):
            x, y = i * res, j * res
            if math.hypot(x, y) <= .15 + 1e-9:
                points.append((x, y0 + y))
    ax.scatter([p[0] for p in points], [p[1] for p in points], s=20, color=color)
    ax.add_patch(Circle((0, y0), .15, fill=False, ec="black", ls=":", lw=1.5))
    ax.text(.34, y0, f"{name}\nresolution={res:.2f} m\nsupport square half-width={half:.2f} m\ngoal samples={len(points)}", va="center", fontsize=9)
ax.set(xlim=(-.35, 1.2), ylim=(-.75, 1.35), xlabel="Local X (m)", title="Same parameters, different discretization")
ax.set_yticks([])
ax.grid(axis="x", alpha=.2)

ax = fig.add_subplot(gs[0, 2])
ax.axis("off")
boxes = [
    (.05, .76, .9, .15, "Real static map\nclutter / object / stale points"),
    (.05, .52, .9, .15, "Column classifier\nAGL threshold 0.10 m; 3 points; span 0.20 m"),
    (.05, .28, .9, .15, "Full-height XY column + 0.30 m inflation\n(no virtual ceiling means map-top column)"),
    (.05, .04, .9, .15, "Fixed endpoint x=3.0 becomes occupied\n0.15 m neighborhood also has no free goal"),
]
for x, y, w, h, label in boxes:
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=.015", fc="#eef3f8", ec="#406080", lw=1.5))
    ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=9)
for y in (.71, .47, .23):
    ax.annotate("", xy=(.5, y - .03), xytext=(.5, y + .04), arrowprops=dict(arrowstyle="-|>", lw=1.5))
ax.set_title("Plausible amplification chain\n(first failed map was not recorded)", fontsize=11)

fig.suptitle("Board visual-interrupt abort: confirmed endpoint failure and likely real-map amplifiers", fontsize=14)
fig.tight_layout(rect=(0, 0, 1, .93))
fig.savefig(OUT, dpi=160, bbox_inches="tight")
