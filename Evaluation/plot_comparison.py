"""
Turns the benchmark output (Results/*.csv, written by `VirtualPottery --bench`)
into comparison figures and a summary table.

    python Evaluation/plot_comparison.py            # reads and writes Results/

Figures
    fig1_push_force_depth_contacts.png   force, depth and contacts against time
                                         while the tool is pushed in (Lee 2008,
                                         Fig. 8), one panel per quantity
    fig2_force_detail.png                100 ms of the groove at tick level
    fig3_force_steps.png                 largest force change between two ticks
    fig4_resolution.png                  peak push force on a coarse and a fine grid
    fig5_timing.png                      physics step time, and the work left in
                                         the 1 kHz loop, against the grid size
Table
    summary.md
"""

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE    = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "Results")

# One colour per method, the same in every figure.
COLOR   = {"baseline": "#2a78d6", "method2": "#1baf7a"}
NAME    = {"baseline": "Baseline", "method2": "Method 2"}
ORDER   = ["baseline", "method2"]
SURFACE = "#fcfcfb"
INK     = "#0b0b0b"
INK2    = "#52514e"
GRID    = "#e4e3df"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.labelsize": 10,
    "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "legend.fontsize": 9, "font.size": 10,
    "lines.linewidth": 2.0,
})


# ------------------------------------------------------------------------------
# Data
# ------------------------------------------------------------------------------

def load_runs():
    runs = []
    with open(os.path.join(RESULTS, "runs.csv")) as f:
        for r in csv.DictReader(f):
            d = np.genfromtxt(os.path.join(RESULTS, r["file"]), delimiter=",", names=True)
            r["data"]  = d
            r["rows"]  = int(r["rows"])
            r["cols"]  = int(r["cols"])
            r["force"] = np.sqrt(d["fx"] ** 2 + d["fy"] ** 2 + d["fz"] ** 2)
            runs.append(r)
    return runs


def pick(runs, tag, scenario, rows):
    for r in runs:
        if r["tag"] == tag and r["scenario"] == scenario and r["rows"] == rows:
            return r
    return None


def filter_delay_ms(run):
    """Delay of the force sent behind the force before filtering, found as the
    shift that best maps one onto the other while the force is rising."""
    d, sent, raw = run["data"], run["force"], run["data"]["f_raw"]
    start = int(np.argmax(d["contacts"] > 0))
    end   = int(np.argmax(sent)) if np.argmax(sent) > start + 50 else len(sent) - 1
    best, best_err = 0, np.inf
    for lag in range(0, 80):
        a = sent[start + lag:end]
        b = raw[start:end - lag]
        n = min(len(a), len(b))
        if n < 20:
            break
        err = np.mean((a[:n] - b[:n]) ** 2)
        if err < best_err:
            best, best_err = lag, err
    return best


def place_labels(ax, x, items, gap_pt=13):
    """Direct labels at the right end of lines, in ink, pushed apart so they
    never overlap. items: [(y, text)]."""
    fig = ax.figure
    fig.canvas.draw()
    to_pt = 72.0 / fig.dpi
    ys = [ax.transData.transform((x, y))[1] * to_pt for y, _ in items]
    order = sorted(range(len(items)), key=lambda i: ys[i])
    placed = {}
    last = -1e9
    for i in order:
        p = max(ys[i], last + gap_pt)
        placed[i] = p
        last = p
    for i, (y, text) in enumerate(items):
        ax.annotate(text, (x, y), xytext=(10, placed[i] - ys[i]), textcoords="offset points",
                    va="center", fontsize=9, color=INK)


# ------------------------------------------------------------------------------
# Figures
# ------------------------------------------------------------------------------

def fig_push(runs, rows=200):
    fig, axes = plt.subplots(3, 1, figsize=(8.5, 8.2), sharex=True)
    quantities = [("Force [N]", "force", 1.0), ("Penetration depth [mm]", "mean_depth", 1e3),
                  ("Colliding elements", "contacts", 1.0)]

    for ax, (label, key, scale) in zip(axes, quantities):
        for tag in ORDER:
            r = pick(runs, tag, "push", rows)
            if r is None:
                continue
            d = r["data"]
            y = r["force"] if key == "force" else d[key] * scale
            ax.plot(d["t"], y, color=COLOR[tag], label=NAME[tag], zorder=3)
            if key == "force" and tag == "baseline":
                ax.plot(d["t"], d["f_raw"], color=COLOR[tag], lw=1.0, alpha=0.45,
                        label="Baseline before its 10 Hz filter", zorder=2)
        ax.set_ylabel(label)
        ax.set_ylim(bottom=0)

    axes[0].set_title(f"Pushing the tool into the clay, wheel stopped ({rows} x 180 grid)", loc="left")
    axes[0].legend(loc="upper left", ncol=2)
    axes[-1].set_xlabel("Time [s]")
    fig.text(0.01, 0.005, "Depth: penetration averaged over the contact area. "
             "Method 2 has no filter; its force is continuous by construction.",
             fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(os.path.join(RESULTS, "fig1_push_force_depth_contacts.png"), dpi=150)
    plt.close(fig)


def fig_detail(runs, rows=400):
    fig, ax = plt.subplots(figsize=(8.5, 4.2))
    t0, t1 = 2.0, 2.1
    for tag in ORDER:
        r = pick(runs, tag, "groove", rows)
        d = r["data"]
        m = (d["t"] >= t0) & (d["t"] <= t1)
        if tag == "baseline":
            ax.plot(d["t"][m] * 1e3, d["f_raw"][m], color=COLOR[tag], lw=1.0, alpha=0.45,
                    label="Baseline before filter", drawstyle="steps-post")
        ax.plot(d["t"][m] * 1e3, r["force"][m], color=COLOR[tag], label=NAME[tag], drawstyle="steps-post")
    ax.set_title(f"100 ms of the groove at 1 kHz ({rows} x 360 grid, wheel turning)", loc="left")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Force [N]")
    ax.legend(loc="upper left", ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "fig2_force_detail.png"), dpi=150)
    plt.close(fig)


def fig_steps(runs, rows=400):
    items = []
    b = pick(runs, "baseline", "groove", rows)
    items.append(("Baseline, before filter", np.abs(np.diff(b["data"]["f_raw"])).max() * 1e3, COLOR["baseline"], 0.45))
    items.append(("Baseline, filtered (sent)", np.abs(np.diff(b["force"])).max() * 1e3, COLOR["baseline"], 1.0))
    r = pick(runs, "method2", "groove", rows)
    items.append((f"{NAME['method2']} (no filter)", np.abs(np.diff(r["force"])).max() * 1e3, COLOR["method2"], 1.0))

    fig, ax = plt.subplots(figsize=(8.5, 3.4))
    y = np.arange(len(items))[::-1]
    for yi, (label, v, c, a) in zip(y, items):
        ax.barh(yi, v, height=0.55, color=c, alpha=a)
        ax.annotate(f"{v:.1f} mN", (v, yi), xytext=(6, 0), textcoords="offset points",
                    va="center", fontsize=9, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels([i[0] for i in items])
    ax.set_xlabel("Largest force change between two 1 ms ticks [mN]")
    ax.set_title(f"Force steps over the whole groove ({rows} x 360 grid)", loc="left")
    ax.grid(axis="y", visible=False)
    ax.set_xlim(0, max(i[1] for i in items) * 1.18)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "fig3_force_steps.png"), dpi=150)
    plt.close(fig)


def fig_resolution(runs):
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    grids = [200, 800]
    labels = []
    for tag in ORDER:
        peaks = [pick(runs, tag, "push", g)["force"].max() for g in grids]
        ax.plot([0, 1], peaks, color=COLOR[tag], marker="o", ms=8, mec=SURFACE, mew=2)
        spread = abs(peaks[1] - peaks[0]) / max(peaks) * 100
        labels.append((peaks[1], f"{NAME[tag]}  {spread:.1f} %"))
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["200 x 180", "800 x 360"])
    ax.set_xlim(-0.2, 1.75)
    ax.set_ylabel("Peak force while pushing [N]")
    place_labels(ax, 1, labels)
    ax.set_title("Same push, coarse and fine grid", loc="left")
    fig.text(0.01, 0.01, "Label: change of the peak force between the two grids.", fontsize=8.5, color=INK2)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(RESULTS, "fig4_resolution.png"), dpi=150)
    plt.close(fig)


def fig_timing(runs):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    ends = [[], []]
    for tag in ORDER:
        rs = sorted([r for r in runs if r["tag"] == tag and r["scenario"] == "groove"],
                    key=lambda r: r["rows"] * r["cols"])
        n = [r["rows"] * r["cols"] / 1000 for r in rs]
        phys  = [np.mean(r["data"]["physics_us"]) for r in rs]
        servo = [np.mean(r["data"]["servo_us"]) for r in rs]
        axes[0].plot(n, phys, color=COLOR[tag], marker="o", ms=7, mec=SURFACE, mew=2)
        axes[1].plot(n, servo, color=COLOR[tag], marker="o", ms=7, mec=SURFACE, mew=2)
        ends[0].append((phys[-1], NAME[tag]))
        ends[1].append((servo[-1], NAME[tag]))
        xmax = n[-1]

    axes[0].set_title("Physics step (detection, deformation, propagation)", loc="left")
    axes[0].set_ylabel("Mean time per step [us]")
    axes[1].set_title("Work inside the 1 kHz loop per tick", loc="left")
    axes[1].set_ylabel("Mean time per tick [us], log scale")
    axes[1].set_yscale("log")
    axes[1].axhline(1000, color=INK2, lw=1, ls="--")
    axes[1].annotate("1 ms budget", (axes[1].get_xlim()[0], 1000), xytext=(4, 4),
                     textcoords="offset points", fontsize=8.5, color=INK2)
    for k, ax in enumerate(axes):
        ax.set_xlabel("Elements in the force grid [thousands]")
        ax.set_xlim(right=ax.get_xlim()[1] * 1.25)
        place_labels(ax, xmax, ends[k])
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "fig5_timing.png"), dpi=150)
    plt.close(fig)


# ------------------------------------------------------------------------------
# Summary table
# ------------------------------------------------------------------------------

def summary(runs):
    lines = ["# Benchmark summary", "",
             "Data from `VirtualPottery --bench`, summarised by `Evaluation/plot_comparison.py`.", "",
             "| | Baseline | Method 2 |", "|---|---|---|"]

    def row(name, fn):
        lines.append("| " + name + " | " + " | ".join(fn(t) for t in ORDER) + " |")

    row("Physics rate", lambda t: "1000 Hz (in servo loop)" if t == "baseline" else "250 Hz (own thread)")
    row("Peak push force, 200x180 grid [N]", lambda t: f"{pick(runs, t, 'push', 200)['force'].max():.3f}")
    row("Peak push force, 800x360 grid [N]", lambda t: f"{pick(runs, t, 'push', 800)['force'].max():.3f}")
    row("Change between grids", lambda t: "%.1f %%" % (
        abs(pick(runs, t, 'push', 800)['force'].max() - pick(runs, t, 'push', 200)['force'].max())
        / max(pick(runs, t, 'push', 800)['force'].max(), pick(runs, t, 'push', 200)['force'].max()) * 100))
    row("Largest step before filter, groove 400x360 [mN]",
        lambda t: f"{np.abs(np.diff(pick(runs, t, 'groove', 400)['data']['f_raw'])).max() * 1e3:.1f}")
    row("Largest step sent, groove 400x360 [mN]",
        lambda t: f"{np.abs(np.diff(pick(runs, t, 'groove', 400)['force'])).max() * 1e3:.1f}")
    row("Delay added by the force filter [ms]",
        lambda t: f"{filter_delay_ms(pick(runs, t, 'push', 200))}" if t == "baseline" else "0 (no filter)")
    for g in [200, 400, 800, 1600]:
        row(f"Physics step, groove {g}x{pick(runs, 'baseline', 'groove', g)['cols']} [us]",
            lambda t, g=g: f"{np.mean(pick(runs, t, 'groove', g)['data']['physics_us']):.0f}")
    row("1 kHz loop work, 1600x720: mean / max [us]",
        lambda t: "%.1f / %.0f" % (np.mean(pick(runs, t, 'groove', 1600)['data']['servo_us']),
                                   np.max(pick(runs, t, 'groove', 1600)['data']['servo_us'])))
    row("Servo rate, 1600x720 [Hz]", lambda t: "%.0f" % (
        len(pick(runs, t, 'groove', 1600)['force'])
        / (pick(runs, t, 'groove', 1600)['data']['t'][-1] - pick(runs, t, 'groove', 1600)['data']['t'][0])))

    with open(os.path.join(RESULTS, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


# ------------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) > 1:
        RESULTS = sys.argv[1]
    runs = load_runs()
    fig_push(runs)
    fig_detail(runs)
    fig_steps(runs)
    fig_resolution(runs)
    fig_timing(runs)
    summary(runs)
    print("figures written to", os.path.abspath(RESULTS))
