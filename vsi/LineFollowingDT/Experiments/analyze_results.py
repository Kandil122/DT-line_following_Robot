#!/usr/bin/env python3
import argparse
import csv
import os
from collections import defaultdict
from dataclasses import dataclass
from statistics import mean, median
from typing import Callable, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class RunRow:
    experiment: str
    run_name: str
    path: str
    path_length: Optional[float]
    kp: Optional[float]
    ki: Optional[float]
    kd: Optional[float]
    noise: Optional[float]
    disturbance: Optional[float]
    final_x: Optional[float]
    overshoot: Optional[float]
    settling_s: Optional[float]
    sse: Optional[float]


def _to_float(v: Optional[str]) -> Optional[float]:
    if v is None:
        return None
    v = str(v).strip()
    if v == "" or v.lower() == "none":
        return None
    try:
        return float(v)
    except Exception:
        return None


def load_rows(csv_path: str, default_experiment: str = "") -> List[RunRow]:
    rows: List[RunRow] = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return rows

        # New format (LineFollowingRobot/src/visualizer.py after March 2026 edits)
        new_cols = {
            "Experiment",
            "RunName",
            "Path",
            "PathLength",
            "Kp",
            "Ki",
            "Kd",
            "Noise",
            "Disturbance",
            "FinalX",
            "Overshoot",
            "SettlingTimeS",
            "SteadyStateError",
        }

        fieldset = set(reader.fieldnames)
        is_new = new_cols.issubset(fieldset)

        for r in reader:
            if is_new:
                rows.append(
                    RunRow(
                        experiment=((r.get("Experiment") or "").strip() or default_experiment),
                        run_name=(r.get("RunName") or "").strip(),
                        path=(r.get("Path") or "").strip(),
                        path_length=_to_float(r.get("PathLength")),
                        kp=_to_float(r.get("Kp")),
                        ki=_to_float(r.get("Ki")),
                        kd=_to_float(r.get("Kd")),
                        noise=_to_float(r.get("Noise")),
                        disturbance=_to_float(r.get("Disturbance")),
                        final_x=_to_float(r.get("FinalX")),
                        overshoot=_to_float(r.get("Overshoot")),
                        settling_s=_to_float(r.get("SettlingTimeS")),
                        sse=_to_float(r.get("SteadyStateError")),
                    )
                )
            else:
                # Old format (kept for backward compatibility)
                # Path,Kp,Ki,Kd,Noise,Disturbance,Overshoot,SettlingTime,SteadyStateError
                rows.append(
                    RunRow(
                        experiment=((r.get("Experiment") or "").strip() or default_experiment),
                        run_name=(r.get("RunName") or "").strip(),
                        path=(r.get("Path") or "").strip(),
                        path_length=_to_float(r.get("PathLength")),
                        kp=_to_float(r.get("Kp")),
                        ki=_to_float(r.get("Ki")),
                        kd=_to_float(r.get("Kd")),
                        noise=_to_float(r.get("Noise")),
                        disturbance=_to_float(r.get("Disturbance")),
                        final_x=_to_float(r.get("FinalX")),
                        overshoot=_to_float(r.get("Overshoot")),
                        settling_s=_to_float(r.get("SettlingTimeS") or r.get("SettlingTime")),
                        sse=_to_float(r.get("SteadyStateError")),
                    )
                )
    return rows


def group_stats(rows: List[RunRow], key_fn: Callable[[RunRow], Tuple]) -> List[Dict]:
    groups: Dict[Tuple, List[RunRow]] = defaultdict(list)
    for r in rows:
        groups[key_fn(r)].append(r)
    out = []
    for k, rs in sorted(groups.items(), key=lambda kv: str(kv[0])):
        overs = [x.overshoot for x in rs if x.overshoot is not None]
        sse = [x.sse for x in rs if x.sse is not None]
        sett = [x.settling_s for x in rs if x.settling_s is not None]
        out.append(
            {
                "key": k,
                "n": len(rs),
                "overshoot_mean": mean(overs) if overs else None,
                "overshoot_median": median(overs) if overs else None,
                "sse_mean": mean(sse) if sse else None,
                "sse_median": median(sse) if sse else None,
                "settling_mean_s": mean(sett) if sett else None,
                "settling_median_s": median(sett) if sett else None,
            }
        )
    return out


def fmt(v):
    if v is None:
        return ""
    return f"{v:.4g}"


def write_markdown(path: str, title: str, content: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# {title}\n\n")
        f.write(content)


def md_table(headers: list[str], rows: list[list[str]]) -> str:
    out = []
    out.append("| " + " | ".join(headers) + " |")
    out.append("|" + "|".join(["---"] * len(headers)) + "|")
    for r in rows:
        out.append("| " + " | ".join(r) + " |")
    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--csv",
        default="output/results/results.csv",
        help="Input results CSV (relative to LineFollowingRobot/ when run from there)",
    )
    ap.add_argument(
        "--default_experiment",
        default="",
        help="Fill empty Experiment cells with this value (useful for older CSVs like E3_results.csv).",
    )
    ap.add_argument("--outdir", default="output/analysis", help="Output directory")
    ap.add_argument("--success_x_frac", type=float, default=0.98, help="E3 success: FinalX >= frac*PathLength")
    ap.add_argument("--success_sse", type=float, default=0.2, help="E3 success: SSE <= this")
    args = ap.parse_args()

    csv_path = args.csv
    if not os.path.isfile(csv_path):
        raise SystemExit(f"results CSV not found: {csv_path}")

    rows = load_rows(csv_path, default_experiment=args.default_experiment)
    if not rows:
        raise SystemExit(f"no rows found in: {csv_path}")

    # Per-condition summary: Experiment + Path + gains + noise/disturbance.
    def key_fn(r: RunRow) -> Tuple:
        return (
            r.experiment,
            r.path,
            r.kp,
            r.ki,
            r.kd,
            r.noise,
            r.disturbance,
        )

    stats = group_stats(rows, key_fn)

    # Pick "best" E1 gains as lowest SSE (then overshoot).
    e1_stats = [s for s in stats if s["key"][0] == "E1" and s["key"][1] == "straight"]
    best_e1 = None
    best_score = None
    for s in e1_stats:
        sse_m = s["sse_mean"]
        o_m = s["overshoot_mean"]
        t_m = s["settling_mean_s"]
        if sse_m is None or o_m is None:
            continue
        score = float(sse_m) + 0.5 * float(o_m) + 0.05 * float(t_m or 0.0)
        if best_score is None or score < best_score:
            best_score = score
            best_e1 = s

    # E3 success rate by noise/disturbance.
    e3_rows = [r for r in rows if r.experiment == "E3"]
    e3_groups: Dict[Tuple, List[RunRow]] = defaultdict(list)
    for r in e3_rows:
        e3_groups[(r.noise, r.disturbance)].append(r)

    e3_rate_rows = []
    for (noise, disturbance), rs in sorted(e3_groups.items(), key=lambda kv: (kv[0][0] or 0.0, kv[0][1] or 0.0)):
        ok = 0
        tot = 0
        for r in rs:
            if r.path_length is None or r.final_x is None or r.sse is None:
                continue
            tot += 1
            if (r.final_x >= args.success_x_frac * r.path_length) and (r.sse <= args.success_sse):
                ok += 1
        rate = (ok / tot) if tot else None
        e3_rate_rows.append([fmt(noise), fmt(disturbance), str(ok), str(tot), fmt(rate)])

    # Markdown report
    md = []
    md.append(f"- Input: `{csv_path}`\n")
    if best_e1 is not None:
        k = best_e1["key"]
        md.append(
            f"- Best E1 gains (by mean SSE + overshoot): Kp={fmt(k[2])} Ki={fmt(k[3])} Kd={fmt(k[4])}\n"
        )
    md.append("\n## Per-Condition KPIs\n\n")
    table_rows = []
    for s in stats:
        (exp, path, kp, ki, kd, noise, disturbance) = s["key"]
        table_rows.append(
            [
                exp,
                path,
                fmt(kp),
                fmt(ki),
                fmt(kd),
                fmt(noise),
                fmt(disturbance),
                str(s["n"]),
                fmt(s["overshoot_mean"]),
                fmt(s["settling_mean_s"]),
                fmt(s["sse_mean"]),
            ]
        )
    md.append(
        md_table(
            [
                "Exp",
                "Path",
                "Kp",
                "Ki",
                "Kd",
                "Noise",
                "Dist",
                "N",
                "OvershootMean",
                "SettlingMeanS",
                "SSEMean",
            ],
            table_rows,
        )
    )

    md.append("\n## E3 Success Rate\n\n")
    if e3_rate_rows:
        md.append(md_table(["Noise", "Dist", "OK", "Total", "Rate"], e3_rate_rows))
        md.append(f"\nSuccess rule: `FinalX >= {args.success_x_frac} * PathLength` and `SSE <= {args.success_sse}`.\n")
    else:
        md.append("No E3 rows found.\n")

    out_md = os.path.join(args.outdir, "summary.md")
    write_markdown(out_md, "Line Following Experiments Summary", "".join(md))

    # Also write a compact CSV summary for quick plotting elsewhere.
    out_csv = os.path.join(args.outdir, "summary.csv")
    os.makedirs(args.outdir, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "Experiment",
                "Path",
                "Kp",
                "Ki",
                "Kd",
                "Noise",
                "Disturbance",
                "N",
                "OvershootMean",
                "SettlingMeanS",
                "SSEMean",
            ]
        )
        for s in stats:
            (exp, path, kp, ki, kd, noise, disturbance) = s["key"]
            w.writerow(
                [
                    exp,
                    path,
                    kp,
                    ki,
                    kd,
                    noise,
                    disturbance,
                    s["n"],
                    s["overshoot_mean"],
                    s["settling_mean_s"],
                    s["sse_mean"],
                ]
            )

    print(f"Wrote: {out_md}")
    print(f"Wrote: {out_csv}")

    # Optional plots (skip if matplotlib isn't installed in the environment).
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return 0

    os.makedirs(args.outdir, exist_ok=True)

    # E1 plot: mean SSE per gain set.
    if e1_stats:
        labels = []
        y = []
        for s in e1_stats:
            (exp, path, kp, ki, kd, _noise, _dist) = s["key"]
            labels.append(f"Kp={fmt(kp)} Ki={fmt(ki)} Kd={fmt(kd)}")
            y.append(s["sse_mean"] if s["sse_mean"] is not None else float("nan"))
        plt.figure(figsize=(10, 4))
        plt.bar(range(len(y)), y)
        plt.xticks(range(len(y)), labels, rotation=45, ha="right", fontsize=8)
        plt.ylabel("Mean SSE")
        plt.title("E1: Mean SSE by Gain Set (Straight)")
        plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, "e1_mean_sse.png"), dpi=160)
        plt.close()

    # E3 plot: success rate by (noise, disturbance).
    if e3_rate_rows:
        xs = []
        ys = []
        for noise_s, dist_s, ok_s, tot_s, rate_s in e3_rate_rows:
            xs.append(f"n={noise_s}, d={dist_s}")
            ys.append(float(rate_s) if rate_s else float("nan"))
        plt.figure(figsize=(10, 4))
        plt.bar(range(len(ys)), ys)
        plt.ylim(0.0, 1.0)
        plt.xticks(range(len(ys)), xs, rotation=45, ha="right", fontsize=8)
        plt.ylabel("Success Rate")
        plt.title("E3: Success Rate by Noise/Disturbance")
        plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, "e3_success_rate.png"), dpi=160)
        plt.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
