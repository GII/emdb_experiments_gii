#!/usr/bin/env python3
"""Progress of a UR5 e-MDB run: pace, ETA, success rate, steps per success, policy learning.

Usage:  progress.py [run folder]     (default: the newest run folder under the results folder,
        $EMDB_RESULTS or <workspace>/results; see compare_runs.find_results)

The times of the iterations come from <run>/perf.txt, where the main loop writes the end of each iteration
(run_ur5.sh sets EMDB_PERF_FILE to it), and the number of iterations of the run from its experiment file.
"""
import csv
import glob
import os
import statistics
import sys
import time

import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
from compare_runs import find_results  # noqa: E402

EXPERIMENTS = os.path.join(HERE, "..", "..", "experiments", "experiments")  # the yamls of the experiments package


def newest_run():
    runs = glob.glob(os.path.join(find_results(), "*", "seed*_*"))
    return max(runs, key=os.path.getmtime) if runs else None


def iteration_times(perf_path):
    """(first, last) (time, iteration) of the main loop, from the lines "<iteration> <time>" of perf.txt."""
    rows = []
    if os.path.exists(perf_path):
        with open(perf_path) as f:
            for line in f:
                c = line.split()
                if len(c) == 2 and c[0].isdigit():
                    rows.append((float(c[1]), int(c[0])))
    return (rows[0], rows[-1]) if rows else (None, None)


def planned_iterations(run):
    """Iterations of the run, from its experiment file (the run folder is <results>/<experiment>/seed<N>_<date>)."""
    experiment = os.path.basename(os.path.dirname(os.path.abspath(run)))
    try:
        with open(os.path.join(EXPERIMENTS, experiment + ".yaml")) as f:
            return int(yaml.safe_load(f)["Experiment"]["parameters"]["iterations"])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else newest_run()
    if not run:
        sys.exit("No run folder found")
    print(f"Run: {run}")
    first, last = iteration_times(os.path.join(run, "perf.txt"))
    if last:
        total = planned_iterations(run)
        elapsed = last[0] - first[0]
        pace = elapsed / max(1, last[1] - first[1])
        eta = f"ETA {pace*(total-last[1])/3600:.1f} h  " if total else ""
        print(f"Iteration {last[1]}/{total or '?'}  elapsed {elapsed/3600:.1f} h  {pace:.1f} s/iteration  "
              f"{eta}(last iteration ended {time.time()-last[0]:.0f} s ago)")
    else:
        print("No iteration times yet: perf.txt is missing or empty (run_ur5.sh sets EMDB_PERF_FILE)")

    rows = []
    path = os.path.join(run, "trials_0.txt")
    if os.path.exists(path):
        for line in open(path):
            c = line.strip().split("\t")
            if len(c) in (4, 5) and c[0].isdigit():
                rows.append((int(c[0]), int(c[-2]), c[-1] == "True"))
    if rows:
        ok = [r for r in rows if r[2]]
        print(f"Trials {len(rows)}, successful {len(ok)}" + (f", first at iteration {ok[0][0]}" if ok else ""))
        for n in (20, 50):
            if len(rows) >= n:
                w = rows[-n:]
                w_ok = [r[1] for r in w if r[2]]
                med = f", median {statistics.median(w_ok):.0f} steps per success" if w_ok else ""
                print(f"  last {n} trials: {100*len(w_ok)/n:.0f} % success{med}")

    for traces in glob.glob(os.path.join(run, "logs", "policy_*", "traces.csv")):
        with open(traces) as f:
            src = [r["source_policy"] or "novelty_exploration" for r in csv.DictReader(f)]  # exploration episodes have no parent policy
        counts = {s: src.count(s) for s in sorted(set(src))}
        prog = os.path.join(os.path.dirname(traces), "progress.csv")
        trainings = max(0, sum(1 for _ in open(prog)) - 1) if os.path.exists(prog) else 0
        print(f"{os.path.basename(os.path.dirname(traces))}: {len(src)} traces {counts}, {trainings} SAC trainings")


if __name__ == "__main__":
    main()
