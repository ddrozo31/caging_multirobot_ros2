"""
Plots and error table of one or more laps (Step 10, like Fig. 12-15 and Table 2 of the paper).

Input: the CSV files written by trajectory_tracker (one per lap, 20 Hz).
Output, for each CSV:  <name>.png with four plots
    (a) Fig. 12: XY paths of the object, the leader, the follower and the desired circle
    (b) Fig. 13: x_O and y_O versus time (and their references)
    (c) Fig. 14: theta_L versus time (and theta*)
    (d) Fig. 15: d, alpha and theta_F error versus time
and, with --table, one Markdown table of the mean errors of all the CSV files.

Step 11:
  - Repeated runs: files whose names differ only in the date (<date>_<time>_box_static_smooth.csv)
    are ONE configuration. The table shows mean ± std over the runs (std of the per-run means).
  - Object retention: for each run, the largest distance of the object from the midpoint of the
    robots ("cage" column). The object is RETAINED if it never exceeds --cage-tolerance
    (0.05 m, the same limit as safety_supervisor). Runs that were aborted write no CSV: count
    them with the summary of run_experiments.sh.

No ROS here: only csv, math and matplotlib.

Run:
  ros2 run caging_control plot_results results/*.csv
  ros2 run caging_control plot_results results/*.csv --table --out docs/figures
  ros2 run caging_control plot_results results/repeat/*.csv --table --no-plots    # Step 11
  ros2 run caging_control plot_results results/repeat/*.csv --no-plots --runs-csv runs.csv
      # one row per run (mean errors, max cage, retained): a small file to keep instead of all CSVs
"""
import argparse
import csv
import math
import os
import re
import statistics

import matplotlib
matplotlib.use('Agg')               # draw to files, no window needed
import matplotlib.pyplot as plt     # noqa: E402

# (column, label, unit, Table 2 values of the box: experiment 1, experiment 2)
ERRORS = [('e_dO', 'e_dO', 'm', 0.10, 0.13), ('e_xO', 'e_xO', 'm', 0.07, 0.09),
          ('e_yO', 'e_yO', 'm', 0.06, 0.07), ('e_thL', 'e_θL', 'rad', 0.08, 0.15),
          ('e_d', 'e_d', 'm', 0.01, 0.01), ('e_thF', 'e_θF', 'rad', 0.09, 0.10),
          ('e_alpha', 'e_α', 'rad', 0.06, 0.09)]


def read_csv(path):
    """CSV -> dict of columns (lists of floats)."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: [float(row[key]) for row in rows] for key in rows[0]}


def unwrap(angles):
    """Remove the jumps of +-2 pi, so an angle plot is a continuous line."""
    out = [angles[0]]
    for a in angles[1:]:
        out.append(out[-1] + math.remainder(a - out[-1], 2 * math.pi))
    return out


def plot_lap(data, title, path):
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    t = data['t']

    # (a) Fig. 12: paths in the plane
    a = ax[0][0]
    a.plot(data['x_ref'], data['y_ref'], 'k--', label='desired (object)')
    a.plot(data['x_O'], data['y_O'], color='tab:orange', label='object')
    a.plot(data['x_L'], data['y_L'], color='tab:red', label='leader')
    a.plot(data['x_F'], data['y_F'], color='tab:blue', label='follower')
    a.set_xlabel('x [m]'), a.set_ylabel('y [m]'), a.set_aspect('equal'), a.grid(True)
    a.legend(fontsize=8), a.set_title('(a) paths (Fig. 12)')

    # (b) Fig. 13: object position versus time
    a = ax[0][1]
    a.plot(t, data['x_ref'], 'k--', lw=1, label='x*')
    a.plot(t, data['x_O'], color='tab:orange', label='x_O')
    a.plot(t, data['y_ref'], 'k:', lw=1, label='y*')
    a.plot(t, data['y_O'], color='tab:green', label='y_O')
    a.set_xlabel('t [s]'), a.set_ylabel('[m]'), a.grid(True), a.legend(fontsize=8)
    a.set_title('(b) object position (Fig. 13)')

    # (c) Fig. 14: leader heading = orientation of the system
    a = ax[1][0]
    a.plot(t, unwrap(data['th_ref']), 'k--', label='θ*')
    a.plot(t, unwrap(data['th_L']), color='tab:red', label='θ_L')
    a.set_xlabel('t [s]'), a.set_ylabel('[rad]'), a.grid(True), a.legend(fontsize=8)
    a.ticklabel_format(useOffset=False)   # show real values (no "+3.141" offset on the axis)
    a.set_title('(c) leader orientation (Fig. 14)')

    # (d) Fig. 15: formation errors
    a = ax[1][1]
    a.plot(t, [1000 * (d - r) for d, r in zip(data['d'], data['d_ref'])], label='d − d* [mm]')
    a.plot(t, [math.degrees(x) for x in data['alpha']], label='α [deg]')
    a.plot(t, [math.degrees(x) for x in data['e_thF']], label='|θ_F − θ_F*| [deg]')
    a.set_xlabel('t [s]'), a.grid(True), a.legend(fontsize=8)
    a.set_title('(d) Leader-Follower state (Fig. 15)')

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def configuration(name):
    """'20261019_083654_box_static_smooth' -> 'box_static_smooth' (remove the date and time)."""
    return re.sub(r'^\d{8}_\d{6}_', '', name)


def mean_and_std(values, decimals):
    """'0.047' for one run, '0.047 ± 0.002' for several runs."""
    mean = statistics.mean(values)
    if len(values) == 1:
        return f'{mean:.{decimals}f}'
    return f'{mean:.{decimals}f} ± {statistics.stdev(values):.{decimals}f}'


def main():
    parser = argparse.ArgumentParser(description='Plots and error table of trajectory laps (Steps 10-11)')
    parser.add_argument('csv', nargs='+', help='CSV files written by trajectory_tracker')
    parser.add_argument('--out', default=None, help='folder for the PNG files (default: next to each CSV)')
    parser.add_argument('--table', action='store_true', help='print a Markdown table of the mean errors')
    parser.add_argument('--no-plots', action='store_true', help='only the table, no PNG files')
    parser.add_argument('--runs-csv', default=None, help='write one summary row per run to this CSV file')
    parser.add_argument('--cage-tolerance', type=float, default=0.05,
                        help='object retained if its distance to the robots midpoint stays below this [m]')
    args = parser.parse_args()

    # One entry per configuration: {configuration: [results of each run]}
    groups = {}
    for path in sorted(args.csv):
        data = read_csv(path)
        name = os.path.splitext(os.path.basename(path))[0]
        if not args.no_plots:
            out_dir = args.out or os.path.dirname(os.path.abspath(path))
            os.makedirs(out_dir, exist_ok=True)
            png = os.path.join(out_dir, name + '.png')
            plot_lap(data, name, png)
            print(f'written {png}')
        run = {key: sum(data[key]) / len(data[key]) for key, *_ in ERRORS}   # mean error of this lap
        run['max_cage'] = max(data['cage'])                                  # worst cage of this lap
        run['retained'] = int(run['max_cage'] < args.cage_tolerance)
        run['file'] = name
        groups.setdefault(configuration(name), []).append(run)

    if args.runs_csv:
        with open(args.runs_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            keys = [key for key, *_ in ERRORS] + ['max_cage', 'retained']
            writer.writerow(['configuration', 'file'] + keys)
            for config, runs in groups.items():
                for r in runs:
                    writer.writerow([config, r['file']] + [f'{r[k]:.5f}' if k != 'retained' else r[k]
                                                           for k in keys])
        print(f'written {args.runs_csv}')

    if args.table:
        print()
        print('| configuration | runs | ' + ' | '.join(f'{label} [{unit}]' for _, label, unit, *_ in ERRORS)
              + ' | max cage [mm] | retained |')
        print('|---|---|' + '---|' * len(ERRORS) + '---|---|')
        for config, runs in groups.items():
            errors = ' | '.join(mean_and_std([r[key] for r in runs], 3) for key, *_ in ERRORS)
            worst_cage = max(r['max_cage'] for r in runs)
            retained = sum(r['max_cage'] < args.cage_tolerance for r in runs)
            print(f'| {config} | {len(runs)} | {errors} | {worst_cage * 1000:.1f} | {retained}/{len(runs)} |')
        print('| paper, box, exp. 1 | 1? | ' + ' | '.join(f'{e1:.2f}' for *_, e1, _ in ERRORS) + ' | – | – |')
        print('| paper, box, exp. 2 | 1? | ' + ' | '.join(f'{e2:.2f}' for *_, e2 in ERRORS) + ' | – | – |')


if __name__ == '__main__':
    main()
