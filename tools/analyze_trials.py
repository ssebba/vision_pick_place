#!/usr/bin/env python3
"""Offline analysis of the pick-and-place trials (output of pp_bringup run_trials.sh).

  tools/analyze_trials.py <results_dir> [--out docs/results]

Writes summary.md (tables for the README), trials.csv (one row per object), tracking.csv
(one row per trial, from the controller state bags) and the figure trials.png, with its dark
variant trials_dark.png. Analysis only: nothing here runs on the robot. Run it from a sourced
workspace (rosbag2_py, xacro and pinocchio read the bags and the robot model).
"""

import argparse
import os
import subprocess

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

CONTROLLERS = ['arm_controller', 'ctc_controller']
LABELS = {'arm_controller': 'Joint trajectory (stock, position)',
          'ctc_controller': 'Computed torque (ours, effort)'}
SHORT = {'arm_controller': 'JTC', 'ctc_controller': 'CTC'}
TCP_FRAME = 'gripper_tcp'
# Shapes of object_<i>, cycled as pp_bringup's sim.launch.py generates them (OBJECT_TYPES).
TRUE_SHAPES = ['box', 'cylinder', 'box', 'cylinder', 'box']
MOVING_SPEED = 0.01  # [rad/s] reference speed above which the arm counts as moving

# Light and dark themes: categorical slots 1-2 and chart chrome of the reference palette.
THEMES = {
    'light': {'surface': '#fcfcfb', 'ink': '#0b0b0b', 'ink2': '#52514e', 'muted': '#898781',
              'grid': '#e1e0d9', 'axis': '#c3c2b7',
              'series': {'arm_controller': '#2a78d6', 'ctc_controller': '#eb6834'}},
    'dark': {'surface': '#1a1a19', 'ink': '#f0efec', 'ink2': '#c3c2b7', 'muted': '#898781',
             'grid': '#2c2c2a', 'axis': '#383835',
             'series': {'arm_controller': '#3987e5', 'ctc_controller': '#d95926'}},
}


def load(results_dir):
    df = pd.read_csv(os.path.join(results_dir, 'pick_place.csv'))
    parts = df['run_id'].str.extract(r'seed(?P<seed>\d+)_(?P<controller>.+)')
    df = pd.concat([df, parts], axis=1)
    df['seed'] = df['seed'].astype(int)
    # A trial run twice (resumed campaign) keeps its last attempt only.
    last = df.groupby('run_id')['total_time_s'].transform('last')
    return df[df['total_time_s'] == last]


def finished_trials(results_dir):
    """Trials whose task finished, per controller (a crashed trial writes no CSV row)."""
    logs = os.path.join(results_dir, 'logs')
    counts = {}
    for name in sorted(os.listdir(logs)) if os.path.isdir(logs) else []:
        controller = name.removesuffix('.log').split('_', 1)[1]
        with open(os.path.join(logs, name), encoding='utf-8', errors='replace') as f:
            done = 'Task finished' in f.read()
        total, ok = counts.get(controller, (0, 0))
        counts[controller] = (total + 1, ok + int(done))
    return counts


class TcpKinematics:
    """Forward kinematics of the tool center point, from the installed robot description."""

    def __init__(self):
        import pinocchio
        from ament_index_python.packages import get_package_share_directory

        xacro_file = os.path.join(get_package_share_directory('pp_description'), 'urdf',
                                  'pp_robot.urdf.xacro')
        urdf = subprocess.run(['xacro', xacro_file], check=True, capture_output=True,
                              text=True).stdout
        self._pin = pinocchio
        self._model = pinocchio.buildModelFromXML(urdf)
        self._data = self._model.createData()
        self._frame = self._model.getFrameId(TCP_FRAME)

    def positions(self, joint_names, q):
        """TCP positions [n x 3] for joint positions q [n x len(joint_names)]."""
        index = [self._model.idx_qs[self._model.getJointId(name)] for name in joint_names]
        config = self._pin.neutral(self._model)
        out = np.empty((len(q), 3))
        for k, row in enumerate(q):
            config[index] = row
            self._pin.framesForwardKinematics(self._model, self._data, config)
            out[k] = self._data.oMf[self._frame].translation
        return out


def read_states(bag_dir):
    """Controller state samples of one trial: time, joint names, reference, feedback, speed."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir),
                rosbag2_py.ConverterOptions('', ''))
    msg_type = get_message('control_msgs/msg/JointTrajectoryControllerState')
    stamps, reference, feedback, speed, names = [], [], [], [], None
    while reader.has_next():
        data = reader.read_next_ext()[1]  # (topic, data, receive time, send time)
        msg = deserialize_message(data, msg_type)
        names = list(msg.joint_names)
        stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        reference.append(msg.reference.positions)
        feedback.append(msg.feedback.positions)
        speed.append(np.linalg.norm(msg.reference.velocities) if msg.reference.velocities else 0.0)
    return (np.array(stamps), names, np.array(reference), np.array(feedback), np.array(speed))


def tracking(results_dir):
    """One row per trial: joint and TCP tracking errors while moving and at rest."""
    bags = os.path.join(results_dir, 'bags')
    if not os.path.isdir(bags):
        return pd.DataFrame()
    kinematics = TcpKinematics()
    rows = []
    for run_id in sorted(os.listdir(bags)):
        stamps, names, ref, fb, speed = read_states(os.path.join(bags, run_id))
        if len(stamps) == 0:
            continue
        moving = speed > MOVING_SPEED
        joint_error = np.abs(ref - fb)
        tcp_error = np.linalg.norm(kinematics.positions(names, ref) -
                                   kinematics.positions(names, fb), axis=1)
        controller = run_id.split('_', 1)[1]
        rows.append({
            'run_id': run_id, 'controller': controller,
            'moving_s': moving.mean() * (stamps[-1] - stamps[0]),
            'joint_rms_mrad': 1000 * np.sqrt(np.mean(joint_error[moving] ** 2)),
            'joint_max_mrad': 1000 * joint_error[moving].max(),
            'tcp_rms_mm': 1000 * np.sqrt(np.mean(tcp_error[moving] ** 2)),
            'tcp_p99_mm': 1000 * np.percentile(tcp_error[moving], 99),
            'tcp_max_mm': 1000 * tcp_error[moving].max(),
            'tcp_rest_mm': 1000 * np.median(tcp_error[~moving]),
        })
    return pd.DataFrame(rows)


def markdown(table):
    lines = ['| ' + ' | '.join(table.columns) + ' |', '|' + '---|' * len(table.columns)]
    lines += ['| ' + ' | '.join(str(v) for v in r.values) + ' |' for _, r in table.iterrows()]
    return '\n'.join(lines) + '\n'


def summary(df, finished, track):
    rows = []
    for controller in CONTROLLERS:
        cdf = df[df['controller'] == controller]
        if cdf.empty:
            continue
        trials = cdf['run_id'].nunique()
        attempted, completed = finished.get(controller, (trials, trials))
        placed = cdf[cdf['placed'] == 1]
        complete = cdf.groupby('run_id')['placed'].all().sum()
        rows.append({
            'Arm controller': LABELS[controller],
            'Trials finished': f'{completed}/{attempted}',
            'Grasped': f"{int(cdf['grasped'].sum())}/{len(cdf)}",
            'Placed': f'{len(placed)}/{len(cdf)}',
            'Trials with every object placed': f'{complete}/{trials}',
            'Placement error [mm]': f"{1000 * placed['place_error_m'].mean():.1f} "
                                    f"(max {1000 * placed['place_error_m'].max():.1f})",
            'Cycle per object [s]': f"{placed['cycle_time_s'].mean():.1f} ± "
                                    f"{placed['cycle_time_s'].std():.1f}",
            'Planning per object [s]': f"{cdf['planning_time_s'].mean():.2f}",
        })
    text = '### Task\n\n' + markdown(pd.DataFrame(rows))

    truth = df['model'].str.extract(r'object_(\d+)')[0].astype(int) % len(TRUE_SHAPES)
    df = df.assign(true_shape=truth.map(dict(enumerate(TRUE_SHAPES))))
    df = df.assign(classified=df['shape'] == df['true_shape'])
    by_shape = df.groupby('true_shape').agg(
        objects=('placed', 'size'), classified=('classified', 'sum'), placed=('placed', 'sum'),
        detection=('detection_error_m', 'mean'), detection_max=('detection_error_m', 'max'))
    shape_rows = [{'Shape': shape, 'Objects': int(r.objects),
                   'Classified correctly': f'{int(r.classified)}/{int(r.objects)}',
                   'Placed': f'{int(r.placed)}/{int(r.objects)}',
                   'Detection error [mm]': f'{1000 * r.detection:.1f} '
                                           f'(max {1000 * r.detection_max:.1f})'}
                  for shape, r in by_shape.iterrows()]
    text += '\n### Perception\n\n' + markdown(pd.DataFrame(shape_rows))

    if not track.empty:
        track_rows = []
        for controller in CONTROLLERS:
            t = track[track['controller'] == controller]
            if t.empty:
                continue
            track_rows.append({
                'Arm controller': LABELS[controller],
                'Trials': len(t),
                'Joint error RMS [mrad]': f"{t['joint_rms_mrad'].mean():.2f}",
                'TCP error RMS [mm]': f"{t['tcp_rms_mm'].mean():.2f}",
                'TCP error 99th percentile [mm]': f"{t['tcp_p99_mm'].mean():.2f}",
                'TCP error at rest [mm]': f"{t['tcp_rest_mm'].median():.3f}",
            })
        text += ('\n### Tracking (while the reference moves)\n\n' +
                 markdown(pd.DataFrame(track_rows)))
    return text


def strip(ax, values_by_controller, theme, unit, digits):
    """One row per controller: every sample as a dot, the median as a bar and a label."""
    rng = np.random.default_rng(0)
    for row, controller in enumerate(CONTROLLERS):
        values = values_by_controller.get(controller)
        if values is None or len(values) == 0:
            continue
        y = row + rng.uniform(-0.12, 0.12, len(values))
        ax.scatter(values, y, s=30, color=theme['series'][controller],
                   edgecolors=theme['surface'], linewidths=1.0, zorder=3)
        median = float(np.median(values))
        ax.plot([median, median], [row - 0.28, row + 0.28], color=theme['ink'], lw=2,
                solid_capstyle='round', zorder=4)
        ax.annotate(f'median {median:.{digits}f} {unit}', (median, row - 0.3),
                    textcoords='offset points', xytext=(0, 2), ha='center', va='bottom',
                    fontsize=8, color=theme['ink2'])
    ax.set_ylim(len(CONTROLLERS) - 0.25, -0.6)  # first controller on top
    ax.set_yticks(range(len(CONTROLLERS)), [SHORT[c] for c in CONTROLLERS])
    ax.set_xlim(left=0)


def style(ax, theme, title):
    ax.set_facecolor(theme['surface'])
    ax.set_title(title, loc='left', fontsize=10, color=theme['ink'])
    ax.grid(axis='x', color=theme['grid'], lw=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    ax.spines['bottom'].set_color(theme['axis'])
    ax.tick_params(colors=theme['ink2'], labelsize=9, length=0)
    ax.tick_params(axis='y', labelsize=10, labelcolor=theme['ink'])


def plot(df, track, out):
    panels = [
        ('Cycle per placed object [s, simulated time]',
         {c: df[(df['controller'] == c) & (df['placed'] == 1)]['cycle_time_s'].to_numpy()
          for c in CONTROLLERS}, 's', 1),
    ]
    if not track.empty:
        panels.append((
            'TCP tracking error while moving, RMS per trial [mm]',
            {c: track[track['controller'] == c]['tcp_rms_mm'].to_numpy() for c in CONTROLLERS},
            'mm', 2))
    for name, theme in THEMES.items():
        fig, axes = plt.subplots(1, len(panels), figsize=(4.6 * len(panels), 2.3),
                                 squeeze=False)
        fig.patch.set_facecolor(theme['surface'])
        for ax, (title, values, unit, digits) in zip(axes[0], panels):
            style(ax, theme, title)
            strip(ax, values, theme, unit, digits)
        fig.tight_layout(w_pad=3)
        suffix = '' if name == 'light' else '_dark'
        fig.savefig(os.path.join(out, f'trials{suffix}.png'), dpi=150,
                    facecolor=theme['surface'], bbox_inches='tight')
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('results_dir')
    parser.add_argument('--out', default='docs/results')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    df = load(args.results_dir)
    track = tracking(args.results_dir)
    df.to_csv(os.path.join(args.out, 'trials.csv'), index=False, float_format='%.5g')
    if not track.empty:
        track.to_csv(os.path.join(args.out, 'tracking.csv'), index=False, float_format='%.5g')
    with open(os.path.join(args.out, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(summary(df, finished_trials(args.results_dir), track))
    plot(df, track, args.out)
    print(f"{df['run_id'].nunique()} trials, {len(df)} objects, {len(track)} state bags; "
          f'outputs in {args.out}')


if __name__ == '__main__':
    main()
