"""Evaluator-only overview of prepared physical spaces, never a policy input."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/osp_mission'))
from osp_mission.space_geometry import inside_polygon, rasterize_space


def render(statefile, output):
    state = json.loads(statefile.read_text())
    fig, axes = plt.subplots(2, 4, figsize=(16, 8.6))
    for axis, job in zip(axes.flat, state['jobs']):
        cfg = yaml.safe_load(Path(job['scenario']['file']).read_text())
        resolution = .1; occupied = rasterize_space(cfg, resolution)
        yy, xx = np.indices(occupied.shape)
        inside = inside_polygon((xx+.5)*resolution, (yy+.5)*resolution, cfg['arena']['polygon'])
        data = np.where(~inside, 0, np.where(occupied, 2, 1))
        axis.imshow(data, origin='lower', extent=(0, data.shape[1]*resolution, 0, data.shape[0]*resolution),
            cmap=ListedColormap(['#e8edf2', '#ffffff', '#30485f']), vmin=0, vmax=2, interpolation='nearest')
        x, y, yaw = cfg['start']
        axis.scatter(x, y, c='#178255', s=40, zorder=4)
        axis.arrow(x, y, .8*np.cos(yaw), .8*np.sin(yaw), width=.05, color='#178255', zorder=4)
        for target in cfg['targets']:
            x, y, yaw = target['x'], target['y'], target['yaw']
            axis.scatter(x, y, c='#d14738', s=24, marker='s', zorder=4)
            axis.arrow(x, y, .7*np.cos(yaw), .7*np.sin(yaw), width=.035, color='#d14738', zorder=4)
        shape = cfg['arena']['shape'].replace('_', ' ').title()
        axis.set_title(f"{shape} | {len(cfg['obstacles'])} obstacles", fontsize=12, fontweight='bold')
        axis.set_xlim(-.5, 27.5); axis.set_ylim(-.5, 20.5); axis.set_aspect('equal')
        axis.tick_params(labelsize=8); axis.set_xlabel('m', fontsize=9); axis.set_ylabel('m', fontsize=9)
    legend = axes.flat[-1]; legend.axis('off')
    legend.text(.05, .88, '7 new physical spaces', fontsize=18, weight='bold', color='#183a58')
    legend.text(.05, .68, 'Different exterior shapes and topology\nRandom obstacles, start pose and marker sizes\nWall and obstacle-mounted markers', fontsize=12, linespacing=1.8)
    legend.text(.05, .32, 'Green: robot start + heading\nRed: marker + facing direction', fontsize=12, linespacing=1.8)
    legend.text(.05, .10, 'Offline geometry preview\nNot a Gazebo success result', fontsize=11, color='#8d3b2a')
    fig.suptitle('Prepared Gazebo engineering screen — independent geometry, shared metric scale', fontsize=17, weight='bold')
    fig.tight_layout(rect=(0, 0, 1, .95)); output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor='white'); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('state', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); render(args.state.resolve(), args.output.resolve())
