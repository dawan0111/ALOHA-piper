import rclpy
from rclpy.node import Node
import h5py
import matplotlib.pyplot as plt
import numpy as np
import time
import sys
from pathlib import Path

def print_hdf5_summary(hdf5_path):
    def walk(name, obj):
        indent = "│   " * name.count('/')
        if isinstance(obj, h5py.Dataset):
            print(f"{indent}├── {name.split('/')[-1]}: shape={obj.shape}, dtype={obj.dtype}")
        elif isinstance(obj, h5py.Group):
            print(f"{indent}├── {name.split('/')[-1]}")

    print("[INFO] HDF5 Structure Summary")
    with h5py.File(hdf5_path, 'r') as f:
        f.visititems(walk)

def load_data(hdf5_path):
    with h5py.File(hdf5_path, 'r') as f:
        images = {k: f['observations/images'][k][:] for k in f['observations/images'].keys()}

        if 'observations/qpos' not in f:
            raise RuntimeError("HDF5 file missing required dataset: observations/qpos")
        joint_pos = f['observations/qpos'][:]

        joint_cmd = None
        if 'observations/action' in f:
            joint_cmd = f['observations/action'][:]
        else:
            print("[WARN] action not found. Skipping command plots.")

    return images, joint_pos, joint_cmd


def play_episode(images, joint_pos, joint_cmd=None, fps=30.0):
    keys = list(images.keys())
    n_frames = min([images[k].shape[0] for k in keys] + [joint_pos.shape[0]])
    if joint_cmd is not None:
        n_frames = min(n_frames, joint_cmd.shape[0])
    
    interval = 1.0 / fps
    num_joints = joint_pos.shape[1]

    fig = plt.figure(figsize=(12, 8))
    axs_img = [fig.add_subplot(2, 3, i+1) for i in range(4)]
    ax_joint = fig.add_subplot(2, 3, 5)
    lines_pos, lines_cmd = [], []

    t_vals = np.arange(n_frames)

    for j in range(num_joints):
        (l_pos,) = ax_joint.plot([], [], label=f'j{j}_pos', linestyle='-')
        lines_pos.append(l_pos)
        if joint_cmd is not None:
            (l_cmd,) = ax_joint.plot([], [], label=f'j{j}_cmd', linestyle='--')
            lines_cmd.append(l_cmd)

    ax_joint.set_xlim(0, n_frames)
    all_vals = [joint_pos]
    if joint_cmd is not None:
        all_vals.append(joint_cmd)
    vmin = min([a.min() for a in all_vals])
    vmax = max([a.max() for a in all_vals])
    ax_joint.set_ylim(vmin - 0.1, vmax + 0.1)

    ax_joint.set_title("Joint Positions vs Commands")
    ax_joint.set_xlabel("Timestep")
    ax_joint.set_ylabel("Value")
    ax_joint.grid()
    ax_joint.legend(loc='upper right')

    for t in range(n_frames):
        for i in range(4):
            axs_img[i].clear()
            if i < len(keys):
                axs_img[i].imshow(images[keys[i]][t])
                axs_img[i].set_title(keys[i])
            axs_img[i].axis('off')

        for j in range(num_joints):
            lines_pos[j].set_data(t_vals[:t+1], joint_pos[:t+1, j])
            if joint_cmd is not None:
                lines_cmd[j].set_data(t_vals[:t+1], joint_cmd[:t+1, j])

        ax_joint.relim()
        ax_joint.autoscale_view()
        plt.suptitle(f"Timestep {t}", fontsize=14)
        plt.pause(interval)

    plt.close()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run aloha_piper_viewer episode_viewer /path/to/file.hdf5")
        return

    hdf5_path = sys.argv[1]
    if not Path(hdf5_path).exists():
        print(f"[ERROR] Invalid path: {hdf5_path}")
        return

    print_hdf5_summary(hdf5_path)
    input("[INFO] Press Enter to start playback...")

    images, joint_pos, joint_cmd = load_data(hdf5_path)
    play_episode(images, joint_pos, joint_cmd)

    rclpy.shutdown()