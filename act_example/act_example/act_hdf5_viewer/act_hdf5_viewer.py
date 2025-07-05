import rclpy
from rclpy.node import Node
import h5py
import cv2
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

def show_images_cv2(image_dict, timestep, target_height=256):
    keys = list(image_dict.keys())[:4]
    imgs = [image_dict[k][timestep] for k in keys]

    h0, w0 = imgs[0].shape[:2]
    scale = target_height / h0
    target_width = int(w0 * scale)
    target_size = (target_width, target_height)

    rendered_imgs = []
    for i, img in enumerate(imgs):
        img_bgr = cv2.resize(img, target_size)
        key_name = keys[i]

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale_txt = 0.5
        thickness = 1
        color = (255, 255, 255)
        shadow = (0, 0, 0)

        text_size, _ = cv2.getTextSize(key_name, font, scale_txt, thickness)
        text_x = (img_bgr.shape[1] - text_size[0]) // 2
        text_y = img_bgr.shape[0] - 10

        cv2.putText(img_bgr, key_name, (text_x+1, text_y+1), font, scale_txt, shadow, thickness+1, cv2.LINE_AA)
        cv2.putText(img_bgr, key_name, (text_x, text_y), font, scale_txt, color, thickness, cv2.LINE_AA)

        rendered_imgs.append(img_bgr)

    top = np.hstack(rendered_imgs[:2])
    bottom = np.hstack(rendered_imgs[2:]) if len(rendered_imgs) > 2 else np.zeros_like(top)
    grid = np.vstack([top, bottom])

    cv2.imshow("Camera Views", grid)
    cv2.waitKey(1)



def play_episode(images, joint_pos, joint_cmd=None, fps=50.0):
    keys = list(images.keys())
    n_frames = min([images[k].shape[0] for k in keys] + [joint_pos.shape[0]])
    if joint_cmd is not None:
        n_frames = min(n_frames, joint_cmd.shape[0])

    interval = 1.0 / fps
    t_vals = np.arange(n_frames)
    num_joints = joint_pos.shape[1]
    assert num_joints == 14, f"Expected 14 joints, got {num_joints}"

    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(2, 1, height_ratios=[1, 1])
    ax_joint_left = fig.add_subplot(gs[0])
    ax_joint_right = fig.add_subplot(gs[1])

    lines_left_pos, lines_left_cmd = [], []
    lines_right_pos, lines_right_cmd = [], []

    for j in range(7):
        (lp,) = ax_joint_left.plot([], [], label=f'L_j{j}', linestyle='-')
        lines_left_pos.append(lp)
        if joint_cmd is not None:
            (lc,) = ax_joint_left.plot([], [], label=f'L_j{j}_cmd', linestyle='--')
            lines_left_cmd.append(lc)

        (rp,) = ax_joint_right.plot([], [], label=f'R_j{j}', linestyle='-')
        lines_right_pos.append(rp)
        if joint_cmd is not None:
            (rc,) = ax_joint_right.plot([], [], label=f'R_j{j}_cmd', linestyle='--')
            lines_right_cmd.append(rc)

    def setup_ax(ax, title, *data):
        ax.set_xlim(0, n_frames)
        vmin = min([d[:, :7].min() for d in data])
        vmax = max([d[:, :7].max() for d in data])
        ax.set_ylim(vmin - 0.1, vmax + 0.1)
        ax.set_title(title)
        ax.set_xlabel("Timestep")
        ax.set_ylabel("Value")
        ax.grid()
        ax.legend(loc='upper right')

    setup_ax(ax_joint_left, "Left Arm Joints", joint_pos, joint_cmd if joint_cmd is not None else joint_pos)
    setup_ax(ax_joint_right, "Right Arm Joints", joint_pos[:, 7:], joint_cmd[:, 7:] if joint_cmd is not None else joint_pos[:, 7:])

    for t in range(n_frames):
        show_images_cv2(images, t)

        for j in range(7):
            lines_left_pos[j].set_data(t_vals[:t+1], joint_pos[:t+1, j])
            lines_right_pos[j].set_data(t_vals[:t+1], joint_pos[:t+1, j+7])
            if joint_cmd is not None:
                lines_left_cmd[j].set_data(t_vals[:t+1], joint_cmd[:t+1, j])
                lines_right_cmd[j].set_data(t_vals[:t+1], joint_cmd[:t+1, j+7])

        ax_joint_left.relim(); ax_joint_left.autoscale_view()
        ax_joint_right.relim(); ax_joint_right.autoscale_view()

        plt.suptitle(f"Timestep {t}", fontsize=16)
        plt.pause(interval)

    cv2.destroyAllWindows()
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