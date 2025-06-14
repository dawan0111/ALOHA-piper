import rclpy
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List
from tqdm import tqdm

from act_episode_convert.pipelines import resample_pipeline, decompress_compressed_images_pipeline, save_to_hdf5_pipeline
from act_episode_convert.converts import EpisodeConvert, Compose  # convert_module.py 에 정의된 클래스들


def load_config():
    config_path = Path(get_package_share_directory('act_bringup')) / 'config' / 'episode_record.yaml'
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def list_episode_dirs(bag_root: Path) -> List[Path]:
    episodes = sorted([p for p in bag_root.iterdir() if p.is_dir()])
    print(f"\n[INFO] Found {len(episodes)} episode(s):")
    for i, ep in enumerate(episodes):
        print(f"  [{i}] {ep.name}")
    return episodes


def run_multithread_convert(episodes: List[Path], thread_count: int):
    def worker(ep_path):
        compose = Compose([
            resample_pipeline(0.02, 100),
            decompress_compressed_images_pipeline(),
            save_to_hdf5_pipeline(f"/home/airo/convert/{ep_path.name}.hdf5")
        ])
        convert = EpisodeConvert(compose)
        print(f"[START] {ep_path.name}")
        result = convert.run(str(ep_path))
        print(f"[DONE ] {ep_path.name}")
        return result

    with ThreadPoolExecutor(max_workers=thread_count) as executor:
        futures = {executor.submit(worker, ep): ep for ep in episodes}
        for f in tqdm(as_completed(futures), total=len(futures), desc="Converting Episodes"):
            ep = futures[f]
            try:
                f.result()
            except Exception as e:
                print(f"[ERROR] Failed to convert {ep.name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    config = load_config()

    bag_root = Path(config.get('record_path', ''))
    if not bag_root.exists():
        print(f"[ERROR] Invalid rosbag path: {bag_root}")
        return

    episodes = list_episode_dirs(bag_root)
    if not episodes:
        print("[WARN] No episodes found.")
        return

    choice = input("\nEnter episode index to convert (or 'all'): ").strip()
    if choice == 'all':
        selected = episodes
    else:
        try:
            idx = int(choice)
            selected = [episodes[idx]]
        except (ValueError, IndexError):
            print("[ERROR] Invalid input.")
            return

    try:
        thread_count = int(input("Enter number of threads: ").strip())
    except ValueError:
        print("[ERROR] Invalid thread count.")
        return

    print(f"\n[INFO] Running conversion on {len(selected)} episode(s) using {thread_count} threads...\n")
    run_multithread_convert(selected, thread_count)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
