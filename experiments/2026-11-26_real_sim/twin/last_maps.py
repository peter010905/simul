"""Save each run's last /map (from its rosbag, else the last policy snapshot) as npz (read-only)."""
import glob, os, sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
for run in sys.argv[2:]:
    name = os.path.basename(run); grid = None
    for bag in sorted(glob.glob(run + "/rosbag/*.mcap")):
        try:
            r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
            r.set_filter(__import__("rosbag2_py").StorageFilter(topics=["/map"]))
            while r.has_next():
                try: _, data, _ = r.read_next()
                except Exception: break
                grid = deserialize_message(data, OccupancyGrid)
        except Exception as e:
            print(name, "bag error", e)
    if grid is not None:
        i = grid.info
        np.savez_compressed(f"{out}/{name}.npz", data=np.array(grid.data, np.int8).reshape(i.height, i.width),
                            resolution=i.resolution, origin=np.array([i.origin.position.x, i.origin.position.y]))
        print(name, "bag map", i.width, i.height, round(i.resolution, 3))
        continue
    snaps = sorted(glob.glob(run + "/artifacts/policy_decisions/*.npz"))
    if snaps:
        z = np.load(snaps[-1], allow_pickle=True)
        np.savez_compressed(f"{out}/{name}.npz", data=z["data"], resolution=z["resolution"], origin=z["origin"])
        print(name, "snapshot map", z["data"].shape)
    else:
        print(name, "no map")
