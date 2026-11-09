"""Build the PointCloud2 layout used by the recorded MID360 bag."""
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_livox_pointcloud(points, stamp, frame_id, intensity=None, tag=None, line=None, timestamp=None):
    """Create x/y/z/intensity/tag/line/timestamp with a 26-byte point step."""
    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    count = len(xyz)
    dtype = np.dtype({
        "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 17, 18],
        "itemsize": 26,
    })
    packed = np.zeros(count, dtype=dtype)
    if count:
        packed["x"], packed["y"], packed["z"] = xyz.T
    for name, values in (("intensity", intensity), ("tag", tag), ("line", line)):
        if values is not None:
            values = np.asarray(values).reshape(-1)
            if len(values) == count:
                packed[name] = values
    if timestamp is None:
        timestamp = np.full(count, float(stamp.sec) * 1e9 + float(stamp.nanosec), dtype=np.float64)
    else:
        timestamp = np.asarray(timestamp, dtype=np.float64).reshape(-1)
        if len(timestamp) != count:
            timestamp = np.full(count, float(stamp.sec) * 1e9 + float(stamp.nanosec), dtype=np.float64)
    packed["timestamp"] = timestamp
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = count
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
        PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = msg.point_step * count
    msg.data = packed.tobytes()
    msg.is_dense = bool(np.isfinite(xyz).all())
    return msg
