"""Select bounded, exposure-aligned sensor messages without changing timestamps."""
from collections import deque
import math


def seconds(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9


class SensorHistory:
    def __init__(self):
        self.scans=deque(maxlen=64)
        self.depths=deque(maxlen=8)
        self.selections=deque(maxlen=256)

    def scan(self,message):
        if math.isfinite(seconds(message.header)):self.scans.append(message)

    def depth(self,message):
        if math.isfinite(seconds(message.header)):self.depths.append(message)

    def select(self,image):
        stamp=seconds(image.header)
        # Older laser exposures already have exact-time TF when the RGB TF is
        # ready. Keep the original 250 ms maximum skew and use full TF below.
        scans=[m for m in self.scans if 0<=stamp-seconds(m.header)<=.25+1e-9]
        scan=max(scans,key=lambda m:seconds(m.header),default=None)
        # Only registered, matching-size depth can support this RGB ray.
        depths=[m for m in self.depths if abs(stamp-seconds(m.header))<.15
                and (m.width,m.height)==(image.width,image.height)
                and m.header.frame_id.lstrip('/')==image.header.frame_id.lstrip('/')
                and m.encoding in ('16UC1','32FC1')]
        depth=min(depths,key=lambda m:abs(stamp-seconds(m.header)),default=None)
        self.selections.append((stamp-seconds(scan.header) if scan is not None else None,
                                abs(stamp-seconds(depth.header)) if depth is not None else None))
        return scan,depth

    def report(self):
        return dict(scans_buffered=len(self.scans),depths_buffered=len(self.depths),
                    latest_scan_skew_s=self.selections[-1][0] if self.selections else None,
                    latest_depth_skew_s=self.selections[-1][1] if self.selections else None)
