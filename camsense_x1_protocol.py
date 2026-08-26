"""
Camsense X1 激光雷达通信协议解析模块

协议规范：
- 帧大小：36字节
- 帧头：0x55 0xAA 0x03 0x08
- 转速(Hz) = speed_raw / 3840.0
- 角度(°) = angle_raw / 64.0 - 640.0
"""

import struct
import math
from dataclasses import dataclass, field
from typing import List, Optional

FRAME_HEADER = bytes([0x55, 0xAA, 0x03, 0x08])
FRAME_SIZE = 36
POINTS_PER_FRAME = 8


@dataclass
class LidarPoint:
    """单个测量点"""
    angle: float
    distance: int
    quality: int
    is_valid: bool = True

    @property
    def x(self) -> float:
        return self.distance * math.cos(math.radians(self.angle))

    @property
    def y(self) -> float:
        return self.distance * math.sin(math.radians(self.angle))


@dataclass
class LidarFrame:
    """单帧数据"""
    speed_rpm: float
    speed_hz: float
    start_angle: float
    end_angle: float
    points: List[LidarPoint] = field(default_factory=list)
    crc: int = 0
    raw_data: bytes = b''

    @property
    def valid_points(self) -> List[LidarPoint]:
        return [p for p in self.points if p.is_valid]

    @property
    def angle_span(self) -> float:
        if self.end_angle >= self.start_angle:
            return self.end_angle - self.start_angle
        return (self.end_angle + 360.0) - self.start_angle


@dataclass
class LidarScan:
    """完整一圈扫描"""
    points: List[LidarPoint] = field(default_factory=list)
    frame_count: int = 0
    avg_speed_rpm: float = 0.0

    @property
    def valid_points(self) -> List[LidarPoint]:
        return [p for p in self.points if p.is_valid and p.distance > 0]

    @property
    def min_distance(self) -> float:
        d = [p.distance for p in self.valid_points]
        return min(d) if d else 0.0

    @property
    def max_distance(self) -> float:
        d = [p.distance for p in self.valid_points]
        return max(d) if d else 0.0

    @property
    def avg_distance(self) -> float:
        d = [p.distance for p in self.valid_points]
        return sum(d) / len(d) if d else 0.0


class CamsenseX1Parser:
    """Camsense X1 数据帧解析器，支持流式解析"""

    def __init__(self):
        self._buffer = bytearray()

    def reset(self):
        self._buffer.clear()

    def feed_data(self, data: bytes) -> List[LidarFrame]:
        """喂入原始数据，返回解析出的完整帧列表"""
        self._buffer.extend(data)
        frames = []

        while True:
            header_pos = -1
            for i in range(len(self._buffer) - 3):
                if (self._buffer[i] == 0x55 and self._buffer[i + 1] == 0xAA
                        and self._buffer[i + 2] == 0x03 and self._buffer[i + 3] == 0x08):
                    header_pos = i
                    break

            if header_pos == -1:
                if len(self._buffer) > 3:
                    self._buffer = self._buffer[-3:]
                break

            if header_pos > 0:
                self._buffer = self._buffer[header_pos:]

            if len(self._buffer) < FRAME_SIZE:
                break

            frame_data = bytes(self._buffer[:FRAME_SIZE])
            frame = self._parse_frame(frame_data)
            if frame is not None:
                frames.append(frame)
            self._buffer = self._buffer[FRAME_SIZE:]

        return frames

    def _parse_frame(self, data: bytes) -> Optional[LidarFrame]:
        if len(data) != FRAME_SIZE or data[0:4] != FRAME_HEADER:
            return None
        try:
            speed_raw = struct.unpack_from('<H', data, 4)[0]
            start_angle_raw = struct.unpack_from('<H', data, 6)[0]
            start_angle = start_angle_raw / 64.0 - 640.0

            points = []
            for i in range(POINTS_PER_FRAME):
                base_idx = 8 + i * 3
                distance = struct.unpack_from('<H', data, base_idx)[0]
                quality = data[base_idx + 2]
                points.append(LidarPoint(
                    angle=0.0, distance=distance, quality=quality,
                    is_valid=(quality > 0 and distance > 0)
                ))

            end_angle_raw = struct.unpack_from('<H', data, 32)[0]
            end_angle = end_angle_raw / 64.0 - 640.0
            actual_end = end_angle if end_angle >= start_angle else end_angle + 360.0
            step = (actual_end - start_angle) / max(POINTS_PER_FRAME - 1, 1)
            for i, point in enumerate(points):
                point.angle = (start_angle + step * i) % 360.0

            crc = struct.unpack_from('<H', data, 34)[0]
            return LidarFrame(
                speed_rpm=speed_raw / 64.0, speed_hz=speed_raw / 3840.0,
                start_angle=start_angle % 360.0, end_angle=end_angle % 360.0,
                points=points, crc=crc, raw_data=data
            )
        except (struct.error, IndexError):
            return None
