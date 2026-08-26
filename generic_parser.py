"""通用激光雷达帧解析器 - 根据 JSON 配置解析不同型号雷达数据"""

import json, os, struct, math
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


@dataclass
class LidarPoint:
    angle: float
    distance: int
    quality: int
    is_valid: bool = True

    @property
    def x(self):
        return self.distance * math.cos(math.radians(self.angle))

    @property
    def y(self):
        return self.distance * math.sin(math.radians(self.angle))


@dataclass
class LidarFrame:
    speed_rpm: float
    speed_hz: float
    start_angle: float
    end_angle: float
    points: List[LidarPoint] = field(default_factory=list)
    crc: int = 0
    raw_data: bytes = b''

    @property
    def valid_points(self):
        return [p for p in self.points if p.is_valid]


@dataclass
class LidarScan:
    points: List[LidarPoint] = field(default_factory=list)
    frame_count: int = 0
    avg_speed_rpm: float = 0.0

    @property
    def valid_points(self):
        return [p for p in self.points if p.is_valid and p.distance > 0]

    @property
    def min_distance(self):
        d = [p.distance for p in self.valid_points]
        return min(d) if d else 0.0

    @property
    def max_distance(self):
        d = [p.distance for p in self.valid_points]
        return max(d) if d else 0.0

    @property
    def avg_distance(self):
        d = [p.distance for p in self.valid_points]
        return sum(d) / len(d) if d else 0.0


class ConfigManager:
    """配置文件管理器"""

    def __init__(self, config_path=None):
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 'lidar_configs.json')
        self.config_path = config_path
        self._data = {}
        self.load()

    def load(self):
        with open(self.config_path, 'r', encoding='utf-8') as f:
            self._data = json.load(f)

    def save(self):
        with open(self.config_path, 'w', encoding='utf-8') as f:
            json.dump(self._data, f, indent=4, ensure_ascii=False)

    @property
    def active_profile_name(self):
        return self._data.get('active_profile', '')

    @active_profile_name.setter
    def active_profile_name(self, name):
        self._data['active_profile'] = name
        self.save()

    @property
    def active_profile(self):
        return self._data.get('profiles', {}).get(self.active_profile_name, {})

    @property
    def profile_names(self):
        return list(self._data.get('profiles', {}).keys())

    def get_profile(self, name):
        return self._data.get('profiles', {}).get(name, {})

class GenericLidarParser:
    """通用激光雷达数据帧解析器 - 根据配置动态解析"""

    def __init__(self, config: Dict):
        self._buffer = bytearray()
        self.update_config(config)

    def update_config(self, config: Dict):
        self._cfg = config
        self._header = bytes(config.get('header', []))
        self._frame_size = config.get('frame_size', 36)
        self._points_per_frame = config.get('points_per_frame', 8)
        endian = '<' if config.get('endian', 'little') == 'little' else '>'
        self._h = endian + 'H'
        self._speed_off = config.get('speed_offset', 4)
        self._start_angle_off = config.get('start_angle_offset', 6)
        self._end_angle_off = config.get('end_angle_offset', 32)
        self._points_off = config.get('points_offset', 8)
        self._point_size = config.get('point_size', 3)
        self._qual_off_in_point = config.get('quality_offset_in_point', 2)
        self._crc_off = config.get('crc_offset', 34)
        self._dist_mult = config.get('distance_multiplier', 1)
        self._speed_div = config.get('speed_divisor', 64.0)
        self._speed_hz_div = config.get('speed_hz_divisor', 3840.0)
        self._angle_div = config.get('angle_divisor', 64.0)
        self._angle_off_val = config.get('angle_offset', 0.0)
        self._buffer.clear()

    def reset(self):
        self._buffer.clear()

    def feed_data(self, data: bytes) -> List[LidarFrame]:
        self._buffer.extend(data)
        frames = []
        hl = len(self._header)
        while True:
            pos = -1
            limit = max(len(self._buffer) - hl + 1, 0)
            for i in range(limit):
                if self._buffer[i:i + hl] == self._header:
                    pos = i
                    break
            if pos == -1:
                if len(self._buffer) >= hl:
                    self._buffer = self._buffer[-(hl - 1):]
                break
            if pos > 0:
                self._buffer = self._buffer[pos:]
            if len(self._buffer) < self._frame_size:
                break
            frame = self._parse_frame(bytes(self._buffer[:self._frame_size]))
            if frame:
                frames.append(frame)
            self._buffer = self._buffer[self._frame_size:]
        return frames

    def _parse_frame(self, data: bytes) -> Optional[LidarFrame]:
        if len(data) != self._frame_size or data[:len(self._header)] != self._header:
            return None
        try:
            sr = struct.unpack_from(self._h, data, self._speed_off)[0]
            sar = struct.unpack_from(self._h, data, self._start_angle_off)[0]
            sa = sar / self._angle_div + self._angle_off_val

            pts = []
            for i in range(self._points_per_frame):
                base = self._points_off + i * self._point_size
                if base + self._point_size > len(data):
                    break
                d = struct.unpack_from(self._h, data, base)[0]
                d = int(d * self._dist_mult)
                q = data[base + self._qual_off_in_point] if base + self._qual_off_in_point < len(data) else 0
                pts.append(LidarPoint(angle=0.0, distance=d, quality=q,
                                      is_valid=(q > 0 and d > 0)))

            ear = struct.unpack_from(self._h, data, self._end_angle_off)[0]
            ea = ear / self._angle_div + self._angle_off_val
            ae = ea if ea >= sa else ea + 360.0
            n = max(len(pts) - 1, 1)
            step = (ae - sa) / n
            for i, p in enumerate(pts):
                p.angle = (sa + step * i) % 360.0

            crc = struct.unpack_from(self._h, data, self._crc_off)[0] if self._crc_off + 2 <= len(data) else 0
            return LidarFrame(
                speed_rpm=sr / self._speed_div, speed_hz=sr / self._speed_hz_div,
                start_angle=sa % 360.0, end_angle=ea % 360.0,
                points=pts, crc=crc, raw_data=data)
        except (struct.error, IndexError):
            return None


    @property
    def active_profile_name(self):
        return self._data.get('active_profile', '')

    @active_profile_name.setter
    def active_profile_name(self, name):
        self._data['active_profile'] = name
        self.save()

    @property
    def active_profile(self):
        return self._data.get('profiles', {}).get(self.active_profile_name, {})

    @property
    def profile_names(self):
        return list(self._data.get('profiles', {}).keys())

    def get_profile(self, name):
        return self._data.get('profiles', {}).get(name, {})
