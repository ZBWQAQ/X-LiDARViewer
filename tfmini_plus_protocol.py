"""
TFmini Plus 激光测距传感器通信协议解析模块

协议规范：
- 数据帧大小：9字节，帧头 0x59 0x59
- 配置帧帧头：0x5A，变长
- 配置帧格式: Head(0x5A) + Len + ID + Payload + Checksum
- Checksum = 前面所有字节累加和的低8位
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Dict


@dataclass
class TFminiPlusDataPoint:
    """单次测量数据点"""
    distance: int = 0
    strength: int = 0
    temperature: int = 0
    timestamp: float = 0.0

    @property
    def distance_cm(self) -> float:
        return self.distance / 10.0

    @property
    def distance_m(self) -> float:
        return self.distance / 1000.0

    @property
    def temperature_c(self) -> float:
        return self.temperature / 8.0 - 256.0


class TFminiPlusParser:
    """TFmini Plus 数据帧解析器，支持流式解析"""

    DATA_FRAME_SIZE = 9
    CONFIG_RESPONSE_SIZE_MIN = 4

    def __init__(self, output_mode='cm'):
        """
        output_mode: 'cm' (默认9字节cm), 'mm' (标准9字节mm), 'string_m' (字符串m)
        距离统一存储为 mm
        """
        self._buffer = bytearray()
        self._last_data_point = None
        self._data_points: List[TFminiPlusDataPoint] = []
        self._point_index = 0
        self._last_config_response = None
        self._output_mode = output_mode

    def set_output_mode(self, mode: str):
        self._output_mode = mode

    def reset(self):
        self._buffer.clear()
        self._data_points.clear()
        self._point_index = 0

    def feed_data(self, data: bytes):
        """喂入原始串口数据，返回 (data_points, config_responses)"""
        self._buffer.extend(data)
        data_points = []
        config_responses = []

        while len(self._buffer) >= self.DATA_FRAME_SIZE:
            dp = self._try_parse_data_frame()
            if dp is not None:
                dp.timestamp = self._point_index
                self._point_index += 1
                self._data_points.append(dp)
                self._last_data_point = dp
                data_points.append(dp)
                continue
            cr = self._try_parse_config_response()
            if cr is not None:
                config_responses.append(cr)
                self._last_config_response = cr
                continue
            self._buffer.pop(0)

        return data_points, config_responses

    def _try_parse_data_frame(self):
        if self._buffer[0] != 0x59 or self._buffer[1] != 0x59:
            return None
        data = bytes(self._buffer[:self.DATA_FRAME_SIZE])
        checksum = sum(data[:8]) & 0xFF
        if checksum != data[8]:
            return None
        try:
            raw_dist = struct.unpack_from('<H', data, 2)[0]
            strength = struct.unpack_from('<H', data, 4)[0]
            temp = struct.unpack_from('<H', data, 6)[0]
            # 统一转换为 mm
            if self._output_mode == 'mm':
                dist_mm = raw_dist
            elif self._output_mode == 'string_m':
                dist_mm = int(raw_dist * 1000)
            else:  # 'cm' (默认)
                dist_mm = raw_dist * 10
            self._buffer = self._buffer[self.DATA_FRAME_SIZE:]
            return TFminiPlusDataPoint(distance=dist_mm, strength=strength, temperature=temp)
        except struct.error:
            return None

    def _try_parse_config_response(self):
        if self._buffer[0] != 0x5A:
            return None
        if len(self._buffer) < 3:
            return None
        resp_len = self._buffer[1]
        if resp_len < self.CONFIG_RESPONSE_SIZE_MIN:
            return None
        if len(self._buffer) < resp_len:
            return None
        resp = bytes(self._buffer[:resp_len])
        cksum = sum(resp[:-1]) & 0xFF
        if cksum != resp[-1]:
            return None
        self._buffer = self._buffer[resp_len:]
        return resp

    @property
    def last_data_point(self):
        return self._last_data_point

    @property
    def last_config_response(self):
        return self._last_config_response

    @property
    def point_count(self) -> int:
        return len(self._data_points)

    def get_recent_points(self, count: int):
        return self._data_points[-count:]

    def clear_points(self):
        self._data_points.clear()
        self._point_index = 0


class TFminiPlusCommandBuilder:
    """TFmini Plus 配置命令构建器"""

    @staticmethod
    def _build_frame(payload: bytes) -> bytes:
        """构建配置帧: Head(1) + Len(1) + Payload + Checksum(1)"""
        length = 1 + 1 + len(payload) + 1  # Head + Len + Payload + Checksum
        frame = bytes([0x5A, length]) + payload
        checksum = sum(frame) & 0xFF
        frame += bytes([checksum])
        return frame

    @staticmethod
    def get_firmware_version() -> bytes:
        return bytes([0x5A, 0x04, 0x01, 0x5F])

    @staticmethod
    def system_reset() -> bytes:
        return bytes([0x5A, 0x04, 0x02, 0x60])

    @staticmethod
    def set_output_fps(fps: int) -> bytes:
        if fps < 1 or fps > 1000:
            raise ValueError("FPS must be 1-1000")
        payload = bytes([0x03, fps & 0xFF, (fps >> 8) & 0xFF])
        return TFminiPlusCommandBuilder._build_frame(payload)

    @staticmethod
    def single_trigger() -> bytes:
        return bytes([0x5A, 0x04, 0x04, 0x62])

    @staticmethod
    def set_output_mode(mode: int) -> bytes:
        """
        设置输出模式 (ID=0x05):
        mode=1: 标准9字节(cm)  5A 05 05 01 65
        mode=2: 字符串格式(m)  5A 05 05 02 66
        mode=6: 标准9字节(mm)  5A 05 05 06 6A
        """
        su = (0x5A + 0x05 + 0x05 + mode) & 0xFF
        return bytes([0x5A, 0x05, 0x05, mode, su])

    @staticmethod
    def set_baudrate(baudrate: int) -> bytes:
        baud_bytes = baudrate.to_bytes(4, byteorder='little')
        payload = bytes([0x06]) + baud_bytes
        return TFminiPlusCommandBuilder._build_frame(payload)

    @staticmethod
    def set_output_enable(enable: bool) -> bytes:
        val = 0x01 if enable else 0x00
        su = (0x5A + 0x05 + 0x07 + val) & 0xFF
        return bytes([0x5A, 0x05, 0x07, val, su])

    @staticmethod
    def set_interface(mode: int) -> bytes:
        su = (0x5A + 0x05 + 0x0A + mode) & 0xFF
        return bytes([0x5A, 0x05, 0x0A, mode, su])

    @staticmethod
    def set_i2c_address(addr: int) -> bytes:
        su = (0x5A + 0x05 + 0x0B + addr) & 0xFF
        return bytes([0x5A, 0x05, 0x0B, addr, su])

    @staticmethod
    def set_strength_threshold(threshold: int) -> bytes:
        xx = threshold // 10
        payload = bytes([0x22, xx, 0x00, 0x00, 0x00])
        return TFminiPlusCommandBuilder._build_frame(payload)

    @staticmethod
    def set_low_power_mode(mode: int) -> bytes:
        payload = bytes([0x35, mode & 0xFF, 0x00])
        return TFminiPlusCommandBuilder._build_frame(payload)

    @staticmethod
    def factory_reset() -> bytes:
        return bytes([0x5A, 0x04, 0x10, 0x6E])

    @staticmethod
    def save_settings() -> bytes:
        return bytes([0x5A, 0x04, 0x11, 0x6F])

    @staticmethod
    def set_io_mode(mode: int, dl: int = 0, dh: int = 0,
                    zone_l: int = 0, zone_h: int = 0) -> bytes:
        payload = bytes([0x3B, mode & 0xFF, dl & 0xFF, dh & 0xFF,
                         zone_l & 0xFF, zone_h & 0xFF])
        return TFminiPlusCommandBuilder._build_frame(payload)


class TFminiPlusResponseParser:
    """解析TFmini Plus配置命令的响应"""

    @staticmethod
    def parse_response(data: bytes) -> Dict:
        if len(data) < 4 or data[0] != 0x5A:
            return {'type': 'invalid', 'raw': data.hex()}

        frame_id = data[2]
        result = {'type': 'config_response', 'id': frame_id, 'raw': data.hex()}

        if frame_id == 0x01:
            if len(data) >= 7:
                result['firmware'] = f"V{data[3]}.{data[4]}.{data[5]}"
            result['description'] = '固件版本'

        elif frame_id == 0x05:
            if len(data) >= 5:
                sub_id = data[3]
                success = data[4]
                result['sub_id'] = sub_id
                result['success'] = (success == 0x01)
                result['description'] = '配置成功' if result['success'] else '配置失败'

        elif frame_id == 0x06:
            if len(data) >= 6:
                result['fps'] = data[3] | (data[4] << 8)
            result['description'] = '输出帧率'

        elif frame_id == 0x08:
            if len(data) >= 8:
                result['baudrate'] = int.from_bytes(data[3:7], byteorder='little')
            result['description'] = '波特率'

        else:
            result['description'] = f'ID=0x{frame_id:02X}'

        return result


TFMINI_PLUS_PRESET_COMMANDS = {
    'get_firmware': {
        'name': '获取固件版本',
        'description': '获取TFmini Plus固件版本号',
        'command': TFminiPlusCommandBuilder.get_firmware_version(),
    },
    'system_reset': {
        'name': '系统复位',
        'description': '系统复位(1s重启)',
        'command': TFminiPlusCommandBuilder.system_reset(),
    },
    'factory_reset': {
        'name': '恢复出厂设置',
        'description': '恢复出厂默认设置',
        'command': TFminiPlusCommandBuilder.factory_reset(),
    },
    'save_settings': {
        'name': '保存设置',
        'description': '保存当前配置(必须!)',
        'command': TFminiPlusCommandBuilder.save_settings(),
    },
    'single_trigger': {
        'name': '单次触发',
        'description': '单次触发测距输出',
        'command': TFminiPlusCommandBuilder.single_trigger(),
    },
    'output_enable': {
        'name': '使能数据输出',
        'description': '开启数据持续输出',
        'command': TFminiPlusCommandBuilder.set_output_enable(True),
    },
    'output_disable': {
        'name': '关闭数据输出',
        'description': '关闭数据持续输出',
        'command': TFminiPlusCommandBuilder.set_output_enable(False),
    },
    'mode_cm': {
        'name': '输出模式(cm)',
        'description': '标准9字节，距离单位cm',
        'command': TFminiPlusCommandBuilder.set_output_mode(1),
    },
    'mode_string_m': {
        'name': '输出模式(字符串m)',
        'description': '字符串格式，距离单位m',
        'command': TFminiPlusCommandBuilder.set_output_mode(2),
    },
    'mode_mm': {
        'name': '输出模式(mm)',
        'description': '标准9字节，距离单位mm',
        'command': TFminiPlusCommandBuilder.set_output_mode(6),
    },
}

FPS_PRESETS = [10, 50, 100, 200, 500, 1000]
BAUDRATE_PRESETS = [9600, 14400, 19200, 56000, 115200, 460800, 921600]
