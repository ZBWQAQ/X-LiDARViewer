# X LiDARViewer

![Version](https://img.shields.io/badge/version-V1.1.0-blue)
![Python](https://img.shields.io/badge/python-3.10+-green)
![License](https://img.shields.io/badge/license-MIT-orange)

通用激光雷达上位机，支持实时串口连接、点云可视化、DAT文件回放，可通过 JSON 配置文件适配不同型号雷达。

![preview](X-LiDARViewer.png)

---

## ✨ 功能特性

### 🔌 串口连接
- 自动检测可用串口
- 支持多种波特率（9600 ~ 921600）
- 实时连接/断开，状态指示

### 📊 点云可视化
- **极坐标视图** — 以雷达原点为中心的360°极坐标散点图
- **笛卡尔视图** — X-Y 平面直角坐标散点图
- **距离分布** — 距离区间统计直方图
- 三种视图实时刷新，支持可变距离范围

### 🎬 文件回放
- 支持 `.dat` 串口录制文件回放
- 暂停 / 恢复 / 停止
- 7 档变速：0.25x ~ Max
- 回放完成后可一键 Replay 重播

### 💾 点云导出
- **Save CSV** — 保存当前一圈点云（含角度、距离、可信度、XY坐标）
- **Save All** — 保存全部已采集数据
- **Auto Save** — 每圈自动保存到 `DATA/` 目录

### ⚙️ 通用配置
- 内置 3 款雷达配置（Camsense X1 / TFmini Plus / Custom）
- 支持 **Import Config** 导入外部 JSON 配置文件
- 实时切换雷达型号，无需重启
- 距离单位切换：mm / cm / m

---

## 🚀 快速开始

### 方式一：直接运行 EXE（推荐）

```
dist/X LiDARViewer.exe
```

无需安装 Python，双击即用。

### 方式二：Python 环境运行

```bash
# 安装依赖
pip install pyserial numpy matplotlib PyQt6

# 运行
python lidar_viewer_app.py
```

### 方式三：一键启动

双击 `run.bat`

---

## 📁 项目结构

```
X-LiDARViewer/
├── lidar_viewer_app.py      # 主程序（GUI + 业务逻辑）
├── generic_parser.py         # 通用解析器 + 配置管理器
├── lidar_configs.json        # 雷达配置文件（可编辑）
├── icon.ico                  # 应用图标
├── run.bat                   # 一键启动脚本
├── build_exe.py              # PyInstaller 打包脚本
├── X-LiDARViewer.png         # 项目预览图
├── DATA/                     # 点云数据目录
│   └── *.dat / *.csv
└── dist/
    └── X LiDARViewer.exe     # 独立可执行文件
```

---

## ⚙️ 配置文件说明

编辑 `lidar_configs.json` 即可添加新雷达型号：

```json
{
    "active_profile": "camsense_x1",
    "profiles": {
        "my_new_lidar": {
            "name": "My LiDAR",
            "description": "自定义雷达描述",
            "serial": {
                "baudrate": 115200,
                "bytesize": 8,
                "stopbits": 1,
                "parity": "N"
            },
            "protocol": {
                "header": [0x55, 0xAA, 0x03, 0x08],
                "frame_size": 36,
                "points_per_frame": 8,
                "endian": "little",
                "speed_offset": 4,
                "speed_divisor": 64.0,
                "speed_hz_divisor": 3840.0,
                "start_angle_offset": 6,
                "end_angle_offset": 32,
                "angle_divisor": 64.0,
                "angle_offset": -640.0,
                "points_offset": 8,
                "point_size": 3,
                "distance_bytes": 2,
                "quality_offset_in_point": 2,
                "crc_offset": 34
            },
            "scan": {
                "frames_per_scan": 52,
                "min_distance": 0,
                "max_distance": 5000,
                "quality_threshold": 1
            }
        }
    }
}
```

### 参数详解

| 参数 | 说明 |
|------|------|
| `header` | 帧头字节数组 |
| `frame_size` | 数据帧总长度（字节） |
| `points_per_frame` | 每帧包含的测量点数 |
| `speed_offset` | 转速字段在帧中的起始偏移 |
| `speed_divisor` | 转速原始值的除数（得到 RPM） |
| `start_angle_offset` | 起始角度字段偏移 |
| `end_angle_offset` | 结束角度字段偏移 |
| `angle_divisor` | 角度原始值的除数 |
| `angle_offset` | 角度偏移量 |
| `points_offset` | 点云数据起始偏移 |
| `point_size` | 每个点占用的字节数 |
| `quality_offset_in_point` | 点内可信度字段偏移 |
| `frames_per_scan` | 每圈扫描的帧数 |

---

## 📋 内置雷达配置

| 型号 | 帧头 | 帧大小 | 点数/帧 | 波特率 |
|------|------|--------|---------|--------|
| Camsense X1 | `55 AA 03 08` | 36B | 8 | 115200 |
| TFmini Plus | `59 59` | 9B | 1 | 115200 |
| Custom | `55 AA` | 32B | 8 | 115200 |

---

## 🔨 打包为 EXE

```bash
# 安装 PyInstaller
pip install pyinstaller

# 执行打包
python build_exe.py
```

生成的 EXE 位于 `dist/X LiDARViewer.exe`。

---

## 🖥️ 界面预览

| 功能区 | 说明 |
|--------|------|
| 标题栏 | 应用图标、名称、版本号、连接状态 |
| 配置栏 | 雷达型号选择、导入/重载配置 |
| 串口栏 | 端口扫描、波特率、连接/断开 |
| 回放栏 | 打开DAT、暂停/恢复、变速控制 |
| 显示栏 | 距离范围、单位切换、清除、保存 |
| 图表区 | 极坐标 / 笛卡尔 / 距离分布 三视图 |
| 数据区 | 实时统计、帧详情表格、原始HEX数据 |

---

## 📝 CSV 导出格式

```csv
# Camsense X1 LiDAR Point Cloud
# Date: 2026-08-24 23:00:00
# Frames: 52
# Speed: 290.0 RPM
# Valid Points: 150

index,angle_deg,distance_mm,quality,x_mm,y_mm
0,6.28,252,76,250.47,27.65
1,7.14,254,84,252.08,31.58
...
```

---

## 🛠️ 开发环境

- Python 3.10+
- PyQt6 6.6+
- matplotlib 3.11+
- numpy 2.5+
- pyserial 3.5+

---

## 📄 License

MIT License

---

## 🙏 致谢

- [camsense-X1](https://github.com/Vidicon/camsense-X1) — 协议逆向工程参考
- [Camsense X1 ROS Driver](https://github.com/Vidicon/camsense_driver)
- [Camsense X1 3D Model](https://github.com/Vidicon/camsense-X1)
