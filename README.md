# X LiDARViewer

![Version](https://img.shields.io/badge/version-V1.2.0-blue)
![Python](https://img.shields.io/badge/python-3.10+-green)
![License](https://img.shields.io/badge/license-MIT-orange)

通用激光雷达上位机，支持实时串口连接、点云可视化、DAT文件回放，可通过 JSON 配置文件适配不同型号雷达。V1.2.0 新增 TFmini Plus 单点激光测距传感器完整适配。

![preview](X-LiDARViewer.png)

---

## ✨ 功能特性

### 🔌 串口连接
- 自动检测可用串口，支持多种波特率（9600 ~ 921600）
- 根据配置文件自动选择对应协议解析器
- 实时连接/断开，状态指示

### 📊 点云可视化
- **极坐标视图** — 以雷达原点为中心的360°极坐标散点图
- **笛卡尔视图** — X-Y 平面直角坐标散点图
- **距离分布** — 距离区间统计直方图
- **距离时间线** — 距离随时间变化的实时折线图（通用，支持所有雷达型号）
- 四种视图实时刷新

### 🎯 Auto Range（自动/手动范围调节）
- **Auto Range** 模式：所有视图自动根据数据缩放
- **手动 Range** 模式：指定最大范围，支持 mm / cm / m 单位自动换算，上限 100m

### 🎬 文件回放
- 支持 `.dat` 串口录制文件回放
- **自动格式检测**：读取 DAT 文件头部，自动识别 TFmini Plus / 通用雷达格式
- **格式匹配校验**：加载 DAT 时自动比对当前配置，不匹配则提示错误
- 暂停 / 恢复 / 逐帧前进 / 逐帧后退 / 7档变速

### 💾 点云导出
- **Save DAT** — 保存当前扫描周期的帧数据
- **Save All** — 保存全部已采集数据
- **Auto Save** — 每圈自动保存到 `DATA/` 目录

### ⚙️ 通用配置
- 内置 3 款雷达配置（Camsense X1 / TFmini Plus / Custom）
- 支持 Import Config 导入外部 JSON 配置文件
- 实时切换雷达型号，无需重启
- TFmini Config 标签页根据配置文件自动显示/隐藏

### 📡 TFmini Plus 专属功能
- 专用协议解析器：9字节数据帧 + 配置响应帧完整解析
- Distance Timeline：实时距离折线图 + Dist / Strength / Temperature 统计
- 配置命令发送面板：快捷命令、帧率/波特率设置、输出模式切换、自定义HEX命令

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
├── lidar_viewer_app.py         # 主程序（GUI + 业务逻辑）
├── generic_parser.py           # 通用解析器 + 配置管理器
├── tfmini_plus_protocol.py     # TFmini Plus 协议解析 + 配置命令构建
├── lidar_configs.json          # 雷达配置文件（可编辑）
├── icon.ico                    # 应用图标
├── run.bat                     # 一键启动脚本
├── build_exe.py                # PyInstaller 打包脚本
├── X-LiDARViewer.png           # 项目预览图
├── DATA/                       # 点云数据目录
│   └── *.dat / *.csv
└── dist/
    └── X LiDARViewer.exe       # 独立可执行文件
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
| `type` | 协议类型（`tfmini_plus` 使用专用解析器） |
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

| 型号 | 帧头 | 帧大小 | 点数/帧 | 波特率 | 测距范围 |
|------|------|--------|---------|--------|----------|
| Camsense X1 | `55 AA 03 08` | 36B | 8 | 115200 | 0~5m |
| TFmini Plus | `59 59` | 9B | 1 | 115200 | 10cm~12m |
| Custom | `55 AA` | 32B | 8 | 115200 | 自定义 |

---

## 📡 TFmini Plus 支持的配置命令

| 功能 | HEX 命令 | 说明 |
|------|----------|------|
| 获取固件版本 | `5A 04 01 5F` | 返回固件版本号 |
| 系统复位 | `5A 04 02 60` | 复位后1s重启 |
| 输出帧率 | `5A 06 03 LL HH SU` | 设置1~1000Hz |
| 单次触发 | `5A 04 04 62` | 单次触发测距 |
| 输出模式(cm) | `5A 05 05 01 65` | 标准9字节，单位cm |
| 输出模式(m) | `5A 05 05 02 66` | 字符串格式，单位m |
| 输出模式(mm) | `5A 05 05 06 6A` | 标准9字节，单位mm |
| 波特率设置 | `5A 08 06 H1 H2 H3 H4 SU` | 设置通信波特率 |
| 使能输出 | `5A 05 07 01 67` | 开启数据持续输出 |
| 关闭输出 | `5A 05 07 00 66` | 关闭数据持续输出 |
| 恢复出厂 | `5A 04 10 6E` | 恢复出厂默认设置 |
| 保存设置 | `5A 04 11 6F` | 保存当前配置（必须!） |

---

## 📝 更新日志

### V1.2.0 (2026-08-26)
- ✅ 新增 TFmini Plus 单点激光测距传感器完整适配
- ✅ 新增 Distance Timeline 实时距离时间线视图（通用，支持所有雷达）
- ✅ 新增 TFmini Config 配置命令发送面板（根据配置自动显隐）
- ✅ 新增 Auto Range 自动/手动范围调节（所有视图统一，上限100m）
- ✅ 新增 DAT 文件格式自动检测与配置匹配校验
- ✅ 新增 TFmini Plus DAT 文件回放支持
- ✅ 修复 TFmini Plus 距离单位换算（cm模式 ×10 → mm 统一存储）
- ✅ 修复配置帧 Len 字段计算（包含 Checksum 字节）

### V1.1.0
- 初始版本：Camsense X1 支持、三视图、DAT回放与导出

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
| 回放栏 | 打开DAT、暂停/恢复、变速控制、逐帧步进 |
| 显示栏 | Auto Range 开关、距离范围、单位切换、清除、保存 |
| 图表区 | 极坐标 / 笛卡尔 / 距离分布 / 距离时间线 四视图 |
| TFmini Config | 配置命令面板（仅 TFmini Plus 模式可见） |
| 数据区 | 实时统计、帧详情表格、原始HEX数据 |

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
