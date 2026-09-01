# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/lidar_viewer_app.py'],
    pathex=[],
    binaries=[],
    datas=[('D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/lidar_configs.json', '.'), ('D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/icon.ico', '.'), ('D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/generic_parser.py', '.'), ('D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/tfmini_plus_protocol.py', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='X LiDARViewer',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['D:/02-Agent/Reasonix-DeepSeekV4-Workstation/04-LiDAR/icon.ico'],
)
