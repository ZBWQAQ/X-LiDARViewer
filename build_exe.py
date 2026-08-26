# build_exe.py - Build X LiDARViewer as standalone EXE
# Usage: python build_exe.py

import PyInstaller.__main__
import os

project_dir = os.path.dirname(os.path.abspath(__file__))

PyInstaller.__main__.run([
    os.path.join(project_dir, 'lidar_viewer_app.py'),
    '--name=X LiDARViewer',
    '--onefile',
    '--windowed',
    f'--icon={os.path.join(project_dir, "icon.ico")}',
    f'--add-data={os.path.join(project_dir, "lidar_configs.json")};.',
    f'--add-data={os.path.join(project_dir, "icon.ico")};.',
    f'--add-data={os.path.join(project_dir, "generic_parser.py")};.',
    '--noconfirm',
    '--clean',
    f'--distpath={os.path.join(project_dir, "dist")}',
    f'--workpath={os.path.join(project_dir, "build")}',
    f'--specpath={project_dir}',
])
print('\n=== Build complete! ===')
print(f'Output: {os.path.join(project_dir, "dist", "X LiDARViewer.exe")}')
