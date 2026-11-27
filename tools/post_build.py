import shutil
from pathlib import Path

dist = Path('dist/QTG_Studio')

bat_path = dist / 'Launch_QTG_Studio.bat'
with open(bat_path, 'w', encoding='utf-8') as f:
    f.write('@echo off\ncd /d %~dp0\nstart "" "%~dp0QTG_Studio.exe"\n')

settings_path = dist / 'qtg_settings.json'
with open(settings_path, 'w', encoding='utf-8') as f:
    f.write('{\n  "results_directory": "D:/QTG_Test_Results",\n  "live_hardware": true,\n  "strict_io": true\n}\n')

dll_sources = [
    Path('p3d_bridge.dll'),
    Path('interfaces/brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll'),
    Path('bridge/bin/qtg_p3d_bridge.dll'),
    Path('bridge/bin/qtg_simconnect.dll'),
]
for src in dll_sources:
    if src.exists():
        shutil.copy2(src, dist / src.name)
        print(f'Copied {src.name} to {dist}')

internal = dist / '_internal'
if internal.exists():
    for name in ['backend', 'interfaces', 'reports']:
        src = Path(name)
        dst = internal / name
        if src.exists():
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
            print(f'Synced {name} to {internal}')

print('Dist setup complete. Files:', [f.name for f in dist.iterdir() if f.is_file()])
