"""Build and verify a portable Stage 2 SimPC package; never connects to hardware."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
STAGING = ROOT / 'build/simpc_stage2_dist'
RELEASE = ROOT / 'dist/QTG_Studio_SimPC_BankPID'


def main():
    for path in (STAGING, RELEASE):
        if not path.resolve().is_relative_to(ROOT):
            raise RuntimeError('Build path outside project')
    if RELEASE.exists():
        shutil.rmtree(RELEASE)
    if RELEASE.with_suffix('.zip').exists():
        RELEASE.with_suffix('.zip').unlink()
    if (RELEASE.parent / (RELEASE.name + '.zip.sha256')).exists():
        (RELEASE.parent / (RELEASE.name + '.zip.sha256')).unlink()
    subprocess.run([sys.executable, '-B', '-m', 'PyInstaller', '--noconfirm',
                    '--distpath', str(STAGING), '--workpath', str(ROOT/'build/simpc_stage2_work'),
                    str(ROOT/'QTG_Studio_SimPC.spec')], cwd=ROOT, check=True)
    package=STAGING/'QTG_Studio'
    env=os.environ.copy()
    env['PATH']=str(Path(os.environ.get('WINDIR','C:/Windows'))/'System32')
    for key in ('PYTHONPATH','PYTHONHOME','QT_PLUGIN_PATH','QML2_IMPORT_PATH'):
        env.pop(key,None)
    env['QT_QPA_PLATFORM']='offscreen'
    env['QTG_RESULTS_ROOT']=str(ROOT/'build/simpc_offline_results')
    temp=ROOT/'build/simpc_offline_temp'
    temp.mkdir(parents=True,exist_ok=True)
    env['TEMP']=env['TMP']=str(temp)
    check=package/'offline_check.json'
    env['QTG_DEPLOYMENT_CHECK_OUTPUT']=str(check)
    subprocess.run([str(package/'QTG_Studio.exe'),'--offline-gui-check'],cwd=package,
                   env=env,check=True,timeout=120,
                   creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    result=json.loads(check.read_text(encoding='utf-8'))
    if not result.get('ok') or len(result.get('checks',[])) < 7:
        raise RuntimeError(result)
    for name in ('python313.dll','vcruntime140.dll','vcruntime140_1.dll','msvcp140.dll'):
        if not list((package/'_internal').rglob(name)):
            raise FileNotFoundError('Missing bundled runtime: '+name)
    RELEASE.parent.mkdir(exist_ok=True)
    shutil.copytree(package,RELEASE)
    shutil.copy2(ROOT/'README_SimPC.md',RELEASE/'README_SimPC.md')

    # Copy Tcl and Tk library directories to root lib/ and tcl/ for native C fallback lookups
    tcl_src = None
    tk_src = None
    for cand in [RELEASE/'_internal'/'_tcl_data', RELEASE/'_internal'/'lib'/'tcl8.6', Path(sys.base_prefix)/'tcl'/'tcl8.6']:
        if cand.is_dir() and (cand/'init.tcl').is_file():
            tcl_src = cand
            break
    for cand in [RELEASE/'_internal'/'_tk_data', RELEASE/'_internal'/'lib'/'tk8.6', Path(sys.base_prefix)/'tcl'/'tk8.6']:
        if cand.is_dir() and (cand/'tk.tcl').is_file():
            tk_src = cand
            break
    if tcl_src:
        (RELEASE/'lib'/'tcl8.6').parent.mkdir(parents=True, exist_ok=True)
        if not (RELEASE/'lib'/'tcl8.6').exists():
            shutil.copytree(tcl_src, RELEASE/'lib'/'tcl8.6')
        (RELEASE/'tcl'/'tcl8.6').parent.mkdir(parents=True, exist_ok=True)
        if not (RELEASE/'tcl'/'tcl8.6').exists():
            shutil.copytree(tcl_src, RELEASE/'tcl'/'tcl8.6')
    if tk_src:
        (RELEASE/'lib'/'tk8.6').parent.mkdir(parents=True, exist_ok=True)
        if not (RELEASE/'lib'/'tk8.6').exists():
            shutil.copytree(tk_src, RELEASE/'lib'/'tk8.6')
        (RELEASE/'tcl'/'tk8.6').parent.mkdir(parents=True, exist_ok=True)
        if not (RELEASE/'tcl'/'tk8.6').exists():
            shutil.copytree(tk_src, RELEASE/'tcl'/'tk8.6')

    start_bat = (
        '@echo off\r\n'
        'cd /d "%~dp0"\r\n'
        'set "APP_DIR=%~dp0"\r\n'
        'if exist "%APP_DIR%_internal\\_tcl_data\\init.tcl" (\r\n'
        '    set "TCL_LIBRARY=%APP_DIR%_internal\\_tcl_data"\r\n'
        ') else if exist "%APP_DIR%lib\\tcl8.6\\init.tcl" (\r\n'
        '    set "TCL_LIBRARY=%APP_DIR%lib\\tcl8.6"\r\n'
        ') else if exist "%APP_DIR%_internal\\lib\\tcl8.6\\init.tcl" (\r\n'
        '    set "TCL_LIBRARY=%APP_DIR%_internal\\lib\\tcl8.6"\r\n'
        ')\r\n'
        'if exist "%APP_DIR%_internal\\_tk_data\\tk.tcl" (\r\n'
        '    set "TK_LIBRARY=%APP_DIR%_internal\\_tk_data"\r\n'
        ') else if exist "%APP_DIR%lib\\tk8.6\\tk.tcl" (\r\n'
        '    set "TK_LIBRARY=%APP_DIR%lib\\tk8.6"\r\n'
        ') else if exist "%APP_DIR%_internal\\lib\\tk8.6\\tk.tcl" (\r\n'
        '    set "TK_LIBRARY=%APP_DIR%_internal\\lib\\tk8.6"\r\n'
        ')\r\n'
        'start "" "%APP_DIR%QTG_Studio.exe"\r\n'
    )
    (RELEASE/'START_QTG.bat').write_text(start_bat, encoding='ascii')
    (RELEASE/'READ_ME_FIRST.txt').write_text(
        'QTG Studio - Windows x64 portable SimPC release (Multi-PID Tick System: Airspeed + Bank + Yaw)\n\n'
        '1. Extract the ENTIRE ZIP to a writable folder on the simulator PC.\n'
        '2. Keep QTG_Studio.exe and its _internal folder together.\n'
        '3. Start Prepar3D and CLS2Sim using your established simulator procedure.\n'
        '4. Run QTG_Studio.exe or START_QTG.bat.\n\n'
        'Python, pip, VS Code, Visual Studio and SDK build tools are not required.\n'
        'Existing compatible P3D/SimConnect and configured CLS2Sim/BRUNNER hardware remain required.\n'
        'No replacement simulator DLLs, bridges or hardware drivers need installation.\n'
        'Results default to Documents\\QTG_Test_Results. See README_SimPC.md to change this.\n\n'
        'Checkboxes for Airspeed PID, Bank PID, and Yaw PID allow individual or simultaneous control.\n'
        'Select any single PID, any combination of two, or all three to run together.\n'
        'Leaving all unchecked runs Recorded Replay.\n'
        'Yaw PID steers to the Target Heading (Magnetic) entered in the GUI and coordinates rudder/roll.\n'
        'Keep the existing simulator installation as a rollback option.\n'
        'Offline packaged checks passed; no live simulator/hardware validation was performed.\n'
        'Supervise first-run polarity, tracking, Stop/E-stop, disconnect and watchdog-release checks.\n',encoding='utf-8')
    tracked=['main.py','frontend/main.py','qtg_common_engine.py','backend/control.py','backend/bank_pid.py',
             'backend/bank_pid_runner.py','interfaces/bank_pid_transport.py',
             'reports/bank_pid_report.py','backend/yaw_pid.py','backend/yaw_pid_runner.py',
             'interfaces/yaw_pid_transport.py','reports/yaw_pid_report.py',
             'backend/multi_pid_runner.py','interfaces/multi_pid_transport.py','reports/multi_pid_report.py',
             'test_suites/normal_climb/procedure.py',
             'interfaces/brunner_base/src/brunner_interface.py',
             'interfaces/brunner_base/Super_Mushak_Logger_GUI.py',
             'tools/deployment_check.py','QTG_Studio_SimPC.spec']
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest={'release':'Bank & Yaw PID Stage 3 SimPC','python':sys.version,
              'live_hardware_tested':False,'offline_checks':result,
              'sources':{name:digest(ROOT/name) for name in tracked},
              'executable_sha256':digest(RELEASE/'QTG_Studio.exe')}
    (RELEASE/'build_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    archive=Path(shutil.make_archive(str(RELEASE),'zip',root_dir=RELEASE.parent,base_dir=RELEASE.name))
    if not archive.is_file():raise RuntimeError('ZIP not created')
    (RELEASE.parent/(archive.name+'.sha256')).write_text(digest(archive)+'  '+archive.name+'\n',encoding='ascii')
    print(json.dumps({'release':str(RELEASE),'zip':str(archive),
                      'zip_bytes':archive.stat().st_size,'offline_checks':result},indent=2))


if __name__=='__main__':main()
