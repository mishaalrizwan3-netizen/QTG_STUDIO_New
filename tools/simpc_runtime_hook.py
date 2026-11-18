"""The demonstration deployment always starts with Universal Initialization off and robust Tcl/Tk paths."""
import os
import sys

for name in ('QTG_UNIVERSAL_ENGINE_ACCELERATION', 'QTG_UNIVERSAL_ENGINE_DECELERATION',
             'QTG_UNIVERSAL_INITIALIZATION_ONLY', 'QTG_UNIVERSAL_MANUAL_PLANS',
             'QTG_UNIVERSAL_AUTOMATIC_PLANS'):
    os.environ.pop(name, None)

# Force-resolve TCL_LIBRARY and TK_LIBRARY across all possible deployment locations
_roots = []
if hasattr(sys, '_MEIPASS'):
    _roots.append(sys._MEIPASS)
_exe_dir = os.path.dirname(sys.executable)
_roots.extend([
    _exe_dir,
    os.path.join(_exe_dir, '_internal'),
])

for _root in _roots:
    for _sub in ['_tcl_data', 'lib/tcl8.6', 'tcl/tcl8.6', 'tcl8.6']:
        _cand = os.path.normpath(os.path.join(_root, _sub))
        if os.path.isfile(os.path.join(_cand, 'init.tcl')):
            os.environ['TCL_LIBRARY'] = _cand
            try:
                import ctypes
                ctypes.windll.kernel32.SetEnvironmentVariableW('TCL_LIBRARY', _cand)
            except Exception:
                pass
            break
    if 'TCL_LIBRARY' in os.environ:
        break

for _root in _roots:
    for _sub in ['_tk_data', 'lib/tk8.6', 'tcl/tk8.6', 'tk8.6']:
        _cand = os.path.normpath(os.path.join(_root, _sub))
        if os.path.isfile(os.path.join(_cand, 'tk.tcl')):
            os.environ['TK_LIBRARY'] = _cand
            try:
                import ctypes
                ctypes.windll.kernel32.SetEnvironmentVariableW('TK_LIBRARY', _cand)
            except Exception:
                pass
            break
    if 'TK_LIBRARY' in os.environ:
        break
