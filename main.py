"""Source and packaged entry point for the reorganized QTG Studio."""
import os
import sys

# Ensure Windows OS and Python DLL loaders locate internal modules and Tcl/Tk data
# especially when running from removable USB drives (e.g. E:\, F:\)
if getattr(sys, "frozen", False):
    _exe_dir = os.path.dirname(sys.executable)
    _int_dir = os.path.join(_exe_dir, "_internal")
    _search_bases = [
        getattr(sys, "_MEIPASS", ""),
        _exe_dir,
        _int_dir,
    ]
    for _base in _search_bases:
        if not _base:
            continue
        for _sub in ["_tcl_data", "lib/tcl8.6", "tcl/tcl8.6", "tcl8.6"]:
            _c = os.path.normpath(os.path.join(_base, _sub))
            if os.path.isfile(os.path.join(_c, "init.tcl")):
                os.environ["TCL_LIBRARY"] = _c
                try:
                    import ctypes
                    ctypes.windll.kernel32.SetEnvironmentVariableW("TCL_LIBRARY", _c)
                except Exception:
                    pass
                break
        if "TCL_LIBRARY" in os.environ:
            break

    for _base in _search_bases:
        if not _base:
            continue
        for _sub in ["_tk_data", "lib/tk8.6", "tcl/tk8.6", "tk8.6"]:
            _c = os.path.normpath(os.path.join(_base, _sub))
            if os.path.isfile(os.path.join(_c, "tk.tcl")):
                os.environ["TK_LIBRARY"] = _c
                try:
                    import ctypes
                    ctypes.windll.kernel32.SetEnvironmentVariableW("TK_LIBRARY", _c)
                except Exception:
                    pass
                break
        if "TK_LIBRARY" in os.environ:
            break

from frontend import main as _frontend

# Existing imports of ``main`` and patching of its functions use the same module.
sys.modules[__name__] = _frontend

if __name__ == "__main__":
    _frontend.cli()
