"""Traffic-signal perception and stop-line control from the porting bundle.

Load perception dependencies lazily so the standard-library web launcher can
use ``signal_control.launch`` without importing OpenCV or NumPy.
"""
from importlib import import_module

_EXPORTS = {
    "DEFAULT_CONFIG": "signal_config", "load_config": "signal_config",
    "save_config": "signal_config", "validate_config": "signal_config",
    "StableSignal": "signal_logic", "analyze_frame": "signal_logic",
    "requires_stop": "signal_logic", "SignalPathGate": "signal_path",
    "SignalFramePipeline": "pipeline",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    value = getattr(import_module("." + _EXPORTS[name], __name__), name)
    globals()[name] = value
    return value
