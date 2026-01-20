# Auto-generated init for custom_msgs
import importlib
import sys

__all__ = ['movement', 'signal', 'speech_in']

for name in __all__:
    module = importlib.import_module(f"{__name__}.{name}")
    sys.modules[name] = module
