"""Compatibility import for tunnel planning and its shared geometry helpers.

Alias the implementation so legacy patches retain the planner's runtime globals.
"""
import sys

from planning.tunnel import runtime as _implementation

sys.modules[__name__] = _implementation
