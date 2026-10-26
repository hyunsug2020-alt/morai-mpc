"""Compatibility import for the highway lane map and policy.

Alias the policy module so legacy imports retain its runtime globals.
"""
import sys

from planning.highway import policy as _implementation

sys.modules[__name__] = _implementation
