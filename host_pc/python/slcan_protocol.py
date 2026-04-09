"""
Compatibility shim.

The project now uses python-can instead of the old serial SLCAN transport.
Keep this module so existing imports do not break immediately.
"""

from can_protocol import *  # noqa: F401,F403
