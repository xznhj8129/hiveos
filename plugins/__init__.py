"""MPFC plugin package.

Loading the plugin package resolves the configured/sibling OCCID SDK before
individual endpoint modules import `interop.*`. Normal `main.py` startup already
does this through lib.occid_bus; this keeps direct plugin imports equivalent.
"""

from lib.occid_bus import occid as _occid  # noqa: F401
