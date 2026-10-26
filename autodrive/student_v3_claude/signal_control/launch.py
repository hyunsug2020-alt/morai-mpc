"""Map web-selected signal use to a child-process environment setting."""


def signal_guard_env(enabled):
    if not isinstance(enabled, bool):
        raise ValueError("signal_enabled must be a JSON boolean")
    return "STUDENT_SIGNAL_GUARD=" + ("1" if enabled else "0")


def command_with_signal_setting(command, enabled):
    """Return an env-prefixed argv list; command is never run by this module."""
    if not command:
        raise ValueError("command must not be empty")
    return ["env", signal_guard_env(enabled), *command]
