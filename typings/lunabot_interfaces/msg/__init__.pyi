from typing import ClassVar

class _TimeMsg:
    sec: int
    nanosec: int


class _HeaderMsg:
    stamp: _TimeMsg


class ExcavationCommand:
    COMMAND_STOP: ClassVar[int]
    COMMAND_HOME: ClassVar[int]
    COMMAND_START: ClassVar[int]
    COMMAND_CLEAR_FAULT: ClassVar[int]
    command: int

    def __init__(self) -> None: ...


class ExcavationStatus:
    STATE_IDLE: ClassVar[int]
    STATE_HOMING: ClassVar[int]
    STATE_READY: ClassVar[int]
    STATE_STARTING: ClassVar[int]
    STATE_EXCAVATING: ClassVar[int]
    STATE_STOPPING: ClassVar[int]
    STATE_FAULT: ClassVar[int]
    FAULT_NONE: ClassVar[int]
    FAULT_ESTOP: ClassVar[int]
    FAULT_DRIVER: ClassVar[int]
    FAULT_OVERCURRENT: ClassVar[int]
    FAULT_HOME_SWITCH_INVALID: ClassVar[int]
    FAULT_COMMAND_REJECTED: ClassVar[int]
    header: _HeaderMsg
    state: int
    fault_code: int
    estop_active: bool
    driver_fault: bool
    homed: bool
    motor_enabled: bool
    motor_current_a: float

    def __init__(self) -> None: ...


class ExcavationTelemetry:
    FAULT_NONE: ClassVar[int]
    FAULT_ESTOP: ClassVar[int]
    FAULT_DRIVER: ClassVar[int]
    FAULT_OVERCURRENT: ClassVar[int]
    FAULT_HOME_SWITCH_INVALID: ClassVar[int]
    FAULT_COMMAND_REJECTED: ClassVar[int]
    header: _HeaderMsg
    estop_active: bool
    driver_fault: bool
    home_switch: bool
    motor_enabled: bool
    motor_current_a: float
    fault_code: int

    def __init__(self) -> None: ...


class LocalisationStartZoneStatus:
    stamp: _TimeMsg
    state: str
    ready: bool
    reason_code: str
    reason_detail: str
    stable_lock_age_s: float
    search_elapsed_s: float

    def __init__(self) -> None: ...
