from typing import ClassVar

class Deposit:
    class Goal:
        MODE_AUTO: ClassVar[int]
        MODE_TELEOP_ASSIST: ClassVar[int]
        mode: int
        timeout_s: float
        dump_duration_s: float
        require_close_after_dump: bool

        def __init__(self) -> None: ...

    class Result:
        REASON_SUCCESS: ClassVar[int]
        REASON_TIMEOUT: ClassVar[int]
        REASON_ESTOP: ClassVar[int]
        REASON_DRIVER_FAULT: ClassVar[int]
        REASON_LIMIT_NOT_REACHED: ClassVar[int]
        REASON_INTERLOCK_BLOCKED: ClassVar[int]
        REASON_CANCELED: ClassVar[int]
        REASON_FORCED_FAILURE: ClassVar[int]
        REASON_SHUTDOWN: ClassVar[int]
        success: bool
        reason_code: int
        failure_reason: str
        residual_fill_fraction_estimate: float
        duration_s: float

        def __init__(self) -> None: ...

    class Feedback:
        PHASE_PRECHECK: ClassVar[int]
        PHASE_OPENING: ClassVar[int]
        PHASE_RAISING: ClassVar[int]
        PHASE_DUMPING: ClassVar[int]
        PHASE_CLOSING: ClassVar[int]
        phase: int
        elapsed_s: float
        actuator_current_a: float
        door_open: bool
        bed_raised: bool
        estop_active: bool

        def __init__(self) -> None: ...


class Excavate:
    class Goal:
        MODE_AUTO: ClassVar[int]
        MODE_TELEOP_ASSIST: ClassVar[int]
        mode: int
        timeout_s: float
        target_fill_fraction: float
        max_drive_speed_mps: float

        def __init__(self) -> None: ...

    class Result:
        REASON_SUCCESS: ClassVar[int]
        REASON_TIMEOUT: ClassVar[int]
        REASON_ESTOP: ClassVar[int]
        REASON_DRIVER_FAULT: ClassVar[int]
        REASON_JAM_OR_OVERCURRENT: ClassVar[int]
        REASON_INTERLOCK_BLOCKED: ClassVar[int]
        REASON_CANCELED: ClassVar[int]
        REASON_FORCED_FAILURE: ClassVar[int]
        REASON_SHUTDOWN: ClassVar[int]
        success: bool
        reason_code: int
        failure_reason: str
        collected_mass_kg_estimate: float
        duration_s: float

        def __init__(self) -> None: ...

    class Feedback:
        PHASE_PRECHECK: ClassVar[int]
        PHASE_SPINUP: ClassVar[int]
        PHASE_DIGGING: ClassVar[int]
        PHASE_RETRACT: ClassVar[int]
        phase: int
        elapsed_s: float
        fill_fraction_estimate: float
        excavation_motor_current_a: float
        jam_detected: bool
        estop_active: bool

        def __init__(self) -> None: ...
