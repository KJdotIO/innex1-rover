class ExcavationJog:
    class Request:
        duration_s: float

        def __init__(self) -> None: ...

    class Response:
        success: bool
        message: str

        def __init__(self) -> None: ...
