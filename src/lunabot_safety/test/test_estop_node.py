from std_msgs.msg import Bool

from lunabot_safety.estop_node import EstopNode
from lunabot_interfaces.msg import PowerTelemetry


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


def _bool(value):
    msg = Bool()
    msg.data = value
    return msg


def _node():
    node = object.__new__(EstopNode)
    node._estop_active = False
    node._power_critical = False
    node._inhibited = False
    node._reset_required = False
    node._active_reasons = set()
    node._power_inhibit_enabled = True
    node._motion_inhibit_pub = FakePublisher()
    logger = FakeLogger()
    node.get_logger = lambda: logger
    node._test_logger = logger
    return node


def test_estop_assertion_latches_motion_inhibit():
    node = _node()

    node._estop_callback(_bool(True))

    assert node._estop_active is True
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True]


def test_estop_clear_keeps_motion_inhibit_until_reset():
    node = _node()

    node._estop_callback(_bool(True))
    node._estop_callback(_bool(False))

    assert node._estop_active is False
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True, True]


def test_reset_while_estop_active_is_ignored():
    node = _node()

    node._estop_callback(_bool(True))
    node._reset_callback(_bool(True))

    assert node._estop_active is True
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True, True]


def test_reset_after_estop_clear_allows_motion():
    node = _node()

    node._estop_callback(_bool(True))
    node._estop_callback(_bool(False))
    node._reset_callback(_bool(True))

    assert node._estop_active is False
    assert node._reset_required is False
    assert node._inhibited is False
    assert node._motion_inhibit_pub.messages == [True, True, False]


def test_false_reset_message_is_ignored():
    node = _node()

    node._reset_callback(_bool(False))

    assert node._motion_inhibit_pub.messages == []


def _power_msg(*, critical=False):
    msg = PowerTelemetry()
    msg.state = (
        PowerTelemetry.STATE_LOW_CRITICAL
        if critical
        else PowerTelemetry.STATE_OK
    )
    msg.low_voltage_critical = critical
    return msg


def test_critical_power_latches_motion_inhibit():
    node = _node()

    node._power_callback(_power_msg(critical=True))

    assert node._power_critical is True
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True]


def test_power_recovery_keeps_motion_inhibit_until_reset():
    node = _node()

    node._power_callback(_power_msg(critical=True))
    node._power_callback(_power_msg(critical=False))

    assert node._power_critical is False
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True, True]


def test_reset_while_power_is_critical_is_ignored():
    node = _node()

    node._power_callback(_power_msg(critical=True))
    node._reset_callback(_bool(True))

    assert node._power_critical is True
    assert node._reset_required is True
    assert node._inhibited is True
    assert node._motion_inhibit_pub.messages == [True, True]


def test_reset_after_power_recovery_allows_motion():
    node = _node()

    node._power_callback(_power_msg(critical=True))
    node._power_callback(_power_msg(critical=False))
    node._reset_callback(_bool(True))

    assert node._power_critical is False
    assert node._reset_required is False
    assert node._inhibited is False
    assert node._motion_inhibit_pub.messages == [True, True, False]


def test_estop_and_power_both_clear_before_reset_allows_motion():
    node = _node()

    node._estop_callback(_bool(True))
    node._power_callback(_power_msg(critical=True))
    node._estop_callback(_bool(False))
    node._reset_callback(_bool(True))
    node._power_callback(_power_msg(critical=False))
    node._reset_callback(_bool(True))

    assert node._reset_required is False
    assert node._inhibited is False
    assert node._motion_inhibit_pub.messages[-1] is False


def test_power_inhibit_can_be_disabled():
    node = _node()
    node._power_inhibit_enabled = False

    node._power_callback(_power_msg(critical=True))

    assert node._power_critical is False
    assert node._reset_required is False
    assert node._motion_inhibit_pub.messages == []
