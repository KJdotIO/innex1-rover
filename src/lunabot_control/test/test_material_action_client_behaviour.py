"""Behaviour tests for material action client startup waits."""

from types import SimpleNamespace

from lunabot_control.material_action_client import MaterialActionClient


class _FakeActionClient:
    def __init__(self, ready):
        self._ready = ready
        self.wait_calls = []

    def wait_for_server(self, timeout_sec):
        self.wait_calls.append(timeout_sec)
        return self._ready


def _material_client(*, excavate_ready=True, deposit_ready=True, wait_timeout_s=15.0):
    client = object.__new__(MaterialActionClient)
    client._excavate_client = _FakeActionClient(excavate_ready)
    client._deposit_client = _FakeActionClient(deposit_ready)
    client.get_parameter = lambda name: SimpleNamespace(
        value={
            "action_server_wait_timeout_s": wait_timeout_s,
        }[name]
    )
    client.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    return client


def test_run_sequence_uses_configured_server_wait_timeout():
    client = _material_client(excavate_ready=False, wait_timeout_s=18.0)

    status = client.run_sequence()

    assert status == 1
    assert client._excavate_client.wait_calls == [18.0]
    assert client._deposit_client.wait_calls == []


def test_run_sequence_waits_for_deposit_server_after_excavation_server():
    client = _material_client(deposit_ready=False, wait_timeout_s=14.0)

    status = client.run_sequence()

    assert status == 1
    assert client._excavate_client.wait_calls == [14.0]
    assert client._deposit_client.wait_calls == [14.0]
