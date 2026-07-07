import json
import socket
from pathlib import Path
from types import SimpleNamespace

import pytest

import realtime_ipc
from ak_realtime_core import ControlMode, RealtimeTarget
from run_realtime_controller import RealtimeController, safe_send_response


class ReplySocket:
    def __init__(self, missing_addrs=()):
        self.missing_addrs = set(missing_addrs)
        self.sent = []

    def sendto(self, payload, addr):
        if addr in self.missing_addrs:
            raise FileNotFoundError(2, "No such file or directory", addr)
        self.sent.append((json.loads(payload.decode("utf-8")), addr))


def test_safe_send_response_succeeds_for_live_reply_socket():
    sock = ReplySocket()
    bound = SimpleNamespace(sock=sock)

    assert safe_send_response(bound, {"ok": True}, "/tmp/live.sock") is True
    assert sock.sent == [({"ok": True}, "/tmp/live.sock")]


def test_safe_send_response_returns_false_for_deleted_reply_socket(capsys):
    sock = ReplySocket({"/tmp/gone.sock"})
    bound = SimpleNamespace(sock=sock)

    assert safe_send_response(bound, {"ok": True}, "/tmp/gone.sock") is False
    assert "client socket no longer exists" in capsys.readouterr().out


class ServeSocket(ReplySocket):
    def __init__(self, controller):
        super().__init__({"/tmp/gone.sock"})
        self.controller = controller
        self.requests = [
            (json.dumps({"command": "STATUS", "request_id": "first"}).encode(), "/tmp/gone.sock"),
            (json.dumps({"command": "STATUS", "request_id": "second"}).encode(), "/tmp/live.sock"),
        ]

    def setblocking(self, _blocking):
        pass

    def recvfrom(self, _max_packet_size):
        if self.requests:
            return self.requests.pop(0)
        self.controller.running = False
        raise BlockingIOError


def test_deleted_reply_does_not_stop_serve_or_change_motor_state():
    calibration_path = Path(__file__).resolve().parent / "motor_calibration.yaml"
    controller = RealtimeController(
        "can0",
        [0x001],
        50.0,
        True,
        ControlMode.DEFAULT,
        calibration_path,
    )
    controller.core.on_controller_restart = lambda: None
    target = RealtimeTarget(0x001, 0.1, 8.0, 0.4)
    controller.core.latest_targets[0x001] = target
    sock = ServeSocket(controller)

    assert controller.serve(SimpleNamespace(sock=sock)) == 0
    assert any(response["request_id"] == "second" for response, _addr in sock.sent)
    assert controller.core.bus_fault is False
    assert controller.core.latest_targets[0x001] is target


class TimeoutClientSocket:
    def __init__(self):
        self.bound_path = None
        self.closed = False

    def bind(self, path):
        self.bound_path = Path(path)
        self.bound_path.touch()

    def settimeout(self, _timeout):
        pass

    def sendto(self, _payload, _path):
        pass

    def recvfrom(self, _max_packet_size):
        raise socket.timeout("timed out")

    def close(self):
        self.closed = True


def test_send_request_timeout_reports_context_and_cleans_reply_socket(tmp_path, monkeypatch):
    client = TimeoutClientSocket()
    monkeypatch.setattr(realtime_ipc, "BASE_DIR", tmp_path)
    monkeypatch.setattr(realtime_ipc.socket, "socket", lambda *_args, **_kwargs: client)
    controller_socket = tmp_path / "controller.sock"

    with pytest.raises(TimeoutError) as exc_info:
        realtime_ipc.send_request(
            {"command": "STATUS"},
            controller_socket,
            timeout_sec=0.25,
        )

    message = str(exc_info.value)
    assert "command=STATUS" in message
    assert "timeout_sec=0.25" in message
    assert f"controller_socket={controller_socket}" in message
    assert "reply_socket=" in message
    assert client.closed is True
    assert client.bound_path is not None
    assert not client.bound_path.exists()
