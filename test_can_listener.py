from unittest.mock import MagicMock, call, patch

import can

from can_listener import RawCanPrintListener, cleanup_resources, format_can_message, run_listener


def make_message(arbitration_id=0x02E, data=None, timestamp=123456.789123):
    if data is None:
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    return can.Message(
        timestamp=timestamp,
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
    )


def test_format_can_message_includes_timestamp_id_dlc_and_data():
    msg = make_message()

    formatted = format_can_message(msg)

    assert "t=123456.789123" in formatted
    assert "ID: 0x02E" in formatted
    assert "DLC: 8" in formatted
    assert "Data: [FF, FF, FF, FF, FF, FF, FF, FC]" in formatted


def test_raw_can_print_listener_prints_message_and_increments_count(capsys):
    listener = RawCanPrintListener()

    listener.on_message_received(make_message())

    captured = capsys.readouterr()
    assert listener.rx_count == 1
    assert "ID: 0x02E" in captured.out
    assert "Data: [FF, FF, FF, FF, FF, FF, FF, FC]" in captured.out


def test_raw_can_print_listener_filters_ids_and_ignores_non_matching_frames(capsys):
    listener = RawCanPrintListener(filter_ids={0x02E})

    listener.on_message_received(make_message(arbitration_id=0x02F, data=[0xAA]))
    ignored = capsys.readouterr()
    listener.on_message_received(make_message(arbitration_id=0x02E, data=[0xBB]))
    accepted = capsys.readouterr()

    assert listener.rx_count == 1
    assert ignored.out == ""
    assert "ID: 0x02E" in accepted.out
    assert "Data: [BB]" in accepted.out


def test_raw_can_print_listener_on_error_prints_rx_error(capsys):
    listener = RawCanPrintListener()

    listener.on_error(RuntimeError("listener failed"))

    captured = capsys.readouterr()
    assert "[RxError]" in captured.out
    assert "listener failed" in captured.out


def test_cleanup_resources_stops_notifier_before_bus_shutdown():
    notifier = MagicMock()
    bus = MagicMock()
    order = MagicMock()
    order.attach_mock(notifier.stop, "notifier_stop")
    order.attach_mock(bus.shutdown, "bus_shutdown")

    cleanup_resources(notifier, bus)

    assert order.mock_calls == [call.notifier_stop(), call.bus_shutdown()]


def test_cleanup_resources_still_shuts_down_bus_when_notifier_stop_fails(capsys):
    notifier = MagicMock()
    notifier.stop.side_effect = RuntimeError("stop failed")
    bus = MagicMock()

    cleanup_resources(notifier, bus)

    captured = capsys.readouterr()
    notifier.stop.assert_called_once()
    bus.shutdown.assert_called_once()
    assert "[CleanupError]" in captured.out
    assert "stop failed" in captured.out


def test_run_listener_uses_mocked_bus_and_cleans_up_on_keyboard_interrupt():
    bus = MagicMock()
    notifier = MagicMock()

    with patch("can_listener.can.Bus", return_value=bus) as bus_factory, patch(
        "can_listener.can.Notifier", return_value=notifier
    ) as notifier_factory, patch("can_listener.time.sleep", side_effect=KeyboardInterrupt):
        run_listener(channel="vcan0", interface="socketcan", filter_ids={0x02E})

    bus_factory.assert_called_once_with(interface="socketcan", channel="vcan0")
    notifier_factory.assert_called_once()
    notifier.stop.assert_called_once()
    bus.shutdown.assert_called_once()
