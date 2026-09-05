# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - Raw CSI Record Tests

Unit tests for the transport-neutral CSI raw record parser and dataset writer.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

import copy
import io
import socket
import threading
import time

import numpy as np
import pytest

from tools import espectre_traffic_generator
from tools.lib import dataset_metadata
from tools.lib.csi_io import (
    CSICollector,
    CSIReceiver,
    DirectRawCSIReceiver,
    CSI_HEADER_STRUCT,
    RAW_CSI_FLAG_WIFI_RX_START_TS_NS_VALID,
    RAW_CSI_FLAG_WIFI_RX_TS_VALID,
    RAW_CSI_RECORD_MAGIC,
    RAW_CSI_RECORD_VERSION,
    RAW_CSI_RECORD_VERSION_V7,
    RAW_CSI_RECORD_VERSION_V8,
    RAW_CSI_HTTP_FRAME_STRUCT,
    RAW_CSI_PROTOCOL_VERSION,
    RAW_CSI_RESPONSE_MAGIC,
    load_npz_arrays,
    load_npz_as_packets,
    load_npz_packet_view,
    normalize_stored_csi_bin_layout,
    parse_csi_record,
)

from device_utils import (
    HT20_CENTERED_ONLY_NULL_BINS,
    HT20_CLASSIC_ONLY_NULL_BINS,
)


def _layout_rows(null_bins, num_rows=3):
    """Build CSI rows populated everywhere except one layout's guard bins."""
    rows = np.zeros((num_rows, 128), dtype=np.int8)
    for bin_index in range(64):
        rows[:, bin_index * 2] = 7 + (bin_index % 5)
        rows[:, bin_index * 2 + 1] = -3 - (bin_index % 4)
    for bin_index in null_bins:
        rows[:, bin_index * 2] = 0
        rows[:, bin_index * 2 + 1] = 0
    return rows


def test_stored_classic_layout_is_rotated_to_centered():
    classic = _layout_rows(HT20_CLASSIC_ONLY_NULL_BINS)
    rotated = normalize_stored_csi_bin_layout(classic)
    np.testing.assert_array_equal(rotated, np.roll(classic, 64, axis=1))


def test_stored_centered_layout_is_left_alone():
    centered = _layout_rows(HT20_CENTERED_ONLY_NULL_BINS)
    np.testing.assert_array_equal(normalize_stored_csi_bin_layout(centered), centered)


def test_sparse_rows_are_left_alone():
    """Rows null under both conventions carry no evidence either way."""
    sparse = np.zeros((3, 128), dtype=np.int8)
    sparse[:, :4] = np.array([1, 2, 3, 4], dtype=np.int8)
    np.testing.assert_array_equal(normalize_stored_csi_bin_layout(sparse), sparse)


def test_load_npz_arrays_rotates_classic_recordings(tmp_path):
    filepath = tmp_path / "classic_layout.npz"
    classic = _layout_rows(HT20_CLASSIC_ONLY_NULL_BINS)
    np.savez_compressed(filepath, csi_data=classic, num_subcarriers=64, label="motion", chip="c3")

    loaded = load_npz_arrays(filepath)["csi_data"]
    np.testing.assert_array_equal(loaded, np.roll(classic, 64, axis=1))


def test_load_npz_arrays_returns_read_only_cached_mapping(tmp_path):
    filepath = tmp_path / "read_only_arrays.npz"
    np.savez_compressed(
        filepath,
        csi_data=np.zeros((1, 128), dtype=np.int8),
        num_subcarriers=64,
        label="motion",
        chip="c3",
    )

    arrays = load_npz_arrays(filepath)

    assert arrays["csi_data"].flags.writeable is False
    with pytest.raises(TypeError):
        arrays["extra"] = np.zeros(1, dtype=np.int8)


def test_packet_view_is_read_only_but_packet_loader_returns_writable_copies(tmp_path):
    filepath = tmp_path / "packet_views.npz"
    np.savez_compressed(
        filepath,
        csi_data=np.arange(128, dtype=np.int8).reshape(1, 128),
        num_subcarriers=64,
        label="motion",
        chip="c3",
    )

    packet_view = load_npz_packet_view(filepath)
    assert packet_view[0]["csi_data"].flags.writeable is False
    with pytest.raises(TypeError):
        packet_view[0]["label"] = "changed"

    packets = load_npz_as_packets(filepath)
    assert packets[0]["csi_data"].flags.writeable is True
    packets[0]["csi_data"][0] = -5
    assert load_npz_packet_view(filepath)[0]["csi_data"][0] != -5


def build_packet(
    *,
    seq_num=1,
    chip_code=6,
    flags=0,
    payload=None,
    device_id=0x112233445566,
    device_ticks_us=123456,
    wifi_rx_ts_us=0,
    wifi_rx_start_ts_ns=0,
    channel=6,
    rssi_dbm=-42,
    noise_floor_dbm=-96,
    transport_backpressure_total=0,
    fresh_record_total=0,
    request_accepted_total=0,
    phy_mode=2,
    ltf_type=2,
    channel_width=1,
    version=RAW_CSI_RECORD_VERSION,
):
    payload_values = payload if payload is not None else [1, 2, 3, 4]
    payload = np.array(payload_values, dtype=np.int8).tobytes()
    num_sc = len(payload) // 2
    header = CSI_HEADER_STRUCT.pack(
        RAW_CSI_RECORD_MAGIC,
        version,
        CSI_HEADER_STRUCT.size,
        chip_code,
        flags,
        seq_num,
        num_sc,
        len(payload),
        device_id,
        device_ticks_us,
        wifi_rx_ts_us,
        wifi_rx_start_ts_ns,
        channel,
        rssi_dbm,
        noise_floor_dbm,
        transport_backpressure_total,
        fresh_record_total,
        request_accepted_total,
        phy_mode,
        ltf_type,
        channel_width,
    )
    return header + payload


def test_parse_packet_accepts_transport_neutral_v8_header():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packet = receiver._parse_packet(
        build_packet(
            seq_num=7,
            payload=[10, 20, -30, 40],
            device_ticks_us=987654,
            channel=11,
            rssi_dbm=-55,
        )
    )

    assert packet is not None
    assert packet.seq_num == 7
    assert packet.num_subcarriers == 2
    assert packet.chip == 'C6'
    assert packet.device_ticks_us == 987654
    assert packet.channel == 11
    assert packet.rssi_dbm == -55
    assert packet.transport_backpressure_total == 0
    assert packet.phy_mode == 'ht'
    assert packet.ltf_type == 'ht-ltf'
    assert packet.channel_width == '20'
    np.testing.assert_array_equal(packet.iq_raw, np.array([10, 20, -30, 40], dtype=np.int8))
    np.testing.assert_allclose(packet.iq_complex, np.array([20 + 10j, 40 - 30j], dtype=np.complex64))


def test_parse_csi_record_reads_historical_v7_offline():
    record = build_packet(version=RAW_CSI_RECORD_VERSION_V7, seq_num=17)

    packet, next_offset = parse_csi_record(record, derive_complex=False)

    assert packet is not None
    assert packet.record_version == RAW_CSI_RECORD_VERSION_V7
    assert packet.seq_num == 17
    assert next_offset == len(record)


def test_parse_packet_accepts_v8_transport_neutral_counters_byte_for_byte():
    receiver = CSIReceiver(bind_host="127.0.0.1", derive_complex=False)
    record = build_packet(
        version=RAW_CSI_RECORD_VERSION_V8,
        transport_backpressure_total=11,
        fresh_record_total=22,
        request_accepted_total=33,
        payload=[1, -2, 3, -4],
    )

    packet = receiver._parse_packet(record)

    assert packet is not None
    assert packet.record_version == RAW_CSI_RECORD_VERSION_V8
    assert packet.transport_backpressure_total == 11
    assert packet.fresh_record_total == 22
    assert packet.request_accepted_total == 33
    assert record[: CSI_HEADER_STRUCT.size] == CSI_HEADER_STRUCT.pack(
        RAW_CSI_RECORD_MAGIC,
        RAW_CSI_RECORD_VERSION_V8,
        64,
        6,
        0,
        1,
        2,
        4,
        0x112233445566,
        123456,
        0,
        0,
        6,
        -42,
        -96,
        11,
        22,
        33,
        2,
        2,
        1,
    )


def test_direct_raw_receiver_negotiates_v8_and_feeds_shared_packet_parser():
    session_id = bytes.fromhex("00112233445566778899aabbccddeeff")

    class FakeControl:
        def __init__(self, endpoint, **_kwargs):
            self.endpoint = endpoint
            self.requests = []
            self.closed = False

        def request(self, verb, resource, data=None, **_kwargs):
            self.requests.append((verb, resource, data))
            if resource == "capabilities":
                return {
                    "features": {"csi": True},
                    "csi": {
                        "transport": "http",
                        "protocol_version": RAW_CSI_PROTOCOL_VERSION,
                        "record_version": 8,
                        "frame_prefix_bytes": 60,
                    },
                }
            if resource == "device":
                return {
                    "device_id": "112233445566",
                    "frontend": "native",
                    "firmware": "test",
                    "chip": "esp32c3",
                }
            if resource == "diagnostics":
                return {
                    "raw_csi": {
                        "fresh_record_total": 1,
                        "raw_drop_total": 2,
                        "send_backpressure_total": 3,
                        "stream_sequence": 3,
                    }
                }
            raise AssertionError((verb, resource, data))

        def close(self):
            self.closed = True

    class FakeRawResponse:
        status = 200

        def __init__(self, payload):
            self.chunks = [payload[:17], payload[17:93], payload[93:]]
            self.closed = False

        def read1(self, _size):
            return self.chunks.pop(0)

        def close(self):
            self.closed = True

    class FakeRawConnection:
        def __init__(self):
            self.closed = False
            self.request_args = None
            self.socket_timeouts = []
            self.sock = self
            record = build_packet(
                version=RAW_CSI_RECORD_VERSION_V8,
                flags=1 << 3,
                transport_backpressure_total=2,
                fresh_record_total=1,
                request_accepted_total=1,
            )
            prefix = RAW_CSI_HTTP_FRAME_STRUCT.pack(
                RAW_CSI_RESPONSE_MAGIC,
                RAW_CSI_PROTOCOL_VERSION,
                8,
                RAW_CSI_HTTP_FRAME_STRUCT.size,
                session_id,
                1,
                len(record),
                0,
                1,
                0,
                2,
            )
            self.response = FakeRawResponse(prefix + record)

        def settimeout(self, timeout):
            self.socket_timeouts.append(timeout)

        def request(self, method, path, headers):
            self.request_args = (method, path, headers)

        def getresponse(self):
            return self.response

        def close(self):
            self.closed = True

    control = None
    raw = FakeRawConnection()

    def control_factory(*args, **kwargs):
        nonlocal control
        control = FakeControl(*args, **kwargs)
        return control

    receiver = DirectRawCSIReceiver(
        "192.168.1.23:62587",
        control_client_factory=control_factory,
        raw_connection_factory=lambda *_args, **_kwargs: raw,
        derive_complex=False,
    )
    observed = []
    receiver.add_callback(observed.append)

    receiver._open()
    receiver._raw_buffer.extend(receiver._read_raw_chunk())
    receiver._consume_raw_frames()
    assert observed == []
    receiver._raw_buffer.extend(receiver._read_raw_chunk())
    receiver._consume_raw_frames()
    receiver._raw_buffer.extend(receiver._read_raw_chunk())
    receiver._consume_raw_frames()
    detached_packet = copy.copy(observed[0])
    receiver.stop()
    receiver.apply_final_raw_diagnostics(detached_packet)

    assert len(observed) == 1
    assert observed[0].record_version == RAW_CSI_RECORD_VERSION_V8
    assert observed[0].transport == "http"
    assert observed[0].source_ip == "192.168.1.23"
    assert observed[0].raw_stream_sequence == 1
    assert observed[0].raw_final_stream_sequence == 3
    assert observed[0].fresh_record_total == 1
    assert observed[0].raw_drop_total == 2
    assert observed[0].raw_send_backpressure_total == 3
    assert detached_packet.raw_final_stream_sequence == 3
    assert detached_packet.fresh_record_total == 1
    assert detached_packet.raw_drop_total == 2
    assert detached_packet.raw_send_backpressure_total == 3
    assert raw.request_args[0:2] == ("GET", "/espectre/v1/csi")
    assert "Authorization" not in raw.request_args[2]
    assert raw.socket_timeouts == [None]
    assert [(request[0], request[1]) for request in control.requests] == [
        ("get", "capabilities"),
        ("get", "device"),
        ("get", "diagnostics"),
    ]


def test_direct_raw_receiver_run_timeout_does_not_wait_for_idle_stream():
    class IdleRawResponse:
        def __init__(self):
            self.closed = threading.Event()

        def read1(self, _size):
            self.closed.wait(timeout=2.0)
            return b""

        def close(self):
            self.closed.set()

    class IdleRawConnection:
        def close(self):
            pass

    response = IdleRawResponse()
    receiver = DirectRawCSIReceiver("192.168.1.23", derive_complex=False)
    receiver._raw_response = response
    receiver._raw_connection = IdleRawConnection()
    receiver.running = True
    receiver._open = lambda: None

    started_at = time.monotonic()
    receiver.run(timeout=0.05)
    elapsed = time.monotonic() - started_at
    receiver.stop()

    assert elapsed < 0.5
    assert response.closed.is_set()


def test_direct_raw_receiver_rejects_capability_mismatch_without_transport_fallback():
    class IncompatibleControl:
        def __init__(self, *_args, **_kwargs):
            self.closed = False

        def request(self, verb, resource, *_args, **_kwargs):
            assert (verb, resource) == ("get", "capabilities")
            return {"features": {"csi": False}}

        def close(self):
            self.closed = True

    receiver = DirectRawCSIReceiver(
        "192.168.1.24",
        control_client_factory=IncompatibleControl,
    )

    with pytest.raises(RuntimeError, match="does not advertise"):
        receiver._open()
    assert receiver._control is None


def test_direct_raw_receiver_uses_shared_ghost_port_for_bare_targets():
    receiver = DirectRawCSIReceiver("192.168.1.23", derive_complex=False)

    assert receiver.control_endpoint == "http://192.168.1.23:62587/espectre/v1"
    assert receiver.raw_endpoint == "http://192.168.1.23:62587/espectre/v1/csi"


def test_direct_raw_receiver_parses_aggregated_frames_and_rejects_sequence_mismatch():
    session_id = bytes.fromhex("00112233445566778899aabbccddeeff")

    def frame(stream_sequence, record_sequence=None):
        sequence = stream_sequence if record_sequence is None else record_sequence
        record = build_packet(
            version=RAW_CSI_RECORD_VERSION_V8,
            seq_num=sequence,
            request_accepted_total=sequence,
            fresh_record_total=stream_sequence,
        )
        prefix = RAW_CSI_HTTP_FRAME_STRUCT.pack(
            RAW_CSI_RESPONSE_MAGIC,
            RAW_CSI_PROTOCOL_VERSION,
            8,
            RAW_CSI_HTTP_FRAME_STRUCT.size,
            session_id,
            stream_sequence,
            len(record),
            0,
            stream_sequence,
            0,
            0,
        )
        return prefix + record

    receiver = DirectRawCSIReceiver("192.168.1.23", derive_complex=False)
    receiver._session_id = session_id
    receiver._raw_buffer.extend(frame(1) + frame(2))
    receiver._consume_raw_frames()
    assert [packet.raw_stream_sequence for packet in receiver.buffer] == [1, 2]

    mismatched = DirectRawCSIReceiver("192.168.1.23", derive_complex=False)
    mismatched._session_id = session_id
    mismatched._raw_buffer.extend(frame(1, record_sequence=2))
    with pytest.raises(RuntimeError, match="invalid V8 record"):
        mismatched._consume_raw_frames()


def test_parse_packet_reads_phy_metadata():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packet = receiver._parse_packet(
        build_packet(
            phy_mode=2,
            ltf_type=2,
            channel_width=2,
        )
    )

    assert packet is not None
    assert packet.phy_mode == 'ht'
    assert packet.ltf_type == 'ht-ltf'
    assert packet.channel_width == '40'


def test_parse_packet_rejects_unknown_record_version():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packet_data = bytearray(build_packet())
    packet_data[2] = RAW_CSI_RECORD_VERSION_V7 - 1

    assert receiver._parse_packet(packet_data) is None


def test_parse_packet_preserves_short_transport_payload():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    payload = []
    for idx in range(12):
        payload.extend([idx + 1, -(idx + 1)])

    packet = receiver._parse_packet(build_packet(seq_num=9, payload=payload))

    assert packet is not None
    assert packet.seq_num == 9
    assert packet.num_subcarriers == 12
    assert packet.iq_raw.shape == (24,)
    np.testing.assert_array_equal(packet.iq_raw, np.array(payload, dtype=np.int8))


def test_parse_packet_derives_complex_data_lazily_when_disabled():
    receiver = CSIReceiver(bind_host='127.0.0.1', derive_complex=False)
    packet = receiver._parse_packet(build_packet(seq_num=8, payload=[10, 20, -30, 40]))

    assert packet is not None
    assert packet._iq_complex is None
    np.testing.assert_allclose(packet.iq_complex, np.array([20 + 10j, 40 - 30j], dtype=np.complex64))
    assert packet._iq_complex is not None


def test_external_traffic_generator_configures_low_latency_multicast(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.options = []

        def setsockopt(self, level, option, value):
            self.options.append((level, option, value))

    sock = FakeSocket()
    monkeypatch.setattr(espectre_traffic_generator, "TARGETS", ["239.255.0.1"])

    espectre_traffic_generator.configure_socket(sock)

    assert (socket.IPPROTO_IP, socket.IP_TOS, 46 << 2) in sock.options
    assert (socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1) in sock.options


def test_external_traffic_generator_uses_canonical_ghost_marker():
    assert espectre_traffic_generator.ExternalTrafficGenerator.TRAFFIC_MARKER == "👻"
    assert espectre_traffic_generator.ExternalTrafficGenerator.PAYLOAD == bytes.fromhex("f09f91bb")


def test_external_traffic_generator_help_exits_successfully(capsys):
    assert espectre_traffic_generator.main(["--help"]) == 0
    output = capsys.readouterr().out
    assert "python3 espectre_traffic_generator.py start" in output
    assert "python3 espectre_traffic_generator.py run" in output


def test_external_traffic_generator_rates_each_target_and_stops_safely(monkeypatch):
    sockets = []

    class FakeSocket:
        def __init__(self):
            self.options = []
            self.bound = None
            self.sent = []
            self.closed = False
            sockets.append(self)

        def setsockopt(self, level, option, value):
            self.options.append((level, option, value))

        def bind(self, address):
            self.bound = address

        def sendto(self, payload, destination):
            self.sent.append((payload, destination))
            if len(self.sent) == 4:
                generator._stop_event.set()

        def close(self):
            self.closed = True

    monkeypatch.setattr(espectre_traffic_generator.socket, "socket", lambda *_args: FakeSocket())
    generator = espectre_traffic_generator.ExternalTrafficGenerator(
        ["127.0.0.1", "127.0.0.2"],
        rate_pps=1000,
        source_ip="127.0.0.3",
    )

    with generator as active:
        deadline = time.monotonic() + 1.0
        while active.running and time.monotonic() < deadline:
            time.sleep(0.001)

    assert not generator.running
    assert generator.sent_packets == 4
    assert generator.sent_by_target == {"127.0.0.1": 2, "127.0.0.2": 2}
    assert generator.errors_by_target == {"127.0.0.1": 0, "127.0.0.2": 0}
    assert sockets[0].bound == ("127.0.0.3", 0)
    assert all(payload == bytes.fromhex("f09f91bb") for payload, _destination in sockets[0].sent)
    assert sockets[0].closed


def test_background_traffic_stop_never_signals_pid_from_state_file(tmp_path, monkeypatch):
    pid_file = tmp_path / "traffic.pid"
    control_file = tmp_path / "traffic.control"
    token = "owned-session"
    pid_file.write_text('{"pid": 4242, "token": "owned-session"}', encoding="utf-8")
    control_file.write_text(token, encoding="utf-8")
    signals = []

    def fake_kill(pid, signal_number):
        signals.append((pid, signal_number))
        raise ProcessLookupError

    monkeypatch.setattr(espectre_traffic_generator, "PID_FILE", pid_file)
    monkeypatch.setattr(espectre_traffic_generator, "CONTROL_FILE", control_file)
    monkeypatch.setattr(espectre_traffic_generator.os, "kill", fake_kill)

    espectre_traffic_generator.stop()

    assert signals
    assert all(signal_number == 0 for _pid, signal_number in signals)
    assert not pid_file.exists()
    assert not control_file.exists()


def test_background_traffic_start_cleans_control_after_launch_failure(tmp_path, monkeypatch):
    pid_file = tmp_path / "traffic.pid"
    control_file = tmp_path / "traffic.control"
    monkeypatch.setattr(espectre_traffic_generator, "PID_FILE", pid_file)
    monkeypatch.setattr(espectre_traffic_generator, "CONTROL_FILE", control_file)
    monkeypatch.setattr(
        espectre_traffic_generator.subprocess,
        "Popen",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("cannot launch")),
    )

    with pytest.raises(OSError, match="cannot launch"):
        espectre_traffic_generator.start()

    assert not pid_file.exists()
    assert not control_file.exists()


def test_parse_packet_reads_optional_metadata():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    flags = RAW_CSI_FLAG_WIFI_RX_TS_VALID | RAW_CSI_FLAG_WIFI_RX_START_TS_NS_VALID
    packet = receiver._parse_packet(
        build_packet(
            seq_num=42,
            flags=flags,
            wifi_rx_ts_us=5555,
            wifi_rx_start_ts_ns=987654321,
        )
    )

    assert packet is not None
    assert packet.wifi_rx_ts_us == 5555
    assert packet.wifi_rx_start_ts_ns == 987654321


def test_parse_packet_reads_transport_backpressure_total():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packet = receiver._parse_packet(
        build_packet(
            seq_num=43,
            transport_backpressure_total=17,
        )
    )

    assert packet is not None
    assert packet.transport_backpressure_total == 17


def test_parse_packet_reads_transport_counters():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    receiver._parse_packet(
        build_packet(
            seq_num=44,
            fresh_record_total=123,
            request_accepted_total=140,
        )
    )
def test_receiver_configures_udp_receive_buffer(monkeypatch):
    calls = []

    class FakeSocket:
        def setsockopt(self, level, optname, value):
            calls.append(("setsockopt", level, optname, value))

        def getsockopt(self, level, optname):
            calls.append(("getsockopt", level, optname))
            return 425984

        def bind(self, addr):
            calls.append(("bind", addr))

        def settimeout(self, value):
            calls.append(("settimeout", value))

        def recvfrom(self, _size):
            raise socket.timeout()

        def close(self):
            calls.append(("close",))

    fake_socket = FakeSocket()
    time_values = iter([1000.0, 1000.0, 1002.0])

    monkeypatch.setattr(socket, "socket", lambda *_args, **_kwargs: fake_socket)
    monkeypatch.setattr("tools.lib.csi_io.time.time", lambda: next(time_values))

    receiver = CSIReceiver(bind_host='127.0.0.1', socket_rcvbuf_bytes=262144)
    receiver.run(timeout=1.0, quiet=True)

    assert receiver.effective_socket_rcvbuf_bytes == 425984
    assert ("setsockopt", socket.SOL_SOCKET, socket.SO_RCVBUF, 262144) in calls
    assert ("getsockopt", socket.SOL_SOCKET, socket.SO_RCVBUF) in calls


def test_receiver_filters_out_of_order_packets_before_buffer_and_callbacks(monkeypatch):
    datagrams = [
        build_packet(seq_num=100, device_id=0x1),
        build_packet(seq_num=3, device_id=0x1),
        build_packet(seq_num=101, device_id=0x1),
    ]

    class FakeSocket:
        def setsockopt(self, *_args):
            pass

        def getsockopt(self, *_args):
            return 425984

        def bind(self, _addr):
            pass

        def settimeout(self, _value):
            pass

        def recvfrom(self, _size):
            return datagrams.pop(0), ("192.0.2.1", 5001)

        def close(self):
            pass

    monkeypatch.setattr(socket, "socket", lambda *_args, **_kwargs: FakeSocket())
    receiver = CSIReceiver(bind_host="127.0.0.1")
    observed = []

    def on_packet(packet):
        observed.append(packet.seq_num)
        if packet.seq_num == 101:
            receiver.stop()

    receiver.add_callback(on_packet)
    receiver.run(quiet=True)

    assert observed == [100, 101]
    assert [packet.seq_num for packet in receiver.buffer] == [100, 101]
    assert receiver.packet_count == 2
    assert receiver.out_of_order_count == 1


def test_parse_packets_accepts_multiple_records_in_one_datagram():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    datagram = build_packet(seq_num=10, payload=[1, 2, 3, 4]) + build_packet(seq_num=11, payload=[5, 6, 7, 8])

    packets = receiver._parse_packets(datagram)

    assert len(packets) == 2
    assert packets[0].seq_num == 10
    assert packets[1].seq_num == 11
    np.testing.assert_array_equal(packets[0].iq_raw, np.array([1, 2, 3, 4], dtype=np.int8))
    np.testing.assert_array_equal(packets[1].iq_raw, np.array([5, 6, 7, 8], dtype=np.int8))


def test_parse_packet_rejects_multi_record_datagram():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    datagram = build_packet(seq_num=1) + build_packet(seq_num=2)

    assert receiver._parse_packet(datagram) is None


def test_parse_packet_rejects_legacy_python_header():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    legacy = bytes([0x53, 0x43, 0x04, 0x01, 0x07, 0x40, 0x00]) + bytes(128)
    assert receiver._parse_packet(legacy) is None


def test_save_sample_keeps_existing_schema_and_adds_optional_metadata(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')

    receiver = CSIReceiver(bind_host='127.0.0.1')
    flags = RAW_CSI_FLAG_WIFI_RX_TS_VALID
    packets = [
        receiver._parse_packet(
            build_packet(
                seq_num=100,
                flags=flags,
                payload=[1, 2, 3, 4],
                device_id=0xABCDEF,
                device_ticks_us=1000,
                wifi_rx_ts_us=4000,
                channel=1,
                rssi_dbm=-50,
            )
        ),
        receiver._parse_packet(
            build_packet(
                seq_num=101,
                flags=flags,
                payload=[5, 6, 7, 8],
                device_id=0xABCDEF,
                device_ticks_us=2000,
                wifi_rx_ts_us=5000,
                channel=1,
                rssi_dbm=-49,
            )
        ),
    ]
    for index, packet in enumerate(packets, start=1000):
        packet.transport = 'http'
        packet.transport_target = 'http://192.168.1.23/espectre/v1/csi'
        packet.requested_pps = 200.0
        packet.raw_protocol_version = RAW_CSI_PROTOCOL_VERSION
        packet.record_version = RAW_CSI_RECORD_VERSION_V8
        packet.raw_stream_sequence = index
        packet.frontend = 'esphome'
        packet.firmware_version = '2.0.0-test'
        packet.firmware_identity = 'espectre-esphome-test'
    packets[-1].fresh_record_total = 102
    packets[-1].raw_drop_total = 3
    packets[-1].raw_send_backpressure_total = 4
    packets[-1].raw_final_stream_sequence = 1004

    collector = CSICollector(
        label='static_presence',
        contributor='tester',
        bind_host='127.0.0.1',
        target_pps=100,
    )
    filepath = collector.save_sample(packets)

    assert filepath is not None
    assert filepath.exists()

    data = np.load(filepath, allow_pickle=False)
    assert str(data['label']) == 'static_presence'
    assert str(data['chip']) == 'c6'
    assert int(data['num_subcarriers']) == 2
    assert str(data['format_version']) == '1.2'
    assert int(data['device_id']) == 0xABCDEF
    assert int(data['csi_target_pps']) == 100
    np.testing.assert_array_equal(data['stream_seq_num'], np.array([100, 101], dtype=np.uint32))
    np.testing.assert_array_equal(data['raw_stream_sequence'], np.array([1000, 1001], dtype=np.uint64))
    assert str(data['transport']) == 'http'
    assert str(data['transport_target']) == 'http://192.168.1.23/espectre/v1/csi'
    assert str(data['endpoint']) == 'http://192.168.1.23/espectre/v1/csi'
    assert float(data['observed_pps']) == float(data['effective_pps'])
    assert float(data['requested_pps']) == 200.0
    assert int(data['raw_protocol_version']) == RAW_CSI_PROTOCOL_VERSION
    assert int(data['record_version']) == 8
    assert str(data['frontend']) == 'esphome'
    assert str(data['firmware_version']) == '2.0.0-test'
    assert str(data['firmware_identity']) == 'espectre-esphome-test'
    assert int(data['raw_fresh_record_total']) == 102
    assert int(data['fresh_record_total']) == 102
    assert int(data['raw_drop_total']) == 3
    assert int(data['raw_send_backpressure_total']) == 4
    assert int(data['send_backpressure_total']) == 4
    assert int(data['raw_final_stream_sequence']) == 1004
    np.testing.assert_array_equal(data['phy_mode'], np.array(['ht', 'ht']))
    np.testing.assert_array_equal(data['ltf_type'], np.array(['ht-ltf', 'ht-ltf']))
    np.testing.assert_array_equal(data['channel_width'], np.array(['20', '20']))
    np.testing.assert_array_equal(data['device_ticks_us'], np.array([1000, 2000], dtype=np.uint64))
    np.testing.assert_array_equal(data['wifi_rx_ts_us'], np.array([4000, 5000], dtype=np.uint32))
    np.testing.assert_array_equal(data['csi_data'], np.array([[1, 2, 3, 4], [5, 6, 7, 8]], dtype=np.int8))

    info = dataset_metadata.load_dataset_info()
    assert info['format_version'] == '1.2'
    assert info['files']['static_presence'][0]['filename'] == filepath.name
    assert info['files']['static_presence'][0]['device_id'] == '0000000000abcdef'
    assert info['files']['static_presence'][0]['description'] == 'HT20 static presence sample'
    assert info['files']['static_presence'][0]['nominal_packet_rate'] == 100
    assert info['files']['static_presence'][0]['transport'] == 'http'
    assert info['files']['static_presence'][0]['endpoint'] == 'http://192.168.1.23/espectre/v1/csi'
    assert info['files']['static_presence'][0]['observed_pps'] == info['files']['static_presence'][0]['effective_pps']
    assert info['files']['static_presence'][0]['requested_pps'] == 200.0
    assert info['files']['static_presence'][0]['frontend'] == 'esphome'
    assert info['files']['static_presence'][0]['raw_fresh_record_total'] == 102
    assert info['files']['static_presence'][0]['fresh_record_total'] == 102
    assert info['files']['static_presence'][0]['raw_drop_total'] == 3
    assert info['files']['static_presence'][0]['raw_send_backpressure_total'] == 4
    assert info['files']['static_presence'][0]['send_backpressure_total'] == 4
    assert info['files']['static_presence'][0]['raw_final_stream_sequence'] == 1004
    assert 'dev0000000000abcdef' in filepath.name


def test_save_sample_preserves_short_transport_schema(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')

    short_payload = []
    for idx in range(12):
        short_payload.extend([idx + 1, 50 + idx])

    receiver = CSIReceiver(bind_host='127.0.0.1')
    packet = receiver._parse_packet(build_packet(seq_num=200, payload=short_payload, device_id=0xABCDEF))

    collector = CSICollector(label='short_transport', contributor='tester', bind_host='127.0.0.1')
    filepath = collector.save_sample([packet])

    assert filepath is not None
    data = np.load(filepath, allow_pickle=False)
    assert int(data['num_subcarriers']) == 12
    assert data['csi_data'].shape == (1, 24)
    np.testing.assert_array_equal(data['csi_data'][0], np.array(short_payload, dtype=np.int8))
    assert '_12sc_' in filepath.name


def test_load_historical_dataset_labels_missing_phy_as_ht20(tmp_path):
    filepath = tmp_path / 'historical_ht20.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.zeros((1, 128), dtype=np.int8),
        num_subcarriers=64,
        label='motion',
        chip='esp32',
    )

    packet = load_npz_as_packets(filepath)[0]

    assert packet['phy_format'] == 'ht20'
    assert packet['phy_mode'] == 'ht'
    assert packet['ltf_type'] == 'ht-ltf'
    assert packet['channel_width'] == '20'
    assert packet['layout_id'] == 'ht20_64'
    assert packet['format_metadata_source'] == 'historical_missing_phy'


def test_load_npz_as_packets_rejects_object_arrays(tmp_path):
    filepath = tmp_path / 'malicious_object_array.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.zeros((1, 128), dtype=np.int8),
        num_subcarriers=64,
        label='motion',
        chip=np.array('esp32', dtype=object),
    )

    with pytest.raises(ValueError, match="Unsafe NPZ dataset"):
        load_npz_as_packets(filepath)


def test_load_dataset_preserves_explicit_ht20_phy_metadata(tmp_path):
    filepath = tmp_path / 'explicit_phy.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.zeros((1, 128), dtype=np.int8),
        num_subcarriers=64,
        label='motion',
        chip='esp32',
        phy_mode=np.array(['ht']),
        ltf_type=np.array(['ht-ltf']),
        channel_width=np.array(['20']),
    )

    packet = load_npz_as_packets(filepath, keep_all_phy=True)[0]

    assert packet['phy_format'] == 'ht20'
    assert packet['phy_mode'] == 'ht'
    assert packet['ltf_type'] == 'ht-ltf'
    assert packet['channel_width'] == '20'
    assert packet['layout_id'] == 'ht20_64'


def test_load_npz_filters_non_ht20_packets_by_default(tmp_path):
    filepath = tmp_path / 'mixed_phy.npz'
    rows = np.zeros((3, 128), dtype=np.int8)
    rows[0, :4] = np.array([1, 2, 3, 4], dtype=np.int8)
    rows[1, :4] = np.array([5, 6, 7, 8], dtype=np.int8)
    rows[2, :4] = np.array([9, 10, 11, 12], dtype=np.int8)
    np.savez_compressed(
        filepath,
        csi_data=rows,
        num_subcarriers=64,
        label='motion',
        chip='esp32',
        phy_mode=np.array(['ht', 'legacy', 'ht']),
        ltf_type=np.array(['ht-ltf', 'unknown', 'ht-ltf']),
        channel_width=np.array(['20', '20', '20']),
        stream_seq_num=np.array([10, 11, 12], dtype=np.uint32),
    )

    packets = load_npz_as_packets(filepath)
    assert len(packets) == 2
    assert [packet['phy_mode'] for packet in packets] == ['ht', 'ht']
    assert [packet['stream_seq_num'] for packet in packets] == [10, 12]
    np.testing.assert_array_equal(packets[0]['csi_data'][:4], np.array([1, 2, 3, 4], dtype=np.int8))
    np.testing.assert_array_equal(packets[1]['csi_data'][:4], np.array([9, 10, 11, 12], dtype=np.int8))

    all_packets = load_npz_as_packets(filepath, keep_all_phy=True)
    assert len(all_packets) == 3
    assert [packet['phy_mode'] for packet in all_packets] == ['ht', 'legacy', 'ht']


def test_load_npz_filters_non_ht_ltf_packets_by_default(tmp_path):
    filepath = tmp_path / 'mixed_ltf.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.zeros((2, 128), dtype=np.int8),
        num_subcarriers=64,
        label='motion',
        chip='esp32',
        phy_mode=np.array(['ht', 'ht']),
        ltf_type=np.array(['ht-ltf', 'vht-ltf']),
        channel_width=np.array(['20', '20']),
    )

    packets = load_npz_as_packets(filepath)
    assert len(packets) == 1
    assert packets[0]['ltf_type'] == 'ht-ltf'


def test_load_npz_filters_non_64sc_historical_rows_by_default(tmp_path):
    filepath = tmp_path / 'historical_short.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.array([[1, 2, 3, 4]], dtype=np.int8),
        num_subcarriers=2,
        label='motion',
        chip='esp32',
    )

    packets = load_npz_as_packets(filepath)
    assert packets == []


def test_load_npz_csi_data_filters_non_ht20_rows(tmp_path):
    from tools.lib.csi_io import load_npz_csi_data

    filepath = tmp_path / 'mixed_phy_matrix.npz'
    rows = np.zeros((2, 128), dtype=np.int8)
    rows[0, :4] = np.array([1, 2, 3, 4], dtype=np.int8)
    rows[1, :4] = np.array([5, 6, 7, 8], dtype=np.int8)
    np.savez_compressed(
        filepath,
        csi_data=rows,
        num_subcarriers=64,
        phy_mode=np.array(['legacy', 'ht']),
        ltf_type=np.array(['unknown', 'ht-ltf']),
        channel_width=np.array(['20', '20']),
    )

    filtered = load_npz_csi_data(filepath)
    assert filtered.shape == (1, 128)
    np.testing.assert_array_equal(filtered[0][:4], np.array([5, 6, 7, 8], dtype=np.int8))

    raw = load_npz_csi_data(filepath, keep_all_phy=True)
    assert raw.shape == (2, 128)


def test_load_npz_csi_data_drops_legacy20_rows_even_for_original_esp32(tmp_path):
    from tools.lib.csi_io import load_npz_csi_data

    filepath = tmp_path / 'esp32_legacy_only_matrix.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.array(
            [
                [1, 2, 3, 4],
                [5, 6, 7, 8],
            ],
            dtype=np.int8,
        ),
        chip=np.array('esp32'),
        phy_mode=np.array(['legacy', 'legacy']),
        channel_width=np.array(['20', '20']),
    )

    filtered = load_npz_csi_data(filepath)
    assert filtered.shape == (0, 4)


def test_load_npz_csi_data_rejects_object_arrays(tmp_path):
    from tools.lib.csi_io import load_npz_csi_data

    filepath = tmp_path / 'malicious_csi_object_array.npz'
    np.savez_compressed(
        filepath,
        csi_data=np.array([object()], dtype=object),
        num_subcarriers=64,
        label='motion',
        chip='esp32',
    )

    with pytest.raises(ValueError, match="Unsafe NPZ dataset"):
        load_npz_csi_data(filepath)


def test_save_samples_by_device_splits_capture_window(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')

    receiver = CSIReceiver(bind_host='127.0.0.1')
    collector = CSICollector(label='motion', contributor='tester', bind_host='127.0.0.1')
    packets = [
        receiver._parse_packet(build_packet(seq_num=1, payload=[1, 2, 3, 4], device_id=0x10)),
        receiver._parse_packet(build_packet(seq_num=2, payload=[5, 6, 7, 8], device_id=0x20)),
        receiver._parse_packet(build_packet(seq_num=3, payload=[9, 10, 11, 12], device_id=0x10)),
    ]

    saved_paths = collector.save_samples_by_device(packets)

    assert len(saved_paths) == 2
    saved_names = {path.name for path in saved_paths}
    assert any('dev0000000000000010' in name for name in saved_names)
    assert any('dev0000000000000020' in name for name in saved_names)

    info = dataset_metadata.load_dataset_info()
    assert len(info['files']['motion']) == 2
    assert {entry['device_id'] for entry in info['files']['motion']} == {
        '0000000000000010',
        '0000000000000020',
    }


@pytest.mark.parametrize('label', ('../motion', '/tmp/motion', 'room/left', '.', 'motion label'))
def test_collector_rejects_labels_that_are_not_safe_directory_names(label):
    with pytest.raises(ValueError, match='dataset label'):
        CSICollector(label=label, bind_host='127.0.0.1')


def test_save_sample_removes_archive_when_catalog_update_fails(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')
    receiver = CSIReceiver(bind_host='127.0.0.1')
    collector = CSICollector(label='motion', contributor='tester', bind_host='127.0.0.1')
    packet = receiver._parse_packet(build_packet(seq_num=1, payload=[1, 2, 3, 4], device_id=0x10))
    monkeypatch.setattr(
        collector,
        '_update_dataset_info',
        lambda **_kwargs: (_ for _ in ()).throw(OSError('catalog unavailable')),
    )

    with pytest.raises(OSError, match='catalog unavailable'):
        collector.save_sample([packet])

    assert not list(data_dir.rglob('*.npz'))


def test_multi_device_save_rolls_back_archives_and_catalog(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')
    receiver = CSIReceiver(bind_host='127.0.0.1')
    collector = CSICollector(label='motion', contributor='tester', bind_host='127.0.0.1')
    packets = [
        receiver._parse_packet(build_packet(seq_num=1, payload=[1, 2, 3, 4], device_id=0x10)),
        receiver._parse_packet(build_packet(seq_num=2, payload=[5, 6, 7, 8], device_id=0x20)),
    ]
    original_info = dataset_metadata.load_dataset_info()
    dataset_metadata.save_dataset_info(original_info)
    real_save_sample = collector.save_sample
    save_calls = 0

    def fail_second_device(device_packets):
        nonlocal save_calls
        save_calls += 1
        if save_calls == 2:
            raise OSError('second device failed')
        return real_save_sample(device_packets)

    monkeypatch.setattr(collector, 'save_sample', fail_second_device)

    with pytest.raises(OSError, match='second device failed'):
        collector.save_samples_by_device(packets)

    assert not list(data_dir.rglob('*.npz'))
    assert dataset_metadata.load_dataset_info() == original_info


def test_save_samples_by_device_rejects_missing_device_id(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')

    receiver = CSIReceiver(bind_host='127.0.0.1')
    collector = CSICollector(label='motion', contributor='tester', bind_host='127.0.0.1')
    packet = receiver._parse_packet(build_packet(seq_num=7, payload=[1, 2, 3, 4], device_id=0))

    with pytest.raises(ValueError, match='missing device_id'):
        collector.save_samples_by_device([packet])


def test_save_sample_rejects_mixed_device_packets(tmp_path, monkeypatch):
    data_dir = tmp_path / 'data'
    monkeypatch.setattr(dataset_metadata, 'DATA_DIR', data_dir)
    monkeypatch.setattr(dataset_metadata, 'DATASET_INFO_FILE', data_dir / 'dataset_info.json')

    receiver = CSIReceiver(bind_host='127.0.0.1')
    collector = CSICollector(label='motion', contributor='tester', bind_host='127.0.0.1')
    packets = [
        receiver._parse_packet(build_packet(seq_num=1, payload=[1, 2, 3, 4], device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=2, payload=[5, 6, 7, 8], device_id=0x2)),
    ]

    with pytest.raises(ValueError, match='mixed-device sample'):
        collector.save_sample(packets)


def test_check_sequence_by_device_handles_interleaved_streams():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packets = [
        receiver._parse_packet(build_packet(seq_num=1, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=1, device_id=0x2)),
        receiver._parse_packet(build_packet(seq_num=2, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=2, device_id=0x2)),
        receiver._parse_packet(build_packet(seq_num=4, device_id=0x1)),
    ]

    accepted = [receiver.accept_packet(packet) for packet in packets]

    assert accepted == [True, True, True, True, True]
    assert receiver.dropped_count == 1
    assert receiver.out_of_order_count == 0


def test_receiver_rejects_backward_and_duplicate_sequences_without_poisoning_state():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    packets = [
        receiver._parse_packet(build_packet(seq_num=100, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=101, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=3, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=101, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=102, device_id=0x1)),
    ]

    accepted = [receiver.accept_packet(packet) for packet in packets]

    assert accepted == [True, True, False, False, True]
    assert receiver.dropped_count == 0
    assert receiver.out_of_order_count == 2


def test_receiver_sequence_filter_accepts_uint32_wrap_and_reset():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    wrapped = [
        receiver._parse_packet(build_packet(seq_num=0xFFFFFFFE, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=0xFFFFFFFF, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=0, device_id=0x1)),
        receiver._parse_packet(build_packet(seq_num=1, device_id=0x1)),
    ]

    assert all(receiver.accept_packet(packet) for packet in wrapped)
    assert receiver.dropped_count == 0
    assert receiver.out_of_order_count == 0

    receiver.reset_stats()
    restarted = receiver._parse_packet(build_packet(seq_num=0, device_id=0x1))
    assert receiver.accept_packet(restarted)


def test_summarize_ready_devices_waits_for_all_expected_devices():
    now = 100.0
    warmup_target = 10
    threshold = 1.0
    ready_stable_seconds = 3.0

    summary = CSICollector._summarize_ready_devices(
        {
            0x1: {'processed_packets': warmup_target, 'stable_since': now - 4.0, 'current_metric': 0.2},
        },
        expected_device_count=2,
        warmup_target=warmup_target,
        threshold=threshold,
        now=now,
        ready_stable_seconds=ready_stable_seconds,
    )
    assert summary['ready'] is False
    assert summary['status'] == 'DEVICES 1/2'

    summary = CSICollector._summarize_ready_devices(
        {
            0x1: {'processed_packets': warmup_target, 'stable_since': now - 4.0, 'current_metric': 0.2},
            0x2: {'processed_packets': warmup_target, 'stable_since': now - 1.0, 'current_metric': 0.3},
        },
        expected_device_count=2,
        warmup_target=warmup_target,
        threshold=threshold,
        now=now,
        ready_stable_seconds=ready_stable_seconds,
    )
    assert summary['ready'] is False
    assert summary['status'] == 'STABLE 2/2'
    assert summary['stable_elapsed'] == pytest.approx(1.0)

    summary = CSICollector._summarize_ready_devices(
        {
            0x1: {'processed_packets': warmup_target, 'stable_since': now - 4.0, 'current_metric': 0.2},
            0x2: {'processed_packets': warmup_target, 'stable_since': now - 3.5, 'current_metric': 0.3},
        },
        expected_device_count=2,
        warmup_target=warmup_target,
        threshold=threshold,
        now=now,
        ready_stable_seconds=ready_stable_seconds,
    )
    assert summary['ready'] is True
    assert summary['status'] == 'READY 2/2'


def test_format_ready_device_lines_includes_waiting_ip_and_device_details():
    now = 100.0
    ready_stable_seconds = 3.0
    lines = CSICollector._format_ready_device_lines(
        {
            0x1: {
                'processed_packets': 12,
                'stable_since': now - 3.5,
                'current_metric': 0.2,
                'current_pps': 121,
                'source_ip': '192.168.1.17',
                'chip': 'c6',
                'channel': 6,
                'rssi_dbm': -48,
            },
            0x2: {
                'processed_packets': 8,
                'stable_since': None,
                'current_metric': 0.0,
                'current_pps': 87,
                'source_ip': '192.168.1.24',
                'chip': 'c3',
                'channel': 11,
                'rssi_dbm': -63,
            },
        },
        expected_source_hosts=['192.168.1.17', '192.168.1.24', '192.168.1.29'],
        warmup_target=10,
        threshold=1.0,
        now=now,
        ready_stable_seconds=ready_stable_seconds,
    )

    assert any(
        'ip=192.168.1.29' in line
        and 'chip=?' in line
        and 'ch=--' in line
        and 'rssi=---' in line
        and 'pps=--' in line
        and '[------------------]' in line
        and 'WAITING' in line
        and 'stable' not in line
        for line in lines
    )
    assert any(
        'ip=192.168.1.17' in line
        and 'chip=C6' in line
        and 'ch=06' in line
        and 'rssi=-48' in line
        and 'pps=121' in line
        and '[####--------------]' in line
        and 'READY' in line
        and 'stable' not in line
        for line in lines
    )
    assert any(
        'ip=192.168.1.24' in line
        and 'chip=C3' in line
        and 'ch=11' in line
        and 'rssi=-63' in line
        and 'pps=87' in line
        and '[------------------]' in line
        and 'WARMUP 8/10' in line
        and 'stable' not in line
        for line in lines
    )


def test_summarize_ready_devices_reports_unstable_device_count():
    now = 100.0
    warmup_target = 10
    threshold = 1.0
    ready_stable_seconds = 3.0

    summary = CSICollector._summarize_ready_devices(
        {
            0x1: {'processed_packets': warmup_target, 'stable_since': now - 4.0, 'current_metric': 0.2},
            0x2: {'processed_packets': warmup_target, 'stable_since': None, 'current_metric': 2.5},
        },
        expected_device_count=2,
        warmup_target=warmup_target,
        threshold=threshold,
        now=now,
        ready_stable_seconds=ready_stable_seconds,
    )

    assert summary == {
        'ready': False,
        'status': 'UNSTABLE 1/2',
        'stable_elapsed': 0.0,
        'ready_count': 1,
        'observed_count': 2,
        'required_count': 2,
    }


def test_receiver_drop_rate_uses_expected_packet_total():
    receiver = CSIReceiver(bind_host='127.0.0.1')
    receiver.packet_count = 400
    receiver.dropped_count = 4

    stats = receiver.get_stats()

    assert stats['drop_rate'] == pytest.approx(4 / 404 * 100)


class _TTYBuffer(io.StringIO):
    def isatty(self):
        return True


def test_emit_ready_status_block_uses_ansi_when_inline_enabled():
    stream = _TTYBuffer()

    rendered = CSICollector._emit_ready_status_block(
        'summary 1',
        ['detail 1', 'detail 2'],
        stream=stream,
        inline=True,
    )
    assert rendered == 3
    assert stream.getvalue() == '\x1b[2Ksummary 1\n\x1b[2Kdetail 1\n\x1b[2Kdetail 2\n'

    stream.seek(0)
    stream.truncate(0)

    rendered = CSICollector._emit_ready_status_block(
        'summary 2',
        ['detail 3'],
        previous_line_count=3,
        stream=stream,
        inline=True,
    )
    assert rendered == 2
    assert stream.getvalue() == '\x1b[3F\x1b[2Ksummary 2\n\x1b[2Kdetail 3\n\x1b[2K\n'


def test_emit_ready_status_block_falls_back_to_plain_lines():
    stream = io.StringIO()

    rendered = CSICollector._emit_ready_status_block(
        'summary',
        ['detail'],
        previous_line_count=5,
        stream=stream,
        inline=False,
    )

    assert rendered == 2
    assert stream.getvalue() == 'summary\ndetail\n'
