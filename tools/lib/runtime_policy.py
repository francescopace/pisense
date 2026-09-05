# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
try:
    from collections.abc import Mapping as MappingABC
except ImportError:
    MappingABC = dict

"""
ESPectre - Reference Runtime Policy

Keeps detector evaluation cadence and motion hit filtering aligned with the
ESPHome/C++ runtime behavior.

Author: Francesco Pace <francesco.pace@gmail.com>
"""

from micro_espectre.config import (
    L1_DELTA_LAG_MAX,
    SEG_WINDOW_MAX,
    SEG_WINDOW_MIN,
    SEGMENTATION_WINDOW_SIZE_MS,
)
from micro_espectre.runtime_motion_policy import (
    UINT32_MODULUS as _UINT32_MODULUS,
    RuntimeMotionPolicy,  # noqa: F401 - re-exported for host-side policy consumers
    equivalent_packet_weight,  # noqa: F401 - re-exported for host-side policy consumers
    make_evaluation_cadence,  # noqa: F401 - re-exported for host-side policy consumers
)

DEFAULT_GAP_RESET_RATIO = 4.0
# Medians are refreshed on this stride instead of on every packet.
RATE_ESTIMATOR_REFRESH_STRIDE = 16
DEFAULT_GAP_RESET_SEQ_THRESHOLD = 3
DEFAULT_GAP_RESET_MIN_US = 250_000
# Rolling sample count used to estimate the effective packet cadence. One
# second of packets at the nominal rate is enough to be stable without making
# the estimate slow to follow a genuine rate change.
DEFAULT_RATE_ESTIMATOR_SAMPLES = 64
# Samples required before a rate-derived rule is allowed to fire. Until then
# the estimator has not seen enough of the stream to tell a slower cadence
# from packet loss, and guessing either way is worse than not acting.
DEFAULT_RATE_ESTIMATOR_WARMUP = 16
NOMINAL_PACKET_RATE_PPS = 100
PRODUCTION_L1_DELTA_LAG = 10
PRODUCTION_AUTOCORR_LAG = 1


def nominal_packet_interval_us(window_packets):
    """Return the nominal packet interval implied by one window per second."""
    packets = max(1, int(window_packets))
    return max(1, int(round(1_000_000.0 / float(packets))))


def duration_packet_count(duration_ms, interval_us):
    """Resolve an elapsed duration to packets at one measured cadence."""
    return max(
        1,
        int(round(max(1, int(duration_ms)) * 1000.0 / max(1, int(interval_us)))),
    )


def _median(values):
    """Return the median of a non-empty sequence without importing statistics."""
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


class PacketRateEstimator:
    """Track effective throughput and typical spacing from packet deltas."""

    def __init__(
        self,
        nominal_interval_us,
        samples=DEFAULT_RATE_ESTIMATOR_SAMPLES,
        warmup=DEFAULT_RATE_ESTIMATOR_WARMUP,
    ):
        self.nominal_interval_us = max(1, int(nominal_interval_us))
        self.samples = max(1, int(samples))
        self.warmup = max(1, int(warmup))
        self.reset()

    def reset(self):
        """Forget the observed cadence and fall back to the nominal interval."""
        self._deltas = []
        self._seq_steps = []
        self._interval_cache = None
        self._typical_cache = None
        self._seq_cache = None
        self._since_refresh = 0

    def observe_interval(self, delta_us):
        """Record one inter-packet interval."""
        if delta_us is None:
            return
        delta = int(delta_us)
        if delta <= 0:
            return
        self._deltas.append(delta)
        if len(self._deltas) > self.samples:
            del self._deltas[0]
        # Sorting on every packet would put an O(n log n) step in the hot path
        # for an estimate that cannot move quickly anyway, so the medians are
        # refreshed on a fixed stride and served from cache in between.
        self._since_refresh += 1
        if self._since_refresh >= RATE_ESTIMATOR_REFRESH_STRIDE:
            self._since_refresh = 0
            self._interval_cache = None
            self._typical_cache = None
            self._seq_cache = None

    def observe_sequence_step(self, seq_step):
        """Record one observed advance of the packet sequence counter."""
        if seq_step is None:
            return
        step = int(seq_step)
        if step <= 0:
            return
        self._seq_steps.append(step)
        if len(self._seq_steps) > self.samples:
            del self._seq_steps[0]

    @property
    def ready(self):
        """True once enough intervals have been seen to trust the estimate."""
        return len(self._deltas) >= self.warmup

    @property
    def interval_us(self):
        """Return the mean interval used to resolve temporal window samples."""
        if not self.ready:
            return self.nominal_interval_us
        if self._interval_cache is None:
            self._interval_cache = max(
                1,
                int(round(float(sum(self._deltas)) / len(self._deltas))),
            )
        return self._interval_cache

    @property
    def typical_interval_us(self):
        """Return median spacing for gap classification."""
        if not self.ready:
            return self.nominal_interval_us
        if self._typical_cache is None:
            self._typical_cache = max(1, int(round(_median(self._deltas))))
        return self._typical_cache

    @property
    def sequence_established(self):
        """True once the stream's own sequence step has been observed enough."""
        return len(self._seq_steps) >= self.warmup

    @property
    def sequence_step(self):
        """Return the cadence-normal sequence advance, or 1 until established.

        A stream that natively runs slower than the nominal rate advances its
        sequence counter by more than one per delivered packet. That is the
        stream's own step, not loss, so loss has to be measured against it.
        """
        if not self.sequence_established:
            return 1
        if self._seq_cache is None:
            self._seq_cache = max(1, int(round(_median(self._seq_steps))))
        return self._seq_cache


def derive_detector_timing(interval_us, window_size_ms=SEGMENTATION_WINDOW_SIZE_MS):
    """Resolve the configured time window at one measured packet cadence."""
    interval = max(1, int(interval_us))
    duration_us = max(1, int(window_size_ms)) * 1000
    derived_window = max(1, (duration_us + interval - 1) // interval)
    window_packets = min(max(derived_window, SEG_WINDOW_MIN), SEG_WINDOW_MAX)
    return {
        "interval_us": interval,
        "window_packets": window_packets,
        "lag": min(PRODUCTION_L1_DELTA_LAG, L1_DELTA_LAG_MAX, max(1, window_packets // 2)),
        "autocorr_lag": min(PRODUCTION_AUTOCORR_LAG, max(1, window_packets // 2)),
    }


def _packet_field(packet, key):
    """Return one field from a dict-like packet or packet object."""
    if isinstance(packet, MappingABC):
        return _mapping_field(packet, key)
    return getattr(packet, key, None)


def _mapping_field(packet, key):
    """Return one field from a packet already known to be dict-like."""
    value = packet.get(key)
    if value is None and key == "seq_num":
        return packet.get("stream_seq_num")
    return value


def _packet_fields(packet, keys, out):
    """Return several fields from one packet, checking its shape once.

    The abstract base class check does not depend on the key, and it is the
    expensive part: `Mapping.__instancecheck__` walks the ABC registry, while
    the lookup that follows is a plain dict hit. Every packet reads three
    fields, so hoisting the check out of the per-field call removes two thirds
    of that cost from the timing path.
    """
    if isinstance(packet, MappingABC):
        for i in range(len(keys)):
            out[i] = _mapping_field(packet, keys[i])
    else:
        for i in range(len(keys)):
            out[i] = getattr(packet, keys[i], None)
    return out


_TIMING_PACKET_FIELDS = ("seq_num", "device_ticks_us", "wifi_rx_ts_us")


def _unsigned_delta(current, previous, modulus):
    """Unsigned modular delta between two monotonic counters."""
    return (int(current) - int(previous)) % int(modulus)


class PacketTimingTracker:
    """Track packet timing and gap contamination from stream metadata."""

    def __init__(
        self,
        nominal_packet_interval_us,
        *,
        gap_reset_ratio=DEFAULT_GAP_RESET_RATIO,
        sequence_gap_reset=DEFAULT_GAP_RESET_SEQ_THRESHOLD,
        gap_reset_min_us=DEFAULT_GAP_RESET_MIN_US,
        rate_estimator=None,
    ):
        self.nominal_packet_interval_us = max(1, int(nominal_packet_interval_us))
        self.gap_reset_ratio = float(gap_reset_ratio)
        self.sequence_gap_reset = max(1, int(sequence_gap_reset))
        self.gap_reset_min_us = max(1, int(gap_reset_min_us))
        self.rate = (
            PacketRateEstimator(self.nominal_packet_interval_us)
            if rate_estimator is None
            else rate_estimator
        )
        self._field_values = [None] * len(_TIMING_PACKET_FIELDS)
        self.reset()

    def reset(self):
        """Forget prior packet positions, but keep the learned stream cadence.

        A hole says nothing about how fast the stream runs, and discarding the
        cadence here is self-defeating: the next packet would be judged against
        a step of one again, read as loss, and reset the tracker once more.
        """
        self._last_seq_num = None
        self._last_device_ticks_us = None
        self._last_wifi_rx_ts_us = None

    def _gap_threshold_us(self):
        """Return the elapsed-time hole threshold for the observed cadence.

        Scaling with the measured interval is what keeps a genuinely slower
        stream from reading as one continuous hole; the absolute floor keeps a
        fast stream from calling ordinary jitter a gap.
        """
        return max(
            self.gap_reset_min_us,
            int(round(self.rate.typical_interval_us * self.gap_reset_ratio)),
        )

    def observe_packet(self, packet):
        """Return timing metadata for one packet."""
        seq_num, device_ticks_us, wifi_rx_ts_us = _packet_fields(
            packet, _TIMING_PACKET_FIELDS, self._field_values
        )

        # Loss is measured against the cadence the stream has actually shown,
        # not against a hardcoded step of one. A capture that natively delivers
        # every fourth packet advances the counter by four with nothing missing,
        # and treating that as loss contaminates every packet in the stream.
        seq_step = None
        missing_seq = 0
        if seq_num is not None and self._last_seq_num is not None:
            seq_step = (int(seq_num) - int(self._last_seq_num)) & 0xFFFFFFFF
            if not 0 < seq_step < 0x80000000:
                seq_step = None
            else:
                # Record before judging, so the packet that completes the warmup
                # is measured against the step it just helped establish rather
                # than against the placeholder step of one.
                self.rate.observe_sequence_step(seq_step)
                missing_seq = max(0, int(seq_step) - self.rate.sequence_step)

        delta_us = None
        source = "missing"
        if (
            device_ticks_us is not None
            and self._last_device_ticks_us is not None
            and int(device_ticks_us) >= int(self._last_device_ticks_us)
        ):
            delta_us = int(device_ticks_us) - int(self._last_device_ticks_us)
            source = "device_ticks_us"
        elif (
            wifi_rx_ts_us is not None
            and self._last_wifi_rx_ts_us is not None
        ):
            candidate = _unsigned_delta(
                int(wifi_rx_ts_us),
                int(self._last_wifi_rx_ts_us),
                _UINT32_MODULUS,
            )
            if 0 < candidate < (_UINT32_MODULUS // 2):
                delta_us = int(candidate)
                source = "wifi_rx_ts_us"

        # As with the sequence step, the interval is recorded before the packet
        # is judged. The median is what makes this safe: an occasional hole
        # contributes one large sample and is rejected, while gating the
        # estimate on contamination would deadlock, because a slower stream
        # reads as contaminated until its own cadence is established and can
        # never establish it while being discarded.
        if delta_us is not None:
            self.rate.observe_interval(delta_us)

        contaminated = (
            (
                seq_num is not None
                and self._last_seq_num is not None
                and self.rate.sequence_established
                and missing_seq >= self.sequence_gap_reset
            )
            or (
                delta_us is not None
                and delta_us >= self._gap_threshold_us()
                and (
                    self._last_device_ticks_us is not None
                    or self._last_wifi_rx_ts_us is not None
                    or self._last_seq_num is not None
                )
            )
        )

        if seq_num is not None:
            self._last_seq_num = int(seq_num)
        if device_ticks_us is not None:
            self._last_device_ticks_us = int(device_ticks_us)
        if wifi_rx_ts_us is not None:
            self._last_wifi_rx_ts_us = int(wifi_rx_ts_us)

        return {
            "delta_us": 0 if delta_us is None else int(delta_us),
            "coverage_us": 0 if contaminated or delta_us is None else int(delta_us),
            "missing_seq": int(missing_seq),
            "source": source,
            "contaminated": bool(contaminated),
        }
