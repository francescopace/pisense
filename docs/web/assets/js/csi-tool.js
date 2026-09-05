/*
 * ESPectre - CSI tool
 *
 * Part of the website application shell.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

'use strict';

    /* =========================================================== raw CSI */

    const RAW_CSI_V8_HEADER_BYTES = 64;
    const RAW_CSI_VISUAL_HISTORY = 720;
    const RAW_CSI_AMPLITUDE_SURFACE_PACKETS = 96;
    const RAW_CSI_PHASE_HISTORY = 72;
    const RAW_CSI_IQ_WINDOW_US = 1000000;
    const RAW_CSI_IQ_EXTENT = 128;
    const RAW_CSI_VISUAL_STEP_US = 33333;
    const RAW_CSI_RENDER_INTERVAL_MS = 1000 / 30;
    const RAW_CSI_AMPLITUDE_SCALE_HEADROOM = 1.5;
    const RAW_CSI_AMPLITUDE_SCALE_STEP = 25;
    const RAW_CSI_CHANNEL_PROFILE_DEVIATION_GAIN = 5;
    const RAW_CSI_RELATIVE_PHASE_TRAIL_GAIN = 5;
    const RAW_CSI_SELECTED_SUBCARRIERS = Object.freeze([4, 8, 13, 18, 23, 28, 36, 41, 46, 51, 56, 60]);
    const RAW_CSI_LIVE_SUBCARRIERS = Object.freeze(
        Array.from({ length: 57 }, (_unused, index) => index + 4).filter((index) => index !== 32));
    const RAW_CSI_VISUALIZATIONS = Object.freeze({
        'subcarrier-amplitudes': Object.freeze({
            title: 'Subcarrier amplitudes',
            description: 'Each line tracks one subcarrier across the rolling time window. Colors distinguish lower, middle, and higher frequencies.',
            badge: 'LIVE',
            ariaLabel: 'Rolling CSI amplitude traces for each active subcarrier'
        }),
        'csi-amplitude-surface': Object.freeze({
            title: 'CSI amplitude surface',
            description: 'Shows amplitude across subcarriers and recent packets as a three-dimensional surface.',
            badge: 'LIVE',
            ariaLabel: 'Three-dimensional CSI amplitude surface by subcarrier and packet'
        }),
        'channel-profile-deviation': Object.freeze({
            title: 'Channel profile deviation',
            description: 'Compares the current channel profile with its recent baseline. Deviations are amplified 5× for visibility.',
            badge: 'LIVE',
            ariaLabel: 'Current normalized CSI channel profile and its recent baseline'
        }),
        'iq-constellation': Object.freeze({
            title: 'I/Q constellation',
            description: 'Plots raw I/Q samples collected during the last second.',
            badge: 'LIVE',
            ariaLabel: 'One-second I and Q constellation for all active CSI subcarriers'
        }),
        'relative-phase-trails': Object.freeze({
            title: 'Relative phase trails',
            description: 'Relative I/Q phase after removing common packet rotation and the linear phase ramp. Trail spread is amplified 5×.',
            badge: 'EXPERIMENTAL',
            ariaLabel: 'Experimental relative CSI phase constellation trails'
        })
    });
    const rawCsi = {
        sessionClient: null,
        controller: null,
        demoTimer: null,
        metricsTimer: null,
        demoFresh: 0,
        state: 'idle',
        generation: 0,
        stopPromise: null,
        parser: null,
        analyticsStartedAt: 0,
        analyticsReady: false,
        analyticsReadyAt: 0,
        analyticsSuccessTracked: false,
        visualization: 'subcarrier-amplitudes',
        amplitudePackets: [],
        amplitudePacketSequences: [],
        amplitudePacketSequence: 0,
        amplitudeFrames: [],
        profiles: [],
        deltas: [],
        timestampsUs: [],
        phaseHistory: [],
        iqHistory: [],
        iqTimestampsUs: [],
        baseline: null,
        latestProfile: null,
        latestDelta: null,
        amplitudeMaximum: 0,
        lastCaptureTicksUs: 0,
        lastVisualTicksUs: 0,
        lastRenderAt: 0,
        renderFrame: 0,
        metricsWindowStartedAt: 0,
        metricsReceived: 0,
        metricsRssiSum: 0,
        metricsRssiSamples: 0,
        metricsSnrSum: 0,
        metricsSnrSamples: 0,
        metricsCaptureIntervalMsSum: 0,
        metricsCaptureIntervalSamples: 0,
        metricsLastCaptureTicksUs: 0,
        metricsFresh: 0,
        metricsDropped: 0,
        metricsBackpressure: 0,
        resizeObserver: null
    };

    function rawCsiStatus(message, error = false) {
        const status = $('.js-raw-csi-status');
        if (!status) return;
        status.textContent = message;
        status.hidden = !message;
        status.classList.toggle('is-error', error);
    }

    function rawCsiDirectReady() {
        return conn.mode === 'direct' && conn.status === 'connected' && Boolean(directClient?.connected);
    }

    function rawCsiAnalyticsParams() {
        return {
            ...connectionParams(),
            transport: connectionTransport(),
            input_mode: connectionInputMode()
        };
    }

    function rawCsiBeginTracking() {
        rawCsi.analyticsStartedAt = Date.now();
        rawCsi.analyticsReady = false;
        rawCsi.analyticsReadyAt = 0;
        rawCsi.analyticsSuccessTracked = false;
        track('csi_stream', { ...rawCsiAnalyticsParams(), result: 'attempt' });
    }

    function rawCsiMarkReady() {
        if (!rawCsi.analyticsStartedAt) return;
        if (!rawCsi.analyticsReady) {
            rawCsi.analyticsReady = true;
            rawCsi.analyticsReadyAt = Date.now();
        }
        markToolReady('csi');
        if (!rawCsi.analyticsSuccessTracked) {
            rawCsi.analyticsSuccessTracked = track('csi_stream', {
                ...rawCsiAnalyticsParams(),
                result: 'success',
                latency_ms: Math.max(0, rawCsi.analyticsReadyAt - rawCsi.analyticsStartedAt)
            });
        }
    }

    function rawCsiFinishTracking(result, error, reason) {
        if (!rawCsi.analyticsStartedAt) return;
        const durationSeconds = Math.max(0, Math.round(
            (Date.now() - rawCsi.analyticsStartedAt) / 1000
        ));
        track('csi_stream', {
            ...rawCsiAnalyticsParams(),
            result,
            duration_seconds: durationSeconds,
            ...(error ? { error_type: errorType(error) } : {}),
            ...(reason ? { reason } : {})
        });
        rawCsi.analyticsStartedAt = 0;
        rawCsi.analyticsReady = false;
        rawCsi.analyticsReadyAt = 0;
        rawCsi.analyticsSuccessTracked = false;
    }

    function rawCsiSetAvailable(available) {
        const unavailable = $('.js-raw-csi-unavailable');
        const workspace = $('.js-raw-csi-workspace');
        if (unavailable) unavailable.hidden = available;
        if (workspace) workspace.hidden = !available;
    }

    function rawCsiUseConnection() {
        const onboarding = $('.js-raw-csi-onboarding');
        const unavailable = $('.js-raw-csi-unavailable');
        const workspace = $('.js-raw-csi-workspace');
        const externalHint = $('.js-raw-csi-external-hint');
        if (conn.status !== 'connected' || !['direct', 'demo'].includes(conn.mode)) {
            if (externalHint) externalHint.hidden = true;
            if (onboarding) onboarding.hidden = false;
            if (unavailable) unavailable.hidden = true;
            if (workspace) workspace.hidden = true;
            return false;
        }
        if (onboarding) onboarding.hidden = true;
        if (conn.mode === 'demo') {
            if (externalHint) externalHint.hidden = true;
            rawCsiSetAvailable(true);
            rawCsiStatus('Demo ready. Start the simulated signal stream when you are ready.');
            return true;
        }
        const rawCapability = directClient?.capabilities?.csi;
        const available = directClient?.capabilities?.features?.csi === true
            && rawCapability?.protocol_version === 1
            && rawCapability?.marker === '👻';
        rawCsiSetAvailable(available);
        if (externalHint) externalHint.hidden = !available || conn.csiTrafficMode !== 'external';
        if (available) {
            rawCsiStatus(conn.csiTrafficMode === 'external'
                ? 'Connected. This device is waiting for its external Wi-Fi traffic source before data appears.'
                : 'Connected. Start the temporary signal stream when you are ready. Nothing is uploaded or stored.');
        }
        return available;
    }

    function rawCsiSetState(state) {
        rawCsi.state = state;
        const toggle = $('.js-raw-csi-toggle');
        if (!toggle) return;
        const idle = state === 'idle';
        toggle.textContent = state === 'stopping' ? 'Stopping…' : idle ? 'Start' : 'Stop';
        toggle.disabled = state === 'stopping';
        toggle.classList.toggle('btn-primary', idle);
        toggle.classList.toggle('btn-secondary', !idle);
    }

    function rawCsiCounter(selector, value) {
        const element = $(selector);
        if (element) element.textContent = typeof value === 'bigint'
            ? value.toLocaleString('en-US') : Number(value).toLocaleString('en-US');
    }

    function rawCsiLabel(selector, value) {
        const element = $(selector);
        if (element) element.textContent = value || '—';
    }

    function rawCsiCollectSignalSample(rssi, noiseFloor) {
        rawCsi.metricsRssiSum += rssi;
        rawCsi.metricsRssiSamples += 1;
        if (noiseFloor < 0) {
            rawCsi.metricsSnrSum += rssi - noiseFloor;
            rawCsi.metricsSnrSamples += 1;
        }
    }

    function rawCsiCollectRadioMetrics(view) {
        const rssi = view.getInt8(43);
        const noiseFloor = view.getInt8(44);
        rawCsiCollectSignalSample(rssi, noiseFloor);
    }

    function rawCsiCollectCaptureInterval(captureTicksUs) {
        const previousTicksUs = rawCsi.metricsLastCaptureTicksUs;
        const intervalMs = previousTicksUs > 0 && captureTicksUs > previousTicksUs
            ? (captureTicksUs - previousTicksUs) / 1000 : 0;
        if (intervalMs > 0) {
            rawCsi.metricsCaptureIntervalMsSum += intervalMs;
            rawCsi.metricsCaptureIntervalSamples += 1;
        }
        rawCsi.metricsLastCaptureTicksUs = captureTicksUs;
    }

    function rawCsiResetMetricWindow(now = performance.now()) {
        rawCsi.metricsWindowStartedAt = now;
        rawCsi.metricsReceived = 0;
        rawCsi.metricsRssiSum = 0;
        rawCsi.metricsRssiSamples = 0;
        rawCsi.metricsSnrSum = 0;
        rawCsi.metricsSnrSamples = 0;
        rawCsi.metricsCaptureIntervalMsSum = 0;
        rawCsi.metricsCaptureIntervalSamples = 0;
    }

    function rawCsiFlushMetrics() {
        const now = performance.now();
        const elapsedMs = Math.max(1, now - rawCsi.metricsWindowStartedAt);
        rawCsiCounter('.js-raw-pps', Math.round(rawCsi.metricsReceived * 1000 / elapsedMs));
        rawCsiCounter('.js-raw-fresh', rawCsi.metricsFresh);
        rawCsiCounter('.js-raw-dropped', rawCsi.metricsDropped);
        rawCsiCounter('.js-raw-backpressure', rawCsi.metricsBackpressure);
        rawCsiLabel('.js-raw-capture-interval', rawCsi.metricsCaptureIntervalSamples > 0
            ? (rawCsi.metricsCaptureIntervalMsSum / rawCsi.metricsCaptureIntervalSamples).toFixed(1)
            : '—');
        rawCsiLabel('.js-raw-rssi', rawCsi.metricsRssiSamples > 0
            ? (rawCsi.metricsRssiSum / rawCsi.metricsRssiSamples).toFixed(1)
            : '—');
        rawCsiLabel('.js-raw-snr', rawCsi.metricsSnrSamples > 0
            ? (rawCsi.metricsSnrSum / rawCsi.metricsSnrSamples).toFixed(1)
            : '—');
        rawCsiResetMetricWindow(now);
    }

    function rawCsiStartMetrics() {
        clearInterval(rawCsi.metricsTimer);
        rawCsi.metricsFresh = 0;
        rawCsi.metricsDropped = 0;
        rawCsi.metricsBackpressure = 0;
        rawCsi.metricsLastCaptureTicksUs = 0;
        rawCsiResetMetricWindow();
        ['.js-raw-pps', '.js-raw-fresh', '.js-raw-dropped', '.js-raw-backpressure']
            .forEach((selector) => rawCsiCounter(selector, 0));
        ['.js-raw-capture-interval', '.js-raw-rssi', '.js-raw-snr']
            .forEach((selector) => rawCsiLabel(selector, '—'));
        rawCsi.metricsTimer = setInterval(rawCsiFlushMetrics, 1000);
    }

    function rawCsiStopMetrics() {
        clearInterval(rawCsi.metricsTimer);
        rawCsi.metricsTimer = null;
        rawCsi.metricsReceived = 0;
        rawCsiCounter('.js-raw-pps', 0);
    }

    function rawCsiPushBounded(collection, value, limit) {
        collection.push(value);
        if (collection.length > limit) collection.shift();
    }

    function rawCsiLiveSubcarriers(length) {
        if (length === 64) return RAW_CSI_LIVE_SUBCARRIERS;
        return Array.from({ length }, (_unused, index) => index);
    }

    function rawCsiVisibleSubcarriers(amplitudes) {
        if (amplitudes.length === 64) return RAW_CSI_LIVE_SUBCARRIERS;
        let first = 0;
        while (first < amplitudes.length && amplitudes[first] <= 0) first += 1;
        let last = amplitudes.length - 1;
        while (last >= first && amplitudes[last] <= 0) last -= 1;
        if (last < first) return [];
        return Array.from({ length: last - first + 1 }, (_unused, index) => first + index)
            .filter((index) => amplitudes[index] > 0);
    }

    function rawCsiUpdateAmplitudeMaximum(amplitudes) {
        if (rawCsi.amplitudeMaximum > 0) return;
        let peak = 0;
        rawCsiVisibleSubcarriers(amplitudes).forEach((subcarrier) => {
            peak = Math.max(peak, amplitudes[subcarrier] || 0);
        });
        if (peak <= 0) return;
        rawCsi.amplitudeMaximum = Math.max(
            RAW_CSI_AMPLITUDE_SCALE_STEP,
            Math.ceil(
                peak * RAW_CSI_AMPLITUDE_SCALE_HEADROOM / RAW_CSI_AMPLITUDE_SCALE_STEP
            ) * RAW_CSI_AMPLITUDE_SCALE_STEP
        );
    }

    function rawCsiResetVisualization() {
        rawCsi.amplitudePackets.length = 0;
        rawCsi.amplitudePacketSequences.length = 0;
        rawCsi.amplitudePacketSequence = 0;
        rawCsi.amplitudeFrames.length = 0;
        rawCsi.profiles.length = 0;
        rawCsi.deltas.length = 0;
        rawCsi.timestampsUs.length = 0;
        rawCsi.phaseHistory.length = 0;
        rawCsi.iqHistory.length = 0;
        rawCsi.iqTimestampsUs.length = 0;
        rawCsi.baseline = null;
        rawCsi.latestProfile = null;
        rawCsi.latestDelta = null;
        rawCsi.amplitudeMaximum = 0;
        rawCsi.lastCaptureTicksUs = 0;
        rawCsi.lastVisualTicksUs = 0;
        rawCsi.lastRenderAt = 0;
        rawCsiScheduleRender();
    }

    function rawCsiNormalizeProfile(amplitudes) {
        const profile = new Float32Array(amplitudes.length);
        const liveSubcarriers = rawCsiLiveSubcarriers(amplitudes.length);
        let sum = 0;
        let count = 0;
        liveSubcarriers.forEach((index) => {
            if (amplitudes[index] <= 0) return;
            sum += amplitudes[index];
            count += 1;
        });
        const mean = count ? sum / count : 1;
        liveSubcarriers.forEach((index) => {
            profile[index] = amplitudes[index] / Math.max(mean, 1e-6);
        });
        return profile;
    }

    function rawCsiUpdateBaseline(profile, captureTicksUs) {
        if (!rawCsi.baseline || rawCsi.baseline.length !== profile.length) {
            rawCsi.baseline = profile.slice();
            rawCsi.lastCaptureTicksUs = captureTicksUs;
            return new Float32Array(profile.length);
        }
        const elapsedUs = rawCsi.lastCaptureTicksUs > 0 && captureTicksUs > rawCsi.lastCaptureTicksUs
            ? captureTicksUs - rawCsi.lastCaptureTicksUs : RAW_CSI_VISUAL_STEP_US;
        const alpha = Math.max(0.0002, Math.min(0.25, 1 - Math.exp(-elapsedUs / 5000000)));
        const delta = new Float32Array(profile.length);
        const liveSubcarriers = rawCsiLiveSubcarriers(profile.length);
        liveSubcarriers.forEach((index) => {
            const baseline = rawCsi.baseline[index];
            delta[index] = Math.log((profile[index] + 0.05) / (baseline + 0.05));
            rawCsi.baseline[index] = baseline + alpha * (profile[index] - baseline);
        });
        rawCsi.lastCaptureTicksUs = captureTicksUs;
        return delta;
    }

    function rawCsiRelativePhase(iValues, qValues, profile) {
        if (profile.length !== 64) return null;
        const residualReal = new Float32Array(profile.length - 1);
        const residualImag = new Float32Array(profile.length - 1);
        let commonReal = 0;
        let commonImag = 0;
        for (let left = 4; left < 60; left += 1) {
            if (left === 31 || left === 32) continue;
            const right = left + 1;
            const real = iValues[left] * iValues[right] + qValues[left] * qValues[right];
            const imag = qValues[left] * iValues[right] - iValues[left] * qValues[right];
            const magnitude = Math.hypot(real, imag);
            if (magnitude <= 1e-6) continue;
            residualReal[left] = real / magnitude;
            residualImag[left] = imag / magnitude;
            commonReal += residualReal[left];
            commonImag += residualImag[left];
        }
        const commonMagnitude = Math.hypot(commonReal, commonImag);
        if (commonMagnitude <= 1e-6) return null;
        const commonUnitReal = commonReal / commonMagnitude;
        const commonUnitImag = commonImag / commonMagnitude;
        const result = new Float32Array(RAW_CSI_SELECTED_SUBCARRIERS.length * 2);
        RAW_CSI_SELECTED_SUBCARRIERS.forEach((subcarrier, index) => {
            const left = subcarrier === 60 ? 59 : subcarrier;
            const real = residualReal[left];
            const imag = residualImag[left];
            const relativeReal = real * commonUnitReal + imag * commonUnitImag;
            const relativeImag = imag * commonUnitReal - real * commonUnitImag;
            const radius = 0.32 + 0.68 * Math.min(1, profile[subcarrier] / 2);
            result[index * 2] = relativeReal * radius;
            result[index * 2 + 1] = relativeImag * radius;
        });
        return result;
    }

    function rawCsiIngestVisualFrame(amplitudes, iValues, qValues, captureTicksUs) {
        rawCsiUpdateAmplitudeMaximum(amplitudes);
        rawCsi.amplitudePacketSequence += 1;
        rawCsiPushBounded(
            rawCsi.amplitudePackets, amplitudes.slice(), RAW_CSI_AMPLITUDE_SURFACE_PACKETS);
        rawCsiPushBounded(
            rawCsi.amplitudePacketSequences,
            rawCsi.amplitudePacketSequence,
            RAW_CSI_AMPLITUDE_SURFACE_PACKETS
        );
        const profile = rawCsiNormalizeProfile(amplitudes);
        const delta = rawCsiUpdateBaseline(profile, captureTicksUs);
        rawCsi.latestProfile = profile;
        rawCsi.latestDelta = delta;
        if (rawCsi.lastVisualTicksUs > 0
                && captureTicksUs - rawCsi.lastVisualTicksUs < RAW_CSI_VISUAL_STEP_US) {
            rawCsiScheduleRender();
            return;
        }
        rawCsiPushBounded(rawCsi.profiles, profile, RAW_CSI_VISUAL_HISTORY);
        rawCsiPushBounded(rawCsi.amplitudeFrames, amplitudes.slice(), RAW_CSI_VISUAL_HISTORY);
        rawCsiPushBounded(rawCsi.deltas, delta, RAW_CSI_VISUAL_HISTORY);
        rawCsiPushBounded(rawCsi.timestampsUs, captureTicksUs, RAW_CSI_VISUAL_HISTORY);
        const relativePhase = rawCsiRelativePhase(iValues, qValues, profile);
        if (relativePhase) {
            rawCsiPushBounded(rawCsi.phaseHistory, relativePhase, RAW_CSI_PHASE_HISTORY);
        }
        const iq = new Float32Array(iValues.length * 2);
        iValues.forEach((value, index) => {
            iq[index * 2] = value;
            iq[index * 2 + 1] = qValues[index];
        });
        rawCsi.iqHistory.push(iq);
        rawCsi.iqTimestampsUs.push(captureTicksUs);
        while (rawCsi.iqHistory.length > 1
                && captureTicksUs - rawCsi.iqTimestampsUs[0] > RAW_CSI_IQ_WINDOW_US) {
            rawCsi.iqHistory.shift();
            rawCsi.iqTimestampsUs.shift();
        }
        rawCsi.lastVisualTicksUs = captureTicksUs;
        rawCsiScheduleRender();
    }

    function rawCsiCanvasContext() {
        const canvas = $('.js-raw-visualization');
        const context = canvas?.getContext('2d');
        return canvas && context ? { canvas, context } : null;
    }

    function rawCsiResizeVisualization() {
        const canvas = $('.js-raw-visualization');
        const stage = canvas?.closest('.raw-csi-visualization-stage');
        const width = Math.round(stage?.clientWidth || 0);
        if (!canvas || width < 100) return;
        const height = window.matchMedia('(max-width: 620px)').matches
            ? 260 : Math.min(420, Math.round(width * 420 / 960));
        if (canvas.width === width && canvas.height === height) return;
        canvas.width = width;
        canvas.height = height;
        canvas.style.height = `${height}px`;
        rawCsiScheduleRender();
    }

    function rawCsiClearCanvas(context, canvas) {
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.fillStyle = '#05070d';
        context.fillRect(0, 0, canvas.width, canvas.height);
    }

    function rawCsiDrawEmpty(context, canvas, message = 'Start the stream to reveal the channel.') {
        rawCsiClearCanvas(context, canvas);
        context.fillStyle = 'rgba(255, 255, 255, .48)';
        context.font = '500 15px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(message, canvas.width / 2, canvas.height / 2);
    }

    function rawCsiMotionColor(value, alpha = 1) {
        const intensity = Math.min(1, Math.abs(value));
        const base = [12, 10, 31];
        const target = value < 0 ? [54, 215, 255] : [255, 91, 118];
        const channels = base.map((channel, index) => Math.round(
            channel + (target[index] - channel) * intensity));
        return `rgba(${channels[0]}, ${channels[1]}, ${channels[2]}, ${alpha})`;
    }

    function rawCsiAmplitudeSurfaceColor(value, minimum, maximum) {
        const span = Math.max(1e-6, maximum - minimum);
        const intensity = Math.pow(Math.max(0, Math.min(1, (value - minimum) / span)), .72);
        const hue = Math.round(270 - intensity * 220);
        const lightness = Math.round(48 + intensity * 10);
        return `hsl(${hue} 94% ${lightness}%)`;
    }

    function rawCsiAmplitudeColor(position, count, alpha = 1) {
        const band = Math.min(2, Math.floor(position * 3 / Math.max(1, count)));
        const colors = [[255, 66, 92], [52, 211, 109], [30, 211, 238]];
        const [red, green, blue] = colors[band];
        return `rgba(${red}, ${green}, ${blue}, ${alpha})`;
    }

    function rawCsiDrawSubcarrierAmplitudes(context, canvas) {
        if (!rawCsi.amplitudeFrames.length) {
            rawCsiDrawEmpty(context, canvas);
            return;
        }
        rawCsiClearCanvas(context, canvas);
        const frames = rawCsi.amplitudeFrames;
        const latest = frames[frames.length - 1];
        const visibleSubcarriers = rawCsiVisibleSubcarriers(latest);
        const activeCount = visibleSubcarriers.length;
        const left = 54;
        const right = canvas.width - 22;
        const top = 56;
        const bottom = canvas.height - 42;
        const width = right - left;
        const height = bottom - top;
        const maximum = Math.max(RAW_CSI_AMPLITUDE_SCALE_STEP, rawCsi.amplitudeMaximum);
        const firstHistorySlot = RAW_CSI_VISUAL_HISTORY - frames.length;
        const xForFrame = (index) => left
            + (firstHistorySlot + index) * width / (RAW_CSI_VISUAL_HISTORY - 1);
        const yForAmplitude = (value) => bottom
            - Math.min(maximum, Math.max(0, value)) * height / maximum;

        context.strokeStyle = 'rgba(255, 255, 255, .13)';
        context.lineWidth = 1;
        for (let tick = 0; tick <= 4; tick += 1) {
            const value = maximum * tick / 4;
            const y = yForAmplitude(value);
            context.beginPath();
            context.moveTo(left, y);
            context.lineTo(right, y);
            context.stroke();
            context.fillStyle = 'rgba(255, 255, 255, .48)';
            context.font = '11px ui-monospace, "SFMono-Regular", Consolas, monospace';
            context.textAlign = 'right';
            context.textBaseline = 'middle';
            context.fillText(String(Math.round(value)), left - 9, y);
        }

        visibleSubcarriers.forEach((subcarrier, position) => {
            context.strokeStyle = rawCsiAmplitudeColor(position, visibleSubcarriers.length, 0.26);
            context.lineWidth = 0.9;
            context.beginPath();
            let drawing = false;
            frames.forEach((frame, frameIndex) => {
                const amplitude = frame[subcarrier];
                if (!Number.isFinite(amplitude) || amplitude <= 0) {
                    drawing = false;
                    return;
                }
                const x = xForFrame(frameIndex);
                const y = yForAmplitude(amplitude);
                if (!drawing) context.moveTo(x, y);
                else context.lineTo(x, y);
                drawing = true;
            });
            context.stroke();
        });

        context.strokeStyle = 'rgba(255, 255, 255, .32)';
        context.strokeRect(left + 0.5, top + 0.5, width - 1, height - 1);
        context.fillStyle = 'rgba(255, 255, 255, .55)';
        context.font = '12px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'left';
        context.textBaseline = 'alphabetic';
        context.fillStyle = rawCsiAmplitudeColor(0, 3, .82);
        context.fillText('LOW', left, 28);
        context.fillStyle = rawCsiAmplitudeColor(1, 3, .82);
        context.fillText('MID', left + 42, 28);
        context.fillStyle = rawCsiAmplitudeColor(2, 3, .82);
        context.fillText('HIGH', left + 84, 28);
        context.textAlign = 'right';
        context.fillStyle = 'rgba(255, 255, 255, .84)';
        context.font = '600 16px ui-sans-serif, system-ui, sans-serif';
        const countLabel = canvas.width < 540
            ? `${activeCount} SUBCARRIERS`
            : `${activeCount} SUBCARRIER AMPLITUDES`;
        context.fillText(countLabel, right, 29);
        context.fillStyle = 'rgba(255, 255, 255, .52)';
        context.font = '12px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'left';
        context.fillText('PAST', left, canvas.height - 16);
        context.textAlign = 'center';
        context.fillText('RECENT TIME →', (left + right) / 2, canvas.height - 16);
        context.textAlign = 'right';
        context.fillText('NOW', right, canvas.height - 16);
    }

    function rawCsiDrawAmplitudeSurface(context, canvas) {
        if (rawCsi.amplitudePackets.length < 2) {
            rawCsiDrawEmpty(context, canvas);
            return;
        }
        rawCsiClearCanvas(context, canvas);
        context.fillStyle = '#fbfcfe';
        context.fillRect(0, 0, canvas.width, canvas.height);
        const packets = rawCsi.amplitudePackets;
        const packetSequences = rawCsi.amplitudePacketSequences;
        const latest = packets[packets.length - 1];
        const subcarriers = rawCsiVisibleSubcarriers(latest);
        if (subcarriers.length < 2) {
            rawCsiDrawEmpty(context, canvas, 'Waiting for CSI subcarriers…');
            return;
        }
        const firstSubcarrier = subcarriers[0];
        const lastSubcarrier = subcarriers[subcarriers.length - 1];
        const subcarrierSpan = Math.max(1, lastSubcarrier - firstSubcarrier);
        const originX = canvas.width * .47;
        const originY = canvas.height - 38;
        const subcarrierX = -canvas.width * .34;
        const subcarrierY = -canvas.height * .14;
        const packetX = canvas.width * .44;
        const packetY = -canvas.height * .17;
        const amplitudeSpan = Math.max(60, Math.min(
            canvas.height * .54,
            originY + subcarrierY + packetY - 24
        ));
        let peak = 0;
        let floor = Number.POSITIVE_INFINITY;
        packets.forEach((packet) => {
            subcarriers.forEach((subcarrier) => {
                const amplitude = packet[subcarrier] || 0;
                if (amplitude <= 0) return;
                peak = Math.max(peak, amplitude);
                floor = Math.min(floor, amplitude);
            });
        });
        const maximum = Math.max(RAW_CSI_AMPLITUDE_SCALE_STEP, rawCsi.amplitudeMaximum);
        if (!Number.isFinite(floor)) floor = 0;
        const project = (subcarrier, packetPosition, amplitude) => {
            const frequency = (lastSubcarrier - subcarrier) / subcarrierSpan;
            return {
                x: originX + frequency * subcarrierX + packetPosition * packetX,
                y: originY + frequency * subcarrierY + packetPosition * packetY
                    - Math.max(0, Math.min(maximum, amplitude)) * amplitudeSpan / maximum
            };
        };
        const segments = [[0, subcarriers.length - 1]];

        context.strokeStyle = 'rgba(71, 85, 105, .16)';
        context.lineWidth = 1;
        for (let tick = 0; tick <= 4; tick += 1) {
            const packetPosition = tick / 4;
            const start = project(firstSubcarrier, packetPosition, 0);
            const end = project(lastSubcarrier, packetPosition, 0);
            context.beginPath();
            context.moveTo(start.x, start.y);
            context.lineTo(end.x, end.y);
            context.stroke();
        }
        for (let tick = 0; tick <= 8; tick += 1) {
            const subcarrier = firstSubcarrier + subcarrierSpan * tick / 8;
            const front = project(subcarrier, 0, 0);
            const back = project(subcarrier, 1, 0);
            context.beginPath();
            context.moveTo(front.x, front.y);
            context.lineTo(back.x, back.y);
            context.stroke();
        }
        for (let tick = 0; tick <= 4; tick += 1) {
            const amplitude = maximum * tick / 4;
            const front = project(firstSubcarrier, 0, amplitude);
            const back = project(firstSubcarrier, 1, amplitude);
            const backRight = project(lastSubcarrier, 1, amplitude);
            context.beginPath();
            context.moveTo(front.x, front.y);
            context.lineTo(back.x, back.y);
            context.lineTo(backRight.x, backRight.y);
            context.stroke();
        }

        const projectedRows = packets.map((packet, packetIndex) => {
            const packetPosition = packetIndex / (RAW_CSI_AMPLITUDE_SURFACE_PACKETS - 1);
            return subcarriers.map((subcarrier) => {
                const amplitude = packet[subcarrier] || 0;
                const point = project(subcarrier, packetPosition, amplitude);
                return { ...point, amplitude };
            });
        });
        context.save();
        context.globalAlpha = .95;
        for (let row = projectedRows.length - 2; row >= 0; row -= 1) {
            segments.forEach(([start, end]) => {
                for (let position = start; position < end; position += 1) {
                    const backLeft = projectedRows[row][position];
                    const backRight = projectedRows[row][position + 1];
                    const frontRight = projectedRows[row + 1][position + 1];
                    const frontLeft = projectedRows[row + 1][position];
                    const amplitude = (backLeft.amplitude + backRight.amplitude
                        + frontRight.amplitude + frontLeft.amplitude) / 4;
                    context.fillStyle = rawCsiAmplitudeSurfaceColor(amplitude, floor, peak);
                    context.beginPath();
                    context.moveTo(backLeft.x, backLeft.y);
                    context.lineTo(backRight.x, backRight.y);
                    context.lineTo(frontRight.x, frontRight.y);
                    context.lineTo(frontLeft.x, frontLeft.y);
                    context.closePath();
                    context.fill();
                }
            });
        }
        context.restore();

        const rowStep = Math.max(1, Math.floor(projectedRows.length / 12));
        projectedRows.forEach((points, row) => {
            if (row % rowStep && row !== projectedRows.length - 1) return;
            context.strokeStyle = row === projectedRows.length - 1
                ? 'rgba(15, 23, 42, .34)' : 'rgba(15, 23, 42, .1)';
            context.lineWidth = row === projectedRows.length - 1 ? 1.35 : .65;
            segments.forEach(([start, end]) => {
                context.beginPath();
                for (let position = start; position <= end; position += 1) {
                    const point = points[position];
                    if (position === start) context.moveTo(point.x, point.y);
                    else context.lineTo(point.x, point.y);
                }
                context.stroke();
            });
        });

        const columnStep = Math.max(1, Math.floor(subcarriers.length / 12));
        subcarriers.forEach((_subcarrier, position) => {
            if (position % columnStep && position !== subcarriers.length - 1) return;
            context.strokeStyle = 'rgba(15, 23, 42, .1)';
            context.lineWidth = .6;
            context.beginPath();
            projectedRows.forEach((points, row) => {
                const point = points[position];
                if (row === 0) context.moveTo(point.x, point.y);
                else context.lineTo(point.x, point.y);
            });
            context.stroke();
        });

        const surfaceOrigin = project(firstSubcarrier, 0, 0);
        const subcarrierEnd = project(lastSubcarrier, 0, 0);
        const packetEnd = project(firstSubcarrier, 1, 0);
        const amplitudeTop = project(firstSubcarrier, 0, maximum);
        context.strokeStyle = 'rgba(51, 65, 85, .72)';
        context.lineWidth = 1.2;
        context.beginPath();
        context.moveTo(surfaceOrigin.x, surfaceOrigin.y);
        context.lineTo(subcarrierEnd.x, subcarrierEnd.y);
        context.moveTo(surfaceOrigin.x, surfaceOrigin.y);
        context.lineTo(packetEnd.x, packetEnd.y);
        context.moveTo(surfaceOrigin.x, surfaceOrigin.y);
        context.lineTo(amplitudeTop.x, amplitudeTop.y);
        context.stroke();

        context.fillStyle = 'rgba(15, 23, 42, .72)';
        context.font = '11px ui-sans-serif, system-ui, sans-serif';
        context.textBaseline = 'alphabetic';
        context.textAlign = 'right';
        for (let tick = 0; tick <= 4; tick += 1) {
            const amplitude = maximum * tick / 4;
            const point = project(firstSubcarrier, 0, amplitude);
            context.fillText(String(Math.round(amplitude)), point.x - 8, point.y + 4);
        }

        context.textAlign = 'center';
        for (let tick = 0; tick <= 3; tick += 1) {
            const subcarrier = firstSubcarrier + subcarrierSpan * tick / 3;
            const point = project(subcarrier, 0, 0);
            context.fillText(
                String(Math.round(subcarrier - firstSubcarrier)),
                point.x,
                point.y + 15
            );
        }
        for (let tick = 1; tick <= 3; tick += 1) {
            const packetPosition = tick / 3;
            const point = project(firstSubcarrier, packetPosition, 0);
            const sequenceIndex = Math.round(packetPosition * (packetSequences.length - 1));
            context.fillText(String(packetSequences[sequenceIndex]), point.x + 8, point.y + 15);
        }

        context.fillStyle = 'rgba(15, 23, 42, .9)';
        context.font = '12px ui-sans-serif, system-ui, sans-serif';
        context.textAlign = 'center';
        context.fillText(
            'Subcarrier',
            (surfaceOrigin.x + subcarrierEnd.x) / 2 - 8,
            (surfaceOrigin.y + subcarrierEnd.y) / 2 + 30
        );
        context.fillText(
            'Packets',
            (surfaceOrigin.x + packetEnd.x) / 2 + 12,
            (surfaceOrigin.y + packetEnd.y) / 2 + 30
        );
        context.save();
        context.translate(
            surfaceOrigin.x - 36,
            (surfaceOrigin.y + amplitudeTop.y) / 2
        );
        context.rotate(-Math.PI / 2);
        context.textAlign = 'center';
        context.fillText('Amplitude', 0, 0);
        context.restore();
    }

    function rawCsiDrawChannelProfileDeviation(context, canvas) {
        if (!rawCsi.latestProfile || !rawCsi.baseline) {
            rawCsiDrawEmpty(context, canvas);
            return;
        }
        rawCsiClearCanvas(context, canvas);
        const left = 62;
        const right = canvas.width - 28;
        const top = 38;
        const bottom = canvas.height - 58;
        const middle = (top + bottom) / 2;
        const profileScale = Math.min(108, (bottom - top) * 0.36);
        const subcarriers = rawCsiVisibleSubcarriers(rawCsi.latestProfile);
        if (subcarriers.length < 2) {
            rawCsiDrawEmpty(context, canvas, 'Waiting for CSI subcarriers…');
            return;
        }
        const firstSubcarrier = subcarriers[0];
        const lastSubcarrier = subcarriers[subcarriers.length - 1];
        const subcarrierSpan = Math.max(1, lastSubcarrier - firstSubcarrier);
        const xForSubcarrier = (subcarrier) => left
            + (subcarrier - firstSubcarrier) * (right - left) / subcarrierSpan;
        const yForValue = (value) => middle
            - (Math.max(0, Math.min(2.4, value)) - 1) * profileScale;
        const amplifiedValue = (subcarrier) => rawCsi.baseline[subcarrier]
            + (rawCsi.latestProfile[subcarrier] - rawCsi.baseline[subcarrier])
                * RAW_CSI_CHANNEL_PROFILE_DEVIATION_GAIN;
        context.strokeStyle = 'rgba(122, 105, 210, .18)';
        context.lineWidth = 1;
        [0.5, 1, 1.5, 2].forEach((value) => {
            const y = yForValue(value);
            context.beginPath();
            context.moveTo(left, y);
            context.lineTo(right, y);
            context.stroke();
        });
        for (let position = 0; position < subcarriers.length - 1; position += 1) {
            const subcarrier = subcarriers[position];
            const next = subcarriers[position + 1];
            const x0 = xForSubcarrier(subcarrier);
            const x1 = xForSubcarrier(next);
            const current0 = yForValue(amplifiedValue(subcarrier));
            const current1 = yForValue(amplifiedValue(next));
            const baseline0 = yForValue(rawCsi.baseline[subcarrier]);
            const baseline1 = yForValue(rawCsi.baseline[next]);
            const delta = ((rawCsi.latestDelta[subcarrier] || 0)
                + (rawCsi.latestDelta[next] || 0)) / 2;
            context.fillStyle = rawCsiMotionColor(delta / 0.22, 0.58);
            context.beginPath();
            context.moveTo(x0, baseline0);
            context.lineTo(x1, baseline1);
            context.lineTo(x1, current1);
            context.lineTo(x0, current0);
            context.closePath();
            context.fill();
        }
        context.setLineDash([7, 7]);
        context.strokeStyle = 'rgba(255, 255, 255, .4)';
        context.lineWidth = 1.4;
        context.beginPath();
        subcarriers.forEach((subcarrier, position) => {
            const x = xForSubcarrier(subcarrier);
            const y = yForValue(rawCsi.baseline[subcarrier]);
            if (!position) context.moveTo(x, y);
            else context.lineTo(x, y);
        });
        context.stroke();
        context.setLineDash([]);
        context.strokeStyle = '#8f7aff';
        context.lineWidth = 2.4;
        context.shadowColor = 'rgba(107, 196, 255, .55)';
        context.shadowBlur = 10;
        context.beginPath();
        subcarriers.forEach((subcarrier, position) => {
            const x = xForSubcarrier(subcarrier);
            const y = yForValue(amplifiedValue(subcarrier));
            if (!position) context.moveTo(x, y);
            else context.lineTo(x, y);
        });
        context.stroke();
        context.shadowBlur = 0;
        RAW_CSI_SELECTED_SUBCARRIERS
            .filter((subcarrier) => subcarrier >= firstSubcarrier && subcarrier <= lastSubcarrier)
            .forEach((subcarrier) => {
                const x = xForSubcarrier(subcarrier);
                const y = yForValue(amplifiedValue(subcarrier));
                context.fillStyle = '#d9d2ff';
                context.beginPath();
                context.arc(x, y, 3, 0, 2 * Math.PI);
                context.fill();
            });
        context.fillStyle = 'rgba(255, 255, 255, .55)';
        context.font = '12px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'left';
        context.fillText('— CURRENT', left, canvas.height - 18);
        context.fillStyle = 'rgba(255, 255, 255, .4)';
        context.fillText('┄ BASELINE', left + 118, canvas.height - 18);
        context.textAlign = 'right';
        context.fillStyle = 'rgba(255, 255, 255, .55)';
        context.fillText('5× DEVIATION', right, top - 12);
        context.fillText('SUBCARRIER →', right, canvas.height - 18);
    }

    function rawCsiDrawIqConstellation(context, canvas) {
        if (!rawCsi.iqHistory.length) {
            rawCsiDrawEmpty(context, canvas);
            return;
        }
        rawCsiClearCanvas(context, canvas);
        const latest = rawCsi.iqHistory[rawCsi.iqHistory.length - 1];
        const subcarrierCount = latest.length / 2;
        const amplitudes = new Float32Array(subcarrierCount);
        for (let subcarrier = 0; subcarrier < subcarrierCount; subcarrier += 1) {
            amplitudes[subcarrier] = Math.hypot(
                latest[subcarrier * 2],
                latest[subcarrier * 2 + 1]
            );
        }
        const activeSubcarriers = rawCsiVisibleSubcarriers(amplitudes);
        const extent = RAW_CSI_IQ_EXTENT;
        const panelSize = Math.min(canvas.height - 58, canvas.width - 30);
        const top = (canvas.height - panelSize) / 2;
        const centerX = canvas.width / 2;
        const centerY = top + panelSize / 2;
        const halfSpan = panelSize / 2;
        const pointPosition = (sample, subcarrier) => ({
            x: Math.max(-1, Math.min(1, sample[subcarrier * 2] / extent)) * halfSpan,
            y: Math.max(-1, Math.min(1, sample[subcarrier * 2 + 1] / extent)) * halfSpan
        });
        const left = centerX - halfSpan;
        context.fillStyle = '#09091c';
        context.fillRect(left, top, panelSize, panelSize);
        context.strokeStyle = 'rgba(121, 105, 219, .2)';
        context.lineWidth = 1;
        [0.25, 0.5, 0.75].forEach((fraction) => {
            const offset = fraction * panelSize;
            context.beginPath();
            context.moveTo(left + offset, top);
            context.lineTo(left + offset, top + panelSize);
            context.moveTo(left, top + offset);
            context.lineTo(left + panelSize, top + offset);
            context.stroke();
        });
        context.strokeStyle = 'rgba(255, 255, 255, .25)';
        context.strokeRect(left + 0.5, top + 0.5, panelSize - 1, panelSize - 1);
        activeSubcarriers.forEach((subcarrier, subcarrierIndex) => {
            const hue = 188 + subcarrierIndex * 132 / Math.max(1, activeSubcarriers.length - 1);
            rawCsi.iqHistory.forEach((sample) => {
                const point = pointPosition(sample, subcarrier);
                context.fillStyle = `hsl(${hue} 94% 68%)`;
                context.fillRect(centerX + point.x - 1.2, centerY - point.y - 1.2, 2.4, 2.4);
            });
            const point = pointPosition(latest, subcarrier);
            context.fillStyle = `hsl(${hue} 94% 72%)`;
            context.shadowColor = `hsl(${hue} 94% 62%)`;
            context.shadowBlur = 8;
            context.beginPath();
            context.arc(centerX + point.x, centerY - point.y, 3.8, 0, 2 * Math.PI);
            context.fill();
        });
        context.shadowBlur = 0;
        context.fillStyle = 'rgba(255, 255, 255, .58)';
        context.font = '12px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'center';
        context.fillText('1 SECOND WINDOW', centerX, top - 10);
        context.textAlign = 'right';
        context.fillText('I →', left + panelSize, top + panelSize + 18);
        context.textAlign = 'left';
        context.fillText('Q ↑', left + 6, top + 16);
        context.fillStyle = 'rgba(255, 255, 255, .38)';
        context.fillText(`±${Math.ceil(extent)}`, left + 6, top + panelSize - 8);
    }

    function rawCsiDrawRelativePhaseTrails(context, canvas) {
        if (!rawCsi.phaseHistory.length) {
            rawCsiDrawEmpty(context, canvas);
            return;
        }
        rawCsiClearCanvas(context, canvas);
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2 - 5;
        const radius = Math.min(canvas.width, canvas.height) * 0.36;
        context.strokeStyle = 'rgba(121, 105, 219, .22)';
        context.lineWidth = 1;
        [0.33, 0.66, 1].forEach((scale) => {
            context.beginPath();
            context.arc(centerX, centerY, radius * scale, 0, 2 * Math.PI);
            context.stroke();
        });
        context.beginPath();
        context.moveTo(centerX - radius, centerY);
        context.lineTo(centerX + radius, centerY);
        context.moveTo(centerX, centerY - radius);
        context.lineTo(centerX, centerY + radius);
        context.stroke();
        const centroids = RAW_CSI_SELECTED_SUBCARRIERS.map((_subcarrier, subcarrierIndex) => {
            let centroidReal = 0;
            let centroidImag = 0;
            rawCsi.phaseHistory.forEach((sample) => {
                centroidReal += sample[subcarrierIndex * 2];
                centroidImag += sample[subcarrierIndex * 2 + 1];
            });
            return {
                real: centroidReal / rawCsi.phaseHistory.length,
                imag: centroidImag / rawCsi.phaseHistory.length
            };
        });
        const amplifiedPoint = (subcarrierIndex, phase) => {
            const centroid = centroids[subcarrierIndex];
            let real = centroid.real
                + (phase[subcarrierIndex * 2] - centroid.real)
                    * RAW_CSI_RELATIVE_PHASE_TRAIL_GAIN;
            let imag = centroid.imag
                + (phase[subcarrierIndex * 2 + 1] - centroid.imag)
                    * RAW_CSI_RELATIVE_PHASE_TRAIL_GAIN;
            const magnitude = Math.hypot(real, imag);
            if (magnitude > 1.08) {
                real *= 1.08 / magnitude;
                imag *= 1.08 / magnitude;
            }
            return { real, imag };
        };
        RAW_CSI_SELECTED_SUBCARRIERS.forEach((subcarrier, subcarrierIndex) => {
            const hue = 188 + subcarrierIndex * 12;
            context.beginPath();
            rawCsi.phaseHistory.forEach((phase, historyIndex) => {
                const point = amplifiedPoint(subcarrierIndex, phase);
                const x = centerX + point.real * radius;
                const y = centerY - point.imag * radius;
                if (historyIndex === 0) context.moveTo(x, y);
                else context.lineTo(x, y);
            });
            context.strokeStyle = `hsla(${hue}, 92%, 68%, .62)`;
            context.lineWidth = 1.8;
            context.stroke();
            const latest = rawCsi.phaseHistory[rawCsi.phaseHistory.length - 1];
            const latestPoint = amplifiedPoint(subcarrierIndex, latest);
            const x = centerX + latestPoint.real * radius;
            const y = centerY - latestPoint.imag * radius;
            context.fillStyle = `hsl(${hue} 94% 70%)`;
            context.shadowColor = `hsl(${hue} 94% 60%)`;
            context.shadowBlur = 12;
            context.beginPath();
            context.arc(x, y, 5, 0, 2 * Math.PI);
            context.fill();
        });
        context.shadowBlur = 0;
        context.fillStyle = 'rgba(255, 255, 255, .5)';
        context.font = '12px ui-monospace, "SFMono-Regular", Consolas, monospace';
        context.textAlign = 'left';
        context.fillText('5× TRAIL SPREAD', 12, 20);
        context.textAlign = 'right';
        context.fillText('RELATIVE I', Math.min(canvas.width - 8, centerX + radius + 62), centerY + 4);
        context.textAlign = 'center';
        context.fillText('RELATIVE Q', centerX, centerY - radius - 18);
        context.fillText('CFO/STO-REDUCED PHASE · NOT POSITION', centerX, canvas.height - 16);
    }

    function rawCsiRender(timestamp) {
        rawCsi.renderFrame = 0;
        if (timestamp - rawCsi.lastRenderAt < RAW_CSI_RENDER_INTERVAL_MS) {
            rawCsi.renderFrame = requestAnimationFrame(rawCsiRender);
            return;
        }
        rawCsi.lastRenderAt = timestamp;
        const surface = rawCsiCanvasContext();
        if (!surface) return;
        const { canvas, context } = surface;
        if (rawCsi.visualization === 'subcarrier-amplitudes') {
            rawCsiDrawSubcarrierAmplitudes(context, canvas);
        }
        else if (rawCsi.visualization === 'csi-amplitude-surface') {
            rawCsiDrawAmplitudeSurface(context, canvas);
        }
        else if (rawCsi.visualization === 'channel-profile-deviation') {
            rawCsiDrawChannelProfileDeviation(context, canvas);
        }
        else if (rawCsi.visualization === 'iq-constellation') rawCsiDrawIqConstellation(context, canvas);
        else if (rawCsi.visualization === 'relative-phase-trails') {
            rawCsiDrawRelativePhaseTrails(context, canvas);
        }
    }

    function rawCsiScheduleRender() {
        if (rawCsi.renderFrame) return;
        rawCsi.renderFrame = requestAnimationFrame(rawCsiRender);
    }

    function rawCsiSelectVisualization(value) {
        const visualization = RAW_CSI_VISUALIZATIONS[value]
            ? value : 'subcarrier-amplitudes';
        const metadata = RAW_CSI_VISUALIZATIONS[visualization];
        rawCsi.visualization = visualization;
        const select = $('.js-raw-visualization-select');
        const title = $('.js-raw-visualization-title');
        const description = $('.js-raw-visualization-description');
        const badge = $('.js-raw-visualization-badge');
        const canvas = $('.js-raw-visualization');
        if (select) select.value = visualization;
        if (title) title.textContent = metadata.title;
        if (description) description.textContent = metadata.description;
        if (badge) badge.textContent = metadata.badge;
        if (canvas) canvas.setAttribute('aria-label', metadata.ariaLabel);
        rawCsiScheduleRender();
    }

    function rawCsiConsumeRecord(record, streamSequence) {
        if (!record.byteLength) return;
        if (record.byteLength < RAW_CSI_V8_HEADER_BYTES) {
            throw new Error('Device sent an unsupported CSI record.');
        }
        const view = new DataView(record.buffer, record.byteOffset, record.byteLength);
        const headerLength = view.getUint8(3);
        const subcarriers = view.getUint16(10, true);
        const csiLength = view.getUint16(12, true);
        if (view.getUint16(0, true) !== 0x4353 || view.getUint8(2) !== 8
            || headerLength !== RAW_CSI_V8_HEADER_BYTES || csiLength !== subcarriers * 2
            || headerLength + csiLength > record.byteLength) {
            throw new Error('Device sent a malformed CSI V8 record.');
        }
        const expectedRecordSequence = streamSequence > 0xFFFFFFFFn
            ? 0xFFFFFFFF : Number(streamSequence);
        if (view.getUint32(6, true) !== expectedRecordSequence) {
            throw new Error('Device sent mismatched raw CSI sequence numbers.');
        }
        const amplitudes = new Float32Array(subcarriers);
        const iValues = new Float32Array(subcarriers);
        const qValues = new Float32Array(subcarriers);
        for (let index = 0, offset = headerLength;
            offset < headerLength + csiLength; index += 1, offset += 2) {
            // Espressif CSI stores each complex sample as [imaginary, real].
            qValues[index] = view.getInt8(offset);
            iValues[index] = view.getInt8(offset + 1);
            amplitudes[index] = Math.hypot(iValues[index], qValues[index]);
        }
        rawCsiCollectRadioMetrics(view);
        const capturedTicksUs = Number(view.getBigUint64(22, true))
            || rawCsi.lastCaptureTicksUs + RAW_CSI_VISUAL_STEP_US;
        rawCsiCollectCaptureInterval(capturedTicksUs);
        rawCsiIngestVisualFrame(amplitudes, iValues, qValues, capturedTicksUs);
        rawCsiMarkReady();
    }

    function rawCsiAppend(chunk) {
        if (!rawCsi.parser) throw new Error('Raw CSI parser is not initialized.');
        rawCsi.parser.append(chunk).forEach((frame) => {
            rawCsi.metricsFresh = frame.freshRecordTotal;
            rawCsi.metricsDropped = frame.rawDropTotal;
            rawCsi.metricsBackpressure = frame.sendBackpressureTotal;
            rawCsiConsumeRecord(frame.record, frame.streamSequence);
            rawCsi.metricsReceived += 1;
        });
    }

    function rawCsiDemoFrame(targetPps, intervalMs, startedAtMs) {
        const elapsedSec = (performance.now() - startedAtMs) / 1000;
        const amplitudes = new Float32Array(64);
        const iValues = new Float32Array(64);
        const qValues = new Float32Array(64);
        const motion = 0.08 + conn.movement * 0.92;
        for (let index = 0; index < amplitudes.length; index += 1) {
            const channelShape = 34 + 8 * Math.sin(index * 0.31) + 5 * Math.cos(index * 0.13);
            const disturbance = motion * 18 * Math.sin(elapsedSec * 5.2 + index * 0.19);
            const amplitude = Math.max(4, channelShape + disturbance);
            const phase = index * 0.23 + elapsedSec * (0.7 + motion * 1.8);
            iValues[index] = Math.cos(phase) * amplitude;
            qValues[index] = Math.sin(phase) * amplitude;
            amplitudes[index] = amplitude;
        }
        const captureTicksUs = Math.round(performance.now() * 1000);
        rawCsiCollectCaptureInterval(captureTicksUs);
        rawCsiIngestVisualFrame(amplitudes, iValues, qValues, captureTicksUs);
        rawCsi.demoFresh += Math.max(1, Math.round(targetPps * intervalMs / 1000));
        rawCsi.metricsFresh = rawCsi.demoFresh;
        rawCsi.metricsReceived += 1;
        const rssi = Math.round(-50 + motion * 7);
        const noiseFloor = -96;
        rawCsiCollectSignalSample(rssi, noiseFloor);
        rawCsiMarkReady();
    }

    function rawCsiStartDemo(targetPps) {
        const intervalMs = Math.max(10, Math.round(1000 / targetPps));
        const startedAtMs = performance.now();
        rawCsi.demoFresh = 0;
        rawCsiResetVisualization();
        rawCsiStartMetrics();
        rawCsiSetState('running');
        rawCsiStatus(`Simulated signal stream running at a target of ${targetPps} packets per second.`);
        rawCsiDemoFrame(targetPps, intervalMs, startedAtMs);
        rawCsi.demoTimer = setInterval(
            () => rawCsiDemoFrame(targetPps, intervalMs, startedAtMs), intervalMs);
    }

    async function rawCsiStop(reason = 'user', expectedGeneration = rawCsi.generation) {
        if (expectedGeneration !== rawCsi.generation || rawCsi.state === 'idle') return;
        if (rawCsi.state === 'stopping') return rawCsi.stopPromise;
        rawCsiFinishTracking(rawCsi.analyticsReady ? 'stopped' : 'cancelled', null, reason);
        const stopGeneration = ++rawCsi.generation;
        clearInterval(rawCsi.demoTimer);
        rawCsi.demoTimer = null;
        rawCsi.demoFresh = 0;
        rawCsiStopMetrics();
        rawCsi.controller?.abort('raw stream stopped');
        rawCsi.controller = null;
        rawCsiSetState('stopping');
        rawCsi.parser = null;
        rawCsi.stopPromise = (async () => {
            if (rawCsi.generation !== stopGeneration) return;
            rawCsi.sessionClient = null;
            rawCsi.stopPromise = null;
            rawCsiSetState('idle');
        })();
        return rawCsi.stopPromise;
    }

    async function rawCsiStart() {
        const client = directClient;
        if (rawCsi.state !== 'idle' || conn.status !== 'connected') return;
        const generation = ++rawCsi.generation;
        if (conn.mode === 'demo') {
            rawCsiBeginTracking();
            rawCsiStartDemo(100);
            return;
        }
        if (!rawCsiDirectReady() || client.capabilities?.features?.csi !== true) return;
        rawCsiBeginTracking();
        rawCsiSetState('starting');
        rawCsi.sessionClient = client;
        rawCsiStatus('Starting the signal stream…');
        try {
            if (rawCsi.generation !== generation || rawCsi.state !== 'starting') return;
            rawCsi.parser = new window.ESPectreRawCsiParser();
            rawCsiResetVisualization();
            const controller = new AbortController();
            rawCsi.controller = controller;
            const response = await fetch(client.rawEndpoint, {
                method: 'GET',
                headers: { Accept: 'application/octet-stream' },
                cache: 'no-store',
                signal: controller.signal,
                targetAddressSpace: 'local'
            });
            if (rawCsi.generation !== generation || rawCsi.state !== 'starting') return;
            if (!response.ok || !response.body) throw new Error(`Raw stream returned HTTP ${response.status}.`);
            rawCsiStartMetrics();
            rawCsiSetState('running');
            rawCsiStatus('Live signal data is arriving. Normal motion sensing will resume when you stop the stream.');
            const reader = response.body.getReader();
            while (rawCsi.generation === generation && !controller.signal.aborted) {
                const { value, done } = await reader.read();
                if (done) break;
                if (rawCsi.generation !== generation) break;
                rawCsiAppend(value);
            }
            if (rawCsi.generation === generation && !controller.signal.aborted) {
                throw new Error('Raw stream ended unexpectedly.');
            }
        } catch (error) {
            if (rawCsi.generation === generation && !rawCsi.controller?.signal.aborted) {
                console.warn('Raw CSI stream failed:', error);
                rawCsiStatus('The signal stream stopped unexpectedly. Stop it, then try again.', true);
                rawCsiFinishTracking('failure', error);
            }
        } finally {
            if (rawCsi.generation === generation
                    && rawCsi.state !== 'idle' && rawCsi.state !== 'stopping') {
                await rawCsiStop('stream_error', generation);
            }
        }
    }

    function rawCsiChooseDevice() {
        disconnect();
        directEndpointInput()?.focus();
    }

    function rawCsiToggle() {
        if (rawCsi.state === 'idle') void rawCsiStart();
        else if (rawCsi.state !== 'stopping') void rawCsiStop('user');
    }

    function rawCsiInit() {
        $('.js-raw-csi-choose-device')?.addEventListener('click', rawCsiChooseDevice);
        $('.js-raw-csi-toggle')?.addEventListener('click', rawCsiToggle);
        $('.js-raw-visualization-select')?.addEventListener('change', (event) => {
            rawCsiSelectVisualization(event.target.value);
        });
        const stage = $('.raw-csi-visualization-stage');
        if (stage && typeof ResizeObserver !== 'undefined') {
            rawCsi.resizeObserver = new ResizeObserver(rawCsiResizeVisualization);
            rawCsi.resizeObserver.observe(stage);
        } else {
            window.addEventListener('resize', rawCsiResizeVisualization);
        }
        rawCsiSelectVisualization(rawCsi.visualization);
        rawCsiResizeVisualization();
    }
