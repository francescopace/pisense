/*
 * ESPectre - Monitor tool
 *
 * Part of the website application shell.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

'use strict';

    /* ============================================================= monitor */

    const MONITOR_CHART_WINDOW_MS = 60 * 1000;
    const MONITOR_CALIBRATION_FALLBACK_MS = 45 * 1000;
    const MONITOR_CALIBRATION_SAFETY_MS = 90 * 1000;
    const MONITOR_DEMO_CALIBRATION_MS = 2500;

    function monitorChartMaxPoints() {
        return Math.max(2, Math.ceil(MONITOR_CHART_WINDOW_MS / evaluationIntervalMs()) + 2);
    }

    function monitorChartCoalesceMs() {
        return Math.max(16, Math.min(100, Math.floor(evaluationIntervalMs() / 2)));
    }

    const monitor = {
        demoTimer: null,
        demoT: 0,
        demoMove: 0.05,
        points: [],
        chartFrame: 0,
        lastTelemetryAt: 0,
        startedAt: 0,
        connectedAt: 0,
        entryPoint: '',
        inputMode: null,
        readyState: '',
        readyAt: 0,
        readyTracked: false,
        commands: new Set(),
        commandCatalogReady: false,
        switchingTransport: false,
        diagTimer: null,
        diagIntervalMs: 0,
        diagRequestPending: false,
        calibrating: false,
        calibrationTimer: null
    };

    function monitorStatus(message) {
        const status = $('.js-mon-status');
        if (!status) return;
        status.hidden = !message;
        status.textContent = message || '';
    }

    function monitorDiagStatus(message) {
        const el = $('.js-mon-diag-status');
        if (!el) return;
        el.hidden = !message;
        el.textContent = message || '';
    }

    function monitorResetChart() {
        monitor.points = [];
        monitor.lastTelemetryAt = 0;
        if (monitor.chartFrame) {
            cancelAnimationFrame(monitor.chartFrame);
            monitor.chartFrame = 0;
        }
        const canvas = $('.js-mon-chart');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);
    }

    function resetMonitorLiveView() {
        monitorResetChart();
        const stateEl = $('.js-mon-state');
        if (stateEl) {
            stateEl.textContent = '—';
            stateEl.classList.remove('motion');
        }
        const moveEl = $('.js-mon-move');
        if (moveEl) moveEl.textContent = '—';
        monitorStats({});
        monitorDiagStatus('');
        conn.movement = 0;
        conn.motion = false;
        renderTelemetry();
    }

    function monitorQueueChart() {
        if (monitor.chartFrame) return;
        monitor.chartFrame = requestAnimationFrame(() => {
            monitor.chartFrame = 0;
            monitorDrawChart();
        });
    }

    function monitorFeed(movement, threshold, state) {
        const now = Date.now();
        const motion = state !== null && state !== undefined
            ? state === 1 || state === 'motion'
            : movement >= threshold;
        const last = monitor.points[monitor.points.length - 1];
        if (last && now - last.at < monitorChartCoalesceMs()) {
            last.m = movement;
            last.t = threshold;
            last.at = now;
            last.on = motion;
        } else {
            monitor.points.push({ m: movement, t: threshold, at: now, on: motion });
        }
        const oldest = now - MONITOR_CHART_WINDOW_MS;
        while (monitor.points.length
                && (monitor.points[0].at < oldest || monitor.points.length > monitorChartMaxPoints())) {
            monitor.points.shift();
        }
        const stateEl = $('.js-mon-state');
        if (stateEl) {
            stateEl.textContent = motion ? 'MOTION' : 'IDLE';
            stateEl.classList.toggle('motion', motion);
        }
        const movementEl = $('.js-mon-move');
        if (movementEl) movementEl.textContent = movement.toFixed(3);
        monitorQueueChart();
    }

    function monitorStat(value, digits, suffix) {
        if (value === null || value === undefined || !Number.isFinite(Number(value))) return '—';
        return Number(value).toFixed(digits) + suffix;
    }

    function monitorSetStat(selector, value, digits, suffix) {
        const element = $(selector);
        if (element) element.textContent = monitorStat(value, digits, suffix);
    }

    function monitorStats(data) {
        const errors = [data.csi_rx_error_total, data.csi_rx_end_error_total,
            data.csi_invalid_estimate_total, data.csi_invalid_first_word_total];
        const errorTotal = errors.every(value => Number.isSafeInteger(value) && value >= 0)
            ? errors.reduce((sum, value) => sum + value, 0) : null;
        monitorSetStat('.js-mon-traffic', data.traffic_tx_pps, 1, ' pps');
        monitorSetStat('.js-mon-callbacks', data.csi_callback_pps, 1, ' pps');
        monitorSetStat('.js-mon-accepted', data.csi_accepted_pps, 1, ' pps');
        monitorSetStat('.js-mon-errors', errorTotal, 0, ' total');
        monitorSetStat('.js-mon-occupancy', data.csi_occupancy == null ? null : data.csi_occupancy * 100, 1, '%');
        monitorSetStat('.js-mon-rssi', data.wifi_rssi_dbm, 0, ' dBm');
        monitorSetStat('.js-mon-heap', data.free_memory_kb, 1, ' KiB');
        monitorSetStat('.js-mon-loop', data.loop_time_ms, 2, ' ms');
    }

    function monitorDrawChart() {
        const canvas = $('.js-mon-chart');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;
        ctx.clearRect(0, 0, width, height);
        if (width < 2 || height < 2 || monitor.points.length < 2) return;

        const styles = getComputedStyle(document.documentElement);
        const accent = styles.getPropertyValue('--accent').trim() || '#4f6bff';
        const accentSoft = styles.getPropertyValue('--accent-soft').trim() || 'rgba(79, 107, 255, 0.09)';
        const dim = styles.getPropertyValue('--dim').trim() || '#888';
        const border = styles.getPropertyValue('--border').trim() || '#e6e9ee';
        const labelH = 16;
        const plotH = Math.max(8, height - labelH);
        const now = monitor.points[monitor.points.length - 1].at;
        const t0 = now - MONITOR_CHART_WINDOW_MS;
        const x = (at) => ((at - t0) / MONITOR_CHART_WINDOW_MS) * width;
        const y = (v) => plotH - Math.min(1, Math.max(0, v)) * (plotH - 8) - 4;

        ctx.lineWidth = 1;
        ctx.strokeStyle = border;
        ctx.fillStyle = dim;
        ctx.font = '10px ui-monospace, "SFMono-Regular", Consolas, monospace';
        ctx.textBaseline = 'top';
        const tickMs = 10 * 1000;
        const labelEvery = width >= 420 ? 1 : 2;
        for (let age = MONITOR_CHART_WINDOW_MS; age >= 0; age -= tickMs) {
            const px = Math.max(0.5, Math.min(width - 0.5, x(now - age)));
            ctx.beginPath();
            ctx.moveTo(px, 0);
            ctx.lineTo(px, plotH);
            ctx.stroke();
            const ticks = age / tickMs;
            if (ticks % labelEvery !== 0 && ticks !== 0) continue;
            const label = age === 0 ? 'now' : `−${age / 1000}s`;
            ctx.textAlign = age === 0 ? 'right' : (age === MONITOR_CHART_WINDOW_MS ? 'left' : 'center');
            ctx.fillText(label, px, plotH + 2);
        }

        ctx.fillStyle = accentSoft;
        let bandStart = null;
        monitor.points.forEach((p) => {
            if (p.on) {
                if (bandStart === null) bandStart = p.at;
                return;
            }
            if (bandStart !== null) {
                ctx.fillRect(x(bandStart), 0, Math.max(1, x(p.at) - x(bandStart)), plotH);
                bandStart = null;
            }
        });
        if (bandStart !== null) {
            ctx.fillRect(x(bandStart), 0, Math.max(1, x(now) - x(bandStart)), plotH);
        }

        ctx.strokeStyle = dim;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        monitor.points.forEach((p, i) => {
            const px = x(p.at);
            i === 0 ? ctx.moveTo(px, y(p.t)) : ctx.lineTo(px, y(p.t));
        });
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.lineWidth = 2;
        ctx.strokeStyle = accent;
        ctx.beginPath();
        monitor.points.forEach((p, i) => {
            const px = x(p.at);
            i === 0 ? ctx.moveTo(px, y(p.m)) : ctx.lineTo(px, y(p.m));
        });
        ctx.stroke();
    }

    function monitorResizeChart() {
        const canvas = $('.js-mon-chart');
        if (!canvas) return;
        const rect = canvas.getBoundingClientRect();
        if (rect.width > 0 && canvas.width !== Math.round(rect.width)) {
            canvas.width = Math.round(rect.width);
            monitorDrawChart();
        }
    }

    function markMonitorReady(readiness) {
        if (!monitor.inputMode) return;
        if (!monitor.readyState) monitor.readyAt = Date.now();
        monitor.readyState = readiness;
        if (monitor.readyTracked) return;
        monitor.readyTracked = track('tool_ready', {
            tool_name: 'monitor',
            entry_point: monitor.entryPoint || 'monitor',
            transport: 'simulation',
            input_mode: monitor.inputMode,
            readiness,
            latency_ms: Math.max(0, monitor.readyAt - (monitor.startedAt || monitor.connectedAt))
        });
    }

    function monitorStopAll(reason = 'replaced') {
        if (monitor.inputMode && monitor.connectedAt) {
            track('tool_disconnect', {
                tool_name: 'monitor',
                entry_point: monitor.entryPoint || 'monitor',
                transport: 'simulation',
                input_mode: monitor.inputMode,
                reason,
                duration_seconds: Math.max(0, Math.round((Date.now() - monitor.connectedAt) / 1000))
            });
        }
        resetMonitorSession();
    }

    function monitorPublishCommand(fields, {
        pendingMessage = 'Saving setting…',
        statusFn = monitorStatus,
        timeoutMs = 8000
    } = {}) {
        if (conn.mode === 'direct' && directClient?.connected) {
            const { command, ...params } = fields;
            statusFn(pendingMessage);
            if (command === 'read_diagnostics') {
                return directClient.request('get', 'diagnostics', {}, { timeoutMs });
            }
            if (command === 'recalibrate') {
                return directClient.request('post', 'sensing/calibrations', {}, { timeoutMs });
            }
            if (command === 'check_ota') {
                return directClient.request('post', 'ota/checks', params, { timeoutMs });
            }
            if (command === 'start_ota') {
                return directClient.request('post', 'ota/updates', params, { timeoutMs });
            }
            return directClient.request('patch', 'sensing', params, { timeoutMs });
        }
        const error = new Error('Connect to the device before changing its settings.');
        statusFn(error.message);
        return Promise.reject(error);
    }

    function reportSensingChange(action, result, error) {
        track('sensing_change', {
            ...connectionParams(),
            transport: connectionTransport(),
            input_mode: connectionInputMode(),
            action,
            result,
            ...(error ? { error_type: errorType(error) } : {})
        });
    }

    function diagnosticsRequestPending() {
        return conn.mode === 'direct' && monitor.diagRequestPending;
    }

    function diagnosticsPanelOpen() {
        const panel = $('.device-live-diagnostics');
        const workspace = $('.js-monitor-workspace');
        return !!(panel && panel.open && !document.hidden
            && route === 'tool-monitor' && workspace && !workspace.hidden);
    }

    function stopDiagnosticsPolling() {
        if (!monitor.diagTimer) return;
        clearInterval(monitor.diagTimer);
        monitor.diagTimer = null;
        monitor.diagIntervalMs = 0;
    }

    function syncDiagnosticsPolling() {
        const canPoll = diagnosticsPanelOpen()
            && (conn.mode === 'demo' || conn.mode === 'direct');
        if (!canPoll) {
            stopDiagnosticsPolling();
            return;
        }
        const interval = DIAGNOSTICS_POLL_INTERVAL_MS;
        if (monitor.diagTimer && monitor.diagIntervalMs === interval) return;
        stopDiagnosticsPolling();
        monitorRequestStats();
        monitor.diagIntervalMs = interval;
        monitor.diagTimer = setInterval(monitorRequestStats, interval);
    }

    async function monitorRequestStats() {
        if (!diagnosticsPanelOpen()) {
            stopDiagnosticsPolling();
            return;
        }
        if (conn.mode === 'demo') {
            monitorStats({
                traffic_tx_pps: csiTargetPps(),
                csi_callback_pps: Math.max(1, csiTargetPps() - 4),
                csi_accepted_pps: Math.max(1, csiTargetPps() - 10),
                csi_rx_error_total: 2,
                csi_rx_end_error_total: 0,
                csi_invalid_estimate_total: 4,
                csi_invalid_first_word_total: 0,
                csi_occupancy: Math.max(1, csiTargetPps() - 16) / csiTargetPps(),
                wifi_rssi_dbm: -55,
                free_memory_kb: 161.4,
                loop_time_ms: 0.31
            });
            return;
        }
        if (diagnosticsRequestPending()) return;
        const direct = conn.mode === 'direct';
        if (direct && !directClient?.connected) return;
        try {
            if (direct) monitor.diagRequestPending = true;
            const response = await monitorPublishCommand({ command: 'read_diagnostics' }, {
                pendingMessage: '',
                statusFn: () => {}
            });
            const data = direct ? response : response?.data;
            if (data && typeof data === 'object') {
                markMonitorReady('diagnostics');
                monitorStats(data);
                monitorDiagStatus('');
            }
        } catch (error) {
            if (diagnosticsPanelOpen()) monitorDiagStatus(error.message);
        } finally {
            if (direct) monitor.diagRequestPending = false;
        }
    }

    function monitorOpenConnectivity() {
        setDeviceView('connectivity');
        renderConnection();
    }

    function monitorEditOrCancel() {
        monitorOpenConnectivity();
    }

    async function beginCalibration() {
        if (monitor.calibrating) return;
        setCalibrationBusy(true);
        if (conn.mode === 'demo') {
            toast('Calibration started. (demo)');
            scheduleCalibrationIdle(MONITOR_DEMO_CALIBRATION_MS);
            return;
        }
        try {
            const result = await monitorPublishCommand({ command: 'recalibrate' }, {
                pendingMessage: 'Starting calibration…',
                statusFn: toast
            });
            reportSensingChange('recalibrate', 'accepted');
            toast(result.message || 'Calibration started. Keep the room still.');
            scheduleCalibrationIdle(MONITOR_CALIBRATION_FALLBACK_MS);
        } catch (error) {
            console.warn('Calibration failed:', error);
            reportSensingChange('recalibrate', 'failure', error);
            toast('Calibration could not start. Check the connection and try again.');
            setCalibrationBusy(false);
        }
    }

    async function runSensingCommand(fields, pendingMessage, successMessage, demoUpdate) {
        if (conn.mode === 'demo') {
            if (demoUpdate) demoUpdate();
            toast(successMessage + ' (demo)');
            return;
        }
        try {
            const result = await monitorPublishCommand(fields, { pendingMessage, statusFn: toast });
            reportSensingChange(fields.command, 'accepted');
            toast(result.message || successMessage);
        } catch (error) {
            console.warn('Sensing setting update failed:', error);
            reportSensingChange(fields.command, 'failure', error);
            toast('The setting could not be saved. Check the connection and try again.');
        }
    }

    function selectMonitorTransport(mode) {
        document.querySelector('espectre-connection-picker[data-surface="monitor"]')?.select(mode);
    }

    function monitorInit() {
        const diagnostics = $('.device-live-diagnostics');
        if (diagnostics) {
            diagnostics.addEventListener('toggle', syncDiagnosticsPolling);
        }
        $('.js-device-edit-connectivity').addEventListener('click', monitorEditOrCancel);
        $('.js-monitor-name-trigger').addEventListener('click', startMonitorDeviceNameEdit);
        const nameInput = $('.js-monitor-name-input');
        nameInput.addEventListener('blur', () => { saveMonitorDeviceNameOnBlur(); });
        nameInput.addEventListener('keydown', (event) => {
            if (event.key === 'Enter') {
                event.preventDefault();
                nameInput.blur();
            } else if (event.key === 'Escape') {
                event.preventDefault();
                cancelMonitorDeviceNameEdit();
            }
        });
        bindThresholdControls();
        document.getElementById('sense-detector').addEventListener('change', () => {
            const detector = document.getElementById('sense-detector').value;
            syncSensingControls();
            runSensingCommand(
                { command: 'update_sensing', detector },
                'Changing detection mode…',
                'Detection mode updated.'
            );
        });
        const applyMotionHits = () => {
            const motionOnHits = Number(document.getElementById('sense-motion-on').value);
            const motionOffHits = Number(document.getElementById('sense-motion-off').value);
            if (![motionOnHits, motionOffHits].every((value) => Number.isInteger(value) && value >= 1 && value <= 20)) {
                toast('Use a whole number from 1 to 20 for each advanced motion setting.');
                return;
            }
            runSensingCommand(
                { command: 'update_sensing', motion_on_hits: motionOnHits, motion_off_hits: motionOffHits },
                'Saving advanced motion settings…',
                'Advanced motion settings updated.'
            );
        };
        document.getElementById('sense-motion-on').addEventListener('change', applyMotionHits);
        document.getElementById('sense-motion-off').addEventListener('change', applyMotionHits);
        $('.js-sense-recalibrate').addEventListener('click', beginCalibration);
        document.getElementById('sense-csi-mode').addEventListener('change', () => {
            const csiTrafficMode = document.getElementById('sense-csi-mode').value;
            runSensingCommand(
                { command: 'update_sensing', csi_traffic_mode: csiTrafficMode },
                'Changing the Wi-Fi traffic source…',
                'Wi-Fi traffic source updated.'
            );
        });
        document.getElementById('sense-generator-mode').addEventListener('change', () => {
            const trafficGeneratorMode = document.getElementById('sense-generator-mode').value;
            runSensingCommand(
                { command: 'update_sensing', traffic_generator_mode: trafficGeneratorMode },
                'Changing the built-in traffic type…',
                'Built-in traffic type updated.'
            );
        });
        window.addEventListener('resize', monitorResizeChart);
    }
