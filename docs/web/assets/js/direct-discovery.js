/*
 * ESPectre - Direct discovery
 *
 * Part of the website application shell.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

'use strict';

    function directEndpointInput() {
        return document.querySelector(`[data-page="${route}"] .js-direct-endpoint`)
            || document.querySelector('.js-direct-endpoint');
    }

    function syncDirectEndpointInputs(target) {
        $$('.js-direct-endpoint').forEach((input) => {
            if (input && target) input.value = target;
        });
    }

    function privateIpv4Address(value) {
        const octets = value.split('.');
        if (octets.length !== 4 || octets.some((octet) => !/^(0|[1-9][0-9]{0,2})$/.test(octet))) {
            return false;
        }
        const values = octets.map(Number);
        if (values.some((octet) => octet > 255)) return false;
        return values[0] === 10
            || (values[0] === 172 && values[1] >= 16 && values[1] <= 31)
            || (values[0] === 192 && values[1] === 168);
    }

    function directTargetForEndpoint(endpoint) {
        const url = new URL(DirectProtocolClient.normalizeEndpoint(endpoint));
        const canonical = url.hostname.toLowerCase().match(DIRECT_CANONICAL_HOSTNAME);
        if (canonical) return canonical[1];
        if (privateIpv4Address(url.hostname)) return url.hostname;
        if (url.hostname.includes(':')) return url.hostname;
        return url.hostname;
    }

    function parseDirectTarget(value) {
        const input = String(value || '').trim().toLowerCase();
        if (!input) throw new Error('Enter a device IP address, ID, or name.');
        if (DIRECT_FULL_DEVICE_ID.test(input)) {
            return {
                display: input,
                endpoint: DirectProtocolClient.normalizeEndpoint(`espectre-${input}.local`),
                deviceId: input,
                shortId: '',
                discoveryFallback: true
            };
        }
        if (DIRECT_SHORT_DEVICE_ID.test(input)) {
            return { display: input, endpoint: '', deviceId: '', search: input, shortId: input };
        }
        if (privateIpv4Address(input)) {
            return {
                display: input,
                endpoint: DirectProtocolClient.normalizeEndpoint(input),
                deviceId: '',
                shortId: ''
            };
        }
        try {
            const endpoint = DirectProtocolClient.normalizeEndpoint(input);
            return {
                display: directTargetForEndpoint(endpoint), endpoint, deviceId: '', shortId: ''
            };
        } catch (error) {
            if (/^[a-z][a-z0-9+.-]*:\/\//i.test(input)
                || /^\d{1,3}(?:\.\d{1,3}){3}$/.test(input)) {
                throw new Error('Enter a private device IP address from the same Wi-Fi network.', { cause: error });
            }
            if (input.length <= 63 && [...input].every((character) => character >= ' ')) {
                return { display: input, endpoint: '', deviceId: '', search: input, shortId: '' };
            }
            throw new Error('Enter a device IP address, ID, or name.', { cause: error });
        }
    }

    function consumeDirectHandoff() {
        const params = new URLSearchParams(location.search);
        const directTarget = params.get('target') || '';
        if (!directTarget) return;
        try {
            const target = parseDirectTarget(directTarget);
            syncDirectEndpointInputs(target.display);
            const radio = document.getElementById('monitor-transport-direct');
            if (radio) radio.checked = true;
            history.replaceState(null, '', location.pathname + location.hash);
        } catch (_error) {
            history.replaceState(null, '', location.pathname + location.hash);
            toast('The device handoff contained an invalid local device.');
        }
    }

    function directCapabilitiesSnapshot(capabilities) {
        const methods = new Set((capabilities.operations || []).map((item) => item?.name).filter(Boolean));
        const sections = new Set(capabilities.resources || []);
        monitor.commands = methods;
        monitor.commandCatalogReady = true;
        return {
            supports_wifi_status: sections.has('wifi'),
            supports_wifi_bssid: methods.has('set_wifi_bssid')
                && methods.has('clear_wifi_bssid') && methods.has('scan_wifi'),
            supports_wifi_clear: methods.has('clear_wifi_credentials'),
            supports_mqtt_config: methods.has('update_mqtt'),
            supports_device_config: methods.has('update_device'),
            supports_runtime_threshold: methods.has('update_sensing'),
            supports_runtime_motion_hits: methods.has('update_sensing'),
            supports_runtime_detector: methods.has('update_sensing'),
            supports_manual_recalibration: methods.has('recalibrate'),
            supports_traffic_control: methods.has('update_sensing'),
            supports_ota: sections.has('ota')
        };
    }

    function directSupportsCommand(name) {
        return Boolean(directClient?.capabilities?.operations?.some((item) => item?.name === name));
    }

    function applyDirectConfig(config) {
        const device = config.device || {};
        const runtime = config.runtime || {};
        const wifi = config.wifi || {};
        const mqtt = config.mqtt || {};
        applySysinfo({
            ...device,
            device_label: device.label,
            wifi_configured: wifi.configured,
            wifi_connected: wifi.connected,
            wifi_ssid: wifi.ssid,
            wifi_band: wifi.band,
            wifi_channel: wifi.channel,
            wifi_rssi_dbm: wifi.rssi_dbm,
            wifi_bssid: wifi.bssid,
            wifi_apply_state: wifi.apply_state,
            wifi_apply_message: wifi.apply_message,
            mqtt_configured: mqtt.configured,
            mqtt_scheme: mqtt.scheme,
            mqtt_host: mqtt.host,
            mqtt_port: mqtt.port,
            mqtt_topic_prefix: mqtt.topic_prefix,
            mqtt_username_configured: mqtt.username_configured
        });
        applySensingSnapshot(runtime);
    }

    function applyRuntimeStatus(status) {
        if (!status || status.calibrating === undefined) return;
        const calibrating = sysinfoBoolean(status.calibrating);
        setCalibrationBusy(calibrating);
        if (calibrating) scheduleCalibrationIdle(MONITOR_CALIBRATION_SAFETY_MS);
    }

    function ingestDirectEvent(name, data) {
        if (name === 'motion') {
            applySensingCadence(data);
            applyLiveTelemetry(
                Number(data.score ?? 0),
                Number(conn.threshold),
                data.state
            );
            monitorFeed(conn.movement, conn.threshold, data.state);
            monitorResizeChart();
            return;
        }
        if (name === 'device' || name === 'health' || name === 'capabilities') {
            applySysinfo(data);
            return;
        }
        if (name === 'sensing') {
            applyDirectConfig({ runtime: data });
            applyRuntimeStatus(data);
            return;
        }
        if (name === 'wifi') {
            applyDirectConfig({ wifi: data });
            return;
        }
        if (name === 'ota') applyOtaStatus(data);
        if (name === 'fault') toast(data.message || 'The device reported a runtime fault.');
    }

    const directResourceSessions = new WeakMap();

    function directResourceSession(client) {
        if (!directResourceSessions.has(client)) {
            directResourceSessions.set(client, { snapshots: new Map(), pending: new Map() });
        }
        return directResourceSessions.get(client);
    }

    function rememberDirectResource(client, resource, snapshot) {
        if (directClient === client && client.connected) {
            directResourceSession(client).snapshots.set(resource, snapshot);
        }
    }

    function makeDirectClient(endpoint) {
        const client = new DirectProtocolClient(endpoint);
        client.on('open', () => directResourceSessions.delete(client));
        client.on('event', (name, data) => {
            if (directClient !== client) return;
            if (['device', 'health', 'sensing', 'wifi', 'ota'].includes(name)) {
                rememberDirectResource(client, name, data);
            }
            ingestDirectEvent(name, data);
        });
        client.on('protocol-error', (error) => console.warn('Ignored invalid Direct frame:', error.message));
        client.on('close', ({ expected }) => {
            directResourceSessions.delete(client);
            if (expected || conn.mode !== 'direct') return;
            scheduleDirectReconnect(client);
        });
        return client;
    }

    function directPageOriginKind() {
        return sitePolicy.directOriginKind(location);
    }

    function directBrowserGuidance() {
        if (location.protocol !== 'https:') {
            return 'Local development mode: the firmware must explicitly allow HTTP loopback Origins. Development builds accept localhost on any port.';
        }
        if (browserSupport.hostedDirect === 'targeted'
            && ['windows', 'linux'].includes(browserSupport.operatingSystem)) {
            const platform = browserSupport.operatingSystem === 'windows' ? 'Windows' : 'Linux';
            return `Local connections are supported in desktop Chrome on ${platform}. If search does not find the device, enter its current IP address.`;
        }
        if (browserSupport.hostedDirect === 'targeted') return '';
        return 'This browser may not connect to local devices. Use Chrome 151 or later on macOS, Windows, or native Linux.';
    }

    function renderDirectBrowserGuidance() {
        $$('.js-direct-browser-note').forEach((note) => {
            const message = directBrowserGuidance();
            note.textContent = message;
            note.hidden = !message;
        });
    }

    function directConnectionErrorMessage(error, endpoint, permissionState = 'unavailable') {
        const code = error?.code || 'connection_failed';
        let url;
        try { url = new URL(endpoint); } catch (_error) { url = null; }
        const localName = Boolean(url?.hostname.endsWith('.local'));
        const hostedCleartext = location.protocol === 'https:' && url?.protocol === 'http:';
        if (code === 'local_network_denied' || permissionState === 'denied') {
            return 'Local network access is blocked. Allow it for this site in Chrome settings, then try again. On macOS, you may also need to allow Chrome in System Settings > Privacy & Security > Local Network.';
        }
        if (code === 'timeout') {
            return localName
                ? 'The device did not respond. Make sure it is powered on and connected to the same Wi-Fi network, then search again or enter its IP address.'
                : 'The device did not respond. Make sure it is powered on, connected to the same Wi-Fi network, and still uses this IP address.';
        }
        if (code === 'subprotocol_mismatch' || code === 'unsupported_version'
            || code === 'invalid_capabilities' || code === 'invalid_envelope') {
            return 'This device is not compatible with the current browser tools. Update its firmware, then try again.';
        }
        if (code === 'connection_failed' || code === 'closed') {
            if (directPageOriginKind() === 'other') {
                return 'The device may have rejected this page Origin. Use https://espectre.dev, https://test.espectre.dev, or a loopback development portal explicitly enabled in the firmware.';
            }
            if (directPageOriginKind() === 'loopback') {
                return 'A local HTTP portal does not require a Local network access prompt. Confirm that this is a development firmware with loopback Origins enabled, reflash if it predates any-port localhost support, close other ESPectre tabs, and retry.';
            }
            if (hostedCleartext && browserSupport.hostedDirect === 'unsupported') {
                return 'This browser cannot connect to ESPectre on your local network. Open this page in supported desktop Chrome.';
            }
            if (hostedCleartext && permissionState === 'prompt') {
                return 'Chrome is waiting for local network permission. Try again, allow access for this site, and keep the device on the same Wi-Fi network.';
            }
            const addressHelp = localName
                ? 'Search again or enter the device IP address. '
                : 'Check the device IP address. ';
            return `The browser could not connect to ESPectre. ${addressHelp}Make sure the device is powered on and connected to the same Wi-Fi network, close other ESPectre tabs, and try again.`;
        }
        console.warn('Local device connection failed:', error);
        return 'The browser could not connect to ESPectre. Check the device and try again.';
    }

    function setDirectConnectionHelp(message = '') {
        const help = directEndpointInput()?.closest('.device-connect-card')?.querySelector('.js-direct-help');
        if (!help) return;
        const copy = help.querySelector('.js-direct-help-copy');
        if (copy) copy.textContent = message;
        help.hidden = !message;
    }

    function setDirectConnectionStatus(message = '') {
        const status = directEndpointInput()?.closest('.device-connect-card')?.querySelector('.js-direct-status');
        if (!status) return;
        status.textContent = message;
        status.hidden = !message;
    }

    function directDiscoveryPanel(button) {
        return button?.closest('.device-connect-card')?.querySelector('.js-direct-discovery') || null;
    }

    function cancelDirectDiscovery({ clear = false } = {}) {
        directDiscoveryGeneration += 1;
        const client = directDiscoveryClient;
        directDiscoveryClient = null;
        client?.close();
        setDirectConnectionStatus();
        $$('.js-direct-discover').forEach((button) => {
            button.disabled = false;
            button.setAttribute('aria-disabled', 'false');
        });
        if (clear) {
            $$('.js-direct-discovery').forEach((panel) => {
                panel.hidden = true;
                panel.replaceChildren();
            });
        }
    }

    function discoveredPeerChipLabel(chip) {
        return String(chip || '').toUpperCase().replace(/^ESP32([A-Z]\d)$/, 'ESP32-$1');
    }

    function createDiscoveryDeviceButton({
        deviceId,
        displayDeviceId = deviceId,
        displayName,
        frontend,
        chip,
        endpoint = '',
        className = '',
        actionText = 'Connect →',
        ariaLabel
    }) {
        const button = document.createElement('button');
        button.type = 'button';
        button.className = `btn-ghost direct-discovery-device ${className}`.trim();
        button.dataset.deviceId = deviceId;
        if (endpoint) button.dataset.endpoint = endpoint;
        button.setAttribute('aria-label', ariaLabel);
        const heading = document.createElement('span');
        heading.className = 'direct-discovery-device-heading';
        const name = document.createElement('strong');
        name.className = 'direct-discovery-device-name';
        name.textContent = displayName;
        const action = document.createElement('span');
        action.className = 'direct-discovery-device-action';
        action.setAttribute('aria-hidden', 'true');
        action.textContent = actionText;
        heading.append(name, action);
        const metadata = document.createElement('span');
        metadata.className = 'direct-discovery-device-meta';
        for (const [label, value, valueClass = ''] of [
            ['Frontend', frontend || 'Unknown'],
            ['Hardware', chip || 'Unknown'],
            ['Device ID', displayDeviceId, 'mono']
        ]) {
            const field = document.createElement('span');
            field.className = 'direct-discovery-device-field';
            const fieldLabel = document.createElement('span');
            fieldLabel.className = 'direct-discovery-device-label';
            fieldLabel.textContent = label;
            const fieldValue = document.createElement('span');
            fieldValue.className = `direct-discovery-device-value ${valueClass}`.trim();
            fieldValue.textContent = value;
            field.append(fieldLabel, fieldValue);
            metadata.appendChild(field);
        }
        button.append(heading, metadata);
        return button;
    }

    function renderDiscoveredPeers(panel, result, { selectionRequired = false } = {}) {
        panel.replaceChildren();
        const summary = document.createElement('p');
        summary.className = 'direct-discovery-summary';
        summary.textContent = selectionRequired
            ? `${result.devices.length} matching devices found. Select one to connect.`
            : result.devices.length
            ? `${result.devices.length} local device${result.devices.length === 1 ? '' : 's'} found${result.truncated ? ' (partial result)' : ''}.`
            : 'No compatible devices answered. You can still enter a private IP address or device ID.';
        panel.appendChild(summary);
        if (!result.devices.length) return;
        const list = document.createElement('ul');
        list.className = 'direct-discovery-list';
        for (const peer of result.devices) {
            const item = document.createElement('li');
            const displayName = peer.name || `ESPectre ${peer.device_id.slice(-6)}`;
            const frontend = formatFrontendLabel(peer.frontend);
            const chip = discoveredPeerChipLabel(peer.chip);
            const shortId = peer.device_id.slice(-6);
            const button = createDiscoveryDeviceButton({
                deviceId: peer.device_id,
                displayDeviceId: shortId,
                displayName,
                frontend,
                chip,
                endpoint: peer.endpoints[0],
                ariaLabel: `Connect to ${displayName}, ${frontend}, ${chip}, device ID ${shortId}`
            });
            item.appendChild(button);
            list.appendChild(item);
        }
        panel.appendChild(list);
    }

    function directDiscoveryFailureMessage(error, permissionState) {
        if (error?.code === 'local_network_denied' || permissionState === 'denied') {
            return 'Local network access is blocked. Allow it for this site in Chrome settings, then search again.';
        }
        if (error?.code === 'unsupported_crypto') {
            return 'This browser cannot search for local devices. Enter the device IP address instead.';
        }
        if (error?.code === 'unsupported_capability') {
            return 'This device does not support browser search. Enter its IP address instead.';
        }
        if (error?.code === 'timeout') {
            return 'No device responded in time. Make sure ESPectre is powered on and connected to the same Wi-Fi network, then search again or enter its IP address.';
        }
        if (error?.code === 'invalid_envelope' || error?.code === 'unsupported_version') {
            return 'Search found an incompatible device. Update its firmware, or enter the IP address of another ESPectre device.';
        }
        if (error?.code === 'invalid_peer_result' || error?.code === 'frame_too_large') {
            return 'Search found a device it could not recognize. Enter the IP address of the ESPectre device you trust.';
        }
        if (error?.code === 'connection_failed') {
            return 'Search could not reach any devices. Make sure ESPectre is on the same Wi-Fi network, or enter its IP address.';
        }
        return 'Device search is unavailable on this network. Enter the ESPectre IP address instead.';
    }

    async function queryLocalPeers(onProgress = () => {}) {
        const client = makeDirectClient(DirectProtocolClient.createDiscoveryEndpoint());
        directDiscoveryClient = client;
        try {
            onProgress('Looking for compatible ESPectre devices…');
            try {
                return await client.discoverPeersBootstrap();
            } catch (error) {
                error.discoveryStage = 'query';
                throw error;
            }
        } finally {
            if (directDiscoveryClient === client) directDiscoveryClient = null;
            client.close();
        }
    }

    async function discoverLocalPeers(button) {
        cancelDirectDiscovery({ clear: true });
        const panel = directDiscoveryPanel(button);
        if (!panel) return;
        const generation = directDiscoveryGeneration;
        panel.hidden = false;
        panel.textContent = 'Starting device search…';
        button.disabled = true;
        button.setAttribute('aria-disabled', 'true');
        track('local_discovery', { tool_name: activeToolName(), result: 'attempt' });
        try {
            const result = await queryLocalPeers((message) => {
                if (generation === directDiscoveryGeneration) panel.textContent = message;
            });
            if (generation !== directDiscoveryGeneration) return;
            setDirectConnectionHelp();
            renderDiscoveredPeers(panel, result);
            track('local_discovery', {
                tool_name: activeToolName(), result: result.devices.length ? 'success' : 'empty',
                device_count: result.devices.length, truncated: result.truncated
            });
        } catch (error) {
            if (generation !== directDiscoveryGeneration) return;
            const permissionState = await localNetworkAccessState();
            if (generation !== directDiscoveryGeneration) return;
            panel.textContent = directDiscoveryFailureMessage(error, permissionState);
            track('local_discovery', {
                tool_name: activeToolName(), result: 'failure', error_type: errorType(error)
            });
        } finally {
            if (generation === directDiscoveryGeneration) {
                button.disabled = false;
                button.setAttribute('aria-disabled', 'false');
            }
        }
    }

    async function localNetworkAccessState() {
        const detectState = window.ESPectreBrowserSupport.localNetworkAccessState;
        return typeof detectState === 'function'
            ? detectState(navigator) : 'unavailable';
    }

    function cancelDirectReconnect() {
        clearTimeout(directReconnectTimer);
        directReconnectTimer = 0;
        directReconnectAttempt = 0;
    }

    function scheduleDirectReconnect(client) {
        if (directClient !== client || conn.mode !== 'direct' || directReconnectTimer) return;
        if (pendingConfigVerification?.waitForReconnect) {
            pendingConfigVerification.observedDisconnect = true;
        }
        const extendedVerification = pendingConfigVerification?.waitForReconnect === true
            && Date.now() < pendingConfigVerification.deadlineAt;
        if (directReconnectAttempt >= DIRECT_RECONNECT_DELAYS_MS.length && !extendedVerification) {
            if (pendingConfigVerification?.waitForReconnect) {
                finishConfigVerification(
                    'unconfirmed', 'VerificationTimeout', pendingConfigVerification.timeoutMessage);
            }
            directClient = null;
            teardownConnection('reconnect_failed');
            toast('The device disconnected. Enter its address to reconnect.');
            return;
        }
        const delay = DIRECT_RECONNECT_DELAYS_MS[
            Math.min(directReconnectAttempt++, DIRECT_RECONNECT_DELAYS_MS.length - 1)
        ];
        setStatus('connecting');
        directReconnectTimer = setTimeout(async () => {
            directReconnectTimer = 0;
            if (directClient !== client || conn.mode !== 'direct') return;
            let session;
            try {
                await client.connect({ timeoutMs: 5000 });
                session = directResourceSession(client);
                await client.handshake({ timeoutMs: 5000 });
                if (directResourceSessions.get(client) !== session) return;
                if (directClient !== client || conn.mode !== 'direct') {
                    client.close();
                    return;
                }
                if (!await refreshDirectDevice({ client })) return;
                if (directClient !== client || !client.connected) return;
                directReconnectAttempt = 0;
                setStatus('connected');
                toast('Device reconnected.');
                if (pendingConfigVerification) requestConfigVerification();
            } catch (error) {
                if (directClient !== client
                        || (session && directResourceSessions.get(client) !== session)) return;
                client.close();
                const permissionState = await localNetworkAccessState();
                if (directClient !== client) return;
                if (error?.code === 'local_network_denied' || permissionState === 'denied') {
                    directClient = null;
                    teardownConnection('local_network_denied');
                    const message = directConnectionErrorMessage(error, client.endpoint, permissionState);
                    setDirectConnectionHelp(message);
                    toast(message);
                    return;
                }
                scheduleDirectReconnect(client);
            }
        }, delay);
    }

    async function refreshDirectDevice({ client = directClient } = {}) {
        if (!client?.connected || directClient !== client) return false;
        const session = directResourceSession(client);
        const current = () => directClient === client && client.connected
            && directResourceSessions.get(client) === session;
        const resources = ['device', 'health', 'sensing'];
        // OTA state feeds the shared firmware notice, even outside Device settings.
        if (client.capabilities?.resources?.includes('ota')) resources.push('ota');
        if (activeToolName() === 'configure') {
            resources.push('wifi');
            if (client.capabilities?.resources?.includes('mqtt')) resources.push('mqtt');
            if (directSupportsCommand('read_diagnostics')) resources.push('diagnostics');
        }
        // Keep local-network requests serial. Chrome may still be resolving its
        // Local Network Access grant when the Direct stream and handshake have
        // just completed, and rejects a concurrent fan-out before CORS runs.
        for (const resource of resources) {
            if (!current()) return false;
            const settingsResource = ['wifi', 'mqtt', 'diagnostics'].includes(resource);
            if (settingsResource && activeToolName() !== 'configure') continue;
            if (session.snapshots.has(resource)) continue;
            if (!session.pending.has(resource)) {
                const request = client.request('get', resource).then((snapshot) => {
                    if (settingsResource && activeToolName() !== 'configure') return;
                    // An SSE snapshot received during the GET is newer.
                    if (current() && !session.snapshots.has(resource)) session.snapshots.set(resource, snapshot);
                }).finally(() => session.pending.delete(resource));
                session.pending.set(resource, request);
            }
            try {
                await session.pending.get(resource);
            } catch (error) {
                if (!current()) return false;
                throw error;
            }
        }
        if (!current()) return false;
        const info = session.snapshots.get('device');
        const status = session.snapshots.get('health');
        const sensing = session.snapshots.get('sensing');
        const wifi = session.snapshots.get('wifi');
        const mqtt = session.snapshots.get('mqtt');
        const diagnostics = session.snapshots.get('diagnostics');
        const otaStatus = session.snapshots.get('ota');
        applySysinfo({
            ...directCapabilitiesSnapshot(client.capabilities),
            ...info,
            ...status,
            ...(diagnostics || {})
        });
        applyDirectConfig({ device: info, runtime: sensing, wifi, mqtt });
        if (otaStatus) applyOtaStatus(otaStatus);
        return true;
    }

    async function resolveDiscoveredTarget(target, input) {
        const generation = directDiscoveryGeneration;
        const description = target.deviceId
            ? `device ID ${target.deviceId}`
            : target.shortId ? `device ID …${target.shortId}` : `device name “${target.search}”`;
        setDirectConnectionHelp();
        setDirectConnectionStatus(`Looking for ${description} on this Wi-Fi network.`);
        track('local_discovery', { tool_name: activeToolName(), result: 'attempt' });
        let result;
        try {
            result = await queryLocalPeers();
        } catch (error) {
            if (generation !== directDiscoveryGeneration) return null;
            const permissionState = await localNetworkAccessState();
            if (generation !== directDiscoveryGeneration) return null;
            throw new Error(directDiscoveryFailureMessage(error, permissionState), { cause: error });
        } finally {
            if (generation === directDiscoveryGeneration) setDirectConnectionStatus();
        }
        if (generation !== directDiscoveryGeneration) return null;
        const query = target.search.toLowerCase();
        const matches = result.devices.filter((peer) => target.deviceId
            ? peer.device_id === target.deviceId
            : target.shortId ? peer.device_id.endsWith(target.shortId)
                : [peer.name, peer.instance].some((value) => String(value || '').toLowerCase().includes(query)));
        track('local_discovery', {
            tool_name: activeToolName(), result: matches.length === 1 ? 'success' : (matches.length ? 'multiple' : 'empty'),
            device_count: result.devices.length, truncated: result.truncated
        });
        if (matches.length === 0) {
            throw new Error(`No matching ${description} was found. Search again, or enter the device IP address.`);
        }
        if (matches.length > 1) {
            const panel = directDiscoveryPanel(input);
            if (panel) {
                panel.hidden = false;
                renderDiscoveredPeers(panel, { ...result, devices: matches }, { selectionRequired: true });
            }
            setDirectConnectionHelp();
            return null;
        }
        const peer = matches[0];
        return { display: peer.device_id, endpoint: peer.endpoints[0], deviceId: peer.device_id };
    }

    async function connectDirect({ endpoint, deviceId, openView } = {}) {
        if (directClient || conn.status !== 'disconnected') return;
        cancelDirectDiscovery();
        const generation = directDiscoveryGeneration;
        const input = directEndpointInput();
        let target;
        try {
            if (endpoint) {
                const normalizedEndpoint = DirectProtocolClient.normalizeEndpoint(endpoint);
                target = {
                    display: deviceId || directTargetForEndpoint(normalizedEndpoint),
                    endpoint: normalizedEndpoint,
                    deviceId: deviceId || ''
                };
            } else {
                target = parseDirectTarget(input?.value || '');
                if (target.search) target = await resolveDiscoveredTarget(target, input);
                if (!target) return;
            }
        } catch (error) {
            if (generation !== directDiscoveryGeneration) return;
            setDirectConnectionHelp(error.message);
            toast(error.message);
            input?.setAttribute('aria-invalid', 'true');
            return;
        }
        if (generation !== directDiscoveryGeneration
            || directClient || conn.status !== 'disconnected') return;
        let normalizedEndpoint;
        try {
            normalizedEndpoint = DirectProtocolClient.normalizeEndpoint(target.endpoint);
        } catch (error) {
            console.warn('Invalid local device endpoint:', error);
            toast('This device address is not valid. Enter a private IP address, device ID, or device name.');
            input?.setAttribute('aria-invalid', 'true');
            return;
        }
        input?.removeAttribute('aria-invalid');
        setDirectConnectionHelp();
        rememberConnectionOrigin();
        track('tool_connection', {
            ...connectionParams(), transport: 'direct_http', result: 'attempt'
        });
        setStatus('connecting');
        let client;
        let session;
        try {
            client = makeDirectClient(normalizedEndpoint);
            directClient = client;
            cancelDirectReconnect();
            await client.connect();
            session = directResourceSession(client);
            await client.handshake();
            if (directClient !== client || directResourceSessions.get(client) !== session) return;
            conn.mode = 'direct';
            conn.endpoint = normalizedEndpoint;
            conn.deviceBannerSub = normalizedEndpoint;
            conn.connectedAt = Date.now();
            syncDirectEndpointInputs(target.display);
            if (!await refreshDirectDevice({ client })) return;
            if ((openView || (route === 'tool-monitor' ? 'live' : 'connectivity')) === 'live'
                && directSupportsCommand('update_sensing')
                && session.snapshots.get('sensing')?.enabled !== true) {
                await client.request('patch', 'sensing', { enabled: true });
            }
            if (directClient !== client || directResourceSessions.get(client) !== session) return;
            setStatus('connected');
            setDirectConnectionHelp();
            if (route === 'tool-raw-csi') {
                if (typeof window.rawCsiUseConnection === 'function') {
                    window.rawCsiUseConnection();
                }
            } else if (!LIVE_EXPERIENCE_ROUTES.has(route)) {
                const view = openView || (route === 'tool-monitor' ? 'live' : 'connectivity');
                setDeviceView(view);
            }
            track('tool_connection', {
                ...connectionParams(), transport: 'direct_http', result: 'success'
            });
            if (conn.toolName === 'configure') markToolReady('info');
            if (pendingLiveDestination) completeLiveConnectionNavigation();
        } catch (error) {
            let connectionError = error;
            if (directClient !== client
                    || (session && directResourceSessions.get(client) !== session)) return;
            client?.close();
            directClient = null;
            setStatus('disconnected');
            if (target.discoveryFallback) {
                try {
                    const discoveredTarget = await resolveDiscoveredTarget({
                        deviceId: target.deviceId,
                        search: target.deviceId,
                        shortId: ''
                    }, input);
                    if (discoveredTarget) {
                        return connectDirect({
                            endpoint: discoveredTarget.endpoint,
                            deviceId: discoveredTarget.deviceId,
                            openView
                        });
                    }
                    return;
                } catch (fallbackError) {
                    connectionError = fallbackError;
                }
            }
            track('tool_connection', {
                ...connectionParams(), transport: 'direct_http', result: 'failure',
                error_type: errorType(connectionError)
            });
            const message = directConnectionErrorMessage(
                connectionError, normalizedEndpoint, await localNetworkAccessState());
            setDirectConnectionHelp(message);
            toast(message);
        }
    }
