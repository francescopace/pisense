/*
 * ESPectre Direct HTTP Client
 *
 * Dependency-free client for the versioned local HTTP API. It owns endpoint
 * validation, resource requests, incremental SSE parsing, and device
 * events; UI policy remains with callers.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

(function () {
    'use strict';

    const PROTOCOL_VERSION = '1.0';
    const DNS_SD_SCHEMA_VERSION = 1;
    // Low 16 bits of U+1F47B GHOST (0xF47B), the ESPectre service marker.
    const DEFAULT_PORT = 62587;
    const REQUEST_PATH = '/espectre/v1';
    const EVENTS_PATH = '/espectre/v1/events';
    const RAW_PATH = '/espectre/v1/csi';
    const MAX_REQUEST_BYTES = 4096;
    const MAX_RESPONSE_BYTES = 8192;
    const DEFAULT_TIMEOUT_MS = 8000;
    const PEER_DISCOVERY_TIMEOUT_MS = 10000;
    const PEER_DISCOVERY_MAX_DEVICES = 8;
    const PEER_DISCOVERY_MAX_ADDRESSES = 2;
    const DISCOVERY_NONCE_BYTES = 12;
    const DISCOVERY_HOST_PREFIX = 'espectre-devices-';
    const EVENTS = Object.freeze(['open', 'close', 'event', 'protocol-error']);

    class ESPectreDirectError extends Error {
        constructor(message, code = 'client_error') {
            super(message);
            this.name = 'ESPectreDirectError';
            this.code = code;
        }
    }

    const RAW_HTTP_MAGIC = 0x52505345;
    const RAW_HTTP_PROTOCOL_VERSION = 1;
    const RAW_HTTP_PREFIX_BYTES = 60;
    const RAW_HTTP_MAX_BUFFER_BYTES = 64 * 1024;
    const RAW_CSI_V8_HEADER_BYTES = 64;
    const RAW_CSI_MAX_PAYLOAD_BYTES = 512;

    class ESPectreRawCsiParser {
        #buffer = new Uint8Array(0);
        #sessionBytes;
        #streamSequence = 0n;
        #freshRecordTotal = 0n;
        #rawDropTotal = 0n;
        #sendBackpressureTotal = 0n;

        constructor(sessionId = null) {
            if (typeof sessionId === 'string') {
                if (!/^[0-9a-f]{32}$/.test(sessionId)) {
                    throw new ESPectreDirectError('Raw CSI session ID must be 32 lowercase hexadecimal characters.', 'invalid_raw_session');
                }
                this.#sessionBytes = new Uint8Array(16);
                for (let index = 0; index < this.#sessionBytes.length; index += 1) {
                    this.#sessionBytes[index] = Number.parseInt(sessionId.slice(index * 2, index * 2 + 2), 16);
                }
            } else if (sessionId instanceof Uint8Array && sessionId.length === 16) {
                this.#sessionBytes = sessionId.slice();
            } else if (sessionId == null) {
                this.#sessionBytes = null;
            } else {
                throw new ESPectreDirectError('Raw CSI session ID is invalid.', 'invalid_raw_session');
            }
        }

        get streamSequence() { return this.#streamSequence; }
        get freshRecordTotal() { return this.#freshRecordTotal; }
        get rawDropTotal() { return this.#rawDropTotal; }
        get sendBackpressureTotal() { return this.#sendBackpressureTotal; }
        get bufferedBytes() { return this.#buffer.length; }

        append(chunk) {
            if (!(chunk instanceof Uint8Array)) {
                throw new ESPectreDirectError('Raw HTTP stream requires a byte array.', 'invalid_raw_frame');
            }
            const records = [];
            for (let offset = 0; offset < chunk.length;) {
                const end = Math.min(chunk.length,
                    offset + RAW_HTTP_MAX_BUFFER_BYTES - this.#buffer.length);
                records.push(...this.#appendChunk(chunk.subarray(offset, end)));
                offset = end;
            }
            return records;
        }

        #appendChunk(chunk) {
            const combined = new Uint8Array(this.#buffer.length + chunk.length);
            combined.set(this.#buffer);
            combined.set(chunk, this.#buffer.length);
            this.#buffer = combined;
            const records = [];
            while (this.#buffer.length >= RAW_HTTP_PREFIX_BYTES) {
                const prefix = new DataView(
                    this.#buffer.buffer, this.#buffer.byteOffset, this.#buffer.byteLength);
                if (prefix.getUint32(0, true) !== RAW_HTTP_MAGIC
                    || prefix.getUint8(4) !== RAW_HTTP_PROTOCOL_VERSION
                    || prefix.getUint8(5) !== 8
                    || prefix.getUint16(6, true) !== RAW_HTTP_PREFIX_BYTES) {
                    throw new ESPectreDirectError('Raw HTTP stream lost frame alignment.', 'invalid_raw_frame');
                }
                const recordLength = prefix.getUint16(32, true);
                if (recordLength < RAW_CSI_V8_HEADER_BYTES
                    || recordLength > RAW_CSI_V8_HEADER_BYTES + RAW_CSI_MAX_PAYLOAD_BYTES) {
                    throw new ESPectreDirectError('Raw HTTP frame has an invalid record length.', 'invalid_raw_frame');
                }
                const frameLength = RAW_HTTP_PREFIX_BYTES + recordLength;
                if (this.#buffer.length < frameLength) break;
                records.push(this.#consumeFrame(this.#buffer.subarray(0, frameLength)));
                this.#buffer = this.#buffer.slice(frameLength);
            }
            return records;
        }

        #consumeFrame(frame) {
            const prefix = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
            const session = frame.subarray(8, 24);
            if (this.#sessionBytes === null) this.#sessionBytes = session.slice();
            const sessionMatches = session.length === this.#sessionBytes.length
                && session.every((value, index) => value === this.#sessionBytes[index]);
            const streamSequence = prefix.getBigUint64(24, true);
            const flags = prefix.getUint16(34, true);
            const freshRecordTotal = prefix.getBigUint64(36, true);
            const rawDropTotal = prefix.getBigUint64(44, true);
            const sendBackpressureTotal = prefix.getBigUint64(52, true);
            if (!sessionMatches || flags !== 0 || streamSequence <= this.#streamSequence
                || freshRecordTotal !== this.#freshRecordTotal + 1n
                || rawDropTotal < this.#rawDropTotal
                || sendBackpressureTotal < this.#sendBackpressureTotal) {
                throw new ESPectreDirectError('Raw HTTP frame counters or session are invalid.', 'invalid_raw_frame');
            }

            const record = frame.subarray(RAW_HTTP_PREFIX_BYTES);
            const view = new DataView(record.buffer, record.byteOffset, record.byteLength);
            const recordFlags = view.getUint8(5);
            const subcarriers = view.getUint16(10, true);
            const csiLength = view.getUint16(12, true);
            const saturatedSequence = streamSequence > 0xFFFFFFFFn
                ? 0xFFFFFFFF : Number(streamSequence);
            const saturatedFresh = freshRecordTotal > 0xFFFFFFFFn
                ? 0xFFFFFFFF : Number(freshRecordTotal);
            if (view.getUint16(0, true) !== 0x4353
                || view.getUint8(2) !== 8
                || view.getUint8(3) !== RAW_CSI_V8_HEADER_BYTES
                || (recordFlags & 0x08) === 0 || (recordFlags & 0xF0) !== 0
                || csiLength !== subcarriers * 2
                || RAW_CSI_V8_HEADER_BYTES + csiLength !== record.byteLength
                || view.getUint32(6, true) !== saturatedSequence
                || view.getBigUint64(45, true) !== sendBackpressureTotal
                || view.getUint32(53, true) !== saturatedFresh
                || view.getUint32(57, true) !== saturatedSequence) {
                throw new ESPectreDirectError('Raw HTTP frame contains an invalid CSI V8 record.', 'invalid_raw_record');
            }

            this.#streamSequence = streamSequence;
            this.#freshRecordTotal = freshRecordTotal;
            this.#rawDropTotal = rawDropTotal;
            this.#sendBackpressureTotal = sendBackpressureTotal;
            return {
                record,
                streamSequence,
                freshRecordTotal,
                rawDropTotal,
                sendBackpressureTotal
            };
        }
    }

    function isLocalHostname(hostname) {
        const host = hostname.toLowerCase().replace(/^\[|\]$/g, '');
        if (host === 'localhost' || host.endsWith('.local')) return true;
        if (/^10\./.test(host) || /^192\.168\./.test(host)) return true;
        const ipv4 = host.match(/^172\.(\d{1,3})\./);
        if (ipv4 && Number(ipv4[1]) >= 16 && Number(ipv4[1]) <= 31) return true;
        return host === '::1' || /^fe[89ab][0-9a-f]:/i.test(host) || /^f[cd][0-9a-f]{2}:/i.test(host);
    }

    function isLocalPeerAddress(address) {
        if (typeof address !== 'string' || !address || address.includes('%')) return false;
        if (address.includes('.')) {
            const octets = address.split('.');
            if (octets.length !== 4 || octets.some((octet) => !/^(0|[1-9][0-9]{0,2})$/.test(octet))) return false;
            const values = octets.map(Number);
            if (values.some((value) => value > 255)) return false;
            return values[0] === 10
                || (values[0] === 172 && values[1] >= 16 && values[1] <= 31)
                || (values[0] === 192 && values[1] === 168);
        }
        if (!address.includes(':') || address === '::1') return false;
        try { new URL(`http://[${address}]/`); } catch (_error) { return false; }
        return /^fe[89ab][0-9a-f]:/i.test(address) || /^f[cd][0-9a-f]{2}:/i.test(address);
    }

    function normalizeEndpoint(value) {
        if (typeof value !== 'string' || !value.trim()) {
            throw new ESPectreDirectError('Enter a device IP address, .local hostname, or HTTP URL.', 'invalid_endpoint');
        }
        const input = value.trim();
        const explicitScheme = /^[a-z][a-z0-9+.-]*:\/\//i.test(input);
        let url;
        try { url = new URL(explicitScheme ? input : `http://${input}`); } catch (_error) {
            throw new ESPectreDirectError('The device endpoint is not a valid URL.', 'invalid_endpoint');
        }
        if (!['http:', 'https:'].includes(url.protocol)) {
            throw new ESPectreDirectError('The device endpoint must use http:// or https://.', 'invalid_scheme');
        }
        if (url.username || url.password || url.search || url.hash) {
            throw new ESPectreDirectError('The device endpoint cannot contain credentials, a query, or a fragment.', 'invalid_endpoint');
        }
        if (!isLocalHostname(url.hostname)) {
            throw new ESPectreDirectError('Use a private IP address, localhost, or a .local device name.', 'non_local_endpoint');
        }
        if (!url.port) url.port = String(DEFAULT_PORT);
        if (url.pathname !== '/' && url.pathname !== REQUEST_PATH) {
            throw new ESPectreDirectError(`The Direct endpoint path must be ${REQUEST_PATH}.`, 'invalid_path');
        }
        url.pathname = REQUEST_PATH;
        return url.toString();
    }

    function endpointWithPath(endpoint, path) {
        const url = new URL(endpoint);
        url.pathname = path;
        return url.toString();
    }

    function createDiscoveryEndpoint(randomSource = globalThis.crypto) {
        if (!randomSource || typeof randomSource.getRandomValues !== 'function') {
            throw new ESPectreDirectError('Web Crypto is required for local Auto-discovery.', 'unsupported_crypto');
        }
        const bytes = new Uint8Array(DISCOVERY_NONCE_BYTES);
        randomSource.getRandomValues(bytes);
        const nonce = [...bytes].map((value) => value.toString(16).padStart(2, '0')).join('');
        return normalizeEndpoint(`${DISCOVERY_HOST_PREFIX}${nonce}.local`);
    }

    async function localNetworkAccessState() {
        const detectState = window.ESPectreBrowserSupport?.localNetworkAccessState;
        return typeof detectState === 'function'
            ? detectState(globalThis.navigator) : 'unavailable';
    }

    async function localFetch(url, options) {
        if (await localNetworkAccessState() === 'denied') {
            throw new ESPectreDirectError(
                'Local network access is blocked for this site.',
                'local_network_denied'
            );
        }
        try {
            return await globalThis.fetch(url, {
                ...options,
                cache: 'no-store',
                targetAddressSpace: 'local'
            });
        } catch (error) {
            if (!options.signal?.aborted && await localNetworkAccessState() === 'denied') {
                throw new ESPectreDirectError(
                    'Local network access is blocked for this site.',
                    'local_network_denied'
                );
            }
            throw error;
        }
    }

    function parseObject(text) {
        let data;
        try { data = JSON.parse(text); } catch (_error) {
            throw new ESPectreDirectError('Direct payload must be valid JSON.', 'invalid_json');
        }
        if (!data || typeof data !== 'object' || Array.isArray(data)) {
            throw new ESPectreDirectError('Direct payload must be a JSON object.', 'invalid_envelope');
        }
        return data;
    }

    async function readBoundedResponseText(response, maximumBytes) {
        if (!response.body || typeof response.body.getReader !== 'function') {
            const text = await response.text();
            if (new TextEncoder().encode(text).byteLength > maximumBytes) {
                throw new ESPectreDirectError(
                    `Direct response exceeds the ${maximumBytes}-byte limit.`,
                    'frame_too_large'
                );
            }
            return text;
        }

        const reader = response.body.getReader();
        const decoder = new TextDecoder();
        let receivedBytes = 0;
        let text = '';
        try {
            while (true) {
                const { value, done } = await reader.read();
                if (done) {
                    text += decoder.decode();
                    return text;
                }
                if (!(value instanceof Uint8Array)) {
                    throw new ESPectreDirectError(
                        'Direct response body is not a byte stream.',
                        'invalid_envelope'
                    );
                }
                receivedBytes += value.byteLength;
                if (receivedBytes > maximumBytes) {
                    try { await reader.cancel('response too large'); } catch (_error) { /* already closed */ }
                    throw new ESPectreDirectError(
                        `Direct response exceeds the ${maximumBytes}-byte limit.`,
                        'frame_too_large'
                    );
                }
                text += decoder.decode(value, { stream: true });
            }
        } finally {
            try { reader.releaseLock(); } catch (_error) { /* already released */ }
        }
    }

    function validText(value, maximum, { empty = false, token = false } = {}) {
        if (typeof value !== 'string' || (!empty && !value) || value.length > maximum) return false;
        if (![...value].every((character) => character >= ' ' && character <= '~')) return false;
        return !token || /^[A-Za-z0-9_-]+$/.test(value);
    }

    function validatePeerDiscoveryResult(result) {
        if (!result || typeof result !== 'object' || Array.isArray(result)
            || result.schema_version !== 2
            || !Number.isInteger(result.elapsed_ms) || result.elapsed_ms < 0 || result.elapsed_ms > 10000
            || !['complete', 'timeout'].includes(result.status)
            || typeof result.truncated !== 'boolean'
            || !Number.isInteger(result.rejected_results) || result.rejected_results < 0
            || !Array.isArray(result.devices) || result.devices.length > PEER_DISCOVERY_MAX_DEVICES) {
            throw new ESPectreDirectError('Device returned an invalid peer discovery result.', 'invalid_peer_result');
        }
        const identities = new Set();
        const devices = result.devices.map((peer) => {
            const capabilities = peer?.capabilities;
            const addresses = peer?.addresses;
            const valid = peer && typeof peer === 'object' && !Array.isArray(peer)
                && /^[0-9a-f]{16}$/.test(peer.device_id)
                && validText(peer.instance, 63)
                && validText(peer.hostname, 63, { token: true })
                && validText(peer.name, 63, { empty: true })
                && ['native', 'esphome', 'matter', 'micro'].includes(peer.frontend)
                && peer.dns_sd_schema_version === DNS_SD_SCHEMA_VERSION
                && peer.protocol_version === PROTOCOL_VERSION
                && peer.transport === 'http'
                && peer.path === REQUEST_PATH
                && validText(peer.firmware, 48)
                && validText(peer.chip, 16, { token: true })
                && Number.isInteger(peer.port) && peer.port > 0 && peer.port <= 65535
                && Array.isArray(capabilities) && capabilities.length > 0 && capabilities.length <= 8
                && capabilities.every((capability) => validText(capability, 32, { token: true }))
                && new Set(capabilities).size === capabilities.length
                && Array.isArray(addresses) && addresses.length > 0
                && addresses.length <= PEER_DISCOVERY_MAX_ADDRESSES
                && addresses.every(isLocalPeerAddress);
            if (!valid || identities.has(peer?.device_id)) {
                throw new ESPectreDirectError('Device returned an invalid or duplicate peer.', 'invalid_peer_result');
            }
            identities.add(peer.device_id);
            const endpoints = addresses.map((address) => {
                const host = address.includes(':') ? `[${address}]` : address;
                return normalizeEndpoint(`http://${host}:${peer.port}${peer.path}`);
            });
            return Object.freeze({ ...peer, capabilities: [...capabilities], addresses: [...addresses], endpoints });
        });
        return Object.freeze({ ...result, devices });
    }

    class ESPectreDirectClient {
        static get VERSION() { return '2.0.0'; }
        static get PROTOCOL_VERSION() { return PROTOCOL_VERSION; }
        static get DEFAULT_PORT() { return DEFAULT_PORT; }
        static get ENDPOINT_PATH() { return REQUEST_PATH; }
        static get EVENTS_PATH() { return EVENTS_PATH; }
        static get RAW_PATH() { return RAW_PATH; }
        static get MAX_FRAME_BYTES() { return MAX_REQUEST_BYTES; }
        static get MAX_REQUEST_FRAME_BYTES() { return MAX_REQUEST_BYTES; }
        static get MAX_RESPONSE_FRAME_BYTES() { return MAX_RESPONSE_BYTES; }
        static get EVENTS() { return EVENTS; }
        static normalizeEndpoint(value) { return normalizeEndpoint(value); }
        static createDiscoveryEndpoint(randomSource) { return createDiscoveryEndpoint(randomSource); }
        static validatePeerDiscoveryResult(value) { return validatePeerDiscoveryResult(value); }

        #endpoint;
        #listeners = new Map();
        #compatible = false;
        #capabilities = null;
        #connected = false;
        #closing = false;
        #eventController = null;
        #requestControllers = new Set();

        constructor(endpoint) { this.#endpoint = normalizeEndpoint(endpoint); }

        get endpoint() { return this.#endpoint; }
        get eventsEndpoint() { return endpointWithPath(this.#endpoint, EVENTS_PATH); }
        get rawEndpoint() { return endpointWithPath(this.#endpoint, RAW_PATH); }
        get connected() { return this.#connected; }
        get compatible() { return this.#compatible; }
        get capabilities() { return this.#capabilities; }

        on(event, handler) {
            if (!EVENTS.includes(event)) throw new ESPectreDirectError(`Unknown Direct event ${String(event)}.`);
            if (typeof handler !== 'function') throw new ESPectreDirectError('Event handler must be a function.');
            if (!this.#listeners.has(event)) this.#listeners.set(event, new Set());
            this.#listeners.get(event).add(handler);
            return () => this.off(event, handler);
        }

        off(event, handler) { this.#listeners.get(event)?.delete(handler); }

        #emit(event, ...args) {
            for (const handler of [...(this.#listeners.get(event) || [])]) {
                try { handler(...args); } catch (error) { console.error(`Direct ${event} handler failed:`, error); }
            }
        }

        async connect({ timeoutMs = DEFAULT_TIMEOUT_MS } = {}) {
            if (this.#eventController) throw new ESPectreDirectError('Direct client is already active.');
            if (typeof globalThis.fetch !== 'function') {
                throw new ESPectreDirectError('Streaming fetch is not available in this browser.', 'unsupported');
            }
            this.#closing = false;
            const controller = new AbortController();
            this.#eventController = controller;
            const timer = setTimeout(() => controller.abort('connection timeout'), timeoutMs);
            let response;
            try {
                response = await localFetch(this.eventsEndpoint, {
                    method: 'GET',
                    headers: { Accept: 'text/event-stream' },
                    signal: controller.signal
                });
            } catch (error) {
                if (this.#eventController === controller) this.#eventController = null;
                if (error instanceof ESPectreDirectError) throw error;
                throw new ESPectreDirectError(
                    controller.signal.aborted ? 'Timed out connecting to the local device.' : `Direct HTTP connection failed: ${error.message}`,
                    controller.signal.aborted ? 'timeout' : 'connection_failed'
                );
            } finally {
                clearTimeout(timer);
            }
            if (!response.ok || !response.body || typeof response.body.getReader !== 'function') {
                this.#eventController = null;
                controller.abort();
                throw new ESPectreDirectError(`Direct event stream returned HTTP ${response.status}.`, 'connection_failed');
            }
            this.#connected = true;
            this.#emit('open');
            this.#pumpEvents(response.body.getReader(), controller);
        }

        async #pumpEvents(reader, controller) {
            const decoder = new TextDecoder();
            let buffer = '';
            try {
                while (!controller.signal.aborted) {
                    const { value, done } = await reader.read();
                    if (done) break;
                    buffer += decoder.decode(value, { stream: true });
                    let boundary;
                    while ((boundary = buffer.match(/\r?\n\r?\n/))) {
                        const block = buffer.slice(0, boundary.index);
                        buffer = buffer.slice(boundary.index + boundary[0].length);
                        this.#ingestSseBlock(block);
                    }
                    if (new TextEncoder().encode(buffer).byteLength > MAX_RESPONSE_BYTES) {
                        throw new ESPectreDirectError(
                            'Direct event exceeds the 8192-byte limit.',
                            'frame_too_large'
                        );
                    }
                }
            } catch (error) {
                if (!controller.signal.aborted) {
                    this.#emit('protocol-error', error);
                    try { await reader.cancel?.('protocol error'); } catch (_error) { /* already closed */ }
                }
            } finally {
                try { reader.releaseLock(); } catch (_error) { /* already released */ }
                if (this.#eventController === controller) {
                    this.#eventController = null;
                    const expected = this.#closing || controller.signal.aborted;
                    this.#connected = false;
                    this.#compatible = false;
                    this.#capabilities = null;
                    for (const request of this.#requestControllers) request.abort('event stream ended');
                    this.#requestControllers.clear();
                    this.#emit('close', { code: 0, reason: expected ? 'client closed' : 'event stream ended', expected });
                }
            }
        }

        #ingestSseBlock(block) {
            if (!block || block.startsWith(':')) return;
            let eventName = '';
            const dataLines = [];
            for (const line of block.split(/\r?\n/)) {
                if (line.startsWith('event:')) eventName = line.slice(6).trim();
                else if (line.startsWith('data:')) dataLines.push(line.slice(5).trimStart());
            }
            if (!eventName || !dataLines.length) return;
            try {
                const text = dataLines.join('\n');
                if (new TextEncoder().encode(text).byteLength > MAX_RESPONSE_BYTES) {
                    throw new ESPectreDirectError('Direct event exceeds the 8192-byte limit.', 'frame_too_large');
                }
                const event = parseObject(text);
                this.#emit('event', eventName, event);
            } catch (error) {
                this.#emit('protocol-error', error);
            }
        }

        async handshake(options = {}) {
            const result = await this.request('get', 'capabilities', {}, { ...options, allowBeforeHandshake: true });
            if (!result || typeof result !== 'object' || Array.isArray(result)
                || result.protocol_version !== PROTOCOL_VERSION
                || !Array.isArray(result.operations)
                || result.operations.some((item) => !item || typeof item.name !== 'string'
                    || !/^[A-Za-z0-9_.-]{1,64}$/.test(item.name))) {
                throw new ESPectreDirectError('Device returned invalid capabilities.', 'invalid_capabilities');
            }
            this.#compatible = true;
            this.#capabilities = result;
            return result;
        }

        async request(method, resource, data = {}, options = {}) {
            if (!this.connected) throw new ESPectreDirectError('Direct HTTP is not connected.', 'not_connected');
            return this.#sendRequest(method, resource, data, options);
        }

        async #sendRequest(method, resource, data = {}, {
            timeoutMs = DEFAULT_TIMEOUT_MS,
            allowBeforeHandshake = false
        } = {}) {
            const httpMethod = String(method).toUpperCase();
            if (!['GET', 'PATCH', 'POST', 'PUT', 'DELETE'].includes(httpMethod)) {
                throw new ESPectreDirectError('Direct HTTP method is invalid.', 'invalid_method');
            }
            if (typeof resource !== 'string' || !/^[A-Za-z0-9/-]{1,96}$/.test(resource)) {
                throw new ESPectreDirectError('Direct resource is invalid.', 'invalid_resource');
            }
            if (!data || typeof data !== 'object' || Array.isArray(data)) {
                throw new ESPectreDirectError('Direct data must be an object.', 'invalid_params');
            }
            if (!allowBeforeHandshake && httpMethod !== 'GET' && !this.#compatible) {
                throw new ESPectreDirectError('Complete the Direct capability handshake before changing the device.', 'handshake_required');
            }
            const payload = Object.keys(data).length ? JSON.stringify(data) : null;
            if (new TextEncoder().encode(payload || '').byteLength > MAX_REQUEST_BYTES) {
                throw new ESPectreDirectError('Direct request exceeds the 4096-byte limit.', 'frame_too_large');
            }
            const headers = { Accept: 'application/json' };
            if (payload !== null) headers['Content-Type'] = 'application/json';
            let response;
            let text;
            const attempts = httpMethod === 'GET' ? 2 : 1;
            for (let attempt = 0; attempt < attempts; attempt += 1) {
                const controller = new AbortController();
                this.#requestControllers.add(controller);
                const timer = setTimeout(() => controller.abort('request timeout'), timeoutMs);
                try {
                    response = await localFetch(endpointWithPath(this.#endpoint, `${REQUEST_PATH}/${resource.replace(/^\/+/, '')}`), {
                        method: httpMethod,
                        headers,
                        body: payload,
                        signal: controller.signal
                    });
                    text = await readBoundedResponseText(response, MAX_RESPONSE_BYTES);
                    break;
                } catch (error) {
                    if (error instanceof ESPectreDirectError) throw error;
                    const aborted = controller.signal.aborted;
                    const timedOut = aborted && controller.signal.reason === 'request timeout';
                    if (!aborted && !this.#closing && attempt + 1 < attempts) continue;
                    throw new ESPectreDirectError(
                        timedOut ? `Direct request ${httpMethod} ${resource} timed out.`
                            : aborted ? 'Direct session ended.' : `Direct HTTP request failed: ${error.message}`,
                        timedOut ? 'timeout' : aborted ? 'not_connected' : 'connection_failed'
                    );
                } finally {
                    clearTimeout(timer);
                    this.#requestControllers.delete(controller);
                }
            }
            if (!response.ok) {
                const detail = text.slice(0, 256).trim();
                throw new ESPectreDirectError(detail || `Direct HTTP returned ${response.status}.`, `http_${response.status}`);
            }
            const resultMessage = parseObject(text);
            if (httpMethod === 'GET') return resultMessage;
            if (typeof resultMessage.accepted !== 'boolean'
                || typeof resultMessage.code !== 'string'
                || typeof resultMessage.message !== 'string') {
                throw new ESPectreDirectError('Direct command result is invalid.', 'invalid_envelope');
            }
            if (!resultMessage.accepted) {
                throw new ESPectreDirectError(resultMessage.message, resultMessage.code);
            }
            return resultMessage;
        }

        async discoverPeersBootstrap(options = {}) {
            return validatePeerDiscoveryResult(await this.#sendRequest('get', 'devices', {}, {
                timeoutMs: PEER_DISCOVERY_TIMEOUT_MS,
                allowBeforeHandshake: true,
                ...options
            }));
        }

        close() {
            this.#closing = true;
            this.#connected = false;
            this.#compatible = false;
            this.#capabilities = null;
            this.#eventController?.abort('client closed');
            this.#eventController = null;
            for (const controller of this.#requestControllers) controller.abort('client closed');
            this.#requestControllers.clear();
        }
    }

    window.ESPectreDirectClient = ESPectreDirectClient;
    window.ESPectreDirectError = ESPectreDirectError;
    window.ESPectreRawCsiParser = ESPectreRawCsiParser;
})();
