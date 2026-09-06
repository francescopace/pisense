/*
 * ESPectre - Browser Direct HTTP client unit tests
 *
 * Copyright 2026 Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

import { afterEach, describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { DIRECT_PORT, peerDiscoveryScenarios } from './fixtures/peer_discovery_fixture.mjs';

globalThis.window = globalThis.window ?? {};
createRequire(import.meta.url)('../../docs/web/assets/js/espectre-direct.js');

const Client = window.ESPectreDirectClient;
const DirectError = window.ESPectreDirectError;
const RawParser = window.ESPectreRawCsiParser;

function rawFrame({
    sessionId = '00112233445566778899aabbccddeeff',
    sequence = 1n,
    fresh = 1n,
    dropped = 0n,
    backpressure = 0n,
    payload = new Uint8Array([1, 2, 3, 4])
} = {}) {
    const record = Buffer.alloc(64 + payload.length);
    record.writeUInt16LE(0x4353, 0);
    record.writeUInt8(8, 2);
    record.writeUInt8(64, 3);
    record.writeUInt8(4, 4);
    record.writeUInt8(1 << 3, 5);
    const saturatedSequence = sequence > 0xFFFFFFFFn ? 0xFFFFFFFF : Number(sequence);
    const saturatedFresh = fresh > 0xFFFFFFFFn ? 0xFFFFFFFF : Number(fresh);
    record.writeUInt32LE(saturatedSequence, 6);
    record.writeUInt16LE(payload.length / 2, 10);
    record.writeUInt16LE(payload.length, 12);
    record.writeBigUInt64LE(backpressure, 45);
    record.writeUInt32LE(saturatedFresh, 53);
    record.writeUInt32LE(saturatedSequence, 57);
    Buffer.from(payload).copy(record, 64);

    const prefix = Buffer.alloc(60);
    prefix.writeUInt32LE(0x52505345, 0);
    prefix.writeUInt8(1, 4);
    prefix.writeUInt8(8, 5);
    prefix.writeUInt16LE(60, 6);
    Buffer.from(sessionId, 'hex').copy(prefix, 8);
    prefix.writeBigUInt64LE(sequence, 24);
    prefix.writeUInt16LE(record.length, 32);
    prefix.writeUInt16LE(0, 34);
    prefix.writeBigUInt64LE(fresh, 36);
    prefix.writeBigUInt64LE(dropped, 44);
    prefix.writeBigUInt64LE(backpressure, 52);
    return new Uint8Array(Buffer.concat([prefix, record]));
}

function pendingBody(chunks = []) {
    let index = 0;
    return {
        getReader() {
            return {
                async read() {
                    if (index < chunks.length) return { value: new TextEncoder().encode(chunks[index++]), done: false };
                    return new Promise(() => {});
                },
                releaseLock() {}
            };
        }
    };
}

function installHttpFixture({ eventChunks = [], resultFor = () => ({}) } = {}) {
    const calls = [];
    globalThis.fetch = async (url, options) => {
        calls.push({ url, options });
        if (url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody(eventChunks) };
        const marker = '/espectre/v1/';
        const resource = url.slice(url.indexOf(marker) + marker.length);
        const data = options.body ? JSON.parse(options.body) : {};
        const result = resultFor(options.method.toLowerCase(), resource, data);
        const payload = options.method === 'GET'
            ? result
            : { accepted: true, code: 'ok', message: 'completed', ...result };
        return { ok: true, status: options.method === 'GET' ? 200 : 202, text: async () => JSON.stringify(payload) };
    };
    return calls;
}

async function connectedClient(fixture = {}) {
    const calls = installHttpFixture(fixture);
    const client = new Client('192.168.1.42');
    await client.connect();
    return { client, calls };
}

afterEach(() => {
    delete globalThis.fetch;
    delete window.ESPectreBrowserSupport;
});

describe('Direct HTTP endpoint policy', () => {
    it('normalizes private IPv4, .local, HTTPS, and local IPv6 endpoints', () => {
        assert.equal(Client.DEFAULT_PORT, DIRECT_PORT);
        assert.equal(Client.normalizeEndpoint('192.168.1.42'), `http://192.168.1.42:${DIRECT_PORT}/espectre/v1`);
        assert.equal(Client.normalizeEndpoint('espectre-a1.local'), `http://espectre-a1.local:${DIRECT_PORT}/espectre/v1`);
        assert.equal(Client.normalizeEndpoint('https://espectre-a1.local/espectre/v1'), `https://espectre-a1.local:${DIRECT_PORT}/espectre/v1`);
        assert.equal(Client.normalizeEndpoint('http://[fd12:3456:789a::42]:61443/espectre/v1'), 'http://[fd12:3456:789a::42]:61443/espectre/v1');
    });

    it('rejects WebSocket, public, credentialed, queried, and unrelated endpoints', () => {
        for (const endpoint of [
            'ws://192.168.1.2/espectre/v1/ws', 'wss://espectre-a.local/espectre/v1/ws',
            '8.8.8.8', 'http://user:pass@192.168.1.2', 'http://192.168.1.2/?token=x',
            'http://192.168.1.2/admin'
        ]) assert.throws(() => Client.normalizeEndpoint(endpoint), DirectError);
    });

    it('creates a distinct lowercase 96-bit bootstrap hostname from injected entropy', () => {
        let invocation = 0;
        const randomSource = { getRandomValues(bytes) { bytes.fill(invocation++); return bytes; } };
        assert.equal(Client.createDiscoveryEndpoint(randomSource), `http://espectre-devices-000000000000000000000000.local:${DIRECT_PORT}/espectre/v1`);
        assert.equal(Client.createDiscoveryEndpoint(randomSource), `http://espectre-devices-010101010101010101010101.local:${DIRECT_PORT}/espectre/v1`);
        assert.throws(() => Client.createDiscoveryEndpoint(null), (error) => error.code === 'unsupported_crypto');
    });
});

describe('Raw CSI HTTP parser', () => {
    it('accepts large aggregates with an incomplete frame at either boundary', () => {
        const frames = Array.from({ length: 220 }, (_, index) => rawFrame({
            sequence: BigInt(index + 1),
            fresh: BigInt(index + 1),
            payload: new Uint8Array(512)
        }));
        const bytes = Buffer.concat(frames);
        const expected = new RawParser();
        const expectedRecords = frames.flatMap((frame) => expected.append(frame));
        for (const splitAt of [17, frames[0].length - 1]) {
            const parser = new RawParser();
            assert.equal(parser.append(bytes.subarray(0, splitAt)).length, 0);
            const records = parser.append(bytes.subarray(splitAt, bytes.length - 19));
            assert.equal(records.length, frames.length - 1);
            assert.equal(parser.bufferedBytes, frames.at(-1).length - 19);
            records.push(...parser.append(bytes.subarray(bytes.length - 19)));
            assert.deepEqual(records, expectedRecords);
            assert.equal(parser.bufferedBytes, 0);
        }
    });

    it('reconstructs split and aggregated frames and exposes counters', () => {
        const parser = new RawParser('00112233445566778899aabbccddeeff');
        const first = rawFrame();
        const second = rawFrame({ sequence: 3n, fresh: 2n, dropped: 1n });
        assert.deepEqual(parser.append(first.subarray(0, 17)), []);
        const records = parser.append(new Uint8Array([
            ...first.subarray(17),
            ...second
        ]));
        assert.equal(records.length, 2);
        assert.deepEqual(records.map((record) => record.streamSequence), [1n, 3n]);
        assert.equal(parser.freshRecordTotal, 2n);
        assert.equal(parser.rawDropTotal, 1n);
        assert.equal(parser.sendBackpressureTotal, 0n);
        assert.equal(parser.bufferedBytes, 0);
    });

    it('fails closed on session, flags, counter, and V8 sequence mismatches', () => {
        const wrongSession = rawFrame({ sessionId: '10112233445566778899aabbccddeeff' });
        assert.throws(
            () => new RawParser('00112233445566778899aabbccddeeff').append(wrongSession),
            (error) => error.code === 'invalid_raw_frame'
        );

        const wrongFlags = rawFrame();
        new DataView(wrongFlags.buffer, wrongFlags.byteOffset).setUint16(34, 1, true);
        assert.throws(() => new RawParser('00112233445566778899aabbccddeeff').append(wrongFlags));

        const skippedFresh = rawFrame({ fresh: 2n });
        assert.throws(() => new RawParser('00112233445566778899aabbccddeeff').append(skippedFresh));

        const wrongRecordSequence = rawFrame();
        new DataView(wrongRecordSequence.buffer, wrongRecordSequence.byteOffset)
            .setUint32(60 + 6, 2, true);
        assert.throws(
            () => new RawParser('00112233445566778899aabbccddeeff').append(wrongRecordSequence),
            (error) => error.code === 'invalid_raw_record'
        );
    });
});

describe('Peer discovery schema v2', () => {
    it('uses one bootstrap GET without opening SSE or querying capabilities', async () => {
        const calls = installHttpFixture({ resultFor: (_method, resource) => {
            assert.equal(resource, 'devices');
            return peerDiscoveryScenarios.multiFrontend;
        } });
        const client = new Client(Client.createDiscoveryEndpoint({
            getRandomValues(bytes) { bytes.fill(0xab); return bytes; }
        }));
        const result = await client.discoverPeersBootstrap();
        assert.equal(result.devices.length, 3);
        assert.equal(client.connected, false);
        assert.equal(calls.length, 1);
        assert.equal(calls[0].options.method, 'GET');
        assert.equal(calls[0].options.body, null);
        client.close();
    });

    it('accepts HTTP peers and constructs request endpoints', () => {
        const result = Client.validatePeerDiscoveryResult(peerDiscoveryScenarios.multiFrontend);
        assert.equal(result.devices.length, 3);
        assert.equal(result.devices[0].endpoints[0], `http://192.168.1.42:${DIRECT_PORT}/espectre/v1`);
        const esphome = result.devices.find((device) => device.frontend === 'esphome');
        assert.equal(esphome.endpoints[0], `http://192.168.1.44:${DIRECT_PORT}/espectre/v1`);
    });

    it('accepts partial results, and rejects hostile or old-schema peers', () => {
        for (const name of ['partial', 'truncated', 'timeout', 'mixedAddresses']) {
            assert.doesNotThrow(() => Client.validatePeerDiscoveryResult(peerDiscoveryScenarios[name]));
        }
        for (const name of ['duplicateIdentity', 'malformed', 'oversized', 'nonLocal', 'malformedAddress', 'websocket']) {
            assert.throws(() => Client.validatePeerDiscoveryResult(peerDiscoveryScenarios[name]), DirectError);
        }
    });
});

describe('Direct HTTP request and SSE lifecycle', () => {
    it('does not issue a local request when browser permission is denied', async () => {
        let fetchCalls = 0;
        globalThis.fetch = async () => { fetchCalls += 1; };
        window.ESPectreBrowserSupport = {
            localNetworkAccessState: async () => 'denied'
        };
        const client = new Client('192.168.1.42');
        await assert.rejects(client.connect(), (error) => error.code === 'local_network_denied');
        assert.equal(fetchCalls, 0);
    });

    it('opens SSE and reads resource snapshots without caching', async () => {
        const capabilities = { protocol_version: '1.0', operations: [], resources: ['capabilities', 'device'], features: { csi: false } };
        const { client, calls } = await connectedClient({
            resultFor: (_method, resource) => resource === 'capabilities' ? capabilities : { firmware: '4.0.0' }
        });
        await client.handshake();
        assert.deepEqual(await client.request('get', 'device'), { firmware: '4.0.0' });
        assert.equal(calls[0].url, `http://192.168.1.42:${DIRECT_PORT}/espectre/v1/events`);
        assert.equal(calls[0].options.targetAddressSpace, 'local');
        assert.equal(calls[0].options.cache, 'no-store');
        assert.equal(calls[1].options.method, 'GET');
        assert.equal(calls[1].options.body, null);
        assert.equal(calls[2].url, `http://192.168.1.42:${DIRECT_PORT}/espectre/v1/device`);
        client.close();
    });

    it('parses SSE events split across fetch chunks', async () => {
        const envelope = JSON.stringify({ timestamp_ms: 1000, state: 'motion', score: 0.42 });
        const midpoint = Math.floor(envelope.length / 2);
        installHttpFixture({
            eventChunks: [`event: motion\ndata: ${envelope.slice(0, midpoint)}`, `${envelope.slice(midpoint)}\n\n`]
        });
        const client = new Client('192.168.1.42');
        const eventPromise = new Promise((resolve) => client.on('event', (name, data) => resolve({ name, data })));
        await client.connect();
        const event = await eventPromise;
        assert.deepEqual(event, { name: 'motion', data: { timestamp_ms: 1000, state: 'motion', score: 0.42 } });
        client.close();
    });

    it('parses an SSE delimiter split between CR and LF chunks', async () => {
        const envelope = JSON.stringify({ status: 'ok', online: true, uptime_s: 1, timestamp_ms: 1000 });
        installHttpFixture({ eventChunks: [`event: health\r\ndata: ${envelope}\r`, '\n\r\n'] });
        const client = new Client('192.168.1.42');
        const eventPromise = new Promise((resolve) => client.on('event', (name, data) => resolve({ name, data })));
        await client.connect();
        assert.deepEqual(await eventPromise, { name: 'health', data: { status: 'ok', online: true, uptime_s: 1, timestamp_ms: 1000 } });
        client.close();
    });

    it('rejects an unterminated SSE event before its buffer grows without bound', async () => {
        installHttpFixture({ eventChunks: [`event: motion\ndata: ${'x'.repeat(8192)}`] });
        const client = new Client('192.168.1.42');
        const errorPromise = new Promise((resolve) => client.on('protocol-error', resolve));
        await client.connect();
        const error = await errorPromise;
        assert.equal(error.code, 'frame_too_large');
        client.close();
    });

    it('keeps the request timeout active while reading the response body', async () => {
        globalThis.fetch = async (_url, options) => {
            if (_url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody() };
            return {
                ok: true,
                status: 200,
                body: {
                    getReader() {
                        return {
                            read() {
                                return new Promise((_resolve, reject) => {
                                    options.signal.addEventListener('abort', () => reject(new Error('aborted')), { once: true });
                                });
                            },
                            releaseLock() {}
                        };
                    }
                }
            };
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        await assert.rejects(
            client.request('get', 'capabilities', {}, { allowBeforeHandshake: true, timeoutMs: 10 }),
            (error) => error.code === 'timeout'
        );
        client.close();
    });

    it('aborts pending HTTP reads when SSE ends and permits a clean reconnect', async () => {
        let endStream;
        let requestSignal;
        let reads = 0;
        globalThis.fetch = async (url, options) => {
            if (url.endsWith('/events')) {
                return { ok: true, status: 200, body: {
                    getReader: () => ({
                        read: () => new Promise((resolve) => { endStream = () => resolve({ done: true }); }),
                        releaseLock() {},
                    }),
                } };
            }
            reads += 1;
            if (reads > 1) return { ok: true, status: 200, text: async () => '{"healthy":true}' };
            requestSignal = options.signal;
            return new Promise((_resolve, reject) => {
                options.signal.addEventListener('abort', () => reject(new Error('aborted')), { once: true });
            });
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        const pending = assert.rejects(client.request('get', 'health'), (error) => error.code === 'not_connected');
        endStream();
        await pending;
        assert.equal(requestSignal.aborted, true);
        assert.equal(reads, 1);
        await client.connect();
        assert.deepEqual(await client.request('get', 'health'), { healthy: true });
        assert.equal(reads, 2);
        client.close();
    });

    it('retries a read-only request once after a transport failure', async () => {
        let getCalls = 0;
        const requestUrls = [];
        globalThis.fetch = async (url, options) => {
            if (url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody() };
            getCalls += 1;
            requestUrls.push(url);
            if (getCalls === 1) throw new TypeError('stale persistent connection');
            return { ok: true, status: 200, text: async () => JSON.stringify({ healthy: true }) };
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        assert.deepEqual(await client.request('get', 'diagnostics'), { healthy: true });
        assert.equal(getCalls, 2);
        assert.equal(requestUrls[0], requestUrls[1]);
        client.close();
    });

    it('does not retry a mutating request after a transport failure', async () => {
        let mutationCalls = 0;
        globalThis.fetch = async (_url, options) => {
            if (_url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody() };
            if (options.method === 'GET') {
                return {
                    ok: true,
                    status: 200,
                    text: async () => JSON.stringify({
                        protocol_version: '1.0', operations: [{ name: 'update_sensing' }], resources: ['capabilities']
                    })
                };
            }
            mutationCalls += 1;
            throw new TypeError('connection lost after send');
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        await client.handshake();
        await assert.rejects(client.request('patch', 'sensing', { enabled: true }), (error) => error.code === 'connection_failed');
        assert.equal(mutationCalls, 1);
        client.close();
    });

    it('rejects an oversized response while streaming it', async () => {
        globalThis.fetch = async (_url, options) => {
            if (_url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody() };
            let read = false;
            return {
                ok: true,
                status: 200,
                body: {
                    getReader() {
                        return {
                            async read() {
                                if (read) return { done: true };
                                read = true;
                                return { value: new Uint8Array(8193), done: false };
                            },
                            async cancel() {},
                            releaseLock() {}
                        };
                    }
                }
            };
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        await assert.rejects(
            client.request('get', 'capabilities', {}, { allowBeforeHandshake: true }),
            (error) => error.code === 'frame_too_large'
        );
        client.close();
    });

    it('blocks mutations before handshake and sends resource mutations after it', async () => {
        const { client, calls } = await connectedClient({
            resultFor: (method, resource, data) => {
                if (resource === 'capabilities') {
                    return { protocol_version: '1.0', operations: [{ name: 'update_sensing' }], resources: ['capabilities', 'sensing'], features: { csi: true } };
                }
                assert.deepEqual({ method, resource, data }, { method: 'patch', resource: 'sensing', data: { enabled: true } });
                return { data: { enabled: true } };
            }
        });
        await assert.rejects(client.request('patch', 'sensing', { enabled: true }), (error) => error.code === 'handshake_required');
        await client.handshake();
        const result = await client.request('patch', 'sensing', { enabled: true });
        assert.equal(result.accepted, true);
        assert.equal(calls.at(-1).options.headers.Authorization, undefined);
        client.close();
    });

    it('reports HTTP rate limits explicitly', async () => {
        globalThis.fetch = async (_url, options) => {
            if (_url.endsWith('/events')) return { ok: true, status: 200, body: pendingBody() };
            return { ok: false, status: 429, text: async () => 'rate limited' };
        };
        const client = new Client('192.168.1.42');
        await client.connect();
        await assert.rejects(client.request('get', 'capabilities', {}, { allowBeforeHandshake: true }), (error) => error.code === 'http_429');
        client.close();
    });

    it('rejects invalid requests before sending HTTP traffic', async () => {
        const { client, calls } = await connectedClient();
        for (const [method, resource, data, code] of [
            ['trace', 'health', {}, 'invalid_method'],
            ['get', '../health', {}, 'invalid_resource'],
            ['get', 'health', null, 'invalid_params'],
            ['get', 'health', [], 'invalid_params'],
            ['get', 'health', { value: 'é'.repeat(Client.MAX_REQUEST_FRAME_BYTES / 2) }, 'frame_too_large']
        ]) {
            await assert.rejects(client.request(method, resource, data), (error) => error.code === code);
        }
        assert.equal(calls.length, 1);
        client.close();
        await assert.rejects(client.request('get', 'health'), (error) => error.code === 'not_connected');
    });

    it('keeps mutations blocked after an incompatible capability handshake', async () => {
        for (const capabilities of [
            { protocol_version: 'unsupported', operations: [] },
            { protocol_version: Client.PROTOCOL_VERSION },
            { protocol_version: Client.PROTOCOL_VERSION, operations: [null] },
            { protocol_version: Client.PROTOCOL_VERSION, operations: [{ name: '../sensing' }] }
        ]) {
            const { client, calls } = await connectedClient({ resultFor: () => capabilities });
            await assert.rejects(client.handshake(), (error) => error.code === 'invalid_capabilities');
            assert.equal(client.compatible, false);
            assert.equal(client.capabilities, null);
            await assert.rejects(client.request('patch', 'sensing', { enabled: true }),
                (error) => error.code === 'handshake_required');
            assert.equal(calls.length, 2);
            client.close();
        }
    });

    it('distinguishes malformed responses from device command rejections', async () => {
        const { client } = await connectedClient({ resultFor: () => ({
            protocol_version: Client.PROTOCOL_VERSION, operations: [{ name: 'update_sensing' }]
        }) });
        const capabilities = await client.handshake();
        assert.equal(client.compatible, true);
        assert.deepEqual(client.capabilities, capabilities);
        for (const [payload, code] of [
            ['{', 'invalid_json'],
            ['null', 'invalid_envelope'],
            ['[]', 'invalid_envelope'],
            ['{}', 'invalid_envelope'],
            [JSON.stringify({ accepted: false, code: 'busy', message: 'busy' }), 'busy']
        ]) {
            let requests = 0;
            globalThis.fetch = async () => {
                requests += 1;
                return { ok: true, status: 202, text: async () => payload };
            };
            await assert.rejects(client.request('patch', 'sensing', { enabled: true }),
                (error) => error.code === code);
            assert.equal(requests, 1);
        }
        client.close();
        assert.equal(client.compatible, false);
        assert.equal(client.capabilities, null);
    });

    it('decodes response text with UTF-8 characters split across stream chunks', async () => {
        const { client } = await connectedClient();
        const payload = new TextEncoder().encode(JSON.stringify({ name: 'café' }));
        const split = payload.indexOf(0xc3) + 1;
        let released = false;
        globalThis.fetch = async () => {
            const chunks = [payload.subarray(0, split), payload.subarray(split)];
            return { ok: true, status: 200, body: { getReader: () => ({
                read: async () => chunks.length ? { value: chunks.shift(), done: false } : { done: true },
                releaseLock() { released = true; }
            }) } };
        };
        assert.deepEqual(await client.request('get', 'info'), { name: 'café' });
        assert.equal(released, true);
        client.close();
    });

    it('enforces the response byte limit when streaming is unavailable', async () => {
        const { client } = await connectedClient();
        globalThis.fetch = async () => ({ ok: true, status: 200,
            text: async () => JSON.stringify({ value: 'é'.repeat(Client.MAX_RESPONSE_FRAME_BYTES / 2) }) });
        await assert.rejects(client.request('get', 'info'), (error) => error.code === 'frame_too_large');
        client.close();
    });

    it('supports removing event listeners across reconnects', async () => {
        installHttpFixture();
        const client = new Client('192.168.1.42');
        let opens = 0;
        const unsubscribe = client.on('open', () => { opens += 1; });
        await client.connect();
        assert.equal(opens, 1);
        client.close();
        unsubscribe();
        await client.connect();
        assert.equal(opens, 1);
        assert.equal(client.connected, true);
        client.close();
    });
});
