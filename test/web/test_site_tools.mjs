/*
 * ESPectre - Website tool contracts
 *
 * Copyright 2026 Francesco Pace <francescopace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

import { before, describe, it } from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { AnsiUp } from '../../docs/web/node_modules/ansi_up/ansi_up.js';
import { rollup } from '../../docs/web/node_modules/rollup/dist/es/rollup.js';
import serialConfig from '../../docs/web/rollup.config.mjs';
import { SERIAL_PACKET_HEADER, ImprovSerialMessageType, ImprovSerialCurrentState } from '../../docs/web/node_modules/improv-wifi-serial-sdk/dist/const.js';
import { flashSource, index, read, routeManifest, toolContent } from './fixtures/site_test_helpers.mjs';

function loadFlashRuntime(globals = {}) {
    const window = {};
    const context = {
        window, URL, Set, Map, Object, String, Number, Math, RegExp,
        Promise, TextEncoder, TextDecoder, Uint8Array, Blob,
        setTimeout, clearTimeout, setInterval, clearInterval, console,
        ...globals,
    };
    vm.createContext(context);
    vm.runInContext(flashSource, context);
    context.flashState = vm.runInContext('flash', context);
    return context;
}

function loadFlashCore(globals = {}) {
    return loadFlashRuntime(globals).window.ESPectreFlashCore;
}

function flashReadFixture(contents) {
    const reads = [];
    const writes = [];
    const packets = [];
    const transport = {
        getInfo: () => 'test',
        slipWriter: (packet) => packet,
        async read() {
            assert.ok(packets.length, 'A read must have a pending protocol frame');
            return packets.shift();
        },
        device: {
            writable: {
                getWriter: () => ({
                    async write(packet) {
                        writes.push(packet);
                        if (packet.length !== 24) return;
                        const view = new DataView(packet.buffer, packet.byteOffset);
                        const address = view.getUint32(8, true);
                        const size = view.getUint32(12, true);
                        const blockSize = view.getUint32(16, true);
                        assert.equal(view.getUint32(20, true), blockSize);
                        reads.push([address, size]);
                        // Command response, data frame, and stub's trailing digest.
                        const response = new Uint8Array(10);
                        response[0] = 1;
                        response[1] = packet[1];
                        const bytes = contents(address, size);
                        packets.push(response);
                        for (let offset = 0; offset < bytes.length; offset += blockSize) {
                            packets.push(bytes.slice(offset, offset + blockSize));
                        }
                        packets.push(new Uint8Array(16));
                    },
                    releaseLock() {},
                    async abort() {},
                }),
            },
        },
    };
    const loader = new ESPLoader({
        transport, baudrate: 115200,
        terminal: { clean() {}, writeLine() {}, write() {} },
    });
    return { loader, transport, reads, writes, packets };
}

function loadDeviceHttpRuntime() {
    let now = 0;
    let timerId = 0;
    const timers = new Map();
    const panel = { open: true };
    const context = vm.createContext({
        window: {}, HTMLElement: class {}, customElements: { get: () => true },
        URL, TextEncoder, console,
        Date: class extends Date { static now() { return now; } },
        document: { hidden: false, getElementById: () => null },
        $: (selector) => selector === '.device-live-diagnostics' ? panel : { hidden: false },
        setTimeout: (fn, ms) => { timers.set(++timerId, { fn, at: now + ms }); return timerId; },
        clearTimeout: (id) => timers.delete(id),
        setInterval: (fn, ms) => { timers.set(++timerId, { fn, at: now + ms, interval: ms }); return timerId; },
        clearInterval: (id) => timers.delete(id),
        toast() {}, track() {},
    });
    for (const file of ['device-session', 'monitor-tool', 'configure-tool', 'direct-discovery']) {
        vm.runInContext(read(`docs/web/assets/js/${file}.js`), context);
    }
    context.applySysinfo = (snapshot) => context.evaluateConfigVerification(snapshot);
    context.applySensingSnapshot = () => {};
    context.applyOtaStatus = () => {};
    context.renderWifiAccessPoints = () => {};
    context.monitorStats = () => {};
    context.activeToolName = () => vm.runInContext("route === 'tool-configure' ? 'configure' : 'monitor'", context);
    const run = (code) => vm.runInContext(code, context);
    return {
        context, run, timers, panel,
        setClient(client) {
            context.testClient = client;
            run("directClient = testClient; conn.mode = 'direct'; conn.status = 'connected'; route = 'tool-configure'");
        },
        async nextTimer() {
            for (let i = 0; i < 20; i++) await Promise.resolve();
            const [id, timer] = [...timers].sort((a, b) => a[1].at - b[1].at)[0];
            timers.delete(id);
            now = timer.at;
            if (timer.interval) timers.set(id, { ...timer, at: now + timer.interval });
            timer.fn();
            for (let i = 0; i < 20; i++) await Promise.resolve();
        },
    };
}

describe('device HTTP request budgets', () => {
    for (const [resource, field, value] of [
        ['device', 'device_label', 'office'],
        ['mqtt', 'mqtt_host', 'broker.local'],
        ['wifi/bssid', 'wifi_bssid', 'AA:BB:CC:DD:EE:FF'],
    ]) {
        it(`verifies ${resource} with a single resource read`, async () => {
            const runtime = loadDeviceHttpRuntime();
            const calls = [];
            runtime.setClient({ connected: true, request: async (method, path) => {
                calls.push([method, path]);
                return { label: value, host: value, bssid: value };
            } });
            await runtime.context.cfgApply('save', '', 'patch', resource, {}, (snapshot) => snapshot[field] === value);
            await runtime.nextTimer();
            assert.deepEqual(calls, [['patch', resource], ['get', resource.split('/')[0]]]);
            assert.equal(runtime.run('pendingConfigVerification'), null);
            assert.equal(runtime.timers.size, 0);
        });
    }

    it('keeps one-second diagnostics only while the panel and browser are visible', async () => {
        const runtime = loadDeviceHttpRuntime();
        let reads = 0;
        runtime.setClient({ connected: true, request: async () => { reads++; return {}; } });
        runtime.run("route = 'tool-monitor'");
        runtime.context.syncDiagnosticsPolling();
        assert.equal(reads, 1);
        await runtime.nextTimer();
        assert.equal(reads, 2);
        assert.equal([...runtime.timers.values()][0].interval, 1000);
        runtime.context.document.hidden = true;
        runtime.context.syncDiagnosticsPolling();
        assert.equal(runtime.timers.size, 0);
        await runtime.context.monitorRequestStats();
        assert.equal(reads, 2);
        runtime.context.document.hidden = false;
        runtime.context.syncDiagnosticsPolling();
        assert.equal(reads, 3);
        runtime.panel.open = false;
        runtime.context.syncDiagnosticsPolling();
        assert.equal(runtime.timers.size, 0);
        await runtime.context.monitorRequestStats();
        assert.equal(reads, 3);
    });

    it('limits verification retries to the changed resource and ignores unrelated events', async () => {
        const runtime = loadDeviceHttpRuntime();
        const reads = [];
        runtime.setClient({ connected: true, request: async (method, resource) => {
            if (method === 'get') reads.push(resource);
            return { label: 'old' };
        } });
        await runtime.context.cfgSaveDeviceLabel('new');
        const timer = [...runtime.timers.keys()][0];
        runtime.context.evaluateConfigVerification({ wifi_connected: true });
        assert.equal([...runtime.timers.keys()][0], timer);
        const attempts = runtime.run('CONFIG_VERIFICATION_MAX_ATTEMPTS');
        for (let i = 0; i < attempts; i++) await runtime.nextTimer();
        assert.deepEqual(reads, Array(attempts).fill('device'));
        assert.equal(runtime.run('pendingConfigVerification'), null);
        assert.equal(runtime.timers.size, 0);
    });

    it('backs off scan polling and cancels it without restarting the device scan', async () => {
        const runtime = loadDeviceHttpRuntime();
        const calls = [];
        runtime.setClient({ connected: true, request: async (method, resource) => {
            calls.push([method, resource]);
            return { scanning: true };
        } });
        runtime.run("monitor.commands.add('scan_wifi')");
        const scan = runtime.context.cfgRefreshWifiAccessPoints();
        await runtime.context.cfgRefreshWifiAccessPoints();
        const intervals = [];
        for (let i = 0; i < 3; i++) {
            intervals.push([...runtime.timers.values()][0].at - runtime.context.Date.now());
            await runtime.nextTimer();
        }
        assert.deepEqual(intervals, [1000, 2000, 3000]);
        assert.deepEqual(calls, [['post', 'wifi/scans'], ...Array.from({ length: 3 }, () => ['get', 'wifi/access-points'])]);
        runtime.context.document.hidden = true;
        runtime.context.cancelWifiScan();
        await runtime.nextTimer();
        await scan;
        assert.equal(calls.length, 4);
        assert.equal(runtime.run('wifiScanActive'), false);
        assert.equal(runtime.timers.size, 0);
    });

    it('loads settings on demand and shares concurrent refresh reads', async () => {
        const runtime = loadDeviceHttpRuntime();
        const reads = [];
        runtime.setClient({ connected: true, capabilities: {
            resources: ['device', 'health', 'sensing', 'wifi', 'mqtt', 'ota', 'diagnostics'],
            operations: [{ name: 'read_diagnostics' }],
        }, request: async (_method, resource) => { reads.push(resource); return {}; } });
        runtime.run("route = 'tool-monitor'");
        await Promise.all([runtime.context.refreshDirectDevice(), runtime.context.refreshDirectDevice()]);
        assert.deepEqual(reads, ['device', 'health', 'sensing', 'ota']);
        runtime.run("route = 'tool-configure'");
        await runtime.context.refreshDirectDevice();
        assert.deepEqual(reads.slice(4), ['wifi', 'mqtt', 'diagnostics']);
        await runtime.context.refreshDirectDevice();
        assert.equal(reads.length, 7);
    });

    it('prevents an obsolete refresh from continuing after reconnection', async () => {
        const runtime = loadDeviceHttpRuntime();
        const reads = [];
        let resolveOld;
        runtime.setClient({ connected: true, capabilities: {}, request: async (_method, resource) => {
            reads.push(resource);
            if (reads.length === 1) return new Promise((resolve) => { resolveOld = resolve; });
            return {};
        } });
        runtime.run("route = 'tool-monitor'");
        const old = runtime.context.refreshDirectDevice();
        runtime.run('directResourceSessions.delete(directClient)');
        await runtime.context.refreshDirectDevice();
        resolveOld({});
        assert.equal(await old, false);
        assert.deepEqual(reads, ['device', 'device', 'health', 'sensing']);
    });

    it('retains newer SSE snapshots while an initial GET is pending', async () => {
        const runtime = loadDeviceHttpRuntime();
        let resolveDevice;
        runtime.setClient({ connected: true, capabilities: {}, request: async (_method, resource) => {
            if (resource === 'device') return new Promise((resolve) => { resolveDevice = resolve; });
            return {};
        } });
        runtime.run("route = 'tool-monitor'");
        let label;
        runtime.context.applySysinfo = (snapshot) => { if (snapshot.label) label = snapshot.label; };
        const refresh = runtime.context.refreshDirectDevice();
        runtime.context.rememberDirectResource(runtime.context.testClient, 'device', { label: 'new' });
        resolveDevice({ label: 'old' });
        assert.equal(await refresh, true);
        assert.equal(label, 'new');
    });

    it('stops loading settings resources when the user leaves settings', async () => {
        const runtime = loadDeviceHttpRuntime();
        const reads = [];
        let resolveWifi;
        runtime.setClient({ connected: true, capabilities: {
            resources: ['mqtt'], operations: [{ name: 'read_diagnostics' }],
        }, request: async (_method, resource) => {
            reads.push(resource);
            if (resource === 'wifi') return new Promise((resolve) => { resolveWifi = resolve; });
            return {};
        } });
        const refresh = runtime.context.refreshDirectDevice();
        for (let i = 0; i < 20; i++) await Promise.resolve();
        runtime.run("route = 'tool-monitor'");
        resolveWifi({});
        await refresh;
        assert.deepEqual(reads, ['device', 'health', 'sensing', 'wifi']);
        assert.equal(runtime.run("directResourceSession(directClient).snapshots.has('wifi')"), false);
    });
});

let ImprovSerial;
let ESPLoader;
let Transport;
before(async () => {
    const bundle = await rollup({
        ...serialConfig,
        input: new URL('../../docs/web/headless-entry.js', import.meta.url).pathname,
    });
    try {
        const { output } = await bundle.generate(serialConfig.output);
        ({ ImprovSerial, ESPLoader, Transport } = await import(`data:text/javascript;base64,${Buffer.from(output[0].code).toString('base64')}`));
    } finally {
        await bundle.close();
    }
});

describe('Improv Serial initialization', () => {
    function device(onCommand) {
        let input;
        const commands = [];
        const port = {
            readable: new ReadableStream({ start(controller) { input = controller; } }),
            writable: new WritableStream({
                write(packet) {
                    commands.push(packet[9]);
                    onCommand(packet[9], reply, commands);
                },
            }),
        };
        function reply(type, data) {
            const bytes = [...SERIAL_PACKET_HEADER, type, data.length, ...data];
            input.enqueue(Uint8Array.from([...bytes, bytes.reduce((sum, value) => sum + value, 0) & 0xff, 10]));
        }
        const serial = new ImprovSerial(port, { log() {}, debug() {}, error() {} });
        return { serial, port, commands };
    }

    it('settles a silent probe and releases the pending RPC and serial reader', async () => {
        const { serial, port, commands } = device(() => {});
        await assert.rejects(serial.initialize(20));
        assert.deepEqual(commands, [2]);
        assert.equal(serial._rpcFeedback, undefined);
        assert.equal(port.readable.locked, false);
        assert.equal(port.writable.locked, false);
    });

    it('propagates device errors and closes the probe', async () => {
        const { serial, port } = device((command, reply) => {
            reply(ImprovSerialMessageType.ERROR_STATE, [2]);
        });
        await assert.rejects(serial.initialize(1000), /UNKNOWN_RPC_COMMAND/);
        assert.equal(serial._rpcFeedback, undefined);
        assert.equal(port.readable.locked, false);
    });

    for (const state of [ImprovSerialCurrentState.READY, ImprovSerialCurrentState.PROVISIONED]) {
        it(`retries a booting device and initializes state ${state}`, async () => {
            const url = 'http://espectre.local';
            const { serial, port, commands } = device((command, reply, sent) => {
                const result = (values) => {
                    const data = values.flatMap((value) => [value.length, ...new TextEncoder().encode(value)]);
                    reply(ImprovSerialMessageType.RPC_RESULT, [command, data.length, ...data]);
                };
                if (sent.length === 1) return;
                if (command === 2) {
                    reply(ImprovSerialMessageType.CURRENT_STATE, [state]);
                    if (state === ImprovSerialCurrentState.PROVISIONED) result([url]);
                } else if (command === 3) {
                    result(['ESPectre', '1.0.0', 'ESP32', 'test-device']);
                }
            });
            try {
                const info = await serial.initialize(2000);
                assert.equal(info.firmware, 'ESPectre');
                assert.equal(serial.state, state);
                assert.equal(serial.nextUrl, state === ImprovSerialCurrentState.PROVISIONED ? url : undefined);
                assert.deepEqual(commands, [2, 2, 3]);
            } finally {
                await serial.close();
            }
            assert.equal(port.readable.locked, false);
        });
    }
});

describe('website tool contracts', () => {
    it('simulates motion from captured touch and pen drags while preserving mouse input', () => {
        let now = 0;
        const context = vm.createContext({
            HTMLElement: class {},
            customElements: { get: () => true },
            performance: { now: () => now },
        });
        vm.runInContext(read('docs/web/assets/js/device-session.js'), context);
        vm.runInContext("conn.mode = 'demo'", context);
        let captured = null;
        const surface = { setPointerCapture: (id) => { captured = id; } };
        const event = (pointerType, clientX, onSurface = true) => ({
            pointerType, clientX, clientY: 0, pointerId: 7, isPrimary: true,
            target: { closest: () => onSurface ? surface : null },
        });
        const energy = () => vm.runInContext('demoInputEnergy', context);

        for (const type of ['touch', 'pen']) {
            vm.runInContext('demoInputEnergy = 0', context);
            context.demoStartPointer(event(type, 0, false));
            now += 100;
            context.demoTrackPointer(event(type, 180, false));
            assert.equal(energy(), 0);

            context.demoStartPointer(event(type, 0));
            assert.equal(captured, 7);
            now += 100;
            context.demoTrackPointer({ ...event(type, 180), pointerId: 8, isPrimary: false });
            assert.equal(energy(), 0);
            context.demoTrackPointer(event(type, 180));
            assert.equal(energy(), 1);
            context.demoEndPointer(event(type, 180));
            vm.runInContext('demoInputEnergy = 0', context);
            now += 100;
            context.demoTrackPointer(event(type, 360));
            assert.equal(energy(), 0);
        }

        context.demoTrackPointer(event('mouse', 0, false));
        now += 100;
        context.demoTrackPointer(event('mouse', 180, false));
        assert.equal(energy(), 1);
        vm.runInContext("conn.mode = 'direct'; demoInputEnergy = 0", context);
        now += 100;
        context.demoTrackPointer(event('mouse', 360, false));
        assert.equal(energy(), 0);
    });

    for (const outcome of ['resolve', 'reject', 'cancel']) {
        it(`ignores stale name discovery after ${outcome === 'cancel' ? 'navigation' : `Demo starts (${outcome})`}`, async () => {
            let resolveDiscovery;
            let rejectDiscovery;
            let discoveryClosed = false;
            let connections = 0;
            const timers = [];
            const errors = [];
            const discovery = new Promise((resolve, reject) => {
                resolveDiscovery = resolve;
                rejectDiscovery = reject;
            });
            const input = {
                value: 'office', closest: () => null,
                setAttribute: (...args) => errors.push(args)
            };
            const context = {
                console, URL, HTMLElement: class {},
                customElements: { get: () => true },
                document: { querySelector: () => input },
                $$: () => [], monitor: {},
                setTimeout: (callback) => { timers.push(callback); return timers.length; },
                setInterval: () => 1,
                DirectProtocolClient: {
                    normalizeEndpoint(value) {
                        if (value === 'office') throw new Error('Name requires discovery');
                        return value;
                    },
                    createDiscoveryEndpoint: () => 'bootstrap'
                }
            };
            vm.createContext(context);
            for (const file of ['device-session.js', 'direct-discovery.js']) {
                vm.runInContext(read(`docs/web/assets/js/${file}`), context);
            }
            const state = vm.runInContext('conn', context);
            Object.assign(context, {
                track() {}, activeToolName: () => 'monitor',
                rememberConnectionOrigin() {}, connectionParams: () => ({}),
                setStatus: (status) => { state.status = status; },
                syncFirmwareUpdateNotice() {}, markToolReady() {},
                applySysinfo() {}, monitorResetChart() {},
                setDirectConnectionHelp: (...args) => { if (args[0]) errors.push(args); },
                toast: (...args) => errors.push(args),
                makeDirectClient(endpoint) {
                    if (endpoint !== 'bootstrap') connections += 1;
                    return {
                        discoverPeersBootstrap: () => discovery,
                        close() { discoveryClosed = true; }
                    };
                }
            });
            const pending = context.connectDirect();
            if (outcome === 'cancel') context.cancelDirectDiscovery({ clear: true });
            else {
                context.connectDemo();
                timers.shift()();
                assert.equal(state.mode, 'demo');
            }
            assert.equal(discoveryClosed, true);
            if (outcome === 'reject') rejectDiscovery(new Error('Discovery aborted'));
            else resolveDiscovery({ devices: [{
                name: 'office', device_id: '0011223344556677',
                endpoints: ['http://192.168.1.10:62587']
            }], truncated: false });
            await pending;
            assert.equal(connections, 0);
            assert.equal(state.mode, outcome === 'cancel' ? null : 'demo');
            assert.equal(state.status, outcome === 'cancel' ? 'disconnected' : 'connected');
            assert.deepEqual(errors, []);
        });
    }

    it('publishes the firmware and SDK artifact channels', () => {
        const sdk = read('docs/web/content/sdk.html');
        for (const { sdkChannel: channel, path } of routeManifest.sdkChannels) {
            assert.match(toolContent.flash, new RegExp(`<option value="${channel}"`));
            assert.match(
                sdk,
                new RegExp(`href="${path}"[\\s\\S]*?data-sdk-version="${channel}"`)
            );
        }
        assert.match(
            read('docs/web/content/sdk/api.html'),
            /data-api-index="\/artifacts\/sdk\/api\/api-index\.json"/
        );
    });

    it('exposes an accessible Web Serial workflow', () => {
        const flash = toolContent.flash;
        assert.match(flash, /class="btn-primary js-flash-connect"[^>]+aria-describedby="flash-requirement"/);
        const stages = [...flash.matchAll(/data-flash-step="([^"]+)"/g)]
            .map((match) => match[1]);
        assert.deepEqual(stages.sort(), ['error', 'onboarding', 'review', 'select']);
        assert.match(flash, /<progress[^>]+class="flash-progress js-flash-progress"[^>]+max="100"/);
        assert.match(flash, /class="panel flash-progress-card js-flash-progress-card"[^>]+hidden/);
        assert.match(flash, /class="empty-state connection-card flash-connect-card flash-stage js-flash-stage"/);
        assert.match(flash, /class="panel flash-current-panel"/);
        assert.match(flash, /class="panel flash-change-panel"/);
        assert.match(flash, /class="flash-frontend-switch js-flash-frontend-switch" role="group"/);
        assert.match(flash, /<label for="flash-frontend">[^<]+<\/label>\s*<select id="flash-frontend"/);
        assert.match(flash, /class="js-flash-current-install-slot"/);
        assert.match(flash, /class="js-flash-change-install-slot"/);
        assert.match(flash, /class="flash-option flash-erase-option js-flash-force-erase-wrap"/);
        assert.match(flash, /class="modal-backdrop js-flash-erase-modal" hidden/);
        assert.match(flash, /class="modal-backdrop js-flash-flow-modal" hidden/);
        for (const step of ['onboarding', 'error']) {
            assert.match(
                flash,
                new RegExp(
                    `class="modal-card [^"]*js-flash-stage" data-flash-step="${step}" `
                    + 'role="dialog" aria-modal="true"'
                )
            );
        }
        assert.match(flash, /class="js-flash-wifi-form"/);
        assert.match(flash, /class="btn-secondary btn-sm js-flash-configure-wifi"/);
        assert.match(flash, /class="btn-secondary btn-sm js-flash-show-matter" hidden/);
        assert.match(flash, /class="matter-loading js-matter-loading" role="status"/);
        assert.match(flash, /class="flash-console-output js-flash-console-output"[^>]+tabindex="0"/);
        assert.match(flash, /<details class="panel flash-console js-flash-console" data-flash-session-panel hidden>/);
        assert.match(flash, /js-flash-console-reset"[^>]+disabled/);
    });

    it('keeps nested dialogs modal and restores managed inert state', () => {
        class FakeHTMLElement {
            constructor({ hidden = false } = {}) {
                this.children = [];
                this.dataset = {};
                this.hidden = hidden;
                this.inert = false;
                this.parentElement = null;
            }

            append(...children) {
                for (const child of children) {
                    child.parentElement = this;
                    this.children.push(child);
                }
            }
        }

        const body = new FakeHTMLElement();
        const classes = new Set();
        body.classList = {
            contains: (name) => classes.has(name),
            toggle(name, enabled) {
                if (enabled) classes.add(name);
                else classes.delete(name);
            },
        };
        const navigation = new FakeHTMLElement();
        const main = new FakeHTMLElement();
        const page = new FakeHTMLElement();
        const pageContent = new FakeHTMLElement();
        const modal = new FakeHTMLElement();
        const permanentlyInert = new FakeHTMLElement();
        permanentlyInert.inert = true;
        body.append(navigation, main, permanentlyInert);
        main.append(page);
        page.append(pageContent, modal);
        const nodes = [body, navigation, main, page, pageContent, modal, permanentlyInert];
        const document = {
            body,
            querySelectorAll(selector) {
                if (selector === '.modal-backdrop') return [modal];
                if (selector === '[data-modal-inert="true"]') {
                    return nodes.filter((node) => node.dataset.modalInert === 'true');
                }
                return [];
            },
        };
        const core = loadFlashCore({ document, HTMLElement: FakeHTMLElement });

        core.syncModalOpenState();
        assert.equal(body.classList.contains('modal-open'), true);
        assert.equal(navigation.inert, true);
        assert.equal(main.inert, false);
        assert.equal(pageContent.inert, true);
        assert.equal(modal.inert, false);
        assert.equal(permanentlyInert.inert, true);

        modal.hidden = true;
        core.syncModalOpenState();
        assert.equal(body.classList.contains('modal-open'), false);
        assert.equal(navigation.inert, false);
        assert.equal(pageContent.inert, false);
        assert.equal(permanentlyInert.inert, true);
    });

    it('programs only after download and preserves erase ordering with fake dependencies', async () => {
        const core = loadFlashCore();
        const calls = [];
        const image = new Uint8Array([1, 2, 3]);
        await core.programImage({
            download: async () => { calls.push('download'); return image; },
            erase: true,
            eraseFlash: async () => { calls.push('erase'); },
            writeFlash: async (received) => {
                assert.deepEqual([...received], [...image]);
                calls.push('write');
            },
            onErase: () => calls.push('erase-state'),
            onWrite: () => calls.push('write-state'),
        });
        assert.deepEqual(calls, ['download', 'erase-state', 'erase', 'write-state', 'write']);

        calls.length = 0;
        await core.programImage({
            download: async () => { calls.push('download'); return image; },
            erase: false,
            eraseFlash: async () => calls.push('erase'),
            writeFlash: async () => calls.push('write'),
            onErase: () => calls.push('erase-state'),
            onWrite: () => calls.push('write-state'),
        });
        assert.deepEqual(calls, ['download', 'write-state', 'write']);

        calls.length = 0;
        await assert.rejects(core.programImage({
            download: async () => { calls.push('download'); return image; },
            validate: async () => { calls.push('validate'); throw new Error('session ended'); },
            erase: true,
            eraseFlash: async () => calls.push('erase'),
            writeFlash: async () => calls.push('write'),
            onErase: () => calls.push('erase-state'),
            onWrite: () => calls.push('write-state'),
        }), /session ended/);
        assert.deepEqual(calls, ['download', 'validate']);
    });

    it('starts analytics with the active install attempt and limits terminal results to it', () => {
        const core = loadFlashCore();
        const loader = {};
        let reportedAttempt = null;
        const attempt = core.beginInstallAttempt(true, loader, (activeAttempt) => {
            reportedAttempt = activeAttempt;
        });

        assert.equal(reportedAttempt, attempt);
        assert.equal(attempt.erase, true);
        assert.equal(attempt.loader, loader);
        assert.equal(core.shouldReportInstallResult('flash', attempt), true);
        assert.equal(core.shouldReportInstallResult('flash', null), false);
        assert.equal(core.shouldReportInstallResult('wifi', attempt), false);
    });

    it('keeps the connected-device panels visible beside operation results', () => {
        const { stageVisible } = loadFlashCore();
        for (const activeStep of ['onboarding', 'error']) {
            assert.equal(stageVisible('review', activeStep, true), true);
            assert.equal(stageVisible(activeStep, activeStep, true), true);
        }
        assert.equal(stageVisible('review', 'select', true), false);
        assert.equal(stageVisible('review', 'error', false), false);
        assert.equal(stageVisible('error', 'error', false), true);
    });

    it('hides the serial console until the connection step is complete', () => {
        const { sessionPanelVisible } = loadFlashCore();
        assert.equal(sessionPanelVisible('select', true), false);
        assert.equal(sessionPanelVisible('review', true), true);
        assert.equal(sessionPanelVisible('review', false), false);
    });

    it('matches chip and firmware identity without trusting device URLs', () => {
        const core = loadFlashCore();
        const artifacts = [
            { chip_family: 'ESP32', url: '/firmware-esp32.bin' },
            { chip_family: 'ESP32-C3', url: '/firmware-c3.bin' },
            { chip_family: 'ESP32-C6', url: '/firmware-c6.bin' },
        ];
        assert.equal(core.selectArtifact(artifacts, 'ESP32-D0WDQ6 (revision 1)'), artifacts[0]);
        assert.equal(core.selectArtifact(artifacts, 'esp32-c3'), artifacts[1]);
        assert.equal(core.selectArtifact(artifacts, 'ESP32-C3 (QFN32) (revision v0.4)'), artifacts[1]);
        assert.equal(core.selectArtifact(artifacts, 'ESP8685 (QFN28) (revision v0.4)'), artifacts[1]);
        assert.equal(core.selectArtifact(artifacts, 'ESP32-C6 (revision 1)'), artifacts[2]);
        assert.equal(core.selectArtifact(artifacts, 'esp32c6'), artifacts[2]);
        assert.equal(core.selectArtifact(artifacts, 'ESP32-C61 (revision v0.1)'), null);
        assert.equal(core.selectArtifact(artifacts, 'ESP32-S3'), null);
        assert.equal(core.firmwareMatches(
            'esphome', { firmware: 'ESPectre ESPHome', version: 'v1.2.3' }, '1.2.3'
        ), true);
        assert.equal(core.firmwareMatches(
            'esphome', { firmware: 'francescopace.espectre', version: 'v1.2.3' }, '1.2.3'
        ), true);
        assert.equal(core.firmwareMatches(
            'native', { firmware: 'ESPectre ESPHome', version: '1.2.3' }, '1.2.3'
        ), false);
        assert.equal(core.frontendMatches(
            'native', { firmware: 'ESPectre Native', version: '1.0.0' }
        ), true);
        assert.equal(core.frontendMatches(
            'native', { firmware: 'ESPectre ESPHome', version: '1.0.0' }
        ), false);
        assert.equal(core.currentFrontend({ firmware: 'ESPectre Native' }), 'native');
        assert.equal(core.currentFrontend({ firmware: 'ESPHome ESPectre' }), 'esphome');
        assert.equal(core.currentFrontend({ firmware: 'francescopace.espectre' }), 'esphome');
        assert.equal(core.currentFrontend({ firmware: 'unknown' }), '');

        const local = core.settingsUrl(
            'https://test.espectre.dev',
            'http://192.168.1.5/?target=192.168.1.5&ignored=secret',
            true
        );
        assert.equal(local.origin, 'https://test.espectre.dev');
        assert.equal(local.pathname, '/tools/device-settings/');
        assert.equal(local.search, '?target=192.168.1.5');
        assert.equal(core.settingsUrl(
            'http://localhost:8090', 'https://host.invalid/?target=10.0.0.2', false
        ).search, '');
    });

    it('shows device settings only when the link forwards a known device target', () => {
        const link = {};
        const context = loadFlashRuntime({
            location: { origin: 'https://test.espectre.dev' },
            $$: () => [link],
        });
        for (const nextUrl of ['', 'http://[invalid', 'http://192.168.1.5/']) {
            context.flashState.nextUrl = nextUrl;
            context.flashSyncDeviceSettingsLinks('native');
            assert.equal(link.hidden, true);
            assert.equal(new URL(link.href).search, '');
        }
        context.flashState.nextUrl = 'https://espectre.dev/?target=192.168.1.5';
        context.flashSyncDeviceSettingsLinks('native');
        assert.equal(link.hidden, false);
        assert.equal(new URL(link.href).pathname, '/tools/device-settings/');
        assert.equal(new URL(link.href).searchParams.get('target'), '192.168.1.5');

        context.flashSyncDeviceSettingsLinks('matter');
        assert.equal(link.hidden, true);
        context.flashState.nextUrl = '';
        context.flashSyncDeviceSettingsLinks('native');
        assert.equal(link.hidden, true);
        assert.equal(new URL(link.href).search, '');
    });

    it('applies the upstream esptool-js SPI register correction for C5 and C6', async () => {
        const core = loadFlashCore();
        for (const chipName of ['ESP32-C5', 'ESP32-C6']) {
            const loader = {
                chip: null,
                async readFlashId() { return this.chip.SPI_REG_BASE; },
            };
            core.applyEsptoolSpiRegisterFix(loader);
            loader.chip = { CHIP_NAME: chipName, SPI_REG_BASE: 0x60002000 };
            assert.equal(await loader.readFlashId(), 0x60003000);
        }
    });

    it('uses the esp-web-tools hard-reset sequence after flashing', async () => {
        const core = loadFlashCore();
        const calls = [];
        await core.hardResetLoader(
            { after: async (...args) => calls.push(['after', ...args]) },
            { setRTS: async (value) => calls.push(['rts', value]) }
        );
        assert.deepEqual(calls, [['rts', true], ['after']]);
    });

    it('recognizes Native and Matter from serial app metadata', () => {
        const core = loadFlashCore();
        for (const frontend of ['native', 'matter']) {
            const log = [
                'ESP-ROM:esp32s3-20210327',
                `\x1b[0;32mI (100) app_init: Project name:     espectre-${frontend}\x1b[0m`,
                'I (101) app_init: App version:      2.8.0-417-g2b49a9c',
                '',
            ].join('\n');
            const identity = core.serialIdentity(log);
            assert.equal(core.currentFrontend(identity), frontend);
            assert.equal(identity.version, '2.8.0-417-g2b49a9c');
            assert.equal(identity.chipFamily, 'ESP32-S3');
        }
    });

    it('uses the ESPHome project version instead of the framework version', () => {
        const core = loadFlashCore();
        const log = [
            'ESP-ROM:esp32s3-20210327',
            'I (100) app_init: Project name:     custom-bedroom-node',
            'I (101) app_init: App version:      2026.7.4',
            '[I][app:151]: ESPHome version 2026.7.4 compiled on Sep 4 2026',
            '[I][app:153]: Project francescopace.espectre version 2.8.0-417-g2b49a9c',
            '',
        ].join('\n');
        const identity = core.serialIdentity(log);
        assert.equal(core.currentFrontend(identity), 'esphome');
        assert.equal(identity.version, '2.8.0-417-g2b49a9c');
        assert.equal(identity.chipFamily, 'ESP32-S3');
    });

    it('waits for complete version lines and rejects unidentified serial output', () => {
        const core = loadFlashCore();
        const partial = 'Project name: espectre-native\nApp version: 2.8';
        assert.equal(core.serialIdentity(partial), null);
        assert.equal(core.serialIdentity(partial + '.0\n').version, '2.8.0');
        for (const log of [
            'ESP-ROM:esp32s3-20210327\n',
            'Project name: unrelated\nApp version: 2.8.0\n',
            'Project name: espectre\nApp version: 2026.7.4\n',
            'Project somebody.espectre version 2.8.0\n',
            'MATTER_QR=MT:EXAMPLE\nMATTER_MANUAL_CODE=12345678901\n',
        ]) {
            assert.equal(core.serialIdentity(log), null);
        }
        for (const frontend of ['native', 'matter']) {
            const identity = core.serialIdentity(`ESPectre ${frontend} firmware started\n`);
            assert.equal(core.currentFrontend(identity), frontend);
            assert.equal(identity.version, '');
        }
    });

    it('parses Matter serial onboarding codes independently of firmware identity', () => {
        const core = loadFlashCore();
        const codes = core.matterCodes('MATTER_QR=MT:EXAMPLE\nMATTER_MANUAL_CODE=12345678901\n');
        assert.equal(codes.qr, 'MT:EXAMPLE');
        assert.equal(codes.manual, '12345678901');
    });

    it('recognizes Micro-ESPectre startup and distinguishes its application version from MicroPython', () => {
        const core = loadFlashCore();
        for (const chip of ['ESP32', 'C3', 'C5', 'C6', 'S2', 'S3']) {
            const info = core.serialIdentity(`[INFO] Micro-ESPectre starting...\n[INFO] Detected chip: ${chip}\n`);
            assert.equal(info.firmware, 'Micro-ESPectre');
            assert.equal(info.chipFamily, chip === 'ESP32' ? chip : `ESP32-${chip}`);
            assert.equal(info.version, '');
            assert.equal(info.inferred, undefined);
            assert.equal(core.currentFrontend(info), '');
            assert.equal(core.frontendMatches('native', info), false);
        }
        const branded = core.serialIdentity('MicroPython v1.27.0 on 2026-08-31; ESPectre ESP32-C5 with ESP32-C5\n');
        assert.equal(branded.firmware, 'Micro-ESPectre');
        assert.equal(branded.version, '');
        assert.equal(branded.runtimeVersion, 'v1.27.0');
        assert.equal(branded.inferred, true);
        const generic = core.serialIdentity('MicroPython v1.27.0 on 2026-08-31; Generic ESP32-C5 with ESP32-C5\n');
        assert.equal(generic.firmware, 'MicroPython');
        assert.equal(generic.version, 'v1.27.0');
        assert.equal(generic.inferred, undefined);
        const named = core.serialIdentity('ESP-ROM:esp32c5-20250101\nProject name: micro-espectre\nApp version: 2.8.0-417-g2b49a9c\n');
        assert.equal(named.firmware, 'Micro-ESPectre');
        assert.equal(named.version, '2.8.0-417-g2b49a9c');
        assert.equal(named.inferred, undefined);
        assert.equal(core.serialIdentity('ESP-ROM:esp32c5-20250101\nProject name: micro-espectre\nApp version: 2.8.'), null);
        assert.equal(core.serialIdentity('Micro-ESPectre starting...\n'), null);
    });

    it('stops USB detection at the first successful identity source for each frontend', async () => {
        for (const frontend of ['native', 'matter', 'esphome', 'micro', 'micropython']) {
            const micro = frontend === 'micro' || frontend === 'micropython';
            for (const source of micro ? ['serial'] : ['improv', 'serial', 'descriptor']) {
                const calls = [];
                const info = {
                    firmware: micro ? (frontend === 'micro' ? 'Micro-ESPectre' : 'MicroPython') : `ESPectre ${frontend}`,
                    version: micro ? '' : '2.8.0', chipFamily: 'ESP32-S3'
                };
                const context = loadFlashRuntime({
                    browserSupport: { flash: true },
                    navigator: { serial: { requestPort: async () => ({}) } },
                    document: { getElementById: () => ({ options: [{ value: frontend }] }) },
                    $: () => ({}),
                    track() {},
                    toast() {},
                });
                const flash = context.flashState;
                Object.assign(context, {
                    flashNotifyTransition() {},
                    flashParams: () => ({}),
                    flashLoadHeadless: async () => ({}),
                    flashActivateUsb: (port) => { flash.port = port; },
                    flashProbeImprov: async () => {
                        calls.push('improv');
                        if (source !== 'improv') return null;
                        flash.currentInfo = info;
                        return { info };
                    },
                    flashProbeFirmware: async () => {
                        calls.push('serial');
                        if (source !== 'serial') return null;
                        flash.currentInfo = info;
                        return info;
                    },
                    flashEnterLoader: async () => {
                        calls.push('loader');
                        flash.loader = {};
                        return 'ESP32-S3';
                    },
                    flashReadInstalledFirmwareInfo: async () => {
                        calls.push('descriptor');
                        return info;
                    },
                    flashAnyFrontendSupportsChip: (chip) => chip === 'ESP32-S3',
                    flashFrontendSupportsChip: () => true,
                    flashSyncInstallChoice: () => true,
                    flashSetStep: (step) => { flash.step = step; },
                    flashFail: (error) => { throw error; },
                    flashReadMatterInput: async () => { calls.push('pairing'); },
                });
                await context.flashDetect();
                assert.deepEqual(calls, source === 'improv' ? ['improv']
                    : source === 'serial' ? ['improv', 'serial']
                        : ['improv', 'serial', 'loader', 'descriptor']);
                assert.equal(flash.detectedFrontend, micro ? '' : frontend);
                assert.equal(flash.currentInfo.version, info.version);
                assert.equal(flash.step, 'review');
                assert.equal(flash.installChoice, micro ? 'change' : 'update');
            }
        }
    });

    it('restarts a detected Matter app from the bootloader before requesting its Improv codes', async () => {
        const calls = [];
        const info = { firmware: 'ESPectre Matter', version: '2.8.0' };
        const context = loadFlashRuntime();
        const flash = context.flashState;
        Object.assign(flash, { loader: {}, currentInfo: info });
        Object.assign(context, {
            flashHardResetLoader: async () => { calls.push('reset'); },
            flashCloseMode: async () => { calls.push('close'); flash.loader = null; },
            flashDelay: async () => {},
            flashLoadHeadless: async () => ({}),
            flashProbeImprov: async () => {
                calls.push('improv');
                flash.improv = {
                    info,
                    requestMatterOnboarding: async () => {
                        calls.push('pairing');
                        return { qr: 'MT:EXAMPLE', manual: '12345678901' };
                    },
                };
                return flash.improv;
            },
            flashReadSerial: async () => { calls.push('serial'); },
        });
        const codes = await context.flashReadMatterInput();
        assert.deepEqual(calls, ['reset', 'close', 'improv', 'pairing']);
        assert.equal(codes.qr, 'MT:EXAMPLE');
        assert.equal(codes.manual, '12345678901');
    });

    it('replaces the detected firmware identity after changing firmware type', () => {
        const core = loadFlashCore();
        const installed = core.installedInfo(
            'matter',
            '2.9.0',
            'ESP32-S3',
            'Matter',
            { firmware: 'ESPectre Native', version: '2.8.0', chipFamily: 'ESP32-S3' }
        );
        assert.equal(installed.firmware, 'ESPectre Matter');
        assert.equal(installed.version, '2.9.0');
        assert.equal(installed.chipFamily, 'ESP32-S3');
        assert.equal(core.currentFrontend(installed), 'matter');
    });

    it('normalizes the ESPHome project identity after installation', () => {
        const core = loadFlashCore();
        const installed = core.installedInfo(
            'esphome',
            '2.8.0-417-g2b49a9c',
            'ESP32-S3',
            'ESPHome',
            {
                firmware: 'francescopace.espectre',
                version: '2.8.0-417-g2b49a9c',
                chipFamily: 'ESP32-S3'
            }
        );
        assert.equal(installed.firmware, 'ESPectre ESPHome');
        assert.equal(installed.version, '2.8.0-417-g2b49a9c');
    });

    it('identifies supported firmware from installed app descriptors', async () => {
        const core = loadFlashCore();
        const table = new Uint8Array(0xC00);
        table[0] = 0xAA;
        table[1] = 0x50;
        table[2] = 0x00;
        table[3] = 0x10;
        new DataView(table.buffer).setUint32(4, 0x20000, true);
        new DataView(table.buffer).setUint32(8, 0x3D0000, true);

        for (const [project, version, frontend, expectedVersion] of [
            ['espectre-native', '2.8.0-417-g2b49a9c', 'native', '2.8.0-417-g2b49a9c'],
            ['espectre-matter', '2.8.0-417-g2b49a9c', 'matter', '2.8.0-417-g2b49a9c'],
            ['espectre', '2026.7.4', 'esphome', ''],
            ['unrelated', '1.0.0', '', ''],
        ]) {
            const header = new Uint8Array(0x100);
            header[0] = 0xE9;
            new DataView(header.buffer).setUint32(0x20, 0xABCD5432, true);
            header.set(new TextEncoder().encode(version), 0x30);
            header.set(new TextEncoder().encode(project), 0x50);

            const fixture = flashReadFixture((address) => address === 0x8000 ? table : header);
            const info = await core.readInstalledFirmwareInfo(fixture.loader, 'ESP32-S3');
            assert.deepEqual(fixture.reads, [[0x8000, 0xC00], [0x20000, 0x100]]);
            assert.equal(fixture.packets.length, 0);
            assert.equal(core.currentFrontend(info), frontend);
            if (frontend) {
                assert.equal(info.version, expectedVersion);
                assert.equal(info.chipFamily, 'ESP32-S3');
            } else {
                assert.equal(info, null);
            }
        }
    });

    it('marks a blank MicroPython-style flash layout as inferred and preserves known descriptor identity', async () => {
        const context = loadFlashRuntime({ console: { info() {}, warn() {} } });
        const table = new Uint8Array(3072);
        const entries = [[1, 2, 0x9000, 0x6000], [1, 1, 0xf000, 0x1000], [0, 0, 0x10000, 0x1f0000]];
        entries.forEach(([type, subtype, address, size], index) => {
            const offset = index * 32;
            table.set([0xaa, 0x50, type, subtype], offset);
            const view = new DataView(table.buffer);
            view.setUint32(offset + 4, address, true);
            view.setUint32(offset + 8, size, true);
        });
        const header = new Uint8Array(256);
        header[0] = 0xe9;
        new DataView(header.buffer).setUint32(0x20, 0xabcd5432, true);
        const identify = async () => {
            const fixture = flashReadFixture((address) => address === 0x8000 ? table : header);
            return context.flashReadInstalledFirmwareInfo(fixture.loader, 'esp32c5');
        };
        const inferred = await identify();
        assert.equal(inferred.firmware, 'MicroPython');
        assert.equal(inferred.inferred, true);
        assert.equal(inferred.version, '');
        assert.equal(inferred.chipFamily, 'ESP32-C5');
        header.set(new TextEncoder().encode('unrelated'), 0x50);
        assert.equal(await identify(), null);
        header.fill(0, 0x50, 0x70);
        table[3] = 3;
        assert.equal(await identify(), null);
        header.set(new TextEncoder().encode('micropython'), 0x50);
        header.set(new TextEncoder().encode('v1.27.0'), 0x30);
        const named = await identify();
        assert.equal(named.firmware, 'MicroPython');
        assert.equal(named.inferred, undefined);
        assert.equal(named.version, 'v1.27.0');
        header.fill(0, 0x50, 0x70);
        header.set(new TextEncoder().encode('micro-espectre'), 0x50);
        const micro = await identify();
        assert.equal(micro.firmware, 'Micro-ESPectre');
        assert.equal(micro.version, 'v1.27.0');
        assert.equal(micro.inferred, undefined);
    });

    it('drains the digest before another read without changing the installation transport', async () => {
        const context = loadFlashRuntime();
        const fixture = flashReadFixture((_address, size) => new Uint8Array(size));
        const originalTransport = fixture.loader.transport;
        for (const address of [0x8000, 0x20000]) {
            const bytes = await context.flashReadFirmwareBytes(fixture.loader, address, 256, 'test');
            assert.equal(bytes.length, 256);
        }
        assert.equal(fixture.loader.transport, originalTransport);
        assert.equal(fixture.packets.length, 0);
    });

    it('reads metadata in bounded blocks with cumulative ACKs and rejects truncated frames before ACK', async () => {
        const context = loadFlashRuntime({ console: { info() {}, warn() {} } });
        const blockSize = vm.runInContext('FLASH_METADATA_BLOCK_SIZE', context);
        const size = blockSize * 2 + 256;
        const fixture = flashReadFixture((_address, length) => new Uint8Array(length));
        await context.flashReadFirmwareBytes(fixture.loader, 0x8000, size, 'test');
        assert.deepEqual(fixture.writes.slice(1).map((packet) =>
            new DataView(packet.buffer, packet.byteOffset).getUint32(0, true)),
        [blockSize, blockSize * 2, size]);
        assert.equal(fixture.packets.length, 0);

        for (const corruptBlock of [0, 1]) {
            const broken = flashReadFixture((_address, length) => new Uint8Array(length));
            const readPacket = broken.transport.read;
            let frame = -1;
            broken.transport.read = async () => {
                const packet = await readPacket();
                return frame++ === corruptBlock ? packet.slice(0, -115) : packet;
            };
            await assert.rejects(context.flashReadFirmwareBytes(broken.loader, 0x8000, size, 'test'),
                { name: 'UsbFlashReadError' });
            assert.equal(broken.writes.length, 1 + corruptBlock);
        }
    });

    it('decodes fragmented SLIP input and releases the reader on cancellation', async () => {
        let input;
        const port = {
            readable: new ReadableStream({ start(controller) { input = controller; } }),
        };
        const transport = new Transport(port, false);
        const payload = new Uint8Array([1, 0xc0, 0xdb, 4]);
        const wire = transport.slipWriter(payload);
        const loop = transport.readLoop();
        const decoded = transport.read(1000);
        input.enqueue(wire.slice(0, 4));
        await new Promise((resolve) => setImmediate(resolve));
        input.enqueue(wire.slice(4));
        assert.deepEqual(await decoded, payload);
        await transport.reader.cancel();
        await loop;
        assert.equal(port.readable.locked, false);
        assert.equal(transport.reader, undefined);
    });

    it('opens the bootloader transport with the configured receive buffer', async () => {
        const context = loadFlashRuntime();
        let options;
        context.flashState.port = { async open(value) { options = value; } };
        Object.assign(context, {
            flashCloseMode: async () => {},
            flashWaitForSerialReopen: async () => {},
            flashReportUsbStep() {},
            flashApplyEsptoolSpiRegisterFix() {},
        });
        await context.flashEnterLoader({
            Transport,
            ESPLoader: class {
                constructor(value) { this.options = value; }
                async main() {
                    const { transport, baudrate, serialOptions } = this.options;
                    await transport.connect(baudrate, serialOptions);
                    return 'ESP32-C5';
                }
            },
        });
        assert.equal(options.bufferSize, vm.runInContext('FLASH_SERIAL_BUFFER_SIZE', context));
        assert.equal(options.baudRate, vm.runInContext('FLASH_SERIAL_BAUD', context));
    });

    it('resumes recoverable serial input failures and reports device loss for fatal errors', async () => {
        for (const [name, recoverable] of [['BufferOverrunError', true], ['NetworkError', false]]) {
            let input;
            let current = new ReadableStream({ start(controller) { input = controller; } });
            let disconnected = 0;
            const port = { get readable() { return current; } };
            const transport = new Transport(port, false);
            transport.setDeviceLostCallback(() => { disconnected += 1; });
            const loop = transport.readLoop();
            const first = current;
            current = new ReadableStream({
                start(controller) {
                    controller.enqueue(new Uint8Array([7, 8]));
                    controller.close();
                },
            });
            const error = new Error('test');
            error.name = name;
            input.error(error);
            await loop;
            assert.equal(transport.buffer.length, recoverable ? 2 : 0);
            assert.equal(disconnected, recoverable ? 0 : 1);
            assert.equal(first.locked, false);
            assert.equal(current.locked, false);
        }
    });

    it('bounds every flash-read I/O stage and prevents late ACKs after cancellation', async () => {
        for (const blockedStage of ['command_write', 'command_response', 'data_read', 'ack_write', 'digest_read']) {
            let deadline;
            let signalBlocked;
            let resume;
            let cancelled = 0;
            let aborted = 0;
            let released = 0;
            let writeCount = 0;
            let readCount = 0;
            let deadlineCleared = false;
            const blocked = new Promise((resolve) => { signalBlocked = resolve; });
            const pause = () => {
                signalBlocked();
                return new Promise((resolve) => { resume = resolve; });
            };
            const context = loadFlashRuntime({
                setTimeout: (callback) => { deadline = callback; return 1; },
                clearTimeout: () => { deadlineCleared = true; },
            });
            const fixture = flashReadFixture((_address, size) => new Uint8Array(size));
            fixture.transport.reader = { async cancel() { cancelled += 1; } };
            const originalRead = fixture.transport.read;
            fixture.transport.read = async (...args) => {
                const stage = ['command_response', 'data_read', 'digest_read'][readCount++];
                if (stage === blockedStage) await pause();
                return originalRead(...args);
            };
            const getWriter = fixture.transport.device.writable.getWriter;
            fixture.transport.device.writable.getWriter = () => {
                const writer = getWriter();
                return {
                    async write(packet) {
                        const stage = writeCount++ === 0 ? 'command_write' : 'ack_write';
                        if (stage === blockedStage) await pause();
                        return writer.write(packet);
                    },
                    async abort() { aborted += 1; },
                    releaseLock() { released += 1; },
                };
            };
            const result = context.flashReadFirmwareBytes(fixture.loader, 0x8000, 4, 'partition_table');
            await blocked;
            const rejected = assert.rejects(result, { name: 'UsbFlashReadError', stage: blockedStage, phase: 'partition_table' });
            deadline();
            await rejected;
            assert.equal(cancelled, 1);
            assert.equal(aborted, blockedStage.endsWith('_write') ? 1 : 0);
            assert.equal(deadlineCleared, true);
            const writesAtTimeout = writeCount;
            resume();
            await new Promise((resolve) => setImmediate(resolve));
            assert.equal(writeCount, writesAtTimeout);
            assert.equal(released, writeCount);
        }
    });

    it('propagates flash transport failures instead of treating them as unidentified firmware', async () => {
        const context = loadFlashRuntime({ console: { info() {}, warn() {} } });
        const fixture = flashReadFixture(() => new Uint8Array(0));
        await assert.rejects(
            context.flashReadInstalledFirmwareInfo(fixture.loader, 'ESP32-S3'),
            { name: 'UsbFlashReadError' },
        );
        assert.equal(fixture.writes.length, 1);
    });

    it('shows USB errors before blocked cleanup and bounds waiting without releasing the pending session', async () => {
        let deadline;
        let finishClose;
        const detail = { textContent: '' };
        const context = loadFlashRuntime({
            $: () => detail,
            setTimeout: (callback) => { deadline = callback; return 1; },
            clearTimeout() {},
            console: { warn() {} },
        });
        Object.assign(context, {
            flashShowProgress() {},
            flashShouldReportInstallResult: () => false,
            flashSetStep: (step) => { context.flashState.step = step; },
            flashSetState: (state) => { context.flashState.state = state; },
            flashSyncControls() {},
            flashReportUsbStep() {},
        });
        const transport = { disconnect: () => new Promise((resolve) => { finishClose = resolve; }) };
        context.flashState.transport = transport;
        context.flashState.mode = 'loader';
        const failing = context.flashFail(new Error('test failure'));
        assert.equal(context.flashState.state, 'error');
        assert.equal(context.flashState.step, 'error');
        const closing = context.flashState.closePromise;
        assert.ok(closing);
        await Promise.resolve();
        deadline();
        await failing;
        assert.equal(context.flashState.closePromise, closing);
        assert.equal(context.flashState.loader, null);
        assert.equal(context.flashState.transport, null);
        finishClose();
        await closing;
        assert.equal(context.flashState.closePromise, null);
    });

    it('matches the CLI flash regions while preserving device data', () => {
        const core = loadFlashCore();
        const factory = new Uint8Array(0x50000).fill(0xFF);
        const partition = (offset, type, subtype, address, size) => {
            factory[offset] = 0xAA;
            factory[offset + 1] = 0x50;
            factory[offset + 2] = type;
            factory[offset + 3] = subtype;
            new DataView(factory.buffer).setUint32(offset + 4, address, true);
            new DataView(factory.buffer).setUint32(offset + 8, size, true);
        };
        factory[0] = 0xE9;
        factory[1] = 0;
        factory[23] = 0;
        partition(0x8000, 0x01, 0x02, 0x9000, 0x5000);
        partition(0x8020, 0x01, 0x00, 0xE000, 0x2000);
        partition(0x8040, 0x01, 0x04, 0x10000, 0x1000);
        partition(0x8060, 0x00, 0x10, 0x20000, 0x10000);
        partition(0x8080, 0x00, 0x11, 0x30000, 0x10000);
        factory[0x20000] = 0xE9;
        factory[0x20001] = 0;
        factory[0x20017] = 0;
        const parts = core.preservedParts({ factory, update: null });
        assert.deepEqual(Array.from(parts, (part) => part.address), [0, 0x8000, 0xE000, 0x20000]);
        assert.deepEqual(Array.from(parts, (part) => part.data.length), [32, 0xC00, 0x2000, 32]);
        assert.equal(parts.some((part) => part.address === 0x9000), false);
        assert.equal(parts.some((part) => part.address === 0x30000), false);
    });

    it('bounds and sanitizes the advanced serial console', () => {
        const core = loadFlashCore();
        const state = { active: false };
        const safe = core.sanitizeConsole(
            '\u001b[31mvisible\u001b[0m\n[[secret:start]]\npassword\n[[secret:end]]\ndone\n',
            state
        );
        assert.equal(safe, 'visible\n[sensitive output redacted]\ndone\n');
        assert.equal(state.active, false);
        assert.equal(core.limitConsole('12345', '67890', 6), '567890');
        assert.equal(core.consoleLimit, 512 * 1024);
    });

    it('renders split ANSI color sequences without exposing terminal HTML', () => {
        const renderer = new AnsiUp();
        renderer.url_allowlist = {};
        assert.equal(renderer.ansi_to_html('\x1b[0;'), '');
        const html = renderer.ansi_to_html('32mready <safe>\x1b[0m');
        assert.match(html, /color:rgb\(0,187,0\)/);
        assert.match(html, /ready &lt;safe&gt;/);
        assert.doesNotMatch(html, /\[0;32m|<safe>/);
    });

    it('offers the shared connection picker from every connected browser tool', () => {
        for (const tool of ['configure', 'monitor', 'raw-csi', 'theremin', 'game']) {
            assert.match(
                toolContent[tool],
                new RegExp(`<espectre-connection-picker[^>]*data-surface="${tool}"`)
            );
        }
    });

    it('keeps MQTT configuration values in Device settings', () => {
        const configure = toolContent.configure;
        assert.match(configure, /id="cfg-mqtt-scheme"/);
        assert.match(configure, /id="cfg-mqtt-host"/);
        assert.match(configure, /id="cfg-mqtt-port"/);
        assert.match(configure, /id="cfg-topic-prefix"[^>]*value="espectre\/v1\/devices"/);
        assert.match(configure, /id="cfg-mqtt-credentials-clear"/);
        assert.doesNotMatch(configure, /id="cfg-mqtt-user"[^>]*value=/);
        assert.doesNotMatch(configure, /id="cfg-mqtt-pass"[^>]*value=/);
    });

    it('publishes one Raw CSI visualization selector with stable option values', () => {
        const rawCsi = toolContent['raw-csi'];
        const values = [...rawCsi.matchAll(/<option value="([^"]+)"/g)].map((match) => match[1]);
        assert.deepEqual(values, [
            'subcarrier-amplitudes',
            'csi-amplitude-surface',
            'channel-profile-deviation',
            'iq-constellation',
            'relative-phase-trails',
        ]);
        assert.equal((rawCsi.match(/class="js-raw-visualization"/g) || []).length, 1);
    });

    it('keeps unavailable Relay controls inert', () => {
        const template = index.match(/<template id="connection-picker-template">[\s\S]*?<\/template>/)?.[0] || '';
        const relay = template.match(/data-connection-panel="relay"[\s\S]*?<\/section>/)?.[0] || '';
        assert.match(relay, /class="btn-primary" disabled/);
        assert.doesNotMatch(relay, /<input|<select|js-connect/);
    });
});
