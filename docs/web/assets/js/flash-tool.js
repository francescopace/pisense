/*
 * ESPectre - Headless Web Serial firmware installer
 *
 * Part of the website application shell.
 *
 * Author: Francesco Pace <francescopace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

'use strict';

    const FLASH_SERIAL_BUNDLE = '/vendor/espectre-web-serial-0.6.1-2.8.1/headless.js?v=5';
    const FLASH_ANSI_BUNDLE = '/vendor/ansi_up-6.0.6/ansi_up.js';
    const FLASH_SERIAL_BAUD = 115200;
    const FLASH_IMPROV_PROBE_TIMEOUT_MS = 1500;
    const FLASH_IMPROV_BOOT_TIMEOUT_MS = 10000;
    const FLASH_FIRMWARE_PROBE_TIMEOUT_MS = 10000;
    const FLASH_METADATA_READ_TIMEOUT_MS = 10000;
    const FLASH_METADATA_BLOCK_SIZE = 1024;
    const FLASH_SERIAL_BUFFER_SIZE = 64 * 1024;
    const FLASH_CLOSE_TIMEOUT_MS = 3000;
    const FLASH_SERIAL_SETTLE_MS = 250;
    const FLASH_SERIAL_REOPEN_DELAY_MS = 500;
    const FLASH_PROVISION_TIMEOUT_MS = 45000;
    const FLASH_CONSOLE_LIMIT = 512 * 1024;
    const FLASH_PARTITION_TABLE_LENGTH = 0xC00;
    const FLASH_APP_DESCRIPTOR_LENGTH = 0x100;
    const FLASH_APP_DESCRIPTOR_MAGIC = 0xABCD5432;
    const FLASH_FRONTEND_ORDER = ['native', 'esphome', 'matter'];
    const FLASH_CHANNELS = Object.freeze([
        Object.freeze({ value: 'release', label: 'Release' }),
        Object.freeze({ value: 'preview', label: 'Preview' }),
        Object.freeze({ value: 'develop', label: 'Development' })
    ]);
    const FLASH_FRONTEND_DESCRIPTIONS = Object.freeze({
        native: 'Built-in web tools and Direct HTTP.',
        esphome: 'ESPHome and Home Assistant integration.',
        matter: 'Matter smart home integration.'
    });
    const FLASH_FIRMWARE_NAMES = Object.freeze({
        native: 'ESPectre Native',
        esphome: 'ESPectre ESPHome',
        matter: 'ESPectre Matter'
    });
    const FLASH_CHIP_ORDER = ['esp32', 'esp32s2', 'esp32s3', 'esp32c3', 'esp32c5', 'esp32c6'];
    const FLASH_CRITICAL_STATES = new Set(['erase', 'write']);
    const FLASH_TRANSITION_STATES = new Set(['selecting', 'detecting', 'restart']);
    const FLASH_FLOW_MODAL_STEPS = new Set(['onboarding', 'error']);

    const flash = {
        manifests: {}, catalogReports: new Set(), refreshRequest: 0, loadedChannel: '',
        channelVersionFailures: new Set(), channelVersionsPromise: null,
        targetVersion: '', supportedChipLabels: [], selectedArtifact: null,
        selectedUpdateArtifact: null, frontends: {},
        detectedChip: '', usbDialog: null, usbPortInfo: null,
        port: null, transport: null, loader: null, improv: null,
        reader: null, mode: null, state: 'loading', step: 'select',
        currentInfo: null, detectedFrontend: '', installChoice: 'change', nextUrl: '',
        sameFirmware: false, requiresErase: true,
        resultReported: false, dependency: null, initialized: false,
        installMenus: {},
        eraseDialogResolve: null, eraseDialogReturnFocus: null,
        flowDialogReturnFocus: null,
        consoleOpen: false, consoleText: '', consoleAnsiText: '', consoleRedacting: false,
        ansiDependency: null, ansiConstructor: null, ansiRenderer: null,
        attempt: null, flow: 'flash', operation: 0,
        closePromise: null, lastSerialCloseAt: 0,
        activityTimer: null
    };

    function flashDelay(ms) {
        return new Promise((resolve) => setTimeout(resolve, ms));
    }

    async function flashWithTimeout(promise, timeoutMs, createError) {
        let timer;
        try {
            return await Promise.race([
                promise,
                new Promise((_resolve, reject) => {
                    timer = setTimeout(() => reject(createError()), timeoutMs);
                })
            ]);
        } finally {
            clearTimeout(timer);
        }
    }

    function flashWaitForClose(promise) {
        return flashWithTimeout(promise, FLASH_CLOSE_TIMEOUT_MS, () =>
            new Error('USB cleanup is still pending. Unplug the board, reconnect it, and try again.'));
    }

    async function flashWaitForSerialReopen() {
        const remaining = FLASH_SERIAL_REOPEN_DELAY_MS - (Date.now() - flash.lastSerialCloseAt);
        if (remaining > 0) await flashDelay(remaining);
    }

    function flashUsbId(info) {
        if (!info || (!info.usbVendorId && !info.usbProductId)) return '';
        const hex = (value) => Number(value || 0).toString(16).toUpperCase().padStart(4, '0');
        return hex(info.usbVendorId) + ':' + hex(info.usbProductId);
    }

    function flashUsbIdentity() {
        const chip = String(flash.detectedChip || '').toUpperCase();
        const portId = flashUsbId(flash.usbPortInfo);
        return {
            deviceName: chip || portId || 'USB device',
            chip,
            deviceId: portId,
            firmwareVersion: flash.targetVersion
        };
    }

    function flashActivateUsb(port, updateUi = true) {
        flash.port = port;
        flash.usbDialog = { port };
        try {
            flash.usbPortInfo = port.getInfo();
        } catch (_error) {
            flash.usbPortInfo = null;
        }
        if (!updateUi) return;
        dropdownOpen = false;
        renderConnection();
        syncFirmwareUpdateNotice();
        flashSyncUsbActions();
        const connect = $('.js-flash-connect-label');
        if (connect) connect.textContent = 'Continue with selected device';
    }

    function flashReleaseUsb() {
        flash.usbDialog = null;
        flash.usbPortInfo = null;
        flash.detectedChip = '';
        dropdownOpen = false;
        renderConnection();
        syncFirmwareUpdateNotice();
        flashSyncUsbActions();
        const connect = $('.js-flash-connect-label');
        if (connect) connect.textContent = 'Connect USB device';
    }

    function flashParams() {
        return {
            frontend: document.getElementById('flash-frontend')?.value || '',
            channel: document.getElementById('flash-channel')?.value || '',
            chip: flash.detectedChip
        };
    }

    function flashReportResult(result) {
        if (flash.resultReported) return;
        flash.resultReported = true;
        track('firmware_install_result', { ...flashParams(), result });
    }

    function flashBeginInstallAttempt(erase, loader, onStart) {
        const attempt = { erase, loader };
        flash.attempt = attempt;
        onStart(attempt);
        return attempt;
    }

    function flashShouldReportInstallResult(flow = flash.flow, attempt = flash.attempt) {
        return flow === 'flash' && Boolean(attempt);
    }

    function flashStageVisible(stageStep, activeStep, reviewing) {
        return stageStep === activeStep
            || (reviewing && stageStep === 'review' && activeStep !== 'select');
    }

    function flashSetStep(step) {
        flash.step = step;
        const reviewing = Boolean(flash.detectedChip && flash.port);
        $$('.js-flash-stage').forEach((stage) => {
            stage.hidden = !flashStageVisible(stage.dataset.flashStep, step, reviewing);
        });
        const modal = $('.js-flash-flow-modal');
        const showModal = FLASH_FLOW_MODAL_STEPS.has(step);
        if (showModal && modal.hidden) flash.flowDialogReturnFocus = document.activeElement;
        modal.hidden = !showModal;
        syncModalOpenState();
        if (showModal) {
            modal.querySelector('[data-flash-step="' + step + '"]')?.focus();
        }
        flashSyncUsbActions();
    }

    function flashSetFlow(flow) {
        flash.flow = flow;
    }

    function flashSetState(state, message = '') {
        flash.state = state;
        flashSyncActivity(state, message);
        if (message) toast(message);
    }

    function flashSyncActivity(state, message = '') {
        const active = FLASH_TRANSITION_STATES.has(state);
        if (!active) {
            clearInterval(flash.activityTimer);
            flash.activityTimer = null;
        }
        const connect = $('.js-flash-connect');
        const idleLabel = $('.js-flash-connect-label');
        const spinner = $('.js-flash-activity-spinner');
        const label = $('.js-flash-activity-label');
        const elapsed = $('.js-flash-activity-elapsed');
        if (!connect || !idleLabel || !spinner || !label || !elapsed) return;
        connect.setAttribute('aria-busy', String(active));
        idleLabel.hidden = active;
        spinner.hidden = !active;
        label.hidden = !active;
        elapsed.hidden = !active;
        if (!active) return;
        label.textContent = message || {
            selecting: 'Waiting for USB device selection…',
            detecting: 'Identifying the connected device…',
            restart: 'Restarting the device…'
        }[state];
        if (flash.activityTimer !== null) return;
        const startedAt = Date.now();
        const updateElapsed = () => {
            elapsed.textContent = '(' + Math.floor((Date.now() - startedAt) / 1000) + ' s)';
        };
        updateElapsed();
        flash.activityTimer = setInterval(updateElapsed, 1000);
    }

    function flashNotifyTransition(state, message) {
        flashSetState(state, message);
        flashSyncControls();
    }

    function flashReportUsbStep(message) {
        if (!FLASH_TRANSITION_STATES.has(flash.state)) return;
        flashSyncActivity(flash.state, message);
    }

    function flashTrackUsbMethod(target, method, message, nextMessage = '') {
        const original = target[method];
        if (typeof original !== 'function') return;
        const operation = flash.operation;
        target[method] = async function (...args) {
            if (operation === flash.operation) {
                flashReportUsbStep(typeof message === 'function' ? message() : message);
            }
            const result = await original.apply(this, args);
            if (nextMessage && operation === flash.operation) flashReportUsbStep(nextMessage);
            return result;
        };
    }

    function flashSetProgress(value, label) {
        const progress = $('.js-flash-progress');
        const bounded = Math.max(0, Math.min(100, Math.round(value)));
        progress.value = bounded;
        progress.textContent = bounded + '%';
        $('.js-flash-progress-label').textContent = label;
    }

    function flashShowProgress(visible) {
        const panel = $('.js-flash-progress-card');
        if (panel) panel.hidden = !visible;
    }

    function flashManifestFrontends(manifest) {
        const frontends = manifest && manifest.frontends;
        if (!frontends || typeof frontends !== 'object' || Array.isArray(frontends)) {
            const error = new Error('Firmware catalog format is invalid for browser flashing.');
            error.name = 'FirmwareCatalogFormatError';
            throw error;
        }
        return frontends;
    }

    function flashPreferredOrder(order, a, b) {
        const left = order.indexOf(a);
        const right = order.indexOf(b);
        if (left === -1 && right === -1) return 0;
        if (left === -1) return 1;
        if (right === -1) return -1;
        return left - right;
    }

    async function flashLoadManifest(channel) {
        if (flash.manifests[channel]) return flash.manifests[channel];
        const response = await fetch(
            '/artifacts/firmware/' + channel + '/firmware-manifest-' + channel + '.json',
            { cache: 'no-store' }
        );
        if (!response.ok) {
            const error = new Error('Unable to load the ' + channel + ' firmware catalog.');
            error.status = response.status;
            throw error;
        }
        const manifest = await response.json();
        flash.manifests[channel] = manifest;
        return manifest;
    }

    function flashCreateChannelMenu(mountSelector, initialLabel, onSelect) {
        const mount = $(mountSelector);
        const root = document.createElement('details');
        root.className = 'sdk-download flash-install-menu';
        const toggle = document.createElement('summary');
        toggle.className = 'btn-primary btn-sm';
        toggle.setAttribute('aria-disabled', 'true');
        const label = document.createElement('span');
        label.textContent = initialLabel;
        toggle.appendChild(label);
        toggle.addEventListener('click', (event) => {
            if (toggle.getAttribute('aria-disabled') === 'true') event.preventDefault();
        });
        const menu = document.createElement('nav');
        menu.className = 'sdk-download-menu flash-install-channel-menu';
        menu.setAttribute('aria-label', 'Firmware installation channel');
        const buttons = FLASH_CHANNELS.map((channel) => {
            const button = document.createElement('button');
            button.type = 'button';
            button.className = 'js-flash-start';
            button.dataset.flashChannel = channel.value;
            button.disabled = true;
            const title = document.createElement('strong');
            title.textContent = channel.label;
            const detail = document.createElement('span');
            detail.textContent = 'Loading version…';
            button.append(title, detail);
            button.addEventListener('click', () => { void onSelect(channel.value); });
            menu.appendChild(button);
            return button;
        });
        root.append(toggle, menu);
        mount.replaceChildren(root);
        return { root, toggle, label, buttons };
    }

    function flashRenderChannelVersions() {
        Object.values(flash.installMenus).flatMap((menu) => menu.buttons).forEach((button) => {
            const channel = button.dataset.flashChannel;
            const manifest = flash.manifests[channel];
            const version = manifest?.release_tag || manifest?.version || '';
            const detail = button.querySelector('span');
            if (!detail) return;
            detail.textContent = version
                ? 'Version ' + version
                : flash.channelVersionFailures.has(channel)
                    ? 'Version unavailable'
                    : 'Loading version…';
        });
    }

    function flashChannelAvailable(channel) {
        return Boolean(flash.manifests[channel]) && !flash.channelVersionFailures.has(channel);
    }

    async function flashPopulateChannelVersions() {
        flashRenderChannelVersions();
        if (flash.channelVersionsPromise) return flash.channelVersionsPromise;
        flash.channelVersionsPromise = Promise.all(FLASH_CHANNELS.map(async ({ value: channel }) => {
            try {
                await flashLoadManifest(channel);
                flash.channelVersionFailures.delete(channel);
            } catch (_error) {
                flash.channelVersionFailures.add(channel);
            } finally {
                flashRenderChannelVersions();
                if (flash.initialized) flashSyncInstallActions();
            }
        }));
        return flash.channelVersionsPromise;
    }

    function flashReleaseChannelLabel(manifest) {
        const version = String(manifest?.release_tag || manifest?.version || '').replace(/^v/, '');
        return /(?:^|[-.])rc(?:[.-]?\d+)(?:$|[-.])/i.test(version)
            ? 'Release candidate'
            : 'Stable — recommended';
    }

    function flashFrontendDescription(frontend, supported) {
        if (!supported) return 'Not available for this board.';
        return FLASH_FRONTEND_DESCRIPTIONS[frontend] || 'Alternative ESPectre firmware.';
    }

    function flashApplyFrontendSelection() {
        const frontendKey = document.getElementById('flash-frontend').value;
        const frontend = flash.frontends[frontendKey] || {};
        const frontendArtifacts = frontend.artifacts || [];
        const artifacts = frontendArtifacts
            .filter((artifact) => artifact.build_type === 'factory'
                && artifact.chip_family && artifact.url)
            .sort((a, b) => flashPreferredOrder(FLASH_CHIP_ORDER, a.chip, b.chip));
        flash.availableArtifacts = artifacts;
        flash.supportedChipLabels = artifacts.map((artifact) => artifact.chip_label);
        flash.selectedArtifact = flash.detectedChip
            ? flashSelectArtifact(artifacts, flash.detectedChip)
            : null;
        flash.selectedUpdateArtifact = flash.detectedChip
            ? frontendArtifacts.find((item) => item.build_type === 'ota'
                && flashNormalizeChip(item.chip_family) === flash.detectedChip && item.url) || null
            : null;
        flash.sameFirmware = flashFirmwareMatches(frontendKey, flash.currentInfo);
        flash.requiresErase = !flashFrontendMatches(frontendKey, flash.currentInfo);
        return artifacts;
    }

    async function flashRefresh({ reviewing = false } = {}) {
        const frontendSelect = document.getElementById('flash-frontend');
        const channelSelect = document.getElementById('flash-channel');
        if (!frontendSelect || !channelSelect) return false;
        const requestId = ++flash.refreshRequest;
        const selectedChannel = channelSelect.value;
        flash.targetVersion = '';
        flash.selectedArtifact = null;
        if (!reviewing) flashSetState('loading');
        try {
            const manifest = await flashLoadManifest(selectedChannel);
            if (requestId !== flash.refreshRequest) return false;
            if (selectedChannel === 'release') {
                const option = channelSelect.querySelector('option[value="release"]');
                if (option) option.textContent = flashReleaseChannelLabel(manifest);
            }
            const frontendsMap = flashManifestFrontends(manifest);
            flash.frontends = frontendsMap;
            flash.loadedChannel = selectedChannel;
            flash.channelVersionFailures.delete(selectedChannel);
            flashRenderChannelVersions();
            const frontends = Object.entries(frontendsMap)
                .sort(([a], [b]) => flashPreferredOrder(FLASH_FRONTEND_ORDER, a, b));
            const previous = frontendSelect.value;
            frontendSelect.replaceChildren();
            for (const [key, value] of frontends) {
                const option = document.createElement('option');
                option.value = key;
                option.textContent = value.label || key;
                frontendSelect.appendChild(option);
            }
            if (frontends.some(([key]) => key === previous)) frontendSelect.value = previous;
            flash.targetVersion = manifest.version || '';
            const artifacts = flashApplyFrontendSelection();
            const key = selectedChannel + ':success';
            if (!flash.catalogReports.has(key)) {
                if (track('firmware_catalog', {
                    channel: selectedChannel,
                    result: 'success',
                    frontend_count: frontends.length,
                    artifact_count: frontends.reduce(
                        (total, [, item]) => total + (item.artifacts || []).length, 0
                    )
                })) flash.catalogReports.add(key);
            }
            if (!artifacts.length) {
                flashSetState('error', 'No firmware matches these options.');
            } else if (!browserSupport.flash) {
                flashSetState('error', flashUnsupportedMessage());
            } else if (!reviewing) {
                flashSetState('ready');
            }
            flashSyncControls();
            return true;
        } catch (error) {
            if (requestId !== flash.refreshRequest) return false;
            console.warn('Firmware catalog could not be loaded:', error);
            flash.channelVersionFailures.add(selectedChannel);
            flashRenderChannelVersions();
            flashSetState('error', 'Check your connection, then try again.');
            const key = selectedChannel + ':failure';
            if (!flash.catalogReports.has(key)) {
                if (track('firmware_catalog', {
                    channel: selectedChannel, result: 'failure', error_type: errorType(error)
                })) flash.catalogReports.add(key);
            }
            flashSyncControls();
            return false;
        }
    }

    function flashCreateAnsiRenderer() {
        if (!flash.ansiConstructor) return null;
        const renderer = new flash.ansiConstructor();
        renderer.escape_html = true;
        renderer.url_allowlist = {};
        return renderer;
    }

    async function flashLoadAnsi() {
        if (!flash.ansiDependency) flash.ansiDependency = import(FLASH_ANSI_BUNDLE);
        try {
            const dependency = await flash.ansiDependency;
            if (typeof dependency.AnsiUp !== 'function') {
                throw new TypeError('The ANSI renderer module is invalid.');
            }
            flash.ansiConstructor = dependency.AnsiUp;
            if (!flash.ansiRenderer) flash.ansiRenderer = flashCreateAnsiRenderer();
        } catch (error) {
            flash.ansiDependency = null;
            throw error;
        }
    }

    async function flashLoadHeadless() {
        if (!flash.dependency) flash.dependency = import(FLASH_SERIAL_BUNDLE);
        try {
            const [dependency] = await Promise.all([flash.dependency, flashLoadAnsi()]);
            return dependency;
        } catch (error) {
            flash.dependency = null;
            const failure = new Error(
                'The Web Serial installer is unavailable. Stage the browser dependencies, then reload.'
            );
            failure.name = 'WebSerialBundleError';
            failure.cause = error;
            throw failure;
        }
    }

    async function flashCloseReader() {
        const reader = flash.reader;
        flash.reader = null;
        if (!reader) return;
        flashReportUsbStep('Releasing the USB reader…');
        await reader.cancel().catch(() => {});
        try { reader.releaseLock(); } catch (_error) { /* already released */ }
    }

    async function flashCloseMode({ keepPort = true } = {}) {
        if (flash.closePromise) {
            flashReportUsbStep('Waiting for the USB session to close…');
            await flash.closePromise;
            if (!keepPort && flash.port) {
                flash.port = null;
                flashReleaseUsb();
            }
            return;
        }

        const port = flash.port;
        const closingConsole = flash.mode === 'console';
        if (closingConsole) flash.consoleOpen = false;
        const improv = flash.improv;
        const transport = flash.transport;
        const hadOpenSession = Boolean(improv || transport
            || port?.readable || port?.writable || flash.reader);
        if (hadOpenSession) flashReportUsbStep('Closing the USB session…');
        flash.improv = null;
        flash.transport = null;
        flash.loader = null;
        flash.mode = null;

        const closing = (async () => {
            await flashCloseReader();
            if (improv) await improv.close().catch(() => {});
            if (transport) {
                await transport.disconnect().catch(() => {});
            } else if (port && (port.readable || port.writable)) {
                await port.close().catch(() => {});
            }
            if (hadOpenSession) flash.lastSerialCloseAt = Date.now();
            if (closingConsole) flashSyncConsoleControls(false);
        })();
        flash.closePromise = closing;
        try {
            await closing;
        } finally {
            if (flash.closePromise === closing) flash.closePromise = null;
        }
        if (!keepPort && flash.port === port) {
            flash.port = null;
            flashReleaseUsb();
        }
    }

    async function flashOpenPort(mode) {
        await flashCloseMode({ keepPort: true });
        flashReportUsbStep('Waiting for the USB port to reopen…');
        await flashWaitForSerialReopen();
        const port = flash.port;
        if (!port) throw new DOMException('The USB device is disconnected.', 'NetworkError');
        flashReportUsbStep('Opening the USB port…');
        await port.open({ baudRate: FLASH_SERIAL_BAUD, bufferSize: 8192 });
        flash.mode = mode;
        flashReportUsbStep('Waiting for the USB port to become ready…');
        await flashDelay(FLASH_SERIAL_SETTLE_MS);
    }

    function flashImprovLogger() {
        return {
            log: () => {},
            debug: () => {},
            error: () => {}
        };
    }

    async function flashProbeImprov(api, timeout = FLASH_IMPROV_PROBE_TIMEOUT_MS) {
        await flashOpenPort('probe');
        const improv = new api.ImprovSerial(flash.port, flashImprovLogger());
        flash.improv = improv;
        flashTrackUsbMethod(improv, 'requestCurrentState', 'Waiting for the device to answer Improv…');
        flashTrackUsbMethod(improv, 'requestInfo', 'Reading firmware information through Improv…');
        try {
            flashReportUsbStep('Starting Improv detection…');
            await improv.initialize(timeout);
            flash.currentInfo = improv.info || null;
            flash.nextUrl = improv.nextUrl || '';
            flash.mode = 'improv';
            return improv;
        } catch (error) {
            flash.currentInfo = null;
            flash.nextUrl = '';
            await flashCloseMode({ keepPort: true });
            return null;
        }
    }

    function flashFirmwareMatches(frontend, info, targetVersion = flash.targetVersion) {
        if (!info) return false;
        const actual = String(info.version || '').replace(/^v/, '');
        const target = String(targetVersion || '').replace(/^v/, '');
        return flashFrontendMatches(frontend, info) && actual !== '' && actual === target;
    }

    function flashFrontendMatches(frontend, info) {
        if (!info) return false;
        if (flashIsMicroFirmware(info)) return false;
        const firmware = String(info.firmware || '').toLowerCase();
        const isEspHome = firmware.includes('esphome')
            || firmware === 'francescopace.espectre';
        if (frontend === 'esphome') return isEspHome;
        if (frontend === 'native') {
            return firmware.includes('native')
                || (firmware.includes('espectre')
                    && !isEspHome && !firmware.includes('matter'));
        }
        return firmware.includes(frontend);
    }

    function flashCurrentFrontend(info) {
        return FLASH_FRONTEND_ORDER.find((frontend) => flashFrontendMatches(frontend, info)) || '';
    }

    function flashIsMicroFirmware(info) {
        return ['Micro-ESPectre', 'MicroPython'].includes(info?.firmware);
    }

    function flashInstalledInfo(frontend, version, chipFamily, label, reportedInfo = null) {
        const firmware = FLASH_FIRMWARE_NAMES[frontend]
            || (/^espectre\b/i.test(label || '') ? label : 'ESPectre ' + (label || frontend));
        if (flashCurrentFrontend(reportedInfo) === frontend) {
            return {
                ...reportedInfo,
                firmware,
                version: reportedInfo.version || version,
                chipFamily: reportedInfo.chipFamily || chipFamily
            };
        }
        return { firmware, version, chipFamily };
    }

    function flashRecordInstalledFirmware(frontend, reportedInfo = null) {
        const label = flash.frontends[frontend]?.label || frontend;
        flash.currentInfo = flashInstalledInfo(
            frontend, flash.targetVersion, flash.detectedChip, label, reportedInfo
        );
        flash.detectedFrontend = frontend;
        flash.installChoice = 'update';
        const frontendSelect = document.getElementById('flash-frontend');
        if ([...frontendSelect.options].some((option) => option.value === frontend)) {
            frontendSelect.value = frontend;
        }
        flashSyncInstallChoice();
    }

    function flashFrontendSupportsChip(frontend, chip) {
        const artifacts = (flash.frontends[frontend]?.artifacts || []).filter(
            (artifact) => artifact.build_type === 'factory' && artifact.chip_family && artifact.url
        );
        return Boolean(flashSelectArtifact(artifacts, chip));
    }

    function flashAnyFrontendSupportsChip(chip) {
        return Object.keys(flash.frontends).some(
            (frontend) => flashFrontendSupportsChip(frontend, chip)
        );
    }

    function flashAlternativeFrontend() {
        return [...document.getElementById('flash-frontend').options]
            .map((option) => option.value)
            .find((frontend) => frontend !== flash.detectedFrontend
                && flashFrontendSupportsChip(frontend, flash.detectedChip)) || '';
    }

    function flashRenderFrontendSwitch() {
        const select = document.getElementById('flash-frontend');
        const switcher = $('.js-flash-frontend-switch');
        if (!select || !switcher) return;
        switcher.replaceChildren();
        const alternatives = [...select.options]
            .filter((option) => option.value !== flash.detectedFrontend);
        alternatives.forEach((option) => {
            const button = document.createElement('button');
            const supported = flashFrontendSupportsChip(option.value, flash.detectedChip);
            const selected = flash.installChoice === 'change' && select.value === option.value;
            button.type = 'button';
            button.className = 'flash-frontend-choice';
            button.dataset.frontend = option.value;
            button.setAttribute('aria-pressed', String(selected));
            button.disabled = !supported || flashIsBusy();
            const label = document.createElement('strong');
            label.textContent = option.textContent;
            const detail = document.createElement('span');
            detail.textContent = flashFrontendDescription(option.value, supported);
            button.append(label, detail);
            button.addEventListener('click', () => {
                select.value = option.value;
                flash.installChoice = 'change';
                void flashHandleFirmwareSelection('flash-frontend', 'frontend');
            });
            switcher.appendChild(button);
        });
        if (!alternatives.length) {
            const message = document.createElement('p');
            message.className = 'tool-note';
            message.textContent = 'No other firmware types are available for this board.';
            switcher.appendChild(message);
        }
    }

    function flashSyncInstallChoice({ chooseAlternative = false } = {}) {
        const frontend = document.getElementById('flash-frontend');
        const canUpdate = Boolean(flash.detectedFrontend
            && flashFrontendSupportsChip(flash.detectedFrontend, flash.detectedChip));
        if (flash.installChoice === 'update' && !canUpdate) flash.installChoice = 'change';
        if (flash.installChoice === 'update') {
            frontend.value = flash.detectedFrontend;
        } else if (chooseAlternative && frontend.value === flash.detectedFrontend) {
            const alternative = flashAlternativeFrontend();
            if (alternative) frontend.value = alternative;
        }

        flashApplyFrontendSelection();
        if (canUpdate) {
            const label = flash.frontends[flash.detectedFrontend]?.label || flash.detectedFrontend;
            const title = flash.sameFirmware
                ? 'Reinstall ' + label
                : 'Update ' + label;
            flash.installMenus.current.label.textContent = title;
        } else {
            flash.installMenus.current.label.textContent = 'Firmware unavailable';
        }
        flashRenderFrontendSwitch();
        flashRenderReview();
        if (!flash.selectedArtifact) {
            if (!canUpdate && !flashAlternativeFrontend()) {
                flashSetState('error', flashUnsupportedBoardMessage(flash.detectedChip));
                return false;
            }
        }
        flashSetState('review');
        flashSyncControls();
        void flashPopulateChannelVersions();
        return true;
    }

    function flashNormalizeChip(chip) {
        const description = String(chip || '')
            .trim()
            .toUpperCase()
            .replace(/^CHIP IS\s+/, '')
            .replace(/\s+\(REVISION\s+V?[0-9.]+\)$/, '');
        const compact = description.replace(/[-_]/g, '');
        const reportedFamily = {
            ESP32: 'ESP32', ESP32S2: 'ESP32-S2', ESP32S3: 'ESP32-S3',
            ESP32C3: 'ESP32-C3', ESP32C5: 'ESP32-C5', ESP32C6: 'ESP32-C6'
        }[compact];
        if (reportedFamily) return reportedFamily;
        const family = [
            ['ESP32-C6', /\bESP32-C6(?=\s|\(|$)/],
            ['ESP32-C5', /\bESP32-C5(?=\s|\(|$)/],
            ['ESP32-C3', /\b(?:ESP32-C3|ESP8685|ESP8686)(?=\s|\(|$)/],
            ['ESP32-S3', /\bESP32-S3(?:-PICO-1)?(?=\s|\(|$)/],
            ['ESP32-S2', /\bESP32-S2(?:FH2|FH4|FNR2|R2)?(?=\s|\(|$)/],
        ].find(([, pattern]) => pattern.test(description));
        if (family) return family[0];
        if (description === 'ESP32' || /^ESP32-(?:D[02]WD|S0WD|U4WDH|PICO)/.test(description)
                || description === 'UNKNOWN ESP32') return 'ESP32';
        return description;
    }

    function flashApplyEsptoolSpiRegisterFix(loader) {
        const readFlashId = loader.readFlashId.bind(loader);
        loader.readFlashId = async (...args) => {
            const chip = loader.chip;
            if ((chip?.CHIP_NAME === 'ESP32-C5' || chip?.CHIP_NAME === 'ESP32-C6')
                    && chip.SPI_REG_BASE === 0x60002000) {
                // Fixed upstream after esptool-js 0.6.1; keep the dependency unmodified.
                chip.SPI_REG_BASE = 0x60003000;
            }
            return readFlashId(...args);
        };
    }

    function flashSelectArtifact(artifacts, chip) {
        const normalized = flashNormalizeChip(chip);
        return (artifacts || []).find(
            (artifact) => flashNormalizeChip(artifact.chip_family) === normalized
        ) || null;
    }

    function flashEnglishList(items) {
        if (items.length < 2) return items[0] || '';
        if (items.length === 2) return items.join(' and ');
        return items.slice(0, -1).join(', ') + ', and ' + items.at(-1);
    }

    function flashUnsupportedBoardMessage(chip) {
        const supported = flashEnglishList(flash.supportedChipLabels);
        return (chip ? 'This ' + chip + ' board is not supported.' : 'This board is not supported.')
            + (supported ? ' Published firmware is available for ' + supported + '.' : '');
    }

    function flashTerminal() {
        return {
            clean: () => {},
            write: (data) => flashAppendConsole(data),
            writeLine: (data) => flashAppendConsole(data + '\n')
        };
    }

    async function flashEnterLoader(api) {
        await flashCloseMode({ keepPort: true });
        flashReportUsbStep('Waiting for the USB port before bootloader connection…');
        await flashWaitForSerialReopen();
        const port = flash.port;
        if (!port) {
            const error = new Error('The USB device is disconnected.');
            error.name = 'NetworkError';
            throw error;
        }
        const transport = new api.Transport(port, false);
        flash.transport = transport;
        flash.mode = 'loader';
        transport.setDeviceLostCallback(() => {
            if (FLASH_CRITICAL_STATES.has(flash.state)) {
                void flashFail(new Error('The USB device was disconnected during installation.'));
            }
        });
        const loader = new api.ESPLoader({
            transport, baudrate: FLASH_SERIAL_BAUD, terminal: flashTerminal(),
            serialOptions: { bufferSize: FLASH_SERIAL_BUFFER_SIZE }
        });
        flash.loader = loader;
        flashApplyEsptoolSpiRegisterFix(loader);
        let attempt = 0;
        flashTrackUsbMethod(loader, 'connect', 'Opening the bootloader connection…', 'Reading chip information…');
        flashTrackUsbMethod(loader, '_connectAttempt', () => 'Connecting to bootloader (attempt ' + (++attempt) + ')…');
        flashTrackUsbMethod(loader, 'runStub', 'Starting the flash reader…');
        flashTrackUsbMethod(loader, 'readFlashId', 'Checking the flash connection…');
        flashReportUsbStep('Starting board identification…');
        return flashNormalizeChip(await loader.main('default_reset'));
    }

    async function flashHardResetLoader(loader = flash.loader, transport = flash.transport) {
        if (!loader || !transport) return;
        await transport.setRTS(true);
        await flashDelay(100);
        await loader.after();
    }

    async function flashDetect(reusePort = false) {
        if (!browserSupport.flash) {
            toast(flashUnsupportedMessage());
            return;
        }
        flash.resultReported = false;
        flashSetFlow('flash');
        flash.currentInfo = null;
        flash.detectedFrontend = '';
        flash.installChoice = 'change';
        flash.nextUrl = '';
        $('.js-flash-force-erase').checked = false;
        const operation = ++flash.operation;
        flashNotifyTransition('selecting', 'Loading USB tools…');
        track('firmware_installer_open', flashParams());
        try {
            if (flash.closePromise) await flashWaitForClose(flash.closePromise);
            const api = await flashLoadHeadless();
            if (operation !== flash.operation) return;
            flashReportUsbStep(reusePort
                ? 'Checking the selected USB device…'
                : 'Choose the ESPectre USB device in the browser dialog.');
            const port = reusePort && flash.port ? flash.port : await navigator.serial.requestPort();
            if (operation !== flash.operation) return;
            flashActivateUsb(port, false);
            flashNotifyTransition('detecting', 'USB device selected. Reading its firmware…');
            const improv = await flashProbeImprov(api);
            if (operation !== flash.operation) return;
            let serialInfo = null;
            if (!improv) {
                flashNotifyTransition('detecting', 'Identifying installed firmware…');
                serialInfo = await flashProbeFirmware();
                if (operation !== flash.operation) return;
            }
            const reportedChip = flashNormalizeChip(
                improv?.info?.chipFamily || serialInfo?.chipFamily || ''
            );
            let chip = reportedChip;
            const needsFirmwareDescriptor = (!serialInfo || !serialInfo.version)
                && !flashIsMicroFirmware(serialInfo);
            if (!flashAnyFrontendSupportsChip(reportedChip)
                    || (serialInfo && needsFirmwareDescriptor)) {
                flashNotifyTransition('detecting', 'Identifying the connected board…');
                chip = await flashEnterLoader(api);
            }
            if (!improv && needsFirmwareDescriptor && flash.loader) {
                const storedInfo = await flashReadInstalledFirmwareInfo(flash.loader, chip, (message) => {
                    if (operation === flash.operation) flashReportUsbStep(message);
                });
                if (storedInfo) {
                    flash.currentInfo = storedInfo;
                }
            }
            if (operation !== flash.operation) return;
            flash.detectedChip = chip;
            flash.detectedFrontend = flashCurrentFrontend(flash.currentInfo);
            const frontend = document.getElementById('flash-frontend');
            if (flash.detectedFrontend
                    && [...frontend.options].some((option) => option.value === flash.detectedFrontend)) {
                frontend.value = flash.detectedFrontend;
            }
            flash.installChoice = flash.detectedFrontend
                && flashFrontendSupportsChip(flash.detectedFrontend, chip)
                ? 'update'
                : 'change';
            if (!flashSyncInstallChoice({ chooseAlternative: flash.installChoice === 'change' })) {
                const error = new Error(flashUnsupportedBoardMessage(chip));
                error.name = 'UnsupportedChipError';
                throw error;
            }
            flashActivateUsb(port);
            flashSetStep('review');
            toast('Device connected.');
        } catch (error) {
            if (operation !== flash.operation) return;
            if (error?.name === 'NotFoundError') {
                await flashCleanup(false);
                flashSetStep('select');
                flashSetState('ready');
                flashSyncControls();
                toast('No USB device was selected.');
                return;
            }
            await flashFail(error);
        }
    }

    async function flashContinueToDetection() {
        const reusePort = Boolean(flash.port);
        if (flash.closePromise || flash.mode === 'console') {
            try {
                await flashWaitForClose(flashCloseMode({ keepPort: true }));
            } catch (error) {
                await flashFail(error);
                return;
            }
        }
        if (route !== 'tool-flash') return;
        const consoleDetails = $('.js-flash-console');
        if (consoleDetails) consoleDetails.open = false;
        await flashDetect(reusePort);
    }

    async function flashConfigureWifi(reusePort = false) {
        if (!browserSupport.flash || flash.detectedFrontend === 'matter'
                || flashIsMicroFirmware(flash.currentInfo)) {
            toast('Wi-Fi configuration over USB is available for Native and ESPHome.');
            return;
        }
        flash.resultReported = false;
        flashSetFlow('wifi');
        flash.nextUrl = '';
        const operation = ++flash.operation;
        flashNotifyTransition('selecting', reusePort
            ? 'Preparing Wi-Fi configuration…'
            : 'Choose the ESPectre USB device in the browser dialog.');
        try {
            const api = await flashLoadHeadless();
            if (operation !== flash.operation) return;
            const usingExistingPort = Boolean(reusePort && flash.port);
            const port = usingExistingPort ? flash.port : await navigator.serial.requestPort();
            if (operation !== flash.operation) return;
            flashActivateUsb(port, usingExistingPort);
            let improv = usingExistingPort && flash.mode === 'improv' ? flash.improv : null;
            if (usingExistingPort && flash.mode === 'loader' && flash.loader) {
                flashNotifyTransition('restart', 'Restarting the device for Wi-Fi configuration…');
                await flashHardResetLoader();
                if (operation !== flash.operation) return;
                await flashCloseMode({ keepPort: true });
                if (operation !== flash.operation) return;
                await flashDelay(500);
                if (operation !== flash.operation) return;
            }
            if (!improv) {
                flashNotifyTransition('detecting', 'Connecting to Wi-Fi configuration…');
                improv = await flashProbeImprov(api, FLASH_IMPROV_BOOT_TIMEOUT_MS);
                if (operation !== flash.operation) return;
            }
            if (!improv) {
                const error = new Error(
                    'This device did not respond to Improv Serial. Check that Native or ESPHome is running, then try again.'
                );
                error.name = 'ImprovUnavailableError';
                throw error;
            }
            if (improv.state === api.ImprovSerialCurrentState.STOPPED) {
                const error = new Error('Wi-Fi is disabled on this device and cannot be configured right now.');
                error.name = 'ImprovWifiStoppedError';
                throw error;
            }
            flash.mode = 'improv';
            flashActivateUsb(port);
            $('.js-flash-improv-onboarding').hidden = false;
            $('.js-flash-matter-onboarding').hidden = true;
            flashSetStep('onboarding');
            flashSetState('provisioning');
            toast('Wi-Fi configuration is ready.');
        } catch (error) {
            if (operation !== flash.operation) return;
            if (error?.name === 'NotFoundError') {
                await flashCleanup(false);
                flashSetFlow('flash');
                flashSetStep('select');
                flashSetState('ready');
                flashSyncControls();
                toast('No USB device was selected.');
                return;
            }
            await flashFail(error);
        }
    }

    function flashRenderReview() {
        $('.js-flash-review-chip').textContent = flash.selectedArtifact?.chip_label
            || flash.detectedChip;
        const detectedFrontend = flash.detectedFrontend || flashCurrentFrontend(flash.currentInfo);
        $('.js-flash-review-frontend').textContent = flash.frontends[detectedFrontend]?.label
            || (flash.currentInfo?.firmware
                ? flash.currentInfo.firmware + (flash.currentInfo.inferred ? ' (inferred)' : '') : '')
            || 'Unknown';
        $('.js-flash-review-version').textContent = detectedFrontend || flash.currentInfo?.firmware
            ? flash.currentInfo?.version || (flash.currentInfo?.runtimeVersion
                ? 'MicroPython ' + flash.currentInfo.runtimeVersion + '; application version unavailable.'
                : 'Version unavailable.')
            : 'Firmware not recognized. Choose a firmware type to install.';
        const configureWifi = $('.js-flash-configure-wifi');
        configureWifi.hidden = flash.detectedFrontend === 'matter' || flashIsMicroFirmware(flash.currentInfo);
        flashSyncDeviceSettingsLinks(detectedFrontend);
        const matterCodes = $('.js-flash-show-matter');
        matterCodes.hidden = flash.detectedFrontend !== 'matter';
        $('.js-flash-force-erase').disabled = false;
        flashSyncInstallActions();
    }

    function flashSyncInstallActions() {
        const changing = flash.installChoice === 'change';

        const currentInstallDisabled = flashIsBusy() || !flash.detectedFrontend;
        flashSyncInstallMenu('current', currentInstallDisabled);

        const target = document.getElementById('flash-frontend').value;
        const validChange = changing && target !== flash.detectedFrontend
            && Boolean(flash.selectedArtifact);
        const changeInstallDisabled = flashIsBusy() || !validChange;
        flashSyncInstallMenu('change', changeInstallDisabled, validChange
            ? 'Install ' + (flash.frontends[target]?.label || target)
            : 'Choose a firmware type');
    }

    function flashSyncInstallMenu(kind, disabled, label) {
        const menu = flash.installMenus[kind];
        if (!menu) return;
        menu.buttons.forEach((button) => {
            button.disabled = disabled || !flashChannelAvailable(button.dataset.flashChannel);
        });
        menu.toggle.setAttribute('aria-disabled', String(disabled));
        if (disabled) menu.root.open = false;
        if (label !== undefined) menu.label.textContent = label;
    }

    async function flashFetchBinary(url) {
        const response = await fetch(url, { cache: 'no-store' });
        if (!response.ok) {
            const error = new Error('The firmware image could not be downloaded.');
            error.name = 'FirmwareDownloadError';
            error.status = response.status;
            throw error;
        }
        const data = new Uint8Array(await response.arrayBuffer());
        if (!data.length) {
            const error = new Error('The downloaded firmware image is empty.');
            error.name = 'FirmwareDownloadError';
            throw error;
        }
        return data;
    }

    async function flashDownloadFirmware(erase) {
        flashSetState('download');
        flashSetProgress(5, 'Downloading firmware…');
        const factory = await flashFetchBinary(flash.selectedArtifact.url);
        const update = !erase && flash.selectedUpdateArtifact
            ? await flashFetchBinary(flash.selectedUpdateArtifact.url)
            : null;
        return { factory, update };
    }

    function flashUint32(bytes, offset) {
        return (bytes[offset] | (bytes[offset + 1] << 8)
            | (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24)) >>> 0;
    }

    function flashPartitions(factory, tableOffset = 0x8000) {
        const entries = [];
        for (let offset = tableOffset; offset + 32 <= factory.length; offset += 32) {
            if (factory[offset] !== 0xAA || factory[offset + 1] !== 0x50) break;
            entries.push({
                type: factory[offset + 2],
                subtype: factory[offset + 3],
                address: flashUint32(factory, offset + 4),
                size: flashUint32(factory, offset + 8)
            });
        }
        return entries.sort((a, b) => a.address - b.address);
    }

    function flashCString(bytes, offset, length) {
        const limit = Math.min(bytes.length, offset + length);
        let end = offset;
        while (end < limit && bytes[end] !== 0) end += 1;
        return new TextDecoder().decode(bytes.slice(offset, end)).trim();
    }

    function flashAppDescriptor(bytes) {
        if (bytes.length < 0x70 || bytes[0] !== 0xE9
                || flashUint32(bytes, 0x20) !== FLASH_APP_DESCRIPTOR_MAGIC) return null;
        return {
            version: flashCString(bytes, 0x30, 32),
            projectName: flashCString(bytes, 0x50, 32)
        };
    }

    function flashProjectFrontend(projectName) {
        switch (projectName.toLowerCase()) {
            case 'espectre-native': return 'native';
            case 'espectre-matter': return 'matter';
            case 'espectre': return 'esphome';
            default: return '';
        }
    }

    async function flashReadFirmwareBytes(loader, address, size, phase) {
        const startedAt = Date.now();
        let receivedBytes = 0;
        let stage = 'command_write';
        let stopped = false;
        let writer = null;
        const transport = loader.transport;
        // Isolate metadata-read limits from the loader used for later installation.
        const session = Object.create(loader);
        session.transport = Object.create(transport);
        const ensureActive = () => {
            if (stopped) throw new Error('The USB flash read has been stopped.');
        };
        session.transport.write = async (packet) => {
            ensureActive();
            const acknowledging = stage === 'ack_write';
            if (!transport.device.writable) throw new Error('The USB output stream is closed.');
            const currentWriter = transport.device.writable.getWriter();
            writer = currentWriter;
            try {
                await currentWriter.write(transport.slipWriter(packet));
                ensureActive();
                if (acknowledging) {
                    stage = 'data_read';
                } else {
                    stage = 'command_response';
                }
            } finally {
                currentWriter.releaseLock();
                if (writer === currentWriter) writer = null;
            }
        };
        session.transport.read = async (timeout) => {
            ensureActive();
            const remaining = Math.max(1, FLASH_METADATA_READ_TIMEOUT_MS - (Date.now() - startedAt));
            const packet = await transport.read(Math.min(timeout, remaining));
            ensureActive();
            if (stage === 'data_read') {
                const expectedBlockBytes = Math.min(FLASH_METADATA_BLOCK_SIZE, size - receivedBytes);
                const blockBytes = packet.length;
                receivedBytes += packet.length;
                if (blockBytes !== expectedBlockBytes) {
                    throw new Error('Incomplete or invalid USB flash block: received ' + blockBytes
                        + ' bytes, expected ' + expectedBlockBytes + '.');
                }
                stage = 'ack_write';
            }
            return packet;
        };
        session.checkCommand = async (description, op, data, ...args) => {
            if (op === loader.ESP_READ_FLASH) {
                data = data.slice();
                const request = new DataView(data.buffer, data.byteOffset, data.byteLength);
                // One smaller block in flight; each ACK permits the next block.
                request.setUint32(8, FLASH_METADATA_BLOCK_SIZE, true);
                request.setUint32(12, FLASH_METADATA_BLOCK_SIZE, true);
            }
            const result = await loader.checkCommand.call(session, description, op, data, ...args);
            ensureActive();
            if (result === 0) {
                stage = 'data_read';
            }
            return result;
        };
        try {
            const reading = (async () => {
                const bytes = await session.readFlash(address, size);
                ensureActive();
                // The stub sends a trailing MD5 frame after the final ACK.
                // Consume it so it cannot become the next command's response.
                stage = 'digest_read';
                const digest = await session.transport.read(FLASH_METADATA_READ_TIMEOUT_MS);
                if (digest.length !== 16) throw new Error('Invalid USB flash digest length.');
                return bytes;
            })();
            const bytes = await flashWithTimeout(reading, FLASH_METADATA_READ_TIMEOUT_MS, () =>
                new Error('USB flash identification timed out during ' + phase + ' (' + stage + '). Reconnect the board and try again.'));
            return bytes;
        } catch (error) {
            stopped = true;
            // Stop pending I/O before cleanup; late completions cannot send ACKs.
            const pendingWriter = writer;
            const pendingReader = transport.reader;
            if (pendingWriter) void Promise.resolve().then(() => pendingWriter.abort()).catch(() => {});
            if (pendingReader) void Promise.resolve().then(() => pendingReader.cancel()).catch(() => {});
            const failure = new Error(error?.message || String(error));
            failure.name = 'UsbFlashReadError';
            failure.phase = phase;
            failure.stage = stage;
            throw failure;
        } finally {
            stopped = true;
        }
    }

    async function flashReadInstalledFirmwareInfo(loader, chipFamily, onProgress = () => {}) {
        try {
            onProgress('Reading the flash partition table…');
            const table = await flashReadFirmwareBytes(loader, 0x8000, FLASH_PARTITION_TABLE_LENGTH, 'partition_table');
            const partitions = flashPartitions(table, 0);
            const appPartitions = partitions
                .filter((partition) => partition.type === 0x00);
            let inferredInfo = null;
            for (const [index, partition] of appPartitions.entries()) {
                onProgress('Reading installed firmware (partition ' + (index + 1) + ' of ' + appPartitions.length + ')…');
                const header = await flashReadFirmwareBytes(
                    loader, partition.address, FLASH_APP_DESCRIPTOR_LENGTH,
                    'firmware_partition_' + (index + 1)
                );
                const descriptor = flashAppDescriptor(header);
                const frontend = descriptor && flashProjectFrontend(descriptor.projectName);
                if (frontend) {
                    return {
                        firmware: FLASH_FIRMWARE_NAMES[frontend],
                        // ESPHome's app descriptor stores the framework version.
                        version: frontend === 'esphome' ? '' : descriptor.version,
                        chipFamily: flashNormalizeChip(chipFamily)
                    };
                }
                if (descriptor?.projectName.toLowerCase() === 'micro-espectre') {
                    return {
                        firmware: 'Micro-ESPectre', version: descriptor.version,
                        chipFamily: flashNormalizeChip(chipFamily)
                    };
                }
                if (descriptor?.projectName.toLowerCase() === 'micropython') {
                    inferredInfo = {
                        firmware: 'MicroPython', version: descriptor.version,
                        chipFamily: flashNormalizeChip(chipFamily)
                    };
                } else if (descriptor && !descriptor.projectName && !descriptor.version
                        && flashHasMicroPythonLayout(partitions)) {
                    inferredInfo = {
                        firmware: 'MicroPython', version: '',
                        chipFamily: flashNormalizeChip(chipFamily), inferred: true
                    };
                }
            }
            return inferredInfo;
        } catch (error) {
            if (error?.name === 'UsbFlashReadError') throw error;
            // Invalid descriptors leave the installed firmware unidentified.
        }
        return null;
    }

    function flashHasMicroPythonLayout(partitions) {
        // MicroPython's 4 MiB+ profile leaves the filesystem outside these entries.
        // This layout is evidence of a likely runtime, not proof of an application.
        const layout = [
            [1, 2, 0x9000, 0x6000],
            [1, 1, 0xF000, 0x1000],
            [0, 0, 0x10000, 0x1F0000]
        ];
        return partitions.length === layout.length && partitions.every((entry, index) => {
            const [type, subtype, address, size] = layout[index];
            return entry.type === type && entry.subtype === subtype
                && entry.address === address && entry.size === size;
        });
    }

    function flashEspImageLength(bytes, offset = 0) {
        if (bytes[offset] !== 0xE9 || offset + 24 > bytes.length) {
            throw new Error('The firmware catalog does not contain a valid ESP application image.');
        }
        const segmentCount = bytes[offset + 1];
        let cursor = offset + 24;
        for (let segment = 0; segment < segmentCount; segment += 1) {
            if (cursor + 8 > bytes.length) throw new Error('The ESP application image is truncated.');
            const size = flashUint32(bytes, cursor + 4);
            cursor += 8 + size;
            if (cursor > bytes.length) throw new Error('The ESP application image is truncated.');
        }
        cursor += 1;
        cursor = (cursor + 15) & ~15;
        if (bytes[offset + 23] === 1) cursor += 32;
        if (cursor > bytes.length) throw new Error('The ESP application checksum is truncated.');
        return cursor - offset;
    }

    function flashPreservedParts(payload) {
        const partitions = flashPartitions(payload.factory);
        const appPartition = partitions.find((partition) =>
            partition.type === 0x00 && payload.factory[partition.address] === 0xE9);
        const otaData = partitions.find((partition) =>
            partition.type === 0x01 && partition.subtype === 0x00);
        if (!appPartition || !otaData || payload.factory[0] !== 0xE9) {
            const error = new Error('The image layout cannot be updated safely without an erase.');
            error.name = 'FirmwareLayoutError';
            throw error;
        }
        let app = payload.update;
        if (app) {
            const length = flashEspImageLength(app);
            app = app.slice(0, length);
        } else {
            const length = flashEspImageLength(payload.factory, appPartition.address);
            app = payload.factory.slice(appPartition.address, appPartition.address + length);
        }
        if (app.length > appPartition.size) {
            const error = new Error('The application image does not fit the published partition layout.');
            error.name = 'FirmwareLayoutError';
            throw error;
        }
        const bootloaderLength = flashEspImageLength(payload.factory);
        return [
            { data: payload.factory.slice(0, bootloaderLength), address: 0 },
            {
                data: payload.factory.slice(
                    0x8000,
                    0x8000 + FLASH_PARTITION_TABLE_LENGTH
                ),
                address: 0x8000
            },
            {
                data: payload.factory.slice(otaData.address, otaData.address + otaData.size),
                address: otaData.address
            },
            { data: app, address: appPartition.address }
        ];
    }

    async function flashProgramImage({
        download, validate, erase, eraseFlash, writeFlash, onErase, onWrite
    }) {
        const image = await download();
        if (validate) await validate();
        if (erase) {
            onErase();
            await eraseFlash();
        }
        onWrite();
        await writeFlash(image);
        return image;
    }

    async function flashInstall() {
        if (flash.attempt) return;
        const erase = flash.requiresErase || $('.js-flash-force-erase').checked;
        if (erase && !await flashConfirmEraseDialog()) return;
        if (flash.attempt) return;
        if (!flash.port) {
            const error = new Error(
                'The serial installation session is no longer active. Reconnect the board and try again.'
            );
            error.name = 'SerialSessionError';
            await flashFail(error);
            return;
        }
        let loader = flash.mode === 'loader' ? flash.loader : null;
        const expectedChip = flash.detectedChip;
        const attempt = flashBeginInstallAttempt(erase, loader, () => {
            track('firmware_install_start', flashParams());
        });
        flashShowProgress(true);
        flashSyncControls();
        try {
            if (!loader) {
                flashSetState('restart');
                flashSetProgress(2, 'Opening firmware installation mode…');
                const api = await flashLoadHeadless();
                const detectedChip = await flashEnterLoader(api);
                if (flash.attempt !== attempt) return;
                if (expectedChip && detectedChip !== expectedChip) {
                    const error = new Error(
                        'The connected board changed before installation. Reconnect it and try again.'
                    );
                    error.name = 'SerialDeviceChangedError';
                    throw error;
                }
                loader = flash.loader;
                attempt.loader = loader;
            }
            await flashProgramImage({
                download: () => flashDownloadFirmware(erase),
                validate: () => {
                    if (flash.attempt !== attempt || flash.loader !== loader
                            || flash.mode !== 'loader') {
                        const error = new Error(
                            'The serial installation session ended before writing began. Reconnect the board and try again.'
                        );
                        error.name = 'SerialSessionError';
                        throw error;
                    }
                },
                erase,
                eraseFlash: () => loader.eraseFlash(),
                writeFlash: (payload) => {
                    const fileArray = erase
                        ? [{ data: payload.factory, address: 0 }]
                        : flashPreservedParts(payload);
                    const totals = fileArray.map(() => 0);
                    const allBytes = fileArray.reduce((sum, part) => sum + part.data.length, 0);
                    return loader.writeFlash({
                    fileArray,
                    flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep',
                    eraseAll: false, compress: true,
                    reportProgress: (index, written) => {
                        totals[index] = written;
                        const fraction = allBytes
                            ? totals.reduce((sum, value) => sum + value, 0) / allBytes : 0;
                        flashSetProgress(erase ? 15 + fraction * 80 : 10 + fraction * 85,
                            'Writing firmware… ' + Math.round(fraction * 100) + '%');
                    }
                    });
                },
                onErase: () => {
                    flashSetState('erase');
                    flashSetProgress(12, 'Erasing flash memory…');
                    flashSyncControls();
                },
                onWrite: () => {
                    flashSetState('write');
                    flashSyncControls();
                }
            });
            flashSetState('restart');
            flashSetProgress(98, 'Restarting the board…');
            await flashHardResetLoader(loader, flash.transport);
            await flashCloseMode({ keepPort: true });
            flashSetProgress(100, 'Firmware installed.');
            flashShowProgress(false);
            flashReportResult('success');
            flash.attempt = null;
            const installedFrontend = document.getElementById('flash-frontend').value;
            flashRecordInstalledFirmware(installedFrontend);
            await flashPostInstall(installedFrontend);
        } catch (error) {
            await flashFail(error);
            flash.attempt = null;
        } finally {
            flashSyncControls();
        }
    }

    async function flashPostInstall(frontend) {
        if (frontend === 'matter') {
            const api = await flashLoadHeadless();
            flashSetState('onboarding', 'Waiting for the new firmware…');
            await flashDelay(500);
            const improv = await flashProbeImprov(api, FLASH_IMPROV_BOOT_TIMEOUT_MS);
            if (improv && flashCurrentFrontend(improv.info) === 'matter') {
                flashRecordInstalledFirmware(frontend, improv.info);
                flash.detectedFrontend = 'matter';
            }
            flashSetStep('onboarding');
            $('.js-flash-improv-onboarding').hidden = true;
            $('.js-flash-matter-onboarding').hidden = false;
            flashSetState('onboarding', 'Waiting for Matter setup codes…');
            await flashReadMatterCodes();
            return;
        }
        const api = await flashLoadHeadless();
        flashSetState('onboarding', 'Waiting for the new firmware…');
        await flashDelay(500);
        const improv = await flashProbeImprov(api, FLASH_IMPROV_BOOT_TIMEOUT_MS);
        if (!improv) {
            flashRecordInstalledFirmware(frontend);
            flashComplete('Firmware installed. Configure Wi-Fi later from a supported Improv Serial client.');
            return;
        }
        flashRecordInstalledFirmware(frontend, improv.info);
        flash.nextUrl = improv.nextUrl || '';
        if (improv.state === api.ImprovSerialCurrentState.PROVISIONED || flash.nextUrl) {
            flashComplete('Firmware installed, and the device is already connected to a network.');
            return;
        }
        flash.mode = 'improv';
        flashSetStep('onboarding');
        $('.js-flash-improv-onboarding').hidden = false;
        $('.js-flash-matter-onboarding').hidden = true;
        flashSetState('provisioning', 'Enter a Wi-Fi network for the new device.');
    }

    async function flashProvision(event) {
        event.preventDefault();
        if (!flash.improv) return;
        const operation = flash.operation;
        const ssid = $('.js-flash-wifi-ssid').value;
        const password = $('.js-flash-wifi-password');
        if (!ssid) return;
        $('.js-flash-provision').disabled = true;
        flashSetState('provisioning');
        toast('Sending Wi-Fi settings directly to the device…');
        try {
            await flash.improv.provision(ssid, password.value, FLASH_PROVISION_TIMEOUT_MS);
            if (operation !== flash.operation) return;
            flash.nextUrl = flash.improv.nextUrl || '';
            password.value = '';
            $('.js-flash-wifi-ssid').value = '';
            flashComplete(flash.flow === 'wifi'
                ? 'Wi-Fi settings updated. The device joined the new network.'
                : 'Firmware installed, and the device joined the network.');
        } catch (error) {
            if (operation !== flash.operation) return;
            password.value = '';
            console.warn('Improv Serial provisioning failed:', error);
            toast('The board could not join that network. Check the credentials and try again.');
        } finally {
            $('.js-flash-provision').disabled = false;
        }
    }

    async function flashResetDevice() {
        if (!flash.port) return;
        await flash.port.setSignals({ dataTerminalReady: false, requestToSend: true });
        await flashDelay(100);
        await flash.port.setSignals({ dataTerminalReady: true, requestToSend: false });
    }

    function flashSerialIdentity(input) {
        // eslint-disable-next-line no-control-regex -- strips ANSI terminal escape sequences.
        input = input.replace(/\x1B\[[0-?]*[ -/]*[@-~]/g, '');
        const project = input.match(/Project name:[ \t]*([^\r\n]+)[\r\n]/i)?.[1]?.trim() || '';
        const appVersion = input.match(/App version:[ \t]*([^\r\n]+)[\r\n]/i)?.[1]?.trim() || '';
        const namedMicro = project.toLowerCase() === 'micro-espectre';
        const banner = input.match(/MicroPython(?: \(with v2\.0 preview\))?\s+(\S+) on [^;\r\n]+; ([^\r\n]+)[\r\n]/);
        const microStarted = /Micro-ESPectre starting\.\.\./.test(input);
        const brandedBoard = banner && /\bESPectre ESP32(?:-[CS][2356])?\b/.test(banner[2]);
        if (namedMicro && !appVersion && !microStarted && !banner) return null;
        if (namedMicro || microStarted || banner) {
            const romChip = input.match(/ESP-ROM:(esp32(?:s2|s3|c3|c5|c6)?)/i)?.[1];
            const detected = input.match(/Detected chip:\s*(ESP32|[CS][2356])\s*[\r\n]/)?.[1];
            const chip = romChip || (detected && (detected === 'ESP32' ? detected : 'ESP32-' + detected))
                || banner?.[2].match(/\bESP32(?:-[CS][2356])?\b/)?.[0];
            if (chip) {
                const micro = namedMicro || microStarted || brandedBoard;
                return {
                    firmware: micro ? 'Micro-ESPectre' : 'MicroPython',
                    version: namedMicro ? appVersion : micro ? '' : banner[1],
                    chipFamily: flashNormalizeChip(chip),
                    ...(micro && banner ? { runtimeVersion: banner[1] } : {}),
                    ...(brandedBoard && !microStarted && !namedMicro ? { inferred: true } : {})
                };
            }
        }
        const esphomeProject = input.match(/Project francescopace\.espectre version ([^\r\n]+)[\r\n]/);
        const runtime = input.match(/ESPectre (native|Matter) firmware started/i)?.[1]?.toLowerCase();
        const frontend = esphomeProject ? 'esphome' : flashProjectFrontend(project) || runtime;
        const version = frontend === 'esphome' ? esphomeProject?.[1]?.trim() || '' : appVersion;
        if (!frontend || (!version && !runtime)) return null;
        const romChip = input.match(/ESP-ROM:(esp32(?:s2|s3|c3|c5|c6)?)/i)?.[1] || '';
        return {
            firmware: FLASH_FIRMWARE_NAMES[frontend],
            version,
            chipFamily: flashNormalizeChip(romChip)
        };
    }

    function flashMatterCodes(input) {
        const qr = input.match(/MATTER_QR=(MT:[A-Z0-9.-]+)/);
        const manual = input.match(/MATTER_MANUAL_CODE=([0-9]+)/);
        return qr && manual ? { qr: qr[1], manual: manual[1] } : null;
    }

    async function flashReadSerial(timeoutMs, match) {
        await flashOpenPort('probe');
        flashReportUsbStep('Restarting the device to read startup logs…');
        await flashResetDevice();
        const reader = flash.port.readable.getReader();
        flash.reader = reader;
        const decoder = new TextDecoder();
        const deadline = Date.now() + timeoutMs;
        let input = '';
        try {
            flashReportUsbStep('Waiting for device startup logs…');
            while (Date.now() < deadline) {
                const result = await Promise.race([
                    reader.read(),
                    flashDelay(Math.max(0, deadline - Date.now())).then(() => ({ timedOut: true }))
                ]);
                if (result.timedOut || result.done) break;
                const chunk = decoder.decode(result.value, { stream: true });
                input = (input + chunk).slice(-65536);
                flashAppendConsole(chunk);
                const value = match(input);
                if (value) return value;
            }
        } finally {
            await flashCloseReader();
            await flashCloseMode({ keepPort: true });
        }
        return null;
    }

    async function flashProbeFirmware(timeoutMs = FLASH_FIRMWARE_PROBE_TIMEOUT_MS) {
        const info = await flashReadSerial(timeoutMs, flashSerialIdentity);
        if (info) flash.currentInfo = info;
        return info;
    }

    async function flashReadMatterInput(timeoutMs = 20000) {
        if (!flash.improv) {
            if (flash.loader) {
                await flashHardResetLoader();
                await flashCloseMode({ keepPort: true });
                await flashDelay(500);
            }
            const api = await flashLoadHeadless();
            await flashProbeImprov(api, FLASH_IMPROV_BOOT_TIMEOUT_MS);
        }
        if (flash.improv && flashCurrentFrontend(flash.improv.info) === 'matter'
                && typeof flash.improv.requestMatterOnboarding === 'function') {
            try {
                const codes = await flash.improv.requestMatterOnboarding(timeoutMs);
                if (codes.qr && codes.manual) {
                    return { ...codes, info: flash.improv.info || flash.currentInfo };
                }
            } catch (_error) {
                // Older firmware can still expose onboarding codes in serial logs.
            }
        }
        const result = await flashReadSerial(timeoutMs, (input) => {
            const codes = flashMatterCodes(input);
            return codes ? { ...codes, info: flashSerialIdentity(input) } : null;
        });
        if (result) return result;
        const error = new Error('Matter setup codes were not received. Reset the board, then try again.');
        error.name = 'MatterCodeTimeoutError';
        throw error;
    }

    async function flashReadMatterCodes() {
        const operation = flash.operation;
        const buttons = $$('.js-flash-show-matter');
        buttons.forEach((button) => { button.disabled = true; });
        $('.js-matter-loading').hidden = false;
        $('.js-matter-result').hidden = true;
        try {
            await loadScriptOnce('/vendor/qrcodejs-1.0.0/qrcode.min.js');
            const codes = await flashReadMatterInput();
            if (operation !== flash.operation) return;
            if (codes.info) {
                flashRecordInstalledFirmware('matter', codes.info);
            }
            const canvas = $('.js-matter-canvas');
            canvas.replaceChildren();
            new window.QRCode(canvas, {
                text: codes.qr, width: 220, height: 220,
                colorDark: '#000000', colorLight: '#ffffff',
                correctLevel: window.QRCode.CorrectLevel.M
            });
            $('.js-matter-payload').textContent = codes.qr;
            $('.js-matter-manual').textContent = codes.manual;
            $('.js-matter-loading').hidden = true;
            $('.js-matter-result').hidden = false;
            track('matter_qr_read', { result: 'success' });
            flashComplete('Matter setup codes are ready. Use them to commission the device.', true);
        } catch (error) {
            if (operation !== flash.operation) return;
            console.warn('Matter setup code could not be read:', error);
            toast(error.message || 'The Matter setup code could not be read.');
            track('matter_qr_read', { result: 'failure', error_type: errorType(error) });
        } finally {
            $('.js-matter-loading').hidden = true;
            buttons.forEach((button) => { button.disabled = false; });
        }
    }

    async function flashShowMatterCodes() {
        $('.js-flash-improv-onboarding').hidden = true;
        $('.js-flash-matter-onboarding').hidden = false;
        flashSetStep('onboarding');
        flashSetState('onboarding', 'Waiting for Matter setup codes…');
        await flashReadMatterCodes();
    }

    function flashSettingsUrl(origin, nextUrl, includeTarget) {
        const destination = new URL('/tools/device-settings/', origin);
        if (!includeTarget || !nextUrl) return destination;
        try {
            const target = new URL(nextUrl, origin).searchParams.get('target');
            if (target) destination.searchParams.set('target', target);
        } catch (_error) {
            // Ignore malformed firmware responses instead of forwarding their URL.
        }
        return destination;
    }

    function flashDeviceSettingsUrl(frontend = '') {
        const activeFrontend = frontend || (flash.flow === 'wifi'
            ? flash.detectedFrontend
            : document.getElementById('flash-frontend')?.value);
        return flashSettingsUrl(
            location.origin,
            flash.nextUrl,
            activeFrontend !== 'matter'
        );
    }

    function flashSyncDeviceSettingsLinks(frontend = '') {
        const destination = flashDeviceSettingsUrl(frontend);
        $$('.js-flash-device-settings').forEach((link) => {
            link.href = destination.toString();
            link.hidden = !destination.searchParams.get('target');
        });
    }

    function flashComplete(message, keepOnboarding = false) {
        flashSyncDeviceSettingsLinks();
        if (!keepOnboarding) flashSetStep('review');
        flashSetState('complete', message);
        void flashCloseMode({ keepPort: true });
    }

    async function flashFail(error) {
        const operation = flash.operation;
        console.warn('Web Serial operation failed:', error);
        flashShowProgress(false);
        if (flashShouldReportInstallResult()) {
            flashReportResult(error?.name === 'UnsupportedChipError' ? 'unsupported' : 'failure', error);
        }
        $('.js-flash-error-detail').textContent = error?.message
            || 'The USB operation failed. Reconnect the board and try again.';
        flashSetStep('error');
        flashSetState('error', 'The operation was interrupted.');
        flashSyncControls();
        try {
            await flashWaitForClose(flashCloseMode({ keepPort: true }));
        } catch (closeError) {
            console.warn('USB cleanup failed:', closeError);
            if (operation === flash.operation) {
                $('.js-flash-error-detail').textContent += ' ' + closeError.message;
            }
        }
    }

    function flashRedactConsole(value, redactionState) {
        let output = '';
        for (const line of String(value).split(/(?<=\n)/)) {
            if (/\[\[\s*(?:secret|sensitive)\s*:(?:start|begin)\s*\]\]/i.test(line)) {
                redactionState.active = true;
                output += '\x1B[0m[sensitive output redacted]\x1B[0m\n';
                continue;
            }
            if (/\[\[\s*(?:secret|sensitive)\s*:(?:end|stop)\s*\]\]/i.test(line)) {
                redactionState.active = false;
                continue;
            }
            if (!redactionState.active) output += line;
        }
        return output;
    }

    function flashStripAnsi(value) {
        return String(value).replace(
            // eslint-disable-next-line no-control-regex -- strips ANSI terminal escape sequences.
            /\x1B(?:[@-_][0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1B\\))/g, ''
        );
    }

    function flashSanitizeConsole(value, redactionState) {
        return flashStripAnsi(flashRedactConsole(value, redactionState));
    }

    function flashAnsiFragment(html) {
        const template = document.createElement('template');
        // ansi_up escapes terminal text and emits only its own formatting markup.
        template.innerHTML = html;
        return template.content;
    }

    function flashAppendConsole(value) {
        const redactionState = { active: flash.consoleRedacting };
        const safe = flashRedactConsole(value, redactionState);
        flash.consoleRedacting = redactionState.active;
        if (!safe) return;
        const combinedLength = flash.consoleAnsiText.length + safe.length;
        flash.consoleAnsiText = flashLimitConsole(flash.consoleAnsiText, safe);
        const rebuild = flash.consoleAnsiText.length !== combinedLength;
        const output = $('.js-flash-console-output');
        if (!flash.ansiRenderer) {
            flash.consoleText = flashLimitConsole(flash.consoleText, flashStripAnsi(safe));
            if (output) output.textContent = flash.consoleText;
        } else if (rebuild) {
            flash.ansiRenderer = flashCreateAnsiRenderer();
            const fragment = flashAnsiFragment(
                flash.ansiRenderer.ansi_to_html(flash.consoleAnsiText)
            );
            flash.consoleText = flashLimitConsole('', fragment.textContent);
            if (output) output.replaceChildren(fragment);
        } else {
            const fragment = flashAnsiFragment(flash.ansiRenderer.ansi_to_html(safe));
            flash.consoleText = flashLimitConsole(flash.consoleText, fragment.textContent);
            if (output) output.append(fragment);
        }
        if (output) output.scrollTop = output.scrollHeight;
    }

    function flashLimitConsole(previous, next, limit = FLASH_CONSOLE_LIMIT) {
        return (String(previous || '') + String(next || '')).slice(-limit);
    }

    function flashSyncConsoleControls(connected) {
        const reset = $('.js-flash-console-reset');
        if (reset) reset.disabled = !connected;
    }

    function flashConsoleCanStream() {
        return Boolean(flash.port) && !flash.attempt && !flashIsBusy();
    }

    function flashSessionPanelVisible(step, connected) {
        return Boolean(connected) && step !== 'select';
    }

    function flashSyncUsbActions() {
        const consoleDetails = $('.js-flash-console');
        if (!consoleDetails) return;
        const connected = Boolean(flash.port);
        const visible = flashSessionPanelVisible(flash.step, connected);
        consoleDetails.hidden = !visible;
        if (!visible) consoleDetails.open = false;
    }

    async function flashToggleConsole(open) {
        if (!open) {
            if (flash.mode === 'console') await flashCloseMode({ keepPort: true });
            return;
        }
        if (!flashConsoleCanStream()) {
            flashSyncConsoleControls(false);
            return;
        }
        if (flash.mode === 'console') return;
        flashSyncConsoleControls(false);
        if (!flash.port) {
            try {
                const port = await navigator.serial.requestPort();
                flashActivateUsb(port);
            } catch (error) {
                if (error?.name !== 'NotFoundError') await flashFail(error);
                $('.js-flash-console').open = false;
                return;
            }
        }
        try {
            await flashOpenPort('console');
        } catch (error) {
            await flashFail(error);
            $('.js-flash-console').open = false;
            return;
        }
        flash.consoleOpen = true;
        flashSyncConsoleControls(true);
        const reader = flash.port.readable.getReader();
        flash.reader = reader;
        const decoder = new TextDecoder();
        try {
            while (flash.consoleOpen) {
                const result = await reader.read();
                if (result.done) break;
                flashAppendConsole(decoder.decode(result.value, { stream: true }));
            }
        } catch (error) {
            if (flash.consoleOpen) flashAppendConsole('\n[serial connection closed]\n');
        } finally {
            if (flash.reader === reader) flash.reader = null;
            try { reader.releaseLock(); } catch (_error) { /* already released */ }
            if (flash.mode === 'console' && flash.consoleOpen) {
                await flashCloseMode({ keepPort: true });
            }
        }
    }

    function flashDownloadConsole() {
        const url = URL.createObjectURL(new Blob([flash.consoleText], { type: 'text/plain' }));
        const link = document.createElement('a');
        link.href = url;
        link.download = 'espectre-serial.log';
        link.click();
        setTimeout(() => URL.revokeObjectURL(url), 0);
    }

    function flashSyncConsoleFullscreen() {
        const consoleBody = $('.flash-console-body');
        const button = $('.js-flash-console-fullscreen');
        if (!consoleBody || !button) return;
        const active = document.fullscreenElement === consoleBody;
        button.setAttribute('aria-pressed', String(active));
        button.textContent = active ? 'Exit full screen' : 'Full screen';
    }

    async function flashToggleConsoleFullscreen() {
        const consoleBody = $('.flash-console-body');
        if (!consoleBody?.requestFullscreen || !document.exitFullscreen) return;
        try {
            if (document.fullscreenElement === consoleBody) {
                await document.exitFullscreen();
            } else {
                await consoleBody.requestFullscreen();
            }
        } catch (error) {
            console.warn('Serial console full screen failed:', error);
            toast('The browser could not open the serial console in full screen.');
        }
    }

    async function flashCleanup(reportCancelled = true) {
        flashSyncActivity('ready');
        flash.operation += 1;
        flashShowProgress(false);
        if (reportCancelled && !flash.resultReported && flash.attempt) flashReportResult('cancelled');
        flash.consoleOpen = false;
        await flashCloseMode({ keepPort: false });
        flash.selectedArtifact = null;
        flash.selectedUpdateArtifact = null;
        flash.currentInfo = null;
        flash.detectedFrontend = '';
        flash.installChoice = 'change';
        flash.nextUrl = '';
        flash.attempt = null;
        flash.flow = 'flash';
        const password = $('.js-flash-wifi-password');
        if (password) password.value = '';
    }

    function flashResetUi() {
        flashSyncActivity('ready');
        flashCloseEraseDialog(false);
        flash.flowDialogReturnFocus = null;
        const password = $('.js-flash-wifi-password');
        const ssid = $('.js-flash-wifi-ssid');
        const matterResult = $('.js-matter-result');
        if (password) password.value = '';
        if (ssid) ssid.value = '';
        if (matterResult) matterResult.hidden = true;
        flashSetStep('select');
        flashSetFlow('flash');
    }

    async function flashStartOver() {
        flashResetUi();
        await flashCleanup();
        flash.resultReported = false;
        await flashRefresh();
    }

    function flashIsCritical() {
        return Boolean(flash.attempt) || FLASH_CRITICAL_STATES.has(flash.state);
    }

    function flashIsBusy() {
        return flashIsCritical() || FLASH_TRANSITION_STATES.has(flash.state);
    }

    function flashRouteLeave() {
        if (flashIsCritical()) {
            toast('Wait until the board has finished writing before leaving this page.');
            return false;
        }
        flashResetUi();
        void flashCleanup();
        return true;
    }

    function flashSyncControls() {
        const unsupported = !browserSupport.flash;
        const busy = flashIsBusy();
        const connect = $('.js-flash-connect');
        if (connect) connect.disabled = unsupported || busy || !flash.availableArtifacts?.length;
        const configureWifi = $('.js-flash-configure-wifi');
        if (configureWifi) {
            configureWifi.disabled = unsupported || busy
                || flash.detectedFrontend === 'matter'
                || flashIsMicroFirmware(flash.currentInfo)
                || !flash.currentInfo;
        }
        const matterCodes = $('.js-flash-show-matter');
        if (matterCodes) matterCodes.disabled = unsupported || busy || !flash.port;
        flashSyncUsbActions();
        ['flash-frontend', 'flash-channel'].forEach((id) => {
            const control = document.getElementById(id);
            if (control) control.disabled = busy;
        });
        ['.js-flash-force-erase'].forEach((selector) => {
            const control = $(selector);
            if (control) control.disabled = busy;
        });
        $$('.flash-frontend-choice').forEach((control) => {
            control.disabled = busy
                || !flashFrontendSupportsChip(control.dataset.frontend, flash.detectedChip);
        });
        if (flash.installMenus.current) flashSyncInstallActions();
    }

    async function flashHandleFirmwareSelection(id, selection) {
        track('firmware_selection', {
            selection_type: selection,
            frontend: document.getElementById('flash-frontend').value,
            channel: document.getElementById('flash-channel').value
        });
        const reviewing = Boolean(flash.detectedChip && flash.port);
        const refreshed = await flashRefresh({ reviewing });
        if (!refreshed || !reviewing) return refreshed;
        if (id === 'flash-frontend') {
            flash.installChoice = document.getElementById(id).value === flash.detectedFrontend
                && flashFrontendSupportsChip(flash.detectedFrontend, flash.detectedChip)
                ? 'update'
                : 'change';
        }
        const updateUnavailable = flash.installChoice === 'update'
            && !flashFrontendSupportsChip(flash.detectedFrontend, flash.detectedChip);
        return flashSyncInstallChoice({
            chooseAlternative: flash.installChoice === 'change' || updateUnavailable
        });
    }

    async function flashInstallCurrent(channel) {
        if (flash.attempt || flashIsBusy() || !flashChannelAvailable(channel)) return;
        const channelSelect = document.getElementById('flash-channel');
        flash.installMenus.current.root.open = false;
        if (flash.loadedChannel !== channel) {
            channelSelect.value = channel;
            flashNotifyTransition('detecting', 'Loading the selected firmware channel…');
            const ready = await flashHandleFirmwareSelection('flash-channel', 'channel');
            if (!ready) return;
        }
        flash.installChoice = 'update';
        document.getElementById('flash-frontend').value = flash.detectedFrontend;
        if (!flashSyncInstallChoice() || flash.installChoice !== 'update') return;
        await flashInstall();
    }

    async function flashInstallChange(channel) {
        if (flash.attempt || flashIsBusy() || !flashChannelAvailable(channel)) return;
        const channelSelect = document.getElementById('flash-channel');
        const frontendSelect = document.getElementById('flash-frontend');
        const target = frontendSelect.value;
        flash.installMenus.change.root.open = false;
        if (flash.loadedChannel !== channel) {
            channelSelect.value = channel;
            flashNotifyTransition('detecting', 'Loading the selected firmware channel…');
            const ready = await flashHandleFirmwareSelection('flash-channel', 'channel');
            if (!ready) return;
        }
        if (![...frontendSelect.options].some((option) => option.value === target)) return;
        frontendSelect.value = target;
        flash.installChoice = 'change';
        if (!flashSyncInstallChoice() || !flash.selectedArtifact) return;
        await flashInstall();
    }

    function syncModalOpenState() {
        const openModals = Array.from(document.querySelectorAll('.modal-backdrop'))
            .filter((modal) => !modal.hidden);
        const openModal = openModals.at(-1) || null;
        document.body.classList.toggle('modal-open', Boolean(openModal));

        document.querySelectorAll('[data-modal-inert="true"]').forEach((element) => {
            element.inert = false;
            delete element.dataset.modalInert;
        });
        if (!openModal) return;

        let modalBranch = openModal;
        while (modalBranch && modalBranch !== document.body) {
            const parent = modalBranch.parentElement;
            if (!parent) break;
            Array.from(parent.children).forEach((sibling) => {
                if (!(sibling instanceof HTMLElement)
                        || sibling === modalBranch || sibling.inert) return;
                sibling.inert = true;
                sibling.dataset.modalInert = 'true';
            });
            modalBranch = parent;
        }
    }

    function flashCloseFlowDialog() {
        const modal = $('.js-flash-flow-modal');
        if (!modal || modal.hidden) return;
        const returnFocus = flash.flowDialogReturnFocus;
        flash.flowDialogReturnFocus = null;
        flash.operation += 1;
        $('.js-flash-wifi-password').value = '';
        $('.js-flash-wifi-ssid').value = '';
        const returnStep = flash.detectedChip && flash.port ? 'review' : 'select';
        flashSetStep(returnStep);
        flashSetState('restart');
        flashSyncControls();
        const operation = flash.operation;
        void flashWaitForClose(flashCloseMode({ keepPort: true })).then(() => {
            if (operation !== flash.operation) return;
            flashSetState(returnStep === 'review' ? 'review' : 'ready');
            flashSyncControls();
            if (returnFocus?.isConnected && route === 'tool-flash') returnFocus.focus();
        }).catch((error) => {
            if (operation === flash.operation) void flashFail(error);
        });
    }

    function flashCloseEraseDialog(confirmed = false) {
        const modal = $('.js-flash-erase-modal');
        if (!modal || modal.hidden) return;
        modal.hidden = true;
        syncModalOpenState();
        if (flash.eraseDialogReturnFocus?.isConnected) flash.eraseDialogReturnFocus.focus();
        flash.eraseDialogReturnFocus = null;
        const resolve = flash.eraseDialogResolve;
        flash.eraseDialogResolve = null;
        if (resolve) resolve(confirmed);
    }

    function flashConfirmEraseDialog() {
        if (flash.eraseDialogResolve) flashCloseEraseDialog(false);
        const modal = $('.js-flash-erase-modal');
        $('.js-flash-erase-description').textContent = flash.installChoice === 'change'
            ? 'Changing firmware type requires a full erase.'
            : 'This installation will erase the existing device data.';
        flash.eraseDialogReturnFocus = document.activeElement;
        modal.hidden = false;
        syncModalOpenState();
        $('.js-flash-erase-confirm').focus();
        return new Promise((resolve) => { flash.eraseDialogResolve = resolve; });
    }

    function flashBeforeUnload(event) {
        if (!flashIsCritical()) return;
        event.preventDefault();
        event.returnValue = '';
    }

    function flashInit() {
        if (flash.initialized) return;
        flash.initialized = true;
        flash.installMenus.current = flashCreateChannelMenu(
            '.js-flash-current-install-slot', 'Reinstall firmware', flashInstallCurrent
        );
        flash.installMenus.change = flashCreateChannelMenu(
            '.js-flash-change-install-slot', 'Choose a firmware type', flashInstallChange
        );
        const selectionType = { 'flash-frontend': 'frontend', 'flash-channel': 'channel' };
        Object.entries(selectionType).forEach(([id, selection]) => {
            document.getElementById(id).addEventListener('change', () => {
                void flashHandleFirmwareSelection(id, selection);
            });
        });
        $('.js-flash-connect').addEventListener('click', () => { void flashContinueToDetection(); });
        $('.js-flash-configure-wifi').addEventListener('click', () => {
            void flashConfigureWifi(true);
        });
        $('.js-flash-show-matter').addEventListener('click', () => {
            void flashShowMatterCodes();
        });
        $('.js-flash-force-erase').addEventListener('change', flashSyncInstallActions);
        $$('.js-flash-erase-cancel').forEach((button) => {
            button.addEventListener('click', () => { flashCloseEraseDialog(false); });
        });
        $('.js-flash-erase-confirm').addEventListener('click', () => {
            flashCloseEraseDialog(true);
        });
        $('.js-flash-erase-modal').addEventListener('click', (event) => {
            if (event.target === event.currentTarget) flashCloseEraseDialog(false);
        });
        $$('.js-flash-flow-close').forEach((button) => {
            button.addEventListener('click', flashCloseFlowDialog);
        });
        $('.js-flash-flow-modal').addEventListener('click', (event) => {
            if (event.target === event.currentTarget) flashCloseFlowDialog();
        });
        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape' && !$('.js-flash-erase-modal').hidden) {
                flashCloseEraseDialog(false);
            } else if (event.key === 'Escape' && !$('.js-flash-flow-modal').hidden) {
                flashCloseFlowDialog();
            }
        });
        $('.js-flash-wifi-form').addEventListener('submit', flashProvision);
        $('.js-flash-skip-onboarding').addEventListener('click', () => {
            $('.js-flash-wifi-password').value = '';
            flashComplete(flash.flow === 'wifi'
                ? 'Wi-Fi settings were not changed.'
                : 'Firmware installed. Wi-Fi setup was left for later.');
        });
        $$('.js-flash-new-session').forEach((button) => {
            button.addEventListener('click', () => { void flashStartOver(); });
        });
        $('.js-flash-retry').addEventListener('click', () => {
            if (flash.flow === 'wifi') void flashConfigureWifi(true);
            else void flashDetect(true);
        });
        const consoleDetails = $('.js-flash-console');
        consoleDetails.addEventListener('toggle', () => {
            void flashToggleConsole(consoleDetails.open);
        });
        $('.js-flash-console-reset').addEventListener('click', () => { void flashResetDevice(); });
        $('.js-flash-console-clear').addEventListener('click', () => {
            flash.consoleText = '';
            flash.consoleAnsiText = '';
            flash.ansiRenderer = flashCreateAnsiRenderer();
            $('.js-flash-console-output').replaceChildren();
        });
        $('.js-flash-console-download').addEventListener('click', flashDownloadConsole);
        const fullscreen = $('.js-flash-console-fullscreen');
        fullscreen.hidden = !$('.flash-console-body').requestFullscreen || !document.exitFullscreen;
        fullscreen.addEventListener('click', () => { void flashToggleConsoleFullscreen(); });
        document.addEventListener('fullscreenchange', flashSyncConsoleFullscreen);
        $('.js-firmware-releases').addEventListener('click', () => {
            track('firmware_releases_open', { ...flashParams(), entry_point: 'flash' });
        });
        window.addEventListener('beforeunload', flashBeforeUnload);
        navigator.serial?.addEventListener('disconnect', (event) => {
            if (event.target !== flash.port) return;
            const critical = flashIsCritical();
            flash.operation += 1;
            flash.consoleOpen = false;
            flash.transport?.setDeviceLostCallback(null);
            flash.improv = null;
            flash.transport = null;
            flash.loader = null;
            flash.reader = null;
            flash.mode = null;
            flash.port = null;
            flashReleaseUsb();
            flashSyncConsoleControls(false);
            if (critical) void flashFail(new Error('The USB device was disconnected.'));
            else {
                flashResetUi();
                flashSetState('ready');
                flashSyncControls();
                toast('The USB device was disconnected. Reconnect it to continue.');
            }
        });
        flashSetStep('select');
        flashSyncControls();
    }

    window.flashRouteLeave = flashRouteLeave;
    window.flashCleanup = flashCleanup;
    window.flashDisconnect = flashStartOver;
    window.ESPectreFlashCore = Object.freeze({
        programImage: flashProgramImage,
        beginInstallAttempt: flashBeginInstallAttempt,
        shouldReportInstallResult: flashShouldReportInstallResult,
        sanitizeConsole: flashSanitizeConsole,
        limitConsole: flashLimitConsole,
        settingsUrl: flashSettingsUrl,
        firmwareMatches: flashFirmwareMatches,
        frontendMatches: flashFrontendMatches,
        currentFrontend: flashCurrentFrontend,
        installedInfo: flashInstalledInfo,
        stageVisible: flashStageVisible,
        sessionPanelVisible: flashSessionPanelVisible,
        selectArtifact: flashSelectArtifact,
        applyEsptoolSpiRegisterFix: flashApplyEsptoolSpiRegisterFix,
        hardResetLoader: flashHardResetLoader,
        serialIdentity: flashSerialIdentity,
        matterCodes: flashMatterCodes,
        appDescriptor: flashAppDescriptor,
        readInstalledFirmwareInfo: flashReadInstalledFirmwareInfo,
        espImageLength: flashEspImageLength,
        preservedParts: flashPreservedParts,
        syncModalOpenState,
        consoleLimit: FLASH_CONSOLE_LIMIT
    });
