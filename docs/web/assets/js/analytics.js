/*
 * ESPectre - Privacy-conscious analytics
 *
 * Google Analytics is enabled only on production and allowlisted debug hosts
 * after explicit consent. The SPA router owns manual page views; generated
 * static pages report their canonical path directly.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

const GA_MEASUREMENT_ID = 'G-S0NQNG0V11';
const ANALYTICS_CONSENT_KEY = 'espectre.analytics.consent.v1';
const SITE_POLICY = window.ESPectreSite;
if (!SITE_POLICY) throw new Error('ESPectre site policy is unavailable');
const IS_STATIC_PAGE = document.documentElement.hasAttribute('data-static-page');
const STATIC_PAGE_SECTION = document.documentElement.dataset.siteSection || 'documentation';

const CAPABILITY_BY_ROUTE = {
    'tool-flash': ['web_serial', 'serial']
};

const ANALYTICS_EVENT_PARAMETERS = Object.freeze({
    click_contact: [],
    click_security: [],
    configure_change: ['action', 'result', 'error_type'],
    device_profile: [
        'tool_name', 'entry_point', 'frontend', 'chip', 'detector', 'protocol_version',
        'firmware_version'
    ],
    firmware_catalog: ['channel', 'result', 'frontend_count', 'artifact_count', 'error_type'],
    firmware_install_result: ['frontend', 'channel', 'chip', 'result'],
    firmware_install_start: ['frontend', 'channel', 'chip'],
    firmware_installer_open: ['frontend', 'channel', 'chip'],
    firmware_releases_open: ['frontend', 'channel', 'entry_point'],
    firmware_selection: ['selection_type', 'frontend', 'channel'],
    game_abandon: ['input_mode', 'score', 'distance', 'reason'],
    game_over: ['input_mode', 'score', 'orbs', 'distance'],
    game_start: ['input_mode'],
    local_discovery: ['tool_name', 'result', 'device_count', 'truncated', 'error_type'],
    matter_qr_read: ['result', 'error_type'],
    ota_update_attempt: ['channel', 'transport', 'input_mode', 'result', 'error_type'],
    ota_update_result: ['result', 'ota_state', 'duration_ms', 'channel', 'error_type'],
    csi_stream: [
        'tool_name', 'entry_point', 'transport', 'input_mode', 'result', 'latency_ms',
        'duration_seconds', 'error_type', 'reason'
    ],
    sdk_download: ['channel', 'format', 'link_text'],
    select_404_suggestion: [],
    select_documentation: ['document_name', 'link_text'],
    select_guide: ['guide_name', 'link_text'],
    select_tool: ['tool_name', 'link_text'],
    sensing_change: [
        'tool_name', 'entry_point', 'transport', 'input_mode', 'action', 'result', 'error_type'
    ],
    theremin_configuration: ['control', 'setting_value'],
    tool_capability: ['tool_name', 'capability', 'result'],
    tool_connection: ['tool_name', 'entry_point', 'transport', 'result', 'error_type'],
    tool_demo_start: ['tool_name', 'entry_point'],
    tool_disconnect: [
        'tool_name', 'entry_point', 'transport', 'input_mode', 'reason', 'duration_seconds'
    ],
    tool_ready: [
        'tool_name', 'entry_point', 'transport', 'input_mode', 'readiness', 'latency_ms'
    ]
});

const ANALYTICS_CATEGORY_VALUES = Object.freeze({
    action: new Set([
        'clear_mqtt', 'clear_wifi', 'clear_wifi_bssid', 'recalibrate', 'set_csi_traffic_mode',
        'update_device', 'update_sensing', 'update_mqtt',
        'set_traffic_generator_mode', 'set_wifi_bssid'
    ]),
    capability: new Set(['web_serial']),
    channel: new Set(['develop', 'preview', 'release']),
    chip: new Set([
        'c3', 'c5', 'c6', 'esp32', 'esp32-c3', 'esp32-c5', 'esp32-c6', 'esp32-s2',
        'esp32-s3', 'esp32c3', 'esp32c5', 'esp32c6', 'esp32s2', 'esp32s3', 's2', 's3'
    ]),
    control: new Set(['playback', 'waveform']),
    detector: new Set(['high_accuracy', 'lightweight', 'unknown']),
    entry_point: new Set(['configure', 'flash', 'game', 'monitor', 'raw-csi', 'theremin']),
    format: new Set(['tar.gz', 'zip']),
    frontend: new Set(['custom', 'esphome', 'matter', 'micro', 'native']),
    input_mode: new Set(['demo', 'direct']),
    ota_state: new Set([
        'applying', 'checking', 'downloading', 'error', 'idle', 'reboot_scheduled',
        'reconnected', 'starting', 'unknown', 'up_to_date', 'update_available'
    ]),
    protocol_version: new Set(['1', '1.0', 'unknown']),
    readiness: new Set(['csi', 'info', 'motion']),
    reason: new Set([
        'disconnect', 'local_network_denied', 'page_exit', 'reconnect_failed', 'replaced',
        'restart', 'route_change', 'stream_error', 'user', 'wifi_cleared'
    ]),
    result: new Set([
        'accepted', 'attempt', 'available', 'cancelled', 'empty', 'failure', 'multiple',
        'stopped', 'success', 'unavailable', 'unconfirmed', 'unsupported',
        'validation_failure'
    ]),
    selection_type: new Set(['channel', 'frontend']),
    setting_value: new Set(['sawtooth', 'sine', 'square', 'start', 'stop', 'triangle']),
    tool_name: new Set(['configure', 'flash', 'game', 'monitor', 'raw-csi', 'theremin'])
});

const ANALYTICS_ERROR_TYPES = new Set([
    'AbortError', 'ClientDisconnected', 'DeviceReportedError', 'Error', 'InvalidStateError',
    'NetworkError', 'NotAllowedError', 'NotConnected', 'NotFoundError', 'NotSupportedError',
    'QrRendererMissing', 'RangeError', 'ReferenceError', 'SecurityError', 'StatusTimeout',
    'Superseded', 'SyntaxError', 'TimeoutError', 'TypeError', 'VerificationMismatch',
    'WifiRecoveryRequired', 'WifiRolledBack', 'busy', 'busy_raw_collection', 'client_error',
    'conflict', 'connection_failed', 'forbidden', 'frame_too_large', 'handshake_required',
    'internal_error', 'invalid_capabilities', 'invalid_endpoint', 'invalid_envelope',
    'invalid_json', 'invalid_method', 'invalid_params', 'invalid_path', 'invalid_peer_result',
    'invalid_raw_frame', 'invalid_raw_record', 'invalid_raw_session', 'invalid_request_id',
    'invalid_scheme', 'local_network_denied', 'non_local_endpoint', 'not_connected',
    'not_raw_session_owner', 'timeout', 'unavailable', 'unknown', 'unsupported',
    'unsupported_capability', 'unsupported_crypto', 'unsupported_version'
]);

const ANALYTICS_NUMBER_LIMITS = Object.freeze({
    artifact_count: [0, 1000],
    device_count: [0, 8],
    distance: [0, 1000000000],
    duration_ms: [0, 86400000],
    duration_seconds: [0, 604800],
    frontend_count: [0, 16],
    latency_ms: [0, 86400000],
    orbs: [0, 1000000000],
    score: [0, 1000000000]
});

function normalizeFirmwareVersion(value) {
    if (typeof value !== 'string' || value.length > 48) return 'unknown';
    const match = value.match(
        /^v?([0-9]{1,2})\.([0-9]{1,3})\.([0-9]{1,3})(?:-((?:alpha|beta|rc)[0-9]*(?:-[0-9]{1,6}-g[0-9a-f]{7,40})?|main|develop|[0-9]{1,6}-g[0-9a-f]{7,40}))?$/i
    );
    if (!match) return 'unknown';
    const [, major, minor, patch, suffix = ''] = match;
    if (Number(minor) > 999 || Number(patch) > 999) return 'unknown';
    const core = `${Number(major)}.${Number(minor)}.${Number(patch)}`;
    if (/^(?:(?:alpha|beta|rc)[0-9]*-)?[0-9]{1,6}-g[0-9a-f]{7,40}$/i.test(suffix)) {
        return `${core}-dev`;
    }
    const prerelease = suffix.match(/^(alpha|beta|rc)([0-9]*)$/i);
    if (prerelease && (!prerelease[2] || Number(prerelease[2]) <= 99)) {
        return `${core}-${prerelease[1].toLowerCase()}${prerelease[2]}`;
    }
    if (suffix === 'main' || suffix === 'develop') return `${core}-${suffix}`;
    return suffix ? 'unknown' : core;
}

function normalizeAnalyticsParameter(name, value) {
    const categories = ANALYTICS_CATEGORY_VALUES[name];
    if (categories) {
        if (typeof value !== 'string') return 'unknown';
        const comparison = name === 'chip' ? value.toLowerCase() : value;
        return categories.has(comparison) ? value : 'unknown';
    }
    if (name === 'error_type') {
        if (typeof value !== 'string') return 'unknown';
        return ANALYTICS_ERROR_TYPES.has(value) || /^http_[1-5][0-9]{2}$/.test(value)
            ? value : 'unknown';
    }
    if (name === 'firmware_version') {
        return normalizeFirmwareVersion(value);
    }
    if (name === 'guide_name' || name === 'document_name') {
        return typeof value === 'string' && /^[a-z0-9][a-z0-9_-]{0,63}$/.test(value)
            ? value : 'unknown';
    }
    if (name === 'link_text') {
        if (typeof value !== 'string') return undefined;
        const normalized = value.trim().replace(/\s+/g, ' ').substring(0, 100);
        return normalized || undefined;
    }
    if (name === 'truncated') return typeof value === 'boolean' ? value : undefined;
    const limits = ANALYTICS_NUMBER_LIMITS[name];
    if (limits) {
        return typeof value === 'number' && Number.isFinite(value)
            && value >= limits[0] && value <= limits[1] ? value : undefined;
    }
    return undefined;
}

function sanitizeAnalyticsEvent(eventName, params = {}) {
    const allowed = ANALYTICS_EVENT_PARAMETERS[eventName];
    if (!allowed) return null;
    const source = params && typeof params === 'object' && !Array.isArray(params) ? params : {};
    const sanitized = {};
    for (const name of allowed) {
        if (!Object.hasOwn(source, name)) continue;
        const value = normalizeAnalyticsParameter(name, source[name]);
        if (value !== undefined) sanitized[name] = value;
    }
    return sanitized;
}

const reportedCapabilities = new Set();
let analyticsEnabled = false;
let analyticsConfigured = false;

function currentRoute() {
    const registeredPath = window.ESPectreRoutes?.routeForPath(window.location.pathname);
    if (registeredPath) return registeredPath;
    const legacyHash = (window.location.hash || '').slice(1);
    return window.ESPectreRoutes?.has(legacyHash) ? legacyHash : 'home';
}

function getRouteTitle(route = currentRoute()) {
    const registeredTitle = window.ESPectreRoutes?.title(route);
    if (registeredTitle) return registeredTitle;
    const conventionalPrefix = ['guide-', 'sdk-'].find((prefix) => route.startsWith(prefix));
    if (conventionalPrefix) {
        const slug = route.slice(conventionalPrefix.length);
        const label = slug.replace(/[-_]+/g, ' ').trim();
        if (label) return `${label.charAt(0).toUpperCase()}${label.slice(1)} | ESPectre`;
    }
    return 'ESPectre — Wi-Fi motion sensing';
}

function getSiteSection(route = currentRoute()) {
    if (IS_STATIC_PAGE) return STATIC_PAGE_SECTION;
    return window.ESPectreRoutes?.contentGroup(route) || 'other';
}

function routePath(route) {
    return window.ESPectreRoutes?.get(route)?.staticPath
        || '/';
}

function analyticsAllowedHere() {
    return SITE_POLICY.analyticsAllowed(window.location);
}

function analyticsDebugEnabled() {
    return SITE_POLICY.analyticsDebug(window.location);
}

function storedConsent() {
    try {
        return window.localStorage.getItem(ANALYTICS_CONSENT_KEY);
    } catch (error) {
        return null;
    }
}

function saveConsent(value) {
    try {
        window.localStorage.setItem(ANALYTICS_CONSENT_KEY, value);
    } catch (error) {
        // Consent remains valid for the current page when storage is blocked.
    }
}

function ensureGtagQueue() {
    window.dataLayer = window.dataLayer || [];
    window.gtag = window.gtag || function () { window.dataLayer.push(arguments); };
}

function loadGoogleTag() {
    if (document.querySelector(`script[src*="googletagmanager.com/gtag/js?id=${GA_MEASUREMENT_ID}"]`)) return;
    const script = document.createElement('script');
    script.async = true;
    script.src = `https://www.googletagmanager.com/gtag/js?id=${GA_MEASUREMENT_ID}`;
    script.referrerPolicy = 'strict-origin-when-cross-origin';
    document.head.appendChild(script);
}

function updatePageConfig(pageLocation, pageTitle, contentGroup) {
    window.gtag('config', GA_MEASUREMENT_ID, {
        update: true,
        page_location: pageLocation,
        page_title: pageTitle,
        content_group: contentGroup
    });
}

function sendStaticPageView() {
    const pageLocation = window.location.origin + window.location.pathname;
    updatePageConfig(pageLocation, document.title, STATIC_PAGE_SECTION);
    window.gtag('event', 'page_view', {
        page_location: pageLocation,
        page_path: window.location.pathname,
        page_title: document.title,
        content_group: STATIC_PAGE_SECTION
    });
}

function sendRoutePageView(route = currentRoute()) {
    const path = routePath(route);
    const pageLocation = window.location.origin + path;
    const pageTitle = getRouteTitle(route);
    const contentGroup = getSiteSection(route);
    updatePageConfig(pageLocation, pageTitle, contentGroup);
    window.gtag('event', 'page_view', {
        page_location: pageLocation,
        page_path: path,
        page_title: pageTitle,
        content_group: contentGroup
    });
}

function enableAnalytics({ sendPageView = true } = {}) {
    if (!analyticsAllowedHere()) return;
    const wasEnabled = analyticsEnabled;
    ensureGtagQueue();

    if (!analyticsConfigured) {
        window.gtag('consent', 'default', {
            analytics_storage: 'denied',
            ad_storage: 'denied',
            ad_user_data: 'denied',
            ad_personalization: 'denied',
            wait_for_update: 500
        });
    }

    window.gtag('consent', 'update', {
        analytics_storage: 'granted',
        ad_storage: 'denied',
        ad_user_data: 'denied',
        ad_personalization: 'denied'
    });

    if (!analyticsConfigured) {
        window.gtag('js', new Date());
        const config = {
            send_page_view: false,
            allow_google_signals: false,
            allow_ad_personalization_signals: false
        };
        if (analyticsDebugEnabled()) config.debug_mode = true;
        window.gtag('config', GA_MEASUREMENT_ID, config);
        loadGoogleTag();
        analyticsConfigured = true;
    }

    analyticsEnabled = true;
    if (sendPageView && !wasEnabled) {
        if (IS_STATIC_PAGE) sendStaticPageView();
        else sendRoutePageView();
    }
    if (!wasEnabled) document.dispatchEvent(new CustomEvent('espectre:analytics-enabled'));
}

function clearAnalyticsCookies() {
    document.cookie.split(';').forEach((entry) => {
        const name = entry.split('=')[0].trim();
        if (!name.startsWith('_ga')) return;
        document.cookie = `${name}=; Max-Age=0; Path=/; SameSite=Lax`;
        document.cookie = `${name}=; Max-Age=0; Path=/; Domain=.espectre.dev; SameSite=Lax`;
    });
}

function disableAnalytics() {
    analyticsEnabled = false;
    if (typeof window.gtag === 'function') {
        window.gtag('consent', 'update', {
            analytics_storage: 'denied',
            ad_storage: 'denied',
            ad_user_data: 'denied',
            ad_personalization: 'denied'
        });
    }
    clearAnalyticsCookies();
}

function trackEvent(eventName, params = {}) {
    if (!analyticsEnabled) return false;
    const sanitized = sanitizeAnalyticsEvent(eventName, params);
    if (!sanitized) {
        console.warn(`Analytics event is not registered: ${String(eventName)}`);
        return false;
    }
    window.gtag('event', eventName, {
        content_group: getSiteSection(),
        ...sanitized
    });
    return true;
}

function trackRouteView(route = currentRoute(), { sendPageView = true } = {}) {
    if (!analyticsEnabled) return;
    if (sendPageView) sendRoutePageView(route);

    const capability = CAPABILITY_BY_ROUTE[route];
    if (!capability || reportedCapabilities.has(route)) return;
    reportedCapabilities.add(route);
    trackEvent('tool_capability', {
        tool_name: window.ESPectreRoutes?.get(route)?.analyticsName || route,
        capability: capability[0],
        result: capability[1] in navigator ? 'available' : 'unavailable'
    });
}

function linkText(link) {
    const heading = link.querySelector('h1, h2, h3, .doc-link-title');
    return (heading || link).textContent.trim().replace(/\s+/g, ' ').substring(0, 100);
}

function showConsentBanner() {
    const banner = document.querySelector('.js-consent-banner');
    if (banner) banner.hidden = false;
}

function hideConsentBanner() {
    const banner = document.querySelector('.js-consent-banner');
    if (banner) banner.hidden = true;
}

function initializeConsentControls() {
    document.querySelectorAll('.js-consent-accept').forEach((button) => {
        button.addEventListener('click', () => {
            saveConsent('granted');
            hideConsentBanner();
            enableAnalytics();
        });
    });
    document.querySelectorAll('.js-consent-reject').forEach((button) => {
        button.addEventListener('click', () => {
            saveConsent('denied');
            hideConsentBanner();
            disableAnalytics();
        });
    });
    if (!analyticsAllowedHere()) {
        hideConsentBanner();
        return;
    }

    const consent = storedConsent();
    if (consent === 'granted') enableAnalytics({ sendPageView: IS_STATIC_PAGE });
    else if (consent !== 'denied') showConsentBanner();
}

function initializeAutoTracking() {
    document.addEventListener('click', (event) => {
        if (event.target.closest('.js-cookie-settings')) {
            event.preventDefault();
            showConsentBanner();
            return;
        }
        const link = event.target.closest('a[href]');
        if (!link) return;

        const href = link.getAttribute('href') || '';
        if (href.toLowerCase().startsWith('mailto:contact@')) {
            trackEvent('click_contact');
            return;
        }
        if (href.toLowerCase().startsWith('mailto:security@')) {
            trackEvent('click_security');
            return;
        }

        let url;
        try {
            url = new URL(link.href, window.location.origin);
        } catch (error) {
            return;
        }

        const guideName = url.origin === window.location.origin
            ? window.ESPectreRoutes?.guideNameForPath(url.pathname)
            : '';
        if (guideName) {
            trackEvent('select_guide', {
                guide_name: guideName,
                link_text: linkText(link)
            });
            return;
        }

        const documentName = url.origin === window.location.origin
            ? window.ESPectreRoutes?.documentNameForPath(url.pathname)
            : '';
        if (documentName) {
            trackEvent('select_documentation', {
                document_name: documentName,
                link_text: linkText(link)
            });
            return;
        }

        if (link.dataset.sdkChannel && link.dataset.sdkFormat) {
            trackEvent('sdk_download', {
                channel: link.dataset.sdkChannel,
                format: link.dataset.sdkFormat,
                link_text: linkText(link)
            });
            return;
        }

        const hashRoute = url.hash.replace(/^#/, '');
        const legacyRoute = url.pathname === '/' && window.ESPectreRoutes?.has(hashRoute)
            ? hashRoute
            : '';
        const route = url.origin === window.location.origin
            ? (legacyRoute || window.ESPectreRoutes?.routeForPath(url.pathname))
            : '';
        if (window.ESPectreRoutes?.groupOf(route) === 'tools') {
            const toolName = window.ESPectreRoutes.get(route)?.analyticsName || route;
            trackEvent('select_tool', { tool_name: toolName, link_text: linkText(link) });
        } else if (route === 'guides') {
            const routePath = window.ESPectreRoutes.get(route)?.staticPath || '';
            const guideName = window.ESPectreRoutes.guideNameForPath(routePath);
            if (guideName) {
                trackEvent('select_guide', { guide_name: guideName, link_text: linkText(link) });
            }
        } else if (route === 'sdk' || route.startsWith('sdk-')) {
            const routePath = window.ESPectreRoutes.get(route)?.staticPath || '';
            const documentName = window.ESPectreRoutes.documentNameForPath(routePath);
            if (documentName) {
                trackEvent('select_documentation', {
                    document_name: documentName,
                    link_text: linkText(link)
                });
            }
        }
    });
}

window.trackEvent = trackEvent;
window.trackRouteView = trackRouteView;
window.getSiteSection = getSiteSection;
window.getRouteTitle = getRouteTitle;
document.addEventListener('DOMContentLoaded', async () => {
    try {
        await window.ESPectreRoutesReady;
    } catch (error) {
        console.error('Analytics disabled because route metadata is unavailable:', error);
        return;
    }
    initializeConsentControls();
    initializeAutoTracking();
});
