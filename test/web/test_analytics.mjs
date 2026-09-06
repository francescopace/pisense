/*
 * ESPectre - Website analytics unit tests
 *
 * Copyright 2026 Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const analyticsSource = readFileSync(
    new URL('../../docs/web/assets/js/analytics.js', import.meta.url), 'utf8'
);
const routeRegistrySource = readFileSync(
    new URL('../../docs/web/assets/js/route-registry.js', import.meta.url), 'utf8'
);
const routeManifest = JSON.parse(readFileSync(
    new URL('../../docs/web/routes.json', import.meta.url), 'utf8'
));
const testExports = `
globalThis.__analyticsTest = {
    analyticsAllowedHere, disableAnalytics, enableAnalytics, getRouteTitle, getSiteSection,
    initializeConsentControls, routePath, sanitizeAnalyticsEvent, sendRoutePageView,
    sendStaticPageView, trackEvent, trackRouteView,
    enabled: () => analyticsEnabled
};`;

function analyticsContext({
    hostname = 'espectre.dev', path = '/', hash = '', staticPage = false,
    navigatorValues = {}, storedConsent = null, preloadManifest = true
} = {}) {
    const appendedScripts = [];
    const listeners = new Map();
    let consentValue = storedConsent;
    const consentBanner = { hidden: true };
    const document = {
        cookie: '',
        title: 'Test page | ESPectre',
        documentElement: {
            hasAttribute: (name) => name === 'data-static-page' && staticPage,
            dataset: staticPage ? { siteSection: 'documentation' } : {}
        },
        head: { appendChild: (node) => appendedScripts.push(node) },
        createElement: (tagName) => ({ tagName, addEventListener() {} }),
        querySelector: (selector) => selector === '.js-consent-banner' ? consentBanner : null,
        querySelectorAll: () => [],
        addEventListener: (name, callback) => listeners.set(name, callback),
        dispatchEvent: (event) => listeners.get(event.type)?.(event)
    };
    const location = {
        hostname,
        pathname: path,
        hash,
        origin: `https://${hostname}`
    };
    const window = {
        location,
        localStorage: {
            getItem: () => consentValue,
            setItem: (_key, value) => { consentValue = value; }
        }
    };
    if (preloadManifest) window.ESPectreRouteManifest = routeManifest;
    const context = vm.createContext({
        console, Date, Map, Set, URL, Object, navigator: navigatorValues, document, location,
        fetch: async (url) => ({
            ok: url === '/routes.json',
            status: url === '/routes.json' ? 200 : 404,
            json: async () => routeManifest
        }),
        CustomEvent: class CustomEvent { constructor(type) { this.type = type; } },
        window, globalThis: null
    });
    context.globalThis = context;
    vm.runInContext(routeRegistrySource + analyticsSource + testExports, context);
    return {
        api: context.__analyticsTest,
        sitePolicy: context.window.ESPectreSite,
        window,
        appendedScripts,
        consentBanner,
        listeners
    };
}

describe('analytics privacy boundary', () => {
    it('uses one site policy for production, validation, and loopback hosts', () => {
        const { sitePolicy } = analyticsContext();
        const locationFor = (hostname, protocol = 'https:') => ({ hostname, protocol });

        assert.equal(sitePolicy.analyticsAllowed(locationFor('espectre.dev')), true);
        assert.equal(sitePolicy.analyticsAllowed(locationFor('test.espectre.dev')), true);
        assert.equal(sitePolicy.analyticsAllowed(locationFor('localhost', 'http:')), true);
        assert.equal(sitePolicy.analyticsAllowed(locationFor('example.test')), false);
        assert.equal(sitePolicy.analyticsDebug(locationFor('espectre.dev')), false);
        assert.equal(sitePolicy.analyticsDebug(locationFor('test.espectre.dev')), true);
        assert.equal(sitePolicy.directOriginKind(locationFor('espectre.dev')), 'production');
        assert.equal(sitePolicy.directOriginKind(locationFor('test.espectre.dev')), 'validation');
        assert.equal(sitePolicy.directOriginKind(locationFor('localhost', 'http:')), 'loopback');
        assert.equal(sitePolicy.directOriginKind(locationFor('localhost')), 'other');
    });

    it('does not enable or load GA outside production', () => {
        const { api, appendedScripts, window } = analyticsContext({ hostname: 'example.test' });
        api.enableAnalytics();
        assert.equal(api.enabled(), false);
        assert.equal(appendedScripts.length, 0);
        assert.equal(window.dataLayer, undefined);
    });

    it('enables debug collection on the hosted test site and loopback after consent', () => {
        for (const hostname of ['test.espectre.dev', 'localhost']) {
            const enabled = analyticsContext({ hostname });
            enabled.api.enableAnalytics({ sendPageView: false });
            assert.equal(enabled.api.enabled(), true);
            const config = enabled.window.dataLayer.find((entry) => entry[0] === 'config');
            assert.equal(config[2].debug_mode, true);
        }
    });

    it('shows consent on the hosted test site without loading GA before a choice', () => {
        const { api, appendedScripts, consentBanner, window } = analyticsContext({
            hostname: 'test.espectre.dev'
        });
        api.initializeConsentControls();
        assert.equal(consentBanner.hidden, false);
        assert.equal(appendedScripts.length, 0);
        assert.equal(window.dataLayer, undefined);
    });

    it('uses denied-by-default consent and disables advertising signals', () => {
        const { api, appendedScripts, window } = analyticsContext();
        api.enableAnalytics({ sendPageView: false });

        assert.equal(api.enabled(), true);
        assert.equal(appendedScripts.length, 1);
        assert.match(appendedScripts[0].src, /googletagmanager\.com\/gtag\/js\?id=G-S0NQNG0V11/);
        assert.equal(window.dataLayer[0][0], 'consent');
        assert.equal(window.dataLayer[0][1], 'default');
        assert.equal(window.dataLayer[0][2].analytics_storage, 'denied');
        assert.equal(window.dataLayer[1][1], 'update');
        assert.equal(window.dataLayer[1][2].analytics_storage, 'granted');
        const config = window.dataLayer.find((entry) => entry[0] === 'config');
        assert.equal(config[2].send_page_view, false);
        assert.equal(config[2].allow_google_signals, false);
        assert.equal(config[2].allow_ad_personalization_signals, false);
    });

    it('gates custom events until analytics is enabled', () => {
        const { api, window } = analyticsContext();
        api.trackEvent('firmware_catalog', { result: 'success' });
        assert.equal(window.dataLayer, undefined);
        api.enableAnalytics({ sendPageView: false });
        const before = window.dataLayer.length;
        api.trackEvent('firmware_catalog', { result: 'success' });
        assert.equal(window.dataLayer.length, before + 1);
        assert.equal(window.dataLayer.at(-1)[0], 'event');
        assert.equal(window.dataLayer.at(-1)[1], 'firmware_catalog');
    });

    it('allows the documented tool contract and rejects device-controlled analytics data', () => {
        const { api, window } = analyticsContext();
        api.enableAnalytics({ sendPageView: false });

        assert.equal(api.trackEvent('device_profile', {
            tool_name: 'monitor',
            entry_point: 'monitor',
            frontend: 'native',
            chip: 'ESP32-C3',
            detector: 'lightweight',
            protocol_version: '1',
            firmware_version: '2.8.0-403-g7801312',
            device_id: '0123456789abcdef',
            mqtt_host: 'broker.local'
        }), true);
        const profile = window.dataLayer.at(-1)[2];
        assert.equal(profile.firmware_version, '2.8.0-dev');
        assert.equal(profile.chip, 'ESP32-C3');
        assert.equal(profile.protocol_version, '1');
        assert.equal(Object.hasOwn(profile, 'device_id'), false);
        assert.equal(Object.hasOwn(profile, 'mqtt_host'), false);
        assert.equal(
            api.sanitizeAnalyticsEvent('device_profile', { firmware_version: '0.0.0-main' })
                .firmware_version,
            '0.0.0-main'
        );
        assert.equal(
            api.sanitizeAnalyticsEvent(
                'device_profile', { firmware_version: '3.0.0-rc1-12-gabcdef0' }
            ).firmware_version,
            '3.0.0-dev'
        );

        api.trackEvent('configure_change', {
            action: 'update_mqtt',
            result: 'failure',
            error_type: 'password=user@example.com'
        });
        assert.equal(window.dataLayer.at(-1)[2].error_type, 'unknown');

        api.trackEvent('device_profile', {
            firmware_version: 'device-123-secret',
            frontend: 'native',
            chip: 'esp32-c3'
        });
        assert.equal(window.dataLayer.at(-1)[2].firmware_version, 'unknown');

        const before = window.dataLayer.length;
        assert.equal(api.trackEvent('unregistered_device_event', { payload: 'secret' }), false);
        assert.equal(window.dataLayer.length, before);
    });

    it('does not duplicate a page view when consent is accepted twice', () => {
        const { api, window } = analyticsContext();
        api.enableAnalytics();
        api.enableAnalytics();
        const pageViews = window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        );
        assert.equal(pageViews.length, 1);
    });

    it('leaves the returning visitor page view to the SPA router', () => {
        const spa = analyticsContext({ storedConsent: 'granted' });
        spa.api.initializeConsentControls();
        assert.equal(spa.window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        ).length, 0);

        const staticPage = analyticsContext({ storedConsent: 'granted', staticPage: true });
        staticPage.api.initializeConsentControls();
        assert.equal(staticPage.window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        ).length, 1);
    });

    it('grants consent again and sends one page view after a withdrawal', () => {
        const { api, window } = analyticsContext();
        api.enableAnalytics();
        api.disableAnalytics();
        api.enableAnalytics();
        const consentUpdates = window.dataLayer.filter(
            (entry) => entry[0] === 'consent' && entry[1] === 'update'
        );
        assert.equal(consentUpdates.at(-1)[2].analytics_storage, 'granted');
        const pageViews = window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        );
        assert.equal(pageViews.length, 2);
    });
});

describe('analytics route metadata', () => {
    it('waits for the directly fetched manifest before using route metadata', async () => {
        const { api, listeners, window } = analyticsContext({
            path: '/tools/monitor/', preloadManifest: false, storedConsent: 'granted'
        });
        await listeners.get('DOMContentLoaded')();
        api.sendRoutePageView('tool-monitor');
        const pageViews = window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        );
        assert.equal(pageViews.length, 1);
        assert.equal(pageViews[0][2].page_path, '/tools/monitor/');
        assert.equal(pageViews[0][2].content_group, 'monitor');
    });

    it('uses stable content groups and route paths', () => {
        const { api, window } = analyticsContext({ hash: '#tool-configure' });
        assert.equal(api.getSiteSection('tool-configure'), 'configure');
        assert.equal(api.getSiteSection('tool-monitor'), 'monitor');
        assert.equal(api.getSiteSection('guide-setup'), 'documentation');
        assert.equal(api.getSiteSection('sdk-api'), 'documentation');
        assert.equal(api.getSiteSection('sdk-http-api'), 'documentation');
        assert.equal(api.getSiteSection('sdk-mqtt-api'), 'documentation');
        assert.equal(api.getSiteSection('sdk-detectors'), 'documentation');
        assert.equal(window.ESPectreRoutes.guideNameForPath('/guides/detectors/'), 'detectors');
        assert.equal(window.ESPectreRoutes.guideNameForPath('/guides/future-wifi-sensing/'), 'future-wifi-sensing');
        assert.equal(window.ESPectreRoutes.guideNameForPath('/sdk/api/'), '');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/sdk/api/'), 'api');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/sdk/http-api/'), 'http-api');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/sdk/mqtt-api/'), 'mqtt-api');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/sdk/detectors/'), 'detectors');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/sdk/'), 'overview');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/artifacts/sdk/release/'), 'sdk_release');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/artifacts/sdk/preview/'), 'sdk_preview');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/artifacts/sdk/develop/'), 'sdk_develop');
        assert.equal(window.ESPectreRoutes.documentNameForPath('/guides/detection/'), '');
        assert.equal(api.getSiteSection('privacy'), 'privacy');
        assert.equal(api.routePath('home'), '/');
        assert.equal(api.routePath('tool-flash'), '/tools/flash/');
    });

    it('reports canonical static paths without query parameters', () => {
        const { api, window } = analyticsContext({
            path: '/guides/setup/', staticPage: true
        });
        api.enableAnalytics({ sendPageView: false });
        api.sendStaticPageView();
        const event = window.dataLayer.at(-1);
        assert.equal(event[1], 'page_view');
        assert.equal(event[2].page_path, '/guides/setup/');
        assert.equal(event[2].page_location, 'https://espectre.dev/guides/setup/');
    });

    it('records 404 suggestions only with consent and keeps the requested page location', () => {
        for (const storedConsent of [null, 'denied', 'granted']) {
            const { api, window } = analyticsContext({
                path: '/documentation/setup/', staticPage: true, storedConsent
            });
            api.initializeConsentControls();
            api.trackEvent('select_404_suggestion', { source_url: 'private', destination: 'private' });
            const events = window.dataLayer?.filter((entry) => entry[1] === 'select_404_suggestion') || [];
            assert.equal(events.length, storedConsent === 'granted' ? 1 : 0);
            if (storedConsent === 'granted') {
                assert.deepEqual(Object.keys(events[0][2]), ['content_group']);
                const pageView = window.dataLayer.find((entry) => entry[1] === 'page_view');
                assert.equal(pageView[2].page_location, 'https://espectre.dev/documentation/setup/');
            }
        }
    });

    it('updates the Google tag configuration before a virtual page view', () => {
        const { api, window } = analyticsContext({ hash: '#tool-monitor' });
        api.enableAnalytics({ sendPageView: false });
        api.sendRoutePageView('tool-monitor');
        const update = window.dataLayer.at(-2);
        const pageView = window.dataLayer.at(-1);
        assert.equal(update[0], 'config');
        assert.equal(update[2].update, true);
        assert.equal(update[2].page_location, 'https://espectre.dev/tools/monitor/');
        assert.equal(update[2].page_title, api.getRouteTitle('tool-monitor'));
        assert.equal(update[2].content_group, 'monitor');
        assert.equal(pageView[1], 'page_view');
    });

    it('reports a tool capability after late consent without another page view', () => {
        const { api, window } = analyticsContext({
            hash: '#tool-flash', navigatorValues: { serial: {} }
        });
        api.enableAnalytics();
        const before = window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        ).length;
        api.trackRouteView('tool-flash', { sendPageView: false });
        const capability = window.dataLayer.at(-1);
        assert.equal(capability[1], 'tool_capability');
        assert.equal(capability[2].tool_name, 'flash');
        assert.equal(capability[2].result, 'available');
        const after = window.dataLayer.filter(
            (entry) => entry[0] === 'event' && entry[1] === 'page_view'
        ).length;
        assert.equal(after, before);
    });

    it('reports the CSI visualizer SPA route through its canonical page path', () => {
        const { api, window } = analyticsContext({ path: '/tools/csi-visualizer/' });
        api.enableAnalytics({ sendPageView: false });
        api.sendRoutePageView('tool-raw-csi');
        const pageView = window.dataLayer.at(-1);
        assert.equal(pageView[1], 'page_view');
        assert.equal(pageView[2].page_path, '/tools/csi-visualizer/');
        assert.equal(pageView[2].content_group, 'raw-csi');
    });
});

describe('analytics automatic events', () => {
    it('tracks contact and public tool links added after analytics initialization', async () => {
        const { api, consentBanner, listeners, window } = analyticsContext();
        api.enableAnalytics({ sendPageView: false });
        await listeners.get('DOMContentLoaded')();

        const click = listeners.get('click');
        const dispatchLink = (href, { cookieSettings = false } = {}) => {
            const link = {
                href,
                dataset: {},
                getAttribute: () => href,
                querySelector: () => null,
                textContent: href
            };
            let prevented = false;
            click({
                target: {
                    closest: (selector) => {
                        if (selector === '.js-cookie-settings') return cookieSettings ? link : null;
                        if (selector === 'a[href]') return link;
                        return null;
                    }
                },
                preventDefault: () => { prevented = true; }
            });
            return prevented;
        };

        dispatchLink('mailto:contact@espectre.dev?subject=Product');
        assert.equal(window.dataLayer.at(-1)[1], 'click_contact');
        dispatchLink('mailto:security@espectre.dev');
        assert.equal(window.dataLayer.at(-1)[1], 'click_security');
        dispatchLink('https://espectre.dev/tools/device-settings/');
        assert.equal(window.dataLayer.at(-1)[1], 'select_tool');
        assert.equal(window.dataLayer.at(-1)[2].tool_name, 'configure');
        dispatchLink('https://espectre.dev/#guides');
        assert.equal(window.dataLayer.at(-1)[1], 'select_guide');
        assert.equal(window.dataLayer.at(-1)[2].guide_name, 'overview');
        dispatchLink('https://espectre.dev/#sdk-api');
        assert.equal(window.dataLayer.at(-1)[1], 'select_documentation');
        assert.equal(window.dataLayer.at(-1)[2].document_name, 'api');
        consentBanner.hidden = true;
        assert.equal(dispatchLink('/privacy/#cookie-settings', { cookieSettings: true }), true);
        assert.equal(consentBanner.hidden, false);
    });
});
