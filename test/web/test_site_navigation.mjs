/*
 * ESPectre - Website navigation contracts
 *
 * Copyright 2026 Francesco Pace <francescopace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';
import { runInNewContext } from 'node:vm';
import { index, read, routeBootstrap, routeManifest, routeRegistry, styles } from './fixtures/site_test_helpers.mjs';

const errorSuggestions = JSON.parse(read('docs/web/404-suggestions.json'));

async function renderErrorSuggestion(path, { map = errorSuggestions, ok = true, error = null } = {}) {
    const section = { hidden: true };
    const events = [];
    const listeners = new Map();
    const link = { addEventListener: (name, callback) => listeners.set(name, callback) };
    const location = Object.freeze({
        origin: 'https://espectre.dev', pathname: path, search: '?private=value', hash: '#old-heading'
    });
    await runInNewContext(read('docs/web/assets/js/404-suggestions.js'), {
        URL, Object,
        document: {
            querySelector: (selector) => selector === '.js-error-suggestion' ? section : link
        },
        window: { location, trackEvent: (...args) => events.push(args) },
        fetch: async (url, options) => {
            assert.equal(url, '/404-suggestions.json');
            assert.equal(options.cache, 'no-cache');
            if (error) throw error;
            return { ok, json: async () => map };
        }
    });
    return { section, link, events, click: () => listeners.get('click')?.() };
}

describe('website navigation contracts', () => {
    it('maps retired paths to existing public routes or project documents', () => {
        for (const [path, suggestion] of Object.entries(errorSuggestions)) {
            assert.ok(path.startsWith('/') && path.endsWith('/'));
            const destination = new URL(suggestion.href, 'https://espectre.dev');
            if (destination.origin === 'https://espectre.dev') {
                assert.ok(routeManifest.routes.some((route) => route.staticPath === destination.pathname));
            } else {
                assert.equal(destination.origin, 'https://github.com');
                const prefix = '/francescopace/espectre/blob/develop/';
                assert.ok(destination.pathname.startsWith(prefix));
                assert.ok(existsSync(new URL(`../../${destination.pathname.slice(prefix.length)}`, import.meta.url)));
            }
        }
    });

    it('offers the mapped destination on the 404 without forwarding query parameters or fragments', async () => {
        for (const [path, suggestion] of Object.entries(errorSuggestions)) {
            for (const variant of [path, path.slice(0, -1), `${path}index.html`]) {
                const result = await renderErrorSuggestion(variant);
                assert.equal(result.section.hidden, false);
                assert.equal(result.link.href, new URL(suggestion.href, 'https://espectre.dev').href);
                assert.deepEqual(result.events, []);
                result.click();
                assert.deepEqual(result.events, [['select_404_suggestion']]);
            }
        }
    });

    it('keeps generic navigation for unknown paths, unavailable maps, and unsafe destinations', async () => {
        const scenarios = [
            ['/unknown/'],
            ['/documentation/setup/extra/'],
            ['/documentation/setup/', { ok: false }],
            ['/documentation/setup/', { error: new Error('Offline') }],
            ['/documentation/setup/', { map: null }],
            ['/documentation/setup/', { error: new SyntaxError('Invalid JSON') }],
            ...['javascript:alert(1)', '//example.com/', 'https://github.com/another/project/'].map(
                (href) => ['/documentation/setup/', { map: { '/documentation/setup/': { href, title: 'Link' } } }]
            )
        ];
        for (const [path, options] of scenarios) {
            const result = await renderErrorSuggestion(path, options);
            assert.equal(result.section.hidden, true);
            assert.equal(result.link.href, undefined);
            assert.deepEqual(result.events, []);
        }
    });

    it('selects the newest available firmware for the home badge and preserves the Flash link', async () => {
        const source = read('docs/web/assets/js/app.js');
        const start = source.indexOf('    let releaseBadgeChecked = false;');
        const end = source.indexOf('    /*', start);
        const scenarios = [
            ['3.1.0', '3.0.0-12-gabcdef0', 'release'],
            ['3.1.0', '3.1.0-1-gabcdef0', 'preview'],
            ['3.1.0', '3.1.0', 'release'],
            ['3.1.0', '3.1.0-0-gabcdef0', 'release'],
            ['3.1.0', '3.1.0-rc2-5-gabcdef0', 'release'],
            ['3.1.0-rc2', '3.1.0-rc2-5-gabcdef0', 'preview'],
            ['3.1.0-rc10', '3.1.0-rc2-5-gabcdef0', 'release'],
            ['3.9.0', '3.10.0-1-gabcdef0', 'preview'],
            ['3.1.0', null, 'release'],
            [null, '3.1.0-1-gabcdef0', 'preview'],
            ['snapshot', '3.1.0-1-gabcdef0', 'preview'],
            [new Error('Network unavailable'), '3.1.0-1-gabcdef0', 'preview'],
            ['3.1.0', new SyntaxError('Invalid JSON'), 'release'],
            [null, null, null],
            ['snapshot', '', null],
        ];
        for (const [release, preview, expectedChannel] of scenarios) {
            const versions = { release, preview };
            const badge = { hidden: true, dataset: {} };
            const label = { textContent: '' };
            const requests = [];
            const context = {
                $: (selector) => selector === '.js-release-badge' ? badge : label,
                console: { warn() {} },
                fetch: async (url) => {
                    requests.push(url);
                    const channel = url.split('/')[3];
                    assert.equal(url, `/artifacts/firmware/${channel}/firmware-manifest-${channel}.json`);
                    const version = versions[channel];
                    if (version instanceof Error && !(version instanceof SyntaxError)) throw version;
                    return {
                        ok: version !== null,
                        status: version === null ? 404 : 200,
                        json: async () => {
                            if (version instanceof SyntaxError) throw version;
                            return { version, release_tag: channel === 'preview' ? 'snapshot' : version };
                        }
                    };
                }
            };
            await runInNewContext(source.slice(start, end) + '\nupdateReleaseBadge();', context);
            assert.equal(badge.hidden, expectedChannel === null);
            if (expectedChannel) {
                assert.equal(badge.dataset.firmwareChannel, expectedChannel);
                assert.equal(badge.dataset.firmwareVersion, versions[expectedChannel]);
            }
            await runInNewContext('updateReleaseBadge();', context);
            assert.equal(requests.length, 2);
        }
        const badgeTag = index.match(/<a\b[^>]*class="[^"]*\bjs-release-badge\b[^"]*"[^>]*>/)?.[0];
        assert.match(badgeTag, /href="\/tools\/flash\/"/);
    });

    it('keeps one route registry aligned with the SPA pages and static paths', () => {
        const registeredRoutes = routeManifest.routes.map((route) => route.name).sort();
        const pageRoutes = [...index.matchAll(/<main\b[^>]*\bdata-page="([^"]+)"/g)]
            .map((match) => match[1])
            .sort();
        assert.deepEqual(registeredRoutes, pageRoutes);

        for (const route of routeManifest.routes.filter(({ group }) => group === 'tools')) {
            assert.match(index, new RegExp(`data-page="${route.name}"[\\s\\S]*?<div class="js-static-content">`));
            const content = read(`docs/web/content${route.staticPath.slice(0, -1)}.html`);
            assert.match(content, /class="tool-static-entry"/);
            assert.match(content, /class="tool-interactive"/);
        }
    });

    it('keeps static page browser titles aligned with the route registry', () => {
        const window = { ESPectreRouteManifest: routeManifest };
        runInNewContext(routeRegistry, { Map, Object, Set, URL, window });
        assert.equal(index.match(/<title>([^<]*)<\/title>/)?.[1], window.ESPectreRoutes.title('home'));
        assert.equal(
            index.match(/property="og:title" content="([^"]*)"/)?.[1],
            window.ESPectreRoutes.title('home')
        );
        assert.equal(
            index.match(/name="twitter:title" content="([^"]*)"/)?.[1],
            window.ESPectreRoutes.title('home')
        );
    });

    it('uses canonical paths for static pages and SPA navigation', () => {
        const mainNavigationPaths = routeManifest.navigation.main
            .map((name) => routeManifest.routes.find((route) => route.name === name)?.staticPath);
        const relativePaths = (html) => [...html.matchAll(/<a href="(\/(?:[^"]*\/)?)"/g)]
            .map((match) => match[1]);
        for (const source of [index, read('docs/web/404.html')]) {
            const mainNavigation = source.match(/<nav class="main-nav"[^>]*>([\s\S]*?)<\/nav>/)?.[1] || '';
            const exploreLinks = source.match(/<div class="home-resource-links">([\s\S]*?)<\/div>/)?.[1] || '';
            assert.deepEqual(relativePaths(mainNavigation), mainNavigationPaths);
            assert.deepEqual(relativePaths(exploreLinks), mainNavigationPaths.slice(1));
        }
    });

    it('keeps the Home hero out of the first paint while a deep SPA route boots', () => {
        const attributes = new Map();
        const document = {
            documentElement: {
                dataset: {},
                setAttribute: (name, value) => attributes.set(name, value),
                removeAttribute: (name) => attributes.delete(name),
            },
        };
        const window = {
            location: { hash: '#guide-setup' },
        };
        window.self = window;
        window.top = window;
        runInNewContext(routeBootstrap, { document, URL, window });
        assert.equal(attributes.has('data-spa-booting'), true);
        assert.match(styles, /html\[data-spa-booting\] \.js-page\[data-page="home"\] \{ display: none; \}/);
        assert.match(index, /<script src="\/assets\/js\/route-bootstrap\.js\?v=[a-f0-9]{12}"><\/script>/);
        assert.ok(
            index.indexOf('route-bootstrap.js') < index.indexOf('espectre-direct.js'),
            'the route bootstrap must run before deferred application scripts'
        );
    });

    it('keeps the interactive portal inert when embedded by another origin', () => {
        const attributes = new Map([['data-frame-guard', '']]);
        const document = {
            documentElement: {
                dataset: {},
                setAttribute: (name, value) => attributes.set(name, value),
                removeAttribute: (name) => attributes.delete(name),
            },
        };
        const window = { location: { hash: '#tool-configure' } };
        window.self = window;
        window.top = {};
        runInNewContext(routeBootstrap, { document, URL, window });
        assert.equal(attributes.has('data-frame-guard'), true);
        assert.equal(attributes.has('data-spa-booting'), false);
        assert.match(index, /<html[^>]+data-frame-guard/);
        assert.match(index, /<style>html\[data-frame-guard\] \{ visibility: hidden; \}<\/style>/);
        assert.match(index, /<noscript><style>html\[data-frame-guard\] \{ visibility: visible; \}<\/style><\/noscript>/);
        assert.match(styles, /html\[data-frame-guard\] \{ visibility: hidden; \}/);
    });

    it('resolves canonical page anchors before entering the SPA', () => {
        const window = { ESPectreRouteManifest: routeManifest };
        runInNewContext(routeRegistry, { Map, Object, Set, URL, window });
        const routes = window.ESPectreRoutes;
        assert.equal(routes.siteOrigin, routeManifest.siteOrigin);
        const target = routes.staticTargetForHref(
            '/guides/setup/#setup-native-discovery',
            'https://test.espectre.dev/'
        );

        assert.equal(target.route, 'guide-setup');
        assert.equal(target.anchor, 'setup-native-discovery');
        assert.equal(
            routes.staticTargetForHref('https://example.com/guides/setup/', 'https://test.espectre.dev/'),
            null
        );
        assert.equal(
            routes.staticTargetForHref('/guides/setup/?source=external', 'https://test.espectre.dev/'),
            null
        );
    });

    it('normalizes explicit HTML entries and static-page clicks to root SPA hashes', () => {
        const navigation = read('docs/web/assets/js/navigation.js');
        const listeners = new Map();
        const replacements = [];
        const assignments = [];
        const location = {
            pathname: '/index.html',
            href: 'https://espectre.dev/index.html#contact',
            search: '',
            hash: '#contact',
            assign: (href) => assignments.push(href),
        };
        const history = {
            state: { source: 'test' },
            replaceState: (state, title, href) => replacements.push(String(href)),
        };
        const document = {
            documentElement: { hasAttribute: (name) => name === 'data-static-page' },
            addEventListener: (type, listener) => listeners.set(type, listener),
            querySelectorAll: () => [],
        };
        const window = {
            location,
            history,
            matchMedia: () => ({ matches: false, addEventListener: () => {} }),
            ESPectreRoutes: {
                staticTargetForHref: (href) => {
                    if (href === '/contact/') return { route: 'contact', anchor: '' };
                    if (href === '/roadmap/#roadmap-research-title') {
                        return { route: 'roadmap', anchor: 'roadmap-research-title' };
                    }
                    return null;
                }
            },
        };
        runInNewContext(navigation, { document, URL, URLSearchParams, window });

        assert.deepEqual(replacements, ['https://espectre.dev/#contact']);
        let prevented = false;
        listeners.get('click')({
            defaultPrevented: false,
            button: 0,
            metaKey: false,
            ctrlKey: false,
            shiftKey: false,
            altKey: false,
            target: {
                closest: () => ({
                    target: '',
                    getAttribute: () => '/contact/',
                }),
            },
            preventDefault: () => { prevented = true; },
        });
        assert.equal(prevented, true);
        listeners.get('click')({
            defaultPrevented: false,
            button: 0,
            metaKey: false,
            ctrlKey: false,
            shiftKey: false,
            altKey: false,
            target: {
                closest: () => ({
                    target: '',
                    getAttribute: () => '/roadmap/#roadmap-research-title',
                }),
            },
            preventDefault: () => {},
        });
        location.pathname = '/tools/device-settings/';
        location.href = 'https://espectre.dev/tools/device-settings/?target=192.168.1.42';
        location.search = '?target=192.168.1.42';
        listeners.get('click')({
            defaultPrevented: false,
            button: 0,
            metaKey: false,
            ctrlKey: false,
            shiftKey: false,
            altKey: false,
            target: {
                closest: () => ({
                    target: '',
                    getAttribute: () => '/#tool-configure',
                }),
            },
            preventDefault: () => {},
        });
        assert.deepEqual(assignments, [
            '/#contact',
            '/?anchor=roadmap-research-title#roadmap',
            '/?target=192.168.1.42#tool-configure'
        ]);
    });

    it('returns a refreshed SPA route to the app shell without redirecting direct visits', () => {
        function runBootstrap({ navigationType, rememberedRoute }) {
            const replacements = [];
            const location = {
                pathname: '/tools/monitor/',
                href: 'https://espectre.dev/tools/monitor/?target=192.168.1.42#diagnostics',
                search: '?target=192.168.1.42',
                hash: '#diagnostics',
                replace: (href) => replacements.push(href),
            };
            const document = {
                documentElement: {
                    dataset: { spaRoute: 'tool-monitor' },
                    setAttribute: () => {},
                },
            };
            const window = {
                location,
                history: {
                    state: rememberedRoute ? { espectreRoute: rememberedRoute } : null,
                    replaceState: () => {},
                },
                performance: { getEntriesByType: () => [{ type: navigationType }] },
            };
            runInNewContext(routeBootstrap, { document, URL, window });
            return replacements;
        }

        const reload = runBootstrap({
            navigationType: 'reload',
            rememberedRoute: 'tool-monitor',
        });
        assert.deepEqual(reload, [
            '/?target=192.168.1.42&anchor=diagnostics#tool-monitor'
        ]);

        const directVisit = runBootstrap({
            navigationType: 'navigate',
            rememberedRoute: 'tool-monitor',
        });
        assert.deepEqual(directVisit, []);

        const standaloneReload = runBootstrap({
            navigationType: 'reload',
            rememberedRoute: '',
        });
        assert.deepEqual(standaloneReload, []);
    });

});
