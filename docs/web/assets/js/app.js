/*
 * ESPectre - Website app shell
 *
 * Part of the website application shell.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

'use strict';


    let routeRegistry = null;
    const sitePolicy = window.ESPectreSite;
    if (!sitePolicy) throw new Error('ESPectre site policy is unavailable');
    const browserSupport = window.ESPectreBrowserSupport && window.ESPectreBrowserSupport.current;
    if (!browserSupport) throw new Error('ESPectre browser capability policy is unavailable');
    const DirectProtocolClient = window.ESPectreDirectClient;
    if (!DirectProtocolClient) throw new Error('ESPectre Direct HTTP client is unavailable');

    const $ = (sel) => document.querySelector(sel);
    const $$ = (sel) => Array.from(document.querySelectorAll(sel));

    // analytics.js is optional: the app must work with it blocked or absent.
    const track = (name, params) => window.trackEvent ? window.trackEvent(name, params) : false;
    const errorType = (error) => (error && (error.code || error.name)) || 'Error';
    const toolNameForRoute = (routeName) => routeRegistry.groupOf(routeName) === 'tools'
        ? (routeRegistry.get(routeName)?.analyticsName || routeName)
        : 'monitor';
    const activeToolName = () => toolNameForRoute(route);
    const LEGACY_TOOL_ROUTES = Object.freeze({ device: 'tool-configure' });
    const MQTT_PRESETS = Object.freeze({
        home_assistant: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtt', host: 'homeassistant.local', port: '1883',
                hostPlaceholder: 'homeassistant.local', locked: Object.freeze(['scheme'])
            })
        }),
        lan_broker: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtt', host: '', port: '1883',
                hostPlaceholder: 'broker.local or 192.168.1.20', locked: Object.freeze(['scheme'])
            })
        }),
        emqx_cloud: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtts', host: 'deployment-id.ala.region.emqxsl.com', port: '8883',
                hostPlaceholder: 'deployment-id.ala.region.emqxsl.com',
                locked: Object.freeze(['scheme', 'port'])
            })
        }),
        hivemq_cloud: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtts', host: 'cluster-id.s1.region.hivemq.cloud', port: '8883',
                hostPlaceholder: 'cluster-id.s1.region.hivemq.cloud',
                locked: Object.freeze(['scheme', 'port'])
            })
        }),
        flespi: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtts', host: 'mqtt.flespi.io', port: '8883', hostPlaceholder: 'mqtt.flespi.io',
                locked: Object.freeze(['scheme', 'host', 'port'])
            })
        }),
        cloud_broker: Object.freeze({
            configure: Object.freeze({
                scheme: 'mqtts', host: 'cluster.example.com', port: '', hostPlaceholder: 'cluster.example.com'
            })
        })
    });
    const MQTT_FORM_DEFAULTS = {
        topicPrefix: 'espectre/v1/devices'
    };

    const dependencyPromises = new Map();

    function loadScriptOnce(src, { module = false } = {}) {
        if (dependencyPromises.has(src)) return dependencyPromises.get(src);
        const promise = new Promise((resolve, reject) => {
            const existing = document.querySelector(`script[src="${src}"]`);
            if (existing && existing.dataset.loaded === 'true') {
                resolve();
                return;
            }
            const script = existing || document.createElement('script');
            if (module) script.type = 'module';
            script.src = src;
            script.addEventListener('load', () => {
                script.dataset.loaded = 'true';
                resolve();
            }, { once: true });
            script.addEventListener('error', () => {
                script.remove();
                reject(new Error(`Unable to load ${src}`));
            }, { once: true });
            if (!existing) document.head.appendChild(script);
        });
        dependencyPromises.set(src, promise);
        promise.catch(() => dependencyPromises.delete(src));
        return promise;
    }

    let releaseBadgeChecked = false;

    function parseFirmwareVersion(version) {
        if (typeof version !== 'string') return null;
        const normalized = version.replace(/^v/, '');
        const snapshot = normalized.match(/-(\d+)-g[0-9a-f]+$/i);
        const tag = snapshot ? normalized.slice(0, snapshot.index) : normalized;
        const match = tag.match(/^(\d+)\.(\d+)\.(\d+)(?:-([0-9A-Za-z.-]+))?$/);
        if (!match) return null;
        return {
            version: normalized,
            core: match.slice(1, 4).map(Number),
            prerelease: match[4] || '',
            commits: snapshot ? Number(snapshot[1]) : 0
        };
    }

    function compareFirmwareVersions(left, right) {
        for (let index = 0; index < left.core.length; index++) {
            if (left.core[index] !== right.core[index]) return left.core[index] - right.core[index];
        }
        if (left.prerelease !== right.prerelease) {
            if (!left.prerelease) return 1;
            if (!right.prerelease) return -1;
            return left.prerelease.localeCompare(right.prerelease, 'en', { numeric: true });
        }
        return left.commits - right.commits;
    }

    async function updateReleaseBadge() {
        if (releaseBadgeChecked) return;
        releaseBadgeChecked = true;
        const candidates = await Promise.all(['release', 'preview'].map(async (channel) => {
            try {
                const response = await fetch(
                    `/artifacts/firmware/${channel}/firmware-manifest-${channel}.json`,
                    { cache: 'no-store' }
                );
                if (!response.ok) {
                    const error = new Error(`HTTP ${response.status}`);
                    error.status = response.status;
                    throw error;
                }
                const manifest = await response.json();
                const parsed = parseFirmwareVersion(manifest.version);
                return parsed ? { ...parsed, channel } : null;
            } catch (error) {
                if (error && error.status !== 404) console.warn('Release badge unavailable:', error);
                return null;
            }
        }));
        const latest = candidates.filter(Boolean).reduce((selected, candidate) => (
            !selected || compareFirmwareVersions(candidate, selected) > 0 ? candidate : selected
        ), null);
        if (!latest) return;
        const badge = $('.js-release-badge');
        badge.dataset.firmwareChannel = latest.channel;
        badge.dataset.firmwareVersion = latest.version;
        $('.js-release-text').textContent = `v${latest.version} available`;
        badge.hidden = false;
    }

    /* ============================================================= routing */

    function focusRouteContent(routeName = route) {
        const page = $(`[data-page="${routeName}"]`);
        if (!page) return;
        const target = page.querySelector('h1') || page;
        if (!target.hasAttribute('tabindex')) target.setAttribute('tabindex', '-1');
        target.focus({ preventScroll: true });
    }

    function focusRouteAnchor(routeName, encodedTargetId) {
        let targetId;
        try {
            targetId = decodeURIComponent(encodedTargetId);
        } catch (error) {
            return false;
        }
        const page = $(`[data-page="${routeName}"]`);
        const target = page && document.getElementById(targetId);
        if (!target || !page.contains(target)) return false;
        target.scrollIntoView();
        if (!target.hasAttribute('tabindex')) target.setAttribute('tabindex', '-1');
        target.focus({ preventScroll: true });
        return true;
    }

    let pendingRouteAnchor = '';

    function consumeRouteAnchorHandoff() {
        const url = new URL(location.href);
        const anchor = url.searchParams.get('anchor') || '';
        if (!anchor) return;
        pendingRouteAnchor = anchor;
        url.searchParams.delete('anchor');
        history.replaceState(null, '', url.pathname + url.search + url.hash);
    }

    function normalizedRouteName(candidate) {
        const remapped = LEGACY_TOOL_ROUTES[candidate] || candidate;
        return routeRegistry.has(remapped) ? remapped : 'home';
    }

    function routeFromLocation() {
        const hashRoute = (location.hash || '').slice(1);
        if (routeRegistry.has(LEGACY_TOOL_ROUTES[hashRoute] || hashRoute)) {
            return normalizedRouteName(hashRoute);
        }
        return routeRegistry.routeForPath(location.pathname) || 'home';
    }

    function routeHistoryUrl(routeName, { anchor = '', preserveSearch = false } = {}) {
        const definition = routeRegistry.get(routeName);
        const url = new URL(location.href);
        url.pathname = definition?.staticPath || '/';
        if (!preserveSearch) url.search = '';
        url.hash = anchor ? `#${anchor}` : '';
        return url.pathname + url.search + url.hash;
    }

    function syncRouteMetadata(routeName) {
        const definition = routeRegistry.get(routeName);
        const title = window.getRouteTitle
            ? window.getRouteTitle(routeName)
            : 'ESPectre — Wi-Fi motion sensing';
        const description = definition?.description || '';
        const canonicalLink = document.querySelector('link[rel="canonical"]');
        const canonicalOrigin = canonicalLink
            ? new URL(canonicalLink.href).origin
            : routeRegistry.siteOrigin;
        const canonical = new URL(definition?.staticPath || '/', canonicalOrigin).href;
        document.title = title;
        if (canonicalLink) canonicalLink.href = canonical;
        const ogUrl = document.querySelector('meta[property="og:url"]');
        const ogTitle = document.querySelector('meta[property="og:title"]');
        const ogDescription = document.querySelector('meta[property="og:description"]');
        const twitterTitle = document.querySelector('meta[name="twitter:title"]');
        const twitterDescription = document.querySelector('meta[name="twitter:description"]');
        const metaDescription = document.querySelector('meta[name="description"]');
        if (ogUrl) ogUrl.content = canonical;
        if (ogTitle) ogTitle.content = title;
        if (ogDescription) ogDescription.content = description;
        if (twitterTitle) twitterTitle.content = title;
        if (twitterDescription) twitterDescription.content = description;
        if (metaDescription) metaDescription.content = description;
    }

    function replaceLegacyRouteLocation(routeName) {
        const hashRoute = (location.hash || '').slice(1);
        if (!routeRegistry.has(LEGACY_TOOL_ROUTES[hashRoute] || hashRoute)) return;
        history.replaceState(
            { espectreRoute: routeName },
            '',
            routeHistoryUrl(routeName, {
                anchor: pendingRouteAnchor,
                preserveSearch: true
            })
        );
    }

    function navigateToRoute(next, { anchor = '', focus = true, replace = false } = {}) {
        const target = normalizedRouteName(next);
        const sameRoute = target === route;
        pendingRouteAnchor = anchor;
        history[replace ? 'replaceState' : 'pushState'](
            { espectreRoute: target },
            '',
            routeHistoryUrl(target, { anchor })
        );
        if (!sameRoute) {
            setRoute(target, { focus });
            return;
        }
        if (!anchor) return;
        pendingRouteAnchor = '';
        void loadStaticContent(target).then((ready) => {
            if (ready && !focusRouteAnchor(target, anchor)) focusRouteContent(target);
        });
    }

    window.navigateToRoute = navigateToRoute;

    function onPopState() {
        const target = routeFromLocation();
        const hash = (location.hash || '').slice(1);
        const anchor = routeRegistry.has(LEGACY_TOOL_ROUTES[hash] || hash) ? '' : hash;
        if (target === route) {
            if (anchor) {
                void loadStaticContent(target).then((ready) => {
                    if (ready && !focusRouteAnchor(target, anchor)) focusRouteContent(target);
                });
            }
            return;
        }
        pendingRouteAnchor = anchor;
        setRoute(target);
    }

    function applyRoute({ focus = true } = {}) {
        const routeAtStart = route;
        const anchorAtStart = pendingRouteAnchor;
        pendingRouteAnchor = '';
        $$('.js-page').forEach((page) => {
            const current = page.dataset.page === route;
            page.hidden = !current;
            if (current) page.id = 'main-content';
            else page.removeAttribute('id');
        });
        document.documentElement.removeAttribute('data-spa-booting');
        $$('[data-route-link]').forEach((link) => {
            const target = link.dataset.routeLink;
            const active = target === route
                || routeRegistry.groupOf(route) === target;
            link.classList.toggle('active', active);
            if (active) link.setAttribute('aria-current', 'page');
            else link.removeAttribute('aria-current');
        });
        syncRouteMetadata(route);
        window.scrollTo(0, 0);
        if (route !== 'tool-theremin' && typeof window.thereminStop === 'function') {
            window.thereminStop();
        }
        const contentPromise = $(`[data-page="${routeAtStart}"] .js-static-content`)
            ? prepareRouteContent(routeAtStart)
            : Promise.resolve(true);
        if (route === 'home') updateReleaseBadge();
        contentPromise.then((ready) => {
            if (!ready || route !== routeAtStart) return;
            renderBrowserSupport();
            renderDirectBrowserGuidance();
            renderConnection();
            consumeDirectHandoff();
            if (routeAtStart === 'tool-configure' && conn.status === 'connected') {
                void cfgRefreshDevice();
            }
            if (routeAtStart === 'tool-monitor') window.monitorResizeChart();
            if (routeAtStart === 'tool-raw-csi') window.rawCsiUseConnection();
            if (routeAtStart === 'tool-game') {
                void window.gameLoadFactoryImage();
                requestAnimationFrame(() => {
                    window.gameResizeCanvas();
                    window.gameSetFlight(window.gameSensingActive());
                    window.gameStartPreview();
                });
            }
            if (routeAtStart === 'tool-flash') {
                flashRefresh();
            }
            if (focus || anchorAtStart) {
                if (!anchorAtStart || !focusRouteAnchor(routeAtStart, anchorAtStart)) {
                    focusRouteContent(routeAtStart);
                }
            }
        });
        // The router owns navigation, so it reports it.
        if (window.trackRouteView) window.trackRouteView(route);
    }

    function clearApiReferenceLocation(previousRoute, nextRoute) {
        if (previousRoute !== 'sdk-api' || nextRoute === 'sdk-api') return;
        const url = new URL(location.href);
        if (!url.searchParams.has('api') && !url.searchParams.has('member')) return;
        url.searchParams.delete('api');
        url.searchParams.delete('member');
        history.replaceState(history.state, '', url.pathname + url.search + url.hash);
    }

    /**
     * Single entry point for navigation. `force` applies the current route on
     * startup; without it a repeated route is ignored so one navigation never
     * reports two page views.
     */
    function setRoute(next, { force = false, focus = true } = {}) {
        const target = normalizedRouteName(next);
        if (!force && target === route) return;
        if (route === 'tool-flash' && target !== route
                && typeof window.flashRouteLeave === 'function' && !window.flashRouteLeave()) {
            history.replaceState(
                { espectreRoute: route }, '', routeHistoryUrl(route)
            );
            return;
        }
        cancelDirectDiscovery({ clear: true });
        const previousRoute = route;
        if (previousRoute === 'tool-configure' && target !== previousRoute) {
            cancelWifiScan();
            // Re-read resources without SSE updates on the next settings visit.
            const session = directClient && directResourceSessions.get(directClient);
            for (const resource of session?.snapshots.keys() || []) {
                if (!directClient.capabilities?.events?.includes(resource)) session.snapshots.delete(resource);
            }
        }
        clearApiReferenceLocation(previousRoute, target);
        if (previousRoute === 'tool-raw-csi' && target !== 'tool-raw-csi'
                && typeof window.rawCsiStop === 'function') {
            void window.rawCsiStop('route_change');
        }
        if (pendingLiveDestination) {
            if (LIVE_EXPERIENCE_ROUTES.has(target)) pendingLiveDestination = target;
            else if (target !== 'tool-monitor' && target !== 'tool-configure') pendingLiveDestination = '';
        }
        if (previousRoute === 'tool-game' && target !== 'tool-game'
                && typeof window.gameExitFullscreen === 'function') {
            window.gameExitFullscreen();
            window.reportGameAbandon('route_change');
        }
        if (target === 'tool-game' && previousRoute !== 'tool-game') resetGameThreshold();
        route = target;
        dropdownOpen = false;
        applyRoute({ focus });
        renderConnection();
    }

    /* ======================================================= static content */

    /*
     * Tools, guides, docs, media, and the roadmap live in shared HTML
     * fragments, which also build their canonical static pages. The SPA
     * fetches each fragment on first visit so content is not duplicated and
     * the device connection survives.
    */
    const staticContentCache = new Map();
    const staticContentLoads = new Map();
    const initializedToolRoutes = new Set();
    const toolInitializers = Object.freeze({
        'tool-flash': 'flashInit',
        'tool-configure': 'configureInit',
        'tool-monitor': 'monitorInit',
        'tool-raw-csi': 'rawCsiInit',
        'tool-theremin': 'thereminInit',
        'tool-game': 'gameInit'
    });

    function toolScriptUrl(routeName) {
        return $(`[data-page="${routeName}"]`)?.dataset.scriptSrc || '';
    }

    async function loadToolScript(routeName) {
        const src = toolScriptUrl(routeName);
        if (src) await loadScriptOnce(src);
    }

    async function ensureActiveToolScripts() {
        await loadToolScript(route);
    }

    function loadStaticContent(route) {
        const container = $(`[data-page="${route}"] .js-static-content`);
        if (!container || container.dataset.loaded === 'true') return Promise.resolve(true);
        if (staticContentLoads.has(route)) return staticContentLoads.get(route);
        const definition = routeRegistry.get(route);
        const contentPath = routeRegistry.contentPath(route);
        if (!contentPath || !definition?.staticPath) return Promise.resolve(false);
        const contentUrl = `/${contentPath}`;
        const load = (async () => {
            try {
                if (!staticContentCache.has(contentUrl)) {
                    const response = await fetch(contentUrl, { cache: 'no-cache' });
                    if (!response.ok) throw new Error('HTTP ' + response.status);
                    staticContentCache.set(contentUrl, await response.text());
                }
                container.innerHTML = staticContentCache.get(contentUrl);
                if (window.initPageTocs) window.initPageTocs(container);
                if (window.initPagePaths) window.initPagePaths(container);
                if (window.initSdkDownloadVersions) window.initSdkDownloadVersions(container);
                if (window.initPublishedReleaseTags) window.initPublishedReleaseTags(container);
                if (window.initCodeTabs) window.initCodeTabs(container);
                if (window.initApiReferenceBrowsers) window.initApiReferenceBrowsers(container);
                container.dataset.loaded = 'true';
                return true;
            } catch (error) {
                console.warn('Static content fetch failed:', error);
                container.innerHTML = '<p class="guide-loading">This page could not be loaded. '
                    + '<a href="' + definition.staticPath + '">Open the standalone page</a>.</p>';
                return false;
            }
        })();
        staticContentLoads.set(route, load);
        load.finally(() => staticContentLoads.delete(route));
        return load;
    }

    async function prepareRouteContent(route) {
        const ready = await loadStaticContent(route);
        const initializerName = toolInitializers[route];
        if (!ready || !initializerName || initializedToolRoutes.has(route)) return ready;
        await loadToolScript(route);
        const initializer = window[initializerName];
        if (typeof initializer !== 'function') {
            throw new Error(`Tool initializer ${initializerName} is unavailable`);
        }
        initializer();
        initializedToolRoutes.add(route);
        if (conn.mode === 'demo' && demoSysinfoSnapshot) {
            applySysinfo(demoSysinfoSnapshot);
        }
        return true;
    }

    /*
     * Fragments and content cards link to the canonical static URLs. Inside
     * the app those clicks use the History API, so the address remains
     * shareable without reloading or dropping an active device connection.
     * Legacy root hash routes remain valid entry points. Modified clicks are
     * left to the browser, and same-page anchors stay on the active route.
     */
    function interceptCanonicalLinks(event) {
        if (event.defaultPrevented || event.button !== 0) return;
        if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
        const link = event.target.closest('a[href]');
        if (!link) return;
        const href = link.getAttribute('href');
        const staticTarget = routeRegistry.staticTargetForHref(href, location.href);
        if (staticTarget) {
            event.preventDefault();
            navigateToRoute(staticTarget.route, { anchor: staticTarget.anchor });
        } else if (href.startsWith('/#')) {
            const legacyRoute = href.slice(2).split('?')[0];
            if (!routeRegistry.has(LEGACY_TOOL_ROUTES[legacyRoute] || legacyRoute)) return;
            event.preventDefault();
            navigateToRoute(legacyRoute);
        } else if (href.startsWith('#') && href.length > 1) {
            const page = $(`[data-page="${route}"]`);
            const targetId = href.slice(1);
            let decodedTargetId;
            try {
                decodedTargetId = decodeURIComponent(targetId);
            } catch (error) {
                return;
            }
            const target = page && document.getElementById(decodedTargetId);
            if (!target || !page.contains(target)) return;
            event.preventDefault();
            history.pushState(
                { espectreRoute: route },
                '',
                routeHistoryUrl(route, { anchor: targetId })
            );
            focusRouteAnchor(route, targetId);
        }
    }

    /* =============================================================== toast */

    let toastTimer = null;

    function toast(message) {
        const el = $('.js-toast');
        el.textContent = message;
        el.hidden = false;
        clearTimeout(toastTimer);
        toastTimer = setTimeout(() => { el.hidden = true; }, 3200);
    }

    function clearDirectConnectionCallout() {
        clearTimeout(connectionCalloutTimer);
        connectionCalloutTimer = null;
        directCalloutVisible = false;
    }

    function showDirectConnectionCallout() {
        clearDirectConnectionCallout();
        directCalloutVisible = true;
        connectionCalloutTimer = setTimeout(() => {
            connectionCalloutTimer = null;
            directCalloutVisible = false;
            syncConnectionCallout();
        }, DIRECT_CALLOUT_DURATION_MS);
    }

    function syncConnectionCallout() {
        const el = $('.js-connection-callout');
        if (!el) return;
        const demo = conn.mode === 'demo' && conn.status === 'connected';
        const direct = conn.mode === 'direct' && conn.status === 'connected' && directCalloutVisible;
        const title = $('.js-connection-callout-title');
        const message = $('.js-connection-callout-message');
        if (title) title.textContent = demo ? 'Demo mode' : 'Device connected';
        if (message) {
            message.textContent = demo
                ? 'Move the pointer, or drag on the chart or pitch display, to simulate motion.'
                : 'ESPectre is ready to use.';
        }
        el.hidden = (!demo && !direct) || dropdownOpen;
    }

    /* ====================================================== scroll narrative */

    let activeScrollyScene = -1;
    let scrollyFrame = null;
    let scrollyKeyTargetScene = null;
    let scrollyKeyTargetTimer = null;
    let heroFrameTimer = null;
    const HERO_FRAME_HOLD = 2000;

    function scrollySceneFromPosition(section, sceneCount) {
        const rect = section.getBoundingClientRect();
        const travel = Math.max(1, rect.height - window.innerHeight);
        const progress = Math.min(1, Math.max(0, -rect.top / travel));
        return Math.min(sceneCount - 1, Math.floor(progress * sceneCount));
    }

    function stopHeroFrameSequence() {
        clearTimeout(heroFrameTimer);
        heroFrameTimer = null;
    }

    function startHeroFrameSequence() {
        const media = $('.hero-media');
        stopHeroFrameSequence();
        media.classList.remove('is-connected');
        if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
            media.classList.add('is-connected');
            return;
        }
        heroFrameTimer = setTimeout(() => {
            media.classList.add('is-connected');
            heroFrameTimer = null;
        }, HERO_FRAME_HOLD);
    }

    function setScrollyScene(scene) {
        if (scene === activeScrollyScene) return;
        activeScrollyScene = scene;

        const scenes = $$('.js-scrolly-scene');
        const useMobileAsset = window.matchMedia('(max-width: 720px)').matches;
        [scene, scene + 1].forEach((index) => {
            const image = scenes[index] && scenes[index].querySelector('img[data-src]');
            if (!image) return;
            image.src = useMobileAsset && image.dataset.srcMobile
                ? image.dataset.srcMobile
                : image.dataset.src;
            image.removeAttribute('data-src');
            image.removeAttribute('data-src-mobile');
        });

        $$('.js-scrolly-scene, .js-scrolly-caption, .js-scrolly-marker').forEach((el) => {
            const isActive = Number(el.dataset.scene) === scene;
            el.classList.toggle('is-active', isActive);
            if (el.classList.contains('js-scrolly-caption')) {
                el.toggleAttribute('inert', !isActive);
                el.setAttribute('aria-hidden', String(!isActive));
            }
        });
        $('.scrolly-stage').classList.toggle('is-intro', scene === 0);
        if (scene === 0) startHeroFrameSequence();
        else stopHeroFrameSequence();
        if (scene > 0) $('.js-scrolly-current').textContent = String(scene).padStart(2, '0');
    }

    function renderScrolly() {
        scrollyFrame = null;
        const section = $('.js-scrolly');
        if (!section || section.offsetParent === null) return;

        const sceneCount = $$('.js-scrolly-scene').length;
        setScrollyScene(scrollySceneFromPosition(section, sceneCount));
    }

    function queueScrollyRender() {
        if (scrollyFrame !== null) return;
        scrollyFrame = requestAnimationFrame(renderScrolly);
    }

    function scrollyHandleKeydown(event) {
        if (event.key !== 'ArrowDown' && event.key !== 'ArrowUp') return;
        if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) return;
        const target = event.target;
        if (target instanceof Element && target.closest('a, button, input, select, textarea, [contenteditable="true"]')) return;

        const section = $('.js-scrolly');
        if (!section || section.offsetParent === null) return;
        const rect = section.getBoundingClientRect();
        if (rect.bottom <= 0 || rect.top >= window.innerHeight) return;

        const sceneCount = $$('.js-scrolly-scene').length;
        const currentScene = scrollyKeyTargetScene === null
            ? scrollySceneFromPosition(section, sceneCount)
            : scrollyKeyTargetScene;
        const direction = event.key === 'ArrowDown' ? 1 : -1;
        const nextScene = Math.min(sceneCount - 1, Math.max(0, currentScene + direction));
        if (nextScene === currentScene) return;

        event.preventDefault();
        scrollyKeyTargetScene = nextScene;
        clearTimeout(scrollyKeyTargetTimer);
        scrollyKeyTargetTimer = setTimeout(() => { scrollyKeyTargetScene = null; }, 500);

        const travel = Math.max(1, rect.height - window.innerHeight);
        const sectionTop = window.scrollY + rect.top;
        const sceneProgress = (nextScene + 0.5) / sceneCount;
        window.scrollTo({
            top: sectionTop + (travel * sceneProgress),
            behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth'
        });
    }

    function scrollyInit() {
        window.addEventListener('scroll', queueScrollyRender, { passive: true });
        window.addEventListener('resize', queueScrollyRender);
        document.addEventListener('keydown', scrollyHandleKeydown);
        renderScrolly();
    }

    async function runToolAction(load, action) {
        try {
            await load();
            return await action();
        } catch (error) {
            console.warn('Browser tool could not be loaded:', error);
            toast('This browser tool could not be loaded. Refresh the page and try again.');
            return undefined;
        }
    }

    function runConfigureAction(action) {
        return runToolAction(() => loadToolScript('tool-configure'), action);
    }

    function runConnectionAction(action) {
        return runToolAction(ensureActiveToolScripts, action);
    }

    function sharedDialogsInit() {
        $$('.js-config-clear-cancel').forEach((button) => {
            button.addEventListener('click', () => {
                void runConfigureAction(() => window.closeConfigClearDialog(false));
            });
        });
        $('.js-config-clear-confirm').addEventListener('click', () => {
            void runConfigureAction(() => window.closeConfigClearDialog(true));
        });
        $('.js-config-clear-modal').addEventListener('click', (event) => {
            if (event.target === event.currentTarget) {
                void runConfigureAction(() => window.closeConfigClearDialog(false));
            }
        });
        $('.js-ota-start').addEventListener('click', () => {
            void runConfigureAction(() => window.cfgOtaStart());
        });
        document.getElementById('ota-channel').addEventListener('change', () => {
            void runConfigureAction(() => {
                if (conn.mode === null) return;
                otaChannelChanged = true;
                window.startManualOtaCheck();
            });
        });
        $$('.js-ota-close').forEach((button) => {
            button.addEventListener('click', () => {
                void runConfigureAction(() => window.otaClose());
            });
        });
        $('.js-ota-modal').addEventListener('click', (event) => {
            if (event.target === event.currentTarget) {
                void runConfigureAction(() => window.otaClose());
            }
        });
        document.addEventListener('keydown', (event) => {
            if (event.key !== 'Escape') return;
            if (!$('.js-config-clear-modal').hidden) {
                void runConfigureAction(() => window.closeConfigClearDialog(false));
            } else if (!$('.js-ota-modal').hidden) {
                void runConfigureAction(() => window.otaClose());
            }
        });
    }

    function sharedToolControlsInit() {
        document.addEventListener('click', (event) => {
            if (!(event.target instanceof Element)) return;
            const connectButton = event.target.closest('.js-connect-direct');
            if (connectButton) {
                void runConnectionAction(() => connectDirect({
                    openView: connectButton.closest('espectre-direct-connect')?.dataset.openView
                }));
                return;
            }
            const discoveryButton = event.target.closest('.js-direct-discover');
            if (discoveryButton) {
                void runConnectionAction(() => discoverLocalPeers(discoveryButton));
                return;
            }
            const discoveredDevice = event.target.closest('.direct-discovery-device');
            if (discoveredDevice?.dataset.endpoint) {
                const input = discoveredDevice.closest('.device-connect-card')
                    ?.querySelector('.js-direct-endpoint');
                if (input) input.value = discoveredDevice.dataset.deviceId;
                void runConnectionAction(() => connectDirect({
                    endpoint: discoveredDevice.dataset.endpoint,
                    deviceId: discoveredDevice.dataset.deviceId,
                    openView: discoveredDevice.closest('espectre-direct-connect')?.dataset.openView
                }));
                return;
            }
            const startButton = event.target.closest('.js-start-detection');
            if (startButton) {
                void runConnectionAction(() => startDetection(startButton.dataset.liveTransport || ''));
                return;
            }
            const demoButton = event.target.closest('.js-demo');
            if (demoButton) {
                void runConnectionAction(() => connectDemo(
                    demoButton.closest('espectre-connection-picker')?.dataset.openView || ''
                ));
                return;
            }
            const firmwareButton = event.target.closest('.js-firmware-update-notice');
            if (firmwareButton) {
                void runConfigureAction(() => window.otaOpen(firmwareButton));
            }
        });
    }

    /* ================================================================ init */

    async function init() {
        routeRegistry = await window.ESPectreRoutesReady;
        if (!routeRegistry) throw new Error('ESPectre route registry is unavailable');
        scrollyInit();

        renderBrowserSupport();
        renderDirectBrowserGuidance();
        consumeRouteAnchorHandoff();
        const initialRoute = routeFromLocation();
        replaceLegacyRouteLocation(initialRoute);
        sharedDialogsInit();
        sharedToolControlsInit();
        $('.js-header-connect').addEventListener('click', () => {
            void runConnectionAction(() => {
                window.selectMonitorTransport('direct');
                if (route === 'tool-monitor') {
                    document.getElementById('monitor-direct-endpoint')?.focus();
                    return;
                }
                pendingLiveDestination = '';
                navigateToRoute('tool-monitor');
            });
        });
        $('.js-disconnect').addEventListener('click', disconnect);
        $('.js-dropdown-toggle').addEventListener('click', (event) => {
            event.stopPropagation();
            dropdownOpen = !dropdownOpen;
            renderConnection();
        });
        document.addEventListener('click', (event) => {
            if (dropdownOpen && !event.target.closest('.conn')) {
                dropdownOpen = false;
                renderConnection();
            }
        });
        document.addEventListener('click', interceptCanonicalLinks);
        $('.skip-link').addEventListener('click', (event) => {
            event.preventDefault();
            focusRouteContent();
        });
        document.addEventListener('pointerdown', demoStartPointer);
        document.addEventListener('pointermove', demoTrackPointer, { passive: true });
        document.addEventListener('pointerup', demoEndPointer);
        document.addEventListener('pointercancel', demoEndPointer);
        document.addEventListener('lostpointercapture', demoEndPointer);
        window.addEventListener('popstate', onPopState);
        setRoute(initialRoute, { force: true, focus: false });
    }

    document.addEventListener('espectre:analytics-enabled', () => {
        if (window.trackRouteView) window.trackRouteView(route, { sendPageView: false });
        if (conn.readyState) markToolReady(conn.readyState);
        if (monitor.readyState && typeof window.markMonitorReady === 'function') {
            window.markMonitorReady(monitor.readyState);
        }
        if (conn.mode === 'direct' && directClient && typeof window.cfgRefreshDevice === 'function') {
            window.cfgRefreshDevice();
        }
        if (route === 'tool-flash') {
            void prepareRouteContent(route).then((ready) => {
                if (ready && route === 'tool-flash') flashRefresh();
            });
        }
    });
    document.addEventListener('visibilitychange', () => {
        syncDiagnosticsPolling();
        if (document.hidden) cancelWifiScan();
    });
    window.addEventListener('pagehide', (event) => {
        if (event.persisted) return;
        if (typeof window.flashCleanup === 'function') void window.flashCleanup();
        if (typeof window.rawCsiStop === 'function') void window.rawCsiStop('page_exit');
        if (typeof window.reportGameAbandon === 'function') {
            window.reportGameAbandon('page_exit');
        }
        if (conn.mode) teardownConnection('page_exit');
        else if (typeof window.monitorStopAll === 'function') window.monitorStopAll('page_exit');
    });
    document.addEventListener('DOMContentLoaded', () => {
        void init().catch((error) => console.error('Unable to initialize ESPectre:', error));
    });
