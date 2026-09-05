/*
 * ESPectre - Shared responsive navigation
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

(function () {
    'use strict';

    const compactToc = window.matchMedia('(max-width: 1439px)');
    const pagePathLabels = Object.freeze({
        guides: 'Guides',
        sdk: 'SDK guide',
        tools: 'Tools'
    });
    let pageTocUpdateFrame = 0;

    function normalizeSpaEntryPath() {
        if (window.location.pathname !== '/index.html') return;
        const canonicalUrl = new URL('/', window.location.href);
        canonicalUrl.search = window.location.search;
        canonicalUrl.hash = window.location.hash;
        window.history.replaceState(window.history.state, '', canonicalUrl);
    }

    function interceptStaticPageRoute(event) {
        if (!document.documentElement.hasAttribute('data-static-page')) return;
        if (event.defaultPrevented || event.button !== 0) return;
        if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
        const link = event.target.closest('a[href]');
        if (!link || (link.target && link.target !== '_self')) return;
        const href = link.getAttribute('href');
        let target = window.ESPectreRoutes?.staticTargetForHref(href, window.location.href);
        let directTarget = '';
        if (!target) {
            const legacyRoute = href.startsWith('/#tool-') ? href.slice(2) : '';
            if (!legacyRoute) return;
            target = { route: legacyRoute, anchor: '' };
            directTarget = new URLSearchParams(window.location.search).get('target') || '';
        }
        event.preventDefault();
        const destination = new URL('/', window.location.href);
        if (directTarget) destination.searchParams.set('target', directTarget);
        if (target.anchor) destination.searchParams.set('anchor', target.anchor);
        destination.hash = target.route;
        window.location.assign(destination.pathname + destination.search + destination.hash);
    }

    normalizeSpaEntryPath();

    function setSideRailMode(rail) {
        rail.open = !compactToc.matches;
    }

    function pageTocHeadingId(heading) {
        if (heading.id) return heading.id;
        const base = heading.textContent.trim().toLowerCase()
            .normalize('NFKD')
            .replace(/[\u0300-\u036f]/g, '')
            .replace(/[^a-z0-9]+/g, '-')
            .replace(/^-|-$/g, '') || 'section';
        let candidate = base;
        let suffix = 2;
        while (document.getElementById(candidate)) {
            candidate = `${base}-${suffix}`;
            suffix += 1;
        }
        heading.id = candidate;
        return candidate;
    }

    function buildPageToc(container) {
        if (container.querySelector('.page-toc')) return;
        const headings = [...container.querySelectorAll('h2:not([data-toc-exclude])')]
            .filter((heading) => heading.closest('[data-page-toc]') === container);
        if (headings.length < 2) return;

        const toc = document.createElement('details');
        toc.className = 'page-toc';
        toc.dataset.tocGenerated = 'true';
        const summary = document.createElement('summary');
        summary.textContent = 'Sections';
        const nav = document.createElement('nav');
        nav.setAttribute('aria-label', 'Page sections');
        const title = document.createElement('strong');
        title.textContent = 'On this page';
        nav.append(title);
        headings.forEach((heading) => {
            const link = document.createElement('a');
            link.href = `#${pageTocHeadingId(heading)}`;
            link.textContent = heading.dataset.tocLabel || heading.textContent.trim().replace(/[.:]$/, '');
            nav.append(link);
        });
        toc.append(summary, nav);

        let insertionPoint = headings[0];
        while (insertionPoint.parentElement !== container) {
            insertionPoint = insertionPoint.parentElement;
        }
        container.insertBefore(toc, insertionPoint);
    }

    function currentPagePathRoute(container) {
        const spaRoute = container.closest('main[data-page]')?.dataset.page;
        if (spaRoute) return spaRoute;
        return window.ESPectreRoutes?.routeForPath(window.location.pathname) || '';
    }

    function buildPagePath(container) {
        if (container.querySelector(':scope > .page-path')) return;
        const group = container.dataset.pagePath;
        const entries = window.ESPectreRoutes?.membersOf(group) || [];
        if (entries.length < 2) return;

        const label = pagePathLabels[group] || group;
        const currentRoute = currentPagePathRoute(container);
        const path = document.createElement('details');
        path.className = 'page-path';
        path.dataset.pathGenerated = 'true';
        const summary = document.createElement('summary');
        summary.textContent = label;
        const nav = document.createElement('nav');
        nav.setAttribute('aria-label', label);
        const title = document.createElement('strong');
        title.textContent = label;
        nav.append(title);
        entries.forEach((entry) => {
            const link = document.createElement('a');
            link.href = entry.staticPath;
            link.textContent = entry.pathLabel;
            if (entry.name === currentRoute) link.setAttribute('aria-current', 'page');
            nav.append(link);
        });
        path.append(summary, nav);

        container.append(path);
    }

    function ensurePageTocs(root) {
        const containers = [...root.querySelectorAll('[data-page-toc]')];
        if (root.matches?.('[data-page-toc]')) containers.unshift(root);
        containers.forEach(buildPageToc);
    }

    function ensurePagePaths(root) {
        const containers = [...root.querySelectorAll('[data-page-path]')];
        if (root.matches?.('[data-page-path]')) containers.unshift(root);
        containers.forEach(buildPagePath);
    }

    function updatePageToc(toc) {
        const links = [...toc.querySelectorAll('a[href^="#"]')];
        if (compactToc.matches) {
            toc.classList.remove('is-visible');
            links.forEach((link) => link.removeAttribute('aria-current'));
            return;
        }

        const article = toc.closest('[data-page-toc]') || toc.closest('.guide-article');
        if (!article) return;
        const headerHeight = parseFloat(getComputedStyle(document.documentElement)
            .getPropertyValue('--header-height')) || 64;
        const railTop = headerHeight + 36;
        const articleRect = article.getBoundingClientRect();
        const articleIsReadable = articleRect.top < window.innerHeight && articleRect.bottom > railTop + 24;
        toc.classList.toggle('is-visible', articleIsReadable);
        if (!links.length) return;

        let activeLink = links[0];
        const remainingArticleScroll = Math.max(0, articleRect.bottom - window.innerHeight);
        const endAwareLine = window.innerHeight - remainingArticleScroll - 24;
        const activationLine = window.scrollY > 0
            ? Math.max(railTop + 24, endAwareLine)
            : railTop + 24;
        links.forEach((link) => {
            const target = document.getElementById(link.hash.slice(1));
            if (target && target.getBoundingClientRect().top <= activationLine) activeLink = link;
        });
        links.forEach((link) => {
            if (link === activeLink) link.setAttribute('aria-current', 'location');
            else link.removeAttribute('aria-current');
        });
    }

    function updatePageTocs() {
        document.querySelectorAll('details.page-toc[data-toc-initialized]')
            .forEach(updatePageToc);
    }

    function schedulePageTocUpdate() {
        if (pageTocUpdateFrame) return;
        pageTocUpdateFrame = window.requestAnimationFrame(() => {
            pageTocUpdateFrame = 0;
            updatePageTocs();
        });
    }

    function initPageTocs(root = document) {
        ensurePageTocs(root);
        root.querySelectorAll('details.page-toc:not([data-toc-initialized])').forEach((toc) => {
            toc.dataset.tocInitialized = 'true';
            setSideRailMode(toc);
        });
        schedulePageTocUpdate();
    }

    function initPagePaths(root = document) {
        ensurePagePaths(root);
        root.querySelectorAll('details.page-path:not([data-page-path-initialized])').forEach((path) => {
            path.dataset.pagePathInitialized = 'true';
            setSideRailMode(path);
        });
    }

    const sdkVersionPromises = new Map();

    function sdkVersion(channel) {
        if (sdkVersionPromises.has(channel)) return sdkVersionPromises.get(channel);
        const promise = fetch(`/artifacts/sdk/${channel}/sdk-manifest-${channel}.json`)
            .then((response) => {
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                return response.json();
            })
            .then((manifest) => {
                if (typeof manifest.version !== 'string' || !manifest.version) {
                    throw new Error('SDK manifest version is unavailable');
                }
                return manifest.version;
            });
        sdkVersionPromises.set(channel, promise);
        return promise;
    }

    function initSdkDownloadVersions(root = document) {
        root.querySelectorAll('[data-sdk-version]:not([data-sdk-version-initialized])').forEach((label) => {
            label.dataset.sdkVersionInitialized = 'true';
            const channel = label.dataset.sdkVersion;
            sdkVersion(channel).then((version) => {
                label.textContent = `Version ${version}`;
            }).catch(() => {
                label.title = `${channel} version unavailable`;
            });
        });
    }

    let publishedReleaseTagPromise;

    function publishedReleaseTag() {
        if (publishedReleaseTagPromise) return publishedReleaseTagPromise;
        publishedReleaseTagPromise = fetch('/artifacts/firmware/release/firmware-manifest-release.json', {
            cache: 'no-store'
        })
            .then((response) => {
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                return response.json();
            })
            .then((manifest) => {
                const version = String(manifest.release_tag || manifest.version || '').replace(/^v/, '');
                if (!version) throw new Error('Published release tag is unavailable');
                return `v${version}`;
            });
        return publishedReleaseTagPromise;
    }

    function initPublishedReleaseTags(root = document) {
        root.querySelectorAll('[data-published-release-tag]:not([data-published-release-tag-initialized])').forEach((label) => {
            label.dataset.publishedReleaseTagInitialized = 'true';
            publishedReleaseTag().then((tag) => {
                label.textContent = `${tag} available`;
            }).catch(() => {
                label.title = 'Published release unavailable';
            });
        });
    }

    function activateCodeTab(group, nextTab, moveFocus = false) {
        const tabs = [...group.querySelectorAll('[role="tab"]')];
        const panels = [...group.querySelectorAll('[role="tabpanel"]')];
        tabs.forEach((tab) => {
            const active = tab === nextTab;
            tab.setAttribute('aria-selected', String(active));
            tab.tabIndex = active ? 0 : -1;
        });
        panels.forEach((panel) => {
            panel.hidden = panel.id !== nextTab.getAttribute('aria-controls');
        });
        if (moveFocus) nextTab.focus();
    }

    function initCodeTabs(root = document) {
        root.querySelectorAll('[data-code-tabs]:not([data-code-tabs-initialized])').forEach((group) => {
            group.dataset.codeTabsInitialized = 'true';
            const tabs = [...group.querySelectorAll('[role="tab"]')];
            if (!tabs.length) return;
            const initialTab = tabs.find((tab) => tab.getAttribute('aria-selected') === 'true') || tabs[0];
            activateCodeTab(group, initialTab);
            tabs.forEach((tab, index) => {
                tab.addEventListener('click', () => activateCodeTab(group, tab));
                tab.addEventListener('keydown', (event) => {
                    let nextIndex;
                    if (event.key === 'ArrowRight') nextIndex = (index + 1) % tabs.length;
                    else if (event.key === 'ArrowLeft') nextIndex = (index - 1 + tabs.length) % tabs.length;
                    else if (event.key === 'Home') nextIndex = 0;
                    else if (event.key === 'End') nextIndex = tabs.length - 1;
                    else return;
                    event.preventDefault();
                    activateCodeTab(group, tabs[nextIndex], true);
                });
            });
        });
    }

    const apiReferenceManifestPromises = new Map();

    function apiReferenceManifest(url) {
        if (apiReferenceManifestPromises.has(url)) return apiReferenceManifestPromises.get(url);
        const promise = fetch(url, { cache: 'no-cache' })
            .then((response) => {
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                return response.json();
            })
            .then((manifest) => {
                if (!Array.isArray(manifest.entries) || !manifest.entries.length || !manifest.default) {
                    throw new Error('API reference manifest is incomplete');
                }
                return manifest;
            });
        apiReferenceManifestPromises.set(url, promise);
        return promise;
    }

    function apiReferenceLocation() {
        const params = new URLSearchParams(window.location.search);
        return { refid: params.get('api') || '', member: params.get('member') || '' };
    }

    function updateApiReferenceLocation(refid, member, replace = false) {
        const url = new URL(window.location.href);
        if (refid) url.searchParams.set('api', refid);
        else url.searchParams.delete('api');
        if (member) url.searchParams.set('member', member);
        else url.searchParams.delete('member');
        window.history[replace ? 'replaceState' : 'pushState'](
            window.history.state, '', `${url.pathname}${url.search}${url.hash}`
        );
    }

    function apiReferenceEntryLabel(entry, manifest) {
        if (entry.refid === manifest.default) return 'Overview';
        return `${entry.name.replace(/^espectre::/, '')} — ${entry.kind}`;
    }

    function renderApiReferencePicker(browser, manifest, query = '') {
        const filter = browser.querySelector('[data-api-reference-filter]');
        const popover = browser.querySelector('[data-api-reference-popover]');
        const results = browser.querySelector('[data-api-reference-results]');
        if (!filter || !results || !manifest) return;
        const current = browser.dataset.apiReferenceCurrent || manifest.default;
        const entry = manifest.entries.find((candidate) => candidate.refid === current);
        if (popover?.hidden !== false) {
            filter.value = entry ? apiReferenceEntryLabel(entry, manifest) : 'Choose an API symbol';
        }
        const normalizedQuery = query.trim().toLocaleLowerCase();
        const visibleEntries = manifest.entries.filter((candidate) => {
            if (candidate.discoverable === false) return false;
            if (!normalizedQuery) return true;
            return `${candidate.name} ${candidate.kind} ${apiReferenceEntryLabel(candidate, manifest)}`
                .toLocaleLowerCase().includes(normalizedQuery);
        });
        if (!visibleEntries.length) {
            const empty = document.createElement('p');
            empty.className = 'api-reference-picker-empty';
            empty.textContent = 'No matching API symbols.';
            results.replaceChildren(empty);
            return;
        }
        results.replaceChildren(...visibleEntries.map((candidate) => {
            const option = document.createElement('button');
            option.type = 'button';
            option.dataset.apiReferenceRef = candidate.refid;
            option.setAttribute('role', 'option');
            option.setAttribute('aria-selected', String(candidate.refid === current));
            const name = document.createElement('span');
            name.textContent = candidate.refid === manifest.default
                ? 'Overview'
                : candidate.name.replace(/^espectre::/, '');
            option.append(name);
            if (candidate.refid !== manifest.default) {
                const kind = document.createElement('small');
                kind.textContent = candidate.kind;
                option.append(kind);
            }
            return option;
        }));
    }

    function setApiReferencePickerOpen(browser, open) {
        const popover = browser.querySelector('[data-api-reference-popover]');
        const filter = browser.querySelector('[data-api-reference-filter]');
        if (!filter || !popover) return;
        filter.setAttribute('aria-expanded', String(open));
        popover.hidden = !open;
        if (open) {
            renderApiReferencePicker(browser, browser.apiReferenceManifest);
            window.requestAnimationFrame(() => filter.select());
        }
    }

    function closeApiReferencePicker(browser) {
        setApiReferencePickerOpen(browser, false);
        renderApiReferencePicker(browser, browser.apiReferenceManifest);
    }

    function refreshApiReferenceToc(browser) {
        browser.querySelector(':scope > .page-toc[data-toc-generated]')?.remove();
        initPageTocs(browser);
    }

    const passiveApiReferenceTags = new Set([
        'a', 'article', 'aside', 'blockquote', 'br', 'code', 'dd', 'details', 'div', 'dl',
        'dt', 'em', 'h1', 'h2', 'h3', 'h4', 'hr', 'kbd', 'li', 'ol', 'p', 'pre', 'samp',
        'section', 'small', 'span', 'strong', 'sub', 'summary', 'sup', 'table', 'tbody',
        'td', 'tfoot', 'th', 'thead', 'tr', 'ul', 'var', 'wbr',
    ]);
    const passiveApiReferenceAttributes = new Set([
        'class', 'colspan', 'data-api-reference-fragment', 'data-api-reference-member',
        'data-api-reference-ref', 'href', 'id', 'name', 'role', 'rowspan', 'scope', 'style',
        'title',
    ]);

    function isSafeApiReferenceHref(value) {
        // eslint-disable-next-line no-control-regex -- rejects control characters in untrusted URLs.
        if (!value || /[\u0000-\u0020]/.test(value)) return false;
        if (value.startsWith('#') || /^\/(?!\/)/.test(value)
            || value.startsWith('./') || value.startsWith('../')) return true;
        if (!/^[a-z][a-z0-9+.-]*:/i.test(value)) return false;
        try {
            return ['http:', 'https:', 'mailto:', 'tel:'].includes(new URL(value).protocol);
        } catch {
            return false;
        }
    }

    function parsePassiveApiReferenceFragment(fragment) {
        const template = document.createElement('template');
        template.innerHTML = fragment;
        for (const element of template.content.querySelectorAll('*')) {
            if (!passiveApiReferenceTags.has(element.localName)) {
                throw new Error(`Unsafe API reference element: ${element.localName}`);
            }
            for (const attribute of element.attributes) {
                const name = attribute.name.toLowerCase();
                if (name.startsWith('on')
                    || (!name.startsWith('aria-') && !passiveApiReferenceAttributes.has(name))) {
                    throw new Error(`Unsafe API reference attribute: ${name}`);
                }
                if (name === 'style' && !/^width:\s*1%\s*;?$/i.test(attribute.value)) {
                    throw new Error('Unsafe API reference inline style');
                }
                if (name === 'href' && !isSafeApiReferenceHref(attribute.value)) {
                    throw new Error('Unsafe API reference link');
                }
            }
        }
        return template.content;
    }

    async function showApiReference(browser, refid, member = '', options = {}) {
        const manifest = browser.apiReferenceManifest;
        if (!manifest) return;
        const requestId = (browser.apiReferenceRequestId || 0) + 1;
        browser.apiReferenceRequestId = requestId;
        const entries = new Map(manifest.entries.map((entry) => [entry.refid, entry]));
        const entry = entries.get(refid) || entries.get(manifest.default) || manifest.entries[0];
        const content = browser.querySelector('[data-api-reference-content]');
        const showingOverview = entry.refid === manifest.default;
        if (showingOverview) member = '';
        content.setAttribute('aria-busy', 'true');
        try {
            let fragment = null;
            if (!showingOverview) {
                const response = await fetch(`/artifacts/sdk/api/${entry.fragment}`);
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                fragment = await response.text();
            }
            if (browser.apiReferenceRequestId !== requestId) return;
            if (showingOverview) {
                const overview = browser.apiReferenceOverview.cloneNode(true);
                content.replaceChildren(...overview.childNodes);
            } else {
                content.replaceChildren(parsePassiveApiReferenceFragment(fragment));
            }
            browser.dataset.apiReferenceCurrent = entry.refid;
            renderApiReferencePicker(browser, manifest);
            refreshApiReferenceToc(browser);
            if (options.updateHistory) {
                updateApiReferenceLocation(showingOverview ? '' : entry.refid, member);
            }
            if (member) {
                window.requestAnimationFrame(() => {
                    const target = document.getElementById(member);
                    if (target && content.contains(target)) target.scrollIntoView({ block: 'start' });
                });
            } else if (options.scrollToBrowser) {
                browser.scrollIntoView({ block: 'start' });
            }
        } catch (error) {
            if (browser.apiReferenceRequestId !== requestId) return;
            console.warn('API reference fetch failed:', error);
            const failure = document.createElement('p');
            failure.className = 'guide-loading';
            failure.textContent = 'The generated API reference could not be loaded.';
            content.replaceChildren(failure);
            refreshApiReferenceToc(browser);
        } finally {
            if (browser.apiReferenceRequestId === requestId) content.removeAttribute('aria-busy');
        }
    }

    function apiReferenceClick(event, browser) {
        if (event.defaultPrevented || event.button !== 0) return;
        if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
        const link = event.target.closest('[data-api-reference-ref]');
        if (!link || !browser.apiReferenceManifest) return;
        event.preventDefault();
        closeApiReferencePicker(browser);
        showApiReference(
            browser,
            link.dataset.apiReferenceRef,
            link.dataset.apiReferenceMember || '',
            { updateHistory: true, scrollToBrowser: true }
        );
    }

    function initApiReferenceBrowsers(root = document) {
        root.querySelectorAll('[data-api-reference-browser]:not([data-api-reference-initialized])').forEach((browser) => {
            browser.dataset.apiReferenceInitialized = 'true';
            const content = browser.querySelector('[data-api-reference-content]');
            browser.apiReferenceOverview = content.cloneNode(true);
            browser.addEventListener('click', (event) => apiReferenceClick(event, browser));
            const picker = browser.querySelector('[data-api-reference-picker]');
            const popover = browser.querySelector('[data-api-reference-popover]');
            const filter = browser.querySelector('[data-api-reference-filter]');
            const results = browser.querySelector('[data-api-reference-results]');
            filter?.addEventListener('focus', () => {
                setApiReferencePickerOpen(browser, true);
            });
            filter?.addEventListener('input', () => {
                if (popover?.hidden !== false) setApiReferencePickerOpen(browser, true);
                renderApiReferencePicker(browser, browser.apiReferenceManifest, filter.value);
            });
            filter?.addEventListener('keydown', (event) => {
                if (event.key === 'Escape') {
                    closeApiReferencePicker(browser);
                    filter.blur();
                } else if (event.key === 'ArrowDown') {
                    event.preventDefault();
                    results?.querySelector('button')?.focus();
                }
            });
            results?.addEventListener('keydown', (event) => {
                const options = [...results.querySelectorAll('button')];
                const index = options.indexOf(document.activeElement);
                if (event.key === 'Escape') {
                    filter?.focus();
                    closeApiReferencePicker(browser);
                } else if (event.key === 'ArrowDown' && index >= 0) {
                    event.preventDefault();
                    options[Math.min(index + 1, options.length - 1)]?.focus();
                } else if (event.key === 'ArrowUp' && index >= 0) {
                    event.preventDefault();
                    if (index === 0) filter?.focus();
                    else options[index - 1]?.focus();
                }
            });
            document.addEventListener('click', (event) => {
                if (popover?.hidden === false && !picker?.contains(event.target)) {
                    closeApiReferencePicker(browser);
                }
            });
            apiReferenceManifest(browser.dataset.apiIndex).then((manifest) => {
                browser.apiReferenceManifest = manifest;
                browser.querySelector('[data-api-reference-version]').textContent = `SDK ${manifest.sdk_version}`;
                renderApiReferencePicker(browser, manifest);
                const initial = apiReferenceLocation();
                showApiReference(browser, initial.refid || manifest.default, initial.member);
            }).catch((error) => {
                console.warn('API reference manifest fetch failed:', error);
                browser.querySelector('[data-api-reference-version]').textContent = 'index unavailable';
            });
        });
        root.querySelectorAll('[data-api-reference-ref]:not([data-api-reference-link-initialized])').forEach((link) => {
            if (link.closest('[data-api-reference-browser]')) return;
            link.dataset.apiReferenceLinkInitialized = 'true';
            link.addEventListener('click', (event) => {
                const browser = document.querySelector('[data-api-reference-browser]');
                if (browser) apiReferenceClick(event, browser);
            });
        });
    }

    window.initPageTocs = initPageTocs;
    window.initPagePaths = initPagePaths;
    window.initSdkDownloadVersions = initSdkDownloadVersions;
    window.initPublishedReleaseTags = initPublishedReleaseTags;
    window.initCodeTabs = initCodeTabs;
    window.initApiReferenceBrowsers = initApiReferenceBrowsers;
    compactToc.addEventListener('change', () => {
        document.querySelectorAll('details.page-toc, details.page-path').forEach(setSideRailMode);
        schedulePageTocUpdate();
    });
    window.addEventListener?.('scroll', schedulePageTocUpdate, { passive: true });
    window.addEventListener?.('resize', schedulePageTocUpdate);
    window.addEventListener?.('popstate', () => {
        const location = apiReferenceLocation();
        document.querySelectorAll('[data-api-reference-browser][data-api-reference-initialized]').forEach((browser) => {
            if (browser.apiReferenceManifest) showApiReference(browser, location.refid, location.member);
        });
    });

    function closeNavigation(toggle, nav) {
        nav.classList.remove('is-open');
        toggle.setAttribute('aria-expanded', 'false');
        const label = toggle.querySelector('.sr-only');
        if (label) label.textContent = 'Open navigation';
    }

    document.addEventListener('DOMContentLoaded', async () => {
        try {
            await window.ESPectreRoutesReady;
        } catch (error) {
            console.error('Unable to initialize website navigation:', error);
            return;
        }
        initPageTocs();
        initPagePaths();
        initSdkDownloadVersions();
        initPublishedReleaseTags();
        initCodeTabs();
        initApiReferenceBrowsers();
        const toggle = document.querySelector('.nav-toggle');
        const nav = document.getElementById('main-navigation');
        if (!toggle || !nav) return;

        toggle.addEventListener('click', () => {
            const opening = !nav.classList.contains('is-open');
            nav.classList.toggle('is-open', opening);
            toggle.setAttribute('aria-expanded', String(opening));
            const label = toggle.querySelector('.sr-only');
            if (label) label.textContent = opening ? 'Close navigation' : 'Open navigation';
        });

        nav.addEventListener('click', (event) => {
            if (event.target.closest('a')) closeNavigation(toggle, nav);
        });

        document.addEventListener('click', (event) => {
            document.querySelectorAll('details.sdk-download[open]').forEach((menu) => {
                if (!menu.contains(event.target)) menu.open = false;
            });
            if (!nav.classList.contains('is-open')) return;
            if (nav.contains(event.target) || toggle.contains(event.target)) return;
            closeNavigation(toggle, nav);
        });

        document.addEventListener('keydown', (event) => {
            if (event.key !== 'Escape' || !nav.classList.contains('is-open')) return;
            closeNavigation(toggle, nav);
            toggle.focus();
        });
    });
    document.addEventListener('click', interceptStaticPageRoute);
})();
