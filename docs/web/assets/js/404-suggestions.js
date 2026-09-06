/*
 * ESPectre - Suggested destinations for retired website pages
 *
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */

(async function () {
    const section = document.querySelector('.js-error-suggestion');
    const link = document.querySelector('.js-error-suggestion-link');
    if (!section || !link) return;

    try {
        const response = await fetch('/404-suggestions.json', { cache: 'no-cache' });
        if (!response.ok) return;
        const suggestions = await response.json();
        const path = window.location.pathname.replace(/\/index\.html$/, '/')
            .replace(/\/+$/, '') + '/';
        if (!Object.hasOwn(suggestions, path)) return;
        const suggestion = suggestions[path];
        if (typeof suggestion?.href !== 'string' || typeof suggestion?.title !== 'string') return;

        const destination = new URL(suggestion.href, window.location.origin);
        const local = destination.origin === window.location.origin;
        const projectDoc = destination.origin === 'https://github.com'
            && destination.pathname.startsWith('/francescopace/espectre/');
        if (!local && !projectDoc) return;

        link.href = destination.href;
        link.textContent = suggestion.title;
        link.addEventListener('click', () => window.trackEvent?.('select_404_suggestion'));
        section.hidden = false;
    } catch {
        // Keep the generic navigation available if the suggestion map cannot load.
    }
})();
