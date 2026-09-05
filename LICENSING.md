# Licensing

ESPectre first-party code is available under the open-source GPLv3 license, and eligible portions may also be licensed under a separate commercial agreement when GPLv3 obligations do not fit your product.

## GPLv3 (open source)

Unless a file carries a different SPDX license identifier or third-party notice, ESPectre first-party source code is released under the [GNU General Public License v3.0](LICENSE). You are free to use, study, modify, and redistribute that code, provided that firmware and applications that include it comply with GPLv3, including making the corresponding source available.

## Commercial license

Manufacturers and firmware teams that want to embed ESPectre in proprietary firmware, without the source-disclosure obligations of GPLv3, can obtain a commercial license. Its exact scope is defined by the signed agreement.

For commercial licensing inquiries, contact our team at <contact@espectre.dev>.

## Integration services

Optional architecture review, firmware integration, validation, and tuning services may also be available under a separately scoped services agreement. These services are not included in a commercial license unless they are expressly included in the signed agreement.

For integration-service inquiries, contact our team at <contact@espectre.dev>.

## Contributions

Contributions are accepted so they can be distributed under both licensing tracks:

- Every commit must carry a DCO `Signed-off-by` trailer (`git commit -s`), certifying the origin of the change.
- Contributors sign the [CLA.md](CLA.md) once. The CLA grants the maintainer the rights needed to distribute contributions under both GPLv3 and the commercial license, while contributors retain ownership of their work.

## Third-party components

A commercial license may cover eligible ESPectre first-party material, including the shared `core` and `runtime` layers and the `Native` or `Matter` frontends. 
It does not replace third-party license terms, grant rights to third-party trademarks or media, or cover the GPL-only `ESPHome` frontend.

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the complete attribution and dependency record. Published firmware builds include an SPDX SBOM, a notice summary, and the license files for their exact components, grouped in `firmware-compliance-<channel-or-version>.zip` on GitHub Releases and alongside the corresponding image on the ESPectre website.
