#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Enforce firmware licenses and publish vulnerability findings as SARIF."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from license_expression import ExpressionError, get_spdx_licensing

# Reviewed permissive terms for both GPLv3 and commercial firmware distributions.
PERMISSIVE_LICENSES = {
    "Apache-2.0", "Apache-2.0 WITH LLVM-exception", "MIT", "BSD-2-Clause",
    "BSD-2-Clause-Views", "BSD-3-Clause", "ISC", "Zlib", "CC0-1.0", "Unlicense",
}
OWN_LICENSE = "LicenseRef-ESPectre-Commercial"


def license_allowed(expression: str, frontend: str, *, first_party: bool = False) -> bool:
    licensing = get_spdx_licensing()
    try:
        parsed = licensing.parse(expression, strict=True)
    except ExpressionError:
        return False
    if parsed is None:
        return False
    allowed = PERMISSIVE_LICENSES | ({"GPL-3.0-only"} if frontend == "esphome" else set())
    if first_party:
        allowed = allowed | {OWN_LICENSE}
    # WITH is one indivisible term; OR permits a choice, while AND requires both.
    substitutions = {
        symbol: licensing.TRUE if str(symbol) in allowed else licensing.FALSE
        for symbol in parsed.get_symbols()
    }
    return parsed.subs(substitutions).simplify() == licensing.TRUE


def check_licenses(sbom: dict, frontend: str) -> tuple[list[str], list[str]]:
    packages = {package["SPDXID"]: package for package in sbom["packages"]}
    edges: dict[str, list[str]] = {}
    for relation in sbom["relationships"]:
        if relation["relationshipType"] in {"DESCRIBES", "DEPENDS_ON"}:
            edges.setdefault(relation["spdxElementId"], []).append(relation["relatedSpdxElement"])
    pending = list(edges.get("SPDXRef-DOCUMENT", []))
    if not pending:
        raise ValueError("SBOM has no described firmware package")
    visited: set[str] = set()
    errors, unknown = [], []
    while pending:
        package_id = pending.pop()
        if package_id in visited:
            continue
        visited.add(package_id)
        package = packages[package_id]
        pending.extend(edges.get(package_id, []))
        # The commercial choice is assigned only to owned components by the
        # compliance generator, and must not authorize a third-party LicenseRef.
        first_party = package_id in {
            "SPDXRef-Package-Firmware", "SPDXRef-PROJECT-espectre-native",
            "SPDXRef-PROJECT-espectre-matter", "SPDXRef-COMPONENT-espectre", "SPDXRef-COMPONENT-main",
        } and package.get("licenseDeclared") == f"GPL-3.0-only OR {OWN_LICENSE}"
        expressions = {
            package.get(key, "NOASSERTION") for key in ("licenseDeclared", "licenseConcluded")
        } - {"NOASSERTION", "NONE", ""}
        label = f"{package['name']} ({package_id})"
        if not expressions:
            unknown.append(label)
        for expression in sorted(expressions):
            if not license_allowed(expression, frontend, first_party=first_party):
                errors.append(f"{label}: {expression}")
    return errors, unknown


def check_vulnerabilities(sbom_path: Path, timeout: int) -> dict:
    with tempfile.TemporaryDirectory(prefix="espectre-audit-") as directory:
        report_path = Path(directory) / "vulnerabilities.json"
        result = subprocess.run(
            [sys.executable, "-m", "esp_idf_sbom", "--no-progress", "check",
             "--format", "json", "--output-file", str(report_path), str(sbom_path)],
            capture_output=True, text=True, timeout=timeout,
        )
        if result.returncode not in (0, 1):
            raise RuntimeError(f"Vulnerability scanner exited {result.returncode}: {result.stderr[-8000:]}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        records = report["records"]
        if any(record["vulnerable"] not in {"YES", "MAYBE", "NO", "EXCLUDED", "SKIPPED"} for record in records):
            raise ValueError("Vulnerability scanner returned an unrecognized assessment")
        if not records or not any(record["vulnerable"] != "SKIPPED" for record in records):
            raise ValueError("Vulnerability scanner has no package coverage")
        findings = [record for record in records if record["vulnerable"] in {"YES", "MAYBE"}]
        if result.returncode == 1 and not findings:
            raise ValueError("Scanner failed without vulnerability details")
        return {"findings": findings, "skipped": len({
            record["pkg_name"] for record in records if record["vulnerable"] == "SKIPPED"
        })}


def vulnerability_identity(package: dict) -> str:
    """Return the SPDX fields that affect an ESP-IDF vulnerability lookup."""
    references = [
        {
            "referenceCategory": reference.get("referenceCategory", ""),
            "referenceType": reference.get("referenceType", ""),
            "referenceLocator": reference.get("referenceLocator", ""),
        }
        for reference in package.get("externalRefs", [])
        if (reference.get("referenceCategory") == "SECURITY"
            and reference.get("referenceType") == "cpe23Type")
        or reference.get("referenceType") in {"purl", "repository"}
    ]
    return json.dumps(
        {
            "name": package.get("name", ""),
            "versionInfo": package.get("versionInfo", ""),
            "supplier": package.get("supplier", ""),
            "originator": package.get("originator", ""),
            "downloadLocation": package.get("downloadLocation", ""),
            "summary": package.get("summary", ""),
            "comment": package.get("comment", ""),
            "externalRefs": sorted(references, key=lambda reference: json.dumps(reference, sort_keys=True)),
        },
        sort_keys=True,
    )


def build_vulnerability_union(frontend: str, sboms: list[dict]) -> dict:
    """Merge target SBOM dependency graphs while deduplicating scan inputs."""
    if not sboms:
        raise ValueError("No SBOMs are available for the vulnerability audit")

    document = copy.deepcopy(sboms[0])
    union_root = "SPDXRef-Package-Audit-Union"
    packages = [{
        "SPDXID": union_root,
        "name": f"ESPectre {frontend} firmware audit union",
        "versionInfo": "NOASSERTION",
        "downloadLocation": "NOASSERTION",
        "filesAnalyzed": False,
        "licenseDeclared": "NOASSERTION",
        "licenseConcluded": "NOASSERTION",
        "copyrightText": "NOASSERTION",
    }]
    package_ids: dict[str, str] = {}
    relationships: set[tuple[str, str]] = set()
    roots: set[str] = set()

    for sbom in sboms:
        source_ids: dict[str, str] = {}
        for package in sbom["packages"]:
            identity = vulnerability_identity(package)
            package_id = package_ids.get(identity)
            if package_id is None:
                package_id = "SPDXRef-UNION-" + hashlib.sha256(identity.encode()).hexdigest()[:20]
                package_ids[identity] = package_id
                merged = copy.deepcopy(package)
                merged["SPDXID"] = package_id
                packages.append(merged)
            source_ids[package["SPDXID"]] = package_id
        for relationship in sbom["relationships"]:
            if relationship.get("relationshipType") == "DESCRIBES" and relationship.get("spdxElementId") == "SPDXRef-DOCUMENT":
                root = source_ids.get(relationship.get("relatedSpdxElement", ""))
                if root:
                    roots.add(root)
            elif relationship.get("relationshipType") == "DEPENDS_ON":
                source = source_ids.get(relationship.get("spdxElementId", ""))
                destination = source_ids.get(relationship.get("relatedSpdxElement", ""))
                if source and destination:
                    relationships.add((source, destination))

    document["name"] = f"ESPectre {frontend} firmware audit union"
    document["documentNamespace"] = f"https://espectre.dev/sbom/audit/{frontend}"
    document["packages"] = packages
    document["documentDescribes"] = [union_root]
    document["relationships"] = [
        {"spdxElementId": "SPDXRef-DOCUMENT", "relationshipType": "DESCRIBES", "relatedSpdxElement": union_root},
        *[
            {"spdxElementId": union_root, "relationshipType": "DEPENDS_ON", "relatedSpdxElement": root}
            for root in sorted(roots)
        ],
        *[
            {"spdxElementId": source, "relationshipType": "DEPENDS_ON", "relatedSpdxElement": destination}
            for source, destination in sorted(relationships)
        ],
    ]
    return document


def write_reports(directory: Path, frontend: str, errors: list[str], unknown: list[str],
                  result: dict, failure: str | None, *, expected_targets: list[str] | None = None,
                  available_targets: list[str] | None = None) -> None:
    """Keep incomplete scans distinct from successful scans with no findings."""
    directory.mkdir(parents=True, exist_ok=True)
    sarif_path = directory / "results.sarif"
    sarif_path.unlink(missing_ok=True)
    summary = [f"### Firmware audit: {frontend}", "",
               "Status: **incomplete**" if failure else "Status: **completed**", "",
               f"- License policy violations: {len(errors)}",
               f"- Unidentified licenses: {len(unknown)}", ""]
    if expected_targets is not None:
        available = available_targets or []
        missing = sorted(set(expected_targets) - set(available))
        summary += [
            f"- Target coverage: {len(available)}/{len(expected_targets)}",
            f"- Audited targets: {', '.join(available) or 'none'}",
        ]
        if missing:
            summary.append(f"- Missing audit inputs: {', '.join(missing)}")
        summary.append("")
    summary.extend(f"- {error}" for error in errors)
    summary.extend(f"- License requires review: {package}" for package in unknown)
    if failure:
        summary += ["", f"Audit error: {failure}"]
    summary += ["", "Missing metadata is not evidence of safety or license compatibility.", ""]
    (directory / "summary.md").write_text("\n".join(summary), encoding="utf-8")
    # Do not upload a partial analysis: it could close real alerts for missing targets.
    if failure:
        return
    results, rules = [], {}
    manifest = ("src/cpp/frontend/esphome/components/espectre/idf_component.yml"
                if frontend == "esphome" else f"src/cpp/frontend/{frontend}/espectre/idf_component.yml")
    for finding in result["findings"]:
        cve = finding["cve_id"]
        rule_id = f"{cve}/{finding['pkg_name']}"
        properties = {"tags": ["security", "dependencies"]}
        score = finding.get("cvss_base_score")
        if score and 0 <= float(score) <= 10:
            properties["security-severity"] = str(score)
        rules[rule_id] = {
            "id": rule_id, "shortDescription": {"text": f"{cve} in {finding['pkg_name']}"},
            "helpUri": f"https://nvd.nist.gov/vuln/detail/{cve}", "properties": properties,
        }
        identity = f"{frontend}:{finding['pkg_name']}:{cve}"
        results.append({
            "ruleId": rule_id, "level": "warning",
            "message": {"text": f"{finding['pkg_name']} {finding.get('pkg_version', '')}: {cve}. "
                        f"Applicability: {finding.get('vulnerable', 'unknown')}. "
                        "The location identifies the frontend manifest, not the vulnerable source line."},
            "locations": [{"physicalLocation": {"artifactLocation": {"uri": manifest}}}],
            "partialFingerprints": {"dependency/v1": hashlib.sha256(identity.encode()).hexdigest()},
        })
    sarif = {"version": "2.1.0", "$schema": "https://json.schemastore.org/sarif-2.1.0.json", "runs": [{
        "tool": {"driver": {"name": "ESPectre firmware dependencies", "rules": list(rules.values())}},
        "results": results, "invocations": [{"executionSuccessful": True}],
    }]}
    sarif_path.write_text(json.dumps(sarif, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sbom", type=Path, action="extend", nargs="+", default=[])
    parser.add_argument("--frontend", choices=("esphome", "native", "matter"), required=True)
    parser.add_argument("--expected-target", action="append", default=[],
                        help="Target label expected to provide one SBOM input.")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--timeout", type=int, default=600, help="Maximum seconds for the NVD scan.")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    default_directory = (args.sbom[0].parent if args.sbom else Path.cwd()) / "audit" / args.frontend
    directory = args.output_dir or default_directory
    errors, unknown = [], []
    result = {"findings": [], "skipped": 0}
    failures = []
    inputs: dict[str, dict] = {}
    try:
        for path in args.sbom:
            target = path.name.removesuffix(".spdx.json")
            if target in inputs:
                raise ValueError(f"Duplicate SBOM input for target {target}")
            inputs[target] = json.loads(path.read_text(encoding="utf-8"))
        if not inputs:
            raise ValueError("No SBOM inputs were provided")
        for sbom in inputs.values():
            license_errors, license_unknown = check_licenses(sbom, args.frontend)
            errors.extend(license_errors)
            unknown.extend(license_unknown)
        errors = sorted(set(errors))
        unknown = sorted(set(unknown))
        with tempfile.TemporaryDirectory(prefix="espectre-audit-union-") as union_directory:
            union_path = Path(union_directory) / f"{args.frontend}-union.spdx.json"
            union_path.write_text(json.dumps(build_vulnerability_union(args.frontend, list(inputs.values()))), encoding="utf-8")
            result = check_vulnerabilities(union_path, args.timeout)
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
        failures.append(str(error))
    missing_targets = sorted(set(args.expected_target) - set(inputs))
    if missing_targets:
        failures.append("Missing audit input for target(s): " + ", ".join(missing_targets))
    failure = "; ".join(failures) if failures else None
    write_reports(directory, args.frontend, errors, unknown, result, failure,
                  expected_targets=args.expected_target or None,
                  available_targets=sorted(inputs))
    blocked = failure is not None or bool(errors)
    status = "incomplete" if failure else "violations" if errors else "completed"
    # Detailed untrusted package metadata stays in the report, outside workflow commands.
    level = "error" if blocked else "warning"
    if failure or errors or unknown:
        print(f"::{level}::Firmware license audit {status}; review the audit summary.")
    print(f"Firmware license audit: {status}; {len(errors)} license violations, "
          f"{len(unknown)} unidentified licenses. Vulnerability findings are published through SARIF.")
    return 2 if failure else 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
