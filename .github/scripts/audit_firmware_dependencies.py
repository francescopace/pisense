#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Report firmware dependency findings, with an optional release gate."""

from __future__ import annotations

import argparse
import hashlib
import os
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


def write_reports(directory: Path, frontend: str, errors: list[str], unknown: list[str],
                  result: dict, failure: str | None) -> None:
    """Keep incomplete scans distinct from successful scans with no findings."""
    directory.mkdir(parents=True, exist_ok=True)
    sarif_path = directory / "results.sarif"
    sarif_path.unlink(missing_ok=True)
    summary = [f"### Firmware audit: {frontend}", "",
               "Status: **incomplete**" if failure else "Status: **completed**", "",
               f"- License policy violations: {len(errors)}",
               f"- Unidentified licenses: {len(unknown)}",
               f"- Vulnerability findings: {len(result['findings'])}",
               f"- Packages without vulnerability identifiers: {result['skipped']}", ""]
    summary.extend(f"- {error}" for error in errors)
    summary.extend(f"- License requires review: {package}" for package in unknown)
    for finding in result["findings"]:
        summary.append(f"- {finding['pkg_name']} {finding.get('pkg_version', '')}: "
                       f"{finding['cve_id']} ({finding.get('cvss_base_severity') or 'unknown severity'}, "
                       f"assessment {finding.get('vulnerable', 'unknown')})")
    if failure:
        summary += ["", f"Scanner error: {failure}"]
    summary += ["", "Missing metadata is not evidence of safety or license compatibility.", ""]
    (directory / "summary.md").write_text("\n".join(summary), encoding="utf-8")
    # Do not upload an empty analysis on scanner failure: it could close real alerts.
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


def release_blocked(errors: list[str], findings: list[dict]) -> bool:
    # NVD applicability still requires human triage; MAYBE remains informational.
    return bool(errors) or any(
        finding.get("vulnerable") == "YES"
        and (finding.get("cvss_base_severity") or "UNKNOWN").upper() in {"HIGH", "CRITICAL", "UNKNOWN"}
        for finding in findings
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sbom", type=Path, required=True)
    parser.add_argument("--frontend", choices=("esphome", "native", "matter"), required=True)
    parser.add_argument("--policy", choices=("report", "release"),
                        default=os.environ.get("ESPECTRE_AUDIT_POLICY", "report"))
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--timeout", type=int, default=600, help="Maximum seconds for the NVD scan.")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    directory = args.output_dir or args.sbom.parent / "audit" / args.frontend
    errors, unknown = [], []
    result = {"findings": [], "skipped": 0}
    failure = None
    try:
        sbom = json.loads(args.sbom.read_text(encoding="utf-8"))
        errors, unknown = check_licenses(sbom, args.frontend)
        result = check_vulnerabilities(args.sbom, args.timeout)
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
        failure = str(error)
    write_reports(directory, args.frontend, errors, unknown, result, failure)
    blocked = args.policy == "release" and (failure is not None or release_blocked(errors, result["findings"]))
    status = "incomplete" if failure else "findings" if errors or result["findings"] else "completed"
    # Detailed untrusted package metadata stays in the report, outside workflow commands.
    level = "error" if blocked else "warning"
    if failure or errors or unknown or result["findings"] or result["skipped"]:
        print(f"::{level}::Firmware audit {status}; review the audit summary.")
    print(f"Firmware audit ({args.policy}): {status}; {len(errors)} license violations, "
          f"{len(unknown)} unidentified licenses, {len(result['findings'])} vulnerability findings.")
    return (2 if failure else 1) if blocked else 0


if __name__ == "__main__":
    raise SystemExit(main())
