#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Summarize informational SARIF analyses without treating missing reports as clean."""

import argparse
import json
import os
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    try:
        report = json.loads(args.report.read_text(encoding='utf-8'))
        runs = report['runs']
        if not runs or any(not invocation.get('executionSuccessful', True)
                           for run in runs for invocation in run.get('invocations', [])):
            raise ValueError('analysis did not complete')
        count = sum(len(run.get('results', [])) for run in runs)
        message = f'{args.name}: {count} quality findings. Informational; does not block merging.'
    except (OSError, ValueError, KeyError):
        message = f'{args.name}: analysis incomplete; inspect the scanner step. No clean result is available.'
        print('::warning::Quality analysis did not produce a complete report.')
    print(message)
    if summary_path := os.environ.get('GITHUB_STEP_SUMMARY'):
        with Path(summary_path).open('a', encoding='utf-8') as summary:
            summary.write(f'### {args.name}\n\n{message}\n')


if __name__ == '__main__':
    main()
