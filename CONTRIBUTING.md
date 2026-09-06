# Contributing

Contributions are accepted through pull requests against `develop`. Use this guide for the development environment, tests, datasets, documentation, and review expectations.

## Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating, you agree to uphold this code. Please report unacceptable behavior to contact@espectre.dev.

## First-Time Contributors

Before opening your first pull request:

1. Review [closed `good first issue` contributions](https://github.com/francescopace/espectre/issues?q=is%3Aissue+is%3Aclosed+label%3A%22good+first+issue%22) for examples.
2. Read the setup and test sections relevant to your change.
3. Ask implementation questions on the issue that owns the work.

## Ways to Contribute

| Type | Description | Skill Level |
|------|-------------|-------------|
| **Bug Reports** | Report issues with clear reproduction steps | Beginner |
| **Documentation** | Improve guides, fix typos, add examples | Beginner |
| **Data Collection** | Contribute labeled CSI datasets | Beginner |
| **Code Review** | Review Pull Requests | Intermediate |
| **Bug Fixes** | Fix reported issues | Intermediate |
| **New Features** | Implement roadmap items | Advanced |
| **Algorithm R&D** | Develop new detection algorithms | Advanced |

## Development Setup

### Prerequisites

- Python 3.14 (required for the maintained host and ML workflows)
- ESP32 device (S3/C6 recommended)
- Home Assistant (optional, for testing ESPHome integration)

### Environment Setup

```bash
# Clone the repository
git clone https://github.com/francescopace/espectre.git
cd espectre

# Create and activate virtual environment
python3.14 -m venv .venv
source .venv/bin/activate  # On macOS/Linux
# .venv\Scripts\activate   # On Windows

# Install dependencies
pip install -r requirements.txt

# Optional: ML training extras
# Install these only if you need to retrain or inspect the exported model.
pip install -r requirements-ml.txt
```

### Source Layout

The production firmware code lives under `src/`:

- `src/cpp/core/` for reusable detectors, temporal CSI admission, and math
- `src/cpp/runtime/` for the shared runtime contract and `src/cpp/runtime/esp_idf/` for the current ESP-IDF-specific orchestration
- `src/cpp/frontend/esphome/`, `native/`, and `matter/` for the published firmware adapters
- `src/python/micro_espectre/` for the MicroPython device path

See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for the layering and dependency direction.

### Running Tests

```bash
# C++ tests (host-side core/runtime/frontend suite)
cmake -S test/cpp -B test/cpp/build
cmake --build test/cpp/build
ctest --test-dir test/cpp/build --output-on-failure

# C++ tests with coverage
./test/cpp/run_coverage.sh

# Python tests (device runtime, CLI, tools, and validation)
.venv/bin/pytest test/python -v

# Python tests with coverage
./test/python/run_coverage.sh

# Web tests with coverage
./test/web/run_coverage.sh

# Static website preview
python -m http.server 8080 --directory docs/web
```

Direct single-config CMake builds and `run_all_tests.sh` default to `RelWithDebInfo` with assertions enabled; `run_coverage.sh` uses an instrumented `Debug` build.

Python test auto-parallelism is capped at four workers because replay-heavy tests become slower under higher process counts. Set `PYTEST_XDIST_AUTO_NUM_WORKERS` to a positive integer to override the cap.

The coverage helper is a Bash script used on macOS/Linux and CI. On Windows, run the CMake/CTest commands above for the host-side C++ suite, or use WSL/Git Bash if you specifically need the coverage script.

## Code Contributions

### Branching Model

ESPectre uses a simple branching model:

- **`develop`**: Active development branch. All PRs should target this branch.
- **`main`**: Stable release branch. Receives releases from `develop` with linear history.

Use **Rebase and merge** to integrate pull requests. Merge commits and squash merges are disabled, and `main` and `develop` require linear history. Pull requests must not contain merge commits, including updates to Dependabot branches.

To update your pull request branch, rebase it onto its target branch. In GitHub, choose **Update with rebase**, not **Update with merge commit**. Locally, run the following from your pull request branch:

```bash
git fetch origin
git rebase origin/develop
git push --force-with-lease
```

Replace `develop` with the pull request's target branch when appropriate. Rewrite only your pull request branch; never force-push `main` or `develop`.

### Workflow

1. **Fork** the repository on GitHub
2. **Clone** your fork locally
3. **Create a branch** from `develop`:
   ```bash
   git checkout develop
   git pull --ff-only origin develop
   git checkout -b feature/your-feature-name
   ```
4. **Make changes** with tests and documentation
5. **Run tests** to ensure nothing is broken
6. **Commit** with clear messages (see [Commit Guidelines](#commit-guidelines))
7. **Push** to your fork
8. **Open a Pull Request** to the `develop` branch

### Commit Guidelines

Use clear, descriptive commit messages:

```
<type>: <short description>

<optional body with more details>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `test`: Adding or updating tests
- `refactor`: Code refactoring (no functional change)
- `perf`: Performance improvement
- `chore`: Maintenance tasks

**Examples:**
```
feat: add low-pass filter for noise reduction
fix: correct calibration for edge cases
docs: update TUNING.md with filter examples
test: add unit tests for Hampel filter
```

### DCO Sign-off (required)

This repository enforces the Developer Certificate of Origin (DCO) in CI. Every human-authored commit in a pull request must include a valid `Signed-off-by` trailer. Dependabot-authored commits are exempt from DCO sign-off, but all commits must satisfy the linear-history check.

### CLA (required once)

ESPectre is dual-licensed (see [LICENSING.md](LICENSING.md)), so the project also requires a one-time [Contributor License Agreement](CLA.md) signature. To sign, add your GitHub login to `.github/cla-signatures.json` in your first pull request, as described in [CLA.md](CLA.md); the CLA check on the pull request verifies the entry. One signature covers all your past and future contributions while you retain ownership of your work.

Use:

```bash
git commit -s -m "type: short description"
```

For existing commits, add the sign-off and force-push your branch:

```bash
git commit --amend -s
git push --force-with-lease
```

### Code Style

#### C++

- Keep shared `core` and `runtime` code frontend-agnostic
- All code and comments in English

**File Header:**
```cpp
/*
 * ESPectre - [Component Name]
 *
 * [Brief description]
 *
 * Author: [your name] <[your email]>
 * SPDX-License-Identifier: GPL-3.0-only
 * Commercial licensing available under separate agreement; see LICENSING.md.
 */
```

#### Python

- CPython host tooling under `src/python/espectre_cli/`, `tools/`, and `test/python/`
- MicroPython device path under `src/python/micro_espectre/`
- All code and comments in English

**File Header:**
```python
# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""
ESPectre - [Module Name]

[Brief description]

Author: [your name] <[your email]>
"""
```

Executable tool scripts may keep a `#!/usr/bin/env python3` shebang above the header.
Third-party files retain their upstream license notices; do not replace them with the ESPectre header.

### Pull Request Guidelines

- **Target branch**: Always `develop` (not `main`)
- **Title**: Clear, descriptive title
- **Description**: Explain what and why
- **Tests**: Include tests for new functionality
- **Documentation**: Update relevant docs
- **Single focus**: One feature/fix per PR

### Quality Standards

| Requirement | Target |
|-------------|--------|
| Test coverage | Run `./test/cpp/run_coverage.sh` and avoid unexplained regressions in the relevant layer |
| CI passing | All checks must pass |
| Documentation | Features require docs |
| Code review | At least one approval |

## Data Contributions

Help build a diverse CSI dataset for ML training. For v3, the most useful data improves room-state robustness across real homes, routers, and ESP32 boards.

### How to Contribute Data

1. **Collect data** following [ML_DATA_COLLECTION.md](docs/ML_DATA_COLLECTION.md)
2. **Ensure quality**:
   - At least 10 samples per label
   - 30+ seconds per sample
   - Quiet room for `static_presence` or `empty` recordings
3. **Document your setup**:
   - ESP32 model (S3, C6, etc.)
   - Distance from router
   - Room type (living room, office, etc.)
   - Any notable characteristics
4. **Submit via Pull Request**:
   - Add your data to `data/<label>/`
   - Include a brief description in the PR

### Priority Labels

We're particularly looking for room-state datasets:

| Priority | Label | Description | Use Case |
|----------|-------|-------------|----------|
| High | `empty` | Empty room, no movement | Hard-negative coverage and false-positive reduction |
| High | `static_presence` | Person present but mostly still | Occupancy-like stillness coverage |
| High | `motion` | Walking or ordinary room movement | Recall across homes, routers, and board variants |

Gesture recognition, HAR, and people counting are useful future research tracks, but they are not the primary v3 dataset request.

### Data Privacy

- CSI captures do not contain images or audio, but they are not inherently anonymous: persistent device identifiers, timestamps, contributor names, environment labels, packet-level radio metadata, and inferred presence or activity can still be identifying or sensitive
- Collect data only in spaces where you have the right to do so, inform affected people, and follow applicable privacy laws
- Before opening a pull request, inspect the `.npz` metadata and `data/dataset_info.json`, remove unnecessary identifying details, and use a pseudonymous contributor value when attribution does not require your real name
- Do not include Wi-Fi credentials, SSIDs, BSSIDs, local IP addresses, serial logs, or unrelated personal information
- You retain ownership of your contributions
- All contributions will be credited

## Documentation

Update the document that owns the behavior you changed.

### Types of Documentation

| Type | Location | Description |
|------|----------|-------------|
| **README** | `README.md` | Project overview, quick start |
| **Setup Guide** | `docs/SETUP.md` | Shared setup hub and frontend chooser |
| **Tuning Guide** | `docs/TUNING.md` | Parameter optimization and tuning rationale |
| **Algorithms** | `docs/ALGORITHMS.md` | Scientific documentation |
| **Frontend READMEs** | `src/cpp/frontend/*/README.md` | Frontend-specific setup, workflow, protocol, and surface documentation |
| **API Docs** | Code comments | Function/class documentation |

### Documentation Guidelines

- Start with the reader's task or the fact they need. Avoid openings such as "This guide provides..." when the title and first section already establish the scope.
- Name the component, action, or evidence. Prefer direct verbs such as "is", "has", and "does" to vague phrases such as "serves as", "helps ensure", or "highlights".
- Do not claim importance, maturity, broad applicability, or wider impact without repository evidence.
- Use contrast when it changes a decision. Avoid repeated formulas such as "not X but Y", "not just X but Y", and "X rather than Y".
- Use bullets and tables for real mappings, comparisons, or procedures. Do not turn a short explanation into a list to make it appear exhaustive.
- Remove duplicate recaps and generic future-work conclusions. Link to the topic owner when another document already has the detail.
- Distinguish deployed, partial, and target behavior explicitly.
- Test every command and code sample you add.
- Keep one source of truth per topic: `docs/SETUP.md` for the shared hub, frontend READMEs for frontend-specific workflows and surfaces, `docs/TUNING.md` for tuning guidance, and `docs/ALGORITHMS.md` for theory.

## Reporting Issues

### Before Reporting

1. **Search existing issues** to avoid duplicates
2. **Read [SETUP.md](docs/SETUP.md)** and the relevant frontend README for current operator behavior
3. **Try the latest version** from `develop` branch

### Bug Reports

Include:
- **Description**: Clear description of the bug
- **Steps to reproduce**: Numbered steps
- **Expected behavior**: What should happen
- **Actual behavior**: What actually happens
- **Environment**:
  - ESP32 model (S3, C6, etc.)
  - ESPectre version
  - Home Assistant version (if applicable)
  - Relevant configuration

### Feature Requests

Include:
- **Use case**: Why is this feature needed?
- **Proposed solution**: How might it work?
- **Alternatives**: Other approaches considered

## Community

### Getting Help

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: Questions and design discussions

### Stay Updated

- **Watch** the repository for updates
- **Star** if you find it useful
- **Share** with others who might benefit

## Recognition

All contributors are recognized in:
- Pull Request acknowledgments
- Release notes for significant contributions
- Data contributors credited in dataset documentation

All contributions must also be certified under the Developer Certificate of Origin (DCO) by adding the `Signed-off-by` trailer to each commit, and covered by a one-time [CLA](CLA.md) signature. The DCO certifies that you have the right to submit the contribution; the CLA lets the project distribute it under both licensing tracks described in [LICENSING.md](LICENSING.md).

See [LICENSE](LICENSE) and [LICENSING.md](LICENSING.md) for details.
