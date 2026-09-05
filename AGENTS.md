# ESPectre Agent Rules

## Scope And Routing

- Keep changes surgical, prefer direct implementations, match neighboring style, and write code, comments, documentation, filenames, and commit messages in English. Use the Oxford comma in project documentation and user-facing text.
- Treat `src/cpp/` as production firmware, `src/python/micro_espectre/` as the MicroPython device path, `src/python/espectre_cli/` as host CLI code, and `tools/` as host-side analysis and maintenance code.
- Update the existing topic owner by default. Create a new Markdown document only when the user requests a persistent record or repository policy requires a new ADR or review record.
- Before modifying or reviewing a specialized subtree, read each `AGENTS.md` from the repository root down to that subtree once. Do not load instructions for unrelated subtrees.
- Specialized rules live in `.github/AGENTS.md`, `src/cpp/AGENTS.md`, `src/python/AGENTS.md`, `test/AGENTS.md`, `docs/AGENTS.md`, `docs/web/AGENTS.md`, and `tools/AGENTS.md`.

## Context Discipline

- Build a task-scoped file list before reading implementation details. Use the narrowest implementation, schema, test, or document that owns the requested behavior.
- For repository changes or reviews, start with compact state such as `git status --short`, `git diff --stat`, and `git diff --name-only`. On a dirty worktree, inspect diffs only for paths in scope; never dump the entire diff as an exploratory step.
- Treat files over 500 lines or 40 KB as large. Locate symbols or Markdown headings with `rg`, then read bounded ranges of at most 160 lines. Do not use unbounded `cat`, `nl`, or `sed` on large files.
- Keep normal command output below roughly 120 lines or 20 KB. Do not combine several large file reads, diffs, or verbose commands into one tool call; parallelize only when every result is independently small.
- Read each owning document once per task and only at the relevant headings. Do not scan the entire documentation map unless ownership is ambiguous, and do not reread settled files unless new evidence invalidates the current understanding.
- Exclude build trees, vendored code, generated pages, datasets, and generated reports from discovery unless the task explicitly targets them.
- For broad reviews, group files by subsystem and finish one group before loading the next. Do not expand a review into implementation, hardware diagnosis, release work, or Git operations unless the user requests that phase.
- Prefer concise test output. Run the narrow owner check with quiet or short-traceback options where supported, and rerun only failures verbosely. For noisy builds or hardware workflows, keep successful output to the summary and expose bounded diagnostic tails on failure.
- Redirect verbose build, test, benchmark, and hardware output to a temporary log. On success, report only the exit status and summary. On failure, read at most the final 120 lines, then inspect narrower sections only as needed.
- Never run an unbounded serial monitor in the foreground. Use a fixed duration, an event filter, or a temporary log, and read only the relevant interval.

## Source Of Truth

- Start from the implementation owner and its local README. Use the documentation map in `README.md` only when ownership is unclear.
- Use `./espectre --help` for current CLI syntax and the canonical registry, schema, or generated artifact for machine-consumed contracts.
- Treat large ledgers such as `docs/FEATURES.md` and `docs/LITERATURE.md` as targeted references: search for the relevant entry or heading instead of reading them in full.
- Verify current-state documentation against implementation, runtime schemas, and generated artifacts. Distinguish deployed, partial, and target behavior.

## Production Boundaries

- Preserve **one message model, multiple transports**: MQTT, Direct HTTP, and future transports carry the same canonical JSON contract and application version; transport framing and delivery policy stay outside that model.
- Treat `docs/API.md`, `docs/DISCOVERY.md`, and the canonical protocol registry or schema as the owners of messages, operations, discovery metadata, and version semantics. Do not create transport-specific envelopes, aliases, or constants.
- Require cross-transport parity for serialized messages, validation, and capability schemas; engine-level semantic parity alone is insufficient.
- Enforce the dependency direction `Frontend -> Runtime -> Core`; lower layers must not include, query, or call higher layers.
- Do not modify the CSI data format without updating both `C++` and Python implementations.
- Prototype sensing and detector changes in Python, validate them, and then port production behavior to the shared `C++` layers and relevant frontends. Keep shared detection and calibration algorithms aligned between `C++` and Python.
- Do not hardcode WiFi or MQTT credentials. Use local configuration files that remain outside production defaults.

## Workflow And Validation

- Preserve unrelated user changes in dirty worktrees and inspect the final scoped diff for accidental edits.
- Run the narrowest relevant checks first, followed only by parity, integration, generated-artifact, or frontend gates required by the changed surface.
- When tests fail, investigate the root cause. Never skip, disable, or weaken tests to make them pass, and ask before changing a supported behavior expectation.
- Do not claim that a check passed unless it ran successfully. Report checks not run with the exact command and blocker.
- Update the owning documentation when public behavior, configuration, protocol, or operator workflow changes.
- Do not mutate GitHub state, including commenting, closing, merging, labeling, pushing, or changing releases, unless the user explicitly requests it.

## Environment, Contributions, And Dependencies

- Prefer the repository `./espectre` wrapper for local workflows. Use the repository virtual environment for direct Python commands when available; the wrapper uses `.venv` automatically.
- ESP-IDF frontend builds require the ESP-IDF environment that provides `idf.py`.
- Use Conventional Commits with an imperative, concrete, lower-case subject of at most 72 characters. Commits intended for contribution require a valid `Signed-off-by` trailer; prefer `git commit -s`.
- Use `develop` as the default PR target; `main` is release-only. Do not bypass branch protection, push directly to `main`, force-push protected branches, or merge with failing required checks.
- Keep pull requests free of merge commits, update their branches by rebasing, and integrate them with Rebase and merge. Do not use merge commits or squash merges.
- Use `--force-with-lease` only on a pull request source branch when a user-authorized rebase or amendment requires rewriting published commits.
- ESPectre is GPLv3 with separately offered commercial licenses. Do not add dependencies incompatible with GPLv3, and preserve the dual-distribution constraints described by the specialized subtree rules.
- Declare Python dependencies in `requirements.txt` for the base workflow or `requirements-ml.txt` for ML and training extras.
