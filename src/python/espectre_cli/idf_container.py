# SPDX-License-Identifier: GPL-3.0-only
# Commercial licensing available under separate agreement; see LICENSING.md.
"""Docker execution support for ESP-IDF firmware builds."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Callable


IDF_VERSION = "5.5.5"
IDF_DOCKER_IMAGE_DIGEST = "a9231d0697ab8f7517cc072e93b7c83e04907bfbfba80b6440d7dbbf90665cf2"
IDF_DOCKER_IMAGE = f"espressif/idf:v{IDF_VERSION}@sha256:{IDF_DOCKER_IMAGE_DIGEST}"
DOCKER_PULL_POLICIES = ("ask", "missing", "never")


class DockerBackendError(RuntimeError):
    """Report that the Docker build backend cannot be prepared."""


def docker_executable() -> str | None:
    """Return the Docker CLI path when it is installed."""
    return shutil.which("docker")


def docker_daemon_is_running(docker: str) -> bool:
    """Return whether the Docker CLI can reach a running engine."""
    try:
        result = subprocess.run(
            [docker, "info", "--format", "{{.ServerVersion}}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0


def docker_image_is_present(docker: str, image: str = IDF_DOCKER_IMAGE) -> bool:
    """Return whether the pinned ESP-IDF image is already cached locally."""
    try:
        result = subprocess.run(
            [docker, "image", "inspect", image],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0


def _interactive_terminal() -> bool:
    return sys.stdin.isatty()


def ensure_docker_backend(
    pull_policy: str,
    *,
    image: str = IDF_DOCKER_IMAGE,
    input_fn: Callable[[str], str] = input,
) -> str:
    """Prepare Docker and the pinned image, prompting only for required user actions."""
    if pull_policy not in DOCKER_PULL_POLICIES:
        raise ValueError(f"Unsupported Docker pull policy: {pull_policy}")

    docker = docker_executable()
    if docker is None:
        raise DockerBackendError(
            f"Docker is not installed. Install Docker, or install ESP-IDF {IDF_VERSION} for local builds."
        )

    while not docker_daemon_is_running(docker):
        if not _interactive_terminal():
            raise DockerBackendError(
                "Docker is installed, but its engine is not running. Start Docker and rerun the command."
            )
        try:
            response = input_fn(
                "Docker is installed, but its engine is not running. Start Docker, then press Enter "
                "to retry, or enter q to quit: "
            )
        except EOFError as exc:
            raise DockerBackendError("Docker must be running to use the container build backend.") from exc
        if response.strip().lower() in {"q", "quit", "n", "no"}:
            raise DockerBackendError("Docker must be running to use the container build backend.")

    if docker_image_is_present(docker, image):
        return docker

    if pull_policy == "never":
        raise DockerBackendError(
            "The pinned ESP-IDF Docker image is not cached. Rerun with --pull missing to download it."
        )

    if pull_policy == "ask":
        if not _interactive_terminal():
            raise DockerBackendError(
                "The pinned ESP-IDF Docker image is not cached. Rerun with --pull missing to download it."
            )
        try:
            response = input_fn(
                "The ESP-IDF build image is not cached and may require a multi-gigabyte download. "
                "Download it now? [y/N] "
            )
        except EOFError as exc:
            raise DockerBackendError("The ESP-IDF Docker image download was not approved.") from exc
        if response.strip().lower() not in {"y", "yes"}:
            raise DockerBackendError("The ESP-IDF Docker image download was not approved.")

    try:
        subprocess.run([docker, "pull", image], check=True)
    except OSError as exc:
        raise DockerBackendError("The Docker CLI could not be started.") from exc
    except subprocess.CalledProcessError as exc:
        raise DockerBackendError(f"Docker could not download the ESP-IDF build image (exit {exc.returncode}).") from exc
    return docker


def build_toolchain_docker_command(
    docker: str,
    *,
    frontend: str,
    workdir: Path,
    commands: list[list[str]],
    repo_root: Path,
    environment: dict[str, str] | None = None,
    image: str = IDF_DOCKER_IMAGE,
) -> list[str]:
    """Build a Docker command for toolchain work inside the repository."""
    resolved_root = repo_root.resolve()
    resolved_workdir = workdir.resolve()
    try:
        workdir_relative = resolved_workdir.relative_to(resolved_root)
    except ValueError as exc:
        raise DockerBackendError(
            f"ESP-IDF work directory is outside the repository: {resolved_workdir}"
        ) from exc

    container_home_relative = Path(".cache") / "build" / f"{frontend}-home"
    container_home = resolved_root / container_home_relative
    root_managed_components = container_home / "root_managed_components"
    ccache_dir = container_home / "ccache"
    container_home.mkdir(parents=True, exist_ok=True)
    root_managed_components.mkdir(parents=True, exist_ok=True)
    ccache_dir.mkdir(parents=True, exist_ok=True)

    command = [docker, "run", "--rm"]
    if hasattr(os, "getuid") and hasattr(os, "getgid"):
        command.extend(["--user", f"{os.getuid()}:{os.getgid()}"])
    command.extend(
        [
            "-e",
            f"HOME=/work/{container_home_relative.as_posix()}",
            "-e",
            "IDF_CCACHE_ENABLE=1",
            "-e",
            f"CCACHE_DIR=/work/{container_home_relative.as_posix()}/ccache",
            "-e",
            "CCACHE_MAXSIZE=2G",
        ]
    )
    for key, value in (environment or {}).items():
        command.extend(["-e", f"{key}={value}"])
    command.extend(
        [
            "-v",
            f"{root_managed_components}:/opt/esp/root_managed_components",
            "-v",
            f"{resolved_root}:/work",
            "-w",
            f"/work/{workdir_relative.as_posix()}",
            image,
            "bash",
            "-lc",
            " && ".join(shlex.join(item) for item in commands),
        ]
    )
    return command


def build_docker_command(
    docker: str,
    *,
    frontend: str,
    app_path: Path,
    commands: list[list[str]],
    repo_root: Path,
    sdkconfig_defaults: str,
    image: str = IDF_DOCKER_IMAGE,
) -> list[str]:
    """Build the Docker command for a standard ESP-IDF frontend."""
    return build_toolchain_docker_command(
        docker,
        frontend=frontend,
        workdir=app_path,
        commands=commands,
        repo_root=repo_root,
        environment={"SDKCONFIG_DEFAULTS": sdkconfig_defaults},
        image=image,
    )


def run_idf_container(
    *,
    frontend: str,
    app_path: Path,
    commands: list[list[str]],
    repo_root: Path,
    sdkconfig_defaults: str,
    pull_policy: str,
    docker: str | None = None,
) -> None:
    """Run ESP-IDF build commands in the pinned Docker image."""
    run_toolchain_container(
        frontend=frontend,
        workdir=app_path,
        commands=commands,
        repo_root=repo_root,
        pull_policy=pull_policy,
        docker=docker,
        environment={"SDKCONFIG_DEFAULTS": sdkconfig_defaults},
        error_label="Docker ESP-IDF build",
    )


def run_toolchain_container(
    *,
    frontend: str,
    workdir: Path,
    commands: list[list[str]],
    repo_root: Path,
    pull_policy: str,
    docker: str | None = None,
    environment: dict[str, str] | None = None,
    error_label: str = "Docker ESP-IDF toolchain build",
) -> None:
    """Run arbitrary repository toolchain commands in the pinned IDF image."""
    docker = docker or ensure_docker_backend(pull_policy)
    command = build_toolchain_docker_command(
        docker,
        frontend=frontend,
        workdir=workdir,
        commands=commands,
        repo_root=repo_root,
        environment=environment,
    )
    try:
        subprocess.run(command, check=True)
    except OSError as exc:
        raise DockerBackendError("The Docker CLI could not be started.") from exc
    except subprocess.CalledProcessError as exc:
        raise DockerBackendError(f"{error_label} failed with exit code {exc.returncode}.") from exc
