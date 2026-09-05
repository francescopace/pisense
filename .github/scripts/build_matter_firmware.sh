#!/usr/bin/env bash
set -euo pipefail

: "${MATTER_TARGET:?MATTER_TARGET is required}"
: "${MATTER_OUTPUT:?MATTER_OUTPUT is required}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="build-container-${MATTER_TARGET}"
DOCKER_IMAGE="${MATTER_DOCKER_IMAGE:-espressif/idf:v5.5.5@sha256:a9231d0697ab8f7517cc072e93b7c83e04907bfbfba80b6440d7dbbf90665cf2}"
MATTER_HOME="${REPO_ROOT}/.cache/build/matter-home"
MATTER_ROOT_MANAGED_COMPONENTS="${MATTER_HOME}/root_managed_components"
MATTER_CCACHE="${MATTER_HOME}/ccache-${MATTER_TARGET}"
OUTPUT_DIR="$(dirname "${MATTER_OUTPUT}")"
MATTER_OUTPUT_IN_WORK="/work/${MATTER_OUTPUT#"${REPO_ROOT}"/}"
MATTER_SDKCONFIG_DEFAULTS="${MATTER_SDKCONFIG_DEFAULTS:-}"
ESPECTRE_GIT_VERSION="${ESPECTRE_GIT_VERSION:-$(python3 "${REPO_ROOT}/.github/scripts/detect_git_version.py")}"

mkdir -p "${MATTER_HOME}" "${MATTER_ROOT_MANAGED_COMPONENTS}" "${MATTER_CCACHE}" "${OUTPUT_DIR}"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e HOME="/work/.cache/build/matter-home" \
  -e ESPECTRE_AUDIT_POLICY="${ESPECTRE_AUDIT_POLICY:-report}" \
  -e ESPECTRE_GIT_VERSION="${ESPECTRE_GIT_VERSION}" \
  -e IDF_CCACHE_ENABLE=1 \
  -e CCACHE_DIR="/work/.cache/build/matter-home/ccache-${MATTER_TARGET}" \
  -e CCACHE_MAXSIZE=750M \
  -e SDKCONFIG_DEFAULTS="${MATTER_SDKCONFIG_DEFAULTS}" \
  -e MATTER_OUTPUT="${MATTER_OUTPUT_IN_WORK}" \
  -v "${MATTER_ROOT_MANAGED_COMPONENTS}:/opt/esp/root_managed_components" \
  -v "${REPO_ROOT}:/work" \
  -w "/work/src/cpp/frontend/matter/app" \
  "${DOCKER_IMAGE}" \
  bash -lc "
    set -euo pipefail
    case \"${MATTER_TARGET}\" in
      esp32) MATTER_CHIP=esp32 ;;
      esp32c3) MATTER_CHIP=c3 ;;
      esp32c5) MATTER_CHIP=c5 ;;
      esp32c6) MATTER_CHIP=c6 ;;
      esp32s3) MATTER_CHIP=s3 ;;
      *) echo \"Unsupported Matter target: ${MATTER_TARGET}\" >&2; exit 1 ;;
    esac
    # ESP-IDF activates a venv where --user installs are rejected; install into HOME instead.
    SITE_PACKAGES=\"\${HOME}/.local/lib/python/site-packages\"
    REQUIREMENTS_HASH=\"\$(sha256sum /work/requirements.txt | cut -d ' ' -f 1)\"
    REQUIREMENTS_MARKER=\"\${HOME}/.espectre-requirements-\${REQUIREMENTS_HASH}\"
    export PYTHONPATH=\"\${SITE_PACKAGES}\${PYTHONPATH:+:\${PYTHONPATH}}\"
    if [ ! -f \"\${REQUIREMENTS_MARKER}\" ]; then
      rm -rf \"\${SITE_PACKAGES}\"
      mkdir -p \"\${SITE_PACKAGES}\"
      python -m pip install --target \"\${SITE_PACKAGES}\" -r /work/requirements.txt
      rm -f \"\${HOME}\"/.espectre-requirements-*
      touch \"\${REQUIREMENTS_MARKER}\"
    fi
    if [ -n \"\${SDKCONFIG_DEFAULTS:-}\" ]; then
      export SDKCONFIG_DEFAULTS
    fi
    export ESPECTRE_IDF_BUILD_DIR=${BUILD_DIR}
    cd /work
    python /work/espectre matter build --chip \"\${MATTER_CHIP}\" --backend local
    cd /work/src/cpp/frontend/matter/app/${BUILD_DIR}
    if python -m esptool merge-bin -h >/dev/null 2>&1; then
      python -m esptool --chip ${MATTER_TARGET} merge-bin --pad-to-size 4MB -o \"\${MATTER_OUTPUT}\" @flash_args
    else
      python -m esptool --chip ${MATTER_TARGET} merge_bin --fill-flash-size 4MB -o \"\${MATTER_OUTPUT}\" @flash_args
    fi
    python /work/.github/scripts/build_firmware_compliance.py \
      --frontend matter \
      --project-description /work/src/cpp/frontend/matter/app/${BUILD_DIR}/project_description.json \
      --firmware \"\${MATTER_OUTPUT}\" \
      --output-dir \"\$(dirname \"\${MATTER_OUTPUT}\")\"
    python /work/.github/scripts/audit_firmware_dependencies.py \
      --frontend matter \
      --sbom \"\${MATTER_OUTPUT%.bin}-sbom.spdx.json\" \
      --output-dir /work/.cache/reports/firmware/matter
  "
