#!/usr/bin/env bash
set -euo pipefail

echo "[lilsim] Installing build prerequisites (requires sudo for apt)..."
sudo apt update
sudo apt install -y \
  cmake ninja-build build-essential clang lld libstdc++-12-dev \
  pkg-config ccache unzip curl git

VCPKG_DIR="${HOME}/.local/vcpkg"
if [ ! -d "${VCPKG_DIR}" ]; then
  echo "[lilsim] Cloning vcpkg into ${VCPKG_DIR}..."
  mkdir -p "${HOME}/.local"
  git clone https://github.com/microsoft/vcpkg "${VCPKG_DIR}"
fi

echo "[lilsim] Bootstrapping vcpkg..."
"${VCPKG_DIR}/bootstrap-vcpkg.sh" -disableMetrics

if ! grep -q "VCPKG_ROOT=${VCPKG_DIR}" "${HOME}/.bashrc" 2>/dev/null; then
  echo "export VCPKG_ROOT=\"${VCPKG_DIR}\"" >> "${HOME}/.bashrc"
  echo "[lilsim] Added VCPKG_ROOT to ~/.bashrc"
fi

export VCPKG_ROOT="${VCPKG_DIR}"
echo "[lilsim] Done. Open a new shell or 'source ~/.bashrc' so VCPKG_ROOT is available."


