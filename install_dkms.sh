#!/usr/bin/env bash
set -euo pipefail

PKG_NAME="razer-pwm-pc-fan"
PKG_VER="2.12"
MODULE_NAME="razer_pwm_pc_fan_controller"

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST_DIR="/usr/src/${PKG_NAME}-${PKG_VER}"

need_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Run as root: sudo $0"
    exit 1
  fi
}

have_cmd() { command -v "$1" >/dev/null 2>&1; }

apt_install_if_missing() {
  local pkg="$1"
  dpkg -s "$pkg" >/dev/null 2>&1 || apt-get install -y "$pkg"
}

secure_boot_enabled() {
  # mokutil --sb-state prints "SecureBoot enabled" or "SecureBoot disabled"
  mokutil --sb-state 2>/dev/null | grep -qi "enabled"
}

mok_key_enrolled() {
  # Best-effort: mokutil --test-key returns success if key is enrolled.
  # Not all distros ship --test-key, so fall back to list-enrolled check.
  local cert="/var/lib/dkms/mok.pub"
  [[ -f "$cert" ]] || return 1

  if mokutil --help 2>&1 | grep -q -- "--test-key"; then
    mokutil --test-key "$cert" >/dev/null 2>&1
    return $?
  fi

  # Fallback: try matching certificate hash/name is messy; just say "unknown"
  return 1
}

ensure_sources_present() {
  for f in razer_pwm_pc_fan_controller.c Makefile dkms.conf; do
    [[ -f "${SRC_DIR}/${f}" ]] || { echo "Missing file: ${SRC_DIR}/${f}"; exit 1; }
  done
}

setup_dkms_mok_autosign() {
  # DKMS reads /etc/dkms/framework.conf and /etc/dkms/framework.conf.d/*.conf :contentReference[oaicite:2]{index=2}
  # Variables mok_signing_key/mok_certificate are supported and DKMS can re-create them if missing. :contentReference[oaicite:3]{index=3}
  mkdir -p /etc/dkms/framework.conf.d

  cat >/etc/dkms/framework.conf.d/${PKG_NAME}.conf <<'EOF'
# DKMS Secure Boot module signing configuration
# DKMS will use these for automatic module signing at build/install time.
mok_signing_key="/var/lib/dkms/mok.key"
mok_certificate="/var/lib/dkms/mok.pub"
EOF
}

main() {
  need_root
  ensure_sources_present

  echo "[1/7] Installing build dependencies..."
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -y
  apt_install_if_missing build-essential
  apt_install_if_missing dkms
  apt_install_if_missing "linux-headers-$(uname -r)"
  apt_install_if_missing mokutil
  apt_install_if_missing openssl

  echo "[2/7] Installing sources into ${DST_DIR}..."
  rm -rf "${DST_DIR}"
  mkdir -p "${DST_DIR}"
  cp -a "${SRC_DIR}/." "${DST_DIR}/"

  echo "[3/7] Configuring DKMS auto-signing key paths (for Secure Boot)..."
  setup_dkms_mok_autosign

  echo "[4/7] DKMS add/build/install..."
  dkms remove -m "${PKG_NAME}" -v "${PKG_VER}" --all >/dev/null 2>&1 || true
  dkms add    -m "${PKG_NAME}" -v "${PKG_VER}"
  dkms build  -m "${PKG_NAME}" -v "${PKG_VER}"
  dkms install -m "${PKG_NAME}" -v "${PKG_VER}"

  echo "[5/7] Secure Boot handling..."
  if secure_boot_enabled; then
    echo "  - Secure Boot: ENABLED"
    echo "  - Your DKMS signing certificate should be: /var/lib/dkms/mok.pub"
    echo "  - If this key is NOT enrolled into MOK, the kernel will refuse to load the module. :contentReference[oaicite:4]{index=4}"

    if mok_key_enrolled; then
      echo "  - MOK key already enrolled."
    else
      echo "  - MOK key not detected as enrolled."
      echo "  - Enrolling MOK key now (you will set a one-time password, then enroll on next reboot):"
      mokutil --import /var/lib/dkms/mok.pub
      echo
      echo "  NEXT STEPS (required):"
      echo "    1) Reboot"
      echo "    2) In the blue 'MOK Manager' screen: Enroll MOK -> Continue -> Yes -> enter the password -> Reboot :contentReference[oaicite:5]{index=5}"
      echo
      echo "  After that, run: sudo modprobe ${MODULE_NAME}"
    fi
  else
    echo "  - Secure Boot: DISABLED"
  fi

  echo "[6/7] Loading module..."
  if modprobe "${MODULE_NAME}" 2>/dev/null; then
    echo "  - Module loaded: ${MODULE_NAME}"
  else
    echo "  - Module did not load (this is expected if Secure Boot is enabled but MOK not enrolled yet)."
    echo "    Check: dmesg | tail -n 50"
  fi

  echo "[7/7] Quick sanity checks:"
  echo "  - DKMS status:"
  dkms status | grep -i "${PKG_NAME}" || true
  echo "  - hwmon devices (look for razer_pwm_pc_fan):"
  grep -R . /sys/class/hwmon/hwmon*/name 2>/dev/null | grep -i razer || true

  echo
  echo "Done."
  echo "If you want standard tools: sudo apt install -y lm-sensors fancontrol && sensors && sudo pwmconfig"
}

main "$@"
