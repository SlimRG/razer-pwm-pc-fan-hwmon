#!/usr/bin/env bash
set -euo pipefail

PKG_NAME="razer-pwm-pc-fan"
PKG_VER="2.12"
MODULE_NAME="razer_pwm_pc_fan_controller"

need_root() { [[ "${EUID}" -eq 0 ]] || { echo "Run: sudo $0"; exit 1; }; }

main() {
  need_root

  modprobe -r "${MODULE_NAME}" >/dev/null 2>&1 || true
  dkms remove -m "${PKG_NAME}" -v "${PKG_VER}" --all >/dev/null 2>&1 || true

  rm -rf "/usr/src/${PKG_NAME}-${PKG_VER}" || true
  rm -f "/etc/dkms/framework.conf.d/${PKG_NAME}.conf" || true

  echo "Removed."
}

main "$@"
