#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Keep the Orin Linux clock locked to this laptop over the wired LAN.
# Laptop: userspace SNTP source on UDP 192.168.0.11:3232 (no root).
# Orin: system chrony, only server 192.168.0.11 port 3232.
#
#   ./scripts/setup_lan_chrony.sh
#   ./scripts/setup_lan_chrony.sh --check
#   ORIN_SSH=hku_reconova@192.168.0.100 ORIN_SSH_SUDO_PASSWORD=... ./scripts/setup_lan_chrony.sh
#
# Do not commit the sudo password.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORIN_SSH="${ORIN_SSH:-hku_reconova@192.168.0.100}"
ORIN_SSH_SUDO_PASSWORD="${ORIN_SSH_SUDO_PASSWORD:-${ORIN_SSH_PASSWORD:-}}"
LAN_BIND="${LAN_NTP_BIND:-192.168.0.11}"
LAN_PORT="${LAN_NTP_PORT:-3232}"
UNIT="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/roboticarm-lan-ntp.service"
MODE="setup"

usage() {
  cat <<'EOF'
usage: setup_lan_chrony.sh [--check] [--help]

Start the laptop LAN time source and point Orin chrony at it.
--check only prints the current offset.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) MODE="check"; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

_ssh() {
  ssh -o BatchMode=yes -o ConnectTimeout=8 "${ORIN_SSH}" "$@"
}

_sudo_remote() {
  if [[ -n "${ORIN_SSH_SUDO_PASSWORD}" ]]; then
    printf '%s\n' "${ORIN_SSH_SUDO_PASSWORD}" | ssh -o BatchMode=yes -o ConnectTimeout=8 "${ORIN_SSH}" "sudo -S -p '' $*"
  else
    _ssh "sudo -n $*"
  fi
}

_offset() {
  python3 - <<PY
import subprocess, time
t0 = time.time()
raw = subprocess.check_output(
    ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
     "${ORIN_SSH}", "date +%s.%N"],
    text=True,
).strip()
t1 = time.time()
remote = float(raw)
mid = (t0 + t1) / 2.0
print(f"{remote - mid:.6f}  ssh_rtt_s={t1 - t0:.3f}")
PY
}

_write_unit() {
  mkdir -p "$(dirname "${UNIT}")"
  cat >"${UNIT}" <<EOF
[Unit]
Description=Robot LAN SNTP source for Orin chrony
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 ${ROOT}/scripts/lan_ntp_server.py --bind ${LAN_BIND} --port ${LAN_PORT}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=default.target
EOF
}

_start_laptop() {
  _write_unit
  systemctl --user daemon-reload
  systemctl --user enable --now roboticarm-lan-ntp.service
  sleep 0.3
  systemctl --user --no-pager --full status roboticarm-lan-ntp.service | head -20
}

_configure_orin() {
  _sudo_remote "DEBIAN_FRONTEND=noninteractive apt-get install -y -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true chrony"
  _sudo_remote "timedatectl set-ntp false" || true
  _sudo_remote "systemctl stop systemd-timesyncd.service" || true
  _sudo_remote "systemctl disable systemd-timesyncd.service" || true
  _ssh "cat > /tmp/roboticarm-chrony.conf" <<EOF
# Orin follows the laptop LAN clock only. Internet NTP was several hundred
# milliseconds off this cell and broke D555/TF joins.
confdir /etc/chrony/conf.d
sourcedir /run/chrony-dhcp
driftfile /var/lib/chrony/chrony.drift
ntsdumpdir /var/lib/chrony
keyfile /etc/chrony/chrony.keys
leapsectz right/UTC
makestep 1.0 -1
maxupdateskew 100.0
rtcsync
server ${LAN_BIND} iburst minpoll 2 maxpoll 6 port ${LAN_PORT}
EOF
  _sudo_remote "install -m 644 /tmp/roboticarm-chrony.conf /etc/chrony/chrony.conf"
  _sudo_remote "systemctl enable chrony.service"
  _sudo_remote "systemctl restart chrony.service"
  sleep 2
  _sudo_remote "chronyc makestep" || true
  _ssh "chronyc tracking; echo ---; chronyc sources -v; echo ---; timedatectl | sed -n '1,12p'"
}

echo "laptop=$(date +%s.%N) bind=${LAN_BIND}:${LAN_PORT}"
echo "orin_ssh=${ORIN_SSH}"
if [[ "${MODE}" == "check" ]]; then
  echo "ssh_date_offset_s=$(_offset)  # includes asymmetric SSH login; use chronyc below"
  _ssh "chronyc tracking 2>/dev/null || true; chronyc sources -v 2>/dev/null || true"
  exit 0
fi

_start_laptop
_configure_orin
echo "offset_orin_minus_laptop_s=$(_offset)"
echo "restart Orin perception after a large step so the D555 mapper resets"
