#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# One-shot: set the Orin system clock from this laptop over the wired LAN.
# D555 image stamps are Orin host ROS time. Laptop /joint_states and /tf use
# laptop time. DETECT_TF_FAILED / Extrapolation often means those clocks
# drifted. This does not replace the Orin D555 device-clock mapper.
#
#   ./scripts/sync_orin_clock.sh --check
#   ./scripts/sync_orin_clock.sh
#   ORIN_SSH=hku_reconova@192.168.0.100 ORIN_SSH_PASSWORD=... ./scripts/sync_orin_clock.sh
#
# SSH key is preferred. Password auth is used only when ORIN_SSH_PASSWORD is
# set (also feeds sudo -S). Do not commit the password.
set -euo pipefail

ORIN_SSH="${ORIN_SSH:-hku_reconova@192.168.0.100}"
ORIN_SSH_PASSWORD="${ORIN_SSH_PASSWORD:-}"
WARN_S="${ORIN_CLOCK_WARN_S:-0.05}"
FAIL_S="${ORIN_CLOCK_FAIL_S:-0.50}"
MODE="set"
DISABLE_ORIN_NTP=0
CTRL_DIR="${ORIN_SSH_CONTROL:-/tmp/orin-clock-ssh-$$}"
CTRL_PATH="${CTRL_DIR}/%r@%h:%p"

usage() {
  cat <<'EOF'
usage: sync_orin_clock.sh [--check] [--disable-orin-ntp] [--help]

Align the Orin Linux clock to this laptop over SSH (default
hku_reconova@192.168.0.100). --check only prints the offset.

Set ORIN_SSH_PASSWORD if the key is missing or sudo needs a password.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) MODE="check"; shift ;;
    --disable-orin-ntp) DISABLE_ORIN_NTP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

_orin_host() {
  local spec="${ORIN_SSH#*@}"
  echo "${spec%%:*}"
}

_cleanup() {
  ssh -o ControlPath="${CTRL_PATH}" -O exit "${ORIN_SSH}" >/dev/null 2>&1 || true
  rm -rf "${CTRL_DIR}"
}
trap _cleanup EXIT

_ssh_base() {
  ssh \
    -o ControlMaster=auto \
    -o "ControlPath=${CTRL_PATH}" \
    -o ControlPersist=30 \
    -o ConnectTimeout=8 \
    "$@"
}

_open_master_key() {
  mkdir -p -m 700 "${CTRL_DIR}"
  _ssh_base -o BatchMode=yes "${ORIN_SSH}" true >/dev/null 2>&1
}

_open_master_password() {
  mkdir -p -m 700 "${CTRL_DIR}"
  ORIN_SSH="${ORIN_SSH}" ORIN_SSH_PASSWORD="${ORIN_SSH_PASSWORD}" \
  CTRL_PATH="${CTRL_PATH}" python3 - <<'PY'
import os, pexpect, sys
host = os.environ["ORIN_SSH"]
password = os.environ["ORIN_SSH_PASSWORD"]
ctrl = os.environ["CTRL_PATH"]
cmd = [
    "ssh",
    "-o", "ControlMaster=auto",
    "-o", "ControlPath=" + ctrl,
    "-o", "ControlPersist=30",
    "-o", "PreferredAuthentications=password",
    "-o", "PubkeyAuthentication=no",
    "-o", "NumberOfPasswordPrompts=1",
    "-o", "ConnectTimeout=8",
    host,
    "true",
]
child = pexpect.spawn(cmd[0], cmd[1:], timeout=20, encoding="utf-8")
i = child.expect(["password:", "Password:", pexpect.EOF, pexpect.TIMEOUT])
if i in (0, 1):
    child.sendline(password)
    child.expect(pexpect.EOF)
elif i == 3:
    sys.exit(1)
sys.exit(0 if child.exitstatus in (0, None) else child.exitstatus)
PY
}

_ssh() {
  _ssh_base "${ORIN_SSH}" "$@"
}

_sudo() {
  local remote="$1"
  if [[ -n "${ORIN_SSH_PASSWORD}" ]]; then
    _ssh "sudo -S -p '' ${remote}" <<<"${ORIN_SSH_PASSWORD}" >/dev/null
  else
    _ssh "sudo -n ${remote}" >/dev/null
  fi
}

_measure() {
  python3 - <<PY
import subprocess, os
host = os.environ["ORIN_SSH"]
ctrl = os.environ["CTRL_PATH"]

def laptop_now():
    return float(subprocess.check_output(["date", "+%s.%N"], text=True))

def remote_now():
    out = subprocess.check_output(
        ["ssh", "-o", "ControlPath=" + ctrl, "-o", "ControlMaster=auto", host, "date +%s.%N"],
        text=True,
    )
    return float(out)

samples = []
for _ in range(3):
    t0 = laptop_now()
    remote = remote_now()
    t1 = laptop_now()
    samples.append((remote - 0.5 * (t0 + t1), t1 - t0))
offset = sum(s[0] for s in samples) / len(samples)
rtt = sum(s[1] for s in samples) / len(samples)
print("%.6f %.6f" % (offset, rtt))
PY
}

_report() {
  python3 -c "
print('%s: orin-laptop=%+.1f ms  ssh_rtt=%.1f ms' % ('$1', float('$2') * 1000.0, float('$3') * 1000.0))
"
}

export ORIN_SSH CTRL_PATH

host="$(_orin_host)"
echo "ORIN_SSH=${ORIN_SSH} host=${host}"
if ! ping -c 1 -W 1 "$host" >/dev/null 2>&1; then
  echo "cannot ping ${host}. Orin must be on the robot LAN." >&2
  exit 1
fi

auth="key"
if ! _open_master_key; then
  if [[ -z "${ORIN_SSH_PASSWORD}" ]]; then
    echo "SSH key failed (${ORIN_SSH}). Set ORIN_SSH_PASSWORD or run ssh-copy-id." >&2
    exit 1
  fi
  if ! _open_master_password; then
    echo "SSH password login failed (${ORIN_SSH})." >&2
    exit 1
  fi
  auth="password"
fi
echo "ssh_auth=${auth}"

read -r offset rtt < <(_measure)
_report "before" "$offset" "$rtt"

if [[ "$MODE" == "check" ]]; then
  python3 -c "
import sys
offset = abs(float('$offset'))
if offset > float('$FAIL_S'):
    sys.exit(1)
"
  exit "$?"
fi

if [[ -z "${ORIN_SSH_PASSWORD}" ]]; then
  if ! _ssh 'sudo -n true' >/dev/null 2>&1; then
    echo "Orin sudo -n failed. Set ORIN_SSH_PASSWORD for sudo -S." >&2
    exit 1
  fi
fi

if [[ "$DISABLE_ORIN_NTP" == 1 ]]; then
  _sudo "timedatectl set-ntp false"
  echo "disabled Orin NTP so the clock will not jump back"
else
  ntp_state="$(_ssh 'timedatectl show -p NTP --value 2>/dev/null || echo unknown')"
  echo "Orin NTP=${ntp_state} (pass --disable-orin-ntp if it drifts back)"
fi

python3 - <<PY
import os, subprocess, sys

host = os.environ["ORIN_SSH"]
ctrl = os.environ["CTRL_PATH"]
password = os.environ.get("ORIN_SSH_PASSWORD", "")
warn = float("${WARN_S}")
fail = float("${FAIL_S}")

def ssh(remote, stdin=None):
    return subprocess.run(
        ["ssh", "-o", "ControlPath=" + ctrl, "-o", "ControlMaster=auto", host, remote],
        input=stdin, text=True, capture_output=True, check=True,
    ).stdout.strip()

def sudo(remote):
    if password:
        ssh("sudo -S -p '' " + remote, stdin=password + "\n")
    else:
        ssh("sudo -n " + remote)

def laptop_now():
    return float(subprocess.check_output(["date", "+%s.%N"], text=True))

def measure():
    samples = []
    for _ in range(3):
        t0 = laptop_now()
        remote = float(ssh("date +%s.%N"))
        t1 = laptop_now()
        samples.append((remote - 0.5 * (t0 + t1), t1 - t0))
    return (
        sum(s[0] for s in samples) / len(samples),
        sum(s[1] for s in samples) / len(samples),
    )

apply_delay = 0.08
offset = 1.0
rtt = 0.0
for _ in range(4):
    stamp = laptop_now() + apply_delay
    sudo("date -s @{:.9f} >/dev/null".format(stamp))
    offset, rtt = measure()
    print("after: orin-laptop=%+.1f ms  ssh_rtt=%.1f ms" % (offset * 1000.0, rtt * 1000.0))
    if abs(offset) < 0.020:
        break
    apply_delay -= offset

sudo("hwclock -w")
if abs(offset) > fail:
    print("residual larger than %.0f ms; rerun or check sudo/NTP" % (fail * 1000.0), file=sys.stderr)
    sys.exit(1)
if abs(offset) > warn:
    print("residual larger than %.0f ms; acceptable for bringup, rerun if DETECT_TF_FAILED repeats" % (warn * 1000.0))
PY

echo "OK. Restart Orin perception if it was already running so new stamps apply."
