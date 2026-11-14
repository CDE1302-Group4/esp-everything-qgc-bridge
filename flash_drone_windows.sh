#!/usr/bin/env bash
#
# Build and flash one drone from Git Bash on Windows.
#
#   ./flash_drone_windows.sh                 # prompts for drone ID and COM port
#   ./flash_drone_windows.sh 4 COM4          # build and flash drone 4
#   ./flash_drone_windows.sh 4 COM4 monitor  # flash, then open IDF monitor
#
# The script temporarily pins sdkconfig to the selected drone and restores the
# exact pre-run file on success, failure, Ctrl-C, and after the monitor exits.

set -euo pipefail

[ "$#" -le 3 ] || { echo "usage: $0 [drone_id] [COM_port] [monitor]" >&2; exit 2; }
MONITOR_MODE="${3:-}"
case "$MONITOR_MODE" in ""|monitor) ;; *) echo "third argument must be 'monitor'" >&2; exit 2;; esac

ID_MAX=30
HOST_IP_PREFIX="192.168.1"
HOST_IP_BASE=100
DRONE_IP_PREFIX="192.168.1"
DRONE_IP_BASE=200
WIFI_GATEWAY="192.168.1.1"
WIFI_NETMASK="255.255.255.0"

die()  { printf '\nERROR: %s\n' "$*" >&2; exit 1; }
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

command -v git >/dev/null 2>&1 || die "Git is required"
command -v powershell.exe >/dev/null 2>&1 || die "Run this from Git Bash on Windows"
command -v cygpath >/dev/null 2>&1 || die "cygpath is missing; use Git Bash"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && git rev-parse --show-toplevel 2>/dev/null)" \
  || die "not inside a git work tree"
cd "$PROJECT_DIR"
SDKCONFIG="$PROJECT_DIR/sdkconfig"
[ -f "$SDKCONFIG" ] || die "no sdkconfig in $PROJECT_DIR"
git ls-files --error-unmatch sdkconfig >/dev/null 2>&1 \
  || die "sdkconfig is not tracked by git -- refusing to modify it"

# Never guess which physical airframe is attached.
DRONE_ID="${1:-}"
if [ -z "$DRONE_ID" ]; then
    read -r -p "Drone ID (0-$ID_MAX): " DRONE_ID || die "drone ID is required"
fi
case "$DRONE_ID" in ''|*[!0-9]*) die "drone ID must be a whole number 0-$ID_MAX";; esac
[ "${#DRONE_ID}" -le 2 ] || die "drone ID must be 0-$ID_MAX"
DRONE_ID=$((10#$DRONE_ID))
[ "$DRONE_ID" -le "$ID_MAX" ] || die "drone ID $DRONE_ID is outside 0-$ID_MAX"

# ---------------------------------------------------------------- ESP-IDF
step "ESP-IDF environment"

# If Git Bash inherited an activated ESP-IDF environment, use it. Otherwise,
# read the selected Windows installation from Espressif Installation Manager
# and run idf.py through EIM's PowerShell activation script. ESP-IDF refuses to
# run under Git Bash ("MSys/Mingw is not supported") because MSYSTEM is set, and
# the MSYS runtime re-injects MSYSTEM into every native child it spawns, so it
# cannot be unset from bash -- PowerShell removes it before activating instead.
# An activated project venv is cleared too: EIM's profile ends with the IDF
# venv's Activate.ps1, which "deactivates" first by restoring _OLD_VIRTUAL_PATH
# and would wipe the cmake/toolchain entries the profile just put on PATH.
if ! command -v idf.py >/dev/null 2>&1; then
    EIM_JSON_WIN="${EIM_IDF_JSON:-C:\\Espressif\\tools\\eim_idf.json}"
    EIM_JSON_UNIX="$(cygpath -u "$EIM_JSON_WIN")"
    [ -f "$EIM_JSON_UNIX" ] || die "ESP-IDF is not active and $EIM_JSON_WIN was not found"

    read_selected() {
        local property="$1"
        powershell.exe -NoProfile -Command \
          "\$c=Get-Content -Raw -LiteralPath '$EIM_JSON_WIN'|ConvertFrom-Json; \$i=\$c.idfInstalled|Where-Object id -eq \$c.idfSelectedId; \$i.$property" \
          | tr -d '\r'
    }

    IDF_PATH_WIN="$(read_selected path)"
    IDF_ACTIVATE_WIN="$(read_selected activationScript)"
    [ -n "$IDF_PATH_WIN" ] || die "no selected ESP-IDF installation in $EIM_JSON_WIN"
    [ -n "$IDF_ACTIVATE_WIN" ] && [ -f "$(cygpath -u "$IDF_ACTIVATE_WIN")" ] \
      || die "missing ESP-IDF activation script: $IDF_ACTIVATE_WIN"
    IDF_PATH="$(cygpath -u "$IDF_PATH_WIN")"

    idf.py() {
        local args="" a
        for a in "$@"; do args="$args '${a//\'/\'\'}'"; done
        powershell.exe -NoProfile -ExecutionPolicy Bypass -Command \
          "Remove-Item Env:MSYSTEM,Env:VIRTUAL_ENV,Env:_OLD_VIRTUAL_PATH,Env:_OLD_VIRTUAL_PYTHONHOME -ErrorAction SilentlyContinue; . '$IDF_ACTIVATE_WIN' *> \$null; idf.py$args; exit \$LASTEXITCODE"
    }
    echo "using EIM activation $IDF_ACTIVATE_WIN"
fi

command -v idf.py >/dev/null 2>&1 || die "idf.py is unavailable after ESP-IDF activation"
echo "using $(command -v idf.py) -- $(idf.py --version 2>/dev/null)"

PIN="$PROJECT_DIR/tools/idf-pin.env"
if [ -f "$PIN" ] && [ -n "${IDF_PATH:-}" ]; then
    want="$(sed -n 's/^IDF_CHECKOUT_REF=//p' "$PIN")"
    have="$(git -C "$IDF_PATH" rev-parse HEAD 2>/dev/null || echo unknown)"
    if [ -n "$want" ] && [ "$want" != "$have" ]; then
        echo "WARNING: installed ESP-IDF is $have"
        echo "         repository pin is $want ($(sed -n 's/^IDF_EXPECTED_DESCRIBE=//p' "$PIN"))"
        echo "         the firmware may differ from the team build"
    fi
fi

# ---------------------------------------------------------- sdkconfig custody
BACKUP="$(mktemp -t sdkconfig.before.XXXXXX)"
cp "$SDKCONFIG" "$BACKUP"
MUTATED=0

cleanup() {
    rc=$?
    set +e
    trap '' PIPE HUP INT TERM
    trap - EXIT
    if [ "$MUTATED" -eq 1 ]; then
        cp "$BACKUP" "$SDKCONFIG"
        printf '\n\033[1;36m==> Restoring sdkconfig\033[0m\nrestored exact pre-run sdkconfig\n'
    fi
    rm -f "$BACKUP"
    if [ "$rc" -eq 0 ]; then printf '\033[1;32mDONE\033[0m\n'
    else printf '\033[1;31mFAILED (exit %s)\033[0m\n' "$rc"; fi
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

step "Resetting temporary build configuration"
if ! git diff --quiet -- sdkconfig; then
    echo "NOTE: sdkconfig has local changes; they will be restored exactly after this run."
    if [ -t 0 ]; then
        read -r -p "continue? [y/N] " answer
        case "$answer" in y|Y|yes|YES) ;; *) die "aborted by user";; esac
    fi
fi
MUTATED=1
git restore sdkconfig

# ---------------------------------------------------------- drone configuration
step "Drone configuration"
HOST_IP="$HOST_IP_PREFIX.$((HOST_IP_BASE + DRONE_ID))"
DRONE_IP="$DRONE_IP_PREFIX.$((DRONE_IP_BASE + DRONE_ID))"
TMP="$(mktemp -t sdkconfig.new.XXXXXX)"
awk -v id="$DRONE_ID" -v host_ip="$HOST_IP" -v drone_ip="$DRONE_IP" \
    -v gateway="$WIFI_GATEWAY" -v netmask="$WIFI_NETMASK" '
  index($0, "CONFIG_DRONE_ID=") == 1 { print "CONFIG_DRONE_ID=" id; si=1; next }
  index($0, "# CONFIG_DRONE_ID is not set") == 1 { print "CONFIG_DRONE_ID=" id; si=1; next }
  index($0, "CONFIG_HOST_IPV4_ADDR=") == 1 { print "CONFIG_HOST_IPV4_ADDR=\"" host_ip "\""; sh=1; next }
  index($0, "# CONFIG_HOST_IPV4_ADDR is not set") == 1 { print "CONFIG_HOST_IPV4_ADDR=\"" host_ip "\""; sh=1; next }
  index($0, "CONFIG_DRONE_STATIC_IPV4_ADDR=") == 1 { print "CONFIG_DRONE_STATIC_IPV4_ADDR=\"" drone_ip "\""; sd=1; next }
  index($0, "# CONFIG_DRONE_STATIC_IPV4_ADDR is not set") == 1 { print "CONFIG_DRONE_STATIC_IPV4_ADDR=\"" drone_ip "\""; sd=1; next }
  index($0, "CONFIG_WIFI_GATEWAY_IPV4_ADDR=") == 1 { print "CONFIG_WIFI_GATEWAY_IPV4_ADDR=\"" gateway "\""; sg=1; next }
  index($0, "# CONFIG_WIFI_GATEWAY_IPV4_ADDR is not set") == 1 { print "CONFIG_WIFI_GATEWAY_IPV4_ADDR=\"" gateway "\""; sg=1; next }
  index($0, "CONFIG_WIFI_NETMASK_IPV4_ADDR=") == 1 { print "CONFIG_WIFI_NETMASK_IPV4_ADDR=\"" netmask "\""; sn=1; next }
  index($0, "# CONFIG_WIFI_NETMASK_IPV4_ADDR is not set") == 1 { print "CONFIG_WIFI_NETMASK_IPV4_ADDR=\"" netmask "\""; sn=1; next }
  { print }
  END { if (!si) print "CONFIG_DRONE_ID=" id
        if (!sh) print "CONFIG_HOST_IPV4_ADDR=\"" host_ip "\""
        if (!sd) print "CONFIG_DRONE_STATIC_IPV4_ADDR=\"" drone_ip "\""
        if (!sg) print "CONFIG_WIFI_GATEWAY_IPV4_ADDR=\"" gateway "\""
        if (!sn) print "CONFIG_WIFI_NETMASK_IPV4_ADDR=\"" netmask "\"" }
' "$SDKCONFIG" > "$TMP"
cat "$TMP" > "$SDKCONFIG"
rm -f "$TMP"

grep -Fxq "CONFIG_DRONE_ID=$DRONE_ID" "$SDKCONFIG" || die "failed to set drone ID"
grep -Fxq "CONFIG_HOST_IPV4_ADDR=\"$HOST_IP\"" "$SDKCONFIG" || die "failed to set laptop IP"
grep -Fxq "CONFIG_DRONE_STATIC_IPV4_ADDR=\"$DRONE_IP\"" "$SDKCONFIG" || die "failed to set drone IP"
grep -Fxq "CONFIG_WIFI_GATEWAY_IPV4_ADDR=\"$WIFI_GATEWAY\"" "$SDKCONFIG" || die "failed to set gateway"
grep -Fxq "CONFIG_WIFI_NETMASK_IPV4_ADDR=\"$WIFI_NETMASK\"" "$SDKCONFIG" || die "failed to set netmask"
echo "drone $DRONE_ID -> laptop $HOST_IP, ESP32 $DRONE_IP"

# --------------------------------------------------------------------- build
step "Building drone $DRONE_ID"
idf.py build

HDR="$PROJECT_DIR/build/config/sdkconfig.h"
[ -f "$HDR" ] || die "build did not create $HDR"
grep -Fxq "#define CONFIG_DRONE_ID $DRONE_ID" "$HDR" || die "built binary has wrong drone ID"
grep -Fxq "#define CONFIG_HOST_IPV4_ADDR \"$HOST_IP\"" "$HDR" || die "built binary has wrong laptop IP"
grep -Fxq "#define CONFIG_DRONE_STATIC_IPV4_ADDR \"$DRONE_IP\"" "$HDR" || die "built binary has wrong drone IP"
grep -Fxq "#define CONFIG_WIFI_GATEWAY_IPV4_ADDR \"$WIFI_GATEWAY\"" "$HDR" || die "built binary has wrong gateway"
grep -Fxq "#define CONFIG_WIFI_NETMASK_IPV4_ADDR \"$WIFI_NETMASK\"" "$HDR" || die "built binary has wrong netmask"
echo "verified generated firmware configuration"

# ---------------------------------------------------------------- COM port
step "Serial port"
PORT="${2:-}"
list_ports() {
    powershell.exe -NoProfile -Command '[System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object' \
      | tr -d '\r'
}

if [ -z "$PORT" ]; then
    mapfile -t PORTS < <(list_ports)
    if [ "${#PORTS[@]}" -eq 1 ]; then
        PORT="${PORTS[0]}"
        echo "detected $PORT"
    else
        echo "Available serial ports:"
        if [ "${#PORTS[@]}" -eq 0 ]; then echo "  (none detected)"
        else printf '  %s\n' "${PORTS[@]}"; fi
        read -r -p "COM port for drone $DRONE_ID (for example COM4): " PORT \
          || die "COM port is required"
    fi
fi
PORT="$(printf '%s' "$PORT" | tr '[:lower:]' '[:upper:]')"
case "$PORT" in COM[0-9]|COM[0-9][0-9]|COM[0-9][0-9][0-9]) ;; *) die "invalid Windows COM port: $PORT";; esac

# Capture first: piping into grep -q under pipefail reports a false "missing"
# when grep exits early on a match and PowerShell then hits a closed pipe.
# No -i: Git Bash's grep 3.0 aborts on -F with -i, and both sides are already
# uppercase (PORT is upper-cased above; Windows reports ports as COMnn).
CONNECTED_PORTS="$(list_ports)"
if ! grep -Fxq "$PORT" <<<"$CONNECTED_PORTS"; then
    echo "WARNING: $PORT is not currently reported by Windows."
    if [ -t 0 ]; then
        read -r -p "try flashing it anyway? [y/N] " answer
        case "$answer" in y|Y|yes|YES) ;; *) die "connect the ESP32 and retry";; esac
    else
        die "$PORT is not currently connected"
    fi
fi

# --------------------------------------------------------------------- flash
step "Flashing drone $DRONE_ID on $PORT"
idf.py -p "$PORT" flash
echo "flashed drone $DRONE_ID (ESP32 $DRONE_IP, laptop $HOST_IP) on $PORT"

if [ "$MONITOR_MODE" = "monitor" ]; then
    step "Serial monitor (exit with Ctrl+])"
    idf.py -p "$PORT" monitor
fi
