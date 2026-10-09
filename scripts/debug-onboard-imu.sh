#!/bin/bash
# Build/run the documented ST-LINK + GDB diagnostic without opening CubeIDE.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/debug-onboard-imu.sh probe|verify

  probe   Attach to the existing IMU-test firmware; read WHO_AM_I at 0x68/0x69
          on I2C1/I2C2, then resume. Does not flash or reset the sensor.
          Disconnect the board's USB data cable first; leave ST-LINK connected.
  verify  Build, flash and verify 6storm32-test, reset, check live IMU samples
          for 5 seconds, then resume and detach.

Both modes briefly halt the MCU. Keep the board powered throughout the test.
Probe requires this project's current ELF to match the firmware already flashed.

Optional environment variables:
  IMU_CUBEIDE_APP     CubeIDE app path (default /Applications/STM32CubeIDE.app)
  IMU_STLINK_SERIAL   Probe serial (default: read 6storm32-test.launch)
  IMU_GDB_PORT        Unused localhost TCP port (default 61234)
  IMU_VERIFY_SECONDS  Verification time, 1-45 seconds (default 5)

Logs are retained in a temporary directory printed when the script starts.
EOF
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  probe|verify) mode=$1 ;;
  *) usage >&2; exit 2 ;;
esac
[[ $# -eq 1 ]] || { usage >&2; exit 2; }

repo_root=$(cd "$(dirname "$0")/.." && pwd)
cubeide_app=${IMU_CUBEIDE_APP:-/Applications/STM32CubeIDE.app}
gdb_port=${IMU_GDB_PORT:-61234}
verify_seconds=${IMU_VERIFY_SECONDS:-5}
command -v python3 >/dev/null || { echo 'python3 is required.' >&2; exit 1; }

find_tool() {
  local candidate
  for candidate in "$cubeide_app"/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools."$1".*/tools/bin/"$2"; do
    if [[ -x "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  echo "Cannot find $2 in $cubeide_app" >&2
  return 1
}

gdb=$(find_tool gnu-tools-for-stm32 arm-none-eabi-gdb)
server=$(find_tool stlink-gdb-server ST-LINK_gdbserver)
programmer=$(find_tool cubeprogrammer STM32_Programmer_CLI)
make_tool=$(find_tool make make)
elf="$repo_root/6storm32-test/Debug/6storm32-test.elf"

stlink_serial=${IMU_STLINK_SERIAL:-}
if [[ -z "$stlink_serial" ]]; then
  stlink_serial=$(python3 - "$repo_root/6storm32-test/6storm32-test.launch" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
key = 'com.st.stm32cube.ide.mcu.debug.stlink.stlink_txt_serial_number'
print(next((item.get('value', '') for item in root if item.get('key') == key), ''))
PY
  )
fi
[[ -n "$stlink_serial" ]] || { echo 'Set IMU_STLINK_SERIAL to the connected probe serial.' >&2; exit 1; }

# Refuse to reuse a port owned by an existing debug session.
python3 - "$gdb_port" "$verify_seconds" <<'PY'
import socket
import sys
port, seconds = map(int, sys.argv[1:])
if not 1024 <= port <= 65535 or not 1 <= seconds <= 45:
    sys.exit('Use a port from 1024-65535 and IMU_VERIFY_SECONDS from 1-45.')
with socket.socket() as sock:
    try:
        sock.bind(('127.0.0.1', port))
    except OSError as exc:
        sys.exit(f'Debug port {port} is unavailable: {exc}')
PY

if [[ "$mode" == verify ]]; then
  export PATH="$(dirname "$gdb"):$PATH"
  "$make_tool" -C "$repo_root/6storm32-test/Debug" -s -j4 all
fi
[[ -f "$elf" ]] || { echo "Missing $elf; build the project first." >&2; exit 1; }

log_dir=$(mktemp -d "${TMPDIR:-/tmp}/storm32-imu-debug.XXXXXX")
echo "Mode: $mode; ST-LINK: $stlink_serial; logs: $log_dir"
server_pid=''
cleanup() {
  if [[ -n "$server_pid" ]]; then
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

server_args=(-e -d -i "$stlink_serial" -p "$gdb_port" --frequency 4000 -s -cp "$(dirname "$programmer")")
probe_mode=1
if [[ "$mode" == verify ]]; then
  server_args+=(-k)
  probe_mode=0
else
  server_args+=(-g)
fi
"$server" "${server_args[@]}" >"$log_dir/server.log" 2>&1 &
server_pid=$!

# Check the server's own readiness message without consuming its GDB connection.
python3 - "$server_pid" "$log_dir/server.log" <<'PY'
import os
from pathlib import Path
import sys
import time
pid, log = int(sys.argv[1]), Path(sys.argv[2])
for _ in range(100):
    output = log.read_text(errors='replace')
    if 'Waiting for debugger connection' in output:
        break
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        sys.exit(output or 'ST-LINK server exited before becoming ready.')
    time.sleep(0.1)
else:
    sys.exit(output + '\nST-LINK server did not become ready within 10 seconds.')
PY

cd "$repo_root"
if [[ "$mode" == probe ]]; then
  # Compare before calling target functions: stale symbols can call wrong code.
  # Read bytes directly; avoid relying on the debug server's remote CRC support.
  python3 - "$elf" "$log_dir" <<'PY'
from pathlib import Path
import struct
import sys
elf, logs = Path(sys.argv[1]), Path(sys.argv[2])
data = elf.read_bytes()
if data[:6] != b'\x7fELF\x01\x01':
    sys.exit('Expected a 32-bit little-endian STM32 ELF.')
header = struct.unpack_from('<HHIIIIIHHHHHH', data, 16)
offset, entry_size, count, names_index = header[5], header[10], header[11], header[12]
sections = [struct.unpack_from('<10I', data, offset + i * entry_size) for i in range(count)]
names_header = sections[names_index]
names = data[names_header[4]:names_header[4] + names_header[5]]
commands = []
found = set()
for section in sections:
    name = names[section[0]:].split(b'\0', 1)[0].decode()
    if name not in ('.isr_vector', '.text', '.rodata'):
        continue
    found.add(name)
    (logs / ('expected' + name + '.bin')).write_bytes(data[section[4]:section[4] + section[5]])
    target = 'actual' + name + '.bin'
    commands.append(f'dump binary memory {target} {section[3]:#x} {section[3] + section[5]:#x}')
if len(found) != 3:
    sys.exit('Required firmware sections were not found.')
(logs / 'compare.gdb').write_text('\n'.join(commands) + '\n')
PY
  (cd "$log_dir"; "$gdb" --quiet --batch "$elf" \
    -ex 'set pagination off' \
    -ex 'set confirm off' \
    -ex 'set remotetimeout 10' \
    -ex "target remote 127.0.0.1:$gdb_port" \
    -x "$log_dir/compare.gdb" \
    -ex 'detach') 2>&1 | tee "$log_dir/compare.log"
  python3 - "$log_dir" <<'PY'
from pathlib import Path
import sys
logs = Path(sys.argv[1])
for name in ('.isr_vector', '.text', '.rodata'):
    expected = (logs / ('expected' + name + '.bin')).read_bytes()
    actual_file = logs / ('actual' + name + '.bin')
    if not actual_file.exists():
        sys.exit(f'Could not read {name} from the target; inspect compare.log.')
    actual = actual_file.read_bytes()
    if expected != actual:
        sys.exit(f'Firmware section {name} does not match this ELF; run verify before probe.')
    print(f'{name}: matched ({len(actual)} bytes).')
PY
fi
"$gdb" --quiet --batch "$elf" \
  -ex 'set pagination off' \
  -ex 'set confirm off' \
  -ex 'set remotetimeout 10' \
  -ex "set \$imu_probe_mode = $probe_mode" \
  -ex "set \$imu_verify_seconds = $verify_seconds" \
  -ex "target remote 127.0.0.1:$gdb_port" \
  -x "$repo_root/scripts/imu-debug.gdb" 2>&1 | tee "$log_dir/gdb.log"
echo "Finished; logs: $log_dir"
