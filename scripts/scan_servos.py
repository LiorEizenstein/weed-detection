#!/usr/bin/env python3
"""Read-only SO-101 Feetech servo bus scan. Pings IDs 1..N and prints
telemetry. Sends ONLY ping + read instructions — never writes, never moves.

Requires: pip install --user --break-system-packages feetech-servo-sdk
Usage:    python3 scripts/scan_servos.py [/dev/ttyACM0] [1000000]
"""
import sys
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
IDS  = range(1, 7)          # SO-101 = 5 arm joints (+ gripper = 6)

# STS3215 control-table addresses (read-only here)
ADDR_POS, ADDR_LOAD, ADDR_VOLT, ADDR_TEMP = 56, 60, 62, 63

ph = PortHandler(PORT)
pk = PacketHandler(0)       # protocol_end=0 -> STS/SMS series

if not ph.openPort():
    sys.exit(f"FAIL: could not open {PORT}")
if not ph.setBaudRate(BAUD):
    sys.exit(f"FAIL: could not set baud {BAUD}")
print(f"Port {PORT} @ {BAUD} baud open. Scanning IDs {IDS.start}..{IDS.stop-1} (read-only)\n")

print(f"{'ID':>3} {'model':>6} {'pos(raw)':>9} {'deg':>7} {'volt':>6} {'temp':>5} {'load':>6}")
print("-" * 50)
found = []
for sid in IDS:
    model, comm, err = pk.ping(ph, sid)
    if comm != COMM_SUCCESS:
        continue
    found.append(sid)
    pos,  _, _ = pk.read2ByteTxRx(ph, sid, ADDR_POS)
    load, _, _ = pk.read2ByteTxRx(ph, sid, ADDR_LOAD)
    volt, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    temp, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_TEMP)
    deg = pos / 4096 * 360.0
    print(f"{sid:>3} {model:>6} {pos:>9} {deg:>7.1f} {volt*0.1:>5.1f}V {temp:>4}C "
          f"{load & 0x3FF:>6}")

ph.closePort()
print("-" * 50)
print(f"Found {len(found)} servo(s): {found}" if found
      else "No servos responded. Check: arm powered (servo LEDs)? baud? board->servo cable?")
