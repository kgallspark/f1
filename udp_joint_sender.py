
#!/usr/bin/env python3
# udp_joint_sender_resilient.py  –––  keeps running even on I/O hiccups
# (only lines marked  ### CHANGED / NEW  differ from your original code)

import socket, time, json, argparse, sys, os, traceback   ### NEW
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ───── user settings ─────
SERIAL_PORT  = "/dev/ttyACM1"
BAUDRATE     = 1_000_000
PROTO        = 2.0
ADDR_PRESENT = 132
CAL_FILE     = "/home/task-control/remote_control/dock_home_90.json"
JOINT_IDS    = [1, 2, 3, 4, 5, 6, 8]
J3, J4, J5   = 3, 4, 5
SEND_PERIOD  = 0.05          # 50 ms
USB_REOPEN_S = 2.0           # wait before re-opening USB  ### NEW
MAX_SERVO_RETRIES = 3        # missed polls before we drop a joint ### NEW
# ─────────────────────────

def to_i32(v): return v - 0x100000000 if v & 0x80000000 else v

def open_bus():
    while True:                                                  ### NEW
        port = PortHandler(SERIAL_PORT)
        if port.openPort() and port.setBaudRate(BAUDRATE):
            return port, PacketHandler(PROTO)
        print("⚠️  USB open failed – retrying in 2 s")
        time.sleep(USB_REOPEN_S)

def read_all(pkt, port, bad_cnt):                                 ### NEW
    data = {}
    for i in JOINT_IDS:
        v, res, err = pkt.read4ByteTxRx(port, i, ADDR_PRESENT)
        if res == COMM_SUCCESS and err == 0:
            data[i] = to_i32(v)
            bad_cnt[i] = 0                                        ### NEW
        else:
            bad_cnt[i] += 1
            if bad_cnt[i] <= MAX_SERVO_RETRIES:                   ### NEW
                # keep previous value (or home) for a few cycles
                data[i] = data.get(i, home.get(str(i), 0))
            else:
                raise RuntimeError(f"ID {i} unresponsive")
    return data

# ───── load calibration ─────
if not os.path.exists(CAL_FILE):
    sys.exit(f"⚠️  {CAL_FILE} not found – run dock_and_calibrate.py first")

with open(CAL_FILE) as f:
    ref = json.load(f)
dock, home, ninety = ref["dock"], ref["home"], ref["ninety"]

deg_tick = {
    j: 90.0 / float(ninety[str(j)] - home[str(j)])
    for j in JOINT_IDS if j != 8
}

# ───── CLI dest IP ─────
ap = argparse.ArgumentParser()
ap.add_argument("--dest", required=True, help="uZed Tailscale IP")
ap.add_argument("--port", type=int, default=5005)
ap.add_argument("--period", type=float, default=SEND_PERIOD)
args = ap.parse_args()
DEST = (args.dest, args.port)

# ───── open HW + UDP ─────
port, pkt = open_bus()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"🟢 streaming only ANGLES to {DEST[0]}:{DEST[1]} every {args.period*1000:.0f} ms")

seq = 0
bad_cnt = {j: 0 for j in JOINT_IDS}                               ### NEW
while True:
    try:
        raw = read_all(pkt, port, bad_cnt)

        # relative counts (raw – home) for calibrated joints
        rel = {j: raw[j] - home[str(j)] for j in JOINT_IDS if j != 8}
        deg = {j: rel[j] * deg_tick[j]   for j in rel}

        diff_rot = (deg[J4] - deg[J5]) / 2.0
        diff_rad = (deg[J4] + deg[J5]) / 2.0 - deg[J3]

        angles = {
            "1": deg.get(1, 0.0),
            "2": deg.get(2, 0.0),
            "3": deg.get(3, 0.0),
            "4": diff_rad,
            "5": diff_rot,
            "6": deg.get(6, 0.0),
            "8": raw[8]
        }

        packet = {"ts": time.time(), "seq": seq, "angles": angles}
        try:
            sock.sendto(json.dumps(packet).encode(), DEST)
        except OSError as e:                                      ### NEW
            print("⚠️  UDP send failed:", e)

        seq += 1
        time.sleep(args.period)

    except RuntimeError as e:
        print("⚠️ ", e, "→ resetting bus…")
        port.closePort()
        time.sleep(USB_REOPEN_S)
        port, pkt = open_bus()
        bad_cnt = {j: 0 for j in JOINT_IDS}

    except KeyboardInterrupt:
        break

print("⏹ stopped.")
port.closePort()

