#!/usr/bin/env python3
"""
combined_remote_send.py
—————————
Reads joystick input and streams Dynamixel joint angles to LEFT or RIGHT arm over UDP,
interpolating transitions for smooth motion, including on startup.
"""

import serial, socket, argparse, time, json, threading, sys, os, math, traceback
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import netifaces  # Added to detect outbound interface IP

# ───── Joystick settings ─────
JOY_SERIAL_PORT = "/dev/ttyACM0"
JOY_BAUD        = 115200
JOY_HZ          = 50.0
JOY_X_KEY       = "x"
HAT_KEY         = "hat"
HAT_PRESSED_VAL = 0
X_LEFT_THRESH   = 100
X_RIGHT_THRESH  = 900
# ─────────────────────────────

# ───── Dynamixel settings ─────
SERVO_SERIAL_PORT  = "/dev/ttyACM1"
SERVO_BAUDRATE     = 1_000_000
SERVO_PROTO        = 2.0
ADDR_PRESENT       = 132
CAL_FILE           = "/home/task-control/remote_control/dock_home_90.json"
JOINT_IDS          = [1, 2, 3, 4, 5, 6, 8]
J3, J4, J5         = 3, 4, 5
SERVO_HZ           = 20.0
USB_REOPEN_S       = 2.0
MAX_SERVO_RETRIES  = 3
TARGET_SPEED_DPS   = 30.0  # degrees per second
# ─────────────────────────────

# ───── Shared state ─────
robotToReceive = "left"
robotLock = threading.Lock()
ready_flags = {"left": False, "right": False}
controller_ip = None
# ─────────────────────────

def parse_args():
    ap = argparse.ArgumentParser(description="Joysticks + Dynamixel angle UDP sender")
    ap.add_argument("--leftArmIP",    required=True)
    ap.add_argument("--rightArmIP",   required=True)
    ap.add_argument("--taskbotMCPIP", required=True)
    ap.add_argument("--mcpPort",      type=int, default=6000)
    ap.add_argument("--armPort",      type=int, default=5005)
    ap.add_argument("--joyPort",      default=JOY_SERIAL_PORT)
    ap.add_argument("--servoPort",    default=SERVO_SERIAL_PORT)
    ap.add_argument("--joyHz",        type=float, default=JOY_HZ)
    ap.add_argument("--servoHz",      type=float, default=SERVO_HZ)
    ap.add_argument("--statusPort",   type=int, default=5006)
    return ap.parse_args()

def unified_listener(sock, args, ready_flags):
    print(f"📡 Unified listener running on port {args.armPort}")

    # Send hello messages using the provided socket
    controller_ip = get_real_local_ip(args.rightArmIP)
    hello_msg = json.dumps({
        "hello": True,
        "controller_ip": controller_ip,
        "handshake": "initial_hello"
    }).encode()

    for ip in [args.leftArmIP, args.rightArmIP]:
        sock.sendto(hello_msg, (ip, args.armPort))
        print(f"👋 Sent hello to {ip}:{args.armPort}")

    # Receive loop
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            sender = addr[0]

            hs = msg.get("handshake", "")
#            print("msg")
            if msg.get("state") == "ready_to_receive_new_positions":
                #if sender == args.leftArmIP:
                ready_flags["left"] = True
                #print("✅ LEFT robot ready")
                if sender == args.rightArmIP:
                    ready_flags["right"] = True
                    print("✅ RIGHT robot ready")
                else:
                    print(f"⚠️ Unknown robot IP: {sender}")
            elif hs.endswith("_ack"):
                print(f"📥 Status from {sender}: {msg}")
            elif hs == "periodic_update":
                print(f"🔁 Periodic torque update from {sender}: {msg.get('torque')}")
            else:
                print(f"📥 Other talkback from {sender}: {msg}")

        except Exception as e:
            print("⚠️ Unified listener error:", e)

def get_real_local_ip(remote_ip="8.8.8.8"):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((remote_ip, 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ───────── Joystick Thread ─────────
def joystick_worker(args):
    global robotToReceive
    period = 1.0 / args.joyHz
    ser = serial.Serial(args.joyPort, JOY_BAUD, timeout=1)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mcp_addr = (args.taskbotMCPIP, args.mcpPort)

    print(f"🕹️  Forwarding joystick → {mcp_addr}  @ {args.joyHz} Hz")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not (line.startswith("{") and line.endswith("}")):
                continue

            js = json.loads(line)
            x = js.get(JOY_X_KEY)
            hat = js.get(HAT_KEY)

            with robotLock:
                current_target = robotToReceive
#            print(f"🕹️  X={x}  hat={hat}  → target={current_target}")

            sock.sendto(line.encode(), mcp_addr)

            if hat == HAT_PRESSED_VAL and isinstance(x, (int, float)):
                with robotLock:
                    if x <= X_LEFT_THRESH and robotToReceive != "left":
                        robotToReceive = "left"
                        print("🔀 switched target → LEFT arm")
                    elif x >= X_RIGHT_THRESH and robotToReceive != "right":
                        robotToReceive = "right"
                        print("🔀 switched target → RIGHT arm")

        except Exception as e:
            print("⚠️  Joystick thread error:", e)
        time.sleep(period)

# ───────── Dynamixel Helpers ─────────
def to_i32(v): return v - 0x100000000 if v & 0x80000000 else v

def open_bus(path):
    while True:
        port = PortHandler(path)
        if port.openPort() and port.setBaudRate(SERVO_BAUDRATE):
            return port, PacketHandler(SERVO_PROTO)
        print("⚠️  USB open failed – retrying in 2 s")
        time.sleep(USB_REOPEN_S)

def read_all(pkt, port, bad_cnt, home):
    data = {}
    for i in JOINT_IDS:
        v, res, err = pkt.read4ByteTxRx(port, i, ADDR_PRESENT)
        if res == COMM_SUCCESS and err == 0:
            data[i] = to_i32(v); bad_cnt[i] = 0
        else:
            bad_cnt[i] += 1
            data[i] = data.get(i, home.get(str(i), 0))
            if bad_cnt[i] > MAX_SERVO_RETRIES:
                raise RuntimeError(f"ID {i} unresponsive")
    return data

# ───────── Servo Streaming Thread ─────────
def servo_worker(args, sock):
    global robotToReceive, ready_flags

    if not os.path.exists(CAL_FILE):
        sys.exit(f"⚠️  {CAL_FILE} not found – run dock_and_calibrate.py first")

    with open(CAL_FILE) as f:
        calib = json.load(f)
        home   = calib["home"]
        ninety = calib["ninety"]

    deg_tick = {j: 90.0 / (ninety[str(j)] - home[str(j)])
                for j in JOINT_IDS if j != 8}

    period = 1.0 / args.servoHz
    port, pkt = open_bus(args.servoPort)
#    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    zero_pose = {
        "1": 0.0, "2": 0.0, "3": 0.0,
        "4": 0.0, "5": 0.0, "6": 0.0, "8": 0
    }
    last_sent_angles = {"left": zero_pose.copy(), "right": zero_pose.copy()}
    interp_queue = []
    last_target = None
    seq, bad_cnt = 0, {j: 0 for j in JOINT_IDS}

    print(f"🤖 Streaming angles @ {args.servoHz} Hz → port {args.armPort}")

    # Wait for both arms to be ready
    print("⏳ Waiting for BOTH remote arms to signal readiness...")
    while not (ready_flags["left"] and ready_flags["right"]):
        time.sleep(0.5)

    print("✅ BOTH arms ready — starting streaming")

    while True:
        try:
            raw = read_all(pkt, port, bad_cnt, home)
            rel = {j: raw[j] - home[str(j)] for j in JOINT_IDS if j != 8}
            deg = {j: rel[j] * deg_tick[j] for j in rel}

            diff_rot = (deg[J4] - deg[J5]) / 2
            diff_rad = (deg[J4] + deg[J5]) / 2 - deg[J3]

            current_angles = {
                "1": deg.get(1, 0.0),
                "2": deg.get(2, 0.0),
                "3": deg.get(3, 0.0),
                "4": diff_rad,
                "5": diff_rot,
                "6": deg.get(6, 0.0),
                "8": raw[8]
            }

            with robotLock:
                target_arm = robotToReceive

            if target_arm != last_target:
                last_target = target_arm
                start_pose = last_sent_angles.get(target_arm)

                delta = math.sqrt(sum(
                    (current_angles[k] - start_pose[k]) ** 2
                    for k in current_angles if k != "8"
                ))
                interp_delay = 1.0 / args.servoHz
                steps = max(1, math.ceil(delta / (TARGET_SPEED_DPS * interp_delay)))
                interp_queue.clear()
                for i in range(1, steps + 1):
                    t = i / steps
                    interp_angles = {
                        k: (
                            current_angles[k] if k == "8"
                            else start_pose[k] + (current_angles[k] - start_pose[k]) * t
                        )
                        for k in current_angles
                    }
                    interp_queue.append(interp_angles)
                print(f"🔀 Switching to {target_arm.upper()} arm – "
                      f"distance: {delta:.1f}°, steps: {steps}, duration: {steps * interp_delay:.2f}s")

            send_angles = interp_queue.pop(0) if interp_queue else current_angles

            packet = {"ts": time.time(), "seq": seq, "angles": send_angles}
            dest_ip = args.leftArmIP if target_arm == "left" else args.rightArmIP
            sock.sendto(json.dumps(packet).encode(), (dest_ip, args.armPort))

            last_sent_angles[target_arm] = send_angles.copy()
            seq += 1

            time.sleep(interp_delay if interp_queue else period)

        except Exception as e:
            print("⚠️ Servo thread error:", e)
            time.sleep(period)

# ───────── Main ─────────
def main():
    print("🚀 Starting combined_remote_send.py")
    args = parse_args()
    print("✅ Parsed args:", args)

    # Shared state for robot readiness
    global ready_flags
    ready_flags = {"left": False, "right": False}

    # 🎯 Create and bind a single UDP socket for receiving talkback/status
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(("0.0.0.0", args.armPort))
    print(f"🔌 Bound UDP socket to 0.0.0.0:{args.armPort}")

    # 🔁 Start unified listener with the bound socket
    threading.Thread(
        target=unified_listener,
        args=(udp_sock, args, ready_flags),
        daemon=True
    ).start()

    # 🎮 Joystick thread
    threading.Thread(
        target=joystick_worker,
        args=(args,),
        daemon=True
    ).start()
    print("🎮 Joystick thread started")

    # 🦾 Start servo streaming loop (blocks)
    try:
        servo_worker(args, udp_sock)
    except KeyboardInterrupt:
        print("\n⏹  Stopped.")


if __name__ == "__main__":
    main()


