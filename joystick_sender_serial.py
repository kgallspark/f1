
#!/usr/bin/env python3
import serial, socket, argparse, time, json

SERIAL_PORT = "/dev/ttyACM0"
BAUD        = 115200
DEST_IP     = "100.78.69.83"
DEST_PORT   = 6000
HZ          = 50.0

ap = argparse.ArgumentParser()
ap.add_argument("--port",  default=SERIAL_PORT)
ap.add_argument("--dest",  default=DEST_IP)
ap.add_argument("--dport", type=int, default=DEST_PORT)
ap.add_argument("--hz",    type=float, default=HZ)
args = ap.parse_args()
period = 1.0 / args.hz

ser  = serial.Serial(args.port, BAUD, timeout=1)
ser.reset_input_buffer()  # This clears any cached/stale data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"🚀 Forwarding Xiao joystick serial JSON → {args.dest}:{args.dport}")

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("{") and line.endswith("}"):
            # Confirm it's valid JSON
            js = json.loads(line)
            print(js)
            # Send it as-is
            sock.sendto(line.encode(), (args.dest, args.dport))
    except Exception:
        pass
    time.sleep(period)


