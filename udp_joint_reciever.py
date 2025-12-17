#!/usr/bin/env python3
# Compatible with Python 3.5+

import socket
import json
import os
import time

LISTEN_IP     = "0.0.0.0"
LISTEN_PORT   = 5005
OUTPUT_FILE   = "/srv/samba/share/joint_pos.txt"
TEMP_FILE     = OUTPUT_FILE + ".tmp"
STATUS_FILE   = "/srv/samba/share/udp_status_return.txt"
BUF_SIZE      = 4096  # plenty for small JSON blobs

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
print("Listening on UDP {}:{}".format(LISTEN_IP, LISTEN_PORT))

def read_status():
    if not os.path.exists(STATUS_FILE):
        return {"state": "unknown", "torque": None}
    try:
        with open(STATUS_FILE, "r") as f:
            return json.load(f)
    except Exception as e:
        print("⚠️ Error reading status file:", e)
        return {"state": "error", "torque": None}

while True:
    data, addr = sock.recvfrom(BUF_SIZE)
    try:
        payload = json.loads(data.decode())
    except Exception as e:
        print("Bad JSON from {}: {}".format(addr, e))
        continue

    # Only write if packet has joint angle data
    if "angles" in payload:
        with open(TEMP_FILE, "w") as f:
            json.dump(payload, f, indent=2)
        os.replace(TEMP_FILE, OUTPUT_FILE)
    else:
        print("ℹ️ Ignoring packet without angles from {}".format(addr[0]))
        

    # Build a talkback message
    status = read_status()
    talkback = {
        "handshake": payload.get("handshake", "none") + "_ack",
        "state": status.get("state", "unknown"),
        "torque": status.get("torque", {})
    }

    try:
        sock.sendto(json.dumps(talkback).encode(), addr)
#        print("↩️ Sent talkback to {}: {}".format(addr[0], talkback))
    except Exception as e:
        print("⚠️ Failed to send talkback to {}: {}".format(addr[0], e))


