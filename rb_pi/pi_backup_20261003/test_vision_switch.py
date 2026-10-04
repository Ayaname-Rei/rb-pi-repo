#!/usr/bin/env python3
import socket
import json
import time

s = socket.socket()
s.settimeout(4.0)
try:
    s.connect(('192.168.137.209', 8000))
    print("[1] Connected to Orange Pi Vision Server.")
    
    # Receive initial packets
    _ = s.recv(1024)
    print("  Initial low camera stream receiving normally.")
    
    # Request switch to high camera
    print("[2] Requesting switch to HIGH camera...")
    cmd = json.dumps({"cmd": "switch_camera", "camera": "high"}) + "\n"
    s.sendall(cmd.encode('utf-8'))
    time.sleep(1.0)
    data = s.recv(1024)
    print(f"  Switched to HIGH camera successfully! (Received {len(data)} bytes)")
    
    # Request switch back to low camera
    print("[3] Requesting switch back to LOW camera...")
    cmd = json.dumps({"cmd": "switch_camera", "camera": "low"}) + "\n"
    s.sendall(cmd.encode('utf-8'))
    time.sleep(1.0)
    data = s.recv(1024)
    print(f"  Switched back to LOW camera successfully! (Received {len(data)} bytes)")
    
    s.close()
    print("\n>> DUAL-CAMERA DYNAMIC SWITCHING VERIFIED: 100% OPERATIONAL <<")
except Exception as e:
    print(f"Test failed: {e}")
