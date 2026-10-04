#!/usr/bin/env python3
import socket
import json
import time

s = socket.socket()
s.settimeout(3.0)
try:
    s.connect(('192.168.137.209', 8000))
    print("SUCCESS: Connected to Orange Pi Vision Server on 192.168.137.209:8000")
    t0 = time.time()
    count = 0
    first_packet = None
    while time.time() - t0 < 3.0:
        data = s.recv(2048).decode('utf-8', errors='ignore')
        for line in data.strip().split('\n'):
            line = line.strip()
            if line.startswith('{') and line.endswith('}'):
                try:
                    parsed = json.loads(line)
                    count += 1
                    if first_packet is None:
                        first_packet = parsed
                except Exception:
                    pass
        time.sleep(0.01)
    s.close()
    print(f"Received {count} vision packets in 3.0s ({count/3.0:.1f} Hz)")
    if first_packet:
        print("Sample Packet Structure:")
        print(json.dumps(first_packet, indent=2))
except Exception as e:
    print(f"FAILED to connect to Orange Pi vision server: {e}")
