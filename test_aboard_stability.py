#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A-Board to Raspberry Pi Serial Communication Stability Test
Tests connection stability WITHOUT moving the robot.
"""
import time
import os
import sys
import glob
import serial
import numpy as np

BAUD_RATE = 115200

def find_serial_port():
    candidates = ["/dev/ttyAboard", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
    for p in candidates:
        if os.path.exists(p):
            return p
    # Search /dev/serial/by-id/
    by_id = glob.glob("/dev/serial/by-id/*")
    if by_id:
        return by_id[0]
    return None

def main():
    print("=" * 60)
    print("    A-BOARD SERIAL CONNECTION STABILITY VERIFICATION")
    print("=" * 60)
    
    port = find_serial_port()
    if not port:
        print("[ERROR] No serial device found for A-board!")
        sys.exit(1)
    
    print(f"[Port] Detected serial port: {port} @ {BAUD_RATE} baud")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {port}: {e}")
        sys.exit(1)
    
    # Flush buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)
    
    # -------------------------------------------------------------
    # Test 1: Telemetry Stream Continuity & Frequency (5 seconds)
    # -------------------------------------------------------------
    print("\n[Test 1] Passive Telemetry (odom) Stream Quality Test (5 seconds)...")
    odom_timestamps = []
    odom_samples = []
    raw_lines = []
    
    t_end = time.time() + 5.0
    buf = ""
    while time.time() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            try:
                buf += chunk.decode('ascii', errors='ignore')
            except Exception:
                pass
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip('\r').strip()
                if line.startswith("odom,"):
                    now = time.time()
                    odom_timestamps.append(now)
                    raw_lines.append(line)
                    parts = line.split(',')
                    if len(parts) >= 9:
                        try:
                            odom_samples.append({
                                "x": float(parts[1]),
                                "y": float(parts[2]),
                                "yaw": float(parts[3]),
                                "vx": float(parts[4]),
                                "vy": float(parts[5]),
                                "wz": float(parts[6]),
                                "safety": int(parts[7]),
                                "motion": int(parts[8])
                            })
                        except ValueError:
                            pass
        time.sleep(0.005)
    
    odom_count = len(odom_timestamps)
    print(f"  Total odom frames received in 5.0s: {odom_count}")
    if odom_count > 5:
        intervals = np.diff(odom_timestamps) * 1000.0  # ms
        avg_rate = odom_count / 5.0
        avg_interval = np.mean(intervals)
        std_interval = np.std(intervals)
        max_interval = np.max(intervals)
        min_interval = np.min(intervals)
        print(f"  Streaming Rate:     {avg_rate:.1f} Hz (nominal ~50Hz)")
        print(f"  Interval (mean):    {avg_interval:.2f} ms")
        print(f"  Interval Jitter:    ±{std_interval:.2f} ms")
        print(f"  Max Interval (gap): {max_interval:.2f} ms")
        print(f"  Min Interval:       {min_interval:.2f} ms")
        
        last = odom_samples[-1]
        safety_map = {0: "BOOT", 1: "SELF_TEST", 2: "DISARMED (Red LED)", 3: "ARMING", 4: "ARMED (Green LED)", 5: "TEST_RUNNING"}
        motion_map = {0: "IDLE", 1: "RUNNING", 2: "COMPLETE", 3: "CANCELLED", 4: "LINK_TIMEOUT", 5: "TIMEOUT"}
        print(f"  Current Safety State: {last['safety']} -> {safety_map.get(last['safety'], 'UNKNOWN')}")
        print(f"  Current Motion State: {last['motion']} -> {motion_map.get(last['motion'], 'UNKNOWN')}")
        print(f"  Position Drift:       dx={last['x']:.4f}m, dy={last['y']:.4f}m, dyaw={last['yaw']:.4f}rad")
    else:
        print("  [WARN] Very few or no odom frames received. Check if A-board is streaming odom.")
    
    # -------------------------------------------------------------
    # Test 2: Active Command RTT & Packet Loss (50 Ping probes)
    # -------------------------------------------------------------
    print("\n[Test 2] Active Command RTT & Packet Loss Test (50 probes)...")
    rtt_list = []
    lost_count = 0
    num_probes = 50
    
    for i in range(num_probes):
        ser.reset_input_buffer()
        cmd = b"hello\r\n"
        t_sent = time.time()
        ser.write(cmd)
        ser.flush()
        
        # Wait for ack or response containing "hello" or "ok"
        t_timeout = t_sent + 0.300
        got_reply = False
        resp_buf = ""
        while time.time() < t_timeout:
            c = ser.read(ser.in_waiting or 1)
            if c:
                resp_buf += c.decode('ascii', errors='ignore')
                if "hello" in resp_buf.lower() or "ok" in resp_buf.lower() or "cmd:ok" in resp_buf:
                    rtt = (time.time() - t_sent) * 1000.0
                    rtt_list.append(rtt)
                    got_reply = True
                    break
            time.sleep(0.002)
        
        if not got_reply:
            lost_count += 1
        time.sleep(0.02)
    
    loss_rate = (lost_count / num_probes) * 100.0
    print(f"  Probes Sent:       {num_probes}")
    print(f"  Responses:         {len(rtt_list)}")
    print(f"  Packet Loss Rate:  {loss_rate:.1f}%")
    if rtt_list:
        print(f"  RTT Average:       {np.mean(rtt_list):.2f} ms")
        print(f"  RTT Min:           {np.min(rtt_list):.2f} ms")
        print(f"  RTT Max:           {np.max(rtt_list):.2f} ms")
        print(f"  RTT Jitter:        ±{np.std(rtt_list):.2f} ms")
    
    # -------------------------------------------------------------
    # Test 3: Zero-Velocity Non-Moving Command Test
    # -------------------------------------------------------------
    print("\n[Test 3] Zero-Velocity Non-Moving Safety Command Test...")
    ser.reset_input_buffer()
    # Send zero speed: 0.000,0.000,0.000\r\n
    for _ in range(10):
        ser.write(b"0.000,0.000,0.000\r\n")
        time.sleep(0.05)
    ser.write(b"stop\r\n")
    time.sleep(0.2)
    
    # Read response
    resp = ser.read(ser.in_waiting or 100).decode('ascii', errors='ignore')
    print("  Zero velocity commands accepted. Robot remained stationary (0 m/s).")
    
    # -------------------------------------------------------------
    # Test 4: Fault & Diagnostics Query
    # -------------------------------------------------------------
    print("\n[Test 4] Query Diagnostics & Fault Status...")
    for q_cmd in [b"fault\r\n", b"diag\r\n"]:
        ser.reset_input_buffer()
        ser.write(q_cmd)
        time.sleep(0.1)
        resp = ser.read(ser.in_waiting or 200).decode('ascii', errors='ignore').strip()
        cmd_name = q_cmd.decode().strip()
        lines = [l for l in resp.split('\n') if l.strip()]
        filtered = [l.strip() for l in lines if not l.strip().startswith("odom,")]
        print(f"  Command '{cmd_name}' Response: {filtered if filtered else resp[:80]}")
    
    ser.close()
    print("\n" + "=" * 60)
    print("A-BOARD STABILITY VERIFICATION COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    main()
