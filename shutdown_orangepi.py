#!/usr/bin/env python3
import time
import socket
import paramiko
import sys

def try_shutdown(ip, username='orangepi', password='orangepi'):
    print(f"[*] Trying to connect to {username}@{ip}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(ip, username=username, password=password, timeout=3)
        print(f"[+] Connected to {ip}! Sending shutdown command...")
        stdin, stdout, stderr = client.exec_command('sudo -S poweroff', get_pty=True)
        time.sleep(0.5)
        stdin.write(f"{password}\n")
        stdin.flush()
        out = stdout.read().decode(errors='ignore')
        err = stderr.read().decode(errors='ignore')
        print(f"[+] Shutdown output:\n{out}\n{err}")
        return True
    except Exception as e:
        print(f"[-] Failed to connect to {ip}: {e}")
        return False
    finally:
        client.close()

if __name__ == '__main__':
    # Potential IPs
    ips = ['192.168.137.209', 'orangepi.local', 'rb-pi.local', '192.168.137.210', '192.168.137.231']
    for ip in ips:
        if try_shutdown(ip):
            print("[+] Orange Pi shutdown initiated successfully!")
            sys.exit(0)
    print("[-] Could not connect to any known IP address.")
