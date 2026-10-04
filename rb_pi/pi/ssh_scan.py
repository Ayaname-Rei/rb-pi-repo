import socket
import paramiko

def scan():
    for i in range(1, 255):
        ip = f'192.168.137.{i}'
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.1)
            result = s.connect_ex((ip, 22))
            if result == 0:
                print(f"Port 22 open on: {ip}")
            s.close()
        except:
            pass

scan()
