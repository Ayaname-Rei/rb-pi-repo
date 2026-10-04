import paramiko
import os
import socket

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    key_path = os.path.expanduser('~/.ssh/id_rsa')
    if not os.path.exists(key_path):
        key_path = os.path.expanduser('~/.ssh/id_ed25519')
    
    key = paramiko.RSAKey.from_private_key_file(key_path) if 'rsa' in key_path else paramiko.Ed25519Key.from_private_key_file(key_path)
    
    print('Connecting to IPv6...')
    # Use socket directly to support IPv6 with zone index
    addrinfo = socket.getaddrinfo('fe80::37f:a221:757c:2c53%7', 22, socket.AF_INET6, socket.SOCK_STREAM)
    sock = socket.socket(addrinfo[0][0], addrinfo[0][1], addrinfo[0][2])
    sock.settimeout(5)
    sock.connect(addrinfo[0][4])
    
    client.connect('hostname', sock=sock, username='pinqu', pkey=key, timeout=5)
    print('Connected!')
    
    sftp = client.open_sftp()
    
    local_conf = r'd:\competition_code\rb_competition_code\rb_pi\pi\RG\raspberrypi\config.py'
    remote_conf = '/home/pinqu/RG/raspberrypi/config.py'
    sftp.put(local_conf, remote_conf)
    
    local_mot = r'd:\competition_code\rb_competition_code\rb_pi\pi\RG\raspberrypi\motion_client.py'
    remote_mot = '/home/pinqu/RG/raspberrypi/motion_client.py'
    sftp.put(local_mot, remote_mot)
    
    sftp.close()
    print('Upload successful!')
except Exception as e:
    print('Error:', e)
finally:
    client.close()
