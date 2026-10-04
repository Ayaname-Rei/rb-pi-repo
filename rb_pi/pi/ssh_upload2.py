import paramiko
import os

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    key_path = os.path.expanduser('~/.ssh/id_rsa')
    if not os.path.exists(key_path):
        key_path = os.path.expanduser('~/.ssh/id_ed25519')
    
    key = paramiko.RSAKey.from_private_key_file(key_path) if 'rsa' in key_path else paramiko.Ed25519Key.from_private_key_file(key_path)
    
    print('Connecting to 192.168.137.210...')
    client.connect('192.168.137.210', username='pinqu', pkey=key, timeout=5)
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
