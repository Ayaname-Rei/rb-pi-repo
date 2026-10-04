import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    print('Connecting to 192.168.137.209...')
    client.connect('192.168.137.209', username='orangepi', password='orangepi', timeout=5)
    print('Connected!')
    
    sftp = client.open_sftp()
    local_path = r'd:\competition_code\rb_competition_code\rb_pi\pi\RG\orangepi\config.py'
    remote_path = '/home/orangepi/vision_node/config.py'
    sftp.put(local_path, remote_path)
    sftp.close()
    print('Upload successful!')
except Exception as e:
    print('Error:', e)
finally:
    client.close()
