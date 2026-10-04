import paramiko

def try_ssh(ip, username, password):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(ip, username=username, password=password, timeout=2)
        print(f"Success: {username}@{ip} with password {password}")
        return True
    except Exception as e:
        # print(f"Failed: {username}@{ip} - {e}")
        return False
    finally:
        client.close()

ips = ['192.168.137.210', '192.168.137.231']
credentials = [('pi', 'raspberry'), ('pi', 'pi'), ('root', 'root'), ('root', 'raspberry'), ('admin', 'admin'), ('orangepi', 'orangepi')]

for ip in ips:
    for user, pwd in credentials:
        if try_ssh(ip, user, pwd):
            break
