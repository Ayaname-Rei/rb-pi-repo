import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    client.connect('192.168.137.209', username='orangepi', password='orangepi', timeout=5)
    stdin, stdout, stderr = client.exec_command('ls -l')
    print('~:', stdout.read().decode())
    
    stdin, stdout, stderr = client.exec_command('find ~ -name config.py')
    print('config locations:', stdout.read().decode())
    
except Exception as e:
    print('Error:', e)
finally:
    client.close()
