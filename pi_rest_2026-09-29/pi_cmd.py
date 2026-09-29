import sys
import paramiko

def run_cmd(cmd):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('172.20.10.3', port=22, username='pinqu', password='521297', timeout=10)
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    exit_code = stdout.channel.recv_exit_status()
    client.close()
    return exit_code, out, err

if __name__ == '__main__':
    if len(sys.argv) > 1:
        cmd = ' '.join(sys.argv[1:])
        code, out, err = run_cmd(cmd)
        if out:
            print(out, end='')
        if err:
            print(err, end='', file=sys.stderr)
        sys.exit(code)
    else:
        print('No command specified')

