#!/usr/bin/env python3
"""
OTA Client - Upload files to Pico over WiFi
============================================

Usage:
    python ota_client.py <robot_ip> upload <file> [remote_name]
    python ota_client.py <robot_ip> upload-lib <file>    # Upload to root as lib
    python ota_client.py <robot_ip> restart
    python ota_client.py <robot_ip> list
    python ota_client.py <robot_ip> delete <file>
    python ota_client.py <robot_ip> exec "<code>"
    python ota_client.py <robot_ip> ping

Examples:
    python ota_client.py 192.168.28.219 upload main.py
    python ota_client.py 192.168.28.219 upload libs/picobot.py picobot.py
    python ota_client.py 192.168.28.219 restart
    python ota_client.py 192.168.28.219 exec "machine.freq()"
"""

import socket
import sys
import os
import time

PORT = 5007
CHUNK_SIZE = 1024
TIMEOUT = 5.0


def send_command(ip, command, expect_multi=False):
    """Send command and get response."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)

    try:
        sock.sendto(command, (ip, PORT))
        response, _ = sock.recvfrom(4096)
        return response.decode()
    except socket.timeout:
        return "ERR:Timeout"
    finally:
        sock.close()


def upload_file(ip, local_path, remote_name=None):
    """Upload a file to the Pico."""
    if not os.path.exists(local_path):
        print(f"Error: File not found: {local_path}")
        return False

    if remote_name is None:
        remote_name = os.path.basename(local_path)

    with open(local_path, 'rb') as f:
        data = f.read()

    size = len(data)
    print(f"Uploading {local_path} -> {remote_name} ({size} bytes)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)

    try:
        # Send header + first chunk
        header = f"UPLOAD:{remote_name}:{size}\n".encode()
        first_chunk = min(CHUNK_SIZE - len(header), size)
        packet = header + data[:first_chunk]
        sock.sendto(packet, (ip, PORT))

        sent = first_chunk
        response, _ = sock.recvfrom(1024)
        response = response.decode()

        # Send remaining chunks if needed
        while response.startswith('CONTINUE:'):
            progress = response.split(':')[1]
            print(f"\r  Progress: {progress}", end='', flush=True)

            chunk = data[sent:sent + CHUNK_SIZE]
            if not chunk:
                break
            sock.sendto(chunk, (ip, PORT))
            sent += len(chunk)

            response, _ = sock.recvfrom(1024)
            response = response.decode()

        print()  # New line after progress

        if response.startswith('OK:'):
            print(f"  {response[3:]}")
            return True
        else:
            print(f"  Error: {response}")
            return False

    except socket.timeout:
        print("  Error: Timeout waiting for response")
        return False
    finally:
        sock.close()


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    ip = sys.argv[1]
    cmd = sys.argv[2]

    if cmd == 'ping':
        response = send_command(ip, b'PING')
        print(response)

    elif cmd == 'list':
        response = send_command(ip, b'LIST')
        print(response)

    elif cmd == 'restart':
        response = send_command(ip, b'RESTART')
        print(response)

    elif cmd == 'upload' and len(sys.argv) >= 4:
        local_path = sys.argv[3]
        remote_name = sys.argv[4] if len(sys.argv) > 4 else None
        success = upload_file(ip, local_path, remote_name)
        sys.exit(0 if success else 1)

    elif cmd == 'upload-lib' and len(sys.argv) >= 4:
        # Upload library file to root
        local_path = sys.argv[3]
        remote_name = os.path.basename(local_path)
        success = upload_file(ip, local_path, remote_name)
        sys.exit(0 if success else 1)

    elif cmd == 'delete' and len(sys.argv) >= 4:
        filename = sys.argv[3]
        response = send_command(ip, f'DELETE:{filename}'.encode())
        print(response)

    elif cmd == 'exec' and len(sys.argv) >= 4:
        code = sys.argv[3]
        response = send_command(ip, f'EXEC:{code}'.encode())
        print(response)

    elif cmd == 'deploy':
        # Quick deploy: upload common files and restart
        files = [
            ('libs/picobot.py', 'picobot.py'),
            ('libs/oled.py', 'oled.py'),
            ('libs/bmi160.py', 'bmi160.py'),
            ('libs/ota.py', 'ota.py'),
        ]
        if len(sys.argv) >= 4:
            # Add main.py from specified lab
            files.append((sys.argv[3], 'main.py'))

        for local, remote in files:
            if os.path.exists(local):
                upload_file(ip, local, remote)

        print("\nRestarting...")
        send_command(ip, b'RESTART')

    else:
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
