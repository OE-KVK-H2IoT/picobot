# find_robots.py
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 37020))

print("Waiting for robots...")

while True:
    data, addr = sock.recvfrom(256)
    print(data.decode())
