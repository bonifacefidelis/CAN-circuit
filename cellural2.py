import socket

HOST = "0.0.0.0"   # Your PC local address
PORT = 12345            # Must match your SIM7600 code

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print(f"Listening on {HOST}:{PORT}...")

conn, addr = server.accept()
print(f"Connected by {addr}")

while True:
    data = conn.recv(1024)
    if not data:
        break
    print("Received:", data.decode().strip())
