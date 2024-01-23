import socket


class Receiver:
    def __init__(self, host="127.0.0.1", port=65432):
        self.host = host
        self.port = port
        self.last_received_data = None  # New attribute to store the last received data

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            print(f"Server started. Listening on {self.host}:{self.port}")
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    self.last_received_data = data.decode()  # Store the received data
                    print(f"Received data: {self.last_received_data}")