import unittest
import threading
import socket
import time

# Import your Receiver and Sender classes
from src.receiver import Receiver
from src.sender import Sender


class TestNetworkOperations(unittest.TestCase):

    def setUp(self):
        # Setup for each test
        self.host = "127.0.0.1"
        self.port = 65432
        self.receiver = Receiver(self.host, self.port)
        self.sender = Sender(self.host, self.port)

    def test_receiver_start_server(self):
        # Test to check if the server starts correctly
        server_thread = threading.Thread(target=self.receiver.start_server)
        server_thread.start()

        # Allow some time for the server to start
        time.sleep(1)

        # Check if the server is listening
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            result = s.connect_ex((self.host, self.port))
            self.assertEqual(result, 0)  # 0 indicates success

        # Close the server
        server_thread.join()

    def test_sender_send_data(self):
        # Test to check if the sender can send data to the receiver
        server_thread = threading.Thread(target=self.receiver.start_server)
        # Set as a daemon so it will close when the main thread exits
        server_thread.daemon = True
        server_thread.start()

        # Allow some time for the server to start
        time.sleep(1)

        # Send data
        test_data = "Hello, world!"
        self.sender.send_data(test_data)

        # Allow some time for the data to be sent and processed
        time.sleep(1)

        # Check if the receiver got the data
        self.assertEqual(self.receiver.last_received_data, test_data)

        # Note: The server thread will close when the test exits, as it's a daemon thread


if __name__ == '__main__':
    unittest.main()