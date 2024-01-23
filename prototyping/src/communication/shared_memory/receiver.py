import numpy as np
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import Process
import matplotlib.pyplot as plt


class Receiver:
    def __init__(self):
        self.data = None
        self.sm = SharedMemory("MyMemory")
        self.last_received_data = None  # New attribute to store the last received data

    def receive(self, size):
        self.data = [int(self.sm.buf[i]) for i in range(size)]
        array_reshape = np.array(self.data).reshape(968, 1226, 3)
        sm.close()
