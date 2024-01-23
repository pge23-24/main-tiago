import numpy as np
from PIL import Image
import time
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import Process


class Sender:
    def __init__(self, image):
        self.image = image
        self.shared_mem = None
        self.img_array = None
        self.size = None

    def prepare_image(self):
        self.img_array = np.array(self.image).flatten()
        self.img_array = self.img_array.reshape(-1, 1, 1)
        self.size = len(self.img_array)

    def create_shared_memory(self):
        self.shared_mem = SharedMemory(name="MyMemory", create=True, size=self.size)

    def send_data(self):
        self.shared_mem.buf[: self.size] = bytearray(self.img_array)
        self.shared_mem.close()

    def get_shared_memory(self):
        return self.shared_mem

    def get_size(self):
        return self.size


if __name__ == "__main__":
    sender = Sender()
    sender.prepare_image()
    sender.create_shared_memory()

    process = Process(target=sender.send_data)
    process.start()
    process.join()

    shared_mem = sender.get_shared_memory()
    shared_mem.close()
    shared_mem.unlink()
