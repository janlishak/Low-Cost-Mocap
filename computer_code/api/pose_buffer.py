import struct
import time
from multiprocessing import shared_memory
import numpy as np

class PoseBufferReader:
    def __init__(self, name: str, max_lines: int):
        self.floats_per_line = 12  # 3 position + 9 rotation
        self.max_lines = max_lines
        self.total_floats = self.floats_per_line * max_lines
        self.byte_size = self.total_floats * 4  # f32 = 4 bytes

        self.shm = shared_memory.SharedMemory(name=name)
        self.buffer = self.shm.buf

    def get_line(self, index: int):
        if index < 0 or index >= self.max_lines:
            raise IndexError("Index out of bounds")

        start = index * self.floats_per_line * 4
        end = start + self.floats_per_line * 4
        data = self.buffer[start:end]
        return struct.unpack('f' * self.floats_per_line, data)
    
    def get_R_t(self, index):
        floats = self.get_line(index)
        tx, ty, tz = floats[0:3]
        
        # Reconstruct column-major rotation matrix
        col0 = np.array(floats[3:6])  # X axis
        col1 = np.array(floats[6:9])  # Y axis
        col2 = np.array(floats[9:12]) # Z axis
        
        R = np.stack([col0, col1, col2], axis=1)  # Columns -> shape (3,3)
        t = np.array([tx, ty, tz])

        return R, t

    def close(self):
        self.shm.close()

def main():
    print("start")
    reader = PoseBufferReader(name="bevy_pose_input_app", max_lines=4)
    try:
        while True:
            for i in range(reader.max_lines):
                pose = reader.get_line(i)
                print(f"Line {i}: position = {pose[:3]}, rotation_matrix = {pose[3:]}")
                R, t = reader.get_R_t(i)
                print(R, t)
                print("--- --- ---")
            time.sleep(4)
    finally:
        reader.close()

if __name__ == "__main__":
    main()