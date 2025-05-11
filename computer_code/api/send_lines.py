import numpy as np
import struct
from multiprocessing import shared_memory

class LineBufferWriter:
    FLOAT_SIZE = 4
    FLOATS_PER_LINE = 6
    LINE_SIZE = FLOATS_PER_LINE * FLOAT_SIZE  # 24 bytes per line

    def __init__(self, name):
        self.shm = shared_memory.SharedMemory(name=name)
        self.index = 0
        self.max_lines = 512

    def set_line(self, index, start, end):
        start = np.asarray(start, dtype=np.float32).flatten()
        end = np.asarray(end, dtype=np.float32).flatten()

        if start.shape[0] != 3 or end.shape[0] != 3:
            raise ValueError("Start and End must be 3D points (length 3)")

        offset = index * self.LINE_SIZE
        line_data = struct.pack('6f', *(start.tolist() + end.tolist()))
        self.shm.buf[offset:offset + self.LINE_SIZE] = line_data

    def next_line(self, start, end):
        self.set_line(self.index, start, end)
        self.index = (self.index + 1) % self.max_lines

    def reset(self):
        zero = np.zeros(3, dtype=np.float32)
        for i in range(self.max_lines):
            self.set_line(i, zero, zero)
        self.index = 0

    def close(self):
        self.shm.close()

# Example usage
if __name__ == "__main__":
    writer = LineBufferWriter("bevy_line_input_app")

    P = [
            ( 0.0   , 0.0 ,  0.0  ),   # 0 - origin
            (-0.075 , 0.0 ,  0.0  ),   # 1 point
            ( 0.075 , 0.0 ,  0.0  ),   # 2 point
            ( 0.0   , 0.0 , -0.058)    # 3 point
    ]

    writer.set_line(0, P[1], P[2])
    writer.set_line(1, P[2], P[3])
    writer.set_line(2, P[3], P[1])

    writer.close()
