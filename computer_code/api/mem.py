import numpy as np
import cv2
from multiprocessing import shared_memory
import numpy as np

class GreenExampleCamera:
    def __init__(self):
        self.bgr_frame = np.zeros((512, 512, 3), dtype=np.uint8)
        self.bgr_frame[:] = (0, 255, 0)  # BGR format: Blue=0, Green=255, Red=0
        
    def read_frame(self):
        return self.bgr_frame
    

class BevyCamera:
    def __init__(self, name, width=512, height=512, channels=4) -> None:
        try:
            self.shm = shared_memory.SharedMemory(name=name)
        except:
            raise Exception("simulation is not running - cannot access shared memeory")
        self.frame = np.ndarray((height, width, channels), dtype=np.uint8, buffer=self.shm.buf)

    def read_frame(self):
        # bgr_frame = cv2.cvtColor(self.frame, cv2.COLOR_RGBA2BGR)
        return self.frame
    
    def close(self):
        # todo: call this somewhere
        self.shm.close()


class ImageProcessor:
    def __init__(self, input_count) -> None:
        self.image_read_functions = [None] * input_count
        self.input_count = input_count
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('image', 300, 500)

    def set_frame_provider(self, image_read_function, index: int):
        self.image_read_functions[index] = image_read_function

    def get_frame(self, index: int):
        read_function = self.image_read_functions[index]
        if not read_function: 
            raise Exception(f"Set image read function for index {index}.")
        return read_function()

    def update(self):
        frames = []
        for i in range(self.input_count):
            frames.append(self.get_frame(i))
        combined = np.hstack(frames)
        cv2.imshow('image', combined)

        # don't delete
        if cv2.waitKey(1) == ord('q'):
            self.quit()
            exit()

    def quit(self):
        # todo: close all sources
        # Release the capture and close windows
        cv2.destroyAllWindows()


if __name__ == '__main__':
    CAMERA_COUNT = 4
    image_processor = ImageProcessor(CAMERA_COUNT)

    for i in range(CAMERA_COUNT):
        bevy_camera= BevyCamera(f"cam{i}_frame")
        image_processor.set_frame_provider(bevy_camera.read_frame, i)

    while True:
        image_processor.update()

