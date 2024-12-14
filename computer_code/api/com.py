import serial
import re
import threading
import time
import numpy as np
from scipy.optimize import fsolve

beacons = [
    [-1, 1, 0], # Beacon A0
    [ 1, 1, 0],  # Beacon A1
    [ 0,-1, 0], # Beacon A2
]

def equations(p, beacons, distances):
    x, y, z = p
    eq1 = (x - beacons[0][0])**2 + (y - beacons[0][1])**2 + (z - beacons[0][2])**2 - distances[0]**2
    eq2 = (x - beacons[1][0])**2 + (y - beacons[1][1])**2 + (z - beacons[1][2])**2 - distances[1]**2
    eq3 = (x - beacons[2][0])**2 + (y - beacons[2][1])**2 + (z - beacons[2][2])**2 - distances[2]**2
    return [eq1, eq2, eq3]

def triangulate(beacons, distances):
    initial_guess = np.mean(beacons, axis=0)
    solution = fsolve(equations, initial_guess, args=(beacons, distances))
    return solution


class SerialReader(threading.Thread):
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.anchors = {"an0": 3, "an1": 3, "an2": 3}
        self.anchor_regex = re.compile(r"^an(\d+):(\d+\.\d+)m$")

    def run(self):
        """Thread's main loop to read serial data."""
        self.running = True
        ser = serial.Serial(self.port, self.baudrate, timeout=1)

        try:
            while self.running:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()

                # Match the line against the regex
                match = self.anchor_regex.match(line)
                if match:
                    anchor_id = f"an{match.group(1)}"
                    distance = float(match.group(2))

                    # Update the corresponding anchor distance
                    if anchor_id in self.anchors:
                        self.anchors[anchor_id] = distance
                        l = [self.anchors["an0"], self.anchors["an1"], self.anchors["an2"]]
                        print(f"Updated {match.group(1)}: {distance}")
                        print(f"Would triangulate {l}")
                        print(beacons)
                        coordinates = triangulate(beacons, l)
                        print("Calculated coordinates:", coordinates)

        except Exception as e:
            print(f"Error: {e}")
        finally:
            ser.close()
            print("Serial port closed.")

    def stop(self):
        """Stops the thread."""
        self.running = False

if __name__ == "__main__":
    # Replace 'COM3' with your actual serial port name (e.g., '/dev/ttyUSB0' on Linux)
    serial_reader = SerialReader(port='/dev/ttyUSB3')
    serial_reader.start()

    time.sleep(10)

    # try:
    #     while True:
    #         # Main thread can access the anchors dictionary here
    #         print(serial_reader.anchors)

    # except KeyboardInterrupt:
    #     print("Exiting on user request.")
    #     serial_reader.stop()
    #     serial_reader.join()
