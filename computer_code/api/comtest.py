import serial

def read_serial(port, baudrate=9600):
    """Simple program to read raw data from a serial port."""
    ser = serial.Serial(port, baudrate, timeout=1)

    try:
        while True:
            # Read a line from the serial port
            line = ser.readline()
            print(line)

    except KeyboardInterrupt:
        print("Exiting on user request.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    # Replace 'COM3' with your actual serial port name (e.g., '/dev/ttyUSB0' on Linux)
    read_serial(port='COM3')
