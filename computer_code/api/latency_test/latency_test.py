import av

device = "video=Camera Name"

for idx, stream in enumerate(av.devices.list_input_devices()):
    print(f"{idx}: {stream}")

# Open device
container = av.open(device, format="v4l2")  # use format="dshow" on Windows

for stream in container.streams.video:
    print(f"Resolution: {stream.width}x{stream.height}, FPS: {stream.average_rate}")