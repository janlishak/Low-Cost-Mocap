from multiprocessing import shared_memory

i = 0
name = f"cam{i}_frame"
shm = shared_memory.SharedMemory(name)
shm.close()