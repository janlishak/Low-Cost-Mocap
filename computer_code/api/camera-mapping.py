from pseyepy import Camera
 
# Initialize cameras
cameras = Camera()
 
# Dictionary to store camera mapping by unique identifier
camera_mapping = {}

print(cameras)
 
# # Loop through each camera and get its unique identifier
# for index, cam in enumerate(cameras):
#     unique_id = cam.ps3eye_get_unique_identifier()  # Retrieve the unique identifier
#     camera_mapping[unique_id] = index  # Store in dictionary with unique ID as the key
 
# # Print out the mapping
# print("Camera Mapping (Unique ID -> Index):")
# for unique_id, index in camera_mapping.items():
#     print(f"Camera {index}: Unique ID = {unique_id}")
 
# # Clean up
# cameras.release()