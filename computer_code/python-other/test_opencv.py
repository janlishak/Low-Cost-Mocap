# import cv2
# import numpy as np

# # Create a black image (480x640)
# img = np.zeros((480, 640, 3), dtype=np.uint8)

# cv2.imshow("test", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# import cv2
# import numpy as np

# points = np.array([[[0.0, 0.0]], [[1.0, 1.0]], [[2.0, 2.0]]], dtype=np.float32)
# K = np.eye(3, dtype=np.float32)

# try:
#     retval, Rs, Ts, points3d = cv2.sfm.reconstruct(points, K)
#     print("SFM module works.")
# except Exception as e:
#     print(f"SFM test failed: {e}")

import cv2
print(dir(cv2.sfm))