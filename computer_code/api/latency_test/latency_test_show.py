import cv2

# Open the default camera (usually the first one)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # 0.25 = manual, 0.75 = auto
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75) # 0.25 = manual, 0.75 = auto
cap.set(cv2.CAP_PROP_EXPOSURE, -7)

# for val in range(-20, 0):
#     cap.set(cv2.CAP_PROP_EXPOSURE, val)
#     print(f"Set exposure to {val}")
#     ret, frame = cap.read()
#     cv2.imshow('Exposure Test', frame)
#     cv2.waitKey(3000)
   
exit(0)
if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame")
        break

    # frame = cv2.cvtColor(frame, cv2.cvt)

    # Display the resulting frame
    cv2.imshow('Camera Feed', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
