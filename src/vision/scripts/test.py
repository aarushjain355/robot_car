import cv2

cap = cv2.VideoCapture(0)  # Assuming you are capturing frames from a camera

ret, frame = cap.read()  # Read a frame

frame_height, frame_width, _ = frame.shape  # Extract the height and width of the frame

print("Frame size: {}x{}".format(frame_width, frame_height))

cap.release()  # Release the capture
