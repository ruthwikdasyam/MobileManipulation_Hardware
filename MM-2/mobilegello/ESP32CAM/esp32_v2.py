import cv2
import time
import torch

# Replace this with the correct ESP32-CAM stream URL
# url = "http://192.168.230.187:81/stream"
url = 4

# Open the video stream
cap = cv2.VideoCapture(url)

# Check if the stream is opened successfully
if not cap.isOpened():
    print(f"Error: Unable to open video stream at {url}")
    exit()

# Set up VideoWriter to save frames into a video file
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output.avi', fourcc, 10.0, (640, 480))  # 10 FPS, 640x480 resolution

# Set your desired FPS (frames per second)
fps = 10
frame_interval = 1 / fps

while True:
    # start_time = time.time()

    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to retrieve frame")
        break

    # Display the frame
    cv2.imshow("ESP32-CAM Stream", frame)
    print(frame.shape)
    image = torch.from_numpy(frame).permute([2, 0, 1])
    print(image.shape)
    # Write the current frame to the video file
    # out.write(frame)

    # Control the frame rate to match the desired FPS
    # elapsed_time = time.time() - start_time
    # if elapsed_time < frame_interval:
    #     time.sleep(frame_interval - elapsed_time)

    # Press 'q' to quit the stream window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture, video writer, and close OpenCV windows
cap.release()
# out.release()
cv2.destroyAllWindows()
