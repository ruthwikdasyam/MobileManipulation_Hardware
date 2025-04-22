import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        cp1 = time.time()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue
        
        cp2 = time.time()
        print(f"Time to get frame: {cp2 - cp1}")
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        color_image = color_image[100:420, 200:520]
        # print shape
        print(color_image.shape)

        # Display the image
        cv2.imshow('RealSense', color_image)

        # Break out of the loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
