import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient  # Ensure this import is included

# Initialize ROS node
rospy.init_node('yolo_realsense_publisher', anonymous=True)
image_pub = rospy.Publisher('/output_image', Image, queue_size=10)
bridge = CvBridge()

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Initialize the RoboFlow InferenceHTTPClient
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="hbbXDBkmtSzM6Pp4AjaF"
)

# Set up RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

depth_scale = 0.0010000000474974513  # Depth scale factor

rate = rospy.Rate(30)  # 30 Hz loop
while not rospy.is_shutdown():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale  # Convert to meters

    # Send the captured image to RoboFlow for inference
    result = CLIENT.infer(color_image, model_id="poop-detector-2/1")

    # Print the full result to inspect the structure
    print("Inference result:", result)

    # Process RoboFlow inference result
    if result and 'predictions' in result:
        predictions = result['predictions']
        print("Predictions:", predictions)  # Print out the predictions to understand the structure
        for prediction in predictions:
            print("Prediction:", prediction)  # Debug: See individual prediction details

            # Extract coordinates (assuming format x, y, width, height, etc.)
            x = int(prediction.get('x', 0))
            y = int(prediction.get('y', 0))
            width = int(prediction.get('width', 0))
            height = int(prediction.get('height', 0))
            label = prediction.get('class', 'Unknown')
            confidence = prediction.get('confidence', 0)

            # Convert to (x1, y1, x2, y2) format
            x1 = x
            y1 = y
            x2 = x + width
            y2 = y + height

            # Print out the bounding box coordinates for debugging
            print(f"Bounding box: x1={x1}, y1={y1}, x2={x2}, y2={y2}")

            # Ensure that coordinates are within image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(color_image.shape[1], x2), min(color_image.shape[0], y2)

            # Compute object depth
            object_depth = np.median(depth_image[y1:y2, x1:x2])  # Depth within bounding box
            label_text = f"{label}: {object_depth:.2f}m, {confidence*100:.2f}%"

            # Draw bounding box and label
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (252, 119, 30), 2)
            cv2.putText(color_image, label_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

            print(f"{label}: {object_depth:.2f}m, Confidence: {confidence*100:.2f}%")
    else:
        print("No predictions found in the inference result.")

    # Convert image to ROS message and publish
    ros_image = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
    image_pub.publish(ros_image)

    # Display image with OpenCV
    cv2.imshow("YOLO RealSense Output", color_image)
    cv2.waitKey(1)

    rate.sleep()

