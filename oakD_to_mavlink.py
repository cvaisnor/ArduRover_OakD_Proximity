'''Script for sending depth data from OAK-D to flight controller via MAVLink'''

# from https://discuss.ardupilot.org/t/simple-object-avoidance-using-oak-d-lite-camera-and-pymavlink-is-not-working/103145/9

import cv2
import numpy as np
import time

import depthai as dai
from pymavlink import mavutil

newConfig = False

########################## 
# Setup DepthAI pipeline #
##########################

# Create DepthAI pipeline
pipeline = dai.Pipeline()

# Define DepthAI sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Setting properties for depth AI pipeline
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Config
topLeft = dai.Point2f(0.45, 0.45)
bottomRight = dai.Point2f(0.55, 0.55)

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 400
config.depthThresholds.upperThreshold = 9000
calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
config.roi = dai.Rect(topLeft, bottomRight)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

# Create RGB camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(640, 360)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

# Create XLinkOut for RGB frames
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.preview.link(xoutRgb.input)

########################## 
# Setup MAVLINK connection #
##########################

# Create a MAVLink connection
jetson_nano = mavutil.mavlink_connection('/dev/ttyTHS1', baud=115200)

# Distance sensor frequency should be much faster than heartbeat
heartbeat_frequency = 1  # 1 Hz
distance_sensor_frequency = 40  # 40 Hz

last_heartbeat_time = time.time()
last_distance_sensor_time = time.time()

########################## 
# Setup video writer #
##########################

# Define the codec using VideoWriter_fourcc() and create a VideoWriter object.
# Ensure the frame size matches the dimensions of the frames you're writing, 
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# # Variables to write video to sd card
# video_file_path = '/home/tsungxu/video_test.mp4'
# frame_size = (640, 360)
# frame_rate = 30.0
# # instantiate cv2.VideoWriter before the loop
# out = cv2.VideoWriter(video_file_path, fourcc, frame_rate, frame_size)

##########################
# Connect to device and start pipeline
##########################

with dai.Device(pipeline) as device:

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    color = (255, 255, 255)

    # print("Use WASD keys to move ROI!")

    #!!! rgb output
    rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

        inRgb = rgbQueue.get()

        # !!! Convert the retrieved rgb frame into a numpy array
        frame = inRgb.getCvFrame()

        # Write the frame to the file
        out.write(frame)

        spatialData = spatialCalcQueue.get().getSpatialLocations()

        ##################
        # send heartbeat to flight controller #
        ##################

        # Send a heartbeat message every second. https://mavlink.io/en/mavgen_python/#heartbeat
        current_time = time.time()
        
        if current_time - last_heartbeat_time > heartbeat_frequency:
            # The arguments for the heartbeat message are type, autopilot, base_mode, custom_mode, system_status, and mavlink_version.
            jetson_nano.mav.heartbeat_send(
                18,  # Type of the system - MAV_TYPE_ONBOARD_CONTROLLER: https://mavlink.io/en/messages/minimal.html#MAV_TYPE
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                0,  # System mode https://mavlink.io/en/messages/common.html#MAV_MODE
                0,  # Custom mode, this is system specific
                3,  # System status https://mavlink.io/en/messages/common.html#MAV_STATE
            )
            last_heartbeat_time = current_time

        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=frame.shape[1], height=frame.shape[0])  # adjust this to frame's dimensions
            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            depthMin = depthData.depthMin
            depthMax = depthData.depthMax

            fontType = cv2.FONT_HERSHEY_TRIPLEX

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 1)
            cv2.putText(frame, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
            cv2.putText(frame, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
            cv2.putText(frame, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, color)

            ##################
            # send distance sensor data to flight controller #
            ##################

            # Get the depth in cm (assuming depthData.spatialCoordinates.z is in mm)
            depth = int(depthData.spatialCoordinates.z) / 10

            # Send DISTANCE_SENSOR message and reset last distance sensor time
            # see https://mavlink.io/en/messages/common.html for fields
            if current_time - last_distance_sensor_time > 1/distance_sensor_frequency:
                jetson_nano.mav.distance_sensor_send(
                    time_boot_ms=0,  # timestamp in ms since system boot
                    min_distance=40,  # minimum distance the sensor can measure (cm)
                    max_distance=900,  # maximum distance the sensor can measure (cm)
                    current_distance=int(depth),  # current distance measured (cm)
                    type=4,  # type of distance sensor: 0 = laser, 4 = unknown. See MAV_DISTANCE_SENSOR
                    id=1,  # onboard ID of the sensor
                    orientation=0,  # forward facing, see MAV_SENSOR_ORIENTATION
                    covariance=0,  # measurement covariance in centimeters, 0 for unknown / invalid readings
                )
                last_distance_sensor_time = current_time


            # # DEBUG print distance sensor data
            # if(depth > 300):
            #     print(depth)

        # !!! Show the rgb frame
        cv2.imshow("rgb", frame)

        key = cv2.waitKey(1) & 0xFF  # Get ASCII value of key
        if key == ord('q'):
            break

    # release video writer
    # out.release()