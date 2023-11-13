# from: https://discuss.ardupilot.org/t/simple-object-avoidance-using-oak-d-lite-camera-and-pymavlink-is-not-working/103145/10

import cv2
import numpy as np
import time

import depthai as dai
from pymavlink import mavutil

########################## 
# Setup DepthAI pipeline #
##########################

pipeline = dai.Pipeline()

# Define sources and outputs
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

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Config
topLeft = dai.Point2f(0.4, 0.6)
bottomRight = dai.Point2f(0.6, 0.8)

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 100
config.depthThresholds.upperThreshold = 10000
calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
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
jetson_nano = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)

# Distance sensor frequency should be much faster than heartbeat
HEARTBEAT_FREQUENCY = 1  # 1 Hz
DISTANCE_SENSOR_FREQUENCY = 30  # 30 Hz

last_heartbeat_time = time.time()
last_distance_sensor_time = time.time()

##########################
# Connect to device and start pipeline
##########################

with dai.Device(pipeline) as device:

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    color = (255, 255, 255)

    #!!! rgb output
    rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

        inRgb = rgbQueue.get()

        # Convert the retrieved rgb frame into a numpy array
        frame = inRgb.getCvFrame()

        spatialData = spatialCalcQueue.get().getSpatialLocations()

        ##################
        # send heartbeat to flight controller
        ##################

        # Send a heartbeat message every second. https://mavlink.io/en/mavgen_python/#heartbeat
        current_time = time.time()
        
        if current_time - last_heartbeat_time > HEARTBEAT_FREQUENCY:
            # The arguments for the heartbeat message are type, autopilot, base_mode, custom_mode, system_status, and mavlink_version.
            jetson_nano.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                            0,
                            0,
                            0)
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

            coords = depthData.spatialCoordinates # X, Y, Z in mm
            distance_in_meters = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2) / 1000  # convert to m
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2) / 10  # convert to cm (needed for MAVLink message)

            fontType = cv2.FONT_HERSHEY_TRIPLEX
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, thickness=2)

            # Print distance in meters and draw bounding box around ROI

            cv2.putText(frame, "{:.1f} m".format(distance_in_meters), (xmin + 10, ymin + 20), fontType, 0.6, color)

            ##################
            # send distance sensor data to flight controller
            ##################
            
            # https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
            if current_time - last_distance_sensor_time > 1/DISTANCE_SENSOR_FREQUENCY:
                jetson_nano.mav.distance_sensor_send(
                    time_boot_ms=0,  # timestamp in ms since system boot
                    min_distance=40,  # minimum distance the sensor can measure (cm)
                    max_distance=900,  # maximum distance the sensor can measure (cm)
                    current_distance=int(distance),  # current distance measured (cm)
                    type=4,  # type of distance sensor: 0 = laser, 4 = unknown. See MAV_DISTANCE_SENSOR
                    id=1,  # onboard ID of the sensor
                    orientation=0,  # forward facing, see MAV_SENSOR_ORIENTATION
                    covariance=0,  # measurement covariance in centimeters, 0 for unknown / invalid readings
                )
                # reset last distance sensor time
                last_distance_sensor_time = current_time


            # DEBUG print distance sensor data
            print('Distance Reading:', distance, 'm')

        # !!! Show the frame
        cv2.imshow("RGB", frame)

        key = cv2.waitKey(1) & 0xFF  # Get ASCII value of key
        if key == ord('q'):
            break