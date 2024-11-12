#optimized + buzzer + mqtt(for serial monitor) also for lcd + led

import cv2
from threading import Thread
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
from datetime import datetime

# MQTT broker details
broker = "broker.hivemq.com"  # You can use any broker like Mosquitto
port = 1883
topic = "object_detection/topic"

# Threshold for object detection
thres = 0.45
# Load class names
classNames = []
classFile = "/home/pi/Desktop/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")
# Paths to model configuration and weights
configPath = "/home/pi/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/Desktop/Object_Detection_Files/frozen_inference_graph.pb"
# Initialize DNN model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Setup GPIO for buzzer (GPIO pin 3)
GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.OUT)

# MQTT Publisher function
def publish_object_detection(detected):
    client = mqtt.Client("RaspberryPi")  # Create a new MQTT client
    client.connect(broker, port)  # Connect to the broker
    
    if detected:
        message = "Object detected"
    else:
        message = "Object not detected"
    
    result = client.publish(topic, message)
    status = result[0]
    
    if status == 0:
        print(f"Sent `{message}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

# Function for object detection
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    if len(objects) == 0:
        objects = classNames
    objectInfo = []
    detected = False # Variable to track if the object is detected
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                detected = True # Mark as detected
                objectInfo.append([box, className])
                if draw:
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                    cv2.putText(img, className.upper(), (box[0] + 10, box[1] + 30),
                               cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                               cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    return img, objectInfo, detected

# Video capture thread to avoid frame drop
class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(3, 320) # Set width
        self.cap.set(4, 240) # Set height
        self.ret, self.frame = self.cap.read()
        self.stopped = False
    
    def start(self):
        Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True
        self.cap.release()

# Main code
if __name__ == "__main__":
    cap = VideoStream().start()
    frame_skip = 3 # Process every 3rd frame
    count = 0
    buzzer_on = False # Flag to track the state of the buzzer
    
    while True:
        img = cap.read()
        if count % frame_skip == 0:
            result, objectInfo, detected = getObjects(img, thres, nms=0.2, objects=['bird','cat','dog'])
            
            # Control the buzzer based on detection
            if detected and not buzzer_on:
                GPIO.output(3, GPIO.HIGH) # Turn on buzzer
                buzzer_on = True
                publish_object_detection(True) # Publish to MQTT
            elif not detected and buzzer_on:
                GPIO.output(3, GPIO.LOW) # Turn off buzzer
                buzzer_on = False
                publish_object_detection(False) # Publish to MQTT
        
        cv2.imshow("Output", result)
        count += 1
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cap.stop()
    GPIO.cleanup() # Clean up GPIO to release resources
    cv2.destroyAllWindows()