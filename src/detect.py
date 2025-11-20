#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import torch
import os
import threading
from gtts import gTTS
from collections import Counter
from sensor_msgs.msg import Image
# from test_grocery.msg import Boundingbox
from std_msgs.msg import String
from enum import Enum
from cv_bridge import CvBridge, CvBridgeError
from fyp_pang.srv import StartDetection, StartDetectionResponse, LatestDetection, LatestDetectionResponse, StartMonitoring, StartMonitoringResponse
import ultralytics
import math
import firebase_admin
from firebase_admin import credentials, db

# Initialize Firebase
cred = credentials.Certificate("/home/mustar/catkin_ws/src/fyp_pang/src/serviceAccountKey.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': "https://product-monitoring-fe713-default-rtdb.asia-southeast1.firebasedatabase.app/"
})


class GroceryDetection:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('start_detection')
        
        # Load the trained YOLO model - using the same model as ObjectPickerGUI
        # self.model = ultralytics.YOLO('/home/mustar/catkin_ws/src/robot_mouse_control/scripts/9125.pt')
        self.model = ultralytics.YOLO('/home/mustar/catkin_ws/src/fyp_pang/src/best_test.pt')
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.CONFIDENCE_THRESHOLD = 0.7  # Using same confidence as ObjectPickerGUI
        self.service = rospy.Service('startDetect', StartDetection, self.handle_detection_request)
        self.service = rospy.Service('latestDetect', LatestDetection, self.get_latest_detections)
        self.service = rospy.Service('startMonitoring', StartMonitoring, self.handle_monitoring_request)
        self.cv_image = None 
        self.GRIPPER_REGION = (-1, -1, -1, -1)
        self.display_image = None  # Initialize display image
        self.latest_frame = None
        self.lock = threading.Lock()
        self.product_db_ref = db.reference('product_counts')
        self.stock_db_ref = db.reference('stock_counts')
        
        # Object categories for storing groceries task - NEW MODEL
        self.OBJECT_CATEGORIES = {
            'drinks': ['water', 'coffee', 'juice', 'milk', 'soda'],
            'food': ['tuna', 'cup noodle', 'cereal', 'jam', 'yogurt'],
            'snacks': ['biscuits', 'chips', 'chocolate']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'level 1',                   # Middle shelf, left side
            'food': 'level 2',                   # Bottom shelf, middle (heavy items)
        }

        # YOLO model class mappings
        self.OBJECT_NAMES = {
            0: "cup noodle",
            1: "juice",
            2: "milk",
            3: "water",
        }
        '''
        self.OBJECT_NAMES = {
            0: "biscuits",
            1: "cereal",
            2: "chips",
            3: "chocolate",
            4: "coffee",
            5: "cup noodle",
            6: "jam",
            7: "juice",
            8: "milk",
            9: "soda",
            10: "tuna",
            11: "water",
            12: "yogurt"
        }
        '''

        self.LOW_STOCK_THRESHOLD = 2
        self.EXISTING_CLASSES = ['water', 'milk', 'cup noodle', 'juice']

        # announcement control
        self.last_announced_detection = None
        self.announcement_cooldown = 3.0  # seconds between announcements
        self.last_announcement_time = 0

        # Start detection and display threads
        threading.Thread(target=self.detection_loop, daemon=True).start()

    def image_callback(self, msg_color):
        """Only store latest frame, no detection here."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg_color, "bgr8")
            frame = np.flip(frame, axis=1)
            with self.lock:
                self.latest_frame = frame
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def detection_loop(self):
        """Run YOLO detection in a separate thread."""
        rate = rospy.Rate(10)  # Run detection at max 10 FPS
        while not rospy.is_shutdown():
            frame_copy = None
            with self.lock:
                if self.latest_frame is not None:
                    frame_copy = self.latest_frame.copy()

            if frame_copy is not None:
                self.update_display_image(frame_copy)
                #detections = self.detect_objects(frame_copy)
                #self.update_product_firebase(detections)
            
            rate.sleep()
    
    def update_display_image(self, frame):
        """Run YOLO and update display_image."""
        results = self.model(frame, conf=self.CONFIDENCE_THRESHOLD, show=False)

        if results[0].boxes:
            for box in results[0].boxes.data:
                x1, y1, x2, y2, conf, cls = box[:6]
                cls = int(cls)
                if cls in self.OBJECT_NAMES:
                    if self.is_in_gripper_region(x1, y1, x2, y2):
                        color = (0, 0, 255)
                        status = " (IN GRIPPER)"
                    else:
                        color = (0, 255, 0)
                        status = ""
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv2.putText(frame, f"{self.OBJECT_NAMES[cls]} ({conf:.2f}){status}",
                                (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        gx1, gy1, gx2, gy2 = self.GRIPPER_REGION
        cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (255, 0, 0), 2)
        cv2.putText(frame, "GRIPPER REGION", (gx1, gy1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        with self.lock:
            self.display_image = frame
            self.cv_image = frame

    def is_in_gripper_region(self, bx1, by1, bx2, by2):
        # Define a fixed gripper region in image coordinates (tune values)
        gx1, gy1, gx2, gy2 = self.GRIPPER_REGION
        # Return True if the bounding box overlaps with the gripper region
        return not (bx2 < gx1 or bx1 > gx2 or by2 < gy1 or by1 > gy2)


    def detect_objects(self, frame):
        """
        Detect objects using the same logic as ObjectPickerGUI
        Returns list of detected objects with their bounding boxes
        """
        detected_objects = []
        
        # Run YOLO detection - same parameters as ObjectPickerGUI
        results = self.model(frame, conf=self.CONFIDENCE_THRESHOLD, show=False)
        print("Detecting")
        # Get image dimensions for coordinate correction
        img_height, img_width = frame.shape[:2]
        
        if results[0].boxes:
            for box in results[0].boxes.data:
                x1, y1, x2, y2, conf, cls = box[:6]
                cls = int(cls)      
                
                # Only process objects that are in our defined classes
                if cls in self.OBJECT_NAMES:
                    # Since the image was flipped horizontally, we need to correct the x-coordinates
                    # Convert flipped coordinates back to original coordinates
                    corrected_x1 = img_width - int(x2)  # x2 becomes x1 after flip correction
                    corrected_x2 = img_width - int(x1)  # x1 becomes x2 after flip correction
                    corrected_y1 = int(y1)  # y coordinates don't change with horizontal flip
                    corrected_y2 = int(y2)
                    
                    if not self.is_in_gripper_region(x1, y1, x2, y2):
                        print(f"{self.OBJECT_NAMES[cls]} not in gripper region")
                        detected_objects.append({
                            'x1': corrected_x1, 'y1': corrected_y1, 'x2': corrected_x2, 'y2': corrected_y2,
                            'confidence': float(conf), 'class_id': cls,
                            'class_name': self.OBJECT_NAMES[cls]
                        })
                    
                        # Draw bounding box on frame (using original flipped coordinates for visualization)
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            f"{self.OBJECT_NAMES[cls]} ({conf:.2f})",
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )
        # Draw gripper region for reference
        gx1, gy1, gx2, gy2 = self.GRIPPER_REGION
        cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (0, 0, 255), 2)
        self.cv_image = frame.copy()
        
        return detected_objects

    def get_latest_detections(self, req):
        with self.lock:
            if self.latest_frame is None:
                return LatestDetectionResponse(
                    xmin=[], xmax=[], ymin=[], ymax=[],
                    confidence=[], class_name=[],
                    success=False,
                    message="No camera image available"
                )
            frame_copy = self.latest_frame.copy()

        detections = self.detect_objects(frame_copy)

        if not detections:
            return LatestDetectionResponse(
                xmin=[], xmax=[], ymin=[], ymax=[],
                confidence=[], class_name=[],
                success=False,
                message="No objects detected"
            )

        return LatestDetectionResponse(
            xmin=[obj['x1'] for obj in detections],
            xmax=[obj['x2'] for obj in detections],
            ymin=[obj['y1'] for obj in detections],
            ymax=[obj['y2'] for obj in detections],
            confidence=[obj['confidence'] for obj in detections],
            class_name=[obj['class_name'] for obj in detections],
            success=True,
            message=f"Detected {len(detections)} objects"
        )

    def get_object_category(self, object_name):
        """
        Determine which category an object belongs to based on its name
        Returns the category name or 'unknown' if not found
        """
        object_name_lower = object_name.lower()
        
        for category, objects in self.OBJECT_CATEGORIES.items():
            if object_name_lower in [obj.lower() for obj in objects]:
                return category
        
        # If not found in predefined categories, check generic object types
        if object_name_lower in ['cup', 'bowl', 'plate', 'fork', 'knife', 'spoon']:
            return 'dishes'
        elif object_name_lower in ['bottle', 'can']:
            return 'drinks'  # Assuming bottles/cans are drinks
        
        return 'unknown'

    def announce_object_category(self, object_name):
        """
        Announce the category of the detected object and its storage location
        """
        category = self.get_object_category(object_name)
        
        if category != 'unknown':
            announcement = f"I detected a {object_name} which belongs to the {category} category."
        else:
            announcement = f"I detected a {object_name} but I'm not sure which category it belongs to."
        
        return announcement
    
    def count_objects(self, detected_objects):
        """
        Count occurrences of each detected object class
        Returns a dictionary with class names as keys and counts as values
        """
        object_counts = {}
        for obj in detected_objects:
                obj_class = obj['class_name']
                if obj_class in self.EXISTING_CLASSES:
                    object_counts[obj_class] = object_counts.get(obj_class, 0) + 1
        return object_counts
    
    def find_object_position(self, target_class, detected_objects):
        """
        Find the position of the target class in the detected objects
        Returns the bounding box coordinates if found, else None
        """
        found = any(obj['class_name'] == target_class for obj in detected_objects)
        if found:
            print("Found target class")
            filtered = [obj for obj in detected_objects if obj['class_name'] == target_class]
            best_object = max(filtered, key=lambda obj: obj['confidence'])
            return best_object
        else:
            for i in range(3):
                detected_objects = self.detect_objects(self.latest_frame.copy())
                if target_class in detected_objects:
                    filtered = [obj for obj in detected_objects if obj['class'] == target_class]
                    best_object = max(filtered, key=lambda obj: obj['confidence'])
                    break
                if i == 2:
                    return None
    
    def update_stock_firebase(self, stock_counts):
        """Update the Firebase database with detection counts for all stocks."""
        try:
            # Fetch all existing classes from Firebase
            existing_data = self.stock_db_ref.get()
            existing_classes = existing_data.keys() if existing_data else []

            # Update Firebase with detected classes
            for class_name, count in stock_counts.items():
                self.stock_db_ref.child(class_name).set({'count': count})

            # Set missing classes (no longer detected) to 0
            for class_name in existing_classes:
                if class_name not in stock_counts:
                    self.stock_db_ref.child(class_name).set({'count': 0})

            rospy.loginfo(f"Updated Firebase with all detected stocks: {stock_counts}")
        except Exception as e:
            rospy.logerr(f"Failed to update Firebase: {e}")

    def update_product_firebase(self, product_counts):
        """Update the Firebase database with detection counts for all products."""
        try:
            # Fetch all existing classes from Firebase
            existing_data = self.product_db_ref.get()
            existing_classes = existing_data.keys() if existing_data else []

            # Update Firebase with detected classes
            for class_name, count in product_counts.items():
                self.product_db_ref.child(class_name).set({'count': count})

            # Set missing classes (no longer detected) to 0
            for class_name in existing_classes:
                if class_name not in product_counts:
                    self.product_db_ref.child(class_name).set({'count': 0})

            rospy.loginfo(f"Updated Firebase with all detected products: {product_counts}")
        except Exception as e:
            rospy.logerr(f"Failed to update Firebase: {e}")
    
    def handle_monitoring_request(self, req):
        print("Start Monitoring Request Received")
        response = StartMonitoringResponse()
        detected_objects = self.detect_objects(self.latest_frame.copy())
        print("Passing to firebase")
        
        rospy.loginfo(f"Detection completed, found {len(detected_objects)} objects")
        if detected_objects:
            # Count how many times each object appears
            object_counts = self.count_objects(detected_objects)
            self.update_product_firebase(object_counts)

            # Create a dictionary for low-stock objects (below threshold)
            low_stock = {}
            for obj, count in object_counts.items():  # use .items() to get both key and value
                if count < 3:  # threshold condition
                    low_stock[obj] = count  # store the object and its count

            if low_stock:
                response.low_stock_name = list(low_stock.keys())
                response.low_stock_count = list(low_stock.values())
            else:
                return None
        else:
            low_stock = dict(map(lambda x: (x, 0), self.EXISTING_CLASSES))
            response.low_stock_name = list(low_stock.keys())
            response.low_stock_count = list(low_stock.values())
            
        return response

    def handle_detection_request(self, req):
        mode = req.mode
        target_class = req.class_name

        max_wait_time = 5.0
        wait_start = rospy.Time.now()

        while self.latest_frame is None and (rospy.Time.now() - wait_start).to_sec() < max_wait_time:
            rospy.sleep(0.1)

        if self.latest_frame is None:
            return StartDetectionResponse(0, 0, 0, 0, "", False, "No camera image available")

        try:
            if mode == 'place':
                self.GRIPPER_REGION = (274, 388, 372, 478) 
            else:
                # ADD THIS ELSE CLAUSE FOR PICKUP MODE
                rospy.loginfo("Clearing gripper region for pickup mode")
                self.GRIPPER_REGION = (0, 0, 0, 0)  # No gripper filtering during pickup

            detected_objects = self.detect_objects(self.latest_frame.copy())
            rospy.loginfo(f"Detection completed, found {len(detected_objects)} objects")

            message = ""
            if detected_objects:
                if mode == 'pickup':
                    object_counts = self.count_objects(detected_objects)
                    self.update_stock_firebase(object_counts)
                    
                    best_object = self.find_object_position(target_class, detected_objects)
                    print("target class:" , target_class)
                    print("best object:", best_object['class_name'])
                    if best_object:
                        response = StartDetectionResponse(
                            xmin=best_object['x1'],
                            xmax=best_object['x2'],
                            ymin=best_object['y1'],
                            ymax=best_object['y2'],
                            class_name=best_object['class_name'],
                            success=True,
                            message=message
                        )
                    else:
                        response = StartDetectionResponse(
                            xmin=0, xmax=0, ymin=0, ymax=0,
                            class_name="", success=False,
                            message=f"No objects of class {target_class} detected"
                        )

                    rospy.loginfo(f"Detected object: {best_object['class_name']} with confidence {best_object['confidence']:.2f}")
                    return response

                elif mode == 'place':

                    placement = self.find_object_position(target_class, detected_objects)    
                    if placement:
                        response = StartDetectionResponse(
                            xmin=placement['x1'],
                            xmax=placement['x2'],
                            ymin=placement['y1'],
                            ymax=placement['y2'],
                            class_name=placement['class_name'],
                            success=True,
                            message=message
                        )
                    else:
                        response = StartDetectionResponse(
                            xmin=0, xmax=0, ymin=0, ymax=0,
                            class_name="", success=False,
                            message=f"No objects of class {target_class} detected"
                        )
                    rospy.loginfo(f"Detected placement: {placement['class_name']} with confidence {placement['confidence']:.2f}")
                    return response
            else:
                rospy.logwarn("No detected objects")
                return StartDetectionResponse(0, 0, 0, 0, "", False, "No detected objects")
        except Exception as e:
            return StartDetectionResponse(0, 0, 0, 0, "", False, str(e))

    def speak(self, text):
        tts = gTTS(text=text, lang='en')
        tts.save("/tmp/detection.mp3")
        os.system("mpg123 /tmp/detection.mp3") 

    def announce_detections(self, detected_objects):
            
        if not detected_objects:
            self.speak("No objects detected")
            return

        # Count occurrences of each class name
        counts = Counter(obj['class_name'] for obj in detected_objects)

        # Build speech text
        parts = []
        for name, count in counts.items():
            if count == 1:
                parts.append(f"1 {name}")
            else:
                parts.append(f"{count} {name}s")  # plural form (basic)
        sentence = "I see " + " and ".join(parts)

        return sentence
        

if __name__ == "__main__":
    try:
        grocery = GroceryDetection()
        # Display loop in main thread
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            img = None
            with grocery.lock:
                if grocery.display_image is not None:
                    img = grocery.display_image.copy()

            if img is not None:
                cv2.imshow("YOLO Detections", img)
                cv2.waitKey(1)
            rate.sleep()

        cv2.destroyAllWindows()
        rospy.spin()
        # Make sure to properly handle shutdown
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error with yolondistance.py")