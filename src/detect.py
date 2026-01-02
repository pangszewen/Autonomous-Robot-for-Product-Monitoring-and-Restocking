#!/usr/bin/env python

import rospy
import cv2
import threading
from sensor_msgs.msg import Image
from fyp_pang.msg import Boundingbox
from cv_bridge import CvBridge, CvBridgeError
from fyp_pang.srv import StartDetection, StartDetectionResponse, StartMonitoring, StartMonitoringResponse
import ultralytics
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
        
        self.model = ultralytics.YOLO('/home/mustar/catkin_ws/src/fyp_pang/src/best_test2.pt')
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.service = rospy.Service('startDetect', StartDetection, self.handle_detection_request)
        self.service = rospy.Service('startMonitoring', StartMonitoring, self.handle_monitoring_request)
        self.cv_image = None 
        self.GRIPPER_REGION = (256, 450, 357, 472)
        self.display_image = None 
        self.latest_frame = None
        self.lock = threading.Lock()
        self.product_db_ref = db.reference('product_counts')
        self.stock_db_ref = db.reference('stock_counts')
        
        self.OBJECT_CATEGORIES = {
            'drinks': ['water', 'coffee', 'juice', 'milk', 'soda'],
            'food': ['tuna', 'cup noodle', 'cereal', 'jam', 'yogurt'],
            'snacks': ['biscuits', 'chips', 'chocolate']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'level 1',                   
            'food': 'level 2',                  
        }

        self.OBJECT_NAMES = {
            0: "chips",
            1: "juice",
            2: "milk",
            3: "water",
            4: "yogurt"
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
        self.CONFIDENCE_THRESHOLD = 0.7  
        self.EXISTING_CLASSES = ['milk', 'chips', 'juice', 'yogurt']

        # Start detection and display threads
        threading.Thread(target=self.detection_loop, daemon=True).start()

    def image_callback(self, msg_color):
        """Only store latest frame, no detection here."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg_color, "bgr8")
            #frame = np.flip(frame, axis=1)
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
                self.detect_objects(frame_copy)
            
            rate.sleep()
    
    def detect_objects(self, frame):
        """Run YOLO and update display_image."""
        results = self.model(frame, conf=self.CONFIDENCE_THRESHOLD, show=False)
        detected_objects = []

        if results[0].boxes:
            for box in results[0].boxes.data:
                x1, y1, x2, y2, conf, cls = box[:6]
                cls = int(cls)      
                
                if cls in self.OBJECT_NAMES:
                    if not self.is_in_gripper_region(x1, y1, x2, y2):
                        detected_objects.append({
                            'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
                            'confidence': float(conf), 'class_id': cls,
                            'class_name': self.OBJECT_NAMES[cls]
                        })
                    
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
  
        with self.lock:
            self.display_image = frame
            self.cv_image = frame

        return detected_objects
        

    def is_in_gripper_region(self, bx1, by1, bx2, by2):
        gx1, gy1, gx2, gy2 = self.GRIPPER_REGION
        # Return True if the bounding box overlaps with the gripper region
        return not (bx2 < gx1 or bx1 > gx2 or by2 < gy1 or by1 > gy2)
    
    def count_objects(self, detected_objects):
        object_counts = {}
        for obj in detected_objects:
                obj_class = obj['class_name']
                if obj_class in self.EXISTING_CLASSES:
                    object_counts[obj_class] = object_counts.get(obj_class, 0) + 1
        return object_counts
    
    def find_object_position(self, target_class, detected_objects):
        found = any(obj['class_name'] == target_class for obj in detected_objects)
        if found:
            filtered = [obj for obj in detected_objects if obj['class_name'] == target_class]
            best_object = max(filtered, key=lambda obj: obj['confidence'])
            return best_object
        else:
            for i in range(10):
                detected_objects = self.detect_objects(self.latest_frame.copy())
                if any(obj['class_name'] == target_class for obj in detected_objects):
                    filtered = [obj for obj in detected_objects if obj['class_name'] == target_class]
                    best_object = max(filtered, key=lambda obj: obj['confidence'])
                    return best_object
        return None
    
    def update_stock_firebase(self, stock_counts):
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
        print("Monitoring Request Received")
        response = StartMonitoringResponse()
        detected_objects = self.detect_objects(self.latest_frame.copy())
        rospy.loginfo(f"Detection completed, found {len(detected_objects)} objects")
        
        # Count detected objects
        object_counts = self.count_objects(detected_objects) if detected_objects else {}
        # Add zero counts for any classes not detected
        for cls in self.EXISTING_CLASSES:
            if cls not in object_counts:
                object_counts[cls] = 0
        
        self.update_product_firebase(object_counts)
        
        # Create a dictionary for low-stock objects (count < 3)
        low_stock = {obj: count for obj, count in object_counts.items() if count < 3}
        if low_stock:
            response.low_stock_name = list(low_stock.keys())
            response.low_stock_count = list(low_stock.values())
        else:
            response.low_stock_name = []
            response.low_stock_count = []
            
        return response

    def get_all_detections(self, detected_objects):
        response = StartDetectionResponse(
            xmin=0, xmax=0, ymin=0, ymax=0,
            class_name="", success=True,
            message=f"Returning all detected objects"
        )
        for obj in detected_objects:
            msg_obj = Boundingbox()
            msg_obj.xmin = obj['x1']
            msg_obj.ymin = obj['y1']
            msg_obj.xmax = obj['x2']
            msg_obj.ymax = obj['y2']
            msg_obj.class_name = obj['class_name']
                    
            response.objects.append(msg_obj)
        return response

    def pickup_mode(self, target_class, detected_objects, update):
        object_counts = self.count_objects(detected_objects)
        if update:
            self.update_stock_firebase(object_counts)
                    
        best_object = self.find_object_position(target_class, detected_objects)
        if best_object:
            response = StartDetectionResponse(
                xmin=best_object['x1'],
                xmax=best_object['x2'],
                ymin=best_object['y1'],
                ymax=best_object['y2'],
                class_name=best_object['class_name'],
                success=True,
                message=f"Returning best detected object of class {target_class}",
                objects = None
            )
        else:
            response = StartDetectionResponse(
                xmin=0, xmax=0, ymin=0, ymax=0,
                class_name="", success=False,
                message=f"No objects of class {target_class} detected",
                objects = None
            )

        rospy.loginfo(f"Detected object: {best_object['class_name']} with confidence {best_object['confidence']:.2f}")
        return response

    def place_mode(self, target_class, detected_objects):
        placement = self.find_object_position(target_class, detected_objects)    
        if placement:
            response = StartDetectionResponse(
                xmin=placement['x1'],
                xmax=placement['x2'],
                ymin=placement['y1'],
                ymax=placement['y2'],
                class_name=placement['class_name'],
                success=True,
                message=f"Returning best detected object of class {target_class}",
                objects = None
            )
        else:
            response = StartDetectionResponse(
                xmin=0, xmax=0, ymin=0, ymax=0,
                class_name="", success=False,
                message=f"No objects of class {target_class} detected",
                objects = None
            )
        return response
    
    def all_detections_mode(self, target_class, detected_objects):
        if target_class == "":     
            response = self.get_all_detections(detected_objects)
        else:
            print("Filtering for target class in all mode")
            filtered = [obj for obj in detected_objects if obj['class_name'] == target_class]
            response = self.get_all_detections(filtered)
        return response
        
    def handle_detection_request(self, req):
        mode = req.mode
        target_class = req.class_name
        update = req.startdetect

        max_wait_time = 5.0
        wait_start = rospy.Time.now()

        while self.latest_frame is None and (rospy.Time.now() - wait_start).to_sec() < max_wait_time:
            rospy.sleep(0.1)

        if self.latest_frame is None:
            return StartDetectionResponse(0, 0, 0, 0, "", False, "No camera image available", None)

        try:
            detected_objects = self.detect_objects(self.latest_frame.copy())
            rospy.loginfo(f"Detection completed, found {len(detected_objects)} objects")
            
            if mode == 'all':
                response = self.all_detections_mode(target_class, detected_objects)
                return response

            if detected_objects:
                if mode == 'pickup':
                    response = self.pickup_mode(target_class, detected_objects, update)
                elif mode == 'place':
                    response = self.place_mode(target_class, detected_objects)
                    
                return response
            else:
                rospy.logwarn("No detected objects")
                return StartDetectionResponse(0, 0, 0, 0, "", False, "No detected objects", None)
        except Exception as e:
            return StartDetectionResponse(0, 0, 0, 0, "", False, str(e), None)
            
        

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

        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error with yolondistance.py")