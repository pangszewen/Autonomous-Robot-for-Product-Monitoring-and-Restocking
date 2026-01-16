#!/usr/bin/env python
import rospy
import math
import numpy as np
import threading
import json
import os
from datetime import datetime
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from fyp_pang.srv import ArmHeadGripper, ArmHeadGripperResponse, Navigate
from fyp_pang.srv import StartDetection, StartDetectionRequest

class ArmManipulation:
    def __init__(self):
        rospy.init_node("head_arm_hand", anonymous=True)
        
        # Camera and sensor setup
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        # Publishers
        self.pub_arm1 = rospy.Publisher('/arm1_joint/command', Float64, queue_size=10)
        self.pub_arm2 = rospy.Publisher('/arm2_joint/command', Float64, queue_size=10)
        self.pub_arm3 = rospy.Publisher('/arm3_joint/command', Float64, queue_size=10)
        self.pub_arm4 = rospy.Publisher('/arm4_joint/command', Float64, queue_size=10)
        self.pub_gripper = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Robot parameters
        self.camera_height = 45.0
        self.camera_offset = 25.0
        self.camera_angle = 30
        self.L2, self.L3, self.L4 = 10.16, 10.16, 6.0
        self.JOINT_MIN = -1.0  # radians
        self.JOINT_MAX = 1.0
        self.alpha = 90
        self.gripper_z_offset =  self.L4 * math.cos(self.alpha)
        self.gripper_x_offset =  self.L4 * math.sin(self.alpha)

        # Current sensor data
        self.depth_frame = None
        self.lock = threading.Lock()
        self.latest_detection_result = None  
        self.gripper_region = {
            "xmin": 256,
            "ymin": 450,
            "xmax": 357,
            "ymax": 472
        }
        self.gripper_angle = 0.3
        
        # Navigation parameters
        self.center_threshold = 150
        self.distance_threshold = 70.0
        self.placement_distance_threshold = 60.0
        self.placement_center_radius = 100
        self.obstacle_distance_threshold = 0.5
        self.front_angle_range = 60
        
        # Movement parameters
        self.rotation_step = 0.3
        self.movement_step = 0.15
        self.step_duration = 0.05

        # Empty location detection parameters
        self.empty_location_search_radius = 200  # pixels
        self.min_empty_area_size = 100  # minimum pixels for empty area
        self.depth_threshold_for_empty = 5.0  # cm difference to consider empty
        self.max_rotation_for_empty_search = 360  # degrees
        self.rotation_step_for_search = 15  # degrees per search step
        self.preferred_placement_distance = 80.0  # cm preferred distance for placement

        self.viz_data_file = "/tmp/robot_viz_data.json"

        self.OBJECT_CATEGORIES = {
            'drinks': ['water', 'coffee', 'juice', 'milk', 'soda'],
            'food': ['tuna', 'cup noodle', 'cereal', 'jam', 'yogurt', 'biscuits', 'chips', 'chocolate']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'level 1',                   
            'food': 'level 2',
            'juice': 'left',
            'chips': 'left',
            'milk': 'right',
            'yogurt': 'right'                 
        }

        # Detection service client
        self.detection_client = None
        self.setup_detection_client()
        
        # Service server
        self.service = rospy.Service('arm_manipulation', ArmHeadGripper, self.handle_arm_manipulation)
        rospy.loginfo("Arm Manipulation Service initialized and ready")

    def publish_viz(self, object_class="", distance=0, theta2=0, theta3=0, theta4=0, positional_error=0):
        """Simple visualization publisher"""
        data = {
            "object_class": object_class,
            "distance": distance,
            "theta2": theta2,
            "theta3": theta3,
            "theta4": theta4,
            "positional_error": positional_error,
            "timestamp": datetime.now().strftime("%H:%M:%S")
        }
        
        try:
            with open(self.viz_data_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            rospy.logwarn(f"Viz update failed: {e}")

    def setup_detection_client(self):
        """Initialize the detection service client"""
        try:
            rospy.loginfo("Waiting for detection service...")
            rospy.wait_for_service('startDetect', timeout=10.0)
            self.detection_client = rospy.ServiceProxy('startDetect', StartDetection)
            rospy.loginfo("Detection service client initialized successfully")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to detection service: {e}")
            self.detection_client = None

    def setup_navigation_client(self, target_location):
        rospy.wait_for_service('navigate')
        try:
            get_location_service = rospy.ServiceProxy('navigate', Navigate)
            print("in navigation service")
            self.location_response = get_location_service(target_location)
            self.location_response_message = self.location_response.reach
            
            return self.location_response_message
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def get_fresh_detection(self, mode, class_name, update = False):                                       
        if self.detection_client is None:
            rospy.logerr("Detection service not available, attempting to reconnect...")
            self.setup_detection_client()
            if self.detection_client is None:
                return None

        try:
            request = StartDetectionRequest()
            request.mode = mode
            request.class_name = class_name
            request.startdetect = update
            
            rospy.loginfo(f"Requesting fresh detection: mode={mode}, class={class_name}")
            response = self.detection_client(request)
            
            if response.success:
                detection_result = {
                    'xmin': response.xmin,
                    'xmax': response.xmax,
                    'ymin': response.ymin,
                    'ymax': response.ymax,
                    'class_name': response.class_name,
                    'success': True,
                    'message': response.message,
                    'objects': response.objects
                }
                rospy.loginfo(f"Fresh detection successful: {response.class_name} at ({response.xmin}, {response.ymin}, {response.xmax}, {response.ymax})")
                
                with self.lock:
                    self.latest_detection_result = detection_result

                return detection_result
            else:
                rospy.logerr(f"Detection failed: {response.message}")
                return None
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Detection service call failed: {e}")
            return None
    
    def pick_mode(self, req):
        response = ArmHeadGripperResponse()
        message = None
        self.set_gripper_joint(req.class_name)
        self.approach_object(0, 0, 0, 0, req.class_name, 60)
        # Get fresh detection for pickup
        detection = self.get_fresh_detection("pickup", req.class_name)
        if detection is None:
            response.success = False
            response.message = "Failed to get fresh detection for pickup"
            return response
                
        success = self.execute_pick_sequence(
            detection['xmin'], detection['xmax'], 
            detection['ymin'], detection['ymax'], 
            detection['class_name']
        )

        if success:
            message = f"Successfully picked up the {req.class_name}"
        else:
            message = f"Failed to pick up the {req.class_name}"

        response.success = success
        response.message = message 
        return response
    
    def place_mode(self, req):
        response = ArmHeadGripperResponse()
        object_location = self.get_object_location(req.class_name)
        detection = self.get_fresh_detection("place", req.class_name)
        if detection:
            if object_location == "level 2":
                approached = self.approach_object(
                    detection['xmin'], detection['xmax'], 
                    detection['ymin'], detection['ymax'], 
                    detection['class_name'], 80)
            else:
                approached = self.approach_object(
                    detection['xmin'], detection['xmax'], 
                    detection['ymin'], detection['ymax'], 
                    detection['class_name'], 52)
        else:
            object_side = self.CATEGORY_LOCATIONS.get(req.class_name)
            if object_side == "left":
                self.setup_navigation_client("left_shelf")
            else:
                self.setup_navigation_client("right_shelf")
            if object_location == "level 2":
                success = self.move_to_level2_place_position(13)
            else:
                success = self.move_to_default_place_position(33)
            approached = False
            message = "Placement succeed for zero products on shelf"
    
        if approached:
            center_clear, forward = self.move_away_from_objects_to_center(req.class_name, object_location)
            if center_clear:
                rospy.loginfo("Center area is clear for placement")
                # If no coordinates are given, use default place position
                if object_location == "level 2":
                    rospy.loginfo("Using default place position")
                    success = self.move_to_level2_place_position(forward)
                else:
                    rospy.loginfo(f"Placing at given coordinates")
                    success = self.move_to_default_place_position(forward)
                        
                object_name = req.class_name if req.class_name else "object"
                message = f"I have placed down the {object_name} in the cabinet"
            else:
                # Fallback to rotation-based empty location search
                rospy.logerr("Center clearing failed")
                success = False
                message = "Could not find safe placement position"

        response.success = success
        response.message = message 
        return response

    def handle_arm_manipulation(self, req):
        """
        Service handler for arm manipulation requests
        """
        response = ArmHeadGripperResponse()
        try:
            rospy.loginfo(f"Received arm manipulation request: mode={req.mode}, class={req.class_name}")
            
            if req.mode.lower() == "pick":
                response = self.pick_mode(req)
                
            elif req.mode.lower() == "place":
                response = self.place_mode(req) 
            
        except Exception as e:
            rospy.logerr(f"Error in arm manipulation service: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        return response
    
    def get_object_location(self, class_name):
        # Find which category this object belongs to
        category = None
        for cat, items in self.OBJECT_CATEGORIES.items():
            if class_name in items:
                category = cat
                break
        
        # If category found, get its location
        if category:
            location = self.CATEGORY_LOCATIONS.get(category)
            return location
        else:
            rospy.logwarn(f"Object '{class_name}' not found in any category")
            return None
    
    def set_gripper_joint(self, class_name):
        if class_name == 'juice':
            self.gripper_angle = 0.25  # Open wider for drinks
        elif class_name in ['milk', 'yogurt']:
            self.gripper_angle = 0.33  # Tighter grip for milk
        elif class_name == 'chips':
            self.gripper_angle = 0  # Medium grip for chips
        else:
            self.gripper_angle = 0.3  # Default grip for other items

    def approach_object(self, xmin, xmax, ymin, ymax, class_name, stop_distance):
        """
        Move the robot in front of the detected object based on its coordinates.
        stop_distance: distance in meters to stop before object
        """
        print(f"Approaching {class_name} until {stop_distance} m away")
        # Get object's bounding box center
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        box_width = abs(xmax - xmin)
        box_height = abs(ymax - ymin)

        # Get distance from depth camera
        distance = self.get_robust_depth(center_x, center_y, box_width, box_height)
        if distance is None:
            rospy.logerr("No depth data available for approach.")
            return False

        # Rotate to center object in view
        image_center_x = self.depth_frame.shape[1] / 2
        error_x = center_x - image_center_x

        angular_speed = -0.002 * error_x 
        forward_speed = 0.5  # m/s

        twist = Twist()
        target_center = (center_x, center_y)
        # Keep moving until close enough
        while distance > stop_distance:
            # Rotate to align
            twist.angular.z = angular_speed
            twist.linear.x = forward_speed if abs(error_x) < 20 else 0  # move only if roughly centered
            self.pub_base.publish(twist)

            rospy.sleep(0.2)

            # Update detection & depth
            obj = self.get_closest_detection(class_name, target_center)
            if not obj:
                rospy.logwarn("Lost sight of object during approach.")
                continue
            
            center_x = (obj.xmin + obj.xmax) / 2
            center_y = (obj.ymin + obj.ymax) / 2
            box_width = abs(obj.xmax - obj.xmin)
            box_height = abs(obj.ymax - obj.ymin)
            target_center = (center_x, center_y)
            print("New center: ", center_x, center_y)
            distance = self.get_robust_depth(center_x, center_y, box_width, box_height)
            error_x = center_x - image_center_x
            angular_speed = -0.002 * error_x
            forward_speed = 0.1
            rospy.loginfo(f"Object {class_name} at distance: {distance:.2f} m")

            if not distance:
                return False

        # Stop movement
        self.pub_base.publish(Twist())
        rospy.loginfo(f"Reached approach distance {stop_distance:.2f} m from {class_name}")
        return True
    
    def get_closest_detection(self, class_name, last_position, max_distance=1000):
        """
        Get the detection of class_name closest to last_position.
        """
        detections = self.get_fresh_detection("all", class_name)
        
        if not detections['success']:
            print("No closest detection")
            return None
        
        last_x, last_y = last_position
        closest_obj = None
        min_distance = float('inf')

        for obj in detections['objects']:
            obj_center_x = (obj.xmin + obj.xmax) / 2
            obj_center_y = (obj.ymin + obj.ymax) / 2
            
            # Calculate Euclidean distance
            dist = ((obj_center_x - last_x)**2 + (obj_center_y - last_y)**2)**0.5
            
            if dist < min_distance and dist < max_distance:
                min_distance = dist
                closest_obj = obj
                print("Closest object found")
        
        return closest_obj

    def execute_pick_sequence(self, xmin, xmax, ymin, ymax, class_name):
        """Execute the complete pick sequence with navigation and grabbing"""
        rospy.loginfo(f"Starting pick sequence for {class_name}...")
        
        # Calculate bounding box properties
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        box_width = abs(xmax - xmin)
        box_height = abs(ymax - ymin)
        
        # Navigate to optimal position with periodic re-detection
        if not self.navigate_to_object_with_redetection("pickup", class_name, center_x, center_y, box_width, box_height):
            rospy.logwarn("Failed to navigate to object")
            return False
        
        # Get final fresh detection before pickup
        final_detection = self.get_fresh_detection("pickup", class_name)
        if final_detection is None:
            rospy.logwarn("Could not get final detection before pickup")
            return False
        
        # Update coordinates with final detection
        final_center_x = (final_detection['xmin'] + final_detection['xmax']) / 2
        final_center_y = (final_detection['ymin'] + final_detection['ymax']) / 2
        final_box_width = abs(final_detection['xmax'] - final_detection['xmin'])
        final_box_height = abs(final_detection['ymax'] - final_detection['ymin'])
        
        # Get final depth measurement
        distance = self.get_robust_depth(final_center_x, final_center_y, final_box_width, final_box_height)
        if distance is None:
            rospy.logwarn("Could not get valid depth measurement")
            return False
        
        # Align gripper and pick up object
        self.align_gripper_with_object(final_center_x)

        # Transform coordinates and execute grab
        robot_x, robot_y, robot_z = self.transform_to_robot_frame_depth(distance, final_center_x, final_center_y)
        success = self.pickup_object(robot_x, robot_z, self.alpha, class_name)
        self.move_forward(-5)
        
        rospy.loginfo(f"Pick sequence {'completed successfully' if success else 'failed'}")
        return success

    def navigate_to_object_with_redetection(self, mode, class_name, center_x, center_y, box_width, box_height):
        """Navigate robot to optimal picking position with periodic re-detection"""
        max_attempts = 50  
        attempt = 0
        redetection_interval = 10  # Re-detect every 10 attempts
        
        while attempt < max_attempts:
            attempt += 1
            
            # Periodic re-detection to get fresh coordinates
            if attempt % redetection_interval == 0:
                rospy.loginfo("Performing periodic re-detection during navigation...")
                fresh_detection = self.get_fresh_detection(mode, class_name)
                if fresh_detection is not None:
                    center_x = (fresh_detection['xmin'] + fresh_detection['xmax']) / 2
                    center_y = (fresh_detection['ymin'] + fresh_detection['ymax']) / 2
                    box_width = abs(fresh_detection['xmax'] - fresh_detection['xmin'])
                    box_height = abs(fresh_detection['ymax'] - fresh_detection['ymin'])
                    rospy.loginfo("Updated object coordinates from fresh detection")
            
            # Get current distance
            distance = self.get_robust_depth(center_x, center_y, box_width, box_height)
            if distance is None:
                rospy.logwarn("Cannot get distance to object")
                return False
            self.publish_viz(class_name, distance, 0, 0, 0)
            
            # Check if object is centered and reachable
            is_centered = self.check_object_centered(center_x)
            is_reachable = self.check_reachability(center_x, center_y, distance)
            
            rospy.loginfo(f"Navigation attempt {attempt}: Centered={is_centered}, Reachable={is_reachable}, Distance={distance:.1f}cm")
            
            if is_centered and is_reachable:
                rospy.loginfo("Object perfectly positioned!")
                return True
            
            if not is_centered:
                self.move_to_center_object(center_x)
                rospy.sleep(0.1)
            elif not is_reachable:
                self.move_forward(3)
        
        rospy.logwarn("Navigation failed - maximum attempts reached")
        return False

    def depth_callback(self, msg):
        try:            
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(f"Depth image conversion failed: {e}")
            self.depth_frame = None

    def get_depth_at_pixel(self, x, y):
        if self.depth_frame is None:
            rospy.logwarn("No depth frame available")
            return None
        
        height, width = self.depth_frame.shape
        # Ensure coordinates are within bounds
        x = int(max(0, min(x, width - 1)))
        y = int(max(0, min(y, height - 1)))
        
        try:
            depth_value = self.depth_frame[y, x]
            
            if np.isnan(depth_value) or depth_value == 0:
                rospy.logwarn(f"Invalid depth at pixel ({x},{y})")
                return None
            # Convert from millimeters to centimeters
            converted_depth = depth_value / 10.0
            return converted_depth
                 
        except Exception as e:
            rospy.logerr(f"Error getting depth at pixel ({x},{y}): {e}")
            return None

    def get_robust_depth(self, center_x, center_y, box_width, box_height):
        if self.depth_frame is None:
            rospy.logwarn("No depth frame available for robust depth calculation")
            return None
        
        # Increase sample radius and add more sample points
        sample_radius = max(min(box_width, box_height) * 0.3, 10)  # Minimum 10 pixel radius
        depth_values = []
        
        # Sampling pattern
        sample_points = [
            (center_x, center_y),  # Center
            # Cross pattern
            (center_x - sample_radius/2, center_y),
            (center_x + sample_radius/2, center_y),
            (center_x, center_y - sample_radius/2),
            (center_x, center_y + sample_radius/2),
            # Diagonal pattern
            (center_x - sample_radius/3, center_y - sample_radius/3),
            (center_x + sample_radius/3, center_y - sample_radius/3),
            (center_x - sample_radius/3, center_y + sample_radius/3),
            (center_x + sample_radius/3, center_y + sample_radius/3),
            # Wider sampling
            (center_x - sample_radius, center_y),
            (center_x + sample_radius, center_y),
            (center_x, center_y - sample_radius),
            (center_x, center_y + sample_radius),
        ]
        
        rospy.loginfo(f"Sampling depth around ({center_x:.1f}, {center_y:.1f}) with radius {sample_radius:.1f}")
        
        valid_samples = 0
        for i, (x, y) in enumerate(sample_points):
            depth = self.get_depth_at_pixel(x, y)
            if depth is not None and depth > 0:
                depth_values.append(depth)
                valid_samples += 1
                rospy.logdebug(f"Sample {i}: ({x:.1f},{y:.1f}) = {depth:.1f} cm")
        
        if not depth_values:
            rospy.logwarn("No valid depth values found in sampling area")
            
            # Fallback: try a larger area sampling
            rospy.loginfo("Attempting fallback sampling with larger area...")
            fallback_radius = sample_radius * 2
            fallback_points = []
            
            # Create a grid of points around the center
            for dx in range(-int(fallback_radius), int(fallback_radius), 5):
                for dy in range(-int(fallback_radius), int(fallback_radius), 5):
                    if dx*dx + dy*dy <= fallback_radius*fallback_radius:  # Within circle
                        fallback_points.append((center_x + dx, center_y + dy))
            
            for x, y in fallback_points:
                depth = self.get_depth_at_pixel(x, y)
                if depth is not None and depth > 0:
                    depth_values.append(depth)
                    if len(depth_values) >= 5:  # Stop after finding 5 valid points
                        break
        
        if not depth_values:
            rospy.logerr("No valid depth values found even after fallback sampling")
            return None
        
        # Use median for robustness
        median_depth = np.median(depth_values)
        mean_depth = np.mean(depth_values)
        std_depth = np.std(depth_values)
        
        rospy.loginfo(f"Depth statistics: {len(depth_values)} samples, median={median_depth:.1f}cm, mean={mean_depth:.1f}cm, std={std_depth:.1f}cm")
        
        return median_depth
  
    # Navigation helper methods
    def check_object_centered(self, center_x):
        frame_width = self.depth_frame.shape[1] if self.depth_frame is not None else 640
        camera_center_x = frame_width / 2
        offset = abs(center_x - camera_center_x)
        return offset < self.center_threshold

    def check_reachability(self, center_x, center_y, distance):
        try:
            robot_x, robot_y, robot_z = self.transform_to_robot_frame_depth(distance, center_x, center_y)
            theta2, theta3, theta4 = self.calculate_inverse_kinematics(robot_x, robot_z, 90)
            return theta2 is not None and theta3 is not None and theta4 is not None
        except Exception:
            return False       
        
    def boxes_overlap(self, a, b):
        print("Calculating overlap")
        return not (a["xmax"] < b["xmin"] or   # A is left of B
                    a["xmin"] > b["xmax"] or   # A is right of B
                    a["ymax"] < b["ymin"] or   # A is above B
                    a["ymin"] > b["ymax"])  

    def check_region_clear(self, center_box):
        print("Checking center region for obstacles")
        
        detections = self.get_fresh_detection("all", "") 
        if detections is None:
            rospy.loginfo("No detections — center is empty")
            return True

        for obj in detections['objects']:
            obj_box = {
                'xmin': obj.xmin,
                'ymin': obj.ymin,
                'xmax': obj.xmax,
                'ymax': obj.ymax
            }

            print("Object box: ", obj_box)
            print("Center box: ", center_box)

            if self.boxes_overlap(center_box, obj_box):
                rospy.loginfo("Center region is blocked by an object.")
                return False

        rospy.loginfo("Center region is empty — safe to place")
        return True
        
    # Movement methods
    def move_to_center_object(self, center_x):
        frame_width = self.depth_frame.shape[1] if self.depth_frame is not None else 640
        camera_center_x = frame_width / 2
        
        twist = Twist()
        error = center_x - camera_center_x
        rotation_speed = min(abs(error) / 200.0, 1.0) * self.rotation_step
        
        if error > 0:
            twist.angular.z = -rotation_speed
        else:
            twist.angular.z = rotation_speed
        
        self.pub_base.publish(twist)
        rospy.sleep(self.step_duration)
        self.stop_robot()
        rospy.sleep(0.02)
    
    def move_away_from_objects_to_center(self, class_name, level):
        """
        Move robot to avoid objects in the center area for safe placement.
        Re-detects object every few attempts to ensure accuracy.
        Returns True if safe center area is found, False otherwise.
        """
        frame_width = self.depth_frame.shape[1] if self.depth_frame is not None else 640
        frame_height = self.depth_frame.shape[0] if self.depth_frame is not None else 480
        camera_center_x = frame_width / 2
        camera_center_y = frame_height / 2
        center_region_width = 80  # pixels
        center_region_height = 80  # pixels
        forward = 0
        # Adjust vertical position based on shelf level
        if level == "level 2":
            y_offset = 75  # Shift box downward
            x_offset = 25  # Make box smaller
        else:
            y_offset = 0
            x_offset = 0
        
        center_box = {
            "xmin": int(camera_center_x - center_region_width/3 + x_offset),
            "xmax": int(camera_center_x + center_region_width/3 - x_offset),
            "ymin": int(camera_center_y - center_region_height/3 + y_offset),
            "ymax": int(camera_center_y + center_region_height/3 + y_offset)
        }
        print("center box: ", center_box)
        center_x = (center_box["xmin"] + center_box["xmax"]) / 2
        center_y = (center_box["ymin"] + center_box["ymax"]) / 2
        box_width = center_box["xmax"] - center_box["xmin"]
        box_height = center_box["ymax"] - center_box["ymin"]

        max_attempts = 10
        redetection_interval = 3
        attempt = 0
        direction = None

        detection = self.get_fresh_detection("place", class_name)
        det_center_x = (detection['xmin'] + detection['xmax']) / 2
        det_center_y = (detection['ymin'] + detection['ymax']) / 2
        det_box_width = abs(detection['xmax'] - detection['xmin'])
        det_box_height = abs(detection['ymax'] - detection['ymin'])
        object_distance = self.get_robust_depth(det_center_x, det_center_y, det_box_width, det_box_height)

        while attempt < max_attempts:
            attempt += 1
            if attempt == 1 or attempt % redetection_interval == 0:
                rospy.loginfo(f"Re-detecting object ({class_name}) during avoidance...")
                detection = self.get_fresh_detection("place", class_name)

            if detection is None:
                rospy.loginfo("No objects detected — center area is clear")
                return True

            # Compute object's center
            det_center_x = (detection['xmin'] + detection['xmax']) / 2
            det_center_y = (detection['ymin'] + detection['ymax']) / 2
            det_box_width = abs(detection['xmax'] - detection['xmin'])
            det_box_height = abs(detection['ymax'] - detection['ymin'])

            distance_from_center = abs(det_center_x - camera_center_x)
            

            # Too close — move away
            rospy.loginfo(f"Object too close to center at x={det_center_x:.1f}, moving away...")

            twist = Twist()
            error = det_center_x - camera_center_x  # positive = object on right

            rotation_speed = min(abs(error) / 150.0, 1.5) * self.rotation_step + 0.5
            if direction == "left":
                twist.angular.z = rotation_speed
                rospy.loginfo(f"{direction} -Rotating left")
            elif direction == "right":
                twist.angular.z = -rotation_speed
                rospy.loginfo(f"{direction} -Rotating right")
            elif error > 0:
                twist.angular.z = rotation_speed
                direction = "left"
                rospy.loginfo(f"{direction} - Rotating left to avoid object on right")
            else:
                twist.angular.z = -rotation_speed
                direction = "right"
                rospy.loginfo(f"{direction} -Rotating right to avoid object on left")

            self.pub_base.publish(twist)
            rospy.sleep(self.step_duration * 2)
            self.stop_robot()
            rospy.sleep(0.1)

            if distance_from_center > center_region_width / 2:
                rospy.loginfo(f"Object at x={det_center_x:.1f} is far enough from center — safe to place")
                if self.check_region_clear(center_box):
                    placement_distance = self.get_robust_depth(center_x, center_y, box_width, box_height) - 7
                    print(f"Object distance ({object_distance:.1f}cm), placement distance ({placement_distance:.1f}cm)")
                    if placement_distance < object_distance:
                        print("Object is in front of placement area, need to adjust")
                        if attempt > 3:
                            if direction == "left":
                                print("Changed direction to right")
                                direction = "right"
                            else:                    
                                print("Changed direction to left")        
                                direction = "left"
                            attempt = -10
                    else:
                        print ("Object is behind placement area, safe to place")
                        if placement_distance > 55:
                            forward = placement_distance - object_distance
                        else:
                            forward = 2
                        return True, forward
                else:
                    rospy.loginfo("Center region still blocked")
                    if attempt == max_attempts-1:
                        if direction == "left":
                            print("Changed direction to right")
                            direction = "right"
                        else:                    
                            print("Changed direction to left")        
                            direction = "left"
                        attempt = -10
                    continue

        rospy.logwarn("Could not find safe center area after maximum attempts")
        return False, forward

    def stop_robot(self):
        twist = Twist()
        self.pub_base.publish(twist)

    # Coordinate transformation
    def transform_to_robot_frame_depth(self, distance_cm, center_x, center_y):
        distance = distance_cm
        horizontal_distance = math.cos(math.radians(self.camera_angle)) * distance
        vertical_offset = self.camera_height - math.sin(math.radians(self.camera_angle)) * distance
        
        robot_x = horizontal_distance - self.camera_offset
        robot_z = vertical_offset
        
        frame_width = self.depth_frame.shape[1] if self.depth_frame is not None else 640
        robot_y = (center_x - frame_width / 2) * 0.01
        
        return robot_x, robot_y, robot_z

    # Inverse kinematics
    def calculate_inverse_kinematics(self, x, z, alpha_deg):
        print("x: ", x, "z: ", z)
        alpha = math.radians(alpha_deg)
        # End effector position
        m = z - self.L4 * math.cos(alpha)
        n = x - self.L4 * math.sin(alpha)

        # Reachability check
        if math.sqrt(m**2 + n**2) > (self.L2 + self.L3 + self.L4):
            return None, None, None

        # Elbow angle
        cos_theta3 = (m**2 + n**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        theta3 = math.acos(cos_theta3)
        # Shoulder angle
        theta12 = math.atan2(n, m)
        beta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = theta12 - beta
        # Wrist angle
        theta4 = alpha - (theta2 + theta3)

        # Convert degrees to radians
        theta2_rad = math.radians(theta2)
        theta3_rad = math.radians(theta3)
        theta4_rad = math.radians(theta4)

        # Joint limit check
        if not (self.JOINT_MIN <= theta2_rad <= self.JOINT_MAX and
                self.JOINT_MIN <= theta3_rad <= self.JOINT_MAX and
                self.JOINT_MIN <= theta4_rad <= self.JOINT_MAX):
            rospy.logwarn("IK solution violates joint limits")
            return None, None, None
        return math.degrees(theta2), math.degrees(theta3), math.degrees(theta4)

    # Arm control methods
    def move_to_ready_position(self):
        rospy.loginfo("Moving to ready position")
        self.pub_arm2.publish(Float64(-1.65806))
        self.pub_arm3.publish(Float64(2))
        rospy.sleep(1)
        self.pub_arm4.publish(Float64(1.09956))
        self.pub_gripper.publish(Float64(self.gripper_angle))
        rospy.sleep(5)
        self.pub_arm1.publish(Float64(0))

    def move_to_level2_place_position(self, forward):
        rospy.loginfo("Moving the arm to the place-down position.")
        self.pub_arm1.publish(Float64(0))
        self.pub_arm2.publish(Float64(math.radians(56)))
        self.pub_arm3.publish(Float64(math.radians(21)))
        self.pub_arm4.publish(Float64(math.radians(2)))

        rospy.sleep(5)
        self.move_forward(forward + 10)
        rospy.sleep(1)
        self.pub_gripper.publish(Float64(0.1))
        rospy.sleep(1)
        self.pub_gripper.publish(Float64(-0.2))
        rospy.sleep(1)
        self.pub_gripper.publish(Float64(-0.3))  # Open gripper
        rospy.sleep(1)

        self.move_forward(-forward-10)
        self.gripper_angle = 0.3
        # Return to the ready position after placing
        self.move_to_ready_position()
        return True

    def move_to_default_place_position(self, forward):
        rospy.loginfo("Moving the arm to default position.")
        self.move_forward(forward)
        self.pub_arm1.publish(Float64(0))
        self.pub_arm2.publish(Float64(math.radians(32)))
        self.pub_arm3.publish(Float64(math.radians(0.00)))
        self.pub_arm4.publish(Float64(math.radians(60)))

        rospy.sleep(3)
        self.pub_gripper.publish(Float64(0.1))
        rospy.sleep(1)
        self.pub_gripper.publish(Float64(-0.2))
        rospy.sleep(1)
        self.pub_gripper.publish(Float64(-0.3))  # Open gripper
        rospy.sleep(1)

        # Return to the ready position after placing
        self.move_to_ready_position()
        self.move_forward(-forward)
        return True
    
    def align_gripper_with_object(self, obj_center_x):
        frame_width = self.depth_frame.shape[1] if self.depth_frame is not None else 640
        CAMERA_CENTER_X = frame_width / 2
        MAX_ANGLE = 1.0
        LEFT_OFFSET = 0.15
        RIGHT_OFFSET = -0
        
        # Calculate the error: positive means object is to the right, negative means left
        error = CAMERA_CENTER_X - obj_center_x
        
        # Convert pixel error to angle: positive error (right) = positive angle (turn right)
        angle_to_object = (error / CAMERA_CENTER_X) * MAX_ANGLE
        print("Angle to object:", angle_to_object)
        
        # Apply offset compensation for gripper mechanics if needed
        if angle_to_object > 0.2:  # Object is to the left, gripper needs to turn left
            angle_to_object += LEFT_OFFSET + 0.1
        elif angle_to_object > 0.01:  # Object is to the left, gripper needs to turn left
            angle_to_object += LEFT_OFFSET
            print("Adjusting angle for left offset compensation:", angle_to_object)
        elif angle_to_object > -0.2:
            LEFT_OFFSET = 0.1
            angle_to_object += LEFT_OFFSET
        elif angle_to_object < -0.2:  # Object is to the right, gripper needs to turn right
            angle_to_object += RIGHT_OFFSET
            print("Adjusting angle for right offset compensation:", angle_to_object)

        angle_to_object += 0
        
        
        rospy.loginfo(f"Object at x={obj_center_x:.1f}, center={CAMERA_CENTER_X}, error={error:.1f}, angle={angle_to_object:.2f}")
        self.pub_arm1.publish(Float64(angle_to_object))
        rospy.sleep(0.5)

    def move_forward(self, distance_cm):
        distance_m = distance_cm / 100.0

        speed = 0.05  # m/s
        direction = 1.0 if distance_m >= 0 else -1.0

        twist = Twist()
        twist.linear.x = direction * speed

        duration = abs(distance_m) / speed
        rate = rospy.Rate(10)
        ticks = int(duration * 10)

        rospy.loginfo(f"Moving {'forward' if direction > 0 else 'backward'} {abs(distance_cm)} cm")

        for _ in range(ticks):
            self.pub_base.publish(twist)
            rate.sleep()

        # Stop robot
        twist.linear.x = 0.0
        self.pub_base.publish(twist)

    def pickup_object(self, x, z, alpha_deg, class_name):
        rospy.loginfo("Executing pickup sequence")
        theta2, theta3, theta4 = self.calculate_inverse_kinematics(x, z, alpha_deg)
        target_x, target_z = self.calculate_forward_kinematics(theta2, theta3, theta4)
        positional_error = self.calculate_position_error(target_x, target_z, x, z)
        
        if theta2 is None or theta3 is None or theta4 is None:
            rospy.logwarn("Failed to calculate joint angles for pickup")
            return False
        
        rospy.loginfo(f"Pickup angles - Theta2: {theta2:.2f}°, Theta3: {theta3:.2f}°, Theta4: {theta4:.2f}°")
        self.publish_viz(class_name, x, theta2, theta3, theta4, positional_error)
        
        # Open gripper
        self.pub_gripper.publish(Float64(-0.3))
        rospy.sleep(2)
        
        # Move to object
        self.pub_arm2.publish(Float64(math.radians(theta2)))
        self.pub_arm3.publish(Float64(math.radians(theta3)))
        self.pub_arm4.publish(Float64(math.radians(theta4)))
        rospy.sleep(5)
        
        # Close gripper
        self.pub_gripper.publish(Float64(self.gripper_angle))
        rospy.sleep(1)
        
        # Return to ready position
        self.move_to_ready_position()
        return True

    def calculate_forward_kinematics(self, theta2_deg, theta3_deg, theta4_deg):
        # Convert degrees to radians
        theta2 = math.radians(theta2_deg)
        theta3 = math.radians(theta3_deg)
        theta4 = math.radians(theta4_deg)
        # Compute cumulative angle
        alpha = theta2 + theta3 + theta4

        # Forward kinematics equations
        x = (
            self.L2 * math.sin(theta2) +
            self.L3 * math.sin(theta2 + theta3) +
            self.L4 * math.sin(alpha)
        )
        z = (
            self.L2 * math.cos(theta2) +
            self.L3 * math.cos(theta2 + theta3) +
            self.L4 * math.cos(alpha)
        )

        return x, z

    def calculate_position_error(self, target_x, target_z, actual_x, actual_z): 
        error = math.sqrt((actual_x - target_x)**2 +
                  (actual_z - target_z)**2)

        print(f"Position Error: {error:.3f} m")
        return error

if __name__ == '__main__':
    try:
        service = ArmManipulation()
        rospy.loginfo("Arm Manipulation Service started successfully")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm Manipulation Service terminated")