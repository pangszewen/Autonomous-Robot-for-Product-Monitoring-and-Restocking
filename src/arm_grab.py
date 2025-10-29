#!/usr/bin/env python

import rospy
import math
import numpy as np
import threading
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Import your custom service message types
from FYP_Pang.srv import ArmHeadGripper, ArmHeadGripperResponse
from FYP_Pang.srv import StartDetection, StartDetectionRequest


class ArmManipulationService:
    def __init__(self):
        rospy.init_node("head_arm_hand", anonymous=True)
        
        # Camera and sensor setup
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

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

        # Current sensor data
        self.frame = None
        self.depth_frame = None
        self.lidar_data = None
        self.image_pause = False
        self.lock = threading.Lock()
        self.latest_detection_result = None  # Shared result container
        
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

        # Detection service client
        self.detection_client = None
        self.setup_detection_client()
        
        # def execute_place_sequence(self, xmin, xmax, ymin, ymax, class_name=None):
        # """Execute placement sequence at detected location with object-based distance"""
        # rospy.loginfo("Starting place sequence at detected location...")
        
        # center_x = (xmin + xmax) / 2
        # center_y = (ymin + ymax) / 2
        # box_width = abs(xmax - xmin)
        # box_height = abs(ymax - ymin)
        
        # # SIMPLE OBJECT-BASED DISTANCE CONTROL
        # if class_name:
        #     # Heavy items = close (low shelf)
        #     if class_name in ['Bottle', 'Can', 'Milk']:
        #         distance = 30  # Close = low shelf
        #         shelf_name = "bottom shelf"
        #     # Light items = far (high shelf) - but check if reachable!
        #     elif class_name in ['Cup', 'Spoon', 'Fork', 'Cereal']:
        #         distance = 70  # Far = high shelf
        #         shelf_name = "top shelf"
        #     # Medium items = medium distance
        #     else:
        #         distance = 50  # Medium = mid shelf
        #         shelf_name = "middle shelf"
        # else:
        #     distance = 50  # Default distance for placement
        #     shelf_name = "shelf"
        
        # rospy.loginfo(f"Placing {class_name} at distance {distance}cm ({shelf_name})")
        
        # # Transform coordinates
        # robot_x, robot_y, robot_z = self.transform_to_robot_frame_depth(distance, center_x, center_y)
        
        # # Check if position is reachable before attempting
        # theta2, theta3, theta4 = self.calculate_inverse_kinematics(robot_x, robot_z, 90)
        
        # if theta2 is None or theta3 is None or theta4 is None:
        #     rospy.logwarn(f"Cannot reach {shelf_name} at distance {distance}cm - need help!")
        #     return False, f"I cannot reach the {shelf_name}. I need help placing the {class_name}."
        
        # # Execute placement
        # success = self.place_object(robot_x, robot_z)
        # message = f"I have placed the {class_name} on the {shelf_name}" if success else f"Failed to place {class_name}"
        
        # rospy.loginfo(f"Place sequence {'completed successfully' if success else 'failed'}")
        # return success, message

        # Service server
        self.service = rospy.Service('arm_manipulation', ArmHeadGripper, self.handle_arm_manipulation_new)
        
        rospy.loginfo("Arm Manipulation Service initialized and ready")
        rospy.loginfo("Empty location detection enabled for placement operations")

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

    def get_fresh_detection(self, mode, class_name):
        """
        Thread-safe call to detection service
        """
        if self.detection_client is None:
            rospy.logwarn("Detection service not available, attempting to reconnect...")
            self.setup_detection_client()
            if self.detection_client is None:
                return None

        try:
            request = StartDetectionRequest()
            request.mode = mode
            request.class_name = class_name
            
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
                    'message': response.message
                }
                rospy.loginfo(f"Fresh detection successful: {response.class_name} at ({response.xmin}, {response.ymin}, {response.xmax}, {response.ymax})")
                
                with self.lock:
                    self.latest_detection_result = detection_result

                return detection_result
            else:
                rospy.logwarn(f"Detection failed: {response.message}")
                return None
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Detection service call failed: {e}")
            return None

    def handle_arm_manipulation(self, req):
        """
        Service handler for arm manipulation requests
        Now uses fresh detection data instead of relying on passed coordinates
        """
        response = ArmHeadGripperResponse()
        message = None
        try:
            rospy.loginfo(f"Received arm manipulation request: mode={req.mode}, class={req.class_name}")
            
            if req.mode.lower() == "pick":
                self.approach_object(req.xmin, req.xmax, req.ymin, req.ymax, req.class_name)
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
                message = f"I have picked up the {detection['class_name']} from the table"
                
            elif req.mode.lower() == "place":
                self.move_forward(0)
                
                # Use the new safe placement method
                rospy.loginfo("Finding safe placement position...")
                safe_position_found = self.move_to_safe_placement_position(req.class_name)
                
                if safe_position_found:
                    rospy.loginfo("Safe position found, proceeding with placement")
                    # If no coordinates are given, use default place position
                    if req.xmin == 0 and req.xmax == 0 and req.ymin == 0 and req.ymax == 0:
                        rospy.loginfo("Using default place position")
                        success = self.move_to_place_position()
                    else:
                        rospy.loginfo(f"Placing at given coordinates")
                        success = self.execute_place_sequence(req.xmin, req.xmax, req.ymin, req.ymax)
                else:
                    rospy.logwarn("Could not find safe placement position, using default")
                    success = self.move_to_place_position()
                
                # FIX: Use req.class_name instead of undefined detection variable
                object_name = req.class_name if req.class_name else "object"
                message = f"I have placed down the {object_name} in the cabinet"
            
            else:
                rospy.logwarn(f"Unknown mode: {req.mode}")
                success = False
            
            response.success = success
            response.message = message if success else "Operation failed"
            
        except Exception as e:
            rospy.logerr(f"Error in arm manipulation service: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def handle_arm_manipulation_new(self, req):
        """
        Service handler for arm manipulation requests
        Now uses fresh detection data instead of relying on passed coordinates
        """
        response = ArmHeadGripperResponse()
        message = None
        try:
            rospy.loginfo(f"Received arm manipulation request: mode={req.mode}, class={req.class_name}")
            
            if req.mode.lower() == "pick":
                self.approach_object(req.xmin, req.xmax, req.ymin, req.ymax, req.class_name, 58)
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
                message = f"I have picked up the {detection['class_name']} from the table"
                
            elif req.mode.lower() == "place":
                self.move_forward(0)
                detection = self.get_fresh_detection("place", req.class_name)
                self.approach_object(
                    detection['xmin'], detection['xmax'], 
                    detection['ymin'], detection['ymax'], 
                    detection['class_name'], 60)
                success = True
                message = "Success"

            response.success = success
            response.message = message if success else "Operation failed"
            
        except Exception as e:
            rospy.logerr(f"Error in arm manipulation service: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def approach_object(self, xmin, xmax, ymin, ymax, class_name, stop_distance):
        """
        Move the robot in front of the detected object based on its coordinates.
        stop_distance: distance in meters to stop before object
        """
        print("Approaching object")
        # 1️⃣ Get object's bounding box center
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        box_width = abs(xmax - xmin)
        box_height = abs(ymax - ymin)

        # 2️⃣ Get distance from depth camera
        distance = self.get_robust_depth(center_x, center_y, box_width, box_height)
        if distance is None:
            rospy.logwarn("No depth data available for approach.")
            return False

        rospy.loginfo(f"Object {class_name} at distance: {distance:.2f} m")

        # 3️⃣ Rotate to center object in view
        image_center_x = self.frame.shape[1] / 2
        error_x = center_x - image_center_x

        angular_speed = -0.002 * error_x  # tune this value
        forward_speed = 0.15  # m/s

        twist = Twist()

        # Keep moving until close enough
        while distance > stop_distance:
            # Rotate to align
            twist.angular.z = angular_speed
            twist.linear.x = forward_speed if abs(error_x) < 20 else 0  # move only if roughly centered
            self.pub_base.publish(twist)

            rospy.sleep(0.1)

            # Update detection & depth
            obj = self.get_fresh_detection("pickup", class_name)
            if not obj:
                rospy.logwarn("Lost sight of object during approach.")
                break

            center_x = (obj['xmin'] + obj['xmax']) / 2
            center_y = (obj['ymin'] + obj['ymax']) / 2
            box_width = abs(obj['xmax'] - obj['xmin'])
            box_height = abs(obj['ymax'] - obj['ymin'])
            print("New center: ", center_x, center_y)
            distance = self.get_robust_depth(center_x, center_y, box_width, box_height)
            error_x = center_x - image_center_x
            angular_speed = -0.002 * error_x

        # Stop movement
        self.pub_base.publish(Twist())
        rospy.loginfo(f"Reached approach distance {stop_distance:.2f} m from {class_name}")
        return True


    def find_and_rotate_to_empty_location(self, class_name):
        """
        Rotate the robot to find an empty location suitable for placing objects
        Returns True if empty location found, False otherwise
        """
        rospy.loginfo("Starting search for empty placement location...")
        
        original_rotation = 0  # Keep track of total rotation
        search_step = self.rotation_step_for_search
        max_rotation = self.max_rotation_for_empty_search
        
        # Try current position first
        if self.is_current_view_suitable_for_placement(class_name):
            rospy.loginfo("Current location is suitable for placement")
            return True
        
        # Search by rotating
        for total_rotation in range(0, max_rotation, search_step):
            rospy.loginfo(f"Searching... rotated {total_rotation}° so far")
            
            # Rotate by search step
            self.rotate_robot(search_step)
            rospy.sleep(0.5)  # Wait for stabilization
            
            # Check if current view is suitable
            if self.is_current_view_suitable_for_placement(class_name):
                rospy.loginfo(f"Found suitable empty location after rotating {total_rotation + search_step}°")
                return True
        
        rospy.logwarn("Could not find suitable empty location after full rotation")
        return False

    def is_current_view_suitable_for_placement(self, class_name):
        """
        Check if the current camera view has a suitable empty area for placement
        Returns True if suitable, False otherwise
        """
        if self.depth_frame is None or self.frame is None:
            rospy.logwarn("No camera data available for empty location detection")
            return False
        
        # Get frame center region for analysis
        frame_height, frame_width = self.frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2
        
        # Define search area around center
        search_radius = min(self.empty_location_search_radius, min(frame_width, frame_height) // 3)
        
        # Sample depth values in the center region
        empty_areas = self.find_empty_areas_in_region(center_x, center_y, search_radius)
        
        if not empty_areas:
            rospy.logdebug("No empty areas found in current view")
            return False
        
        # Check if any empty area is suitable for placement
        for area in empty_areas:
            if self.is_area_suitable_for_placement(area, class_name):
                rospy.loginfo(f"Found suitable empty area: center=({area['center_x']:.1f}, {area['center_y']:.1f}), size={area['size']}")
                return True
        
        return False

    def find_empty_areas_in_region(self, center_x, center_y, radius):
        """
        Find empty areas in the specified region using depth analysis
        Returns list of empty area dictionaries
        """
        if self.depth_frame is None:
            return []
        
        empty_areas = []
        frame_height, frame_width = self.depth_frame.shape
        
        # Create a grid of sample points
        sample_points = []
        grid_size = 20  # pixels between sample points
        
        for dx in range(-radius, radius + 1, grid_size):
            for dy in range(-radius, radius + 1, grid_size):
                if dx*dx + dy*dy <= radius*radius:  # Within circle
                    sample_x = max(0, min(center_x + dx, frame_width - 1))
                    sample_y = max(0, min(center_y + dy, frame_height - 1))
                    sample_points.append((sample_x, sample_y))
        
        # Group nearby empty points into areas
        empty_points = []
        for x, y in sample_points:
            if self.is_point_empty(x, y):
                empty_points.append((x, y))
        
        if not empty_points:
            return []
        
        # Cluster empty points into areas
        areas = self.cluster_empty_points(empty_points)
        
        return areas

    def is_point_empty(self, x, y):
        """
        Check if a specific point represents empty space suitable for placement
        """
        depth = self.get_depth_at_pixel(x, y)
        if depth is None:
            return False
        
        # Consider it empty if it's at a reasonable distance for placement
        # and not too close (which might indicate an obstacle)
        min_distance = self.preferred_placement_distance - 20
        max_distance = self.preferred_placement_distance + 30
        
        return min_distance <= depth <= max_distance

    def cluster_empty_points(self, empty_points):
        """
        Cluster nearby empty points into areas
        """
        if not empty_points:
            return []
        
        areas = []
        used_points = set()
        cluster_distance = 30  # pixels
        
        for point in empty_points:
            if point in used_points:
                continue
            
            # Start new cluster
            cluster = [point]
            used_points.add(point)
            
            # Find nearby points
            for other_point in empty_points:
                if other_point in used_points:
                    continue
                
                # Check if within cluster distance of any point in current cluster
                for cluster_point in cluster:
                    dist = math.sqrt((other_point[0] - cluster_point[0])**2 + 
                                   (other_point[1] - cluster_point[1])**2)
                    if dist <= cluster_distance:
                        cluster.append(other_point)
                        used_points.add(other_point)
                        break
            
            # Create area from cluster
            if len(cluster) >= 3:  # Minimum points for a valid area
                center_x = sum(p[0] for p in cluster) / len(cluster)
                center_y = sum(p[1] for p in cluster) / len(cluster)
                
                area = {
                    'center_x': center_x,
                    'center_y': center_y,
                    'size': len(cluster),
                    'points': cluster
                }
                areas.append(area)
        
        return areas

    def is_area_suitable_for_placement(self, area, class_name):
        """
        Check if an empty area is suitable for object placement
        """
        # Check minimum size
        if area['size'] < self.min_empty_area_size / 10:  # Adjust for grid sampling
            return False
        
        # Check if area center has good depth reading
        center_depth = self.get_depth_at_pixel(area['center_x'], area['center_y'])
        if center_depth is None:
            return False
        
        # Check if depth is in preferred range
        min_distance = self.preferred_placement_distance - 25
        max_distance = self.preferred_placement_distance + 35
        
        if not (min_distance <= center_depth <= max_distance):
            return False
        
        # Check for object detection conflicts (optional)
        if self.has_objects_in_area(area, class_name):
            return False
        
        return True

    def has_objects_in_area(self, area, class_name):
        """
        Check if the area contains detected objects that would interfere with placement
        This is an optional additional check
        """
        # Try to get detection in the area
        try:
            detection = self.get_fresh_detection("place", class_name)
            if detection is not None:
                # Check if detection overlaps with our area
                det_center_x = (detection['xmin'] + detection['xmax']) / 2
                det_center_y = (detection['ymin'] + detection['ymax']) / 2
                
                area_radius = 50  # approximate radius around area center
                distance = math.sqrt((det_center_x - area['center_x'])**2 + 
                                   (det_center_y - area['center_y'])**2)
                
                if distance < area_radius:
                    rospy.loginfo("Area contains detected objects, skipping")
                    return True
        except Exception as e:
            rospy.logdebug(f"Object detection check failed: {e}")
        
        return False

    def rotate_robot(self, angle_degrees):
        """
        Rotate the robot by the specified angle (in degrees)
        Positive angle = counterclockwise, negative = clockwise
        """
        twist = Twist()
        angular_speed = 0.2  # rad/s - adjust for your robot
        angle_radians = math.radians(abs(angle_degrees))
        
        # Set rotation direction
        if angle_degrees > 0:
            twist.angular.z = angular_speed
        else:
            twist.angular.z = -angular_speed
        
        # Calculate duration
        duration = angle_radians / angular_speed
        
        rospy.logdebug(f"Rotating {angle_degrees}° (duration: {duration:.2f}s)")
        
        # Execute rotation
        rate = rospy.Rate(10)
        ticks = int(duration * 10)
        
        for _ in range(ticks):
            self.pub_base.publish(twist)
            rate.sleep()
        
        # Stop rotation
        self.stop_robot()
        rospy.sleep(0.1)  # Brief pause for stabilization

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
        success = self.pickup_object(robot_x, robot_z, 90)
        
        rospy.loginfo(f"Pick sequence {'completed successfully' if success else 'failed'}")
        return success

    def execute_place_sequence(self, xmin, xmax, ymin, ymax):
        """Execute placement sequence at detected location"""
        rospy.loginfo("Starting place sequence at detected location...")
        
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        box_width = abs(xmax - xmin)
        box_height = abs(ymax - ymin)
        
        # Get depth for placement location
        distance = 40  # Default distance for placement

        # Transform coordinates
        robot_x, robot_y, robot_z = self.transform_to_robot_frame_depth(distance, center_x, center_y)
        
        # Execute placement
        success = self.place_object(robot_x, robot_z)
        rospy.loginfo(f"Place sequence {'completed successfully' if success else 'failed'}")
        return success

    def navigate_to_object_with_redetection(self, mode, class_name, center_x, center_y, box_width, box_height):
        """Navigate robot to optimal picking position with periodic re-detection"""
        max_attempts = 50  # Reduced attempts since we're re-detecting
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
                rospy.logwarn("Object centered but not reachable - manual adjustment needed")
                return False
        
        rospy.logwarn("Navigation failed - maximum attempts reached")
        return False

    # Image and sensor callbacks
    def image_callback(self, msg):
        if self.image_pause:
            return
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"Image conversion failed: {e}")

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def depth_callback(self, msg):
        """Improved depth callback with better error handling and debugging"""
        try:            
            if msg.encoding == "16UC1":
                self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            elif msg.encoding == "32FC1":
                self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            else:
                rospy.logwarn(f"Unknown depth encoding: {msg.encoding}, trying passthrough")
                self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Debug: Print some statistics about the depth frame
            if self.depth_frame is not None:
                non_zero_count = np.count_nonzero(self.depth_frame)
                total_pixels = self.depth_frame.shape[0] * self.depth_frame.shape[1]
                
                # Print min/max values for debugging
                if non_zero_count > 0:
                    non_zero_values = self.depth_frame[self.depth_frame > 0]
            
        except CvBridgeError as e:
            rospy.logerr(f"Depth image conversion failed: {e}")
            self.depth_frame = None

    def get_depth_at_pixel(self, x, y):
        """Improved depth extraction with better debugging"""
        if self.depth_frame is None:
            rospy.logwarn("No depth frame available")
            return None
        
        height, width = self.depth_frame.shape
        
        # Ensure coordinates are within bounds
        x = int(max(0, min(x, width - 1)))
        y = int(max(0, min(y, height - 1)))
        
        try:
            depth_value = self.depth_frame[y, x]
            
            if self.depth_frame.dtype == np.uint16:
                if depth_value == 0:
                    rospy.logdebug(f"Zero depth at pixel ({x},{y})")
                    return None
                # Convert from millimeters to centimeters
                converted_depth = depth_value / 10.0
                rospy.logdebug(f"Converted depth (uint16): {converted_depth} cm")
                return converted_depth
                
            elif self.depth_frame.dtype == np.float32:
                if np.isnan(depth_value) or depth_value == 0:
                    rospy.logdebug(f"Invalid float32 depth at pixel ({x},{y}): {depth_value}")
                    return None
                # Convert from meters to centimeters
                converted_depth = depth_value * 100.0
                rospy.logdebug(f"Converted depth (float32): {converted_depth} cm")
                return converted_depth
                
            else:
                rospy.logwarn(f"Unsupported depth frame dtype: {self.depth_frame.dtype}")
                return None
                
        except Exception as e:
            rospy.logerr(f"Error getting depth at pixel ({x},{y}): {e}")
            return None

    def get_robust_depth(self, center_x, center_y, box_width, box_height):
        """Enhanced robust depth calculation with more sampling points and better debugging"""
        if self.depth_frame is None:
            rospy.logwarn("No depth frame available for robust depth calculation")
            return None
        
        # Increase sample radius and add more sample points
        sample_radius = max(min(box_width, box_height) * 0.3, 10)  # Minimum 10 pixel radius
        depth_values = []
        
        # More comprehensive sampling pattern
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
        
        # Use median for robustness, but also log statistics
        median_depth = np.median(depth_values)
        mean_depth = np.mean(depth_values)
        std_depth = np.std(depth_values)
        
        rospy.loginfo(f"Depth statistics: {len(depth_values)} samples, median={median_depth:.1f}cm, mean={mean_depth:.1f}cm, std={std_depth:.1f}cm")
        
        return median_depth
  
    # Navigation helper methods
    def check_object_centered(self, center_x):
        frame_width = self.frame.shape[1] if self.frame is not None else 640
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
    
    # Movement methods
    def move_to_center_object(self, center_x):
        frame_width = self.frame.shape[1] if self.frame is not None else 640
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
    
    def move_away_from_objects_to_center(self, class_name):
        """
        Move robot to avoid objects in the center area for safe placement.
        Re-detects object every few attempts to ensure accuracy.
        Returns True if safe center area is found, False otherwise.
        """
        frame_width = self.frame.shape[1] if self.frame is not None else 640
        frame_height = self.frame.shape[0] if self.frame is not None else 480
        camera_center_x = frame_width / 2
        camera_center_y = frame_height / 2

        center_region_width = 80  # pixels
        max_attempts = 20
        redetection_interval = 3
        attempt = 0

        detection = None  # start with no detection

        while attempt < max_attempts:
            attempt += 1

            # Re-detect every few steps
            if attempt == 1 or attempt % redetection_interval == 0:
                rospy.loginfo("Re-detecting object during avoidance...")
                detection = self.get_fresh_detection("place", class_name)

            if detection is None:
                rospy.loginfo("No objects detected — center area is clear")
                return True

            # Compute object's center
            det_center_x = (detection['xmin'] + detection['xmax']) / 2
            det_center_y = (detection['ymin'] + detection['ymax']) / 2

            distance_from_center = abs(det_center_x - camera_center_x)

            if distance_from_center > center_region_width / 2:
                rospy.loginfo(f"Object at x={det_center_x:.1f} is far enough from center — safe to place")
                return True

            # Too close — move away
            rospy.loginfo(f"Object too close to center at x={det_center_x:.1f}, moving away...")

            twist = Twist()
            error = det_center_x - camera_center_x  # positive = object on right

            rotation_speed = min(abs(error) / 150.0, 1.5) * self.rotation_step + 0.5

            if error > 0:
                twist.angular.z = rotation_speed
                rospy.loginfo("Rotating left to avoid object on right")
            else:
                twist.angular.z = -rotation_speed
                rospy.loginfo("Rotating right to avoid object on left")

            self.pub_base.publish(twist)
            rospy.sleep(self.step_duration * 2)
            self.stop_robot()
            rospy.sleep(0.1)

        rospy.logwarn("Could not find safe center area after maximum attempts")
        return False

    def move_to_safe_placement_position(self, class_name):
        """
        Combined method to find a safe placement position by avoiding center objects
        and optionally using the empty location search
        """
        rospy.loginfo("Finding safe placement position...")
        detection = self.get_fresh_detection("place", class_name)
        xmin = detection['xmin']
        xmax = detection['xmax']
        ymin = detection['ymin']
        ymax = detection['ymax']
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        box_width = abs(xmax - xmin)
        box_height = abs(ymax - ymin)
        self.navigate_to_object_with_redetection("place", class_name, center_x, center_y, box_width, box_height)

        # First try to clear the center area
        center_clear = self.move_away_from_objects_to_center(class_name)
        
        if center_clear:
            rospy.loginfo("Center area is clear for placement")
            return True
        else:
            # Fallback to rotation-based empty location search
            rospy.loginfo("Center clearing failed, trying rotation-based search...")
            return self.find_and_rotate_to_empty_location(class_name)

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
        
        frame_width = self.frame.shape[1] if self.frame is not None else 640
        robot_y = (center_x - frame_width / 2) * 0.01
        
        return robot_x, robot_y, robot_z

    # Inverse kinematics
    def calculate_inverse_kinematics(self, x, z, alpha_deg):
        print("x: ", x, "z: ", z)
        alpha = math.radians(alpha_deg)
        m = z - self.L4 * math.cos(alpha)
        n = x - self.L4 * math.sin(alpha)

        if math.sqrt(m**2 + n**2) > (self.L2 + self.L3 + self.L4 + 200):
            return None, None, None

        cos_theta3 = (m**2 + n**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        theta3 = math.acos(cos_theta3)
        theta12 = math.atan2(n, m)
        beta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
        theta2 = theta12 - beta
        theta4 = alpha - (theta2 + theta3)
        return math.degrees(theta2), math.degrees(theta3), math.degrees(theta4)

    # Arm control methods
    def move_to_ready_position(self):
        rospy.loginfo("Moving to ready position")
        self.pub_arm2.publish(Float64(-1.65806))
        self.pub_arm3.publish(Float64(2))
        rospy.sleep(1)
        self.pub_arm4.publish(Float64(1.09956))
        self.pub_gripper.publish(Float64(0.5))
        rospy.sleep(5)
        self.pub_arm1.publish(Float64(0))

    def move_to_place_position(self):
        """Moves the arm to the place-down position."""
        rospy.loginfo("Moving the arm to the place-down position.")
        self.pub_arm1.publish(Float64(0))
        self.pub_arm2.publish(Float64(math.radians(26.77)))
        self.pub_arm3.publish(Float64(math.radians(0.00)))
        self.pub_arm4.publish(Float64(math.radians(63.23)))

        rospy.sleep(5)

        # Open the gripper to place down the object
        rospy.loginfo("Placing object down by opening the gripper.")
        self.pub_gripper.publish(Float64(-0.5))  # Open gripper
        rospy.sleep(1)

        # Return to the ready position after placing
        self.move_to_ready_position()
        return True

    def align_gripper_with_object(self, obj_center_x):
        CAMERA_CENTER_X = 320
        MAX_ANGLE = 1.0
        LEFT_OFFSET = 0.25
        
        # Calculate the error: positive means object is to the right, negative means left
        error = CAMERA_CENTER_X - obj_center_x
        
        # Convert pixel error to angle: positive error (right) = positive angle (turn right)
        angle_to_object = (error / CAMERA_CENTER_X) * MAX_ANGLE
        
        # Apply offset compensation for gripper mechanics if needed
        if angle_to_object > 0.1:  # Object is to the left, gripper needs to turn left
            angle_to_object += LEFT_OFFSET
            print("Adjusting angle for left offset compensation:", angle_to_object)
        
        print("Angle to object:", angle_to_object)
        
        rospy.loginfo(f"Object at x={obj_center_x:.1f}, center={CAMERA_CENTER_X}, error={error:.1f}, angle={angle_to_object:.2f}")
        self.pub_arm1.publish(Float64(angle_to_object))
        rospy.sleep(0.5)

    def move_forward(self, distance_cm):
        # Convert cm to meters
        distance_m = distance_cm / 100.0
        speed = 0.05  # m/s safe speed
        twist = Twist()
        twist.linear.x = speed

        duration = distance_m / speed
        rate = rospy.Rate(10)
        ticks = int(duration * 10)

        rospy.loginfo(f"Moving forward {distance_cm} cm before placing...")
        for _ in range(ticks):
            self.pub_base.publish(twist)
            rate.sleep()

        # Stop
        twist.linear.x = 0.0
        self.pub_base.publish(twist)

    def pickup_object(self, x, z, alpha_deg):
        rospy.loginfo("Executing pickup sequence")
        theta2, theta3, theta4 = self.calculate_inverse_kinematics(x, z, alpha_deg)
        
        if theta2 is None or theta3 is None or theta4 is None:
            rospy.logwarn("Failed to calculate joint angles for pickup")
            return False
        
        rospy.loginfo(f"Pickup angles - Theta2: {theta2:.2f}°, Theta3: {theta3:.2f}°, Theta4: {theta4:.2f}°")
        
        # Open gripper
        self.pub_gripper.publish(Float64(-0.3))
        rospy.sleep(2)
        
        # Move to object
        self.pub_arm2.publish(Float64(math.radians(theta2)))
        self.pub_arm3.publish(Float64(math.radians(theta3)))
        self.pub_arm4.publish(Float64(math.radians(theta4)))
        rospy.sleep(5)
        
        # Close gripper
        self.pub_gripper.publish(Float64(1.0))
        rospy.sleep(1)
        
        # Return to ready position
        self.move_to_ready_position()
        return True

    def place_object(self, robot_x, robot_z):
        rospy.loginfo("Executing placement sequence")
        theta2, theta3, theta4 = self.calculate_inverse_kinematics(robot_x, robot_z, 90)
        
        if theta2 is None or theta3 is None or theta4 is None:
            rospy.logwarn("Cannot reach placement position")
            return False
        
        rospy.loginfo(f"Placement angles - Theta2: {theta2:.2f}°, Theta3: {theta3:.2f}°, Theta4: {theta4:.2f}°")
        
        # Move to placement position
        self.pub_arm1.publish(Float64(0))
        self.pub_arm2.publish(Float64(math.radians(theta2)))
        self.pub_arm3.publish(Float64(math.radians(theta3)))
        self.pub_arm4.publish(Float64(math.radians(theta4)))
        rospy.sleep(3)
        
        # Open gripper to place object
        self.pub_gripper.publish(Float64(-0.5))
        rospy.sleep(1)
        
        # Return to ready position
        self.move_to_ready_position()
        return True


if __name__ == '__main__':
    try:
        service = ArmManipulationService()
        rospy.loginfo("Arm Manipulation Service started successfully")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm Manipulation Service terminated")