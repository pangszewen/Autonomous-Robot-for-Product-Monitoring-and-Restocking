#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import actionlib
import os
from gtts import gTTS
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from fyp_pang.srv import Navigate, NavigateResponse

# Global variables for storing initial pose only once
original = 0
start = 0

class NavToPoint:
    def __init__(self):
        # Ensure cleanup function is called on shutdown
        rospy.on_shutdown(self.cleanup)

        # Create an action client to interact with the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server.")

        # Subscribe to RViz initial pose topic to set robot's starting location
        initial_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        
        # Wait until user sets the initial pose in RViz
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

        # Wait until the timestamp is set, indicating a valid pose
        while initial_pose.header.stamp == "":
           rospy.sleep(1)

        rospy.loginfo("Starting navigation node...")
        rospy.sleep(1)
        
        # Get initial location
        # quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        # self.origin = Pose(Point(0, 0, 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        # --------------------------------------------------------------------------
        
        # Predefined named locations with their corresponding coordinates and orientations
        '''
        How to get location?
        ## Run the navigation with your map first
        ## Control the robot to the locations that you wanna save
        ## Run `rostopic echo /amcl_pose` to get the coodinate x, y and orientation z, w
        ## Update the locations
        '''

        self.locations = {
            'home': Pose(Point(0.3830797187370426, 0.14357691122259328, 0), Quaternion(0, 0, -0.9108690217236833, 0.41269556002474755)),
            'shelf': Pose(Point(-1.1899734354625298, 0.120133026173707, 0), Quaternion(0, 0, -0.9119565070585661, 0.4102868864996045)),
            'storage': Pose(Point(-1.5360975623732296, 1.0040125505470325, 0), Quaternion(0, 0, 0.8973757880141544, 0.4412671470730349))
        }

        # --------------------------------------------------------------------------
        # Start a ROS service called 'navigate' to receive navigation requests
        self.service = rospy.Service('navigate', Navigate, self.nav_to_point)
        

    def nav_to_point(self, request):
        """
        Service callback to navigate the robot to the requested location.
        """
        self.goal = MoveBaseGoal()
        rospy.loginfo("Ready to go.")

        # Set goal frame and timestamp
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Get destination coordinates from the dictionary based on request
        coordinate = self.locations[request.target_location]
        self.goal.target_pose.pose = coordinate

        rospy.loginfo(f"Going to {request}")
        self.move_base.send_goal(self.goal)

        location = request.target_location
        location = location.replace("_", "")

        # Wait up to 300 seconds for the robot to reach the goal
        waiting = self.move_base.wait_for_result(rospy.Duration(300))
        if waiting:
            rospy.loginfo(f"Reached {request}")
            return NavigateResponse(reach=True, message=f"I have reached the {request.target_location}")
        else:
            return NavigateResponse(reach=False, message="Failed to reach point")
    

    def update_initial_pose(self, initial_pose):
        """
        Callback function to update the robot's initial pose once.
        """
        self.initial_pose = initial_pose
        global original
        if original == 0:
            # Store initial pose only once
            self.origin = self.initial_pose.pose.pose
            original = 1

    def cleanup(self):
        """
        Called on shutdown. Cancels current move_base goal.
        """
        rospy.loginfo("Shutting down navigation...")
        self.move_base.cancel_goal()
    
    def speak(self, text):
        tts = gTTS(text=text, lang='en')
        tts.save("/tmp/detection.mp3")
        os.system("mpg123 /tmp/detection.mp3")

if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass
