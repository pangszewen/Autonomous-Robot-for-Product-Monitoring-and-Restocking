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
            'shelves': Pose(Point(4.63097078134008, 2.683006458335114, 0), Quaternion(0, 0, 0.6838905091922369, 0.7295846567991838)),
            'dishwasher_tab': Pose(Point(5.450863989861071, 2.764117693605161, 0), Quaternion(0, 0, 0.7318640813945486, 0.6814506338426236)),
            'dining_table': Pose(Point(6.546260768446007, 1.7398538474720548, 0), Quaternion(0, 0, -0.7085465297677525, 0.7056640951288903)),
            'bottle': Pose(Point(0.8889310929397624, 0.5201471781069732, 0), Quaternion(0, 0, -0.481049374822354, 0.876693503445202)),
            'mid_point': Pose(Point(0.09668431321600376, 1.3018808500846424, 0), Quaternion(0, 0, -0.9845392032826615, 0.17516437194687165)),
            'bottle_return': Pose(Point(-0.7397455653959026, 0.8305652373880503, 0), Quaternion(0, 0, -0.9458597445172536, 0.3245756363342074)),
            'bag_location': Pose(Point(1.3816444943182358, -0.234840282425134, 0), Quaternion(0, 0, 0.07404929272095194, 0.9972545824650427)),
            'maker_chair': Pose(Point(4.806, 0.4116, 0), Quaternion(0, 0, -0.1347, 0.9908)),
            # 'maker_table': Pose(Point(4.4608, -2.0033, 0), Quaternion(0, 0, -0.7158, 0.6982)),
            'maker_table': Pose(Point(1.561718588073762, -0.5880517290996721, 0), Quaternion(0, 0, -0.691833560277741, 0.7220570094344527)),
            'kitchen': Pose(Point(4.7412, -0.54295, 0), Quaternion(0, 0,-0.3371, 0.9414)),
            'Shelf1': Pose(Point(4.1618, -1.5304, 0), Quaternion(0, 0, 0.9976, 0.2170)),
            'Shelf2': Pose(Point(4.1781, -1.1627, 0), Quaternion(0, 0, -0.9992,  0.03939)),
            'Shelf3': Pose(Point(4.24252, -0.8534, 0), Quaternion(0, 0,  -0.9984, 0.05482)),
            'RedTable': Pose(Point(5.2189, -1.2945, 0), Quaternion(0, 0, -0.6971, 0.7169)),
            'table1': Pose(Point(5.0133, -0.4718, 0), Quaternion(0, 0, 0.6557, 0.7549)),
            'table2': Pose(Point(5.7666, -0.5163, 0), Quaternion(0, 0, 0.6296, 0.7768)),
            'halfway': Pose(Point(2.9546, 0.8705, 0), Quaternion(0, 0, -0.7496, 0.6618)),
            'entrance recpt': Pose(Point(0.5808, 1.1929, 0), Quaternion(0, 0, 0.99790, 0.06462)),
            'seat_find': Pose(Point(1.1175, -0.1197, 0), Quaternion(0, 0, -0.5155, 0.8568)),
            'direct_sofa': Pose(Point(2.2832, -0.1862, 0), Quaternion(0, 0, -0.6013, 0.7989)),
            'blue_seat': Pose(Point(0.8119, -1.650, 0), Quaternion(0, 0, -0.5293, 0.84837))
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
