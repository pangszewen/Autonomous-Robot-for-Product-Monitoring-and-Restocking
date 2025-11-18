#!/usr/bin/env python3
# filepath: /path/to/breakfast/src/example_main.py
from gtts import gTTS
import rospy
from fyp_pang.srv import Navigate, StartDetection, StartMonitoring
from fyp_pang.srv import ArmHeadGripper
import os
import time
import speech_recognition as sr
import threading
import queue

class Run:
    def __init__(self):
        rospy.init_node('start')
        self.location_response_message = False
        self.detection_response_message = ""
        self.monitoring_response_message = []
        self.target_transformation_response_message = []

        self.low_stock_product = []
        self.low_stock_count = []
        self.out_of_stock = []
        
        self.OBJECT_CATEGORIES = {
            'drinks': ['water', 'coffee', 'juice', 'milk', 'soda'],
            'food': ['tuna', 'cup noodle', 'cereal', 'jam', 'yogurt'],
            'snacks': ['biscuits', 'chips', 'chocolate']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'level 1',                   # Middle shelf, left side
            'food': 'level 2',                   # Bottom shelf, middle (heavy items)
        }
        
        # Voice recognition threading setup
        self.voice_queue = queue.Queue()
        self.listening = False
        self.voice_thread = None

    def text2audio(self, text):
        tts = gTTS(text)
        tts.save("main_audio.mp3")
        os.system("mpg321 main_audio.mp3")
        os.remove("main_audio.mp3")
    
    def alert_notification(self, product):
        # add update firebase data here
        alert_message = f"Alert: The product {product} is out of stock. Please restock it as soon as possible."
        self.text2audio(alert_message)
    
    def get_object_location(self, class_name):
        """
        Get the location of the object based on its class name.
        Returns a dictionary with coordinates.
        """
        for category, items in self.OBJECT_CATEGORIES.items():
            if class_name in items:
                rospy.loginfo(f"Object {class_name} belongs to category {category}")
                return self.CATEGORY_LOCATIONS[category]
        
        return None

    """ Below are the service calls examples """

    def navigation(self, target_location):
        rospy.wait_for_service('navigate')
        try:
            print("in navigation service")
            get_location_service = rospy.ServiceProxy('navigate', Navigate)
            self.location_response = get_location_service(target_location)
            self.location_response_message = self.location_response.reach
            
            return self.location_response_message
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def detection(self, mode, class_name, location_response_message):
        rospy.wait_for_service('startDetect')
        try:
            get_detection_service = rospy.ServiceProxy('startDetect', StartDetection)
            print("in detect service")
            self.detection_response = get_detection_service(mode, class_name, location_response_message)
            print("Response from server:", self.detection_response)
            return self.detection_response
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def monitoring(self):
        rospy.wait_for_service('startMonitoring')
        try:
            get_monitoring_service = rospy.ServiceProxy('startMonitoring', StartMonitoring)
            print("in monitoring service")
            self.monitoring_response = get_monitoring_service()
            print("Response from server:", self.monitoring_response)
            return self.monitoring_response
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)
    
    def arm_manipulation(self, mode, xmin, ymin, xmax, ymax, class_name):
        """
        Call the new arm manipulation service
        
        Args:
            mode: "pick" or "place"
            xmin, ymin, xmax, ymax: bounding box coordinates
            class_name: object class name
        """
        rospy.wait_for_service('arm_manipulation')
        print("in arm service")
        try:
            get_arm_service = rospy.ServiceProxy('arm_manipulation', ArmHeadGripper)
            response = get_arm_service(mode, xmin, ymin, xmax, ymax, class_name)
            print(f"Arm manipulation response: {response}")
            return response
        except rospy.ServiceException as e:
            print("Arm manipulation service call failed:", e)
            return None
        
    def restocking(self, product, status):
        if status:
            print("Navigating to storage")
            location = self.navigation('storage')

        detection = self.detection('pickup', product, True)
        xmin = detection.xmin
        xmax = detection.xmax
        ymin = detection.ymin
        ymax = detection.ymax
        class_name = detection.class_name 
        success = detection.success
        print(xmin, xmax, ymin, ymax)

        if success == False:
            if product not in self.out_of_stock:
                self.out_of_stock.append(product)
            return False

        action2 = "Pick"
        self.text2audio(f"Picking up the {class_name}.")
        arm_target = self.arm_manipulation(action2, xmin, ymin, xmax, ymax, class_name)

        print("Navigating to shelf")
        location = self.navigation('shelf')

        action3 = "Place"
        arm_target = self.arm_manipulation(action3, 0, 0, 0, 0, class_name)

        low_stock = self.monitoring()
        self.low_stock_product = list(low_stock.low_stock_name)
        self.low_stock_count = list(low_stock.low_stock_count)
        return True

    
    def storing_groceries_new(self):
        #loaction1 = self.navigation('dining_table')

        loaction1 = None
        while not loaction1:
            loaction1 = self.detection('pickup', '', True)
            self.text2audio(loaction1.message)

            # Extract values from the response
            xmin = loaction1.xmin
            xmax = loaction1.xmax
            ymin = loaction1.ymin
            ymax = loaction1.ymax
            class_name1 = loaction1.class_name 
            success = loaction1.success
            print(xmin, xmax, ymin, ymax)
            if not loaction1.class_name:
                loaction1 = None
                continue
            action2 = "Pick"
            self.text2audio(f"Picking up the {class_name1}.")
            arm_target = self.arm_manipulation(action2, xmin, ymin, xmax, ymax, class_name1)
            self.assist_load_onto_robot_new(class_name1)

            

        shelf = self.get_object_location(class_name1)
        loaction2 = self.navigation(shelf)

        action3 = "Place"
        arm_target = self.arm_manipulation(action3, 0, 0, 0, 0, class_name1)
        self.assist_unload_to_table_and_confirm_new(class_name1)
        
    def monitoring_restocking(self):
        print("Navigating to shelf for monitoring")
        location = self.navigation('shelf')

        low_stock = self.monitoring()
        if low_stock:
            self.low_stock_product = list(low_stock.low_stock_name)
            self.low_stock_count = list(low_stock.low_stock_count)

            combined = list(zip(self.low_stock_product, self.low_stock_count))
            combined.sort(key=lambda x: x[1])  # x[1] = count

            self.low_stock_product, self.low_stock_count = map(list, zip(*combined))

            self.text2audio(f"The stock of {self.low_stock_product} is low.")

            status = True
            while self.low_stock_product:
                print(self.low_stock_product)
                product = self.low_stock_product.pop(0)
                count = self.low_stock_count.pop(0)
                if product in self.out_of_stock:
                    continue
                else:
                    while count < 3:
                        print(f"Enter restocking loop with product {product}")
                        status = self.restocking(product, status)
                        if status:
                            continue
                        else:
                            print(f"Out of stock list: {self.out_of_stock}")
                            self.alert_notification(product)
                            break
                    if status:
                        self.text2audio(f"The product {product} has been restocked.")
            
            return True
        else:
            # No low stock products
            return False
            
    """----------------------------------------------------------------------------------- """

if __name__=="__main__":
    """ Below are how to develop a task using the service calls """
    try:
        grocery = Run()
        rospy.sleep(3)
        print("say hello")
        grocery.text2audio('hello')
        status = grocery.monitoring_restocking()
        while status:
            status = grocery.monitoring_restocking()

        grocery.text2audio('There are no low stock products at the moment. I will return to home now.')
        grocery.navigation('home')
                
        rospy.spin()
    except rospy.ROSInterruptException:
        pass