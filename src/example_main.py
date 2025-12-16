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
import firebase_admin
from firebase_admin import credentials, db

# Initialize Firebase
cred = credentials.Certificate("/home/mustar/catkin_ws/src/fyp_pang/src/serviceAccountKey.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': "https://product-monitoring-fe713-default-rtdb.asia-southeast1.firebasedatabase.app/"
})

class Run:
    def __init__(self):
        rospy.init_node('start')
        self.location_response_message = False
        self.detection_response_message = ""
        self.monitoring_response_message = []
        self.target_transformation_response_message = []
        self.stock_db_ref = db.reference('stock_counts')

        self.low_stock_product = []
        self.low_stock_count = []
        self.out_of_stock = []
        
        self.OBJECT_CATEGORIES = {
            'drinks': ['water', 'coffee', 'juice', 'milk', 'soda'],
            'food': ['tuna', 'cup noodle', 'cereal', 'jam', 'yogurt', 'biscuits', 'chips', 'chocolate']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'level 1',                   
            'food': 'level 2',                   
        }

        self.OBJECT_LOCATIONS = {
            'juice': 'left',
            'milk': 'right',
            'chips': 'left',
            'yogurt': 'right',
        }

    def text2audio(self, text):
        tts = gTTS(text)
        tts.save("main_audio.mp3")
        os.system("mpg321 main_audio.mp3")
        os.remove("main_audio.mp3")
    
    def alert_notification(self, product):
        alert_message = f"Alert: The product {product} is out of stock. Please restock it as soon as possible."
        self.text2audio(alert_message)
    
    def get_object_location(self, class_name):
        for category, items in self.OBJECT_CATEGORIES.items():
            if class_name in items:
                rospy.loginfo(f"Object {class_name} belongs to category {category}")
                return self.CATEGORY_LOCATIONS[category]
        
        return None
    
    def get_oos_list(self):
        existing_data = self.stock_db_ref.get()
        oos_list = []

        if existing_data:
            for class_name, info in existing_data.items():
                # Ensure entry has 'count'
                count = info.get('count', None)

                if count == 0:
                    oos_list.append(class_name)

        return oos_list

    """ Below are the service calls examples """

    def navigation(self, target_location):
        rospy.wait_for_service('navigate')
        try:
            get_location_service = rospy.ServiceProxy('navigate', Navigate)
            print("in navigation service")
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
        print("Navigating to shelf for monitoring")
        self.navigation('shelf')
        rospy.wait_for_service('startMonitoring')
        try:
            get_monitoring_service = rospy.ServiceProxy('startMonitoring', StartMonitoring)
            print("in monitoring service")
            self.monitoring_response = get_monitoring_service()
            print("Response from server:", self.monitoring_response)
            low_stock = self.monitoring_response
            if low_stock:
                self.low_stock_product = list(low_stock.low_stock_name)
                self.low_stock_count = list(low_stock.low_stock_count)
                return True
            else:
                return False
        except rospy.ServiceException as e:
            print("Service call failed:", e)
    
    def arm_manipulation(self, mode, xmin, ymin, xmax, ymax, class_name):
        rospy.wait_for_service('arm_manipulation')
        try:
            get_arm_service = rospy.ServiceProxy('arm_manipulation', ArmHeadGripper)
            print("in arm service")
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
        
        pick_status = False
        while not pick_status:
            detection = self.detection('pickup', product, True)
            xmin = detection.xmin
            xmax = detection.xmax
            ymin = detection.ymin
            ymax = detection.ymax
            class_name = detection.class_name 
            success = detection.success
            message = detection.message
            print(xmin, xmax, ymin, ymax)
            print(message)

            if not success:
                return False

            action2 = "Pick"
            self.text2audio(f"Picking up the {class_name}.")
            pick_status = self.arm_manipulation(action2, xmin, ymin, xmax, ymax, class_name)
            if not pick_status:
                location = self.navigation('storage')
        

        print("Navigating to shelf")
        location = self.navigation('shelf')

        action3 = "Place"
        place_status = False
        while not place_status:
            place_status = self.arm_manipulation(action3, 0, 0, 0, 0, class_name)
        
        return True
        
    def monitoring_restocking(self):
        low_stock = self.monitoring()
        if low_stock:
            combined = list(zip(self.low_stock_product, self.low_stock_count))
            combined.sort(key=lambda x: x[1])

            self.low_stock_product, self.low_stock_count = map(list, zip(*combined))

            if all(item in self.out_of_stock for item in self.low_stock_product):
                return False
            else:
                self.text2audio(f"The stock of {self.low_stock_product} is low.")

            stock_status = True
            while self.low_stock_product:
                print(self.low_stock_product)
                product = self.low_stock_product[0]
                count = self.low_stock_count[0]
                if product in self.out_of_stock:
                    index = self.low_stock_product.index(product)
                    self.low_stock_product.pop(index)
                    self.low_stock_count.pop(index)
                    continue
                else:
                    while count < 3:
                        print(f"Enter restocking loop with product {product}")
                        stock_status = self.restocking(product, stock_status)
                        if stock_status:
                            # Have stock
                            self.monitoring()
                            if product in self.low_stock_product:
                                index = self.low_stock_product.index(product)
                                count = self.low_stock_count[index]
                                continue
                            else:
                                break
                        else:
                            # Out of stock
                            print(f"Out of stock list: {self.out_of_stock}")
                            self.alert_notification(product)
                            break
                    if stock_status:
                        self.text2audio(f"The product {product} has been restocked.")
                # Refresh out of stock list after each product restocking attempt 
                self.out_of_stock = self.get_oos_list()
            return True
        else:
            # No low stock products
            return False
        
    def test(self):
        self.restocking('juice', True)
            
    """----------------------------------------------------------------------------------- """

if __name__=="__main__":
    """ Below are how to develop a task using the service calls """
    try:
        grocery = Run()
        rospy.sleep(3)
        grocery.text2audio('hello, I am a product monitoring and restocking robot.')
        #grocery.test()
        status = grocery.monitoring_restocking()
        while status:
            status = grocery.monitoring_restocking()

        grocery.text2audio('I have finished restocking. I will return to home now.')
        grocery.navigation('home')
                
        rospy.spin()
    except rospy.ROSInterruptException:
        pass