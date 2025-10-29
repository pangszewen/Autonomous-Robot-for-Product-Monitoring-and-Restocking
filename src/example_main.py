#!/usr/bin/env python3
# filepath: /path/to/breakfast/src/example_main.py
from gtts import gTTS
import rospy
from storing_groceries.srv import Navigate, StartDetection
from storing_groceries.srv import ArmHeadGripper
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
        self.target_transformation_response_message = []
        self.deadline = time.time() + 5*60 - 10
        self.CLASS = ["biscuit", "cereal", "coffee", "cola", "cornae", "cup noodle", "lay stax", "milk", "water", "paprika", "pocky", "potae", "pringles"]
        self.CLASS_ASSIST = ["cornae", "paprika", "potae"]
        self.CLASS_ROBOT = ["biscuit", "cereal", "coffee", "cola", "cup noodle", "lay stax", "milk", "water", "pocky", "pringles"]
        # classify object to class
        # Robot reachability - which rack levels robot can reach
        self.OBJECT_CATEGORIES = {
            'drinks': ['cola', 'coffee', 'water', 'milk'],
            'food': ['cereal', 'cup noodle'],
            'snacks': ['biscuit', 'paprika', 'potae', 'cornae', 'pringles', 'lay stax', 'pocky']
        }

        self.CATEGORY_LOCATIONS = {
            'drinks': 'shelves',                   # Middle shelf, left side
            'food': 'dishwasher_tab',                   # Bottom shelf, middle (heavy items)
            'snacks': 'shelves'                   # Top shelf, right side (light items)
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

    def time_ok(self, pad=0):
        return time.time() + pad < self.deadline
    
    def start_listening(self):
        """Start background voice recognition thread"""
        if not self.listening:
            self.listening = True
            # Clear any old commands
            while not self.voice_queue.empty():
                try:
                    self.voice_queue.get_nowait()
                except queue.Empty:
                    break
            
            self.voice_thread = threading.Thread(target=self._voice_listener_worker)
            self.voice_thread.daemon = True
            self.voice_thread.start()
            
    def stop_listening(self):
        """Stop background voice recognition"""
        self.listening = False
        if self.voice_thread:
            self.voice_thread.join(timeout=1.0)
            
    def _voice_listener_worker(self):
        """Background thread that continuously listens for voice commands"""
        # Suppress ALSA warnings
        import os
        os.environ['ALSA_PCM_CARD'] = 'default'
        os.environ['ALSA_PCM_DEVICE'] = '0'
        
        r = sr.Recognizer()
        r.energy_threshold = 300  # Adjust for your environment
        r.dynamic_energy_threshold = True
        
        with sr.Microphone() as source:
            print("🎤 Adjusting for ambient noise...")
            r.adjust_for_ambient_noise(source, duration=1)
            
        print("🎤 Voice recognition ready - listening in background...")
        
        while self.listening and not rospy.is_shutdown():
            try:
                with sr.Microphone() as source:
                    # Listen for audio with shorter timeout
                    audio = r.listen(source, timeout=1, phrase_time_limit=3)
                
                # Recognize speech
                try:
                    result = r.recognize_google(audio, language='en-US')
                    result_lower = result.lower().strip()
                    print(f"🎤 Heard: '{result}'")
                    
                    # Put recognized commands in queue
                    if any(word in result_lower for word in ['yes', 'yeah', 'yep', 'correct', 'done']):
                        self.voice_queue.put('yes')
                        print("✅ 'Yes' command detected!")
                    elif any(word in result_lower for word in ['no', 'nope', 'stop', 'cancel']):
                        self.voice_queue.put('no')
                        print("❌ 'No' command detected!")
                        
                except sr.UnknownValueError:
                    # Couldn't understand audio - that's okay, keep listening
                    pass
                except sr.RequestError as e:
                    print(f"🎤 Speech recognition error: {e}")
                    rospy.sleep(1)  # Wait before retrying
                    
            except sr.WaitTimeoutError:
                # No speech detected - that's okay, keep listening
                pass
            except Exception as e:
                print(f"🎤 Unexpected error in voice recognition: {e}")
                rospy.sleep(1)
                
    def check_voice_command(self, expected_command="yes"):
        """
        Non-blocking check for voice commands
        Returns True if expected command was heard, False otherwise
        """
        try:
            while not self.voice_queue.empty():
                command = self.voice_queue.get_nowait()
                if command == expected_command:
                    return True
                # If it's not the expected command, we can choose to ignore or handle differently
        except queue.Empty:
            pass
        return False
    
    def voice_command_received(self, keyword):
        # obtain audio from the microphone
        r = sr.Recognizer()
            
        with sr.Microphone() as source:
            print(">>> Say something!")
            audio = r.record(source, duration=10)
                
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio)
            if result.lower() == keyword:
                return True
        except sr.UnknownValueError:
            print("SR could not understand audio")
            return False
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
            return False
            
    
    def wait_for_yes(self):
        """
        Wait for 'yes' voice command with instant response using threading
        """
        self.text2audio("Say 'yes' when you are done.")
        self.start_listening()
        
        # Start listening in background
        
        print("🎤 Waiting for 'yes' command... (listening continuously)")
        
        try:
            while not rospy.is_shutdown():
                # Check for voice command every 100ms
                if self.check_voice_command("yes"):
                    print("✅ 'Yes' command received!")
                    return True
                    
                # Small sleep to prevent CPU spinning
                rospy.sleep(0.1)
                
        finally:
            # Always stop listening when done
            self.stop_listening()
            
        return False
    
    def assist_load_onto_robot(self, cls):
        if not self.time_ok():
            return False

        self.text2audio(
            f"I cannot grasp the {cls}. Please place the {cls} on my tray."
        )

        if self.wait_for_yes():
            rospy.loginfo(f"Waiting for response")
            self.text2audio(f"Thank you. I heard you say 'yes'.")
            return True
        return False

    def assist_unload_to_table_and_confirm(self, cls):
        if not self.time_ok():
            return False

        similar_obj = self.detection('place', cls, True)
        similar_class = similar_obj['class_name']
        self.text2audio(
            f"I cannot place the {cls}. Please place the {cls} beside the {similar_class} on the shelves."
        )

        if self.wait_for_yes():
            rospy.loginfo(f"Waiting for response")
            self.text2audio(f"Thank you. I heard you say 'yes'.")
            return True
        return False
    
    def assist_load_onto_robot_new(self, cls):
        # if not self.time_ok():
        #     return False

        self.text2audio(
            f"I cannot grasp the {cls}. It is too high for me, please place the {cls} on my body carefully."
        )

        if self.wait_for_yes():
            rospy.loginfo(f"Waiting for response")
            self.text2audio(f"Thank you. I heard you say 'yes'.")
            return True
        return False

    def assist_unload_to_table_and_confirm_new(self, cls):
        # if not self.time_ok():
        #     return False

        similar_obj = self.detection('place', cls, True)

        self.text2audio(
            f"I cannot place the {cls}, it is too high for me. Please place the {cls} beside the {similar_obj.class_name} on the shelves."
        )

        if self.wait_for_yes():
            rospy.loginfo(f"Waiting for response")
            self.text2audio(f"Thank you. I heard you say 'yes'.")
            return True
        return False
    
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
        
    def storing_groceries(self):
        loaction1 = self.navigation('dining_table')

        loaction1 = None
        while not loaction1 and not loaction1.class_name:
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
            action2 = "Pick"
            if class_name1 in self.CLASS_ROBOT:
                self.text2audio(f"Picking up the {class_name1}.")
                arm_target = self.arm_manipulation(action2, xmin, ymin, xmax, ymax, class_name1)
                self.text2audio(arm_target.message)
            elif class_name1 in self.CLASS_ASSIST: 
                self.assist_load_onto_robot(class_name1)
            else:
                loaction1 = None

        shelf = self.get_object_location(class_name1)
        loaction2 = self.navigation(shelf)

        action3 = "Place"
        if class_name1 in self.CLASS_ROBOT:
            # Place the item on the rack
            # self.text2audio(f"Placing the {class_name1} on the rack area")
            arm_target = self.arm_manipulation(action3, 0, 0, 0, 0, class_name1)
            self.text2audio(arm_target.message)
        else:
            self.assist_unload_to_table_and_confirm(class_name1)
            
    """----------------------------------------------------------------------------------- """

if __name__=="__main__":
    """ Below are how to develop a task using the service calls """
    try:
        grocery = Run()
        rospy.sleep(3)
        grocery.text2audio('hello, I am going to store some groceries')
        
        items = 5
        while items>0:
            grocery.storing_groceries_new()
            items -= 1

        grocery.text2audio('I have finished storing groceries. Thank you')
                
        rospy.spin()
    except rospy.ROSInterruptException:
        pass