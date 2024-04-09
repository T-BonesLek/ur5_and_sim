import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type
from pick_goal import move_to_pose
from place_cotton import move_to_place_cotton
from place_mix import move_to_place_mix
from place_wool import move_to_place_wool
import threading  # Import the threading module
from geometry_msgs.msg import Point  # Import the Point message type
import argparse
import time
from std_msgs.msg import Float32MultiArray
from detect_to_pick_dealy import TimeCalculator
from io_port_toggle import TimeKeeper, toggle_conveyor
import signal  # Import the signal module






class MySubscriber(Node):    

    def __init__(self, z):
        super().__init__('my_subscriber')
        self.z = z  # Specify the z value as 'True' or 'False. true for simulation and false for real robot'
        toggle_conveyor(2, 100, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor

        self.x_received = False
        self.y_received = False
        self.lock = threading.Lock()  # Create a new lock
        # Initialize detected_objects as an empty list
        self.detected_objects = []

        self.subscription_object = self.create_subscription(
            Float32MultiArray,  # Specify the message type
            '/object_pose_topic',  # Specify the topic
            self.listener_callback_object,  # Specify the callback
            0)  # Specify the queue size
        

    def listener_callback_object(self, msg):
            with self.lock:  # Acquire the lock before executing the rest of the method
                self.x = msg.data[0:3]
                self.y = msg.data[2]
  

                if len(msg.data) >= 4:  # Check if msg.data has at least four elements
                    self.id = msg.data[3]
                    self.pose = msg.data[0:2]

                    # Add the detected object to the list
                    self.detected_objects.append({
                        'id': self.id,  # Assign id to the object
                        'pose': self.pose,  # Save the object's pose
                        'timestamp': TimeKeeper.get_total_time()  # Save the current timestamp
                    })
                    # Print the updated list of detected objects
                    print("Object Queue:", self.detected_objects)

                #-----------remove this line if you are using the material topic----------------
                
                # print(f"Received y value: {self.y}")
                if self.y == 0.0 or self.y == 1.0 or self.y == 2.0:
                    self.y_received = True  
                else:
                    self.y_received = False 
                #-----------remove this line if you are using the material topic---------------- 
        
                if self.y_received:  # Only accept the x value if the y value has been received
                    self.x = msg.data[0:3]
                    self.x_received = True
                    if self.y_received:
                        self.load_program()
                        self.x_received = False
                        self.y_received = False                    
                else:
                    self.x_received = False  # If the y value has not been received, do not accept the x value

    # def listener_callback_material(self, msg):
    #     self.y = msg.y
    #     self.y_received = True
    #     if self.x_received:
    #         with self.lock:  # Acquire the lock before executing the rest of the method
    #             self.load_program()
    #         self.x_received = False  # Reset both flags after calling load_program
    #         self.y_received = False
                    
    def calculate_pick_time(self, pickPose):
        time_calculator = TimeCalculator()
        time_to_pick = time_calculator.calculate_time(pickPose)
        print(f"Time to pick: {time_to_pick} seconds")
        return time_to_pick

    def conveyor_control_master_of_time(self, trigger, driveTime):
        toggle_conveyor(trigger, driveTime, simulation=self.z)
        print("Total Time Conveyor has moved:", TimeKeeper.total_time)


    def load_program(self):
        # print(f"Received x value: {self.x}")
        # print(f"Received y value: {self.y}")
        
        if self.y_received and self.x_received:
            if self.y == 999.99:
                print("No objecrt detected")

            # #-----------------Time calculation-----------------    
            # time_calculator = TimeCalculator()
            # pickPose = self.x  # replace with your actual pickPose
            # self.time_to_pick = time_calculator.calculate_time(pickPose)
            # print(f"Time to pick: {self.time_to_pick} seconds")
                
            # #-----------------Time calculation-----------------
            self.time_to_pick = self.calculate_pick_time(self.x)
            self.conveyor_control_master_of_time(1, self.time_to_pick)

            if self.y == 0.0:
                # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_cotton, args=(x, z)).start()
                        self.run_pick_cotton(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")
                else:
                    print("Invalid x value")
                    print(self.x[0:3])


            if self.y == 1.0:
                # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_wool, args=(x, z)).start()
                        self.run_pick_wool(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")
                else:
                    print("Invalid x value")
                    print(self.x[0:3])

            if self.y == 2.0:
                # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
                if self.x[0] >= 0.1 and self.x[0] <= 0.37:
                    try:
                        # Start a new thread to run the move_to_pose function with the received x and y values
                        # threading.Thread(target=self.run_pick_mix, args=(x, z)).start()
                        self.run_pick_mix(self.x[0:3], self.z)
                    except Exception as e:
                        print(f"An error occurred: {e}")    
                else:
                    print("Invalid x value")
                    print(self.x[0:3])
                    
                # Reset the flags after using the x and y values
        self.x_received = False
        self.y_received = False

    def run_pick_cotton(self ,pickPose, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(pickPose, False, sim)
        print("Pick finished")
        move_to_place_cotton(False, sim)
        print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic

        
    def run_pick_wool(self ,pickPose, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(pickPose, False, sim)
        print("Pick finished")
        move_to_place_wool(False, sim)
        print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic

    def run_pick_mix(self ,pickPose, z):
        if z == 1.0:
            sim = True
        else:
            sim = False
        move_to_pose(pickPose, False, sim)
        print("Pick finished")
        move_to_place_mix(False, sim)
        print("Place finished")
        MySubscriber(z)     # Re-subscribe to the topic
        
def signal_handler(sig, frame):
    print("Ctrl+C pressed... stopping conveyor")
    toggle_conveyor(0, 100, simulation=0)
    sys.exit(0)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("z", type=str, help="Specify the z value as 'True' or 'False'")
    args = parser.parse_args()

    z = args.z.lower() == 'true'  # Convert the argument to a boolean

    rclpy.init()  # Initialize the default context
    node = None
    try:
        signal.signal(signal.SIGINT, signal_handler)  # Set up the signal handler
        node = MySubscriber(z)
        rclpy.spin(node)  # Spin the node so it can process callbacks
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()