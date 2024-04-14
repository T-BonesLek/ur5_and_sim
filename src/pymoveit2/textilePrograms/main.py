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
import signal
import random 


class MySubscriber(Node):    
    
    def signal_handler(self,sig, frame):
        print("Ctrl+C pressed... stopping conveyor")
        self.stop_convayor()
        sys.exit(0)
        

    def __init__(self, is_sim):
        super().__init__('my_subscriber')
        self.is_sim = is_sim  # Specify the is_sim value as 'True' or 'False. true for simulation and false for real robot'
        
        # self.object_pose_received = False
        # self.textile_color_received = False
        self.lock = threading.Lock()  # Create a new lock

        self.robot1_ready = threading.Event()
        self.pause_conveyer_for_pickup = threading.Event()
        
        # last robot is robot1
        self.last_robot_is_ready = self.robot1_ready
        

        # self.robot_placing = threading.Event()
        # self.robot_ready = threading.Event()
        # self.conveyor_running = threading.Event()
        # self.robot_ready.set()  # Robot is initially ready
        # Initialize detected_objects as an empty list
        self.detected_objects = []

        self.subscription_object = self.create_subscription(
            Float32MultiArray,  # Specify the message type
            '/object_pose_topic',  # Specify the topic
            self.listener_callback_object,  # Specify the callback
            0)  # Specify the queue size
        
        # Initialize should_exit
        self.should_exit = False

        self.robot_logic_thread = threading.Thread(target=self.robot_logic)
        self.robot_logic_thread.start()

        self.conveyor_pickup_logic_thread = threading.Thread(target=self.conveyor_master_of_pickup_point_logic) 
        self.conveyor_pickup_logic_thread.start() 

    def signal_handler(self, sig, frame):
        print("Ctrl+C pressed... stopping conveyor")
        self.should_exit = True
        self.stop_conveyor()


    def listener_callback_object(self, msg): #This is a threaded callback function that is called when a message is received but dies after the function is done
        threading.Thread(target=self.process_object_message, args=(msg,)).start()
        

    def process_object_message(self, msg):
        with self.lock:  # Acquire the lock before executing the rest of the method
            if len(msg.data) >= 4:  # Check if msg.data has at least four elements
                self.object_pose = msg.data[0:2]
                self.textile_color = msg.data[2]
                self.object_id = msg.data[3]
                self.time_to_pick = self.calculate_pick_time(self.object_pose)

                # Add the detected object to the list
                self.detected_objects.append({
                    'id': self.object_id,  # Assign id to the object
                    'pose': self.object_pose,  # Save the object's pose
                    'timestamp': self.time_to_pick, # Save the timestamp it should be picked
                    'color': self.textile_color,
                    'material': "todo if spectral camera is used"
                })
                # Print the updated list of detected objects
                print("Object Queue:", self.detected_objects)

                # if self.textile_color == 0.0 or self.textile_color == 1.0 or self.textile_color == 2.0:
                #     self.textile_color_received = True  
                # else:
                #     self.textile_color_received = False 

                # if self.textile_color_received:  # Only accept the x value if the y value has been received
                #     self.object_pose_received = True
                #     if self.textile_color_received:
                #         self.load_program()
                #         self.object_pose_received = False
                #         self.textile_color_received = False                    
                # else:
                #     self.object_pose_received = False  # If the y value has not been received, do not accept the x value

        # if self.robot_ready.is_set():
        #     threading.Thread(target=self.load_program).start()

    # def listener_callback_material(self, msg):
    #     self.textile_color = msg.y
    #     self.textile_color_received = True
    #     if self.object_pose_received:
    #         with self.lock:  # Acquire the lock before executing the rest of the method
    #             self.load_program()
    #         self.object_pose_received = False  # Reset both flags after calling load_program
    #         self.textile_color_received = False
                    
    def calculate_pick_time(self, pickPose):
        time_calculator = TimeCalculator()
        time_to_pick = time_calculator.calculate_time(pickPose)
        time_elapsed_since_detection = TimeKeeper.get_total_time()
        self.adjusted_time_to_pick = time_to_pick + time_elapsed_since_detection
        print(f"Adjusted time to pick: {self.adjusted_time_to_pick} seconds")
        return self.adjusted_time_to_pick

    # def conveyor_control_master_of_time(self, trigger, driveTime):
    #     toggle_conveyor(trigger, driveTime, simulation=self.is_sim)
    #     print("Total Time Conveyor has moved:", TimeKeeper.total_time)

    def start_conveyor(self):
        # self.conveyor_control_master_of_time(1, self.adjusted_time_to_pick)
        toggle_conveyor(1, simulation=self.is_sim)
        # self.conveyor_running = True

    def stop_conveyor(self):
        toggle_conveyor(0, simulation=self.is_sim)
        # self.conveyor_running = False

    def conveyor_master_of_pickup_point_logic(self):
        self.start_conveyor()

        while True:
            time.sleep(0.01)
            if self.should_exit:
                return # Exit the thread if should_exit is True
            
            if self.pause_conveyer_for_pickup.is_set():
                self.stop_conveyor()
                continue
            
                        
            with self.lock:
                self.start_conveyor()
                
                if len(self.detected_objects) == 0:
                    continue

                # conv is running AND we have objects to pick
                top_object = self.detected_objects[0]
                time_to_pick = top_object['timestamp']
                
                if time_to_pick < TimeKeeper.get_total_time():
                    self.stop_conveyor()
                    print("Conveyor stopped. Waiting for robot to be ready...")
                    self.last_robot_is_ready.wait()                    











            # if not self.robot1_ready.is_set() and len(self.detected_objects) > 0:

            #     top_object = self.detected_objects[0]
            #     time_to_pick = top_object['timestamp']

            #     print(f"Total time: {TimeKeeper.get_total_time()} seconds")
            #     print(f"Time to pick: {time_to_pick} seconds")

            #     if time_to_pick < TimeKeeper.get_total_time():
            #         self.stop_conveyor()
            #         print("Conveyor stopped. Waiting for robot to be ready...")
            #         self.robot1_ready.wait()

  

    def robot_logic(self):
        
        robot_color_to_action = {
            0.0: move_to_place_cotton,
            1.0: move_to_place_wool,
            2.0: move_to_place_mix
        }

        self.robot1_ready.set()

        while True:
            time.sleep(0.5)
            with self.lock:
                if self.should_exit:
                    return  # Exit the thread if should_exit is True

                if not self.detected_objects:
                    print("No objects detected")
                    continue

                top_object = self.detected_objects[0]  # Get the top object in the queue
                time_to_pick = top_object['timestamp']

                if top_object["pose"][0] < 0.1 or top_object["pose"][0] > 0.37:
                    print("Invalid x value")
                    # Remove the invalid object from the list
                    self.detected_objects.pop(0)
                    continue                

                if time_to_pick > TimeKeeper.get_total_time():
                    print("No ready objects")
                    print(f"Total time: {TimeKeeper.get_total_time()} seconds")
                    print(f"Time to pick: {time_to_pick} seconds")
                    continue

                # Remove the picked object from the list
                self.detected_objects.pop(0)
        
            print(f"Robot is ready to pick object {top_object['id']}")
            self.robot1_ready.clear()                
            self.pause_conveyer_for_pickup.set()                                
            
            move_to_pose(top_object["pose"], False, self.is_sim)
            print(f"Pick finished for object {top_object['id']} conveyer can start again")
            
            self.pause_conveyer_for_pickup.clear()                

            print("############################### pause_conveyer_for_pickup: ", self.pause_conveyer_for_pickup.is_set())


            place_action = robot_color_to_action[top_object['color']]
            place_action(False, self.is_sim)
            print(f"Place finished for object {top_object['id']}")

            
            
            if not self.detected_objects:
                print("All objects processed")
                
            self.robot1_ready.set() # Set the robot1_ready event to indicate that the robot is ready
                        
                    
                






                


                    
                




    #         # print(f"Received x value: {self.object_pose}")
    #         # print(f"Received y value: {self.textile_color}")
            
    #         if self.textile_color_received and self.object_pose_received:
    #             self.robot_ready.clear()
    #             self.robot_picking.set()
            
    #             if self.textile_color == 999.99:
    #                 print("No objecrt detected")
                
    #             self.conveyor_control_master_of_time(1, self.adjusted_time_to_pick)

    #             # Sort the detected_objects list by timestamp
    #             self.detected_objects.sort(key=lambda obj: obj['timestamp'])

    #             # Get the object with the earliest timestamp
    #             if self.detected_objects:
    #                 earliest_object = self.detected_objects.pop(0)
    #                 self.pose = earliest_object['pose']
    #                 self.id = earliest_object['id']
            

    #             if self.textile_color == 0.0:
    #                 # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
    #                 if self.object_pose[0] >= 0.1 and self.object_pose[0] <= 0.37:
    #                     try:
    #                         # Start a new thread to run the pick and place operation
    #                         threading.Thread(target=self.run_pick_cotton, args=(self.pose, self.is_sim)).start()
    #                     except Exception as e:
    #                         print(f"An error occurred: {e}")
    #                 else:
    #                     print("Invalid x value")
    #                     print(self.pose)


    #             if self.textile_color == 1.0:
    #                 # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
    #                 if self.object_pose[0] >= 0.1 and self.object_pose[0] <= 0.37:
    #                     try:
    #                         threading.Thread(target=self.run_pick_wool, args=(self.pose, self.is_sim)).start()
    #                     except Exception as e:
    #                         print(f"An error occurred: {e}")
    #                 else:
    #                     print("Invalid x value")
    #                     print(self.pose)

    #             if self.textile_color == 2.0:
    #                 # toggle_conveyor(1, self.time_to_pick, simulation=self.z)  # Call toggle_conveyor with trigger = 1 to start the conveyor
    #                 if self.object_pose[0] >= 0.1 and self.object_pose[0] <= 0.37:
    #                     try:
    #                         threading.Thread(target=self.run_pick_mix, args=(self.pose, self.is_sim)).start()
    #                     except Exception as e:
    #                         print(f"An error occurred: {e}")    
    #                 else:
    #                     print("Invalid x value")
    #                     print(self.pose)
                        
    #                 # Reset the flags after using the x and y values
    #         self.object_pose_received = False
    #         self.textile_color_received = False

    # def run_pick_cotton(self ,pickPose_xy, z):
    #     # Clear the robot_ready event because the robot is starting to pick
    #     self.robot_ready.clear()

    #     if z == 1.0:
    #         sim = True
    #     else:
    #         sim = False
    #     move_to_pose(pickPose_xy, False, sim)
    #     print("Pick finished")
    #     self.robot_picking.clear()
    #     self.robot_placing.set()
    #     threading.Thread(target=self.start_conveyor).start()
    #     move_to_place_cotton(False, sim)
    #     print("Place finished")
    #     self.robot_placing.clear()
    #     self.robot_ready.set()
    #     # MySubscriber(z)     # Re-subscribe to the topic
        
    # def run_pick_wool(self ,pickPose_xy, z):
    #     self.robot_ready.clear()
    #     if z == 1.0:
    #         sim = True
    #     else:
    #         sim = False
    #     move_to_pose(pickPose_xy, False, sim)
    #     print("Pick finished")
    #     self.robot_picking.clear()
    #     self.robot_placing.set()
    #     threading.Thread(target=self.start_conveyor).start()
    #     move_to_place_wool(False, sim)
    #     print("Place finished")
    #     self.robot_placing.clear()
    #     self.robot_ready.set()
    #     # MySubscriber(z)     # Re-subscribe to the topic

    # def run_pick_mix(self ,pickPose_xy, z):
    #     self.robot_ready.clear()
    #     if z == 1.0:
    #         sim = True
    #     else:
    #         sim = False
    #     move_to_pose(pickPose_xy, False, sim)
    #     print("Pick finished")
    #     self.robot_picking.clear()
    #     self.robot_placing.set()
    #     threading.Thread(target=self.start_conveyor).start()
    #     move_to_place_mix(False, sim)
    #     print("Place finished")
    #     self.robot_placing.clear()
    #     self.robot_ready.set()
    #     # MySubscriber(z)     # Re-subscribe to the topic


def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--is_sim", type=str2bool, default=False, help="Specify the state as 'True' or 'False'")
    args = parser.parse_args()
    print(f"Simulation mode: {args.is_sim}")
    
    import logging
    logger = logging.getLogger()
    logger.setLevel(logging.WARNING)
    logger.disabled = True




    rclpy.init()
    node = None
    try:
        node = MySubscriber(args.is_sim)
        signal.signal(signal.SIGINT, node.signal_handler)  # Register the signal handler
        while rclpy.ok():
            if node.should_exit:
                print("Exiting program...")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()