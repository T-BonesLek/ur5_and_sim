import os
from time import sleep

def run_in_terminal(cmds, terminal='gnome-terminal', title=None):
    # Join the commands into a single string
    cmd_str = '; '.join(cmds)
    # If a title is provided, include it in the command
    if title:
        os.system(f'{terminal} --title "{title}" -e "bash -c \'{cmd_str}; exec bash\'" &')
    else:
        # Run the bash shell with the -c option and pass the command string to it
        os.system(f'{terminal} -e "bash -c \'{cmd_str}; exec bash\'" &')

# Define the commands for each terminal
terminal_moveit = [
    'source /opt/ros/humble/setup.bash',
    'ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=false use_mock_hardware:=false',
]

terminal_robot = [
    'source /opt/ros/humble/setup.bash',
    'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.0.102 use_mock_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller',
]

terminal_gazebo_sim = [
    'cd github/ur5_and_sim',
    'source /opt/ros/humble/setup.bash',
    'source install/setup.bash',
    'ros2 launch multi_robot_arm gazebo_arm.launch.py'
]

terminal_realsense = [
    'cd github/objectDetectionTextile',
    'source /opt/ros/humble/setup.bash',
    'ros2 launch realsense2_camera rs_launch.py camera_namespace:=textilePose camera_name:=D455', 
]

terminal_objectDetection = [
    'cd github/objectDetectionTextile/src/yoloV8',
    'source /opt/ros/humble/setup.bash', 
    'python3 objectDetectioTextile.py', 
]

terminal_main =[
    'cd github/ur5_and_sim',
    'source /opt/ros/humble/setup.bash', 
    'source install/setup.bash',
    'cd src/pymoveit2/textilePrograms',
    # 'python3 main.py --is_sim True simulation',
    'python3 main.py --is_sim False #For realRobot',
]

# Run the commands in new terminals
# run_in_terminal(terminal_moveit, title='MoveIt Terminal')
run_in_terminal(terminal_robot, title='Robot Terminal')
sleep(5)
run_in_terminal(terminal_gazebo_sim, title='Gazebo Terminal')
sleep(5)
run_in_terminal(terminal_realsense, title='Realsense Terminal')
sleep(5)
run_in_terminal(terminal_objectDetection, title='Object Detection Terminal')
sleep(5)
run_in_terminal(terminal_main, title='Main Terminal')
