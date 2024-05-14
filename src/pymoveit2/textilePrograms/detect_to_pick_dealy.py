

from transforms import Transformations

class TimeCalculator:
    def __init__(self):
        self.conveyor_speed = 0.07 # m/s min speed
        # self.conveyor_speed = 0.27 # m/s max speed

    def calculate_time(self, pickPose):
        self.trans = Transformations()
        self.robot_coords = self.trans.camera_to_robot(pickPose)
        self.conveyor_cords = self.trans.camera_to_conveyor(pickPose)
        cam_y_pose = pickPose[1]
        # conveyor_y_pose = self.conveyor_cords[1]
        m_cam2Pick = abs(cam_y_pose + self.robot_coords[1] + self.trans.camera_distance_Y) - 0.3 # subtract 0.3 meters to account for scooping motion and robot movement
        if self.conveyor_speed == 0.07:
            t_cam2Pick =  (abs(m_cam2Pick) / self.conveyor_speed) - 0.5  # subtract 0.5 seconds to account for the time delay in the code and other factors
        
        elif self.conveyor_speed == 0.27:
            t_cam2Pick =  (abs(m_cam2Pick) / self.conveyor_speed) - 1.5
        # print(f"conveyor_y_pose: {conveyor_y_pose}")
        # print(f"cam_y_pose: {cam_y_pose}")
        # print(f"Time to move object to pick position: {t_cam2Pick} seconds")
        print(f"distance between cam an pick: {m_cam2Pick} m")

        return t_cam2Pick
    

if __name__ == "__main__":
    pickPose = [0.5, 0.5, 0.5]
    timeCalc = TimeCalculator()
    timeCalc.calculate_time(pickPose)

