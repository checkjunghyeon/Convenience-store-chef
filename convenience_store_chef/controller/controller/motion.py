import rclpy
import time
import json
import os
from ament_index_python.packages import get_package_share_directory

from controller.config import *
from dsr_msgs2.srv import MoveStop

import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

INIT_POS = [0, 0, 90, 0, 90, 0]


class MotionController:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(MoveStop, "/dsr01/motion/move_stop")

        path = get_package_share_directory('controller')
        json_path = os.path.join(path, 'resource', 'pose_data.json')
        
        self.pose_data = self.load_pose_data(json_path)
        
        from DSR_ROBOT2 import (
            movej, movejx, movel, movec, move_periodic, amove_periodic,
            set_tool, set_tcp, set_ref_coord,
            set_digital_output,
            DR_TOOL,
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_FC_MOD_REL, DR_FC_MOD_ABS,
            DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z,
            DR_BASE, DR_QSTOP,
            release_force,
            release_compliance_ctrl,
            movesx, movesj, moveb,
            wait,
            DR_MV_MOD_REL, DR_MVS_VEL_NONE,
        )
        from DR_common2 import posx, posj

        # 함수와 상수들 alias
        self.release_compliance_ctrl = release_compliance_ctrl
        self.release_force = release_force
        self.check_force_condition = check_force_condition
        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.set_digital_output = set_digital_output
        self.set_ref_coord = set_ref_coord
        self.wait = wait
        self.movec = movec
        self.movel = movel
        self.movesx = movesx
        self.movesj = movesj
        self.amove_periodic = amove_periodic
        self.posx = posx
        self.posj = posj
        self.movej = movej
        self.movejx = movejx
        self.moveb = moveb
        self.move_periodic = move_periodic
        self.grasp = self._grasp
        self.release = self._release

        self.DR_QSTOP = DR_QSTOP
        self.DR_BASE = DR_BASE
        self.DR_TOOL = DR_TOOL
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_MV_MOD_REL = DR_MV_MOD_REL
        self.DR_FC_MOD_REL = DR_FC_MOD_REL
        
        set_tool("C3_1")
        set_tcp("GripperSA_v1")
        set_ref_coord(0)

    def load_pose_data(self, filepath = "/convenience_store_chef/controller/controller/pose_data.json"):
        with open(filepath, 'r') as f:
            return json.load(f)
        
    def get_pose(self, task_name, pose_type, pose_key):
        try:
            raw_pose = self.pose_data[task_name][pose_type][pose_key]
        except KeyError:
            self.node.get_logger().error(f"[POSE ERROR] {task_name}/{pose_type}/{pose_key} not found.")
            raise

        if pose_type == "posx":
            return self.posx(raw_pose)
        elif pose_type == "posj":
            return self.posj(raw_pose)
        else:
            raise ValueError(f"Unsupported pose_type: {pose_type}")

    def get_pose_group(self, task_name, pose_type):
        pose_dict = self.pose_data[task_name][pose_type]
        # sorted_keys = sorted(pose_dict.keys())  # 넣을까 말까
        for k in pose_dict.keys():
            print(pose_dict[k])
        return [self.posx(pose_dict[k]) if pose_type == "posx" else self.posj(pose_dict[k]) for k in pose_dict.keys()]

    def call_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for /move_stop service...')
        request = MoveStop.Request()
        request.stop_mode = self.DR_QSTOP
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        if response:
            self.node.get_logger().info('MoveStop service call success')
        else:
            self.node.get_logger().info('MoveStop service call failed')

    def _grasp(self):
        self.set_digital_output(1, 1)
        self.set_digital_output(2, 0)
        time.sleep(0.5)

    def _release(self):
        self.set_digital_output(1, 0)
        self.set_digital_output(2, 1)
        time.sleep(0.5)

    def init(self):
        self.movej(READY_POS, vel=VELOCITY, acc=ACC)
        self._release()

    def move_to_stock(self):
        poses = self.get_pose_group("move_to_stock", "posx")

        print("poses", poses)

        # self.movesx(poses, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(poses, vel=VELOCITY, acc=ACC)

    def move_from_stock(self):
        poses = reversed(self.get_pose_group("move_from_stock", "posx"))

        print("poses", poses)

        # self.movesx(poses, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(poses, vel=VELOCITY, acc=ACC)
    
    def pick_food_from_stock(self, food_stock_pick):
        self.move_to_stock()
        
        self.movel(self.posx(food_stock_pick), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        
        self.grasp()
        self.wait(0.5)
        
        self.task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.1)
        self.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_FC_MOD_REL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, min=10, ref=self.DR_BASE) == 0:
                break
        self.release_force()
        self.release_compliance_ctrl()

        self.move_periodic(amp=[0, 0, 0, 0, 0, 30], repeat=3, atime=0.02, period=1.0, ref=self.DR_TOOL)

        self.movel(self.posx([0, 0, 10, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.release()
        self.wait(0.5)

        self.movel(self.posx([0, 0, -20, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.grasp()
        self.wait(0.5)

        self.movel(self.posx([0, 0, 200, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)

    # 김밥, 삼각김밥
    ####################################################################################

    def open_microwave(self):
        poses = self.get_pose_group("open_microwave", "posj")

        self.movesj(poses[:2], vel=VELOCITY, acc=ACC)
        self.grasp()
        self.wait(0.5)
        
        self.movej(poses[2], vel=VELOCITY, acc=ACC)
        self.release()
        self.wait(0.5)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        
        self.movej(poses[3], vel=VELOCITY, acc=ACC)
        self.init()

    def place_food_in_microwave(self):
        self.move_from_stock()

        posj_paths = self.get_pose_group("place_food_in_microwave", "posj")

        self.movesj(posj_paths[:6], vel=VELOCITY, acc=ACC)
        
        self.release()
        self.wait(0.5)

        self.movej(posj_paths[6], vel=VELOCITY, acc=ACC)
        self.init()
        
    def close_microwave(self):
        posj_paths = self.get_pose_group("close_microwave", "posj")
        posx_paths = self.get_pose_group("close_microwave", "posx")
        
        self.movesj(posj_paths, vel=VELOCITY, acc=ACC)

        # self.movel(posx_paths[0], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        # self.movel(posx_paths[1], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        # self.movesx(posx_paths, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC], mod=self.DR_MV_MOD_REL)
        self.movesx(posx_paths, vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.init()

    def press_button(self):
        paths = self.get_pose_group("press_button", "posj")
        
        # self.movej(paths[0], vel=VELOCITY, acc=ACC)
        # self.movej(paths[1], vel=VELOCITY, acc=ACC)
        self.movesj(paths, vel=VELOCITY, acc=ACC)

        self.set_ref_coord(self.DR_TOOL)
        self.task_compliance_ctrl(stx=[100]*6)

        time.sleep(0.1)
        self.set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_TOOL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, max=10, ref=self.DR_BASE) == 0:
                break

        self.release_force()
        self.release_compliance_ctrl()

        self.set_ref_coord(self.DR_BASE)
        self.init()

    def pick_up_food(self):
        paths = self.get_pose_group("pick_up_food", "posj")
        
        self.movesx(paths[:3], vel=VELOCITY, acc=ACC)
        
        self.grasp()
        self.wait(0.5)

        self.movej(paths[3], vel=VELOCITY, acc=ACC)
        self.init()
    
    def serve_gimbab(self):
        paths = self.get_pose_group("serve_gimbab", "posx")

        # self.movesx(paths, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(paths, vel=VELOCITY, acc=ACC)
        self.release()
        self.wait(0.5)

        self.init()

    # 라면
    ####################################################################################

    # def pick_up_ramen(self, ramen_pose):
    #     self.move_to_stock()
    #     self.movel(self.posx(ramen_pose), vel=VELOCITY, acc=ACC)

    #     self.grasp()
    #     self.wait(0.5)
        
    #     self.task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
    #     time.sleep(0.1)
    #     self.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_FC_MOD_REL)
    #     while True:
    #         if self.check_force_condition(self.DR_AXIS_Z, min=10, ref=self.DR_BASE) == 0:
    #             break
    #     self.release_force()
    #     self.release_compliance_ctrl()

    #     self.movel(self.posx([0, 0, 10, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
    #     self.release()
    #     self.wait(0.5)

    #     self.movel(self.posx([0, 0, -20, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
    #     self.grasp()
    #     self.wait(0.5)

    #     self.movel(self.posx([0, 0, 200, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)

    #     self.grasp()
    #     self.wait(0.5)

    def put_ramen(self):
        self.move_from_stock()

        poses = self.get_pose_group("put_ramen", "posx")
        
        # self.movesx(poses, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(poses, vel=VELOCITY, acc=ACC)
        self.release()
        self.wait(0.5)

    def pick_up_soup(self):
        poses = self.get_pose_group("pick_up_soup", "posj")
        
        self.movesj(poses[:5], vel=VELOCITY, acc=ACC)
        self.grasp()
        self.wait(0.5)
        self.movej(poses[5], vel=VELOCITY, acc=ACC)

    def put_soup(self):
        posx_paths = self.get_pose_group("put_soup", "posx")
        posj_paths = self.get_pose_group("put_soup", "posj")

        # self.movesx(posx_paths[:4], vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(posx_paths[:4], vel=VELOCITY, acc=ACC)
        
        self.movej([0, 0, 0, 0, 0, -90], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        self.wait(0.5)
        self.movec(posx_paths[4], posx_paths[5], vel=VELOCITY, acc=ACC)
        self.movej([0, 0, 0, 0, 0, 90], vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
        
        self.movesj(posj_paths, vel=VELOCITY, acc=ACC)
        
        self.release()
        self.wait(0.5)

        # path_pose1 = self.posj([-12.66, 23.52, 111.37, -161.94, 40.97, 85.87])
        # path_pose2 = self.posj([-.55, 12.78, 114.16, -115.93, 33.82, 76.33])
        # path_pose3 = self.posj([-1.9, -3.21, 111.14, -38.11, 37.56, 51.12])

        # self.movesj([path_pose1, path_pose2, path_pose3], vel=VELOCITY, acc=ACC)
        # self.movesj([posj_paths[4], posj_paths[3], posj_paths[2]], vel=VELOCITY, acc=ACC)
        # self.movesx(posx_paths[4:], vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(posx_paths[6:], vel=VELOCITY, acc=ACC)
        self.init()

    def press_cooker_button(self):
        paths_j = self.get_pose_group("press_cooker_button", "posj")
        path_x = self.get_pose("press_cooker_button", "posx", "path3")
        
        self.movej(paths_j[0], vel=VELOCITY, acc=ACC)
        self.movej(paths_j[1], vel=VELOCITY, acc=ACC)

        self.set_ref_coord(self.DR_TOOL)
        self.task_compliance_ctrl(stx=[100]*6)
        self.wait(0.5)
        self.set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_TOOL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, max=8, ref=self.DR_BASE) == 0:
                break

        self.release_force()
        self.release_compliance_ctrl()
        self.set_ref_coord(self.DR_BASE)

        self.movel(path_x, vel=VELOCITY, acc=ACC)
        self.init()

    def pick_up_bowl(self):
        paths = self.get_pose_group("pick_up_bowl", "posx")
        
        # self.movesx(paths, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(paths, vel=VELOCITY, acc=ACC)
        
        self.grasp()
        self.wait(0.5)

        self.set_ref_coord(self.DR_TOOL)
        self.task_compliance_ctrl(stx=[100]*6)
        time.sleep(0.1)
        
        self.set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=self.DR_FC_MOD_REL)
        while True:
            if self.check_force_condition(self.DR_AXIS_Z, min=10, ref=self.DR_TOOL) == 0:
                break

        self.release_force()
        self.release_compliance_ctrl()
        self.set_ref_coord(self.DR_BASE)

        self.movel(self.posx([0, 0, 50, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=self.DR_MV_MOD_REL)
    
    def serve_ramen(self):
        paths = self.get_pose_group("serve_ramen", "posx")
        
        # self.movesx(paths, vel=[VELOCITY, DEGREE_VELOCITY], acc=[ACC, DEGREE_ACC])
        self.movesx(paths, vel=VELOCITY, acc=ACC)
        self.release()
        self.wait(0.5)

        self.init()

####################################################################################

def main():
    rclpy.init()
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    test = MotionController(node)
    test.init()
    food_stock_pick = [0, 0, -50, 0, 0, 0]
    test.pick_food_from_stock(food_stock_pick)

if __name__ == "__main__":
    main()
    rclpy.shutdown()