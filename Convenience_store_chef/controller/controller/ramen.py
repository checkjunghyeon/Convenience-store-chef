# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time
from dsr_msgs2.srv import MoveStop

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DEGREE_VELOCITY, DEGREE_ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    client = node.create_client(
            MoveStop, "/dsr01/motion/move_stop"
        )
            
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            movesx,
            movejx,
            moveb,
            move_periodic,
            amove_periodic,
            wait,
            set_digital_output,
            DR_FC_MOD_REL,
            DR_FC_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MVS_VEL_NONE,
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            DR_QSTOP,
        )
        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])

    def call_service():
            while not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().warn('wait')
            request = MoveStop.Request()
            request.stop_mode = DR_QSTOP
            future = client.call_async(request)
            future.add_done_callback(response_callback)
            
    def response_callback(future):
        response = future.result()
        if response:
            node.get_logger().info('success')
        else:
            node.get_logger().info('fail')

    # 그리퍼 제어 함수
    def grasp():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait(0.5)

    def release():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        wait(0.5)
    
    def pick_up_ramen():
        path_pose = posx([0, 0, 0, 0, 0, 0])
        box_pose = posx([0, 0, 0, 0, 0, 0])
        movesx([path_pose, box_pose], 
               [VELOCITY, DEGREE_VELOCITY], 
               [ACC, DEGREE_ACC], 
               mod= DR_MV_MOD_REL)
        # movejx()
        ramen_pose = posx([0, 0, 0, 0, 0, 0])
        movel(ramen_pose, vel=VELOCITY, acc=ACC)

        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        wait(0.5)
        set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=10, ref=DR_BASE) == 0:
                amove_periodic(amp =[0, 0, 0, 0, 0, 0.5], 
                               period=1, 
                               atime=0.5, 
                               repeat=3, 
                               ref=DR_TOOL)
                call_service()
                break

        release_force()
        release_compliance_ctrl()

        grasp()
    
    def put_ramen():
        up_pose = posx([0, 0, 100, 0, 0, 0])
        movel(up_pose, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        path_pose = posx([0, 0, 0, 0, 0, 0])
        box_pose = posx([0, 0, 0, 0, 0, 0])
        movesx([path_pose, box_pose], 
               [VELOCITY, DEGREE_VELOCITY], 
               [ACC, DEGREE_ACC], 
               mod= DR_MV_MOD_REL)
        # movejx()
        down_pose = posx([0, 0, -100, 0, 0, 0])
        movel(down_pose, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
    
    def pick_up_soup():
        path_pose = posx([0, 0, 0, 0, 0, 0])
        box_pose = posx([0, 0, 0, 0, 0, 0])
        movesx([path_pose, box_pose], 
               [VELOCITY, DEGREE_VELOCITY], 
               [ACC, DEGREE_ACC], 
               mod= DR_MV_MOD_REL)
        # movejx()
        stick_pose = posx([0, 0, 0, 0, 0, 0])
        movel(stick_pose, vel=VELOCITY, acc=ACC)

        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        wait(0.5)
        set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=10, ref=DR_BASE) == 0:
                break

        release_force()
        release_compliance_ctrl()

        move_periodic(amp =[0, 0, 0, 0, 0, 0.5], 
                        period=1, 
                        atime=0.5, 
                        repeat=3, 
                        ref=DR_TOOL)
        grasp()
    
    def put_soup():
        movel()
        moveb()
        move_periodic()
        movesx()
        movejx()
        movel()

        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        wait(0.5)
        set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=10, ref=DR_BASE) == 0:
                amove_periodic()
                call_service()
                break

        release_force()
        release_compliance_ctrl()

        release()
    
    def timer(Task_info):
        movesx()
        movej(JReady, vel=VELOCITY, acc=ACC)
        Task_info[state] = 'cook'
        Task_info[start] = time.time()
        return Task_info
    
    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("C3_1")
    set_tcp("GripperSA_v1")

    # 초기 위치로 이동
    movej(JReady, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
