# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

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
            wait,
            DR_FC_MOD_REL,
            DR_FC_MOD_ABS,
            DR_MV_MOD_REL,
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_AXIS_Z,
            DR_BASE,
        )
        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])

    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("C3_1")
    set_tcp("GripperSA_v1")

    # 초기 위치로 이동
    movej(JReady, vel=VELOCITY, acc=ACC)
    
    def detecting():
        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        wait(0.5)
        set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_X, min=10, ref=DR_BASE) == 0:
                node.get_logger().info('X detection!')
                node.get_logger().warn(f"{check_force_condition(DR_AXIS_X, min=10, ref=DR_BASE)}")
                result = 'x'
                break
            
            elif check_force_condition(DR_AXIS_Y, min=10, ref=DR_BASE) == 0:
                node.get_logger().info('Y detection!')
                node.get_logger().warn(f"{check_force_condition(DR_AXIS_Y, min=10, ref=DR_BASE)}")
                result = 'y'
                break
            
            elif check_force_condition(DR_AXIS_Z, min=10, ref=DR_BASE) == 0:
                node.get_logger().info('Z detection!')
                node.get_logger().warn(f"{check_force_condition(DR_AXIS_Z, min=10, ref=DR_BASE)}")
                result = 'z'
                break

        release_force()
        release_compliance_ctrl()
        
        return result

    di = detecting()
    if di == 'x':
        movel(posx([100,0,0,0,0,0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    elif di == 'y':
        movel(posx([0,100,0,0,0,0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    elif di == 'z':
        movel(posx([0,0,100,0,0,0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
