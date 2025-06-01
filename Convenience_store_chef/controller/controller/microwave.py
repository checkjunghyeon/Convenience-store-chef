# file: microwave_pick_place.py
import rclpy
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
TIME = 0  # time=0 → 속도/가속도 기반으로 동작

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def microwave_task():
    from DSR_ROBOT2 import (
        movej, movejx, movel, movec, move_periodic,
        set_tool, set_tcp,
        set_digital_output,
        DR_TOOL,
        task_compliance_ctrl,
        set_desired_force,
        check_force_condition,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        release_force,
        release_compliance_ctrl,
    )
    
    from DR_common2 import posx

    # 툴 설정
    set_tool("Tool Weight_RG2")
    set_tcp("RG2_TCP")

    # 초기 자세 (joint space)
    init_pos = ([0, 0, 90, 0, 90, 0])
    movej(init_pos, vel=VELOCITY, acc=ACC, time=TIME)

    # 좌표 정의
    handle_start        = posx([0, 0, 190, 0, 90, 0])
    handle_pull         = posx([0, 0, 90, 0, 90, 0])

    food_stock_pick     = posx([0, 100, 90, 0, 90, 0])
    microwave_inside    = posx([0, 100, 90, 0, 90, 0])

    button_pos          = posx([0, 0, 90, 100, 90, 0])
    button_push         = posx([0, 100, 90, 0, 90, 0])  # Z축으로 5mm 하강

    handle_close_start  = posx([0, 0, 190, 0, 90, 0])
    handle_close_end    = posx([0, 0, 90, 0, 90, 0])

    # 그리퍼 제어 함수
    def grasp():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        time.sleep(0.5)

    def release():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        time.sleep(0.5)

    # Step 1: 문 열기
    def open_microwave():
        movel(handle_start, vel=VELOCITY, acc=ACC)  # movel로 손잡이 잡으러 가기
        grasp()
        movec(handle_start, handle_pull, vel=VELOCITY, acc=ACC)  # movec로 문 열기
        release()
        print("111111111111111111")

    # Step 2: 재고에서 음식 꺼내기 (힘 기반, 주기 운동)
    def pick_food_from_stock():
        movejx(food_stock_pick, vel=VELOCITY, acc=ACC)
        print("ZZZZZZZZZZ")
        
        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        release_force()
        release_compliance_ctrl()
        move_periodic(amp=[0, 0, 0, 0, 0, 30], repeat=3, atime=0.02, period=1.0, ref=DR_TOOL)
        grasp()
        print("2222222222222222")

    # Step 3: 음식 넣기
    def place_food_in_microwave():
        movejx(microwave_inside, vel=VELOCITY, acc=ACC)
        release()
        print("3333333333")

    # Step 4: 버튼 누르기
    def press_button(seconds=120):
        movel(button_pos, vel=VELOCITY, acc=ACC)
        movel(button_push, vel=VELOCITY, acc=ACC)
        movel(button_pos, vel=VELOCITY, acc=ACC)
        print(f"Pressed button for {seconds} seconds")  
        print("1444444444444")

    # Step 5: 문 닫기
    def close_microwave():
        movejx(handle_close_start, vel=VELOCITY, acc=ACC)
        grasp()
        movec(handle_close_start, handle_close_end, vel=VELOCITY, acc=ACC)
        release()
        print("555555555555555555")

    # 순차적 동작 수행
    open_microwave()
    pick_food_from_stock()
    place_food_in_microwave()
    press_button(seconds=120)
    close_microwave()

    # 초기 위치 복귀
    movej(init_pos, vel=VELOCITY, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("microwave_task_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        microwave_task()
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
