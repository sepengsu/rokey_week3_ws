import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 45, 45
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            set_digital_output,
            get_digital_input,
            wait,
            get_current_posx,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            parallel_axis,
            DR_TOOL,
            set_ref_coord,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    JReady = [0, 0, 90, 0, 90, 0]
    # pos1 = posx([496.06, 93.46, 296.92, 20.75, 179.00, 19.09])
    # pos2 = posx([548.70, -193.46, 96.92, 20.75, 179.00, 19.09])
    # pos3 = posx([596.70, -7.46, 196.92, 20.75, 179.00, 19.09])
    pos1 = posx([500.8733825683594, 147.9160919189453, 29.714839935302734, 16.905956268310547, -179.42726135253906, 15.666494369506836])
    pos2 = posx([398.5826110839844, 148.26611328125, 29.528484344482422, 22.007041931152344, -178.9508514404297, 20.872695922851562])
    pos3 = posx([398.0306091308594, 46.04214096069336, 25.300477981567383, 35.9142951965332, -179.81886291503906, 34.800357818603516])
    pos4 = posx([500.66912841796875, 45.6967887878418, 27.7775821685791, 17.618450164794922, -178.57943725585938, 16.52223777770996])
    pos5 = posx([500.5136413574219, -1.4086207151412964, 31.839954376220703, 10.555408477783203, 177.9816131591797, 9.735601425170898])
    pos6 = posx([397.6333923339844, -2.025733232498169, 33.830440521240234, 1.9760515689849854, 179.99032592773438, 1.8869658708572388])
    pos7 = posx([397.6519775390625, -103.99491882324219, 32.10587692260742, 7.948444843292236, 177.9668426513672, 7.074788570404053])
    pos8 = posx([500.0212097167969, -106.29035949707031, 39.00906753540039, 171.6688995361328, 179.96875, 171.82254028320312])
    JReady = [0, 0, 90, 0, 90, 0]
    # index class 지정
    import random
    # Create a shuffled list of numbers from 0 to 8
    shuffled_list = random.sample(range(9), 9)
    movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        pal1 = [pos1,pos2,pos3,pos4]
        pal2 = [pos5,pos6,pos7,pos8]
        # 초기 위치로 이동
        for i in range(9):
            init_pos = generate_3x3_pattern(pal2,i) # 초기 위치 얻기
            de_pos = generate_3x3_pattern(pal1,shuffled_list[i]) # 목표 위치 얻기
            point_to_point(init_pos,de_pos)
            print("point to point end {}".format(i+1))
        movej(JReady, vel=VELOCITY, acc=ACC)
        break
    rclpy.shutdown()


import numpy as np
def generate_3x3_pattern(points,index):
    # points: list of 4 points with [x, y, z, Rz, Ry, Rz]
    if len(points) != 4:
        raise ValueError("Four reference points are required.")    # Extract the (x, y, z) coordinates from the points
    positions = np.array([point for point in points])    # Calculate the vectors along the edges of the 3x3 grid
    vec1 = (positions[1] - positions[0]) / 2
    vec2 = (positions[3] - positions[0]) / 2    # Generate the 3x3 grid points
    pattern = []
    for i in range(3):
        for j in range(3):
            new_point = positions[0] + i * vec1 + j * vec2
            pattern.append(new_point)
    pattern=np.array(pattern)
    return pattern[index].tolist()

from DSR_ROBOT2 import (
    set_tool,
    set_tcp,
    movej,
    movel,
    set_digital_output,
    get_digital_input,
    wait,
)
def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass
def ungrip():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    # wait_digital_input(2)
def grip():
    ungrip()
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    # wait_digital_input(1)

def point_to_point(position1,position2):
    '''
    두 그리드의 위치를 방아 다음과 같은 실행 진행
    1. point1에서 z+100인 곳으로 이용(moveL)
    2. point1로 z 축 이동 (moveL)
    3. ungrip
    4. gripWeight_2FG
    5. point1에서 z+100인 곳으로 이동 (moveL)
    6. point2에서 z+100인 곳으로 이동 (moveL)
    6. point2으로 이동 (힘 제어도 포함???)
    7. ungrip
    8. point2에서 z+100인 곳으로 이동
    '''
    pos1_zup = position1.copy()
    pos1_zup[2]+=100
    pos2_zup = position2.copy()
    pos2_zup[2]+=100
    movel(pos1_zup,vel=VELOCITY,acc=ACC)
    movel(position1,vel=VELOCITY,acc=ACC)
    grip()
    wait(0.5)
    movel(pos1_zup,vel=VELOCITY,acc=ACC)
    movel(pos2_zup,vel=VELOCITY,acc=ACC)
    place_with_force(position1,position2)
    wait(0.3)
    movel(pos2_zup,vel=VELOCITY,acc=ACC)


from DSR_ROBOT2 import (
    posx,
    posj,
    set_ref_coord,
    parallel_axis,
    task_compliance_ctrl,
    set_desired_force,
    release_compliance_ctrl,
    check_force_condition,
    check_position_condition,
    DR_FC_MOD_REL,
    DR_AXIS_Z,
    DR_BASE,
    DR_TOOL,
)
def place_with_force(pos1:posx,pos2: posx):
    '''
    grip한 상태에서 z축 방향으로 내려올 때 z축 방향에서 외력을 받으면 즉시 중지하고 ungrip
    1. task_compliance_ctrl을 이용하여 z축 방향으로 이동
    2. set_desired_force를 이용하여 힘 제어 시작
    3. check_force_condition을 이용하여 외력을 받으면 ungrip
    4. release_compliance_ctrl을 이용하여 compliance control 해제
    '''
    parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE)
    set_ref_coord(DR_TOOL)
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    wait(1)
    set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    while not check_force_condition(DR_AXIS_Z, max=1):
        if check_position_condition(DR_AXIS_Z, pos1, max=pos2[2]+5):
            ungrip()
            release_compliance_ctrl()
            break
        pass
    ungrip()
    release_compliance_ctrl()
    wait(0.5)
    set_ref_coord(DR_BASE)


if __name__ == "__main__":
    main()