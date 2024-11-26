import rclpy
import DR_init
from ..myutils.drl_function import trans_1d,trans_2d
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60,60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
CUP_HEIGHT = 95
GRIP_HEIGHT = 12
CUP_WIDE = 80

'''
1. UpSideDownController
2. RobotController
'''

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
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_AXIS_Z,
            JOG_AXIS_JOINT_6,
            DR_BASE,
            parallel_axis,
            DR_TOOL,
            set_ref_coord,
            check_position_condition,
            get_current_posj,
            DR_FC_MOD_ABS,
            trans,
            amove_periodic,
            ikin,
            fkin,
            get_solution_space
        )
        from DR_common2 import posx,posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    import numpy as np

    class UpSideDownSpace:
        def __init__(self,upside_pos,moveing_pos):
            self.calculate_upside_down(upside_pos)
            self.calculate_move_spcace(moveing_pos)

        def calculate_upside_down(self,cur_pos):
            '''
            맨처음 pos를 기준으로 여러 포즈 계산
            1. pos1: down_pos에서 100mm 위로 이동
            2. pos2: pos1에서 100mm 위로 이동
            '''
            pos1 = cur_pos.copy() # 초기 위의 위치 
            sol_space = 2 # 계산한 결과 
            pos2 = trans_1d(pos1,'posx',DR_AXIS_Z,-100) # 100mm 아래로 이동
            pos3 = trans_1d(pos2,'posx',DR_AXIS_Z, 100) # 100mm 위로 이동
            pos3j = ikin(pos3, sol_space) # pos3의 joint space -- 주의 요망
            pos4 = trans_1d(pos3j,'posj',JOG_AXIS_JOINT_6,180) # 180도 회전
            pos4x = fkin(pos4, sol_space) # pos4의 x space
            pos5 = trans_1d(pos4x,'posx',DR_AXIS_Z,-80) # 100mm 밑으로 이동
            pos6  = trans_1d(pos5,'posx',DR_AXIS_Y,-150) # 150mm 뒤로 이동 

            self.upside_down_space = [pos1,pos2,pos3,pos4,pos5,pos6]

        def calculate_move_spcace(self,cur_pos):
            '''
            뒤집어진 컵 움직이기 
            '''
            pos1 = cur_pos.copy() # 초기 위치
            pos2 = trans_1d(pos1,'posx',DR_AXIS_Z,-20) # 40mm 아래로 이동
            pos3 = trans_1d(pos2,'posx',DR_AXIS_Z,20) # 40mm 위로 이동
            pos4 = trans_1d(pos3,'posx',DR_AXIS_X,-200) # 200mm 오른쪽으로 이동
            pos5 = trans_1d(pos4,'posx',DR_AXIS_Z,-205) # 205mm 아래로 이동
            pos6 = trans_1d(pos5,'posx',DR_AXIS_Z,150) # 150mm 위로 이동
            self.move_space = [pos1,pos2,pos3,pos4,pos5,pos6]
    
    class UpSideDownController:
        def __init__(self,space:UpSideDownSpace,homing_pos):
            self.upside_down_space = space.upside_down_space
            self.move_space = space.move_space
            self.homing_pos = homing_pos
        def main(self):
            self.grip_process()
            self.upside_down()
            self.put_down()
            self.returning()
            movej(self.homing_pos,vel=VELOCITY,acc=ACC)
            self.moving()

        def grip(self):
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(0.5)

        def ungrip(self):
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            wait(0.5)

        def grip_wide(self):
            set_digital_output(1, ON)
            wait(0.5)
            set_digital_output(2, ON)
            wait(0.5)

        def grip_process(self):
            # self.ungrip()
            wait(1.0)
            movel(self.upside_down_space[0],vel=VELOCITY,acc=ACC) # 컴 피쳐 위치로 이동
            movel(self.upside_down_space[1],vel=VELOCITY,acc=ACC) # 아래로 이동
            wait(1.0)
            self.grip_wide()
            wait(1.5)
        
        def upside_down(self):
            movel(self.upside_down_space[2],vel=VELOCITY,acc=ACC) # 위로 이동
            movej(self.upside_down_space[3],vel=VELOCITY,acc=ACC) # 180도 회전
        
        def put_down(self):
            movel(self.upside_down_space[4],vel=VELOCITY,acc=ACC) # 아래로 이동
            self.ungrip()
            wait(1.5)
        
        def returning(self):
            movel(self.upside_down_space[5],vel=VELOCITY,acc=ACC) # 뒤로 이동
        
        def moving(self):
            movel(self.move_space[0],vel=VELOCITY,acc=ACC) # 초기 위치로 이동
            movel(self.move_space[1],vel=VELOCITY,acc=ACC) # 아래로 이동
            self.grip()
            wait(1.0)
            movel(self.move_space[2],vel=VELOCITY,acc=ACC) # 위쪽으로 이동
            movel(self.move_space[3],vel=VELOCITY,acc=ACC) # 오른쪽으로 이동
            movel(self.move_space[4],vel=VELOCITY,acc=ACC) # 아래로 이동
            print('ungrip')
            self.ungrip()
            wait(1.0)
            movel(self.move_space[5],vel=VELOCITY,acc=ACC) # 위로 이동

    JReady = [0, 0, 90, 0, 90, 0]
    upside_pos = posx([500.978, 27.599, 300.406, 88.119, 90.616, 91.824]) # 위로 올라가는 위치
    moveing_pos = posx([500.28, 223.75,330.06+166,143.90,-177.13,145.79])
    space1 = UpSideDownSpace(upside_pos,moveing_pos)
    controller1 = UpSideDownController(space1, JReady)



    while rclpy.ok():
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")

        # 초기 위치로 이동
        while True:
            print("movej")
            set_ref_coord(DR_BASE)
            controller1.ungrip()
            movej(JReady, vel=VELOCITY, acc=ACC)
            controller1.main()
            break


if __name__ == "__main__":
    main()
