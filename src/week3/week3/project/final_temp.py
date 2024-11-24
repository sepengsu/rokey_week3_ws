import rclpy
import DR_init
from myutils.drl_function import trans_1d, trans_2d
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
            fkin
        )
        from DR_common2 import posx,posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    import numpy as np

    class PyrmidSpace:
        def __init__(self):
            # Data in [x, y, z, 0, 0, 0] format
            data = [
                [-80.0, 60.35898384862246, 0.0, 0, 0, 0],
                [0.0, 60.35898384862246, 0.0, 0, 0, 0],
                [80.0, 60.35898384862246, 0.0, 0, 0, 0],
                [-40.0, -8.92304845413262, 0.0, 0, 0, 0],
                [40.0, -8.92304845413262, 0.0, 0, 0, 0],
                [0.0, -78.20508075688771, 0.0, 0, 0, 0],
                [-40.0, 37.2649730810374, 95.0, 0, 0, 0],
                [40.0, 37.2649730810374, 95.0, 0, 0, 0],
                [0.0, -32.01705922171765, 95.0, 0, 0, 0],
                [0.0, 0.0, 190.0, 0, 0, 0]]

            self.first = data[:6] # 1st layer (6 points)
            self.second = data[6:9] # 2nd layer (3 points)
            self.second.reverse()
            self.third = data[9:] # 3rd layer (1 point)

        def get_space_abs(self,top_pos):
            self.abs_space = []
            for point in self.first:
                pos = top_pos.copy()
                pos[0] += point[0]
                pos[1] += point[1]
                pos[2] += point[2]
                self.abs_space.append(pos)
            for point in self.second:
                pos = top_pos.copy()
                pos[0] += point[0]
                pos[1] += point[1]
                pos[2] += point[2]
                self.abs_space.append(pos)
            for point in self.third:
                pos = top_pos.copy()
                pos[0] += point[0]
                pos[1] += point[1]
                pos[2] += point[2]
                self.abs_space.append(pos)
            return self.abs_space
        
    class RobotController:
        def __init__(self,stack_pos,space: PyrmidSpace,move5,p_pos):
            self.above_set = [VELOCITY,ACC] # 위로 올라가는 속도
            self.default_set = [VELOCITY,ACC] # 기본 속도
            self.stack_pos = stack_pos
            self.space = space # 전체 스테이지 index로 접근 가능
            self.final_stack_pos = p_pos
            self.final_stack_pos[2] = 355
            self.init_pos = move5
            self.init_pos[2] = 100
        
        def grip(self):
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(0.5)

        
        def ungrip(self):
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            wait(0.5)
        
        def grip_wide(self):
            '''
            천천히 잡기
            '''
            set_digital_output(1, ON)
            wait(0.5)
            set_digital_output(2, ON)
            wait(0.5)
        
        def main(self):
            '''
            각 스테이별 하는 일 
            1. 스테이지별 놓는 위치 지정 
            2. init_pos에서 잡아 올리기
            3. stacking
            4. return_home
            마지막에서는 upsite_down으로 한 후 
            '''
            # for point in self.space:
            #     self.stacking(self.stack_pos,point)
            #     self.return_home(self.stack_pos)
            #     self.stack_pos[2] -= GRIP_HEIGHT
            self.final_stacking(self.init_pos,self.final_stack_pos)
        

        def return_home(self,stack_pos):
            '''
            1. current_pos에서 z축 이동 
            2. stack_pos에서 z축 위로 이동 
            '''
            current_pos = get_current_posx()[0]
            pos = current_pos.copy()
            pos[2] += 100
            movel(pos,vel=self.default_set[0],acc=self.default_set[1])
            stack_pos_zup = stack_pos.copy()
            stack_pos_zup[2] += 100
            self.grip()
            wait(1.0)
            movel(stack_pos_zup,vel=self.default_set[0],acc=self.default_set[1])
            movel(stack_pos,vel=self.default_set[0],acc=self.default_set[1])


        def final_stacking(self,init_pos,final_pos):
            self.ungrip()
            # pos1_zup[2] += 20 # offser
            movel(init_pos,vel=self.above_set[0],acc=self.above_set[1])
            init_pos[2] -= 50
            movel(init_pos,vel=self.above_set[0],acc=self.above_set[1])
            self.grip()
            wait(1.0)
            init_pos[2] = final_pos[2]
            movel(init_pos,vel=self.default_set[0],acc=self.default_set[1])
            movel(final_pos,vel=self.default_set[0],acc=self.default_set[1])
            self.force_release()
            # self.ungrip()
            # wait(1.0)
            final_pos[0]-=120
            movel(final_pos,vel=self.default_set[0],acc=self.default_set[1])
            self.grip()
            wait(1.0)



        def stacking(self,init_pos_astract,final_pos):
            '''
            1. 대충 위치 잡기 (z축 80mm)
            2. z높이 측정
            3. 위치 찾기
            4. 잡기 
            '''
            print("final_pos",final_pos)
            pos1_zup = init_pos_astract.copy()
            pos1_zup[2] += 10 # offser
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            # z_height = self.z_pointing(pos1_zup)
            z_height = 100
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            pos_grip = init_pos_astract.copy()
            pos_grip[2] = z_height -GRIP_HEIGHT # z높이에 맞춰서 가기
            movel(pos_grip,vel=self.default_set[0],acc=self.default_set[1])
            self.grip() # 잡기
            wait(2.5)
            pos1_zup[2] += 100
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1]) # up
            transx = pos1_zup.copy()
            transx[0]+= 100 # x축 이동
            transx[2]+= 100 # z축 이동
            movel(transx,vel=self.above_set[0],acc=self.above_set[1])
            final_pos_abstract = final_pos.copy()
            final_pos_abstract[2] += 100
            movel(final_pos_abstract,vel=self.default_set[0],acc=self.default_set[1]) # 대충 위치 지정 
            final_pos_abstract[2] -= 90
            movel(final_pos_abstract,vel=self.default_set[0],acc=self.default_set[1]) # 대충 위치 지정
            # self.force_release()
            final_zup = final_pos_abstract.copy()
            final_zup[2] += 50
            movel(final_zup,vel=self.above_set[0],acc=self.above_set[1])


        def force_release(self):
            parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            wait(1)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            print("force start")
            set_desired_force(fd=[1, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            self.ungrip()

        def z_pointing(self,postiton: posx) -> float:
            '''
            대충 잡은 위치에서 z높이 측정을 위한 로직 
            1. grip
            2. z축 compliance control
            3. z축 이동
            4. ungrip
            5. z높이 반환
            '''
            self.grip()
            postiton_zup = postiton.copy()
            movel(postiton_zup, vel=VELOCITY, acc=ACC)
            parallel_axis([0, 0, -1], DR_AXIS_Z, DR_BASE)
            wait(1.5)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[2, 2, -20, 2, 2, 2], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pos = get_current_posx()
                pass
            release_compliance_ctrl()
            self.ungrip()
            movel(postiton_zup, vel=VELOCITY, acc=ACC)
            return pos[0][2]
            

    stacked_pos = posx([512.29,225.20,212.78,178.54,179.78,178.14])
    swing_catch_pos = posj([-11.562, 36.378, 98.07, 82.794, 96.067, -43.032])
    swing_up_pos_x = posx([500.978, 27.599, 300.406, 88.119, 90.616, 91.824]) # 위로 올라가는 위치

    move1 = posx([508.28, 223.75,330.06,143.90,-177.13,145.79])
    move2 = move1.copy()
    move2[0] -= 40
    move3  = move2.copy()
    move3[0] += 40
    move4 = posx([301.009, 229.014, 494.853, 137.064, 179.838, 136.231]) # 3번째 위치
    move5 = posx([301.009, 229.014, 289.5, 19.903, -179.93, 19.51])

    p_pos = posx([500, -50, 93, 154.034, 179.304, 152.018])# 3줄의 기준 위치

    JReady = [0, 0, 90, 0, 90, 0]
    deltaspace = PyrmidSpace()
    space = deltaspace.get_space_abs(p_pos)
    controller = RobotController(stacked_pos,space,move5,p_pos)
    # movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # controller.ungrip()
        # wait(1.0)
        # movel(swing_up_pos_x, vel=VELOCITY, acc=ACC)
        # movej(swing_catch_pos, vel=VELOCITY, acc=ACC)
        # wait(2.0)
        # controller.grip_wide()
        # wait(2.0)
        # cur_posx = get_current_posx()[0]
        # cur_posx[2] += 100
        # movel(cur_posx, vel=VELOCITY, acc=ACC)
        # cur_posj = get_current_posj()
        # cur_posj[-1]+= 180
        # movej(cur_posj, vel=VELOCITY, acc=ACC)
        # cur_posx = get_current_posx()[0]
        # cur_posx[2] -= 80
        # movel(cur_posx, vel=VELOCITY, acc=ACC)
        # controller.ungrip()
        # wait(1.5)
        # cur_posx = get_current_posx()[0]
        # cur_posx[1] -= 150
        # movel(cur_posx, vel=VELOCITY, acc=ACC)
        # movej(JReady, vel=VELOCITY, acc=ACC)

        # stacking
        movel(move1, vel=VELOCITY, acc=ACC)
        break
        movel(move2, vel=VELOCITY, acc=ACC)
        controller.grip()
        wait(1.0)
        movel(move3, vel=VELOCITY, acc=ACC)
        movel(move4, vel=VELOCITY, acc=ACC)
        movel(move5, vel=VELOCITY, acc=ACC)
        controller.ungrip()
        wait(1.0)
        cur_pos = get_current_posx()[0]
        cur_pos[2] += 150
        movel(cur_pos, vel=VELOCITY, acc=ACC)
        movel(stacked_pos, vel=VELOCITY, acc=ACC)
        controller.main()
        break
    movej(JReady, vel=VELOCITY, acc=ACC)
    rclpy.shutdown()


class UpSideDownSpace:
    def __init__(self,stack_pos, drop_pos):
        self.space = []
        self.stack_pos = stack_pos
        self.drop_pos = drop_pos

    def get_other_points(self,up_pos,down_pos):
        '''
        처음 pos2개 이동을 가져오기 
        '''
        self.up_pos = up_pos
        self.down_pos = down_pos
    def calculate_space(self,cur_pos):
        '''
        down_pos를 기준으로 여러 포즈 계산
        1. pos1: down_pos에서 100mm 위로 이동
        2. pos2: pos1에서 100mm 위로 이동
        '''
        self.pos_list = []
        pos1 = cur_pos.copy()



    


class UpSideDownController:
    def __def__(self,stack_pos, drop_pos):
        self.stack_pos = stack_pos
        self.drop_pos = drop_pos
        self.default_set = [VELOCITY,ACC]
        self.above_set = [VELOCITY,ACC]
