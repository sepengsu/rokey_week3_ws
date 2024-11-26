import rclpy
import DR_init
# for single robot
from ..myutils.drl_function import trans_1d,trans_2d
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 120,120
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
            DR_FC_MOD_ABS,
            ikin,
            fkin,
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
            pos5 = trans_1d(pos4x,'posx',DR_AXIS_Z,-58) # 100mm 밑으로 이동
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
            movel(self.upside_down_space[0],vel=VELOCITY,acc=ACC) # 컴 피쳐 위치로 이동
            movel(self.upside_down_space[1],vel=VELOCITY,acc=ACC) # 아래로 이동
            wait(1.0)
            self.grip_wide()
            wait(1.5)
        
        def upside_down(self):
            movel(self.upside_down_space[2],vel=VELOCITY,acc=ACC) # 위로 이동
            movej(self.upside_down_space[3],vel=VELOCITY,acc=ACC) # 180도 회전
        
        def put_down(self):
            print("위치 이동")
            movel(self.upside_down_space[4],vel=VELOCITY,acc=ACC) # 아래로 이동
            wait(1.0)
            print("force start")
            self.force_y()
            self.ungrip()
            movel(self.upside_down_space[4],vel=VELOCITY,acc=ACC) # 위로 이동
            wait(1.5)
        
        def returning(self):
            movel(self.upside_down_space[5],vel=VELOCITY,acc=ACC) # 뒤로 이동

        def force_y(self):
            # parallel_axis([0,1,0],DR_AXIS_Z,DR_BASE) # z축 정렬
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            print("force start")
            set_desired_force(fd=[3, 3, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            self.ungrip()

        def force_release(self):
            parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            print("force start")
            set_desired_force(fd=[3, 3, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            self.ungrip()


        def moving(self):
            up_pos  = trans_1d(self.move_space[0],'posx',DR_AXIS_Z,100)
            movel(up_pos,vel=VELOCITY,acc=ACC) # 위로 이동
            movel(self.move_space[0],vel=VELOCITY,acc=ACC) # 초기 위치로 이동
            movel(self.move_space[1],vel=VELOCITY,acc=ACC) # 아래로 이동
            self.grip()
            wait(1.0)
            movel(self.move_space[2],vel=VELOCITY,acc=ACC) # 위쪽으로 이동
            movel(self.move_space[3],vel=VELOCITY,acc=ACC) # 오른쪽으로 이동
            movel(self.move_space[4],vel=VELOCITY,acc=ACC) # 아래로 이동
            self.force_release()
            wait(1.0)
            movel(self.move_space[5],vel=VELOCITY,acc=ACC) # 위로 이동

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
        def __init__(self,stack_pos,space,init_pos,final_stack_pos):
            self.above_set = [VELOCITY,ACC] # 위로 올라가는 속도
            self.default_set = [VELOCITY,ACC] # 기본 속도
            self.stack_pos = stack_pos
            self.space = space # 전체 스테이지 index로 접근 가능
            self.final_stack_pos = final_stack_pos
            self.init_pos = init_pos
        
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
            for index,point in enumerate(self.space):
                is_high = False if index != len(self.space)-1 else True
                self.stacking(self.stack_pos,point,is_high)
                if not is_high:
                    self.return_home(self.stack_pos)
                self.stack_pos[2] -= GRIP_HEIGHT
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
            init_zup = trans_1d(init_pos,'posx',DR_AXIS_Z,120)
            movel(init_zup,vel=self.above_set[0],acc=self.above_set[1])
            init_zup = trans_1d(init_zup,'posx',DR_AXIS_Z,-60)
            movel(init_zup,vel=self.above_set[0],acc=self.above_set[1])
            z_point = self.z_point(init_zup)
            init_pos[2] = z_point+5
            self.grip()
            wait(1.0)
            init_pos[2] = final_pos[2] 
            movel(init_pos,vel=self.default_set[0],acc=self.default_set[1])
            final_up_pos = trans_1d(final_pos,'posx',DR_AXIS_Z,40)
            movel(final_up_pos,vel=self.above_set[0],acc=self.above_set[1])
            self.force_release()
            self.ungrip()
            wait(1.0)
            side_pos = get_current_posx()[0]
            side_pos[1]+=100
            movel(side_pos,vel=self.default_set[0],acc=self.default_set[1])

        def stacking(self,init_pos_astract,final_pos,is_high=False):
            self.grip()
            pos1_zup = init_pos_astract.copy()
            print("go")
            print(pos1_zup)
            wait(1.0)
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            print("pointing start")
            z_height = self.z_point(pos1_zup)
            print("pointing end")
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            pos_grip = init_pos_astract.copy()
            pos_grip[2] = z_height -GRIP_HEIGHT # z높이에 맞춰서 가기
            movel(pos_grip,vel=self.default_set[0],acc=self.default_set[1])
            self.grip() # 잡기
            wait(2.0)

            pos1_zup[2]  = z_height + 100
            print("go up")
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1]) # up
            transx = final_pos.copy()
            transx = trans_1d(transx,'posx',DR_AXIS_X,-100)
            if is_high:
                transx[2] += 100
            movel(transx,vel=self.above_set[0],acc=self.above_set[1])
            print("go transx")
            final_pos_abstract = final_pos.copy()
            final_pos_abstract[2] += 100
            movel(final_pos_abstract,vel=self.default_set[0],acc=self.default_set[1]) # 대충 위치 지정 
            final_pos_abstract[2] -= 90
            movel(final_pos_abstract,vel=self.default_set[0],acc=self.default_set[1]) # 대충 위치 지정
            self.force_release()


        def force_release(self):
            parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            print("force start")
            set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            self.ungrip()

        def z_point(self,postiton) -> float:
            '''
            대충 잡은 위치에서 z높이 측정을 위한 로직 
            1. grip
            2. z축 compliance control
            3. z축 이동
            4. ungrip
            5. z높이 반환
            '''
            print("pos start")
            movel(postiton, vel=VELOCITY, acc=ACC)
            # parallel_axis([0, 0, -1], DR_AXIS_Z, DR_BASE)
            wait(2)
            task_compliance_ctrl(stx=[1000, 1000, 1000, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while check_force_condition(DR_AXIS_Z, max=10):
                pass
            pos = get_current_posx()
            release_compliance_ctrl()
            self.ungrip()
            print("pos end")
            movel(postiton, vel=VELOCITY, acc=ACC)
            return pos[0][2]
            

    stacked_pos = posx([510.29,225.20,447.81195068,178.54,179.78,178.14])
    upside_pos = posx([500.978, 27.599, 300.406, 88.119, 90.616, 91.824]) # 위로 올라가는 위치
    moveing_pos = posx([500.28, 225.75,330.06+166,143.90,-177.13,145.79])
    p_pos = posx([500, -50, 93+226.8, 154.034, 179.304, 152.018])# 3줄의 기준 위치
    JReady = [0, 0, 90, 0, 90, 0]
    space1 = UpSideDownSpace(upside_pos,moveing_pos)
    controller1 = UpSideDownController(space1, JReady)
    deltaspace = PyrmidSpace()
    space = deltaspace.get_space_abs(p_pos)
    final_init_pos = posx([ 300.28,  223.75,  60.90270996,  143.40687561,-177.13638306,  145.29704285])
    final_init_pos[2] += 228.6
    final_stack_pos = p_pos.copy()
    final_stack_pos[2]+=228.6
    
    controller2 = RobotController(stacked_pos,space,final_init_pos,final_stack_pos)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        set_ref_coord(DR_BASE)
        release_compliance_ctrl()
        controller1.ungrip()
        movej(JReady, vel=VELOCITY, acc=ACC)
        # controller1.main()
        print("upside down end")
        controller2.main()
        break
    movej(JReady, vel=VELOCITY, acc=ACC)
    rclpy.shutdown()

