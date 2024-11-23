import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 45, 45
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
CUP_HEIGHT = 95
GRIP_HEIGHT = 12
CUP_WIDE = 80

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
            check_position_condition,
            get_current_posj,
            DR_FC_MOD_ABS,
            trans,
            amove_periodic,
        )
        from DR_common2 import posx,posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    import numpy as np

    class PyrmidSpace:
        def __init__(self,x_delta,z_delta,num):
            self.points = []
            for i in range(num,0,-1):
                x_temp  = np.linspace(-x_delta*(i),x_delta*(i),i+1)
                for x in x_temp:
                    z_values = float(z_delta*(num - i))
                    self.points.append([x,z_values])

        def get_abs_star_pos(self,pos1)->list:
            abs_pos = []
            for point in self.points:
                pos = pos1.copy()
                pos[0] += point[0]
                pos[2] += point[1]
                abs_pos.append(pos)
            self.abs_pos = abs_pos
            return self.abs_pos
    
    class StageSpace:
        def __init__(self,init_posm,std_pos:list,x_delta:float,z_delta:float):
            self.init_pos = init_posm
            self.std_pos = std_pos # 3개의 포지션이 있음 
            self.stage_list = [[2,3,2],[3,3],[1,4,1]]
            self.x_delta = x_delta
            self.z_delta = z_delta
        
        def get_stage_space(self,index):
            stage_std_pos = self.std_pos[:len(self.stage_list[index])] # 줄수 지정 
            stage_space = []
            for i in range(len(stage_std_pos)):
                line_num = self.stage_list[index][i] # 쌓을 줄의 갯수
                space = PyrmidSpace(self.x_delta,self.z_delta,num=line_num) 
                space.get_abs_star_pos(stage_std_pos[i]) # 1줄의 위치들을 가져옴
                stage_space.append(space.abs_pos) # 1줄의 위치들을 추가
            return stage_space # 3줄의 위치들을 반환
        

        
    class RobotController:
        def __init__(self,stack_pos,stack_stage:StageSpace):
            self.above_set = [70,70] # 위에서 움직일 때 빠르게 움직이기 위한 속도
            self.default_set = [VELOCITY,ACC] # 기본 속도
            self.stack_pos = stack_pos
            self.stack_stage = stack_stage # 전체 스테이지 index로 접근 가능
        
        def grip(self):
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(0.5)
            wait(1)

        
        def ungrip(self):
            set_digital_output(1, OFF)
            wait(0.25)
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
            '''
            for stage_num in range(3):
                stage_space = self.stack_stage.get_stage_space(stage_num) # 3줄의 위치들을 가져옴
                for line in stage_space:
                    for pos in line:
                        self.stacking(self.stack_pos,pos) # 쌓기 
                        print("stacking")
                        self.return_home(self.stack_pos) # 초기 위치로 돌아가기
                        print("return_home")
                        self.stack_pos[2]-=self.stack_stage.z_delta # z축 이동


        def final_stacking(self,init_pos,final_pos):
            pass

        def return_home(self,stack_pos):
            '''
            1. current_pos에서 z축 이동 
            2. stack_pos에서 z축 위로 이동 
            '''
            current_pos = get_current_posx()[0]
            pos = current_pos.copy()
            pos[2] += 100
            movel(pos,vel=self.default_set[0],acc=self.default_set[1])
            stack_pos[2] += 100
            movel(stack_pos,vel=self.default_set[0],acc=self.default_set[1])

        def upside_down(self,position):
            '''
            현재 위치에서 z축을 180도 회전 (위아래 뒤집기)
            '''
            pos =get_current_posj()
            pos[0] = 180 if pos[0] == 0 else 0
            movej(pos,vel=self.default_set[0],acc=self.default_set[1])

        def stacking(self,init_pos_astract,final_pos):
            '''
            1. 대충 위치 잡기 (z축 80mm)
            2. z높이 측정
            3. 위치 찾기
            4. 잡기 
            '''
            print("final_pos",final_pos)
            pos1_zup = init_pos_astract.copy()
            pos1_zup[2] += 8 # offser
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            z_height = self.z_pointing(pos1_zup)
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1])
            pos_grip = init_pos_astract.copy()
            pos_grip[2] = z_height -GRIP_HEIGHT # z높이에 맞춰서 가기
            movel(pos_grip,vel=self.default_set[0],acc=self.default_set[1])
            self.grip() # 잡기
            wait(2.0)
            pos1_zup[2] += 100
            movel(pos1_zup,vel=self.above_set[0],acc=self.above_set[1]) # up
            transx = pos1_zup.copy()
            transx[0]+= 100 # x축 이동
            movel(transx,vel=self.above_set[0],acc=self.above_set[1])
            final_pos_abstract = final_pos.copy()
            final_pos_abstract[2] += 20
            movel(final_pos_abstract,vel=self.default_set[0],acc=self.default_set[1]) # 대충 위치 지정 
            self.force_release()

            final_zup = final_pos_abstract.copy()
            final_zup[2] += 50
            movel(final_zup,vel=self.above_set[0],acc=self.above_set[1])


        def force_release(self):
            self.grip()
            # parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            print("force start")
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=10):
                pass
            release_compliance_ctrl()
            self.ungrip()
            wait(0.5)

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
            wait(1)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=10):
                pos = get_current_posx()
                pass
            release_compliance_ctrl()
            wait(0.3)
            self.ungrip()
            movel(postiton_zup, vel=VELOCITY, acc=ACC)
            return pos[0][2]
            

    stacked_pos = posx([269.885, 197.274, 221.556, 37.452, -180.0, 35.479])
    y_delta = CUP_WIDE + 70
    line1_pos = posx([500, -230, 93, 154.034, 179.304, 152.018])# 3줄의 기준 위치
    line2_pos = posx([line1_pos[0], line1_pos[1] + y_delta, line1_pos[2], line1_pos[3], line1_pos[4], line1_pos[5]])
    line3_pos = posx([line1_pos[0], line2_pos[1] + y_delta, line1_pos[2], line1_pos[3], line1_pos[4], line1_pos[5]])
    JReady = [0, 0, 90, 0, 90, 0]
    space = StageSpace(stacked_pos,[line1_pos,line2_pos,line3_pos],x_delta=CUP_WIDE/1.85,z_delta = CUP_HEIGHT)
    controller = RobotController(stacked_pos,space)
    # movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        controller.grip()
        controller.main()
        break
    movej(JReady, vel=VELOCITY, acc=ACC)
    rclpy.shutdown()







