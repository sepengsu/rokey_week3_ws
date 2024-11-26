import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100,100
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
            check_position_condition,
            get_current_posj,
            DR_FC_MOD_ABS,
            trans,
        )
        from DR_common2 import posx,posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    import numpy as np
    class DeltaSpace:
        def __init__(self,x_delta=43,y_delta=24/1.5,num=4):
            self.points = []
            for i in range(num):
                y_temp  = np.linspace(-y_delta*(i),y_delta*(i),i+1)
                for y in y_temp:
                    x_values = float(x_delta*(i+1))
                    self.points.append([x_values,y])

        def get_abs_star_pos(self,pos1):
            abs_pos = []
            for point in self.points:
                pos = pos1.copy()
                pos[0] += point[0]
                pos[1] += point[1]
                abs_pos.append(pos)
            self.abs_pos = abs_pos
        
        def get_line_pos(self,pos1,num=4):
            pos_list = []
            for i in range(num):
                pos = pos1.copy()
                pos[0] += i*30
                pos_list.append(pos)
            self.pos_list= pos_list
            return pos
        
        def total_pos(self):
            return self.pos_list + self.abs_pos
        
    class RobotController:
        def __init__(self,pos_list,x_delta=10,x_init=0):
            self.point_list = [pos_list[i] for i in range(0,len(pos_list))]
            self.new_pointdata_list = []
            self.x_delta = x_delta
            self.x_init = x_init

        def grip(self):
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(0.5)

        
        def ungrip(self):
            set_digital_output(1, OFF)
            wait(0.25)
            set_digital_output(2, ON)
            wait(0.5)
        
        def ungrip_wide(self):
            set_digital_output(1, ON)
            set_digital_output(2, ON)
            wait(0.5)
        
        def point(self):
            for index,point_data in enumerate(self.point_list):
                movel(point_data,vel=VELOCITY,acc=ACC)
                pos_j_init = get_current_posj()
                pos_j = pos_j_init.copy()
                pos_j[-1]+=180/(len(self.point_list)-1)*index
                pos_j[-1] = ranging(pos_j[-1])
                movej(pos_j,vel=VELOCITY,acc=ACC)
                pos_j_final = get_current_posj()
                pos_x = get_current_posx()[0]
                movej(pos_j_init,vel=VELOCITY,acc=ACC)
                self.new_pointdata_list.append(PoseData(pos_x,pos_j_final))
            
            movej(JReady, vel=VELOCITY, acc=ACC)
            return self.new_pointdata_list
        
        def point_to_point(self,position1,position2,index=0):
            '''
            두 그리드의 위치를 방아 다음과 같은 실행 진행
            1. point1에서 z+100인 곳으로 이용(moveL)
            2. point1로 z 축 이동 (moveL)
            3. ungrip
            4. gripWeight_2FG
            5. point1에서 z+100인 곳으로 이동 (moveL)
            6. point2에서 z+100인 곳으로 이동 (moveL)
            6. point2으로 이동 (힘 제어로)
            7. ungrip
            8. point2에서 z+100인 곳으로 이동
            '''
            pos1_zup = position1.copy()
            pos1_zup[2]+=100
            pos2_zup = position2.copy()
            pos2_zup[2]+=100
            movel(pos1_zup,vel=VELOCITY,acc=ACC)
            self.ungrip_wide()
            movel(position1,vel=VELOCITY,acc=ACC) 
            self.grip() # 집기
            wait(0.5)
            movel(pos1_zup,vel=VELOCITY,acc=ACC)
            print("좌표 이동 시작")
            movel(pos2_zup,vel=VELOCITY,acc=ACC)
            print(pos1_zup,pos2_zup)
            pos2_zup[2] = position2[2]+30
            movel(pos2_zup,vel=VELOCITY,acc=ACC)
            print("Angle 조절 시작")
            self.set_joint6_angle()
            self.place_with_force()
            print("PLACE WITH FORCE END")
            wait(0.3)
            pos2_zup[2] = position2[2]+100
            movel(pos2_zup,vel=VELOCITY,acc=ACC)

        def set_joint6_angle(self):
            '''
            현재의 포즈를 posj로 변환, joint6의 각도는 계산값으로 넣어 movej
            '''
            posj_init = get_current_posj()
            posj = posj_init.copy()
            posj6 =0
            posj[-1] = posj6
            movej(posj,vel=VELOCITY,acc=ACC)

        def place_with_force(self):
            '''
            grip한 상태에서 z축 방향으로 내려올 때 z축 방향에서 외력을 받으면 즉시 중지하고 ungrip
            1. task_compliance_ctrl을 이용하여 z축 방향으로 이동
            2. set_desired_force를 이용하여 힘 제어 시작
            3. check_force_condition을 이용하여 외력을 받으면 ungrip
            4. release_compliance_ctrl을 이용하여 compliance control 해제
            '''
            parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            wait(1)
            set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                # if check_position_condition(DR_AXIS_Z, max=pos2[2]+5):
                #     ungrip()
                #     release_compliance_ctrl()
                #     break
                pass
            wait(0.5)
            self.ungrip()
            set_ref_coord(DR_BASE)
            release_compliance_ctrl()
            wait(0.5)

        def final_grip(self,sup_pos,sup_zup):
            self.ungrip_wide()
            movel(sup_zup,vel=60,acc=60)
            movel(sup_pos,vel=VELOCITY,acc=ACC)
            self.grip()
            wait(0.5)
            movel(sup_zup,vel=60,acc=60)

        def strike(self,init_pos):
            init_strike_pos = init_pos.copy()
            init_strike_pos[0] -= 30
            init_strike_pos[2] += 55
            movel(init_strike_pos,vel=VELOCITY,acc=ACC)
            self.joint_5_strike()

        def joint_5_strike(self):
            posj_init = get_current_posj()
            posj = posj_init.copy()
            posj5 = 90-20
            posj[-2] = posj5
            movej(posj,vel=100,acc=90)



        
    JReady = [0, 0, 90, 0, 90, 0]
    init_pos = posx([267.0029602050781, 9.029313087463379, 46.36891174316406, 173.65997314453125, 179.57017517089844, 173.2434844970703])
    x_delta = 300
    space = DeltaSpace(x_delta=37,y_delta=24/1.7,num=4)
    star_init_pos = space.get_line_pos(init_pos,num=4)
    space.get_abs_star_pos(star_init_pos)
    space_list = space.total_pos()
    # import matplotlib.pyplot as plt
    # plt.plot([[pos[0],pos[2]] for pos in sinspace_list],[pos[1] for pos in sinspace_list])
    # plt.show()
    # return 
    sup_pos = posx([270.4926452636719, -230.31503295898438, 49.412559509277344, 125.57516479492188, -179.45201110839844, 125.08116912841797])
    sup_zup = sup_pos.copy()
    sup_zup[2]+=100

    movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        robot = RobotController(space_list,x_delta,init_pos[0])
        # break
        # for i in range(19,len(space_list)):
        #     robot.ungrip() 
        #     robot.point_to_point(position1=sup_pos,position2=space_list[i],index=i)
        #     print("point to point end {}".format(i+1))
        #     movel(sup_zup,vel=VELOCITY,acc=ACC)
        # robot.final_grip(sup_pos,sup_zup)
        # robot.strike(init_pos)
        movej(JReady, vel=VELOCITY, acc=ACC)
        robot.ungrip_wide()
        break
    rclpy.shutdown()
    print("End of program")
    print([item.posj for item in robot.new_pointdata_list])
    print([item.posx for item in robot.new_pointdata_list])

class PoseData:
    def __init__(self,posx,posj):
        self.posx = posx
        self.posj = posj

import numpy as np

def ranging(pos):
    max_pos = 90
    if pos > max_pos:
        pos = -(180-pos)
    elif pos < -max_pos:
        pos = 180+pos
    return pos


