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
    class DeltaSpace:
        def __init__(self,init_pos:list,final_pos:posx):
            self.init_pos_list = init_pos
            self.final_pos = final_pos
            self.final_pos_list = []

        def calculate_final_pos(self):
            transx = self.final_pos[0] - self.init_pos_list[-1][0]
            transy = self.final_pos[1] - self.init_pos_list[-1][1]
            for i in range(len(self.init_pos_list)):
                final_pos_tmp = self.init_pos_list[i].copy()
                final_pos_tmp = trans(final_pos_tmp,[transx,transy,0,0,0,0])
                self.final_pos_list.append(final_pos_tmp)

        def get_space(self):
            self.calculate_final_pos()
            return self.init_pos_list, self.final_pos_list
    class RobotController:
        def __init__(self,init_pos_list,final_pos_list):
            self.init_pos_list = init_pos_list
            self.final_pos_list = final_pos_list
            self.VELOCITY = VELOCITY
            self.ACC = ACC
            self.ON = ON
            self.OFF = OFF

        def grip(self):
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(0.5)

        
        def ungrip(self):
            set_digital_output(1, OFF)
            wait(0.25)
            set_digital_output(2, ON)
            wait(0.5)
        
        def main(self):
            big = 3
            small = 1
            for i in range(big):
                init_pos = self.init_pos_list[i]
                final_pos = self.final_pos_list[i]
                self.point_to_point(init_pos,final_pos)
            for i in range(3,len(self.init_pos_list)):
                init_pos = self.init_pos_list[i]
                final_pos = self.final_pos_list[i]
                self.point_to_point_with_period(init_pos,final_pos)

        def point_to_point(self, position1, position2):
            pos1_zup = position1.copy()
            pos1_zup[2] += 100
            pos2_zup = position2.copy()
            pos2_zup[2] += 100

            movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
            movel(position1, vel=self.VELOCITY, acc=self.ACC)
            self.grip()
            wait(0.5)

            movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
            movel(pos2_zup, vel=self.VELOCITY, acc=self.ACC)
            movel(position2, vel=self.VELOCITY, acc=self.ACC)
            wait(0.3)
            self.ungrip()
            wait(0.3)
            movel(pos2_zup, vel=self.VELOCITY, acc=self.ACC)
        
        def point_to_point_with_period(self,pos1,pos2):
            pos1_zup = pos1.copy()
            pos1_zup[2] += 100
            pos2_zup = pos2.copy()
            pos2_zup[2] += 100

            movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
            movel(pos1, vel=self.VELOCITY, acc=self.ACC)
            self.grip()
            wait(0.5)

            movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
            movel(pos2_zup, vel=self.VELOCITY, acc=self.ACC)
            pos2_init = pos2.copy()
            pos2_init[2] += 30
            # compliance control
            movel(pos2_init, vel=self.VELOCITY, acc=self.ACC)
            self.periodic(pos2_init)
            self.ungrip()
    
        def periodic(self,init_pos):
            pos = init_pos.copy()
            parallel_axis([0,0,-1],DR_AXIS_Z,DR_BASE) # z축 정렬
            set_ref_coord(DR_TOOL) # ord change
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            fd = [0,0,20,0,0,0]
            f_ctrl_dir= [0,0,1,0,0,0]
            set_desired_force(fd,f_ctrl_dir,mod=DR_FC_MOD_REL) # force control
            while True:
                if check_force_condition(axis=DR_AXIS_Z,ref=DR_TOOL,max=10):
                    amove_periodic(amp=[0,0,0,0,0,10],period = 2,atime=0.5,repeat=5,ref=DR_TOOL)
                    break

            while True:
                if  not check_position_condition(axis = DR_AXIS_Z,ref=DR_BASE,max=50): # 40mm 내리기 
                    break
            wait(0.5)
            release_compliance_ctrl()
            set_ref_coord(DR_BASE)
            wait(0.5)


        
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([423.302, -152.38, 38.636, 7.71, -180.0, 7.13]) # 중앙 
    pos2 = posx([456.485, -202.614, 38.992, 120.556, -179.591, 119.923]) # 왼쪽 뒤
    pos3 = posx([450.255, -98.036, 38.659, 91.332, 180.0, 90.535]) # 왼쪽 앞  
    pos4 = posx([363.073, -156.439, 38.986, 138.729, 179.635, 137.863]) # 오른쪽 뒤
    final_pos1 = posx([424.67, 148.428, 40.03, 31.643, -179.197, 30.313]) # 중앙
    space = DeltaSpace([pos2,pos3,pos4,pos1],final_pos1)
    init_pos_list, final_pos_list = space.get_space() # [pos2,pos3,pos4 , pos1]
    controller = RobotController(init_pos_list,final_pos_list)
    movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        movej(JReady, vel=VELOCITY, acc=ACC)
        controller.ungrip()
        controller.main()
        break
    movej(JReady, vel=VELOCITY, acc=ACC)
    rclpy.shutdown()

