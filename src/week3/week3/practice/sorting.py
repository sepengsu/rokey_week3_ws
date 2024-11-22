import rclpy
from rclpy.node import Node
from DR_init import __dsr__id, __dsr__model, __dsr__node
from myutils.drl_function import generate_3x3_pattern

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 45, 45
__dsr__id = ROBOT_ID
__dsr__model = ROBOT_MODEL
__dsr__node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

class RobotController:
    def __init__(self, robot_id="dsr01", robot_model="m0609", node=__dsr__node,Vel=45,Acc=45):
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.node = node

        # 로봇 관련 초기화
        self.VELOCITY = Vel
        self.ACC = Acc
        self.ON = 1
        self.OFF = 0

        # 필요한 모듈 가져오기
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
            from DSR_common2 import posx, posj
            self.set_tool = set_tool
            self.set_tcp = set_tcp
            self.movej = movej
            self.movel = movel
            self.set_digital_output = set_digital_output
            self.get_digital_input = get_digital_input
            self.wait = wait
            self.get_current_posx = get_current_posx
            self.release_compliance_ctrl = release_compliance_ctrl
            self.check_force_condition = check_force_condition
            self.task_compliance_ctrl = task_compliance_ctrl
            self.set_desired_force = set_desired_force
            self.parallel_axis = parallel_axis
            self.set_ref_coord = set_ref_coord
            self.posx = posx
            self.posj = posj
        except ImportError as e:
            self.node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
            raise

    def wait_digital_input(self, sig_num):
        while not self.get_digital_input(sig_num):
            self.wait(0.5)
            self.node.get_logger().info("Wait for digital input")

    def ungrip(self):
        self.set_digital_output(2, self.ON)
        self.set_digital_output(1, self.OFF)

    def grip(self):
        self.ungrip()
        self.set_digital_output(1, self.ON)
        self.set_digital_output(2, self.OFF)

    def point_to_point(self, position1, position2):
        pos1_zup = position1.copy()
        pos1_zup[2] += 30
        pos2_zup = position2.copy()
        pos2_zup[2] += 100

        self.movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
        self.movel(position1, vel=self.VELOCITY, acc=self.ACC)
        self.grip()
        self.wait(0.5)

        pos1_zup[2] += 70
        self.movel(pos1_zup, vel=self.VELOCITY, acc=self.ACC)
        self.movel(pos2_zup, vel=self.VELOCITY, acc=self.ACC)
        self.movel(position2, vel=self.VELOCITY, acc=self.ACC)
        self.wait(0.3)
        self.ungrip()
        self.wait(0.3)
        self.movel(pos2_zup, vel=self.VELOCITY, acc=self.ACC)

    def cleanup(self):
        self.node.destroy_node()
        rclpy.shutdown()

class PointList:
    def __init__(self, robot_id="dsr01", robot_model="m0609", node=__dsr__node):
        self.pallet_1 = []
        self.pallet_2 = []
        try:
            from DSR_common2 import posx
            self.posx = posx
        except ImportError as e:
            node.get_logger().error(f"Error importing DSR_common2: {e}")
            raise
    def __call__(self):
        pos1 = [500.8733825683594, 147.9160919189453, 29.714839935302734, 16.905956268310547, -179.42726135253906, 15.666494369506836]
        pos2 = [398.5826110839844, 148.26611328125, 29.528484344482422, 22.007041931152344, -178.9508514404297, 20.872695922851562]
        pos3 = [398.0306091308594, 46.04214096069336, 25.300477981567383, 35.9142951965332, -179.81886291503906, 34.800357818603516]
        pos4 = [500.66912841796875, 45.6967887878418, 27.7775821685791, 17.618450164794922, -178.57943725585938, 16.52223777770996]
        pos5 = [500.5136413574219, -1.4086207151412964, 31.839954376220703, 10.555408477783203, 177.9816131591797, 9.735601425170898]
        pos6 = [397.6333923339844, -2.025733232498169, 33.830440521240234, 1.9760515689849854, 179.99032592773438, 1.8869658708572388]
        pos7 = [397.6519775390625, -103.99491882324219, 32.10587692260742, 7.948444843292236, 177.9668426513672, 7.074788570404053]
        pos8 = [500.0212097167969, -106.29035949707031, 39.00906753540039, 171.6688995361328, 179.96875, 171.82254028320312]
        self.pallet_1 = [pos1, pos2, pos3, pos4]
        self.pallet_2 = [pos5, pos6, pos7, pos8]
        self.pallet_1 = [self.posx(pos) for pos in self.pallet_1]
        self.pallet_2 = [self.posx(pos) for pos in self.pallet_2]
        return self.pallet_1, self.pallet_2
    
class counter:
    def __init__(self,point_list:list):
        self.count = 0
        self.container = point_list
    def index_output(self):
        index = self.container[self.count]
        self.count+=1
        if self.count > len(self.container):
            raise ValueError("Index out of range")
        return index
class Pallet:
    def __init__(self):
        self.long_list = counter([2,5,8])
        self.middle_list = counter([1,4,7])
        self.short_list = counter([0,3,6])
    def index_returns(self, types: int):
        if types ==0:
            return self.long_list.index_output()
        elif types ==1:
            return self.middle_list.index_output()
        elif types ==2:
            return self.short_list.index_output()
def type_select(z: float, offset=3.0) -> int:
    long = 65
    middle = 55
    short = 45
    if abs(z - long) < offset:
        return 0
    elif abs(z - middle) < offset:
        return 1
    elif abs(z - short) < offset:
        return 2
    else:
        raise ValueError("Invalid type")

def main():
    rclpy.init()
    controller = RobotController()
    JReady = [0, 0, 90, 0, 90, 0]
    # index class 지정
    pallet = Pallet()
    movej(JReady, vel=VELOCITY, acc=ACC)
    
    while rclpy.ok():
        set_tool("Tool Weight_GR")
        set_tcp("GripperDA_11")
        pal1 = [pos1,pos2,pos3,pos4]
        pal2 = [pos5,pos6,pos7,pos8]
        # 초기 위치로 이동
        for i in range(9):
            init_pos = generate_3x3_pattern(pal1,i) # 초기 위치 얻기
            z = z_pointing(init_pos)
            mode = type_select(z)
            print("mdoe is {}".format(mode))
            index = pallet.index_returns(mode)
            print("move to {}".format(index+1))
            de_pos = generate_3x3_pattern(pal2,index)
            point_to_point(init_pos,de_pos)
            print("point to point end {}".format(i+1))
        movej(JReady, vel=VELOCITY, acc=ACC)
        break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
