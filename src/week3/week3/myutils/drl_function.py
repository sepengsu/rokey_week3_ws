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


def trans_1d(pos,mode:str,axis:int,delta:float) -> list:
    '''
    ### parameter
    pos: list of 6 elements (or posx,posj)  
    mode: 'posx or posj'  
    axis: 참고사항에 있는 파라미터 중 하나   
    delta: 변화량

    ### description
    pos의 axis에 해당하는 값을 delta만큼 변화시킨다.  
    주의: posx에서는 pos[3:6]은 변화할 수 없다.

    #### 참고사항
    DR_AXIS_X = 0  
    DR_AXIS_Y = 1  
    DR_AXIS_Z = 2  

    
    JOINT_AXIS_1 = 0  
    JOINT_AXIS_2 = 1  
    JOINT_AXIS_3 = 2  
    JOINT_AXIS_4 = 3  
    JOINT_AXIS_5 = 4  
    JOINT_AXIS_6 = 5  
    
    '''
    trans_pos = pos.copy()
    if mode == 'posx':
        if axis not in [0,1,2]:
            raise ValueError("axis 변환은 x,y,z만 가능합니다.")
    elif mode == 'posj':
        if axis not in [0,1,2,3,4,5]:
            raise ValueError("joint 변환은 1,2,3,4,5,6만 가능합니다.")
    else:
        raise ValueError("mode should be 'posx' or 'posj'")
    
    trans_pos[axis] += delta
    return list(trans_pos)

def trans_2d(pos,mode,axis:tuple,detla:tuple) -> list:
    '''
    ### parameter
    pos: list of 6 elements (or posx,posj)  
    mode: 'posx or posj'  
    axis: 참고사항에 있는 파라미터 중 2개를 튜플로 입력
    delta: 변화량 (2개를 튜플로 입력)

    ### description
    pos의 axis에 해당하는 값을 delta만큼 변화시킨다.  
    주의: posx에서는 pos[3:6]은 변화할 수 없다.

    #### 참고사항
    DR_AXIS_X = 0  
    DR_AXIS_Y = 1  
    DR_AXIS_Z = 2  

    
    JOINT_AXIS_1 = 0  
    JOINT_AXIS_2 = 1  
    JOINT_AXIS_3 = 2  
    JOINT_AXIS_4 = 3  
    JOINT_AXIS_5 = 4  
    JOINT_AXIS_6 = 5
    '''
    trans_pos = pos.copy()

    if mode == 'posx':
        if axis[0] not in [0,1,2] or axis[1] not in [0,1,2]:
            raise ValueError("axis 변환은 x,y,z만 가능합니다.")
        trans_pos[axis[0]] += detla[0]
        trans_pos[axis[1]] += detla[1]
    elif mode == 'posj':
        if axis[0] not in [0,1,2,3,4,5] or axis[1] not in [0,1,2,3,4,5]:
            raise ValueError("joint 변환은 1,2,3,4,5,6만 가능합니다.")
        trans_pos[axis[0]] += detla[0]

    return list(trans_pos)

if __name__ == "__main__":
    import rclpy
    import DR_init
    from DSR_ROBOT2 import DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_AXIS_A, DR_AXIS_B, DR_AXIS_C, JOINT_AXIS_1 ,JOINT_AXIS_2 ,JOINT_AXIS_3 ,JOINT_AXIS_4 ,JOINT_AXIS_5 ,JOINT_AXIS_6
    print(DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_AXIS_A, DR_AXIS_B, DR_AXIS_C)
    print(JOINT_AXIS_1 ,JOINT_AXIS_2 ,JOINT_AXIS_3 ,JOINT_AXIS_4 ,JOINT_AXIS_5 ,JOINT_AXIS_6)

