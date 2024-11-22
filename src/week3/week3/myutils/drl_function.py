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
ON, OFF = 1, 0
def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

def ungrip():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait_digital_input(2)
def grip():
    ungrip()
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait_digital_input(1)

class VirualOnly():
    def ungrip(self):
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        
    def grip(self):
        self.ungrip()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

