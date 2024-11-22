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


def type_select(z: float, offset=3.0):
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

if __name__ == "__main__":
    pal = Pallet()
    z1 = 67
    z2 = 57
    z3 = 47
    print(pal.index_returns(type_select(z1)))
    print(pal.index_returns(type_select(z2)))
    print(pal.index_returns(type_select(z3)))

