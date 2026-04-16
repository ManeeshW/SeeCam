from ctypes import *

class Bar(Structure):
        _fields_ = [("counts", c_int), ("values", POINTER(c_int))]

class Ice(Structure):
        _fields_ = [("counts", c_int), ("control", POINTER(Bar))]

bar = Bar()
ice = Ice()
bar.values = (c_int*1) (9) #(c_int * 3) (1, 2, 3)
bar.count = 1

ice.control.contents = bar
print(bar.count)
print(bar.values[0])
b = ice.control.contents
print(b.values[0])

#for i in range(bar.count):
       #print(bar.values[i])
       # b = ice.control[1].values[1]
        #print(b.values[1])
