import numpy as np
import matplotlib.pyplot as plt
import pathcontrol as pc
import time


# ul = 4
# ur = 4
# lgains = np.linspace(0,-1,8)
# arr = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])
startpy = time.time()
o = 0
for i in range (1000000):
    o = o +1
# v1 = ul + sum(lgains * (1 - arr[:8]))
# v2 = ul + sum(lgains * (1 - arr[:8]))
endpy = time.time()
# print(endpy-startpy)
# startpy = time.time()
# for i in range(8):
#     ul = ul + lgains[i]*(1 - arr[i])
#     ur = ur + lgains[i]*(1 - arr[i])
# endpy = time.time()
print(endpy-startpy)


