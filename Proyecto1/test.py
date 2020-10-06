import numpy as np
import matplotlib.pyplot as plt
import pathcontrol as pc
goals = [(-6.5,  0.5), (-5.5, -3.5), (-4.5, 1.5), (-0.5, -6.5), (1.5, -4.5),
(2.5, -0.5), (3.5, -1.5), (6.5, -4.5), (9.5, -7.5), (11.5, -4.5)]
goals = [(-6.0,  0.5), (-5.5, -3.5), (-4.5, 1.5), (-0.5, -6.5), (1.5, -4.5), (2.5, -0.5), (3.5, -1.5), (6.5, -4.5), (9.5, -5.5), (11.5, -4.5)]
p = np.array(goals)
path = pc.splinePath(p[:,0], p[:,1])
points = np.linspace(min(p[:,0]), max(p[:,0]), num=100, endpoint=True)
plt.plot(p[:,0], p[:,1], "o", points, path(points), "-")
plt.show()