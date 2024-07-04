import pylab as plt
import numpy as np
from matplotlib.path import Path

width, height=2000, 2000

polygon=[(0.1*width, 0.1*height), (0.15*width, 0.7*height), (0.8*width, 0.75*height), (0.72*width, 0.15*height)]
poly_path=Path(polygon)

x, y = np.mgrid[:height, :width]
coors = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1))) # coors.shape is (4000000,2)

mask = poly_path.contains_points(coors)
print(mask.reshape(height, width))
plt.imshow(mask.reshape(height, width))
plt.show()