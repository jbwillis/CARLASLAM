from skimage.draw import line
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

img = np.zeros((14,14))
rr, cc = line(6, 6, 10, 13)
img[rr,cc] = 1
img[10, 13] = 2

fig, ax = plt.subplots(1, 1)
ax.pcolormesh(img, edgecolor='k', linewidth=.5, cmap=mpl.colors.ListedColormap(['1.0', '.4', 'r']))
ax.plot(np.array([6, 13]) + .5, np.array([6, 10]) + .5, linewidth=3)
ax.axis('equal')
plt.show()
