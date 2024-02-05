import numpy as np
import matplotlib.pyplot as plt

# Replace 'path_to_grid.txt' with the path to the transferred file on your desktop
grid = np.loadtxt('/Users/jo6109072/Desktop/grid_2.txt')
grid = np.transpose(grid)
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='gray', origin='lower')
plt.colorbar()
plt.show()
