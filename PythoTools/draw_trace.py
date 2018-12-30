import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	pose = np.loadtxt('/home/steve/temp/pose.csv', delimiter=',')

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.plot(pose[:, 0], pose[:, 1], pose[:, 2], '-+')

	plt.show()
