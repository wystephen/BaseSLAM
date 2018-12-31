
import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from mayavi import mlab

from array import array

if __name__ == '__main__':

	file_name = '/home/steve/temp/local_pose_result.g2o'


	pos_buf = array('d')
	tag_buf = array('d')


	file_all_lines = open(file_name).readlines()

	for current_line in file_all_lines:
		if 'VERTEX_SE3' in current_line:
			all_units = current_line.split(' ')
			print('all units:', all_units)


