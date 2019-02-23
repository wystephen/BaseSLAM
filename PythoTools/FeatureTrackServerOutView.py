import numpy as np
import scipy as sp

from matplotlib import pyplot as plt


from array import array
import os

if __name__ == '__main__':
	out_file = open('/home/steve/temp/feature_track_server_out.txt','r')


	track_cnt_buf = array('i')


	for line_str in out_file:
		# print(line_str)
		if 'track_cnt:' in line_str:
			valid_str = line_str[11:-2]
			# print('valid str:',valid_str)
			print('valid num len:', len(valid_str.split(',')))



