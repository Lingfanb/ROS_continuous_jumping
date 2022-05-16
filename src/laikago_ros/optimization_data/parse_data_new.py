import scipy.io
import numpy as np
import os

#os.chdir("jump_diagonal/chuong_may2021")
#data = scipy.io.loadmat("optimize_jump_yaw-30d_x05_y-03_z00_ry_rz_experiment_with_init_07082021_v1_interp") # chuong


#os.chdir("jump_yaw/down")
#data = scipy.io.loadmat("optimize_yaw90d_x0.6_y0.3_z-0.2_rz_07102021_with_init_v1_interp") # chuong

#os.chdir("jump_yaw/in_place")
#data = scipy.io.loadmat("optimize_yaw_180_rz_07102021_with_init_new_friction_v1_interp") # chuong

os.chdir("jump_left/30cm")
data = scipy.io.loadmat("optimize_left_y03_080421_v1_interp") # chuong


data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
#	if(i == 'taus'):
#		np.savetxt(("data_tau.csv".format(i)), data.get(i), delimiter=",")
#		print("done taus")
#	if(i == 'Qs'):
#		np.savetxt(("data_Q.csv".format(i)), data.get(i), delimiter=",")
#		print("done Qs")

	if(i == 'tau'):
		np.savetxt(("data_tau.csv".format(i)), data.get(i), delimiter=",")
		print("done taus")
	if(i == 'Q'):
		np.savetxt(("data_Q.csv".format(i)), data.get(i), delimiter=",")
		print("done Qs")
	if(i == 'pf'):
		np.savetxt(("data_pf.csv".format(i)), data.get(i), delimiter=",")
		print("done pf")
	if(i == 'vf'):
		np.savetxt(("data_vf.csv".format(i)), data.get(i), delimiter=",")
		print("done vf")
