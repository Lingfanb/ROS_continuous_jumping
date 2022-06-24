import scipy.io
import numpy as np
import os

os.chdir("backflip_3D")

data = scipy.io.loadmat("optimize_backflip_yaw30d_x00_y00_ry_rz_experiment_06192021_v1_interp") # chuong


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
