import scipy.io
import numpy as np

data = scipy.io.loadmat("jumping_x70_x20_cs503050_051522_motor_constraints_interp")
#data = scipy.io.loadmat("jumping_backflip_cs503040_051522_motor_constraints_contact_timings_v3_interp")


data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
	if(i == 'tau'):
		np.savetxt(("data_tau.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'Q'):
		np.savetxt(("data_Q.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'F'):
		np.savetxt(("data_F.csv".format(i)), data.get(i), delimiter=",")
        if(i == 'pf'):
		np.savetxt(("data_pf.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'vf'):
		np.savetxt(("data_vf.csv".format(i)), data.get(i), delimiter=",")
