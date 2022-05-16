import scipy.io
import numpy as np

data = scipy.io.loadmat("jumping_backflip_d50_h00_mu0.4_082921_v1")

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
