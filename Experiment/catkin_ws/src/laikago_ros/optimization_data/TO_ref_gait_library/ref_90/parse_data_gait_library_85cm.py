import scipy.io
import numpy as np

print("script run1")
data = scipy.io.loadmat("jumping_90cm_stepping_stone_noshaking_interp_cs10304040")
print("script run2")
data = {k:v for k, v in data.items() if k[0] != '_'}
print("script run3")
parameter = data.keys()
print("script run4")

dis = 90
for i in parameter:
	print(i)
	if(i == 'tau'):
		np.savetxt(("%scm_tau.csv".format(i)) % dis, data.get(i), delimiter=",")
		print("done taus")
	if(i == 'Q'):
		np.savetxt(("%scm_Q.csv".format(i)) % dis, data.get(i), delimiter=",")
		print("done Q")
	if(i == 'Ff'):
		np.savetxt(("%scm_Ff.csv".format(i)) % dis, data.get(i), delimiter=",")
		print("done Ff")
	if(i == 'Fr'):
		np.savetxt(("%scm_Fr.csv".format(i)) % dis, data.get(i), delimiter=",")
		print("done Fr")


